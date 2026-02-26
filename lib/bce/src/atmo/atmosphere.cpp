/**
 * @file atmosphere.cpp
 * @brief Atmospheric model implementation.
 *
 * Air density, speed of sound, and 4-factor BC correction.
 *
 * Reference formulas:
 *   ρ = P / (R_specific × T_kelvin)   — ideal gas law for dry air
 *   With humidity correction via virtual temperature.
 *
 * 4-factor BC correction (Litz / Army Metro reference):
 *   BC_corrected = BC × FA × (1 + FT - FP) × FR
 *
 * Internal imperial conversions are used where the reference algorithm
 * specifies them (altitude in feet, temperature in °F, pressure in inHg).
 */

#include "atmosphere.h"
#include <cmath>

// Imperial conversion factors (internal use only for reference formulas)
static constexpr float M_TO_FT       = 3.28084f;
static constexpr float PA_TO_INHG    = 0.00029530f;
static constexpr float C_TO_F_OFFSET = 32.0f;
static constexpr float C_TO_F_SCALE  = 1.8f;

void Atmosphere::init() {
    pressure_pa_     = BCE_DEFAULT_PRESSURE_PA;
    temperature_c_   = BCE_DEFAULT_TEMPERATURE_C;
    humidity_        = BCE_DEFAULT_HUMIDITY;
    altitude_m_      = BCE_DEFAULT_ALTITUDE_M;
    baro_offset_pa_  = 0.0f;

    has_baro_pressure_    = false;
    has_baro_temperature_ = false;
    has_baro_humidity_    = false;
    has_override_altitude_  = false;
    has_override_pressure_  = false;
    has_override_temp_      = false;
    has_override_humidity_  = false;
    had_invalid_input_      = false;
    zero_recompute_hint_    = false;
    last_bc_factor_         = 1.0f;

    recompute();
    last_bc_factor_ = correctBC(1.0f);
    zero_recompute_hint_ = false;
}

bool Atmosphere::consumeZeroRecomputeHint() {
    bool pending = zero_recompute_hint_;
    zero_recompute_hint_ = false;
    return pending;
}

void Atmosphere::updateFromBaro(float pressure_pa, float temperature_c, float humidity) {
    had_invalid_input_ = false;

    has_baro_pressure_ = true;
    has_baro_temperature_ = true;

    // Apply calibration offset
    float corrected_pressure = pressure_pa + baro_offset_pa_;
    if (!std::isfinite(corrected_pressure)) {
        corrected_pressure = BCE_DEFAULT_PRESSURE_PA;
        had_invalid_input_ = true;
    }
    if (corrected_pressure < 1000.0f) {
        corrected_pressure = 1000.0f;
        had_invalid_input_ = true;
    }
    if (corrected_pressure > 120000.0f) {
        corrected_pressure = 120000.0f;
        had_invalid_input_ = true;
    }
    pressure_pa_ = corrected_pressure;

    float safe_temp = temperature_c;
    if (!std::isfinite(safe_temp)) {
        safe_temp = BCE_DEFAULT_TEMPERATURE_C;
        had_invalid_input_ = true;
    }
    if (safe_temp < -80.0f) {
        safe_temp = -80.0f;
        had_invalid_input_ = true;
    }
    if (safe_temp > 80.0f) {
        safe_temp = 80.0f;
        had_invalid_input_ = true;
    }
    temperature_c_ = safe_temp;

    if (humidity >= 0.0f && humidity <= 1.0f) {
        has_baro_humidity_ = true;
        humidity_ = humidity;
    } else if (humidity >= 0.0f) {
        had_invalid_input_ = true;
        has_baro_humidity_ = true;
        if (std::isfinite(humidity)) {
            if (humidity < 0.0f) {
                humidity_ = 0.0f;
            } else if (humidity > 1.0f) {
                humidity_ = 1.0f;
            }
        } else {
            humidity_ = BCE_DEFAULT_HUMIDITY;
        }
    }

    recompute();
}

void Atmosphere::applyDefaults(const BCE_DefaultOverrides& ovr) {
    if (ovr.use_altitude) {
        has_override_altitude_ = true;
        altitude_m_ = ovr.altitude_m;
    }
    if (ovr.use_pressure) {
        has_override_pressure_ = true;
        // Override only used if no baro sensor data
        if (!has_baro_pressure_) {
            pressure_pa_ = ovr.pressure_pa;
        }
    }
    if (ovr.use_temperature) {
        has_override_temp_ = true;
        if (!has_baro_temperature_) {
            temperature_c_ = ovr.temperature_c;
        }
    }
    if (ovr.use_humidity) {
        has_override_humidity_ = true;
        if (!has_baro_humidity_) {
            humidity_ = ovr.humidity_fraction;
        }
    }
    recompute();
}

void Atmosphere::calibrateBaro() {
    // Store offset so current pressure reads as standard sea-level
    // This is a simplistic field calibration
    baro_offset_pa_ = BCE_STD_PRESSURE_PA - (pressure_pa_ - baro_offset_pa_);
    recompute();
}

void Atmosphere::recompute() {
    float prev_density = air_density_;
    float prev_sos = speed_of_sound_;
    float prev_bc_factor = last_bc_factor_;

    // Update diagnostic flags
    diag_flags_ = 0;
    if (!has_baro_pressure_ && !has_override_pressure_) {
        diag_flags_ |= BCE_Diag::DEFAULT_PRESSURE;
    }
    if (!has_baro_temperature_ && !has_override_temp_) {
        diag_flags_ |= BCE_Diag::DEFAULT_TEMP;
    }
    if (!has_baro_humidity_ && !has_override_humidity_) {
        diag_flags_ |= BCE_Diag::DEFAULT_HUMIDITY;
    }
    if (!has_override_altitude_) {
        diag_flags_ |= BCE_Diag::DEFAULT_ALTITUDE;
    }

    // Temperature in Kelvin
    float T_kelvin = temperature_c_ + BCE_KELVIN_OFFSET;
    if (T_kelvin < 1.0f) T_kelvin = 1.0f; // safety

    float pressure_pa = pressure_pa_;
    if (!std::isfinite(pressure_pa) || pressure_pa < 1000.0f) {
        pressure_pa = 1000.0f;
        had_invalid_input_ = true;
    }

    float humidity = humidity_;
    if (!std::isfinite(humidity)) {
        humidity = BCE_DEFAULT_HUMIDITY;
        had_invalid_input_ = true;
    }
    if (humidity < 0.0f) {
        humidity = 0.0f;
        had_invalid_input_ = true;
    }
    if (humidity > 1.0f) {
        humidity = 1.0f;
        had_invalid_input_ = true;
    }

    // Virtual temperature accounting for humidity
    // Vapor pressure (Buck equation approximation)
    float e_sat = 611.21f * std::exp((18.678f - temperature_c_ / 234.5f) *
                                      (temperature_c_ / (257.14f + temperature_c_)));
    float e_vapor = humidity * e_sat;

    // Virtual temperature: Tv = T * (1 + 0.378 * e / P)
    float T_virtual = T_kelvin * (1.0f + 0.378f * e_vapor / pressure_pa);
    if (!std::isfinite(T_virtual) || T_virtual < 1.0f) {
        T_virtual = 1.0f;
        had_invalid_input_ = true;
    }

    // Air density via ideal gas law with virtual temperature
    air_density_ = pressure_pa / (BCE_R_DRY_AIR * T_virtual);

    // Speed of sound (approximation for moist air)
    speed_of_sound_ = 20.05f * std::sqrt(T_virtual);

    float current_bc_factor = correctBC(1.0f);
    if (std::fabs(current_bc_factor - prev_bc_factor) >= BCE_ZERO_RECOMPUTE_BC_FACTOR_DELTA ||
        std::fabs(air_density_ - prev_density) >= BCE_ZERO_RECOMPUTE_DENSITY_DELTA ||
        std::fabs(speed_of_sound_ - prev_sos) >= BCE_ZERO_RECOMPUTE_SOS_DELTA) {
        zero_recompute_hint_ = true;
    }
    last_bc_factor_ = current_bc_factor;
}

float Atmosphere::correctBC(float bc_standard) const {
    // Convert to imperial for reference formula compatibility
    float alt_ft   = altitude_m_ * M_TO_FT;
    float press_inhg = pressure_pa_ * PA_TO_INHG;
    float temp_f   = temperature_c_ * C_TO_F_SCALE + C_TO_F_OFFSET;

    // Standard reference values (Army Metro)
    constexpr float std_press_inhg = 29.5300f; // ≈ 101325 Pa
    constexpr float std_temp_f     = 59.0f;    // 15 °C

    // FA — Altitude factor
    // Litz: FA = 1 - 0.00003158 × altitude_ft
    // (approximation for density reduction with altitude)
    float FA = 1.0f - 3.158e-5f * alt_ft;
    if (FA < 0.5f) FA = 0.5f; // clamp for extreme altitudes

    // FT — Temperature factor
    // FT = (temp_f - std_temp_f) / (std_temp_f + 460)
    float FT = (temp_f - std_temp_f) / (std_temp_f + 460.0f);

    // FP — Pressure factor
    // FP = (std_press_inhg - press_inhg) / std_press_inhg
    float FP = (std_press_inhg - press_inhg) / std_press_inhg;

    // FR — Humidity factor
    // Small effect: FR ≈ 1.0 + 0.002 * (humidity_pct - 50)
    // where humidity_pct = humidity_ * 100
    float humidity_pct = humidity_ * 100.0f;
    float FR = 1.0f + 0.00002f * (humidity_pct - 50.0f);

    // Combined correction: BC_corrected = BC × FA × (1 + FT - FP) × FR
    float bc_corrected = bc_standard * FA * (1.0f + FT - FP) * FR;

    // Never let BC go below a minimum reasonable value
    if (bc_corrected < 0.01f) bc_corrected = 0.01f;

    return bc_corrected;
}
