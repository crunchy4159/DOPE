/**
 * @file atmosphere.h
 * @brief Atmospheric model — air density, speed of sound, BC correction.
 *
 * BCE SRS v1.3 — Sections 7.3, 11.2
 *
 * Implements the 4-factor BC correction model (altitude, temperature,
 * pressure, humidity) using reference Army Metro / Litz formulas.
 * Imperial conversions are performed internally for compatibility with
 * the reference drag model.
 */

#pragma once

#include "bce/bce_config.h"
#include "bce/bce_types.h"

class Atmosphere {
public:
    void init();

    /**
     * Update with barometer readings.
     * @param pressure_pa   Absolute pressure in Pascals
     * @param temperature_c Temperature in Celsius
     * @param humidity      Relative humidity 0–1 (or negative if unavailable)
     */
    void updateFromBaro(float pressure_pa, float temperature_c, float humidity);

    /**
     * Apply default overrides for atmosphere parameters.
     */
    void applyDefaults(const BCE_DefaultOverrides& ovr);

    /**
     * Capture current pressure as the baro reference (field calibration).
     */
    void calibrateBaro();

    /**
     * Get current air density (kg/m³).
     */
    float getAirDensity() const { return air_density_; }

    /**
     * Get current speed of sound (m/s).
     */
    float getSpeedOfSound() const { return speed_of_sound_; }

    /**
     * Get pressure (Pa).
     */
    float getPressure() const { return pressure_pa_; }

    /**
     * Get temperature (°C).
     */
    float getTemperature() const { return temperature_c_; }

    /**
     * Get humidity fraction.
     */
    float getHumidity() const { return humidity_; }

    /**
     * Get station altitude (m).
     */
    float getAltitude() const { return altitude_m_; }

    /**
     * Compute corrected BC using Litz 4-factor model.
     * Applies FA (altitude), FT (temperature), FP (pressure), FR (humidity).
     *
     * @param bc_standard  BC at standard (ISA) conditions
     * @return             Corrected BC for current atmosphere
     */
    float correctBC(float bc_standard) const;

    /**
     * Get diagnostic flags for which defaults are active.
     */
    uint32_t getDiagFlags() const { return diag_flags_; }

    /**
     * True if the most recent baro update contained non-physical inputs
     * that were sanitized.
     */
    bool hadInvalidInput() const { return had_invalid_input_; }

    /**
     * True if atmosphere changed enough to justify zero-angle recomputation.
     * Returns and clears the internal pending flag.
     */
    bool consumeZeroRecomputeHint();

private:
    float pressure_pa_     = BCE_DEFAULT_PRESSURE_PA;
    float temperature_c_   = BCE_DEFAULT_TEMPERATURE_C;
    float humidity_        = BCE_DEFAULT_HUMIDITY;
    float altitude_m_      = BCE_DEFAULT_ALTITUDE_M;

    float air_density_     = BCE_STD_AIR_DENSITY;
    float speed_of_sound_  = BCE_SPEED_OF_SOUND_15C;

    float baro_offset_pa_  = 0.0f;  // calibration offset

    bool  has_baro_pressure_   = false;
    bool  has_baro_temperature_ = false;
    bool  has_baro_humidity_   = false;
    bool  has_override_altitude_ = false;
    bool  has_override_pressure_ = false;
    bool  has_override_temp_     = false;
    bool  has_override_humidity_ = false;
    bool  had_invalid_input_     = false;
    bool  zero_recompute_hint_   = false;

    float last_bc_factor_        = 1.0f;

    uint32_t diag_flags_ = 0;

    void recompute();
};
