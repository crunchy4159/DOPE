/**
 * @file bce_types.h
 * @brief All data structures for the Ballistic Core Engine.
 *
 * BCE SRS v1.3 — Sections 3, 5.2, 7, 8, 12, 13
 */

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Operating Modes — SRS §3
// ---------------------------------------------------------------------------
enum class BCE_Mode : uint32_t {
    IDLE           = 0,  // Insufficient data for solution
    SOLUTION_READY = 1,  // Valid firing solution available
    FAULT          = 2   // Required inputs missing or invalid
};

// ---------------------------------------------------------------------------
// Fault Flags (bitfield) — SRS §13
// ---------------------------------------------------------------------------
namespace BCE_Fault {
    constexpr uint32_t NONE            = 0;
    constexpr uint32_t NO_RANGE        = (1u << 0);
    constexpr uint32_t NO_BULLET       = (1u << 1);
    constexpr uint32_t NO_MV           = (1u << 2);
    constexpr uint32_t NO_BC           = (1u << 3);
    constexpr uint32_t ZERO_UNSOLVABLE = (1u << 4);
    constexpr uint32_t AHRS_UNSTABLE   = (1u << 5);
    constexpr uint32_t SENSOR_INVALID  = (1u << 6);
} // namespace BCE_Fault

// ---------------------------------------------------------------------------
// Diagnostic Flags (bitfield) — informational, not faults
// ---------------------------------------------------------------------------
namespace BCE_Diag {
    constexpr uint32_t NONE               = 0;
    constexpr uint32_t CORIOLIS_DISABLED  = (1u << 0);
    constexpr uint32_t DEFAULT_PRESSURE   = (1u << 1);
    constexpr uint32_t DEFAULT_TEMP       = (1u << 2);
    constexpr uint32_t DEFAULT_HUMIDITY   = (1u << 3);
    constexpr uint32_t DEFAULT_ALTITUDE   = (1u << 4);
    constexpr uint32_t DEFAULT_WIND       = (1u << 5);
    constexpr uint32_t MAG_SUPPRESSED     = (1u << 6);
    constexpr uint32_t LRF_STALE          = (1u << 7);
} // namespace BCE_Diag

// ---------------------------------------------------------------------------
// Drag Model Enum — SRS §8
// ---------------------------------------------------------------------------
enum class DragModel : uint8_t {
    G1 = 1,
    G2 = 2,
    G3 = 3,
    G4 = 4,
    G5 = 5,
    G6 = 6,
    G7 = 7,
    G8 = 8
};

// ---------------------------------------------------------------------------
// AHRS Algorithm Selection
// ---------------------------------------------------------------------------
enum class AHRS_Algorithm : uint8_t {
    MADGWICK = 0,
    MAHONY   = 1
};

// ---------------------------------------------------------------------------
// SensorFrame — SRS §7
// ---------------------------------------------------------------------------
struct SensorFrame {
    // Timestamp (microseconds since boot)
    uint64_t timestamp_us;

    // IMU — SRS §7.1
    float accel_x, accel_y, accel_z;   // m/s²
    float gyro_x, gyro_y, gyro_z;     // rad/s
    bool  imu_valid;

    // Magnetometer — SRS §7.2
    float mag_x, mag_y, mag_z;        // µT
    bool  mag_valid;

    // Barometer — SRS §7.3
    float baro_pressure_pa;           // Pa
    float baro_temperature_c;         // °C
    float baro_humidity;              // 0.0–1.0 (optional)
    bool  baro_valid;
    bool  baro_humidity_valid;

    // Laser Rangefinder — SRS §7.4
    float    lrf_range_m;             // meters
    uint64_t lrf_timestamp_us;        // timestamp of LRF reading
    bool     lrf_valid;

    // Zoom Encoder — SRS §7.5
    float encoder_focal_length_mm;    // mm
    bool  encoder_valid;
};

// ---------------------------------------------------------------------------
// Default Overrides — SRS §5.2
// ---------------------------------------------------------------------------
struct BCE_DefaultOverrides {
    bool  use_altitude;
    float altitude_m;

    bool  use_pressure;
    float pressure_pa;

    bool  use_temperature;
    float temperature_c;

    bool  use_humidity;
    float humidity_fraction;

    bool  use_wind;
    float wind_speed_ms;
    float wind_heading_deg;

    bool  use_latitude;
    float latitude_deg;
};

// ---------------------------------------------------------------------------
// Bullet Profile — SRS §8
// ---------------------------------------------------------------------------
struct BulletProfile {
    float     bc;                     // Ballistic coefficient
    DragModel drag_model;             // G1–G8
    float     muzzle_velocity_ms;     // m/s at reference_barrel_length_in
    float     barrel_length_in;       // actual barrel length, inches
    float     reference_barrel_length_in; // SAAMI reference barrel (rifle=24", 9mm=4"); 0 → default 24"
    float     mv_adjustment_factor;   // fps per inch deviation from reference barrel
    float     mass_grains;            // grains (converted internally)
    float     length_mm;              // mm
    float     caliber_inches;         // inches
    float     twist_rate_inches;      // signed: positive = RH, negative = LH
};

// ---------------------------------------------------------------------------
// Zero Configuration — SRS §9
// ---------------------------------------------------------------------------
struct ZeroConfig {
    float zero_range_m;               // meters
    float sight_height_mm;            // mm above bore axis
};

// ---------------------------------------------------------------------------
// Firing Solution — SRS §12
// ---------------------------------------------------------------------------
struct FiringSolution {
    uint32_t solution_mode;           // BCE_Mode cast to uint32
    uint32_t fault_flags;             // BCE_Fault bitfield
    uint32_t defaults_active;         // BCE_Diag bitfield

    float hold_elevation_moa;         // Total elevation hold (MOA)
    float hold_windage_moa;           // Total windage hold (MOA)

    float range_m;                    // Slant range to target
    float horizontal_range_m;         // Horizontal component
    float tof_ms;                     // Time of flight (milliseconds)
    float velocity_at_target_ms;      // Remaining velocity
    float energy_at_target_j;         // Remaining energy (joules)

    float coriolis_windage_moa;       // Coriolis windage component
    float coriolis_elevation_moa;     // Coriolis elevation component
    float spin_drift_moa;            // Spin drift component
    float wind_only_windage_moa;      // Wind-only windage component
    float earth_spin_windage_moa;     // Coriolis + spin drift windage
    float offsets_windage_moa;        // Boresight + reticle horizontal offsets
    float cant_windage_moa;           // Windage added by cant correction

    float cant_angle_deg;            // Current cant / roll angle
    float heading_deg_true;          // True heading from AHRS + mag

    float air_density_kgm3;          // Computed air density

    // Gaussian error propagation — SRS §14
    bool  uncertainty_valid;          // True when uncertainty fields are populated
    float sigma_elevation_moa;        // 1-sigma elevation uncertainty (MOA)
    float sigma_windage_moa;          // 1-sigma windage uncertainty (MOA)
    float covariance_elev_wind;       // Elevation-windage covariance (MOA²)

    // Per-input variance contributions (var_elev, var_wind) for diagnostics.
    // Each pair is (elevation_variance_moa2, windage_variance_moa2) from that input.
    // Total var_e = sum of [i][0], total var_w = sum of [i][1].
    // Index: 0=MV, 1=BC, 2=Range, 3=WindSpeed, 4=WindHeading,
    //        5=Temperature, 6=Pressure, 7=Humidity, 8=SightHeight, 9=Cant,
    //        10=Latitude, 11=Mass, 12=Length, 13=Caliber, 14=Twist,
    //        15=ZeroRange, 16=MVAdjustment
    static const int kNumUncertaintyInputs = 17;
    float uc_var_elev[17];            // per-input elevation variance (MOA²)
    float uc_var_wind[17];            // per-input windage variance (MOA²)
};

// ---------------------------------------------------------------------------
// Realtime Solution (minimal hot-path output)
// ---------------------------------------------------------------------------
struct RealtimeSolution {
    uint32_t solution_mode;           // BCE_Mode cast to uint32
    uint32_t fault_flags;             // BCE_Fault bitfield
    uint32_t defaults_active;         // BCE_Diag bitfield

    float hold_elevation_moa;         // Final elevation hold (MOA)
    float hold_windage_moa;           // Final windage hold (MOA)

    bool  uncertainty_valid;          // True when uncertainty_radius_moa is populated
    float uncertainty_radius_moa;     // Circularized 1-sigma radius in MOA
    float uncertainty_confidence;     // Confidence mass represented by radius (default 1σ = 0.682689)

    float range_m;                    // Slant range to target
    float tof_ms;                     // Time of flight (milliseconds)
    float velocity_at_target_ms;      // Remaining velocity at target
};

// ---------------------------------------------------------------------------
// Generic sensor error table — piecewise-linear sigma vs. a scalar input
// ---------------------------------------------------------------------------
/**
 * @brief A single breakpoint in a sensor accuracy table.
 *
 * `x`     is the independent variable (e.g. range_m for an LRF,
 *          temperature_c for a thermometer).
 * `sigma` is the 1-sigma uncertainty at that x value.
 */
struct BCE_ErrorPoint {
    float x;
    float sigma;
};

/**
 * @brief A reference to an array of BCE_ErrorPoint breakpoints.
 *
 * Pass to BCE_InterpolateSigma() to obtain piecewise-linearly interpolated
 * sigma values at arbitrary x positions.  Clamps to the first/last sigma
 * outside the table range.
 *
 * Set {nullptr, 0} for a "Custom / manual" entry (BCE_InterpolateSigma
 * returns 0.0f for a null table, leaving the caller's sigma unchanged).
 */
struct BCE_ErrorTable {
    const BCE_ErrorPoint* points;
    int                   count;
};

// ---------------------------------------------------------------------------
// Cartridge accuracy (CEP50) table — range-dependent dispersion model
// ---------------------------------------------------------------------------
/**
 * @brief A single breakpoint mapping range to CEP50 radius in MOA.
 */
struct BCE_CEPPoint {
    float range_m;
    float cep50_moa; // CEP50 radius (MOA)
};

/**
 * @brief A reference to an array of CEP breakpoints.
 *
 * The engine treats CEP50 radii as a target dispersion envelope and scales the
 * computed uncertainty to match, preserving directional variance ratios.
 */
struct BCE_CEPTable {
    const BCE_CEPPoint* points;
    int                 count;
};

// ---------------------------------------------------------------------------
// Uncertainty / Error Propagation Config — SRS §14
// ---------------------------------------------------------------------------
/**
 * @brief Per-input 1-sigma uncertainties for Gaussian error propagation.
 *
 * When enabled, BCE_Engine runs central finite differences over each input
 * and accumulates the resulting MOA variance into sigma_elevation_moa,
 * sigma_windage_moa, and covariance_elev_wind in the FiringSolution.
 *
 * Use BCE_GetDefaultUncertaintyConfig() to obtain sensible starting values.
 */

struct UncertaintyConfig {
    bool  enabled;                    // Master enable
    float sigma_muzzle_velocity_ms;   // 1-sigma MV uncertainty (m/s)
    float sigma_bc_fraction;          // 1-sigma BC uncertainty as fraction (e.g. 0.02 = 2%)
    float sigma_range_m;              // 1-sigma range uncertainty (m)
    float sigma_wind_speed_ms;        // 1-sigma wind speed uncertainty (m/s)
    float sigma_wind_heading_deg;     // 1-sigma wind direction uncertainty (degrees)
    float sigma_temperature_c;        // 1-sigma temperature uncertainty (°C)
    float sigma_pressure_pa;          // 1-sigma pressure uncertainty (Pa)
    float sigma_humidity;             // 1-sigma humidity uncertainty (fraction 0–1)
    float sigma_sight_height_mm;      // 1-sigma sight height uncertainty (mm)
    float sigma_cant_deg;             // 1-sigma cant angle uncertainty (degrees)
    float sigma_latitude_deg;         // 1-sigma latitude uncertainty (degrees); Coriolis only
    float sigma_mass_grains;          // 1-sigma bullet mass uncertainty (grains)
    float sigma_length_mm;            // 1-sigma bullet length uncertainty (mm)
    float sigma_caliber_inches;       // 1-sigma bullet caliber uncertainty (inches)
    float sigma_twist_rate_inches;    // 1-sigma twist rate uncertainty (in/turn)
    float sigma_zero_range_m;         // 1-sigma zero range uncertainty (m)
    float sigma_mv_adjustment_fps_per_in; // 1-sigma MV/barrel adjustment uncertainty (fps/in)

    // Optional engine-managed sensor profile inputs provided by the app layer.
    // When enabled, engine derives sigma values from live sensor inputs instead
    // of trusting GUI/manual scalar sigma entry for those channels.
    bool          use_range_error_table;
    BCE_ErrorTable range_error_table;                // x = range_m, sigma = sigma_range_m

    bool          use_temperature_error_table;
    BCE_ErrorTable temperature_error_table;          // x = temp_c, sigma = sigma_temperature_c

    bool          use_pressure_delta_temp_error_table;
    BCE_ErrorTable pressure_delta_temp_error_table;  // x = |delta_temp_c_since_cal|, sigma = sigma_pressure_pa
    float         pressure_uncalibrated_sigma_pa;    // fallback when not calibrated or invalid calibration metadata
    bool          pressure_is_calibrated;
    bool          pressure_has_calibration_temp;
    float         pressure_calibration_temp_c;

    // Optional cartridge-specific dispersion model (CEP50 in MOA) to scale
    // propagated uncertainty at each range.  Disabled when table is null/empty.
    bool         use_cartridge_cep_table;
    BCE_CEPTable cartridge_cep_table; // x = range_m, cep50_moa = CEP radius (MOA)
    float        cartridge_cep_scale_floor; // minimum multiplicative scale (>=1 recommended)
};

// ---------------------------------------------------------------------------
// Boresight / Reticle Offsets — SRS §10
// ---------------------------------------------------------------------------
struct BoresightOffset {
    float vertical_moa;
    float horizontal_moa;
};
