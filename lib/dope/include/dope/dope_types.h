/**
 * @file dope_types.h
 * @brief All data structures for the Ballistic Core Engine.
 *
 * DOPE SRS v1.3 — Sections 3, 5.2, 7, 8, 12, 13
 */

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Operating Modes — SRS §3
// ---------------------------------------------------------------------------
enum class DOPE_Mode : uint32_t {
    IDLE           = 0,  // Insufficient data for solution
    SOLUTION_READY = 1,  // Valid firing solution available
    FAULT          = 2   // Required inputs missing or invalid
};

// ---------------------------------------------------------------------------
// Fault Flags (bitfield) — SRS §13
// ---------------------------------------------------------------------------
namespace DOPE_Fault {
    constexpr uint32_t NONE            = 0;
    constexpr uint32_t NO_RANGE        = (1u << 0);
    constexpr uint32_t NO_BULLET       = (1u << 1);
    constexpr uint32_t NO_MV           = (1u << 2);
    constexpr uint32_t NO_BC           = (1u << 3);
    constexpr uint32_t ZERO_UNSOLVABLE = (1u << 4);
    constexpr uint32_t AHRS_UNSTABLE   = (1u << 5);
    constexpr uint32_t SENSOR_INVALID  = (1u << 6);
} // namespace DOPE_Fault

// ---------------------------------------------------------------------------
// Diagnostic Flags (bitfield) — informational, not faults
// ---------------------------------------------------------------------------
namespace DOPE_Diag {
    constexpr uint32_t NONE               = 0;
    constexpr uint32_t CORIOLIS_DISABLED  = (1u << 0);
    constexpr uint32_t DEFAULT_PRESSURE   = (1u << 1);
    constexpr uint32_t DEFAULT_TEMP       = (1u << 2);
    constexpr uint32_t DEFAULT_HUMIDITY   = (1u << 3);
    constexpr uint32_t DEFAULT_ALTITUDE   = (1u << 4);
    constexpr uint32_t DEFAULT_WIND       = (1u << 5);
    constexpr uint32_t MAG_SUPPRESSED     = (1u << 6);
    constexpr uint32_t LRF_STALE          = (1u << 7);
} // namespace DOPE_Diag

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
// Barrel Material — stiffness/thermal properties selector
// ---------------------------------------------------------------------------
enum class BarrelMaterial : uint8_t {
    CMV          = 0, // 4140/4150 CMV / CrMoV
    STAINLESS_416 = 1,
    CARBON_WRAPPED = 2
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
    // Target vertical offset relative to shooter/muzzle reference.
    // Positive = target above shooter, negative = below (meters).
    float    target_elevation_m;
    // If false, target_elevation_m is ignored and treated as 0.
    bool     target_elevation_valid;

    // Zoom Encoder — SRS §7.5
    float encoder_focal_length_mm;    // mm
    bool  encoder_valid;
};

// ---------------------------------------------------------------------------
// Default Overrides — SRS §5.2
// ---------------------------------------------------------------------------
struct DOPE_DefaultOverrides {
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
    float     muzzle_diameter_in;     // Muzzle outside diameter (inches) for stiffness classification
    BarrelMaterial barrel_material;   // Material type (stiffness/thermal props)
    float     cold_bore_velocity_bias; // First-shot MV jump in fps
    float     angular_sigma_moa; // Rifle's physical cone in MOA
    float     measured_cep50_moa;     // Measured CEP50 (MOA) from manufacturer/test barrel; 0 when absent
    float     manufacturer_spec_moa;  // Manufacturer accuracy guarantee (MOA); 0 when absent
    float     category_radial_moa;    // Category-based radial dispersion default (MOA); 0 when absent
    float     category_vertical_moa;  // Category-based vertical dispersion default (MOA); 0 when absent
    float     stiffness_moa;          // Derived stiffness modifier (MOA radial) from wall thickness; computed by caller
    float     mass_grains;            // grains (converted internally)
    float     length_mm;              // mm
    float     caliber_inches;         // inches
    float     twist_rate_inches;      // signed: positive = RH, negative = LH
    bool      free_floated;           // True when barrel is free-floated (no stock contact)
    bool      suppressor_attached;    // True when a suppressor is attached
    bool      barrel_tuner_attached;  // True when a barrel tuner is installed
};

// ---------------------------------------------------------------------------
// Zero Configuration — SRS §9
// ---------------------------------------------------------------------------
struct ZeroConfig {
    float zero_range_m;               // meters
    float sight_height_mm;            // mm above bore axis
};

// ---------------------------------------------------------------------------
// AHRS Tuning Configuration
//
// IMU-hardware-specific parameters that must be supplied by the application
// layer (GUI / firmware) from its hardware preset tables.  If DOPE_SetAHRSConfig
// is never called the engine starts with built-in neutral defaults.
// ---------------------------------------------------------------------------
struct DOPE_AHRSConfig {
    float    madgwick_beta;           // Madgwick gradient-descent gain (hardware-tuned)
    float    mahony_kp;               // Mahony proportional gain       (hardware-tuned)
    float    mahony_ki;               // Mahony integral gain           (hardware-tuned)
    int      static_window;           // Static-detection ring-buffer size (samples, clamped to [1,64])
    float    static_threshold_mss2;   // Accel variance threshold for static detect ((m/s²)²)
};

// ---------------------------------------------------------------------------
// LRF Hardware Configuration
//
// LRF-hardware-specific parameters supplied by the application layer.
// If DOPE_SetLRFConfig is never called the engine uses built-in defaults.
// ---------------------------------------------------------------------------
struct DOPE_LRFConfig {
    float    filter_alpha;            // IIR smoothing weight [0–1]; higher = less smoothing
    uint32_t stale_threshold_us;      // Range reading marked stale after this many µs
};

// ---------------------------------------------------------------------------
// Firing Solution — SRS §12
// ---------------------------------------------------------------------------
struct FiringSolution {
    uint32_t solution_mode;           // DOPE_Mode cast to uint32
    uint32_t fault_flags;             // DOPE_Fault bitfield
    uint32_t defaults_active;         // DOPE_Diag bitfield

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
    // Index: 0=MV, 1=Range, 2=WindSpeed, 3=WindHeading,
    //        4=Temperature, 5=Pressure, 6=Humidity, 7=SightHeight, 8=Cant,
    //        9=Latitude, 10=Twist, 11=ZeroRange, 12=MVAdjustment
    static const int kNumUncertaintyInputs = 13;
    float uc_var_elev[13];            // per-input elevation variance (MOA²)
    float uc_var_wind[13];            // per-input windage variance (MOA²)
};

// ---------------------------------------------------------------------------
// Realtime Solution (minimal hot-path output)
// ---------------------------------------------------------------------------
struct RealtimeSolution {
    uint32_t solution_mode;           // DOPE_Mode cast to uint32
    uint32_t fault_flags;             // DOPE_Fault bitfield
    uint32_t defaults_active;         // DOPE_Diag bitfield

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
struct DOPE_ErrorPoint {
    float x;
    float sigma;
};

/**
 * @brief A reference to an array of DOPE_ErrorPoint breakpoints.
 *
 * Pass to DOPE_InterpolateSigma() to obtain piecewise-linearly interpolated
 * sigma values at arbitrary x positions.  Clamps to the first/last sigma
 * outside the table range.
 *
 * Set {nullptr, 0} for a "Custom / manual" entry (DOPE_InterpolateSigma
 * returns 0.0f for a null table, leaving the caller's sigma unchanged).
 */
struct DOPE_ErrorTable {
    const DOPE_ErrorPoint* points;
    int                   count;
};

// ---------------------------------------------------------------------------
// Cartridge accuracy (CEP50) table — range-dependent dispersion model
// ---------------------------------------------------------------------------
/**
 * @brief A single breakpoint mapping range to CEP50 radius in MOA.
 */
struct DOPE_CEPPoint {
    float range_m;
    float cep50_moa; // CEP50 radius (MOA)
};


// ---------------------------------------------------------------------------
// Uncertainty / Error Propagation Config — SRS §14
// ---------------------------------------------------------------------------
/**
 * @brief Per-input 1-sigma uncertainties for Gaussian error propagation.
 *
 * When enabled, DOPE_Engine runs central finite differences over each input
 * and accumulates the resulting MOA variance into sigma_elevation_moa,
 * sigma_windage_moa, and covariance_elev_wind in the FiringSolution.
 *
 * Use DOPE_GetDefaultUncertaintyConfig() to obtain sensible starting values.
 */

struct UncertaintyConfig {
    bool  enabled;                    // Master enable
    float sigma_muzzle_velocity_ms;   // 1-sigma MV uncertainty (m/s)
    float sigma_range_m;              // 1-sigma range uncertainty (m)
    float sigma_wind_speed_ms;        // 1-sigma wind speed uncertainty (m/s)
    float sigma_wind_heading_deg;     // 1-sigma wind direction uncertainty (degrees)
    float sigma_temperature_c;        // 1-sigma temperature uncertainty (°C)
    float sigma_pressure_pa;          // 1-sigma pressure uncertainty (Pa)
    float sigma_humidity;             // 1-sigma humidity uncertainty (fraction 0–1)
    float sigma_sight_height_mm;      // 1-sigma sight height uncertainty (mm)
    float sigma_cant_deg;             // 1-sigma cant angle uncertainty (degrees)
    float sigma_latitude_deg;         // 1-sigma latitude uncertainty (degrees); Coriolis only
    float sigma_twist_rate_inches;    // 1-sigma twist rate uncertainty (in/turn)
    float sigma_zero_range_m;         // 1-sigma zero range uncertainty (m)
    float sigma_mv_adjustment_fps_per_in; // 1-sigma MV/barrel adjustment uncertainty (fps/in)

    // Optional engine-managed sensor profile inputs provided by the app layer.
    // When enabled, engine derives sigma values from live sensor inputs instead
    // of trusting GUI/manual scalar sigma entry for those channels.
    bool          use_range_error_table;
    DOPE_ErrorTable range_error_table;                // x = range_m, sigma = sigma_range_m

    bool          use_temperature_error_table;
    DOPE_ErrorTable temperature_error_table;          // x = temp_c, sigma = sigma_temperature_c

    bool          use_pressure_delta_temp_error_table;
    DOPE_ErrorTable pressure_delta_temp_error_table;  // x = |delta_temp_c_since_cal|, sigma = sigma_pressure_pa
    float         pressure_uncalibrated_sigma_pa;    // fallback when not calibrated or invalid calibration metadata
    bool          pressure_is_calibrated;
    bool          pressure_has_calibration_temp;
    float         pressure_calibration_temp_c;

    // Optional cartridge-specific dispersion model (CEP50 in MOA) to scale
};

// ---------------------------------------------------------------------------
// Trajectory Table Point — per-metre record from the ballistic solver
// ---------------------------------------------------------------------------
/**
 * One entry from the solver's 1-metre-resolution trajectory table.
 * Coordinates are in the bore-axis frame (X = downrange horizontal).
 *
 * drop_m is always ≤ 0 at the reference bore line; to convert to world-space
 * height add  x_m * tan(launch_angle_rad).
 */
struct TrajectoryPoint {
    float drop_m;        // Vertical displacement from bore line (m, negative = below bore)
    float windage_m;     // Lateral deflection (m, positive = right)
    float velocity_ms;   // Bullet speed at this range (m/s)
    float tof_s;         // Time of flight to this range (s)
    float energy_j;      // Kinetic energy at this range (J)
};

// ---------------------------------------------------------------------------
// Boresight / Reticle Offsets — SRS §10
// ---------------------------------------------------------------------------
struct BoresightOffset {
    float vertical_moa;
    float horizontal_moa;
};
