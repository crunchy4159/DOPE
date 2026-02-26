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
    float    lrf_confidence;          // 0.0–1.0
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
    float     muzzle_velocity_ms;     // m/s
    float     barrel_length_in;       // inches
    float     mv_adjustment_factor;   // fps per inch deviation from 24"
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

    float cant_angle_deg;            // Current cant / roll angle
    float heading_deg_true;          // True heading from AHRS + mag

    float air_density_kgm3;          // Computed air density
};

// ---------------------------------------------------------------------------
// Boresight / Reticle Offsets — SRS §10
// ---------------------------------------------------------------------------
struct BoresightOffset {
    float vertical_moa;
    float horizontal_moa;
};
