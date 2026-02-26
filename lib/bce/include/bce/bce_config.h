/**
 * @file bce_config.h
 * @brief Compile-time constants and ISA defaults for the Ballistic Core Engine.
 *
 * BCE SRS v1.3 — Sections 4, 5, 6
 */

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Version
// ---------------------------------------------------------------------------
#ifndef BCE_VERSION_MAJOR
#define BCE_VERSION_MAJOR 1
#endif
#ifndef BCE_VERSION_MINOR
#define BCE_VERSION_MINOR 3
#endif

// ---------------------------------------------------------------------------
// Maximum trajectory range (meters) — SRS §6
// ---------------------------------------------------------------------------
#define BCE_MAX_RANGE_M 2500

// Trajectory table size: 1-meter resolution from 0 to BCE_MAX_RANGE_M
#define BCE_TRAJ_TABLE_SIZE (BCE_MAX_RANGE_M + 1)

// ---------------------------------------------------------------------------
// ISA Standard Atmosphere Defaults — SRS §5.1
// ---------------------------------------------------------------------------
constexpr float BCE_DEFAULT_ALTITUDE_M     = 0.0f;
constexpr float BCE_DEFAULT_PRESSURE_PA    = 101325.0f;
constexpr float BCE_DEFAULT_TEMPERATURE_C  = 15.0f;
constexpr float BCE_DEFAULT_HUMIDITY       = 0.50f;
constexpr float BCE_DEFAULT_WIND_SPEED_MS  = 0.0f;
constexpr float BCE_DEFAULT_WIND_HEADING   = 0.0f;

// ---------------------------------------------------------------------------
// Physical Constants
// ---------------------------------------------------------------------------

// Earth rotation rate (rad/s) — SRS §11.4
constexpr float BCE_OMEGA_EARTH = 7.2921e-5f;

// Gravitational acceleration (m/s²)
constexpr float BCE_GRAVITY = 9.80665f;

// Specific gas constant for dry air (J/(kg·K))
constexpr float BCE_R_DRY_AIR = 287.05f;

// Reference speed of sound at 15 °C (m/s)
constexpr float BCE_SPEED_OF_SOUND_15C = 340.29f;

// Standard air density at sea level ISA (kg/m³)
constexpr float BCE_STD_AIR_DENSITY = 1.2250f;

// Temperature lapse rate (K/m) — troposphere
constexpr float BCE_LAPSE_RATE = 0.0065f;

// Reference barometric pressure at sea level (Pa)
constexpr float BCE_STD_PRESSURE_PA = 101325.0f;

// Kelvin offset
constexpr float BCE_KELVIN_OFFSET = 273.15f;

// Pi
constexpr float BCE_PI = 3.14159265358979323846f;

// Conversion helpers
constexpr float BCE_DEG_TO_RAD = BCE_PI / 180.0f;
constexpr float BCE_RAD_TO_DEG = 180.0f / BCE_PI;
constexpr float BCE_MOA_TO_RAD = BCE_PI / (180.0f * 60.0f);
constexpr float BCE_RAD_TO_MOA = (180.0f * 60.0f) / BCE_PI;
constexpr float BCE_GRAINS_TO_KG = 6.479891e-5f;
constexpr float BCE_INCHES_TO_M  = 0.0254f;
constexpr float BCE_MM_TO_M      = 0.001f;

// ---------------------------------------------------------------------------
// AHRS Configuration
// ---------------------------------------------------------------------------

// Sliding window size for static/dynamic detection (samples)
constexpr int BCE_AHRS_STATIC_WINDOW = 64;

// Accel variance threshold for static detection (m/s²)²
constexpr float BCE_AHRS_STATIC_THRESHOLD = 0.05f;

// Default Madgwick beta gain
constexpr float BCE_MADGWICK_DEFAULT_BETA = 0.1f;

// Default Mahony gains
constexpr float BCE_MAHONY_DEFAULT_KP = 2.0f;
constexpr float BCE_MAHONY_DEFAULT_KI = 0.005f;

// ---------------------------------------------------------------------------
// LRF staleness threshold (microseconds)
// ---------------------------------------------------------------------------
constexpr uint32_t BCE_LRF_STALE_US = 2000000; // 2 seconds

// Minimum accepted LRF confidence when provided (0.0 means unknown/unprovided)
constexpr float BCE_LRF_MIN_CONFIDENCE = 0.50f;

// ---------------------------------------------------------------------------
// Solver Configuration
// ---------------------------------------------------------------------------

// Minimum velocity before solver terminates (m/s)
constexpr float BCE_MIN_VELOCITY = 30.0f;

// Ballistic drag conversion constant used by the point-mass retardation model.
// This is a legacy tuning parameter from an older model and is not physically
// based. It is retained for compatibility but should not be modified without
// re-validating the entire system.
constexpr float BCE_BALLISTIC_DRAG_CONSTANT = 900.0f;

// External-reference calibration mode applies a drag scale below 1.0 to
// reduce modeled retardation while preserving legacy default behavior.
constexpr float BCE_EXTERNAL_REFERENCE_DRAG_SCALE = 0.84f;
constexpr float BCE_DEFAULT_DRAG_REFERENCE_SCALE = 1.0f;

// Maximum solver iterations (safety limit)
constexpr uint32_t BCE_MAX_SOLVER_ITERATIONS = 500000;

// Adaptive timestep bounds (seconds)
constexpr float BCE_DT_MIN = 0.00001f;  // 10 µs
constexpr float BCE_DT_MAX = 0.001f;    // 1 ms

// Maximum downrange distance advanced per integration step (meters)
constexpr float BCE_MAX_STEP_DISTANCE_M = 0.25f;

// Zero-angle binary search tolerance (meters of drop at zero range)
constexpr float BCE_ZERO_TOLERANCE_M = 0.001f;

// Zero-angle max iterations
constexpr uint32_t BCE_ZERO_MAX_ITERATIONS = 50;

// Thresholds for triggering zero-angle recomputation when atmosphere changes
constexpr float BCE_ZERO_RECOMPUTE_BC_FACTOR_DELTA = 0.0015f;
constexpr float BCE_ZERO_RECOMPUTE_DENSITY_DELTA = 0.005f;
constexpr float BCE_ZERO_RECOMPUTE_SOS_DELTA = 0.75f;

// ---------------------------------------------------------------------------
// Magnetometer Configuration
// ---------------------------------------------------------------------------

// Expected Earth field magnitude range (µT)
constexpr float BCE_MAG_MIN_FIELD_UT = 20.0f;
constexpr float BCE_MAG_MAX_FIELD_UT = 70.0f;
