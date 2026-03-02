/**
 * @file dope_config.h
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
constexpr float BCE_DEFAULT_HUMIDITY       = 0.5f;
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

// Dynamic-stability (SG) coupling into drag model.
// Effective drag scale is:
//   1 + BCE_SG_DRAG_COUPLING_GAIN * (BCE_SG_REFERENCE - SG)
// then clamped to [BCE_SG_DRAG_SCALE_MIN, BCE_SG_DRAG_SCALE_MAX].
constexpr float BCE_SG_REFERENCE          = 1.5f;
constexpr float BCE_SG_DRAG_COUPLING_GAIN = 0.010f;
constexpr float BCE_SG_DRAG_SCALE_MIN     = 0.985f;
constexpr float BCE_SG_DRAG_SCALE_MAX     = 1.040f;

// AHRS tuning compile-time defaults — SRS §11.1
// These also serve as fallback member-initialiser values in madgwick.h /
// mahony.h / ahrs_manager.h.  Runtime overrides are applied via
// BCE_SetAHRSConfig() / BCE_SetLRFConfig() (see BCE_AHRSConfig / BCE_LRFConfig
// in dope_types.h); the constants below remain the defaults when those calls
// are never made.
constexpr float    BCE_MADGWICK_DEFAULT_BETA   = 0.1f;
constexpr float    BCE_MAHONY_DEFAULT_KP       = 2.0f;
constexpr float    BCE_MAHONY_DEFAULT_KI       = 0.005f;
constexpr int      BCE_AHRS_STATIC_WINDOW      = 64;   // ring-buffer size (samples)
constexpr float    BCE_AHRS_STATIC_THRESHOLD   = 0.05f; // accel variance threshold ((m/s²)²)

// LRF compile-time defaults
constexpr uint32_t BCE_LRF_STALE_US            = 2000000u; // 2 seconds

// ---------------------------------------------------------------------------
// Solver Configuration
// ---------------------------------------------------------------------------

// Minimum velocity before solver terminates (m/s)
constexpr float BCE_MIN_VELOCITY = 30.0f;

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

// ---------------------------------------------------------------------------
// Zoom Encoder / Optical FOV — SRS §7.5  (future use)
// ---------------------------------------------------------------------------

// Zoom Encoder / Optical FOV — SRS §7.5
// ---------------------------------------------------------------------------

// IMX477 (1/2.3") sensor half-dimensions for FOV formula:
//   FOV = 2 · atan(half_dimension / focal_length_mm)
// Half-width = 6.287 mm / 2 = 3.1435 mm
// Half-height = 4.712 mm / 2 = 2.3560 mm
constexpr float BCE_SENSOR_HALF_WIDTH_MM  = 3.1435f;
constexpr float BCE_SENSOR_HALF_HEIGHT_MM = 2.3560f;

// Minimum valid focal length (mm); readings at or below this are rejected.
constexpr float BCE_ENCODER_MIN_FOCAL_LENGTH_MM = 1.0f;

// ---------------------------------------------------------------------------
// Math constant aliases — these moved to dope_math_utils.h under
// namespace bce::math; BCE_-prefixed aliases are kept here so existing
// call-sites (tests, harness) require no further changes.
// ---------------------------------------------------------------------------
#include "dope_math_utils.h"

constexpr float BCE_PI           = bce::math::PI;
constexpr float BCE_DEG_TO_RAD   = bce::math::DEG_TO_RAD;
constexpr float BCE_RAD_TO_DEG   = bce::math::RAD_TO_DEG;
constexpr float BCE_MOA_TO_RAD   = bce::math::MOA_TO_RAD;
constexpr float BCE_RAD_TO_MOA   = bce::math::RAD_TO_MOA;
constexpr float BCE_GRAINS_TO_KG = bce::math::GRAINS_TO_KG;
constexpr float BCE_INCHES_TO_M  = bce::math::INCHES_TO_M;
constexpr float BCE_MM_TO_M      = bce::math::MM_TO_M;
