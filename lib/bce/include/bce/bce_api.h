/**
 * @file bce_api.h
 * @brief Public C-linkage API for the Ballistic Core Engine.
 *
 * BCE SRS v1.3 — All external entry points.
 *
 * This header is the ONLY file application code needs to include.
 * All functions use static internal state — no heap allocation after BCE_Init().
 */

#pragma once

#include "bce_types.h"
#include "bce_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/**
 * Initialize the BCE. Zeros all state, applies ISA defaults.
 * Must be called once before any other BCE function.
 */
void BCE_Init(void);

// ---------------------------------------------------------------------------
// Sensor Ingestion — SRS §7
// ---------------------------------------------------------------------------

/**
 * Feed a sensor frame into the engine.
 * This is the primary update entry point — drives AHRS, atmosphere,
 * range tracking, and solution recomputation.
 */
void BCE_Update(const SensorFrame* frame);

// ---------------------------------------------------------------------------
// Manual Inputs — SRS §8
// ---------------------------------------------------------------------------

/**
 * Set the bullet profile. Triggers zero recomputation.
 */
void BCE_SetBulletProfile(const BulletProfile* profile);

/**
 * Set zero configuration (zero range + sight height). Triggers zero recomputation.
 */
void BCE_SetZeroConfig(const ZeroConfig* config);

/**
 * Set manual wind. Persists until changed.
 * @param speed_ms   Wind speed in m/s
 * @param heading_deg Wind heading in degrees true
 */
void BCE_SetWindManual(float speed_ms, float heading_deg);

/**
 * Set latitude for Coriolis/Eötvös correction.
 * Pass NAN or do not call to leave Coriolis disabled.
 */
void BCE_SetLatitude(float latitude_deg);

// ---------------------------------------------------------------------------
// Default Overrides — SRS §5.2
// ---------------------------------------------------------------------------

/**
 * Apply default overrides (e.g. from user profile).
 * Any field with its use_* flag set replaces the ISA default.
 */
void BCE_SetDefaultOverrides(const BCE_DefaultOverrides* defaults);

// ---------------------------------------------------------------------------
// Calibration — SRS §10
// ---------------------------------------------------------------------------

/**
 * Set accelerometer and gyroscope bias offsets.
 * Applied before AHRS fusion.
 */
void BCE_SetIMUBias(const float accel_bias[3], const float gyro_bias[3]);

/**
 * Set magnetometer hard-iron and soft-iron calibration.
 * hard_iron[3]: offset vector to subtract.
 * soft_iron[9]: 3×3 correction matrix (row-major).
 */
void BCE_SetMagCalibration(const float hard_iron[3], const float soft_iron[9]);

/**
 * Capture current gyroscope readings to establish a bias offset.
 * The rifle must be held perfectly still for ~1 second.
 */
void BCE_CalibrateGyro(void);

/**
 * Set boresight offset (mechanical misalignment between optic and bore).
 */
void BCE_SetBoresightOffset(const BoresightOffset* offset);

/**
 * Set reticle mechanical offset.
 */
void BCE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa);

/**
 * Capture current barometric pressure as the reference baseline.
 */
void BCE_CalibrateBaro(void);

// ---------------------------------------------------------------------------
// AHRS Configuration
// ---------------------------------------------------------------------------

/**
 * Select the AHRS fusion algorithm.
 * Default is MADGWICK.
 */
void BCE_SetAHRSAlgorithm(AHRS_Algorithm algo);

/**
 * Set AHRS filter tuning parameters (filter gains + static-detection thresholds).
 *
 * These values are IMU-hardware-specific and should be supplied by the
 * application layer from its hardware preset tables.  If not called the engine
 * uses built-in neutral defaults suitable for initial integration.
 */
void BCE_SetAHRSConfig(const BCE_AHRSConfig* config);

/**
 * Set LRF hardware parameters (IIR filter weight and staleness threshold).
 *
 * Should be called after BCE_Init() once the LRF hardware type is known.
 * If not called the engine uses built-in defaults.
 */
void BCE_SetLRFConfig(const BCE_LRFConfig* config);

/**
 * Set magnetic declination for heading correction.
 * @param declination_deg  Declination in degrees (east positive).
 */
void BCE_SetMagDeclination(float declination_deg);

/**
 * Enable/disable external-reference calibration mode.
 * When enabled, solver drag is scaled for closer alignment to external
 * reference calculators while preserving default legacy behavior when off.
 */
void BCE_SetExternalReferenceMode(bool enabled);

// ---------------------------------------------------------------------------
// Uncertainty Configuration
// ---------------------------------------------------------------------------

/**
 * Interpolate a sensor accuracy table at a given x value.
 *
 * Returns the piecewise-linearly interpolated 1-sigma value from `table`
 * at position `x`.  Below the first breakpoint the first sigma is returned;
 * above the last breakpoint the last sigma is returned.  Returns 0.0f if
 * `table` is null or empty ("Custom / manual" sentinel).
 *
 * This utility lives in the BCE library so that top-level deployment
 * applications can use the same interpolation implementation as the GUI
 * without carrying their own copy.
 *
 * @param table  Pointer to a populated BCE_ErrorTable (may be null/empty).
 * @param x      Independent variable (e.g. range_m, temperature_c).
 * @return       1-sigma estimate at x, or 0.0f for a null/empty table.
 */
float BCE_InterpolateSigma(const BCE_ErrorTable* table, float x);

/**
 * Set the uncertainty (1σ) configuration for Gaussian error propagation.
 * When enabled, the engine runs perturbed solver passes and computes a
 * 2×2 covariance matrix (elevation × windage) on the firing solution.
 *
 * Application-layer sensor profiles (BCE_ErrorTable) can be supplied here
 * so the engine derives sigma values from live sensor state (range, baro temp,
 * pressure-vs-delta-temp calibration drift) rather than relying on GUI-side
 * interpolation logic.
 */
void BCE_SetUncertaintyConfig(const UncertaintyConfig* config);

/**
 * Retrieve the default uncertainty configuration with sensible hardware
 * defaults pre-populated. Caller can adjust and pass to BCE_SetUncertaintyConfig.
 */
void BCE_GetDefaultUncertaintyConfig(UncertaintyConfig* out);

// ---------------------------------------------------------------------------
// Output — SRS §12
// ---------------------------------------------------------------------------

/**
 * Retrieve the latest firing solution.
 * @param out  Pointer to caller-owned FiringSolution struct to fill.
 */
void BCE_GetSolution(FiringSolution* out);

/**
 * Retrieve the latest minimal realtime solution payload.
 * @param out  Pointer to caller-owned RealtimeSolution struct to fill.
 */
void BCE_GetRealtimeSolution(RealtimeSolution* out);

/**
 * Get the current engine operating mode.
 */
BCE_Mode BCE_GetMode(void);

/**
 * Get current fault flags.
 */
uint32_t BCE_GetFaultFlags(void);

/**
 * Get current diagnostic flags (defaults active, etc.).
 */
uint32_t BCE_GetDiagFlags(void);

/**
 * Get the horizontal field-of-view (degrees) computed from the latest valid
 * zoom encoder reading.  Returns 0 if no valid encoder frame has been received.
 */
float BCE_GetHFOV(void);

/**
 * Get the vertical field-of-view (degrees) computed from the latest valid
 * zoom encoder reading.  Returns 0 if no valid encoder frame has been received.
 */
float BCE_GetVFOV(void);

#ifdef __cplusplus
} // extern "C"
#endif
