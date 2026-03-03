/**
 * @file dope_engine.h
 * @brief Top-level DOPE engine — orchestrates all subsystems.
 *
 * DOPE SRS v1.3 — Sections 3, 9, 12, 13
 *
 * Owns all module instances (AHRS, atmosphere, solver, corrections) as
 * static objects. Implements the state machine (IDLE / SOLUTION_READY / FAULT)
 * and populates the FiringSolution structure.
 */

#pragma once

#include "dope/dope_types.h"
#include "dope/dope_config.h"
#include "../ahrs/ahrs_manager.h"
#include "../mag/mag_calibration.h"
#include "../atmo/atmosphere.h"
#include "../solver/solver.h"
#include "../corrections/wind.h"
#include "../corrections/cant.h"

class DOPE_Engine {
public:
    void init();

    // --- Primary update ---
    void update(const SensorFrame* frame);

    // --- Manual inputs ---
    void setBulletProfile(const BulletProfile* profile);
    void setZeroConfig(const ZeroConfig* config);
    void setWindManual(float speed_ms, float heading_deg);
    void setLatitude(float latitude_deg);
    void setDefaultOverrides(const DOPE_DefaultOverrides* defaults);

    // --- Calibration ---
    void setIMUBias(const float accel_bias[3], const float gyro_bias[3]);
    void setMagCalibration(const float hard_iron[3], const float soft_iron[9]);
    void setBoresightOffset(float vertical_moa, float horizontal_moa);
    void setReticleOffset(float vertical_moa, float horizontal_moa);
    void calibrateBaro();
    void calibrateGyro();
    void setAHRSAlgorithm(AHRS_Algorithm algo);
    void setAHRSConfig(const DOPE_AHRSConfig* config);
    void setLRFConfig(const DOPE_LRFConfig* config);
    void setMagDeclination(float declination_deg);
    void setExternalReferenceMode(bool enabled);

    // --- Output ---
    void getSolution(FiringSolution* out) const;
    void getRealtimeSolution(RealtimeSolution* out) const;
    DOPE_Mode getMode() const { return mode_; }
    uint32_t getFaultFlags() const { return fault_flags_; }
    uint32_t getDiagFlags() const { return diag_flags_; }

    // --- Uncertainty / error propagation --- SRS §14
    void setUncertaintyConfig(const UncertaintyConfig* config);
    static void getDefaultUncertaintyConfig(UncertaintyConfig* out);

    // --- FOV (set by encoder / optical zoom) ---
    void setFOV(float h_deg, float v_deg) { fov_h_deg_ = h_deg; fov_v_deg_ = v_deg; }
    float getHFOV() const { return fov_h_deg_; }
    float getVFOV() const { return fov_v_deg_; }

private:
    // Subsystem instances (all static, no heap)
    AHRSManager ahrs_;
    MagCalibration mag_;
    Atmosphere atmo_;
    BallisticSolver solver_;
    WindCorrection wind_;

    // State
    DOPE_Mode mode_ = DOPE_Mode::IDLE;
    uint32_t fault_flags_ = 0;
    uint32_t diag_flags_ = 0;

    // Latest firing solution
    FiringSolution solution_;

    // Bullet profile
    BulletProfile bullet_;
    bool has_bullet_ = false;

    // Zero config
    ZeroConfig zero_;
    bool has_zero_ = false;
    float zero_angle_rad_ = 0.0f;
    bool zero_dirty_ = true; // needs recomputation

    // LRF state
    float lrf_range_m_ = 0.0f;
    float lrf_range_filtered_m_ = 0.0f; // IIR filtered range
    uint64_t lrf_timestamp_us_ = 0;
    bool has_range_ = false;
    Quaternion lrf_quaternion_; // quaternion snapshot at LRF receipt

    // Latitude
    float latitude_deg_ = 0.0f;
    bool has_latitude_ = false;

    // Boresight & reticle offsets
    BoresightOffset boresight_ = {0.0f, 0.0f};
    BoresightOffset reticle_   = {0.0f, 0.0f};

    // Default overrides
    DOPE_DefaultOverrides overrides_;
    bool has_overrides_ = false;

    // Last gyro reading for calibration capture
    float last_gyro_[3] = {0, 0, 0};

    // Internal timestamps for dt computation
    uint64_t last_imu_timestamp_us_ = 0;
    bool first_update_ = true;

    // Per-frame sensor validity tracker (ingestion-level anomalies)
    bool had_invalid_sensor_input_ = false;

    // Optional solver calibration mode for external-reference alignment
    bool external_reference_mode_ = false;

    // LRF runtime config (defaults match compile-time constants)
    float    lrf_filter_alpha_       = 0.2f;
    uint32_t lrf_stale_threshold_us_ = DOPE_LRF_STALE_US;

    // Uncertainty / error propagation config — SRS §14
    UncertaintyConfig uncertainty_config_;

    // Latest internal barometer temperature sample (from SensorFrame).
    float latest_baro_temp_c_ = 0.0f;
    bool has_baro_temp_ = false;

    // FOV from zoom encoder (degrees; 0 = unknown)
    float fov_h_deg_ = 0.0f;
    float fov_v_deg_ = 0.0f;

    // --- Internal methods ---
    void evaluateState(uint64_t now_us);
    void computeSolution();
    void computeUncertainty();
    void refreshDerivedSigmasFromProfiles();
    void recomputeZero();
    SolverParams buildSolverParams(float range_m) const;
};
