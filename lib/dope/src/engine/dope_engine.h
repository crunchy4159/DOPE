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
    void setAmmoDatasetV2(const AmmoDatasetV2* dataset);
    void setBallisticContext(const BallisticContext* context);
    void setRifleAmmoCalibrationProfile(const RifleAmmoCalibrationProfile* profile);
    void setModuleCapabilities(const ModuleCapabilities* caps);
    void recordShotObservation(const ShotObservation* obs);
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
    void notifyShotFired(uint64_t timestamp_us, float ambient_temp_c);

    // --- Output ---
    void getSolution(FiringSolution* out) const;
    void getRealtimeSolution(RealtimeSolution* out) const;
    DOPE_Mode getMode() const { return mode_; }
    uint32_t getFaultFlags() const { return fault_flags_; }
    uint32_t getDiagFlags() const { return diag_flags_; }

    /**
     * Read one point from the internal trajectory table.
     * Returns false if range_m is out of range or no solution exists.
     */
    bool getTrajectoryPoint(int range_m, TrajectoryPoint* out) const;

    // --- Uncertainty / error propagation --- SRS §14
    void setUncertaintyConfig(const UncertaintyConfig* config);
    static void getDefaultUncertaintyConfig(UncertaintyConfig* out);

    // --- Uncertainty scheduling ---
    void setDeferUncertainty(bool enabled);
    void computeUncertaintyOnly();

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
    AmmoDatasetV2 dataset_v2_;
    bool has_dataset_v2_ = false;
    BallisticContext ballistic_context_;
    bool has_ballistic_context_ = false;
    RifleAmmoCalibrationProfile calibration_profile_;
    bool has_calibration_profile_ = false;
    ModuleCapabilities module_caps_ = {true, true, true};

    // Zero config
    ZeroConfig zero_;
    bool has_zero_ = false;
    float zero_angle_rad_ = 0.0f;
    bool zero_dirty_ = true; // needs recomputation

    // LRF state
    float lrf_range_m_ = 0.0f;
    float lrf_range_filtered_m_ = 0.0f; // IIR filtered range
    float target_elevation_m_ = 0.0f;
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

    // Barrel thermal/stringing state
    float barrel_temp_K_ = 293.15f;        // current barrel temp (K)
    float barrel_ambient_K_ = 293.15f;     // last known ambient (K)
    uint64_t last_barrel_update_us_ = 0;   // last cooling integration timestamp
    uint64_t last_shot_time_us_ = 0;       // last shot event timestamp
    int shots_in_string_ = 0;              // rolling shot count for stringing heuristics

    // FOV from zoom encoder (degrees; 0 = unknown)
    float fov_h_deg_ = 0.0f;
    float fov_v_deg_ = 0.0f;

    // Uncertainty scheduling
    bool defer_uncertainty_ = false;
    bool uncertainty_pending_ = false;

    // Active ammo selector — centralize legacy vs v2 choice for migration
    struct ActiveAmmo {
        bool valid = false;           // true when any source is available
        bool is_v2 = false;           // true when source is AmmoDatasetV2
        float bc = 0.0f;
        float muzzle_velocity_ms = 0.0f;
        float mass_grains = 0.0f;
        float length_mm = 0.0f;
        float caliber_inches = 0.0f;
        float twist_rate_inches = 0.0f;
        float mv_adjustment_fps_per_in = 0.0f;
        float baseline_barrel_length_in = 24.0f;
        DragModel drag_model = static_cast<DragModel>(0);
        // Bitmask of supported drag models from the dataset (Bit0 => G1 ... Bit7 => G8)
        uint8_t supported_drag_models_mask = 0;
    };

    ActiveAmmo selectActiveAmmo() const;

    // --- Internal methods ---
    void evaluateState(uint64_t now_us);
    void computeSolution();
    void computeUncertainty();
    void refreshDerivedSigmasFromProfiles();
    float interpolateProfileValue(const DOPE_ProfilePoint* points, int count, float range_m) const;
    bool computeTableFirstSolution(float slant_range_m, float horizontal_range_m, float target_elevation_m,
                                   float pitch_rad, float roll_rad);
    float getActiveAmmoCep50(float range_m) const;
    void recomputeZero();
    SolverParams buildSolverParams(float range_m) const;
    bool hasUsableSolverInputs() const;
    void integrateBarrelCooling(uint64_t now_us);
    float estimateBarrelMassKg() const;
    float barrelHeatMultiplier() const;   // sigma growth factor for uncertainty scaling
    float thermalMvDeltaFps() const;      // MV shift in fps from barrel heat
    void  thermalPoiDrift(float& drift_x_moa, float& drift_y_moa) const; // POI walk from thermal expansion
};
