/**
 * @file bce_engine.cpp
 * @brief BCE engine implementation — the top-level orchestrator.
 *
 * Pipeline per BCE_Update:
 *   1. Feed IMU/mag → AHRS
 *   2. Feed baro → Atmosphere
 *   3. Store LRF range + quaternion snapshot
 *   4. Evaluate state machine
 *   5. If data sufficient → run solver → apply corrections → populate FiringSolution
 */

#include "bce_engine.h"
#include <cmath>
#include <cstring>

namespace {
constexpr float BCE_LRF_FILTER_ALPHA = 0.2f;
}

void BCE_Engine::init() {
    ahrs_.init();
    mag_.init();
    atmo_.init();
    solver_.init();

    mode_ = BCE_Mode::IDLE;
    fault_flags_ = 0;
    diag_flags_ = 0;

    std::memset(&solution_, 0, sizeof(solution_));
    std::memset(&bullet_, 0, sizeof(bullet_));
    std::memset(&zero_, 0, sizeof(zero_));
    std::memset(&overrides_, 0, sizeof(overrides_));

    has_bullet_ = false;
    has_zero_ = false;
    has_range_ = false;
    has_latitude_ = false;
    has_overrides_ = false;

    zero_angle_rad_ = 0.0f;
    zero_dirty_ = true;

    lrf_range_m_ = 0.0f;
    lrf_timestamp_us_ = 0;
    lrf_quaternion_ = {1, 0, 0, 0};

    latitude_deg_ = 0.0f;
    boresight_ = {0, 0};
    reticle_ = {0, 0};

    std::memset(last_gyro_, 0, sizeof(last_gyro_));
    last_imu_timestamp_us_ = 0;
    first_update_ = true;
    had_invalid_sensor_input_ = false;
    external_reference_mode_ = false;

    getDefaultUncertaintyConfig(&uncertainty_config_);
    fov_h_deg_ = 0.0f;
    fov_v_deg_ = 0.0f;

    solution_.solution_mode = static_cast<uint32_t>(BCE_Mode::IDLE);
}

void BCE_Engine::update(const SensorFrame* frame) {
    if (!frame) return;

    had_invalid_sensor_input_ = false;

    uint64_t now_us = frame->timestamp_us;

    // --- 1. AHRS Update ---
    if (frame->imu_valid) {
        bool imu_finite = std::isfinite(frame->accel_x) && std::isfinite(frame->accel_y) && std::isfinite(frame->accel_z) &&
                          std::isfinite(frame->gyro_x) && std::isfinite(frame->gyro_y) && std::isfinite(frame->gyro_z);
        if (!imu_finite) {
            had_invalid_sensor_input_ = true;
        }

        float dt = 0.01f; // default 100 Hz
        if (!first_update_ && now_us > last_imu_timestamp_us_) {
            dt = static_cast<float>(now_us - last_imu_timestamp_us_) * 1e-6f;
            if (dt > 0.1f) dt = 0.1f; // cap at 100ms for safety
            if (dt < 0.0001f) dt = 0.0001f;
        }
        first_update_ = false;
        last_imu_timestamp_us_ = now_us;

        // Store last gyro for calibration
        if (imu_finite) {
            last_gyro_[0] = frame->gyro_x;
            last_gyro_[1] = frame->gyro_y;
            last_gyro_[2] = frame->gyro_z;
        }

        // Magnetometer: apply calibration and check disturbance
        float mx = frame->mag_x, my = frame->mag_y, mz = frame->mag_z;
        bool use_mag = false;
        if (frame->mag_valid) {
            bool mag_finite = std::isfinite(mx) && std::isfinite(my) && std::isfinite(mz);
            if (!mag_finite) {
                had_invalid_sensor_input_ = true;
            } else {
                bool mag_ok = mag_.apply(mx, my, mz);
                use_mag = mag_ok; // suppress if disturbed
            }
        }

        if (imu_finite) {
            ahrs_.update(frame->accel_x, frame->accel_y, frame->accel_z,
                          frame->gyro_x, frame->gyro_y, frame->gyro_z,
                          mx, my, mz, use_mag, dt);
        }
    }

    // --- 2. Barometer → Atmosphere ---
    if (frame->baro_valid) {
        float humidity = frame->baro_humidity_valid ? frame->baro_humidity : -1.0f;
        atmo_.updateFromBaro(frame->baro_pressure_pa, frame->baro_temperature_c, humidity);
        if (atmo_.consumeZeroRecomputeHint()) {
            zero_dirty_ = true;
        }
    }

    // --- 3. LRF Range ---
    if (frame->lrf_valid) {
        if (!std::isfinite(frame->lrf_range_m)) {
            had_invalid_sensor_input_ = true;
        }

        bool range_valid = std::isfinite(frame->lrf_range_m) &&
                           frame->lrf_range_m > 0.0f &&
                           frame->lrf_range_m <= static_cast<float>(BCE_MAX_RANGE_M);

        float confidence = frame->lrf_confidence;
        bool confidence_provided = confidence > 0.0f;
        bool confidence_in_range = std::isfinite(confidence) && confidence >= 0.0f && confidence <= 1.0f;
        bool confidence_valid = !confidence_provided ||
                                (confidence_in_range && confidence >= BCE_LRF_MIN_CONFIDENCE);

        if (confidence_provided && !confidence_in_range) {
            had_invalid_sensor_input_ = true;
        }

        if (range_valid && confidence_valid) {
            if (!has_range_) {
                // First valid range, initialize filter
                lrf_range_filtered_m_ = frame->lrf_range_m;
            } else {
                // Apply IIR filter
                lrf_range_filtered_m_ = BCE_LRF_FILTER_ALPHA * frame->lrf_range_m +
                                        (1.0f - BCE_LRF_FILTER_ALPHA) * lrf_range_filtered_m_;
            }
            lrf_range_m_ = frame->lrf_range_m;
            lrf_timestamp_us_ = frame->lrf_timestamp_us;
            lrf_quaternion_ = ahrs_.getQuaternion();
            has_range_ = true;
        }
    }

    // --- 4. Zoom encoder → FOV computation — SRS §7.5
    if (frame->encoder_valid &&
        frame->encoder_focal_length_mm > BCE_ENCODER_MIN_FOCAL_LENGTH_MM) {
        float f = frame->encoder_focal_length_mm;
        fov_h_deg_ = 2.0f * std::atan(BCE_SENSOR_HALF_WIDTH_MM  / f) * BCE_RAD_TO_DEG;
        fov_v_deg_ = 2.0f * std::atan(BCE_SENSOR_HALF_HEIGHT_MM / f) * BCE_RAD_TO_DEG;
    }

    // --- 5. Evaluate state and compute solution ---
    evaluateState(now_us);
}

void BCE_Engine::setBulletProfile(const BulletProfile* profile) {
    if (!profile) return;
    bullet_ = *profile;
    has_bullet_ = true;
    zero_dirty_ = true;
}

void BCE_Engine::setZeroConfig(const ZeroConfig* config) {
    if (!config) return;
    zero_ = *config;
    has_zero_ = true;
    zero_dirty_ = true;
}

void BCE_Engine::setWindManual(float speed_ms, float heading_deg) {
    wind_.setWind(speed_ms, heading_deg);
}

void BCE_Engine::setLatitude(float latitude_deg) {
    if (std::isnan(latitude_deg)) {
        has_latitude_ = false;
    } else {
        latitude_deg_ = latitude_deg;
        has_latitude_ = true;
    }
}

void BCE_Engine::setDefaultOverrides(const BCE_DefaultOverrides* defaults) {
    if (!defaults) return;
    overrides_ = *defaults;
    has_overrides_ = true;
    atmo_.applyDefaults(overrides_);

    if (defaults->use_latitude) {
        setLatitude(defaults->latitude_deg);
    }

    if (defaults->use_wind) {
        wind_.setWind(defaults->wind_speed_ms, defaults->wind_heading_deg);
    }

    zero_dirty_ = true; // atmosphere changed → zero must recompute
}

void BCE_Engine::setIMUBias(const float accel_bias[3], const float gyro_bias[3]) {
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    ahrs_.setAccelBias(accel_bias ? accel_bias : zero);
    ahrs_.setGyroBias(gyro_bias ? gyro_bias : zero);
}

void BCE_Engine::setMagCalibration(const float hard_iron[3], const float soft_iron[9]) {
    const float zero_hi[3] = {0.0f, 0.0f, 0.0f};
    const float identity_si[9] = {1.0f, 0.0f, 0.0f,
                                  0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f};
    mag_.setCalibration(hard_iron ? hard_iron : zero_hi,
                        soft_iron ? soft_iron : identity_si);
}

void BCE_Engine::setBoresightOffset(float vertical_moa, float horizontal_moa) {
    boresight_.vertical_moa = vertical_moa;
    boresight_.horizontal_moa = horizontal_moa;
}

void BCE_Engine::setReticleOffset(float vertical_moa, float horizontal_moa) {
    reticle_.vertical_moa = vertical_moa;
    reticle_.horizontal_moa = horizontal_moa;
}

void BCE_Engine::calibrateBaro() {
    atmo_.calibrateBaro();
    zero_dirty_ = true;
}

void BCE_Engine::calibrateGyro() {
    ahrs_.captureGyroBias(last_gyro_[0], last_gyro_[1], last_gyro_[2]);
}

void BCE_Engine::setAHRSAlgorithm(AHRS_Algorithm algo) {
    ahrs_.setAlgorithm(algo);
}

void BCE_Engine::setMagDeclination(float declination_deg) {
    mag_.setDeclination(declination_deg);
}

void BCE_Engine::setExternalReferenceMode(bool enabled) {
    external_reference_mode_ = enabled;
}

void BCE_Engine::getSolution(FiringSolution* out) const {
    if (out) {
        *out = solution_;
    }
}

// ---------------------------------------------------------------------------
// Internal: state machine evaluation
// ---------------------------------------------------------------------------

void BCE_Engine::evaluateState(uint64_t now_us) {
    fault_flags_ = 0;
    diag_flags_ = atmo_.getDiagFlags();

    // Check hard faults — SRS §13
    if (!has_range_) {
        fault_flags_ |= BCE_Fault::NO_RANGE;
    } else if (now_us > lrf_timestamp_us_ + BCE_LRF_STALE_US) {
        // LRF stale — not a hard fault, but range is invalid
        has_range_ = false;
        fault_flags_ |= BCE_Fault::NO_RANGE;
        diag_flags_ |= BCE_Diag::LRF_STALE;
    }

    if (!has_bullet_) {
        fault_flags_ |= BCE_Fault::NO_BULLET;
    } else {
        if (bullet_.muzzle_velocity_ms < 1.0f) {
            fault_flags_ |= BCE_Fault::NO_MV;
        }
        if (bullet_.bc < 0.001f) {
            fault_flags_ |= BCE_Fault::NO_BC;
        }

        if (has_zero_ && (zero_.zero_range_m < 1.0f ||
                          zero_.zero_range_m > static_cast<float>(BCE_MAX_RANGE_M))) {
            fault_flags_ |= BCE_Fault::ZERO_UNSOLVABLE;
        }
    }

    if (!ahrs_.isStable()) {
        fault_flags_ |= BCE_Fault::AHRS_UNSTABLE;
    }

    if (!has_latitude_) {
        diag_flags_ |= BCE_Diag::CORIOLIS_DISABLED;
    }

    if (mag_.isDisturbed()) {
        diag_flags_ |= BCE_Diag::MAG_SUPPRESSED;
    }

    if (!wind_.isSet()) {
        diag_flags_ |= BCE_Diag::DEFAULT_WIND;
    }

    if (atmo_.hadInvalidInput() || had_invalid_sensor_input_) {
        fault_flags_ |= BCE_Fault::SENSOR_INVALID;
    }

    // Determine mode
    if (fault_flags_ != 0) {
        // Check which faults are actually hard faults
        uint32_t hard_faults = fault_flags_ & (BCE_Fault::NO_RANGE |
                                                 BCE_Fault::NO_BULLET |
                                                 BCE_Fault::NO_MV |
                                                 BCE_Fault::NO_BC |
                                                 BCE_Fault::AHRS_UNSTABLE |
                                                 BCE_Fault::ZERO_UNSOLVABLE);
        if (hard_faults != 0) {
            mode_ = BCE_Mode::FAULT;
            solution_.solution_mode = static_cast<uint32_t>(BCE_Mode::FAULT);
            solution_.fault_flags = fault_flags_;
            solution_.defaults_active = diag_flags_;
            return;
        }
    }

    // Have enough data — check if we can compute
    if (has_range_ && has_bullet_ &&
        bullet_.muzzle_velocity_ms > 1.0f && bullet_.bc > 0.001f) {
        computeSolution();
        mode_ = BCE_Mode::SOLUTION_READY;
    } else {
        mode_ = BCE_Mode::IDLE;
        solution_.solution_mode = static_cast<uint32_t>(BCE_Mode::IDLE);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
    }
}

// ---------------------------------------------------------------------------
// Internal: compute firing solution
// ---------------------------------------------------------------------------

void BCE_Engine::computeSolution() {
    // Recompute zero if dirty
    if (zero_dirty_) {
        recomputeZero();
    }

    if (fault_flags_ & BCE_Fault::ZERO_UNSOLVABLE) {
        mode_ = BCE_Mode::FAULT;
        solution_.solution_mode = static_cast<uint32_t>(BCE_Mode::FAULT);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
        return;
    }

    // Get current pitch (bore elevation) and heading from AHRS
    float pitch = ahrs_.getPitch();
    float roll  = ahrs_.getRoll();
    float yaw   = ahrs_.getYaw();
    float heading_true = mag_.computeHeading(yaw);

    // Build solver params
    SolverParams params = buildSolverParams(lrf_range_filtered_m_);
    params.launch_angle_rad = zero_angle_rad_ + pitch;

    // Run solver
    SolverResult result = solver_.integrate(params);

    if (!result.valid) {
        // Zero may be unsolvable
        fault_flags_ |= BCE_Fault::ZERO_UNSOLVABLE;
        mode_ = BCE_Mode::FAULT;
        solution_.solution_mode = static_cast<uint32_t>(BCE_Mode::FAULT);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
        return;
    }

    // Convert drop/windage to MOA holds
    float range = lrf_range_m_;
    float drop_moa = 0.0f;
    float wind_from_wind_moa = 0.0f;

    if (range > 0.0f) {
        // The drop relative to the zero'd sight line:
        // At zero range, the bullet hits the POA. At other ranges,
        // the drop from the sight line must be corrected.
        // Sight line at range R: -sight_height * (R / zero_range) + sight_height
        // Simplification: the zero angle already accounts for this.
        // The solver gives drop from bore line. We need drop from sight line.

        float sight_h = has_zero_ ? zero_.sight_height_mm * BCE_MM_TO_M : 0.0f;
        float zero_range_m = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : range;
        float sight_line_drop = sight_h - (sight_h / zero_range_m) * range;

        // Drop relative to sight line
        float relative_drop = result.drop_at_target_m - sight_line_drop;

        // Convert to angular adjustment in MOA
        drop_moa = -(relative_drop / range) * BCE_RAD_TO_MOA;
        wind_from_wind_moa = -(result.windage_at_target_m / range) * BCE_RAD_TO_MOA;
    }

    // Build directional windage components
    const float windage_earth_spin_moa = result.coriolis_wind_moa + result.spin_drift_moa;
    const float windage_offsets_moa = boresight_.horizontal_moa + reticle_.horizontal_moa;

    // Add corrections
    drop_moa += result.coriolis_elev_moa;
    float windage_moa = wind_from_wind_moa + windage_earth_spin_moa;

    // Add boresight and reticle offsets
    drop_moa += boresight_.vertical_moa + reticle_.vertical_moa;
    windage_moa += windage_offsets_moa;

    // Apply cant correction
    const float windage_before_cant_moa = windage_moa;
    float cant_elev, cant_wind;
    CantCorrection::apply(roll, drop_moa, cant_elev, cant_wind);
    drop_moa = cant_elev;
    windage_moa += cant_wind;
    const float windage_cant_moa = windage_moa - windage_before_cant_moa;

    // Populate firing solution
    solution_.solution_mode = static_cast<uint32_t>(BCE_Mode::SOLUTION_READY);
    solution_.fault_flags = fault_flags_;
    solution_.defaults_active = diag_flags_;

    solution_.hold_elevation_moa = drop_moa;
    solution_.hold_windage_moa = windage_moa;

    solution_.range_m = range;
    solution_.horizontal_range_m = result.horizontal_range_m;
    solution_.tof_ms = result.tof_s * 1000.0f;
    solution_.velocity_at_target_ms = result.velocity_at_target_ms;
    solution_.energy_at_target_j = result.energy_at_target_j;

    solution_.coriolis_windage_moa = result.coriolis_wind_moa;
    solution_.coriolis_elevation_moa = result.coriolis_elev_moa;
    solution_.spin_drift_moa = result.spin_drift_moa;
    solution_.wind_only_windage_moa = wind_from_wind_moa;
    solution_.earth_spin_windage_moa = windage_earth_spin_moa;
    solution_.offsets_windage_moa = windage_offsets_moa;
    solution_.cant_windage_moa = windage_cant_moa;

    solution_.cant_angle_deg = roll * BCE_RAD_TO_DEG;
    solution_.heading_deg_true = heading_true;
    solution_.air_density_kgm3 = atmo_.getAirDensity();

    computeUncertainty();
}

// ---------------------------------------------------------------------------
// Internal: recompute zero angle
// ---------------------------------------------------------------------------

void BCE_Engine::recomputeZero() {
    zero_dirty_ = false;

    if (!has_bullet_ || !has_zero_) {
        zero_angle_rad_ = 0.0f;
        return;
    }

    if (zero_.zero_range_m < 1.0f || zero_.zero_range_m > BCE_MAX_RANGE_M) {
        fault_flags_ |= BCE_Fault::ZERO_UNSOLVABLE;
        zero_angle_rad_ = 0.0f;
        return;
    }

    SolverParams params = buildSolverParams(zero_.zero_range_m);
    float angle = solver_.solveZeroAngle(params, zero_.zero_range_m);

    if (std::isnan(angle)) {
        fault_flags_ |= BCE_Fault::ZERO_UNSOLVABLE;
        zero_angle_rad_ = 0.0f;
    } else {
        zero_angle_rad_ = angle;
    }
}

// ---------------------------------------------------------------------------
// Internal: build solver parameters
// ---------------------------------------------------------------------------

SolverParams BCE_Engine::buildSolverParams(float range_m) const {
    SolverParams p;
    std::memset(&p, 0, sizeof(p));

    // BC with atmospheric correction
    p.bc = atmo_.correctBC(bullet_.bc);
    p.drag_model = bullet_.drag_model;

    // Muzzle velocity adjusted for barrel length (relative to SAAMI reference barrel).
    // If reference_barrel_length_in is unset (0), default to 24" (standard rifle barrel).
    float ref_barrel_in = (bullet_.reference_barrel_length_in > 0.0f)
                              ? bullet_.reference_barrel_length_in
                              : 24.0f;
    float base_mv_fps = bullet_.muzzle_velocity_ms * 3.28084f;
    float barrel_length_delta_in = bullet_.barrel_length_in - ref_barrel_in;
    float mv_adjustment_fps_per_in = std::fabs(bullet_.mv_adjustment_factor);
    float adjusted_mv_fps = base_mv_fps + (barrel_length_delta_in * mv_adjustment_fps_per_in);
    p.muzzle_velocity_ms = adjusted_mv_fps * 0.3048f;

    p.bullet_mass_kg = bullet_.mass_grains * BCE_GRAINS_TO_KG;
    p.sight_height_m = has_zero_ ? zero_.sight_height_mm * BCE_MM_TO_M : 0.0f;

    p.air_density = atmo_.getAirDensity();
    p.speed_of_sound = atmo_.getSpeedOfSound();
    p.drag_reference_scale = external_reference_mode_
        ? BCE_EXTERNAL_REFERENCE_DRAG_SCALE
        : BCE_DEFAULT_DRAG_REFERENCE_SCALE;
    p.target_range_m = range_m;
    p.launch_angle_rad = 0.0f; // set by caller

    // Wind decomposition
    float heading = mag_.computeHeading(ahrs_.getYaw());
    wind_.decompose(heading, p.headwind_ms, p.crosswind_ms);

    // Coriolis
    if (has_latitude_) {
        p.coriolis_enabled = true;
        p.coriolis_lat_rad = latitude_deg_ * BCE_DEG_TO_RAD;
        p.azimuth_rad = heading * BCE_DEG_TO_RAD;
    } else {
        p.coriolis_enabled = false;
    }

    // Spin drift
    if (std::fabs(bullet_.twist_rate_inches) > 0.1f) {
        p.spin_drift_enabled = true;
        p.twist_rate_inches = bullet_.twist_rate_inches;
        p.caliber_m = bullet_.caliber_inches * BCE_INCHES_TO_M;
    } else {
        p.spin_drift_enabled = false;
    }

    return p;
}

// ---------------------------------------------------------------------------
// Uncertainty / error propagation — SRS §14
// ---------------------------------------------------------------------------

void BCE_Engine::setUncertaintyConfig(const UncertaintyConfig* config) {
    if (config) {
        uncertainty_config_ = *config;
    }
}

void BCE_Engine::getDefaultUncertaintyConfig(UncertaintyConfig* out) {
    if (!out) return;
    out->enabled                = true;
    out->sigma_muzzle_velocity_ms = 1.5f;   // ~5 fps standard deviation
    out->sigma_bc_fraction      = 0.02f;    // 2 % BC uncertainty
    out->sigma_range_m          = 1.0f;     // 1 m range uncertainty
    out->sigma_wind_speed_ms    = 1.0f;     // 1 m/s wind speed
    out->sigma_wind_heading_deg = 10.0f;    // 10 ° wind direction
    out->sigma_temperature_c    = 2.0f;     // 2 °C temperature
    out->sigma_pressure_pa      = 150.0f;   // 150 Pa pressure (~4 mmHg)
    out->sigma_humidity         = 0.05f;    // 5 % relative humidity
    out->sigma_sight_height_mm  = 0.5f;     // 0.5 mm sight-height mounting error
    out->sigma_cant_deg         = 1.5f;     // 1.5 ° cant / roll — RM3100 magnetometer default
}

/**
 * @brief Gaussian error propagation via central finite differences.
 *
 * For each uncertain input X with 1-sigma value σ_X, perturbs X by ±σ_X,
 * re-evaluates the firing solution, and accumulates:
 *
 *   var_e  += ((elev_plus - elev_minus) / 2)²
 *   var_w  += ((wind_plus - wind_minus) / 2)²
 *   cov_ew += ((elev_plus - elev_minus) / 2) × ((wind_plus - wind_minus) / 2)
 *
 * This is the standard first-order propagation identity
 *   σ_y² = (∂y/∂x)² σ_x²
 * where ∂y/∂x ≈ Δy / (2σ_x), so σ_y² ≈ (Δy/2)² (the σ_x terms cancel).
 */
void BCE_Engine::computeUncertainty() {
    solution_.uncertainty_valid    = false;
    solution_.sigma_elevation_moa  = 0.0f;
    solution_.sigma_windage_moa    = 0.0f;
    solution_.covariance_elev_wind = 0.0f;

    if (!uncertainty_config_.enabled) return;

    // Must have a ready solution to propagate through
    if (solution_.solution_mode != static_cast<uint32_t>(BCE_Mode::SOLUTION_READY)) return;

    const float range = lrf_range_m_;
    if (range < 1.0f) return;

    const float roll         = ahrs_.getRoll();
    const float launch_angle = zero_angle_rad_ + ahrs_.getPitch();
    const float sight_h      = has_zero_ ? zero_.sight_height_mm * BCE_MM_TO_M : 0.0f;
    const float zero_r       = (has_zero_ && zero_.zero_range_m > 0.0f)
                                   ? zero_.zero_range_m : range;

    // Pre-build the nominal SolverParams once
    SolverParams base = buildSolverParams(range);
    base.launch_angle_rad = launch_angle;

    // Evaluate (elevation_moa, windage_moa) for arbitrary SolverParams + roll
    auto evalMOA = [&](const SolverParams& p, float roll_rad,
                       float& elev_moa, float& wind_moa) -> bool {
        SolverResult r = solver_.integrate(p);
        if (!r.valid) { elev_moa = 0.0f; wind_moa = 0.0f; return false; }

        float sl_drop    = sight_h - (sight_h / zero_r) * range;
        float rel_drop   = r.drop_at_target_m - sl_drop;

        float em = -(rel_drop / range)               * BCE_RAD_TO_MOA + r.coriolis_elev_moa;
        float wm = -(r.windage_at_target_m / range)  * BCE_RAD_TO_MOA
                   + r.coriolis_wind_moa + r.spin_drift_moa;

        float ce, cw;
        CantCorrection::apply(roll_rad, em, ce, cw);
        elev_moa = ce;
        wind_moa = wm + cw;
        return true;
    };

    float var_e  = 0.0f;
    float var_w  = 0.0f;
    float cov_ew = 0.0f;

    // Accumulate variance from a symmetric perturbation pair
    auto accumulate = [&](const SolverParams& pp, const SolverParams& pm) {
        float ep, wp, em, wm;
        if (!evalMOA(pp, roll, ep, wp)) return;
        if (!evalMOA(pm, roll, em, wm)) return;
        float de = (ep - em) * 0.5f;
        float dw = (wp - wm) * 0.5f;
        var_e  += de * de;
        var_w  += dw * dw;
        cov_ew += de * dw;
    };

    // 1. Muzzle velocity
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_muzzle_velocity_ms;
        pp.muzzle_velocity_ms += h;
        pm.muzzle_velocity_ms = std::fmax(1.0f, pm.muzzle_velocity_ms - h);
        accumulate(pp, pm);
    }

    // 2. Ballistic coefficient (as absolute BC units = fraction × base BC)
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_bc_fraction * base.bc;
        pp.bc += h;
        pm.bc = std::fmax(0.01f, pm.bc - h);
        accumulate(pp, pm);
    }

    // 3. Range (re-evaluate with different target distances + matching sight-line geometry)
    {
        float h   = uncertainty_config_.sigma_range_m;
        float rp  = range + h;
        float rm  = std::fmax(1.0f, range - h);

        SolverParams pp = buildSolverParams(rp);
        SolverParams pm = buildSolverParams(rm);
        pp.launch_angle_rad = launch_angle;
        pm.launch_angle_rad = launch_angle;

        auto evalRange = [&](const SolverParams& p, float rng,
                             float& elev_moa, float& wind_moa) -> bool {
            SolverResult r = solver_.integrate(p);
            if (!r.valid) { elev_moa = 0.0f; wind_moa = 0.0f; return false; }
            float zr      = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : rng;
            float sl_drop = sight_h - (sight_h / zr) * rng;
            float rel_drop = r.drop_at_target_m - sl_drop;
            float em = -(rel_drop / rng)              * BCE_RAD_TO_MOA + r.coriolis_elev_moa;
            float wm = -(r.windage_at_target_m / rng) * BCE_RAD_TO_MOA
                       + r.coriolis_wind_moa + r.spin_drift_moa;
            float ce, cw2;
            CantCorrection::apply(roll, em, ce, cw2);
            elev_moa = ce; wind_moa = wm + cw2;
            return true;
        };

        float ep, wp, em, wm;
        if (evalRange(pp, rp, ep, wp) && evalRange(pm, rm, em, wm)) {
            float de = (ep - em) * 0.5f;
            float dw = (wp - wm) * 0.5f;
            var_e  += de * de;
            var_w  += dw * dw;
            cov_ew += de * dw;
        }
    }

    // 4. Wind speed (scale existing headwind/crosswind proportionally)
    {
        SolverParams pp = base, pm = base;
        float h     = uncertainty_config_.sigma_wind_speed_ms;
        float total = std::sqrt(base.headwind_ms * base.headwind_ms
                                + base.crosswind_ms * base.crosswind_ms);
        if (total > 0.01f) {
            float scale_p = (total + h) / total;
            float scale_m = (total - h) / total;
            pp.headwind_ms  = base.headwind_ms  * scale_p;
            pp.crosswind_ms = base.crosswind_ms * scale_p;
            pm.headwind_ms  = base.headwind_ms  * scale_m;
            pm.crosswind_ms = base.crosswind_ms * scale_m;
        } else {
            // No current wind — apply a pure crosswind perturbation
            pp.crosswind_ms = h;
            pm.crosswind_ms = -h;
        }
        accumulate(pp, pm);
    }

    // 5. Wind heading (rotate wind vector by ±sigma degrees)
    {
        float h_rad = uncertainty_config_.sigma_wind_heading_deg * BCE_DEG_TO_RAD;
        float hw    = base.headwind_ms;
        float xw    = base.crosswind_ms;
        float total = std::sqrt(hw * hw + xw * xw);
        if (total > 0.01f) {
            float base_angle = std::atan2(xw, hw);
            SolverParams pp = base, pm = base;
            pp.headwind_ms  = total * std::cos(base_angle + h_rad);
            pp.crosswind_ms = total * std::sin(base_angle + h_rad);
            pm.headwind_ms  = total * std::cos(base_angle - h_rad);
            pm.crosswind_ms = total * std::sin(base_angle - h_rad);
            accumulate(pp, pm);
        }
        // Zero wind → heading uncertainty has no effect
    }

    // 6–8. Atmospheric parameters (temperature, pressure, humidity).
    // Use a temporary copy of the Atmosphere to avoid mutating engine state.
    {
        auto evalAtmo = [&](float dt, float dp, float dh,
                            float& elev_moa, float& wind_moa) -> bool {
            Atmosphere tmp = atmo_;
            tmp.updateFromBaro(atmo_.getPressure()    + dp,
                               atmo_.getTemperature() + dt,
                               atmo_.getHumidity()    + dh);
            SolverParams p   = base;
            p.bc             = tmp.correctBC(bullet_.bc);
            p.air_density    = tmp.getAirDensity();
            p.speed_of_sound = tmp.getSpeedOfSound();
            return evalMOA(p, roll, elev_moa, wind_moa);
        };

        // 6. Temperature
        {
            float h = uncertainty_config_.sigma_temperature_c;
            float ep, wp, em, wm;
            if (evalAtmo(+h, 0.0f, 0.0f, ep, wp) &&
                evalAtmo(-h, 0.0f, 0.0f, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de; var_w += dw * dw; cov_ew += de * dw;
            }
        }

        // 7. Pressure
        {
            float h = uncertainty_config_.sigma_pressure_pa;
            float ep, wp, em, wm;
            if (evalAtmo(0.0f, +h, 0.0f, ep, wp) &&
                evalAtmo(0.0f, -h, 0.0f, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de; var_w += dw * dw; cov_ew += de * dw;
            }
        }

        // 8. Humidity (clamped to [0, 1])
        {
            float cur = atmo_.getHumidity();
            float h   = uncertainty_config_.sigma_humidity;
            float hp  = std::fmin(cur + h, 1.0f) - cur;
            float hm  = cur - std::fmax(cur - h, 0.0f);
            float ep, wp, em, wm;
            if (evalAtmo(0.0f, 0.0f, +hp, ep, wp) &&
                evalAtmo(0.0f, 0.0f, -hm, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de; var_w += dw * dw; cov_ew += de * dw;
            }
        }
    }

    // 9. Sight height
    {
        SolverParams pp = base, pm = base;
        float h_m = uncertainty_config_.sigma_sight_height_mm * BCE_MM_TO_M;
        pp.sight_height_m += h_m;
        pm.sight_height_m = std::fmax(0.0f, pm.sight_height_m - h_m);
        accumulate(pp, pm);
    }

    // 10. Cant angle (perturb roll without changing trajectory calculation)
    {
        float h_rad = uncertainty_config_.sigma_cant_deg * BCE_DEG_TO_RAD;
        float ep, wp, em, wm;
        if (evalMOA(base, roll + h_rad, ep, wp) &&
            evalMOA(base, roll - h_rad, em, wm)) {
            float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
            var_e += de * de; var_w += dw * dw; cov_ew += de * dw;
        }
    }

    solution_.sigma_elevation_moa  = std::sqrt(var_e);
    solution_.sigma_windage_moa    = std::sqrt(var_w);
    solution_.covariance_elev_wind = cov_ew;
    solution_.uncertainty_valid    = true;
}
