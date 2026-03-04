/**
 * @file dope_engine.cpp
 * @brief DOPE engine implementation — the top-level orchestrator.
 *
 * Pipeline per DOPE_Update:
 *   1. Feed IMU/mag → AHRS
 *   2. Feed baro → Atmosphere
 *   3. Store LRF range + quaternion snapshot
 *   4. Evaluate state machine
 *   5. If data sufficient → run solver → apply corrections → populate FiringSolution
 */

#include "dope_engine.h"
#include "dope/dope_math_utils.h"
#include <cmath>
#include <cstring>

namespace {
constexpr float kDefaultPressureUncalibratedSigmaPa = 50.0f;
constexpr float kCep50ToSigma = 1.17741f; // CEP50 radius -> 1-sigma for 2D Gaussian

inline float rss2(float a, float b) {
    return std::sqrt((a * a) + (b * b));
}

float InterpolatePiecewiseSigma(const DOPE_ErrorTable& table, float x) {
    if (!table.points || table.count <= 0)
        return 0.0f;
    if (table.count == 1)
        return table.points[0].sigma;
    if (x <= table.points[0].x)
        return table.points[0].sigma;
    if (x >= table.points[table.count - 1].x)
        return table.points[table.count - 1].sigma;
    for (int i = 0; i < table.count - 1; ++i) {
        if (x <= table.points[i + 1].x) {
            const float dx = table.points[i + 1].x - table.points[i].x;
            if (dx <= 0.0f)
                return table.points[i + 1].sigma;
            const float t = (x - table.points[i].x) / dx;
            return table.points[i].sigma + t * (table.points[i + 1].sigma - table.points[i].sigma);
        }
    }
    return table.points[table.count - 1].sigma;
}

float InterpolateCEP(const DOPE_CEPTable& table, float range_m) {
    if (!table.points || table.count <= 0)
        return 0.0f;
    if (table.count == 1)
        return table.points[0].cep50_moa;
    if (range_m <= table.points[0].range_m)
        return table.points[0].cep50_moa;
    if (range_m >= table.points[table.count - 1].range_m)
        return table.points[table.count - 1].cep50_moa;
    for (int i = 0; i < table.count - 1; ++i) {
        if (range_m <= table.points[i + 1].range_m) {
            const float dx = table.points[i + 1].range_m - table.points[i].range_m;
            if (dx <= 0.0f)
                return table.points[i + 1].cep50_moa;
            const float t = (range_m - table.points[i].range_m) / dx;
            return table.points[i].cep50_moa +
                   t * (table.points[i + 1].cep50_moa - table.points[i].cep50_moa);
        }
    }
    return table.points[table.count - 1].cep50_moa;
}

struct BarrelMaterialProps {
    float density_kg_m3;        // bulk density
    float specific_heat_J_kgK;  // specific heat capacity
    float stiffness_scale;      // relative stiffness factor (1/E)
    float cte_scale;            // relative thermal expansion factor
};

BarrelMaterialProps getBarrelMaterialProps(BarrelMaterial material) {
    switch (material) {
    case BarrelMaterial::CMV:
        return {7850.0f, 460.0f, 0.95f, 0.75f}; // 205 GPa, ~12e-6/K
    case BarrelMaterial::STAINLESS_416:
        return {8000.0f, 500.0f, 1.0f, 1.0f};   // 193 GPa, ~17e-6/K baseline
    case BarrelMaterial::CARBON_WRAPPED:
        return {5200.0f, 720.0f, 1.30f, 0.25f}; // composite wrap + liner
    default:
        return {7850.0f, 460.0f, 1.0f, 1.0f};
    }
}
} // namespace

void DOPE_Engine::init() {
    ahrs_.init();
    mag_.init();
    atmo_.init();
    solver_.init();

    mode_ = DOPE_Mode::IDLE;
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
    latest_baro_temp_c_ = 0.0f;
    has_baro_temp_ = false;
    barrel_ambient_K_ = 293.15f;
    barrel_temp_K_ = barrel_ambient_K_;
    last_barrel_update_us_ = 0;
    last_shot_time_us_ = 0;
    shots_in_string_ = 0;
    fov_h_deg_ = 0.0f;
    fov_v_deg_ = 0.0f;

    solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::IDLE);
}

void DOPE_Engine::update(const SensorFrame* frame) {
    if (!frame)
        return;

    had_invalid_sensor_input_ = false;

    uint64_t now_us = frame->timestamp_us;
    integrateBarrelCooling(now_us);

    // --- 1. AHRS Update ---
    if (frame->imu_valid) {
        bool imu_finite = std::isfinite(frame->accel_x) && std::isfinite(frame->accel_y) &&
                          std::isfinite(frame->accel_z) && std::isfinite(frame->gyro_x) &&
                          std::isfinite(frame->gyro_y) && std::isfinite(frame->gyro_z);
        if (!imu_finite) {
            had_invalid_sensor_input_ = true;
        }

        float dt = 0.01f; // default 100 Hz
        if (!first_update_ && now_us > last_imu_timestamp_us_) {
            dt = static_cast<float>(now_us - last_imu_timestamp_us_) * 1e-6f;
            if (dt > 0.1f)
                dt = 0.1f; // cap at 100ms for safety
            if (dt < 0.0001f)
                dt = 0.0001f;
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
            ahrs_.update(frame->accel_x, frame->accel_y, frame->accel_z, frame->gyro_x,
                         frame->gyro_y, frame->gyro_z, mx, my, mz, use_mag, dt);
        }
    }

    // --- 2. Barometer → Atmosphere ---
    if (frame->baro_valid) {
        if (std::isfinite(frame->baro_temperature_c)) {
            latest_baro_temp_c_ = frame->baro_temperature_c;
            has_baro_temp_ = true;
            barrel_ambient_K_ = 273.15f + latest_baro_temp_c_;
        }
        float humidity = frame->baro_humidity_valid ? frame->baro_humidity : -1.0f;
        atmo_.updateFromBaro(frame->baro_pressure_pa, frame->baro_temperature_c, humidity);
        refreshDerivedSigmasFromProfiles();
        if (atmo_.consumeZeroRecomputeHint()) {
            zero_dirty_ = true;
        }
    }

    // --- 3. LRF Range ---
    if (frame->lrf_valid) {
        const float lrf_range_m = frame->lrf_range_m;
        const bool range_finite = std::isfinite(lrf_range_m);
        if (!range_finite) {
            had_invalid_sensor_input_ = true;
        }

        bool range_valid = range_finite && lrf_range_m > 0.0f &&
                           lrf_range_m <= static_cast<float>(DOPE_MAX_RANGE_M);

        if (range_valid) {
            if (!has_range_) {
                // First valid range, initialize filter
                lrf_range_filtered_m_ = lrf_range_m;
            } else {
                // Apply IIR filter    [MATH §14.3]
                lrf_range_filtered_m_ =
                    lrf_filter_alpha_ * lrf_range_m +
                    (1.0f - lrf_filter_alpha_) * lrf_range_filtered_m_; // [MATH §14.3]
            }
            lrf_range_m_ = lrf_range_m;
            lrf_timestamp_us_ = frame->lrf_timestamp_us;
            lrf_quaternion_ = ahrs_.getQuaternion();
            has_range_ = true;
            refreshDerivedSigmasFromProfiles();
        }
    }

    // --- 4. Zoom encoder → FOV computation — SRS §7.5    [MATH §14.4]
    if (frame->encoder_valid && frame->encoder_focal_length_mm > DOPE_ENCODER_MIN_FOCAL_LENGTH_MM) {
        float f = frame->encoder_focal_length_mm;
        fov_h_deg_ =
            2.0f * std::atan(DOPE_SENSOR_HALF_WIDTH_MM / f) * dope::math::RAD_TO_DEG; // [MATH §14.4]
        fov_v_deg_ =
            2.0f * std::atan(DOPE_SENSOR_HALF_HEIGHT_MM / f) * dope::math::RAD_TO_DEG; // [MATH §14.4]
    }

    // --- 5. Evaluate state and compute solution ---
    evaluateState(now_us);
}

void DOPE_Engine::setBulletProfile(const BulletProfile* profile) {
    if (!profile)
        return;
    bullet_ = *profile;
    if (bullet_.barrel_material != BarrelMaterial::CMV &&
        bullet_.barrel_material != BarrelMaterial::STAINLESS_416 &&
        bullet_.barrel_material != BarrelMaterial::CARBON_WRAPPED) {
        bullet_.barrel_material = BarrelMaterial::CMV;
    }
    has_bullet_ = true;
    zero_dirty_ = true;
}

void DOPE_Engine::setZeroConfig(const ZeroConfig* config) {
    if (!config)
        return;
    zero_ = *config;
    has_zero_ = true;
    zero_dirty_ = true;
}

void DOPE_Engine::setWindManual(float speed_ms, float heading_deg) {
    wind_.setWind(speed_ms, heading_deg);
}

void DOPE_Engine::setLatitude(float latitude_deg) {
    if (std::isnan(latitude_deg)) {
        has_latitude_ = false;
    } else {
        latitude_deg_ = latitude_deg;
        has_latitude_ = true;
    }
}

void DOPE_Engine::setDefaultOverrides(const DOPE_DefaultOverrides* defaults) {
    if (!defaults)
        return;
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

void DOPE_Engine::setIMUBias(const float accel_bias[3], const float gyro_bias[3]) {
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    ahrs_.setAccelBias(accel_bias ? accel_bias : zero);
    ahrs_.setGyroBias(gyro_bias ? gyro_bias : zero);
}

void DOPE_Engine::setMagCalibration(const float hard_iron[3], const float soft_iron[9]) {
    const float zero_hi[3] = {0.0f, 0.0f, 0.0f};
    const float identity_si[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    mag_.setCalibration(hard_iron ? hard_iron : zero_hi, soft_iron ? soft_iron : identity_si);
}

void DOPE_Engine::setBoresightOffset(float vertical_moa, float horizontal_moa) {
    boresight_.vertical_moa = vertical_moa;
    boresight_.horizontal_moa = horizontal_moa;
}

void DOPE_Engine::setReticleOffset(float vertical_moa, float horizontal_moa) {
    reticle_.vertical_moa = vertical_moa;
    reticle_.horizontal_moa = horizontal_moa;
}

void DOPE_Engine::calibrateBaro() {
    atmo_.calibrateBaro();
    zero_dirty_ = true;
}

void DOPE_Engine::calibrateGyro() {
    ahrs_.captureGyroBias(last_gyro_[0], last_gyro_[1], last_gyro_[2]);
}

void DOPE_Engine::setAHRSAlgorithm(AHRS_Algorithm algo) {
    ahrs_.setAlgorithm(algo);
}

void DOPE_Engine::setAHRSConfig(const DOPE_AHRSConfig* config) {
    if (config) {
        ahrs_.applyConfig(*config);
    }
}

void DOPE_Engine::setLRFConfig(const DOPE_LRFConfig* config) {
    if (config) {
        lrf_filter_alpha_       = config->filter_alpha;
        lrf_stale_threshold_us_ = config->stale_threshold_us;
    }
}

void DOPE_Engine::setMagDeclination(float declination_deg) {
    mag_.setDeclination(declination_deg);
}

void DOPE_Engine::setExternalReferenceMode(bool enabled) {
    external_reference_mode_ = enabled;
}

void DOPE_Engine::getSolution(FiringSolution* out) const {
    if (out) {
        *out = solution_;
    }
}

void DOPE_Engine::getRealtimeSolution(RealtimeSolution* out) const {
    if (!out)
        return;

    out->solution_mode = solution_.solution_mode;
    out->fault_flags = solution_.fault_flags;
    out->defaults_active = solution_.defaults_active;

    out->hold_elevation_moa = solution_.hold_elevation_moa;
    out->hold_windage_moa = solution_.hold_windage_moa;

    out->uncertainty_valid = solution_.uncertainty_valid;
    out->uncertainty_radius_moa = 0.0f;
    out->uncertainty_confidence = 0.682689f;
    if (solution_.uncertainty_valid) {
        out->uncertainty_radius_moa =
            std::sqrt(solution_.sigma_elevation_moa * solution_.sigma_elevation_moa +
                      solution_.sigma_windage_moa * solution_.sigma_windage_moa);
    }

    out->range_m = solution_.range_m;
    out->tof_ms = solution_.tof_ms;
    out->velocity_at_target_ms = solution_.velocity_at_target_ms;
}

// ---------------------------------------------------------------------------
// Internal: state machine evaluation
// ---------------------------------------------------------------------------

void DOPE_Engine::evaluateState(uint64_t now_us) {
    fault_flags_ = 0;
    diag_flags_ = atmo_.getDiagFlags();

    // Check hard faults — SRS §13
    if (!has_range_) {
        fault_flags_ |= DOPE_Fault::NO_RANGE;
    } else if (now_us >= lrf_timestamp_us_ && (now_us - lrf_timestamp_us_) > lrf_stale_threshold_us_) {
        // LRF stale — not a hard fault, but range is invalid
        has_range_ = false;
        fault_flags_ |= DOPE_Fault::NO_RANGE;
        diag_flags_ |= DOPE_Diag::LRF_STALE;
    }

    if (!has_bullet_) {
        fault_flags_ |= DOPE_Fault::NO_BULLET;
    } else {
        if (bullet_.muzzle_velocity_ms < 1.0f) {
            fault_flags_ |= DOPE_Fault::NO_MV;
        }
        if (bullet_.bc < 0.001f) {
            fault_flags_ |= DOPE_Fault::NO_BC;
        }

        if (has_zero_ && (zero_.zero_range_m < 1.0f ||
                          zero_.zero_range_m > static_cast<float>(DOPE_MAX_RANGE_M))) {
            fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
        }
    }

    if (!ahrs_.isStable()) {
        fault_flags_ |= DOPE_Fault::AHRS_UNSTABLE;
    }

    if (!has_latitude_) {
        diag_flags_ |= DOPE_Diag::CORIOLIS_DISABLED;
    }

    if (mag_.isDisturbed()) {
        diag_flags_ |= DOPE_Diag::MAG_SUPPRESSED;
    }

    if (!wind_.isSet()) {
        diag_flags_ |= DOPE_Diag::DEFAULT_WIND;
    }

    if (atmo_.hadInvalidInput() || had_invalid_sensor_input_) {
        fault_flags_ |= DOPE_Fault::SENSOR_INVALID;
    }

    // Determine mode
    if (fault_flags_ != 0) {
        // Check which faults are actually hard faults
        uint32_t hard_faults =
            fault_flags_ &
            (DOPE_Fault::NO_RANGE | DOPE_Fault::NO_BULLET | DOPE_Fault::NO_MV | DOPE_Fault::NO_BC |
             DOPE_Fault::AHRS_UNSTABLE | DOPE_Fault::ZERO_UNSOLVABLE);
        if (hard_faults != 0) {
            mode_ = DOPE_Mode::FAULT;
            solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::FAULT);
            solution_.fault_flags = fault_flags_;
            solution_.defaults_active = diag_flags_;
            return;
        }
    }

    // Have enough data — check if we can compute
    if (has_range_ && has_bullet_ && bullet_.muzzle_velocity_ms > 1.0f && bullet_.bc > 0.001f) {
        computeSolution();
        mode_ = DOPE_Mode::SOLUTION_READY;
    } else {
        mode_ = DOPE_Mode::IDLE;
        solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::IDLE);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
    }
}

// ---------------------------------------------------------------------------
// Internal: compute firing solution
// ---------------------------------------------------------------------------

void DOPE_Engine::computeSolution() {
    // Recompute zero if dirty
    if (zero_dirty_) {
        recomputeZero();
    }

    if (fault_flags_ & DOPE_Fault::ZERO_UNSOLVABLE) {
        mode_ = DOPE_Mode::FAULT;
        solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::FAULT);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
        return;
    }

    // Get current pitch (bore elevation) and heading from AHRS
    float pitch = ahrs_.getPitch();
    float roll = ahrs_.getRoll();
    float yaw = ahrs_.getYaw();
    float heading_true = mag_.computeHeading(yaw);

    // Build solver params
    SolverParams params = buildSolverParams(lrf_range_filtered_m_);
    params.launch_angle_rad = zero_angle_rad_ + pitch;

    // Run solver
    SolverResult result = solver_.integrate(params);

    if (!result.valid) {
        // Zero may be unsolvable
        fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
        mode_ = DOPE_Mode::FAULT;
        solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::FAULT);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
        return;
    }

    // Convert drop/windage to MOA holds
    // Use the filtered range consistently for both the solver inputs and hold math to avoid
    // mixing a smoothed trajectory with an unfiltered denominator at very short ranges.
    float range = lrf_range_filtered_m_;
    float drop_moa = 0.0f;
    float wind_from_wind_moa = 0.0f;

    if (range > 0.0f) {
        // The drop relative to the zero'd sight line:
        // At zero range, the bullet hits the POA. At other ranges,
        // the drop from the sight line must be corrected.
        // Sight line at range R: -sight_height * (R / zero_range) + sight_height
        // Simplification: the zero angle already accounts for this.
        // The solver gives drop from bore line. We need drop from sight line.

        float sight_h = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;
        float zero_range_m = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : range;
        float sight_line_drop = sight_h - (sight_h / zero_range_m) * range; // [MATH §14.1]

        // Drop relative to sight line
        float relative_drop = result.drop_at_target_m - sight_line_drop; // [MATH §14.1]

        // Convert to angular adjustment in MOA    [MATH §14.2]
        drop_moa = -(relative_drop / range) * dope::math::RAD_TO_MOA;                        // [MATH §14.2]
        wind_from_wind_moa = -(result.windage_at_target_m / range) * dope::math::RAD_TO_MOA; // [MATH §14.2]
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

    // Apply cant correction    [MATH §15]
    const float windage_before_cant_moa = windage_moa;
    float cant_elev, cant_wind;
    CantCorrection::apply(roll, drop_moa, cant_elev, cant_wind); // [MATH §15]
    drop_moa = cant_elev;
    windage_moa += cant_wind;
    const float windage_cant_moa = windage_moa - windage_before_cant_moa;

    // Populate firing solution
    solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY);
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

    solution_.cant_angle_deg = roll * dope::math::RAD_TO_DEG;
    solution_.heading_deg_true = heading_true;
    solution_.air_density_kgm3 = atmo_.getAirDensity();

    computeUncertainty();
}

float DOPE_Engine::estimateBarrelMassKg() const {
    float length_m = (bullet_.barrel_length_in > 0.0f)
                         ? bullet_.barrel_length_in * dope::math::INCHES_TO_M
                         : 0.6f; // fallback ~24"
    const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
    float od_m = (bullet_.muzzle_diameter_in > 0.0f)
                     ? bullet_.muzzle_diameter_in * dope::math::INCHES_TO_M
                     : 0.01778f; // ~0.7"
    // Cylindrical approximation; conservatively clamp.
    const float area = 0.25f * dope::math::PI * od_m * od_m;
    float volume = area * length_m;
    if (!std::isfinite(volume) || volume <= 0.0f)
        volume = 0.00008f; // ~0.08 L fallback
    float mass = volume * props.density_kg_m3; // material density
    if (!std::isfinite(mass) || mass <= 0.0f)
        mass = 1.5f;
    return std::fmax(0.5f, std::fmin(mass, 4.0f));
}

void DOPE_Engine::integrateBarrelCooling(uint64_t now_us) {
    if (now_us == 0)
        return;
    if (last_barrel_update_us_ == 0) {
        last_barrel_update_us_ = now_us;
        barrel_temp_K_ = barrel_ambient_K_;
        return;
    }
    if (now_us <= last_barrel_update_us_)
        return;

    const float dt_s = static_cast<float>(now_us - last_barrel_update_us_) * 1e-6f;
    // Cooling time constant scales with muzzle diameter (thin barrels shed heat faster).
    const float od_in = (bullet_.muzzle_diameter_in > 0.0f) ? bullet_.muzzle_diameter_in : 0.7f;
    const float od_lo = 0.55f, od_hi = 1.0f;
    const float tau_lo = 70.0f, tau_hi = 120.0f; // seconds
    float tau_s = tau_hi;
    if (od_in <= od_lo) {
        tau_s = tau_lo;
    } else if (od_in >= od_hi) {
        tau_s = tau_hi;
    } else {
        const float t = (od_in - od_lo) / (od_hi - od_lo);
        tau_s = tau_lo + t * (tau_hi - tau_lo);
    }

    const float decay = std::exp(-dt_s / std::fmax(tau_s, 1.0f));
    barrel_temp_K_ = barrel_ambient_K_ + (barrel_temp_K_ - barrel_ambient_K_) * decay;
    last_barrel_update_us_ = now_us;
}

float DOPE_Engine::barrelHeatMultiplier() const {
    const float delta_K = barrel_temp_K_ - barrel_ambient_K_;
    if (!(delta_K > 0.0f))
        return 1.0f;
    const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
    const float alpha_base = 0.01f; // mild growth per K at stainless baseline
    const float alpha = alpha_base * props.cte_scale;
    const float mult = std::sqrt(1.0f + alpha * delta_K);
    return std::fmin(std::fmax(mult, 1.0f), 1.6f); // cap to avoid runaway
}

void DOPE_Engine::notifyShotFired(uint64_t timestamp_us, float ambient_temp_c) {
    // Update ambient if provided.
    if (std::isfinite(ambient_temp_c)) {
        barrel_ambient_K_ = 273.15f + ambient_temp_c;
    }

    // Integrate cooling up to the shot timestamp.
    integrateBarrelCooling(timestamp_us);

    // Reset string counter if the cadence relaxed.
    if (last_shot_time_us_ > 0) {
        const uint64_t delta_us = (timestamp_us > last_shot_time_us_) ? (timestamp_us - last_shot_time_us_) : 0;
        if (delta_us > 120000000ULL) { // >120 s break
            shots_in_string_ = 0;
        }
    }

    // Energy coupled into barrel approximated from muzzle energy with a coupling factor.
    const float bullet_mass_kg = (std::isfinite(bullet_.mass_grains) && bullet_.mass_grains > 0.0f)
                                     ? bullet_.mass_grains * dope::math::GRAINS_TO_KG
                                     : 0.0097f; // ~150 gr fallback
    const float mv = std::fmax(1.0f, bullet_.muzzle_velocity_ms);
    const float muzzle_energy_J = 0.5f * bullet_mass_kg * mv * mv;
    const float energy_to_barrel_J = std::fmin(std::fmax(muzzle_energy_J * 0.2f, 400.0f), 2000.0f);

    const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
    const float heat_capacity = estimateBarrelMassKg() * props.specific_heat_J_kgK;
    if (heat_capacity > 0.0f) {
        const float delta_K = energy_to_barrel_J / heat_capacity;
        if (std::isfinite(delta_K) && delta_K > 0.0f) {
            barrel_temp_K_ += delta_K;
        }
    }

    last_shot_time_us_ = timestamp_us;
    last_barrel_update_us_ = timestamp_us;
    ++shots_in_string_;
}

// ---------------------------------------------------------------------------
// Internal: recompute zero angle
// ---------------------------------------------------------------------------

void DOPE_Engine::recomputeZero() {
    zero_dirty_ = false;

    if (!has_bullet_ || !has_zero_) {
        zero_angle_rad_ = 0.0f;
        return;
    }

    if (zero_.zero_range_m < 1.0f || zero_.zero_range_m > DOPE_MAX_RANGE_M) {
        fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
        zero_angle_rad_ = 0.0f;
        return;
    }

    SolverParams params = buildSolverParams(zero_.zero_range_m);
    float angle = solver_.solveZeroAngle(params, zero_.zero_range_m);

    if (std::isnan(angle)) {
        fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
        zero_angle_rad_ = 0.0f;
    } else {
        zero_angle_rad_ = angle;
    }
}

// ---------------------------------------------------------------------------
// Internal: build solver parameters
// ---------------------------------------------------------------------------

SolverParams DOPE_Engine::buildSolverParams(float range_m) const {
    SolverParams p;
    std::memset(&p, 0, sizeof(p));

    // BC with atmospheric correction
    p.bc = atmo_.correctBC(bullet_.bc);
    p.drag_model = bullet_.drag_model;

    // Muzzle velocity adjusted for barrel length (relative to SAAMI reference barrel).
    // If reference_barrel_length_in is unset (0), default to 24" (standard rifle barrel).
    float ref_barrel_in =
        (bullet_.reference_barrel_length_in > 0.0f) ? bullet_.reference_barrel_length_in : 24.0f;
    float base_mv_fps = bullet_.muzzle_velocity_ms * 3.28084f;               // [MATH §9]
    float barrel_length_delta_in = bullet_.barrel_length_in - ref_barrel_in; // [MATH §9]
    float mv_adjustment_fps_per_in = std::fabs(bullet_.mv_adjustment_factor);
    float adjusted_mv_fps =
        base_mv_fps + (barrel_length_delta_in * mv_adjustment_fps_per_in); // [MATH §9]
    p.muzzle_velocity_ms = adjusted_mv_fps * 0.3048f;                      // [MATH §9]

    p.bullet_mass_kg = bullet_.mass_grains * dope::math::GRAINS_TO_KG;
    p.bullet_length_m = bullet_.length_mm * dope::math::MM_TO_M;
    p.sight_height_m = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;

    p.air_density = atmo_.getAirDensity();
    p.speed_of_sound = atmo_.getSpeedOfSound();
    p.drag_reference_scale = external_reference_mode_ ? DOPE_EXTERNAL_REFERENCE_DRAG_SCALE
                                                      : DOPE_DEFAULT_DRAG_REFERENCE_SCALE;
    p.target_range_m = range_m;
    p.launch_angle_rad = 0.0f; // set by caller

    // Wind decomposition    [MATH §10]
    float heading = mag_.computeHeading(ahrs_.getYaw());
    wind_.decompose(heading, p.headwind_ms, p.crosswind_ms); // [MATH §10]

    // Coriolis
    if (has_latitude_) {
        p.coriolis_enabled = true;
        p.coriolis_lat_rad = latitude_deg_ * dope::math::DEG_TO_RAD;
        p.azimuth_rad = heading * dope::math::DEG_TO_RAD;
    } else {
        p.coriolis_enabled = false;
    }

    // Spin drift
    if (std::fabs(bullet_.twist_rate_inches) > 0.1f) {
        p.spin_drift_enabled = true;
        p.twist_rate_inches = bullet_.twist_rate_inches;
        p.caliber_m = bullet_.caliber_inches * dope::math::INCHES_TO_M;
    } else {
        p.spin_drift_enabled = false;
    }

    return p;
}

// ---------------------------------------------------------------------------
// Uncertainty / error propagation — SRS §14
// ---------------------------------------------------------------------------

void DOPE_Engine::setUncertaintyConfig(const UncertaintyConfig* config) {
    if (config) {
        uncertainty_config_ = *config;
        refreshDerivedSigmasFromProfiles();
    }
}

void DOPE_Engine::getDefaultUncertaintyConfig(UncertaintyConfig* out) {
    if (!out)
        return;
    out->enabled = true;
    out->sigma_muzzle_velocity_ms = 1.5f;       // ~5 fps standard deviation
    out->sigma_bc_fraction = 0.02f;             // 2 % BC uncertainty
    out->sigma_range_m = 1.0f;                  // 1 m range uncertainty
    out->sigma_wind_speed_ms = 0.44704f;        // 1 mph wind speed (0.44704 m/s)
    out->sigma_wind_heading_deg = 2.0f;         // 2 ° wind direction
    out->sigma_temperature_c = 1.111f;          // 2 °F temperature
    out->sigma_pressure_pa = 5.0f;              // 5 Pa pressure
    out->sigma_humidity = 0.05f;                // 5 % relative humidity
    out->sigma_sight_height_mm = 0.075f;        // updated default sight-height mounting error
    out->sigma_cant_deg = 1.5f;                 // 1.5 ° cant / roll — RM3100 magnetometer default
    out->sigma_latitude_deg = 0.0f;             // latitude error (disabled by default)
    out->sigma_mass_grains = 0.5f;              // ~0.5 gr lot/measurement spread
    out->sigma_length_mm = 0.1f;                // ~0.1 mm OAL measurement spread
    out->sigma_caliber_inches = 0.001f;         // ~0.001" diameter spread
    out->sigma_twist_rate_inches = 0.1f;        // 1% of default 10"/turn twist rate
    out->sigma_zero_range_m = 0.13716f;         // ~0.15 yd (0.137 m) zero-range setup error
    out->sigma_mv_adjustment_fps_per_in = 1.0f; // ~1 fps/in barrel adjustment uncertainty
    out->use_range_error_table = false;
    out->range_error_table = {nullptr, 0};
    out->use_temperature_error_table = false;
    out->temperature_error_table = {nullptr, 0};
    out->use_pressure_delta_temp_error_table = false;
    out->pressure_delta_temp_error_table = {nullptr, 0};
    out->pressure_uncalibrated_sigma_pa = kDefaultPressureUncalibratedSigmaPa;
    out->pressure_is_calibrated = false;
    out->pressure_has_calibration_temp = false;
    out->pressure_calibration_temp_c = 0.0f;
    out->use_cartridge_cep_table = false;
    out->cartridge_cep_table = {nullptr, 0};
    out->cartridge_cep_scale_floor = 1.0f;
}

void DOPE_Engine::refreshDerivedSigmasFromProfiles() {
    // Enforce a minimum MV sigma derived from barrel finish (fps → m/s) when present.
    if (has_bullet_ && std::isfinite(bullet_.barrel_finish_sigma_mv_fps) &&
        bullet_.barrel_finish_sigma_mv_fps > 0.0f) {
        const float finish_sigma_ms = bullet_.barrel_finish_sigma_mv_fps * 0.3048f;
        if (finish_sigma_ms > uncertainty_config_.sigma_muzzle_velocity_ms) {
            uncertainty_config_.sigma_muzzle_velocity_ms = finish_sigma_ms;
        }
    }

    if (uncertainty_config_.use_range_error_table &&
        uncertainty_config_.range_error_table.points != nullptr &&
        uncertainty_config_.range_error_table.count > 0 && has_range_ &&
        std::isfinite(lrf_range_m_)) {
        uncertainty_config_.sigma_range_m =
            InterpolatePiecewiseSigma(uncertainty_config_.range_error_table, lrf_range_m_);
    }

    if (uncertainty_config_.use_temperature_error_table &&
        uncertainty_config_.temperature_error_table.points != nullptr &&
        uncertainty_config_.temperature_error_table.count > 0 && has_baro_temp_ &&
        std::isfinite(latest_baro_temp_c_)) {
        uncertainty_config_.sigma_temperature_c = InterpolatePiecewiseSigma(
            uncertainty_config_.temperature_error_table, latest_baro_temp_c_);
    }

    if (!uncertainty_config_.use_pressure_delta_temp_error_table ||
        uncertainty_config_.pressure_delta_temp_error_table.points == nullptr ||
        uncertainty_config_.pressure_delta_temp_error_table.count <= 0) {
        return;
    }

    const float uncalibrated_sigma =
        (std::isfinite(uncertainty_config_.pressure_uncalibrated_sigma_pa) &&
         uncertainty_config_.pressure_uncalibrated_sigma_pa >= 0.0f)
            ? uncertainty_config_.pressure_uncalibrated_sigma_pa
            : kDefaultPressureUncalibratedSigmaPa;

    if (!uncertainty_config_.pressure_is_calibrated) {
        uncertainty_config_.sigma_pressure_pa = uncalibrated_sigma;
        return;
    }

    // Engine-level enforcement: calibrated pressure profile requires calibration temperature
    // metadata.
    if (!uncertainty_config_.pressure_has_calibration_temp ||
        !std::isfinite(uncertainty_config_.pressure_calibration_temp_c)) {
        uncertainty_config_.pressure_is_calibrated = false;
        uncertainty_config_.pressure_has_calibration_temp = false;
        uncertainty_config_.sigma_pressure_pa = uncalibrated_sigma;
        return;
    }

    if (!has_baro_temp_ || !std::isfinite(latest_baro_temp_c_)) {
        uncertainty_config_.sigma_pressure_pa = uncalibrated_sigma;
        return;
    }

    const float delta_temp_c =
        std::fabs(latest_baro_temp_c_ - uncertainty_config_.pressure_calibration_temp_c);
    uncertainty_config_.sigma_pressure_pa = InterpolatePiecewiseSigma(
        uncertainty_config_.pressure_delta_temp_error_table, delta_temp_c);
}

/**
 * @brief Gaussian error propagation via central finite differences.    [MATH §16]
 *
 * For each uncertain input X with 1-sigma value σ_X, perturbs X by ±σ_X,
 * re-evaluates the firing solution, and accumulates:
 *
 *   var_e  += ((elev_plus - elev_minus) / 2)²    [MATH §16.1]
 *   var_w  += ((wind_plus - wind_minus) / 2)²    [MATH §16.1]
 *   cov_ew += ((elev_plus - elev_minus) / 2) × ((wind_plus - wind_minus) / 2)   [MATH §16.1]
 *
 * This is the standard first-order propagation identity
 *   σ_y² = (∂y/∂x)² σ_x²
 * where ∂y/∂x ≈ Δy / (2σ_x), so σ_y² ≈ (Δy/2)² (the σ_x terms cancel).
 */
void DOPE_Engine::computeUncertainty() {
    solution_.uncertainty_valid = false;
    solution_.sigma_elevation_moa = 0.0f;
    solution_.sigma_windage_moa = 0.0f;
    solution_.covariance_elev_wind = 0.0f;

    if (!uncertainty_config_.enabled)
        return;

    // Must have a ready solution to propagate through
    if (solution_.solution_mode != static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY))
        return;

    const float range = lrf_range_m_;
    if (range < 1.0f)
        return;

    const float roll = ahrs_.getRoll();
    const float launch_angle = zero_angle_rad_ + ahrs_.getPitch();
    const float sight_h = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;
    const float zero_r = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : range;
    const float pitch = ahrs_.getPitch();

    // Pre-build the nominal SolverParams once
    SolverParams base = buildSolverParams(range);
    base.launch_angle_rad = launch_angle;

    // Evaluate (elevation_moa, windage_moa) for arbitrary SolverParams + roll
    auto evalMOAWithZeroRange = [&](const SolverParams& p, float roll_rad, float zero_range_m,
                                    float& elev_moa, float& wind_moa) -> bool {
        SolverResult r = solver_.integrate(p);
        if (!r.valid) {
            elev_moa = 0.0f;
            wind_moa = 0.0f;
            return false;
        }

        float safe_zero_r = std::fmax(1.0f, zero_range_m);
        float sl_drop = sight_h - (sight_h / safe_zero_r) * range;
        float rel_drop = r.drop_at_target_m - sl_drop;

        float em = -(rel_drop / range) * dope::math::RAD_TO_MOA + r.coriolis_elev_moa;
        float wm = -(r.windage_at_target_m / range) * dope::math::RAD_TO_MOA + r.coriolis_wind_moa +
                   r.spin_drift_moa;

        float ce, cw;
        CantCorrection::apply(roll_rad, em, ce, cw);
        elev_moa = ce;
        wind_moa = wm + cw;
        return true;
    };

    auto evalMOA = [&](const SolverParams& p, float roll_rad, float& elev_moa,
                       float& wind_moa) -> bool {
        return evalMOAWithZeroRange(p, roll_rad, zero_r, elev_moa, wind_moa);
    };

    float var_e = 0.0f;
    float var_w = 0.0f;
    float cov_ew = 0.0f;
    int input_idx = 0; // tracks which input we're accumulating

    // Zero the per-input breakdown arrays
    for (int i = 0; i < FiringSolution::kNumUncertaintyInputs; ++i) {
        solution_.uc_var_elev[i] = 0.0f;
        solution_.uc_var_wind[i] = 0.0f;
    }

    // Accumulate variance from a symmetric perturbation pair.
    // Records per-input contribution in uc_var_elev/uc_var_wind[input_idx].
    auto accumulate = [&](const SolverParams& pp, const SolverParams& pm) {
        float ep, wp, em, wm;
        if (!evalMOA(pp, roll, ep, wp))
            return;
        if (!evalMOA(pm, roll, em, wm))
            return;
        float de = (ep - em) * 0.5f;
        float dw = (wp - wm) * 0.5f;
        var_e += de * de;
        var_w += dw * dw;
        cov_ew += de * dw;
        if (input_idx < FiringSolution::kNumUncertaintyInputs) {
            solution_.uc_var_elev[input_idx] += de * de;
            solution_.uc_var_wind[input_idx] += dw * dw;
        }
    };

    // 0. Muzzle velocity
    input_idx = 0;
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_muzzle_velocity_ms;
        pp.muzzle_velocity_ms += h;
        pm.muzzle_velocity_ms = std::fmax(1.0f, pm.muzzle_velocity_ms - h);
        accumulate(pp, pm);
    }

    // 1. Ballistic coefficient (as absolute BC units = fraction × base BC)
    input_idx = 1;
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_bc_fraction * base.bc;
        pp.bc += h;
        pm.bc = std::fmax(0.01f, pm.bc - h);
        accumulate(pp, pm);
    }

    // 2. Range (re-evaluate with different target distances + matching sight-line geometry)
    input_idx = 2;
    {
        float h = uncertainty_config_.sigma_range_m;
        float rp = range + h;
        float rm = std::fmax(1.0f, range - h);

        SolverParams pp = buildSolverParams(rp);
        SolverParams pm = buildSolverParams(rm);
        pp.launch_angle_rad = launch_angle;
        pm.launch_angle_rad = launch_angle;

        auto evalRange = [&](const SolverParams& p, float rng, float& elev_moa,
                             float& wind_moa) -> bool {
            SolverResult r = solver_.integrate(p);
            if (!r.valid) {
                elev_moa = 0.0f;
                wind_moa = 0.0f;
                return false;
            }
            float zr = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : rng;
            float sl_drop = sight_h - (sight_h / zr) * rng;
            float rel_drop = r.drop_at_target_m - sl_drop;
            float em = -(rel_drop / rng) * dope::math::RAD_TO_MOA + r.coriolis_elev_moa;
            float wm = -(r.windage_at_target_m / rng) * dope::math::RAD_TO_MOA + r.coriolis_wind_moa +
                       r.spin_drift_moa;
            float ce, cw2;
            CantCorrection::apply(roll, em, ce, cw2);
            elev_moa = ce;
            wind_moa = wm + cw2;
            return true;
        };

        float ep, wp, em, wm;
        if (evalRange(pp, rp, ep, wp) && evalRange(pm, rm, em, wm)) {
            float de = (ep - em) * 0.5f;
            float dw = (wp - wm) * 0.5f;
            var_e += de * de;
            var_w += dw * dw;
            cov_ew += de * dw;
            solution_.uc_var_elev[2] = de * de;
            solution_.uc_var_wind[2] = dw * dw;
        }
    }

    // 3. Wind speed (scale existing headwind/crosswind proportionally)
    input_idx = 3;
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_wind_speed_ms;
        float total =
            std::sqrt(base.headwind_ms * base.headwind_ms + base.crosswind_ms * base.crosswind_ms);
        if (total > 0.01f) {
            float scale_p = (total + h) / total;
            float scale_m = (total - h) / total;
            pp.headwind_ms = base.headwind_ms * scale_p;
            pp.crosswind_ms = base.crosswind_ms * scale_p;
            pm.headwind_ms = base.headwind_ms * scale_m;
            pm.crosswind_ms = base.crosswind_ms * scale_m;
        } else {
            // No current wind — apply a pure crosswind perturbation
            pp.crosswind_ms = h;
            pm.crosswind_ms = -h;
        }
        accumulate(pp, pm);
    }

    // 4. Wind heading (rotate wind vector by ±sigma degrees)
    input_idx = 4;
    {
        float h_rad = uncertainty_config_.sigma_wind_heading_deg * dope::math::DEG_TO_RAD;
        float hw = base.headwind_ms;
        float xw = base.crosswind_ms;
        float total = std::sqrt(hw * hw + xw * xw);
        if (total > 0.01f) {
            float base_angle = std::atan2(xw, hw);
            SolverParams pp = base, pm = base;
            pp.headwind_ms = total * std::cos(base_angle + h_rad);
            pp.crosswind_ms = total * std::sin(base_angle + h_rad);
            pm.headwind_ms = total * std::cos(base_angle - h_rad);
            pm.crosswind_ms = total * std::sin(base_angle - h_rad);
            accumulate(pp, pm);
        }
        // Zero wind → heading uncertainty has no effect
    }

    // 5–7. Atmospheric parameters (temperature, pressure, humidity).
    // Use a temporary copy of the Atmosphere to avoid mutating engine state.
    {
        auto evalAtmo = [&](float dt, float dp, float dh, float& elev_moa,
                            float& wind_moa) -> bool {
            Atmosphere tmp = atmo_;
            tmp.updateFromBaro(atmo_.getPressure() + dp, atmo_.getTemperature() + dt,
                               atmo_.getHumidity() + dh);
            SolverParams p = base;
            p.bc = tmp.correctBC(bullet_.bc);
            p.air_density = tmp.getAirDensity();
            p.speed_of_sound = tmp.getSpeedOfSound();
            return evalMOA(p, roll, elev_moa, wind_moa);
        };

        // 5. Temperature
        input_idx = 5;
        {
            float h = uncertainty_config_.sigma_temperature_c;
            float ep, wp, em, wm;
            if (evalAtmo(+h, 0.0f, 0.0f, ep, wp) && evalAtmo(-h, 0.0f, 0.0f, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de;
                var_w += dw * dw;
                cov_ew += de * dw;
                solution_.uc_var_elev[5] = de * de;
                solution_.uc_var_wind[5] = dw * dw;
            }
        }

        // 6. Pressure
        input_idx = 6;
        {
            float h = uncertainty_config_.sigma_pressure_pa;
            float ep, wp, em, wm;
            if (evalAtmo(0.0f, +h, 0.0f, ep, wp) && evalAtmo(0.0f, -h, 0.0f, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de;
                var_w += dw * dw;
                cov_ew += de * dw;
                solution_.uc_var_elev[6] = de * de;
                solution_.uc_var_wind[6] = dw * dw;
            }
        }

        // 7. Humidity (clamped to [0, 1])
        input_idx = 7;
        {
            float cur = atmo_.getHumidity();
            float h = uncertainty_config_.sigma_humidity;
            float hp = std::fmin(cur + h, 1.0f) - cur;
            float hm = cur - std::fmax(cur - h, 0.0f);
            float ep, wp, em, wm;
            if (evalAtmo(0.0f, 0.0f, +hp, ep, wp) && evalAtmo(0.0f, 0.0f, -hm, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de;
                var_w += dw * dw;
                cov_ew += de * dw;
                solution_.uc_var_elev[7] = de * de;
                solution_.uc_var_wind[7] = dw * dw;
            }
        }
    }

    // 8. Sight height
    input_idx = 8;
    {
        SolverParams pp = base, pm = base;
        float h_m = uncertainty_config_.sigma_sight_height_mm * dope::math::MM_TO_M;
        pp.sight_height_m += h_m;
        pm.sight_height_m = std::fmax(0.0f, pm.sight_height_m - h_m);
        accumulate(pp, pm);
    }

    // 9. Cant angle (perturb roll without changing trajectory calculation)
    input_idx = 9;
    {
        float h_rad = uncertainty_config_.sigma_cant_deg * dope::math::DEG_TO_RAD;
        float ep, wp, em, wm;
        if (evalMOA(base, roll + h_rad, ep, wp) && evalMOA(base, roll - h_rad, em, wm)) {
            float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
            var_e += de * de;
            var_w += dw * dw;
            cov_ew += de * dw;
            solution_.uc_var_elev[9] = de * de;
            solution_.uc_var_wind[9] = dw * dw;
        }
    }

    // 10. Latitude (Coriolis effect) — only meaningful when Coriolis is active
    input_idx = 10;
    if (has_latitude_ && uncertainty_config_.sigma_latitude_deg > 0.0f) {
        float h_rad = uncertainty_config_.sigma_latitude_deg * dope::math::DEG_TO_RAD;
        SolverParams pp = base, pm = base;
        pp.coriolis_lat_rad = base.coriolis_lat_rad + h_rad;
        pm.coriolis_lat_rad = base.coriolis_lat_rad - h_rad;
        float ep, wp, em, wm;
        if (evalMOA(pp, roll, ep, wp) && evalMOA(pm, roll, em, wm)) {
            float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
            var_e += de * de;
            var_w += dw * dw;
            cov_ew += de * dw;
            solution_.uc_var_elev[10] = de * de;
            solution_.uc_var_wind[10] = dw * dw;
        }
    }

    // 11. Bullet mass
    input_idx = 11;
    {
        SolverParams pp = base, pm = base;
        float h_kg = uncertainty_config_.sigma_mass_grains * dope::math::GRAINS_TO_KG;
        pp.bullet_mass_kg += h_kg;
        pm.bullet_mass_kg = std::fmax(1e-6f, pm.bullet_mass_kg - h_kg);
        accumulate(pp, pm);
    }

    // 12. Bullet length (affects dynamic SG / spin drift term)
    input_idx = 12;
    {
        SolverParams pp = base, pm = base;
        float h_m = uncertainty_config_.sigma_length_mm * dope::math::MM_TO_M;
        pp.bullet_length_m = std::fmax(0.0f, pp.bullet_length_m + h_m);
        pm.bullet_length_m = std::fmax(0.0f, pm.bullet_length_m - h_m);
        accumulate(pp, pm);
    }

    // 13. Bullet caliber (affects spin drift term)
    input_idx = 13;
    {
        SolverParams pp = base, pm = base;
        float h_m = uncertainty_config_.sigma_caliber_inches * dope::math::INCHES_TO_M;
        pp.caliber_m = std::fmax(0.0f, pp.caliber_m + h_m);
        pm.caliber_m = std::fmax(0.0f, pm.caliber_m - h_m);
        accumulate(pp, pm);
    }

    // 14. Twist rate
    input_idx = 14;
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_twist_rate_inches;
        float tp = base.twist_rate_inches + h;
        float tm = base.twist_rate_inches - h;

        if (std::fabs(tp) > 0.1f) {
            pp.spin_drift_enabled = true;
            pp.twist_rate_inches = tp;
        } else {
            pp.spin_drift_enabled = false;
            pp.twist_rate_inches = 0.0f;
        }

        if (std::fabs(tm) > 0.1f) {
            pm.spin_drift_enabled = true;
            pm.twist_rate_inches = tm;
        } else {
            pm.spin_drift_enabled = false;
            pm.twist_rate_inches = 0.0f;
        }

        accumulate(pp, pm);
    }

    // 15. Zero range (perturb solved launch angle and sight-line geometry)
    input_idx = 15;
    if (has_zero_ && zero_.zero_range_m > 1.0f && uncertainty_config_.sigma_zero_range_m > 0.0f) {
        float h = uncertainty_config_.sigma_zero_range_m;
        float zp = std::fmin(static_cast<float>(DOPE_MAX_RANGE_M), zero_.zero_range_m + h);
        float zm = std::fmax(1.0f, zero_.zero_range_m - h);

        SolverParams pz = buildSolverParams(zp);
        SolverParams mz = buildSolverParams(zm);
        float launch_p = solver_.solveZeroAngle(pz, zp);
        float launch_m = solver_.solveZeroAngle(mz, zm);

        if (!std::isnan(launch_p) && !std::isnan(launch_m)) {
            SolverParams pp = base, pm = base;
            pp.launch_angle_rad = launch_p + pitch;
            pm.launch_angle_rad = launch_m + pitch;

            float ep, wp, em, wm;
            if (evalMOAWithZeroRange(pp, roll, zp, ep, wp) &&
                evalMOAWithZeroRange(pm, roll, zm, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de;
                var_w += dw * dw;
                cov_ew += de * dw;
                solution_.uc_var_elev[15] = de * de;
                solution_.uc_var_wind[15] = dw * dw;
            }
        }
    }

    // 16. MV adjustment factor (fps/in)
    input_idx = 16;
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_mv_adjustment_fps_per_in;

        float ref_barrel_in = (bullet_.reference_barrel_length_in > 0.0f)
                                  ? bullet_.reference_barrel_length_in
                                  : 24.0f;
        float base_mv_fps = bullet_.muzzle_velocity_ms * 3.28084f;
        float barrel_delta_in = bullet_.barrel_length_in - ref_barrel_in;
        float adj_base = std::fabs(bullet_.mv_adjustment_factor);
        float adj_p = std::fmax(0.0f, adj_base + h);
        float adj_m = std::fmax(0.0f, adj_base - h);

        pp.muzzle_velocity_ms = std::fmax(1.0f, (base_mv_fps + barrel_delta_in * adj_p) * 0.3048f);
        pm.muzzle_velocity_ms = std::fmax(1.0f, (base_mv_fps + barrel_delta_in * adj_m) * 0.3048f);
        accumulate(pp, pm);
    }

    // Optional cartridge dispersion scaling (CEP50 table). Converts CEP50 radius to 1-sigma
    // radial and scales all variances to match while preserving axis ratios.
    // Barrel stiffness + accuracy hierarchy
    bool has_explicit_accuracy = false;
    {
        float radial_moa = 0.0f;
        float vertical_extra_moa = 0.0f;
        has_explicit_accuracy =
            (std::isfinite(bullet_.measured_cep50_moa) && bullet_.measured_cep50_moa > 0.0f) ||
            (std::isfinite(bullet_.manufacturer_spec_moa) && bullet_.manufacturer_spec_moa > 0.0f) ||
            (uncertainty_config_.use_cartridge_cep_table &&
             uncertainty_config_.cartridge_cep_table.points != nullptr &&
             uncertainty_config_.cartridge_cep_table.count > 0);

        // Base radial from stiffness (applies to both axes as a floor)
        if (std::isfinite(bullet_.stiffness_moa) && bullet_.stiffness_moa > 0.0f) {
            radial_moa = bullet_.stiffness_moa;
        }

        // Accuracy hierarchy: measured CEP -> manufacturer -> category defaults
        float cep_source_moa = 0.0f;
        if (std::isfinite(bullet_.measured_cep50_moa) && bullet_.measured_cep50_moa > 0.0f) {
            cep_source_moa = bullet_.measured_cep50_moa;
        } else if (std::isfinite(bullet_.manufacturer_spec_moa) &&
                   bullet_.manufacturer_spec_moa > 0.0f) {
            cep_source_moa = bullet_.manufacturer_spec_moa;
        }

        if (cep_source_moa > 0.0f) {
            // Split 80/20 into radial vs vertical components at the reference test barrel.
            const float radial_part = cep_source_moa * 0.8f;
            const float vertical_part = cep_source_moa * 0.2f;
            radial_moa = rss2(radial_moa, radial_part);
            vertical_extra_moa = vertical_part;
        } else {
            // Fallback to category estimates if provided (already axis-specific).
            if (std::isfinite(bullet_.category_radial_moa) && bullet_.category_radial_moa > 0.0f) {
                radial_moa = rss2(radial_moa, bullet_.category_radial_moa);
            }
            if (std::isfinite(bullet_.category_vertical_moa) &&
                bullet_.category_vertical_moa > 0.0f) {
                vertical_extra_moa = rss2(vertical_extra_moa, bullet_.category_vertical_moa);
            }
        }

        // Stock contact penalty when not free-floated.
        if (!bullet_.free_floated) {
            radial_moa = rss2(radial_moa, 0.75f);
        }

        // Suppressor attachment: scale barrel-side dispersion based on muzzle diameter.
        if (bullet_.suppressor_attached && radial_moa > 0.0f) {
            const float od_in = (bullet_.muzzle_diameter_in > 0.0f) ? bullet_.muzzle_diameter_in : 0.7f;
            const float od_lo = 0.55f, od_hi = 1.0f;
            const float scale_lo = 1.25f, scale_hi = 1.05f;
            float scale = scale_hi;
            if (od_in <= od_lo) {
                scale = scale_lo;
            } else if (od_in >= od_hi) {
                scale = scale_hi;
            } else {
                const float t = (od_in - od_lo) / (od_hi - od_lo);
                scale = scale_lo + t * (scale_hi - scale_lo);
            }
            radial_moa *= scale;
        }

        // Barrel tuner reduces barrel-side dispersion modestly.
        if (bullet_.barrel_tuner_attached && radial_moa > 0.0f) {
            radial_moa *= 0.85f;
        }

        // Barrel heat/stringing multiplier (applies only to barrel-side radial term).
        if (radial_moa > 0.0f) {
            radial_moa *= barrelHeatMultiplier();
        }

        // Add radial to both axes; add vertical_extra to elevation only.
        if (radial_moa > 0.0f) {
            const float r2 = radial_moa * radial_moa;
            var_w += r2;
            var_e += r2;
        }
        if (vertical_extra_moa > 0.0f) {
            const float v2 = vertical_extra_moa * vertical_extra_moa;
            var_e += v2;
        }
    }

    float cep_scale = 1.0f;
    const float base_sigma_radius = std::sqrt(std::fmax(0.0f, var_e + var_w));
    if (uncertainty_config_.use_cartridge_cep_table && base_sigma_radius > 0.0f &&
        uncertainty_config_.cartridge_cep_table.points != nullptr &&
        uncertainty_config_.cartridge_cep_table.count > 0) {
        const float cep_moa = InterpolateCEP(uncertainty_config_.cartridge_cep_table, range);
        if (std::isfinite(cep_moa) && cep_moa > 0.0f) {
            const float sigma_target = cep_moa / kCep50ToSigma;
            if (std::isfinite(sigma_target) && sigma_target > 0.0f) {
                cep_scale = sigma_target / base_sigma_radius;
                const bool should_floor =
                    !(has_explicit_accuracy && cep_scale < 1.0f);
                if (should_floor && uncertainty_config_.cartridge_cep_scale_floor > 0.0f) {
                    cep_scale = std::fmax(cep_scale, uncertainty_config_.cartridge_cep_scale_floor);
                }
            }
        }
    }

    if (cep_scale != 1.0f && cep_scale > 0.0f) {
        const float s2 = cep_scale * cep_scale;
        var_e *= s2;
        var_w *= s2;
        cov_ew *= s2;
        for (int i = 0; i < FiringSolution::kNumUncertaintyInputs; ++i) {
            solution_.uc_var_elev[i] *= s2;
            solution_.uc_var_wind[i] *= s2;
        }
    }

    solution_.sigma_elevation_moa = std::sqrt(var_e); // [MATH §16.2]
    solution_.sigma_windage_moa = std::sqrt(var_w);   // [MATH §16.2]
    solution_.covariance_elev_wind = cov_ew;          // [MATH §16.2]
    solution_.uncertainty_valid = true;
}
