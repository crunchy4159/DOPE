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
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdio>

namespace {
constexpr float kDefaultPressureUncalibratedSigmaPa = 50.0f;
constexpr float kCep50ToSigma = 1.17741f; // CEP50 radius -> 1-sigma for 2D Gaussian
constexpr float kMinMvMs = 1.0f;

inline float clampf(float v, float lo, float hi) {
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

inline float rss2(float a, float b) {
    return std::sqrt((a * a) + (b * b));
}

inline int clampCount(int count, int max_count) {
    if (count < 0)
        return 0;
    if (count > max_count)
        return max_count;
    return count;
}

inline float fallbackMvAdjustmentFpsPerIn(float baseline_mv_ms) {
    const float fps = baseline_mv_ms * dope::math::MPS_TO_FPS;
    if (fps < 650.0f)
        return 12.0f; // pistol / PCC regime
    if (fps < 915.0f)
        return 20.0f; // intermediate carbine regime
    if (fps < 1067.0f)
        return 28.0f; // rifle carbine regime
    return 38.0f;     // full-power rifle regime
}

inline float effectiveMvAdjustmentFpsPerIn(float baseline_mv_ms, float explicit_fps_per_in) {
    if (std::isfinite(explicit_fps_per_in) && explicit_fps_per_in > 0.0f) {
        return std::fabs(explicit_fps_per_in);
    }
    if (std::isfinite(baseline_mv_ms) && baseline_mv_ms > 0.0f) {
        return fallbackMvAdjustmentFpsPerIn(baseline_mv_ms);
    }
    return 0.0f;
}

inline float interpolateBarrelMvMs(const DOPE_BarrelMVPoint* points, int count, float barrel_length_in) {
    if (!points || count <= 0 || !std::isfinite(barrel_length_in)) {
        return 0.0f;
    }
    if (count == 1) {
        return points[0].muzzle_velocity_ms;
    }
    if (barrel_length_in <= points[0].barrel_length_in) {
        return points[0].muzzle_velocity_ms;
    }
    if (barrel_length_in >= points[count - 1].barrel_length_in) {
        return points[count - 1].muzzle_velocity_ms;
    }
    for (int i = 0; i < count - 1; ++i) {
        if (barrel_length_in <= points[i + 1].barrel_length_in) {
            const float dx = points[i + 1].barrel_length_in - points[i].barrel_length_in;
            if (dx <= 0.0f) {
                return points[i + 1].muzzle_velocity_ms;
            }
            const float t = (barrel_length_in - points[i].barrel_length_in) / dx;
            return points[i].muzzle_velocity_ms +
                   t * (points[i + 1].muzzle_velocity_ms - points[i].muzzle_velocity_ms);
        }
    }
    return points[count - 1].muzzle_velocity_ms;
}

inline float localBarrelMvSlopeFpsPerIn(const DOPE_BarrelMVPoint* points, int count,
                                        float barrel_length_in) {
    if (!points || count < 2 || !std::isfinite(barrel_length_in)) {
        return NAN;
    }
    int seg = 0;
    if (barrel_length_in <= points[0].barrel_length_in) {
        seg = 0;
    } else if (barrel_length_in >= points[count - 1].barrel_length_in) {
        seg = count - 2;
    } else {
        for (int i = 0; i < count - 1; ++i) {
            if (barrel_length_in <= points[i + 1].barrel_length_in) {
                seg = i;
                break;
            }
        }
    }
    const float dx = points[seg + 1].barrel_length_in - points[seg].barrel_length_in;
    if (!(dx > 0.0f)) {
        return NAN;
    }
    const float dv_fps =
        (points[seg + 1].muzzle_velocity_ms - points[seg].muzzle_velocity_ms) * dope::math::MPS_TO_FPS;
    return std::fabs(dv_fps / dx);
}

inline float estimateDynamicStabilitySGApprox(float mass_grains, float caliber_in, float length_mm,
                                              float twist_in, float velocity_ms, float air_density) {
    float sg = 1.5f;
    if (!(mass_grains > 10.0f) || !(caliber_in > 0.05f) || !(length_mm > 1.0f) ||
        !(std::fabs(twist_in) > 0.1f) || !(velocity_ms > 50.0f)) {
        return sg;
    }
    const float length_in = length_mm * dope::math::MM_TO_M / dope::math::INCHES_TO_M;
    const float length_cal = length_in / caliber_in;
    const float twist_cal = std::fabs(twist_in) / caliber_in;
    const float vel_fps = velocity_ms * 3.28084f;
    const float density_ratio = (air_density > 1e-6f) ? (DOPE_STD_AIR_DENSITY / air_density) : 1.0f;
    const float denom = (twist_cal * twist_cal) * (caliber_in * caliber_in * caliber_in) *
                        length_cal * (1.0f + length_cal * length_cal);
    if (denom > 1e-6f) {
        const float sg_raw =
            (30.0f * mass_grains / denom) * std::cbrt(vel_fps / 2800.0f) * density_ratio;
        if (std::isfinite(sg_raw) && sg_raw > 0.0f) {
            sg = sg_raw;
        }
    }
    if (sg < 0.5f) sg = 0.5f;
    if (sg > 3.0f) sg = 3.0f;
    return sg;
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
    std::memset(&dataset_v2_, 0, sizeof(dataset_v2_));
    std::memset(&ballistic_context_, 0, sizeof(ballistic_context_));
    std::memset(&calibration_profile_, 0, sizeof(calibration_profile_));
    std::memset(&zero_, 0, sizeof(zero_));
    std::memset(&overrides_, 0, sizeof(overrides_));

    // Provide a conservative default gun profile so uncertainty propagation
    // has a sensible baseline even when the application layer doesn't call
    // DOPE_SetGunProfile. Tests and legacy callers expect a small nonzero
    // mechanical dispersion floor.
    bullet_ = {};
    bullet_.barrel_length_in = 24.0f;
    bullet_.reference_barrel_length_in = 24.0f;
    bullet_.muzzle_diameter_in = 0.7f; // ~0.7" default
    bullet_.barrel_material = BarrelMaterial::CMV;
    bullet_.stiffness_moa = 0.6f; // conservative mechanical dispersion baseline
    // Ensure `angular_sigma_moa` has a sensible default for compatibility.
    bullet_.angular_sigma_moa = bullet_.stiffness_moa;
    bullet_.free_floated = true;
    bullet_.suppressor_attached = false;
    bullet_.barrel_tuner_attached = false;
    bullet_.heat_efficiency_scalar = 1.0f;
    // Do not mark the profile as provided by the application; keep
    // `has_bullet_` false so callers that rely on the NO_BULLET fault
    // still observe its absence when no profile is explicitly set.
    has_bullet_ = false;
    has_dataset_v2_ = false;
    has_ballistic_context_ = false;
    has_calibration_profile_ = false;
    has_zero_ = false;
    has_range_ = false;
    has_latitude_ = false;
    has_overrides_ = false;

    zero_angle_rad_ = 0.0f;
    zero_dirty_ = true;

    lrf_range_m_ = 0.0f;
    target_elevation_m_ = 0.0f;
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
    // Use defaults from class member initialization.

    getDefaultUncertaintyConfig(&uncertainty_config_);
    latest_baro_temp_c_ = 0.0f;
    has_baro_temp_ = false;
    barrel_ambient_K_ = 293.15f;
    barrel_temp_K_ = barrel_ambient_K_;
    chamber_temp_K_ = barrel_ambient_K_;
    last_barrel_update_us_ = 0;
    last_chamber_update_us_ = 0;
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

    // --- Shot events & direct temperature sensors (Section 5) ---
    if (frame->shot_fired) {
        uint64_t shot_ts = frame->shot_timestamp_us ? frame->shot_timestamp_us : now_us;
        float ambient_temp = frame->shot_ambient_temp_c;
        if (!std::isfinite(ambient_temp))
            ambient_temp = latest_baro_temp_c_;
        notifyShotFired(shot_ts, ambient_temp);
    }

    if (frame->barrel_temp_valid && std::isfinite(frame->barrel_temperature_c)) {
        barrel_temp_K_ = frame->barrel_temperature_c + 273.15f;
        last_barrel_update_us_ = now_us;
        zero_dirty_ = true;
    }

    if (frame->chamber_temp_valid && std::isfinite(frame->chamber_temperature_c)) {
        chamber_temp_K_ = frame->chamber_temperature_c + 273.15f;
        last_chamber_update_us_ = now_us;
        zero_dirty_ = true;
    }

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
            float target_elevation_m = 0.0f;
            if (frame->target_elevation_valid ||
                (std::isfinite(frame->target_elevation_m) && std::fabs(frame->target_elevation_m) > 0.0f)) {
                if (std::isfinite(frame->target_elevation_m)) {
                    target_elevation_m = frame->target_elevation_m;
                } else {
                    had_invalid_sensor_input_ = true;
                }
            }
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
            target_elevation_m_ = target_elevation_m;
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

    // Sanitize newly added thermal/geometric fields to safe defaults.
    if (!std::isfinite(bullet_.chamber_time_s) || bullet_.chamber_time_s < 0.0f)
        bullet_.chamber_time_s = 0.0f;
    if (!std::isfinite(bullet_.barrel_profile.wall_thickness_mm) ||
        bullet_.barrel_profile.wall_thickness_mm < 0.0f)
        bullet_.barrel_profile.wall_thickness_mm = 0.0f;
    if (!std::isfinite(bullet_.barrel_profile.taper) || bullet_.barrel_profile.taper < 0.0f)
        bullet_.barrel_profile.taper = 0.0f;
}

void DOPE_Engine::setAmmoDatasetV2(const AmmoDatasetV2* dataset) {
    if (!dataset) {
        std::memset(&dataset_v2_, 0, sizeof(dataset_v2_));
        has_dataset_v2_ = false;
        return;
    }
    // Normalize and validate supported drag-model information before accepting.
    AmmoDatasetV2 ds = *dataset;
    // Backward-compatibility: if explicit supported-models mask is absent but
    // a single `drag_model` is provided, interpret that as the sole supported
    // model.
    if (ds.supported_drag_models_mask == 0 && ds.drag_model != static_cast<DragModel>(0)) {
        const uint8_t bit = static_cast<uint8_t>(1u << (static_cast<uint8_t>(ds.drag_model) - 1));
        ds.supported_drag_models_mask = bit;
    }

    // If both `drag_model` and an explicit mask are provided but the mask does
    // not include the selected drag_model, reject the dataset as invalid.
    if (ds.drag_model != static_cast<DragModel>(0) && ds.supported_drag_models_mask != 0) {
        const uint8_t bit = static_cast<uint8_t>(1u << (static_cast<uint8_t>(ds.drag_model) - 1));
        if ((ds.supported_drag_models_mask & bit) == 0) {
            fault_flags_ |= DOPE_Fault::INVALID_AMMO;
            return;
        }
    }

    // Defensive bounds hardening for table-first inputs.
    ds.num_barrel_mv_points = clampCount(ds.num_barrel_mv_points, DOPE_MAX_BARREL_MV_POINTS);
    {
        int w = 0;
        for (int i = 0; i < ds.num_barrel_mv_points; ++i) {
            const DOPE_BarrelMVPoint p = ds.barrel_mv_by_length_in[i];
            if (!std::isfinite(p.barrel_length_in) || !std::isfinite(p.muzzle_velocity_ms) ||
                p.barrel_length_in <= 0.0f || p.muzzle_velocity_ms <= 0.0f) {
                continue;
            }
            ds.barrel_mv_by_length_in[w++] = p;
        }
        ds.num_barrel_mv_points = w;
        if (ds.num_barrel_mv_points > 1) {
            std::sort(ds.barrel_mv_by_length_in,
                      ds.barrel_mv_by_length_in + ds.num_barrel_mv_points,
                      [](const DOPE_BarrelMVPoint& a, const DOPE_BarrelMVPoint& b) {
                          return a.barrel_length_in < b.barrel_length_in;
                      });
            // De-duplicate identical barrel lengths (keep last as most recent calibration).
            int unique_count = 0;
            for (int i = 0; i < ds.num_barrel_mv_points; ++i) {
                if (unique_count > 0 &&
                    std::fabs(ds.barrel_mv_by_length_in[i].barrel_length_in -
                              ds.barrel_mv_by_length_in[unique_count - 1].barrel_length_in) < 1e-4f) {
                    ds.barrel_mv_by_length_in[unique_count - 1] = ds.barrel_mv_by_length_in[i];
                } else {
                    ds.barrel_mv_by_length_in[unique_count++] = ds.barrel_mv_by_length_in[i];
                }
            }
            ds.num_barrel_mv_points = unique_count;
        }
    }

    ds.num_trajectories = clampCount(ds.num_trajectories, DOPE_MAX_TRAJECTORY_FAMILIES);
    for (int i = 0; i < ds.num_trajectories; ++i) {
        ds.trajectories[i].num_points =
            clampCount(ds.trajectories[i].num_points, DOPE_MAX_TABLE_POINTS);
        ds.trajectories[i].cached_table_num_points =
            clampCount(ds.trajectories[i].cached_table_num_points, DOPE_MAX_TABLE_POINTS);
        if (!std::isfinite(ds.trajectories[i].cached_table_step_m) ||
            ds.trajectories[i].cached_table_step_m <= 0.0f ||
            ds.trajectories[i].cached_table_num_points < 2) {
            ds.trajectories[i].cached_table_present = false;
            ds.trajectories[i].cached_table_step_m = 0.0f;
            ds.trajectories[i].cached_table_num_points = 0;
        }
    }
    ds.num_velocity_points = clampCount(ds.num_velocity_points, DOPE_MAX_TABLE_POINTS);
    ds.num_wind_drift_points = clampCount(ds.num_wind_drift_points, DOPE_MAX_TABLE_POINTS);
    ds.num_energy_points = clampCount(ds.num_energy_points, DOPE_MAX_TABLE_POINTS);
    ds.num_cep50_points = clampCount(ds.num_cep50_points, DOPE_MAX_TABLE_POINTS);
    ds.num_tof_points = clampCount(ds.num_tof_points, DOPE_MAX_TABLE_POINTS);
    ds.num_d_drop_dbc_points = clampCount(ds.num_d_drop_dbc_points, DOPE_MAX_TABLE_POINTS);
    ds.num_uncertainty_points = clampCount(ds.num_uncertainty_points, DOPE_MAX_TABLE_POINTS);
    ds.cached_full_table_num_points =
        clampCount(ds.cached_full_table_num_points, DOPE_MAX_TABLE_POINTS);
    if (!std::isfinite(ds.cached_full_table_step_m) || ds.cached_full_table_step_m <= 0.0f ||
        ds.cached_full_table_num_points < 2) {
        ds.cached_full_table_present = false;
        ds.cached_full_table_step_m = 0.0f;
        ds.cached_full_table_num_points = 0;
    }

    dataset_v2_ = ds;
    has_dataset_v2_ = true;
    refreshDerivedSigmasFromProfiles();
}

void DOPE_Engine::setBallisticContext(const BallisticContext* context) {
    if (!context) {
        std::memset(&ballistic_context_, 0, sizeof(ballistic_context_));
        has_ballistic_context_ = false;
        return;
    }
    ballistic_context_ = *context;
    has_ballistic_context_ = true;
    refreshDerivedSigmasFromProfiles();
}

void DOPE_Engine::setRifleAmmoCalibrationProfile(const RifleAmmoCalibrationProfile* profile) {
    if (!profile) {
        std::memset(&calibration_profile_, 0, sizeof(calibration_profile_));
        has_calibration_profile_ = false;
        return;
    }
    calibration_profile_ = *profile;
    has_calibration_profile_ = true;
    refreshDerivedSigmasFromProfiles();
}

void DOPE_Engine::setModuleCapabilities(const ModuleCapabilities* caps) {
    if (!caps)
        return;
    module_caps_ = *caps;
}

// Centralized selection of the active ammo source. For full migration we
// prefer `AmmoDatasetV2` and treat legacy `BulletProfile` as a deprecated
// fallback (ignored unless migration policy changes). This function returns
// a compact view of the authoritative base parameters used by solver paths.
DOPE_Engine::ActiveAmmo DOPE_Engine::selectActiveAmmo() const {
    ActiveAmmo a;
    if (has_dataset_v2_) {
        a.valid = true;
        a.is_v2 = true;
        a.bc = dataset_v2_.bc;
        a.muzzle_velocity_ms = dataset_v2_.muzzle_velocity_ms;
        a.mass_grains = dataset_v2_.mass_grains;
        a.length_mm = dataset_v2_.length_mm;
        a.caliber_inches = dataset_v2_.caliber_inches;
        a.twist_rate_inches = dataset_v2_.twist_rate_inches;
        a.mv_adjustment_fps_per_in = std::fabs(dataset_v2_.mv_adjustment_fps_per_in);
        a.baseline_barrel_length_in = (dataset_v2_.baseline_barrel_length_in > 0.0f)
                                           ? dataset_v2_.baseline_barrel_length_in
                                           : a.baseline_barrel_length_in;
        a.num_barrel_mv_points =
            clampCount(dataset_v2_.num_barrel_mv_points, DOPE_MAX_BARREL_MV_POINTS);
        for (int i = 0; i < a.num_barrel_mv_points; ++i) {
            a.barrel_mv_by_length_in[i] = dataset_v2_.barrel_mv_by_length_in[i];
        }
        a.drag_model = dataset_v2_.drag_model;
        a.supported_drag_models_mask = dataset_v2_.supported_drag_models_mask;
        return a;
    }

    // Legacy `BulletProfile` no longer contains sparse ballistic params.
    // Under the V2 migration we prefer dataset_v2_. If no dataset is present
    // return an invalid ActiveAmmo so callers fall back to conservative defaults.
    return a;
}

void DOPE_Engine::recordShotObservation(const ShotObservation* obs) {
    if (!obs || !obs->valid || !module_caps_.enable_shot_learning) {
        return;
    }
    if (!has_calibration_profile_) {
        std::memset(&calibration_profile_, 0, sizeof(calibration_profile_));
        calibration_profile_.muzzle_velocity_scale = 1.0f;
        calibration_profile_.uncertainty_scale = 1.0f;
        has_calibration_profile_ = true;
    }
    constexpr float kAlpha = 0.15f;
    calibration_profile_.drop_bias_moa =
        (1.0f - kAlpha) * calibration_profile_.drop_bias_moa + kAlpha * obs->impact_vertical_moa;
    calibration_profile_.wind_bias_moa =
        (1.0f - kAlpha) * calibration_profile_.wind_bias_moa + kAlpha * obs->impact_horizontal_moa;
    calibration_profile_.updated_at_unix_s = obs->timestamp_us / 1000000ULL;
    calibration_profile_.revision += 1u;
}

// Radar assist functionality removed — radar was a scrapped feature.

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

bool DOPE_Engine::getTrajectoryPoint(int range_m, TrajectoryPoint* out) const {
    if (!out)
        return false;
    if (mode_ != DOPE_Mode::SOLUTION_READY)
        return false;
    if (range_m < 0 || range_m > DOPE_MAX_RANGE_M)
        return false;

    // Table-first path: synthesize a trajectory point from the active dataset.
    if (has_dataset_v2_ && dataset_v2_.num_trajectories > 0) {
        const float horizontal_range_m = static_cast<float>(range_m);
        float drop_m = NAN;
        if (dataset_v2_.cached_full_table_present && dataset_v2_.cached_full_table_num_points > 1 &&
            std::isfinite(dataset_v2_.cached_full_table_step_m) &&
            dataset_v2_.cached_full_table_step_m > 0.0f) {
            const float step = dataset_v2_.cached_full_table_step_m;
            const int n = clampCount(dataset_v2_.cached_full_table_num_points, DOPE_MAX_TABLE_POINTS);
            int idx = static_cast<int>(horizontal_range_m / step);
            idx = (idx < 0) ? 0 : (idx >= n - 1) ? (n - 2) : idx;
            const float t = clampf((horizontal_range_m - idx * step) / step, 0.0f, 1.0f);
            drop_m = dataset_v2_.cached_full_trajectory[idx].drop_m * (1.0f - t) +
                     dataset_v2_.cached_full_trajectory[idx + 1].drop_m * t;
        } else {
            const float active_zero = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : 100.0f;
            int best_idx = 0;
            float best_err = std::fabs(dataset_v2_.trajectories[0].zero_range_m - active_zero);
            for (int i = 1; i < dataset_v2_.num_trajectories; ++i) {
                const float err = std::fabs(dataset_v2_.trajectories[i].zero_range_m - active_zero);
                if (err < best_err) {
                    best_err = err;
                    best_idx = i;
                }
            }
            const DOPE_TrajectoryFamily& fam = dataset_v2_.trajectories[best_idx];
            if (fam.num_points > 0) {
                DOPE_ProfilePoint traj[DOPE_MAX_TABLE_POINTS];
                const int traj_count =
                    (fam.num_points > DOPE_MAX_TABLE_POINTS) ? DOPE_MAX_TABLE_POINTS : fam.num_points;
                for (int i = 0; i < traj_count; ++i) {
                    traj[i].range_m = fam.points[i].range_m;
                    traj[i].value = fam.points[i].drop_m;
                }
                drop_m = interpolateProfileValue(traj, traj_count, horizontal_range_m);
            }
        }

        if (std::isfinite(drop_m)) {
            float mv_ratio = 1.0f; // runtime MV / baseline MV
            const ActiveAmmo active = selectActiveAmmo();
            if (active.valid && active.muzzle_velocity_ms > 0.0f) {
                const float ref_barrel_in = (active.baseline_barrel_length_in > 0.0f)
                                                ? active.baseline_barrel_length_in
                                                : 24.0f;
                const float barrel_in = (bullet_.barrel_length_in > 0.0f) ? bullet_.barrel_length_in : ref_barrel_in;
                float runtime_mv_ms = active.muzzle_velocity_ms;
                if (active.num_barrel_mv_points > 0) {
                    runtime_mv_ms = interpolateBarrelMvMs(active.barrel_mv_by_length_in,
                                                          active.num_barrel_mv_points, barrel_in);
                } else {
                    const float base_mv_fps = active.muzzle_velocity_ms * 3.28084f;
                    const float mv_adjust_fps_per_in =
                        effectiveMvAdjustmentFpsPerIn(active.muzzle_velocity_ms,
                                                      active.mv_adjustment_fps_per_in);
                    const float adjusted_mv_fps =
                        base_mv_fps + ((barrel_in - ref_barrel_in) * mv_adjust_fps_per_in);
                    runtime_mv_ms = adjusted_mv_fps * dope::math::FPS_TO_MPS;
                }
                if (has_calibration_profile_) {
                    const float scale = std::isfinite(calibration_profile_.muzzle_velocity_scale) &&
                                                calibration_profile_.muzzle_velocity_scale > 0.1f
                                            ? calibration_profile_.muzzle_velocity_scale
                                            : 1.0f;
                    runtime_mv_ms = runtime_mv_ms * scale + calibration_profile_.muzzle_velocity_bias_ms;
                }
                runtime_mv_ms += thermalMvDeltaFps() * dope::math::FPS_TO_MPS;
                runtime_mv_ms = std::fmax(runtime_mv_ms, kMinMvMs);
                mv_ratio = clampf(runtime_mv_ms / std::fmax(active.muzzle_velocity_ms, kMinMvMs), 0.65f, 1.45f);
            }
            const float drop_scale = 1.0f / (mv_ratio * mv_ratio);
            const float tof_scale = 1.0f / mv_ratio;

            if (dataset_v2_.num_d_drop_dbc_points > 0) {
                const float bc_baseline = dataset_v2_.bc;
                const float bc_corrected = bc_baseline * atmo_.correctBC(1.0f);
                const float d_dbc = interpolateProfileValue(dataset_v2_.d_drop_dbc_by_range,
                                                            dataset_v2_.num_d_drop_dbc_points,
                                                            horizontal_range_m);
                drop_m += (bc_corrected - bc_baseline) * d_dbc;
            } else {
                float density_scale = 1.0f;
                if (std::isfinite(dataset_v2_.baseline_pressure_pa) && dataset_v2_.baseline_pressure_pa > 1000.0f &&
                    std::isfinite(dataset_v2_.baseline_temperature_c)) {
                    Atmosphere baseline;
                    baseline.init();
                    const float h = std::isfinite(dataset_v2_.baseline_humidity)
                                        ? dataset_v2_.baseline_humidity
                                        : 0.5f;
                    baseline.updateFromBaro(dataset_v2_.baseline_pressure_pa,
                                            dataset_v2_.baseline_temperature_c, h);
                    const float rho0 = baseline.getAirDensity();
                    if (rho0 > 0.1f) {
                        density_scale = atmo_.getAirDensity() / rho0;
                    }
                }
                drop_m *= clampf(density_scale, 0.5f, 1.7f);
            }
            drop_m *= drop_scale;

            float windage_m = 0.0f;
            if (dataset_v2_.num_wind_drift_points > 0) {
                float wind_drift_m = interpolateProfileValue(dataset_v2_.wind_drift_by_range,
                                                             dataset_v2_.num_wind_drift_points,
                                                             horizontal_range_m);
                if (std::isfinite(wind_drift_m)) {
                    const float heading = mag_.computeHeading(ahrs_.getYaw());
                    float headwind = 0.0f, crosswind = 0.0f;
                    if (has_ballistic_context_ && ballistic_context_.use_runtime_wind &&
                        std::isfinite(ballistic_context_.wind_speed_ms) &&
                        std::isfinite(ballistic_context_.wind_heading_deg)) {
                        const float rel = (ballistic_context_.wind_heading_deg - heading) * dope::math::DEG_TO_RAD;
                        headwind = ballistic_context_.wind_speed_ms * std::cos(rel);
                        crosswind = ballistic_context_.wind_speed_ms * std::sin(rel);
                    } else {
                        wind_.decompose(heading, headwind, crosswind);
                    }
                    (void)headwind;
                    const float ref_wind = std::fmax(std::fabs(dataset_v2_.baseline_wind_speed_ms), 0.1f);
                    windage_m = std::fabs(wind_drift_m) * tof_scale * (crosswind / ref_wind);
                }
            }

            float vel_ms = interpolateProfileValue(dataset_v2_.velocity_by_range,
                                                   dataset_v2_.num_velocity_points, horizontal_range_m);
            if (std::isfinite(vel_ms) && vel_ms > 0.0f) {
                vel_ms *= mv_ratio;
                vel_ms = std::fmax(vel_ms, kMinMvMs);
            }
            float energy_j = interpolateProfileValue(dataset_v2_.energy_by_range,
                                                     dataset_v2_.num_energy_points, horizontal_range_m);
            float tof_s = interpolateProfileValue(dataset_v2_.tof_by_range, dataset_v2_.num_tof_points,
                                                  horizontal_range_m);
            if (std::isfinite(tof_s) && tof_s > 0.0f) {
                tof_s *= tof_scale;
            }
            if (std::isfinite(energy_j) && energy_j > 0.0f) {
                energy_j *= (mv_ratio * mv_ratio);
            }

            out->drop_m = drop_m;
            out->windage_m = std::isfinite(windage_m) ? windage_m : 0.0f;
            out->velocity_ms = (std::isfinite(vel_ms) && vel_ms > 0.0f) ? vel_ms : 0.0f;
            out->tof_s = (std::isfinite(tof_s) && tof_s > 0.0f)
                             ? tof_s
                             : ((out->velocity_ms > 1.0f) ? (horizontal_range_m / out->velocity_ms) : 0.0f);
            out->energy_j = (std::isfinite(energy_j) && energy_j > 0.0f) ? energy_j : 0.0f;
            return true;
        }
    }

    if (has_dataset_v2_ && !module_caps_.enable_solver_fallback) {
        return false;
    }

    const TrajectoryPoint* p = solver_.getPointAt(range_m);
    if (!p)
        return false;
    *out = *p;
    return true;
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
    (void)now_us; (void)has_zero_; (void)has_latitude_; // debug trace removed
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

    const bool has_v2_dataset = has_dataset_v2_;
    const bool has_legacy_bullet = has_bullet_;
    if (!has_v2_dataset && !has_legacy_bullet) {
        fault_flags_ |= DOPE_Fault::NO_BULLET;
    }
    if (!hasUsableSolverInputs() && !has_v2_dataset) {
        fault_flags_ |= DOPE_Fault::NO_MV;
        fault_flags_ |= DOPE_Fault::NO_BC;
    }
    if (has_zero_ && (zero_.zero_range_m < 1.0f ||
                      zero_.zero_range_m > static_cast<float>(DOPE_MAX_RANGE_M))) {
        fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
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
    if (has_range_ && (has_v2_dataset || has_legacy_bullet)) {
        computeSolution();
        if (solution_.solution_mode == static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY)) {
            mode_ = DOPE_Mode::SOLUTION_READY;
        } else if (solution_.solution_mode == static_cast<uint32_t>(DOPE_Mode::FAULT)) {
            mode_ = DOPE_Mode::FAULT;
        }
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
    (void)defer_uncertainty_; (void)has_range_; (void)has_dataset_v2_; (void)has_bullet_;
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
    const float pitch = ahrs_.getPitch();
    const float roll = ahrs_.getRoll();
    const float yaw = ahrs_.getYaw();
    const float heading_true = mag_.computeHeading(yaw);

    float slant_range_m = lrf_range_filtered_m_;
    if (has_ballistic_context_ && ballistic_context_.use_range_override &&
        std::isfinite(ballistic_context_.range_m) && ballistic_context_.range_m > 1.0f) {
        slant_range_m = ballistic_context_.range_m;
    }
    float target_elevation_m =
        (has_ballistic_context_ && ballistic_context_.use_target_elevation_override &&
         std::isfinite(ballistic_context_.target_elevation_m))
            ? ballistic_context_.target_elevation_m
            : target_elevation_m_;
    if (!std::isfinite(target_elevation_m)) {
        target_elevation_m = 0.0f;
    }
    if (std::fabs(target_elevation_m) >= slant_range_m) {
        const float max_abs_elev = std::fmax(slant_range_m - 0.01f, 0.0f);
        target_elevation_m = std::copysign(max_abs_elev, target_elevation_m);
    }
    const float horizontal_range_m =
        std::sqrt(std::fmax((slant_range_m * slant_range_m) - (target_elevation_m * target_elevation_m),
                            0.01f));

    if (computeTableFirstSolution(slant_range_m, horizontal_range_m, target_elevation_m, pitch, roll)) {
        return;
    }
    if (has_dataset_v2_ && !module_caps_.enable_solver_fallback) {
        // V2 table-first mode is intentionally deterministic: do not run the RK4
        // solver at runtime when dataset channels are insufficient.
        fault_flags_ |= DOPE_Fault::INVALID_AMMO;
        mode_ = DOPE_Mode::FAULT;
        solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::FAULT);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
        uncertainty_pending_ = false;
        solution_.uncertainty_valid = false;
        return;
    }

    // Build solver params (integrates over horizontal distance).
    SolverParams params = buildSolverParams(horizontal_range_m);
    const float launch_angle = zero_angle_rad_ + pitch;
    params.launch_angle_rad = launch_angle;

    // Run solver
    SolverResult result = solver_.integrate(params);

    if (!result.valid) {
        // Zero may be unsolvable
        fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
        mode_ = DOPE_Mode::FAULT;
        solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::FAULT);
        solution_.fault_flags = fault_flags_;
        solution_.defaults_active = diag_flags_;
        uncertainty_pending_ = false;
        solution_.uncertainty_valid = false;
        return;
    }

    // Convert drop/windage to MOA holds
    // Use the filtered range consistently for both the solver inputs and hold math to avoid
    // mixing a smoothed trajectory with an unfiltered denominator at very short ranges.
    float range = slant_range_m;
    float drop_moa = 0.0f;
    float wind_from_wind_moa = 0.0f;

    if (range > 0.0f && horizontal_range_m > 0.0f) {
        // The drop relative to the zero'd sight line:
        // At zero range, the bullet hits the POA. At other ranges,
        // the drop from the sight line must be corrected.
        // Sight line at range R: -sight_height * (R / zero_range) + sight_height
        // Simplification: the zero angle already accounts for this.
        // The solver gives drop from bore line. We need drop from sight line.

        float sight_h = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;
        float zero_range_m =
            (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : horizontal_range_m;
        float sight_line_drop =
            sight_h - (sight_h / zero_range_m) * horizontal_range_m; // [MATH §14.1]

        // Drop relative to sight line
        float relative_drop = result.drop_at_target_m - sight_line_drop; // [MATH §14.1]

        // Convert to angular adjustment in MOA    [MATH §14.2]
        drop_moa = -(relative_drop / horizontal_range_m) * dope::math::RAD_TO_MOA; // [MATH §14.2]
        // Additional vertical angle needed for non-level target geometry (tangent-based).
        drop_moa += (target_elevation_m / horizontal_range_m) * dope::math::RAD_TO_MOA;
        wind_from_wind_moa =
            -(result.windage_at_target_m / horizontal_range_m) * dope::math::RAD_TO_MOA; // [MATH §14.2]
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

    if (has_calibration_profile_) {
        drop_moa += calibration_profile_.drop_bias_moa +
                    interpolateProfileValue(calibration_profile_.drop_residual_by_range,
                                            calibration_profile_.num_drop_residual_points,
                                            horizontal_range_m);
        windage_moa += calibration_profile_.wind_bias_moa +
                       interpolateProfileValue(calibration_profile_.wind_residual_by_range,
                                               calibration_profile_.num_wind_residual_points,
                                               horizontal_range_m);
    }

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

    // ---[ POI Thermal Drift  — MATH §16.4 ]---
    // Apply an optional user-supplied thermal POI walk (MOA/K above ambient).
    // Uses the barrel delta-T as the primary driver.
    {
        float drift_x = 0.0f;
        float drift_y = 0.0f;
        thermalPoiDrift(drift_x, drift_y);
        solution_.hold_windage_moa += drift_x;
        solution_.hold_elevation_moa += drift_y;
    }

    solution_.range_m = range;
    solution_.horizontal_range_m = horizontal_range_m;
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
    solution_.look_angle_deg = std::atan2(target_elevation_m, horizontal_range_m) * dope::math::RAD_TO_DEG;
    solution_.launch_angle_rad = launch_angle;
    solution_.heading_deg_true = heading_true;
    solution_.air_density_kgm3 = atmo_.getAirDensity();

    // Stage uncertainty so callers can defer the expensive pass
    solution_.uncertainty_valid = false;
    uncertainty_pending_ = true;
    if (!defer_uncertainty_) {
        computeUncertainty();
    }
}

float DOPE_Engine::estimateBarrelMassKg() const {
    float length_m = (bullet_.barrel_length_in > 0.0f)
                         ? bullet_.barrel_length_in * dope::math::INCHES_TO_M
                         : 0.6f; // fallback ~24"
    const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
    float od_m = (bullet_.muzzle_diameter_in > 0.0f)
                     ? bullet_.muzzle_diameter_in * dope::math::INCHES_TO_M
                     : 0.01778f; // ~0.7"
    // Cylindrical approximation; if a wall thickness is provided use hollow
    // cylinder (OD/ID) to approximate metal volume more realistically.
    float volume = 0.0f;
    if (bullet_.barrel_profile.wall_thickness_mm > 0.0f) {
        const float thickness_m = bullet_.barrel_profile.wall_thickness_mm * 0.001f;
        const float id_m = od_m - 2.0f * thickness_m;
        if (id_m > 0.0f && id_m < od_m) {
            const float area = 0.25f * dope::math::PI * (od_m * od_m - id_m * id_m);
            volume = area * length_m;
        }
    }
    if (volume <= 0.0f) {
        const float area = 0.25f * dope::math::PI * od_m * od_m;
        volume = area * length_m;
    }
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
        chamber_temp_K_ = barrel_ambient_K_;
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

    // Chamber cooling (throat / chamber area) — shorter time constant driven
    // by `bullet_.chamber_time_s` when provided, otherwise a moderate default.
    float chamber_dt_s = 0.0f;
    if (last_chamber_update_us_ == 0) {
        chamber_dt_s = 0.0f;
        last_chamber_update_us_ = now_us;
    } else {
        chamber_dt_s = static_cast<float>(now_us - last_chamber_update_us_) * 1e-6f;
    }
    float tau_ch_s = 5.0f; // default chamber cooling
    if (std::isfinite(bullet_.chamber_time_s) && bullet_.chamber_time_s > 0.0f) {
        // Use provided soak time as a hint for chamber thermal inertia
        tau_ch_s = std::fmax(0.1f, bullet_.chamber_time_s);
    }
    if (chamber_dt_s > 0.0f) {
        const float decay_ch = std::exp(-chamber_dt_s / tau_ch_s);
        chamber_temp_K_ = barrel_ambient_K_ + (chamber_temp_K_ - barrel_ambient_K_) * decay_ch;
        last_chamber_update_us_ = now_us;
    }
}

float DOPE_Engine::barrelHeatMultiplier() const {
    const float delta_K_barrel = barrel_temp_K_ - barrel_ambient_K_;
    const float delta_K_chamber = chamber_temp_K_ - barrel_ambient_K_;
    const float delta_K = delta_K_barrel + 0.25f * delta_K_chamber; // chamber contributes modestly
    if (!(delta_K > 0.0f))
        return 1.0f;
    const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
    const float alpha_base = 0.01f; // mild growth per K at stainless baseline
    const float alpha = alpha_base * props.cte_scale;
    const float mult = std::sqrt(1.0f + alpha * delta_K);
    return std::fmin(std::fmax(mult, 1.0f), 1.6f); // cap to avoid runaway
}

float DOPE_Engine::thermalMvDeltaFps() const {
    const float delta_K = barrel_temp_K_ - barrel_ambient_K_;
    if (!(delta_K > 0.0f)) {
        return 0.0f;
    }
    float slope_fps_per_K = bullet_.mv_thermal_slope;
    if (has_dataset_v2_ && std::isfinite(dataset_v2_.mv_thermal_slope) &&
        std::fabs(dataset_v2_.mv_thermal_slope) > 0.0f) {
        slope_fps_per_K = dataset_v2_.mv_thermal_slope;
    }
    if (!std::isfinite(slope_fps_per_K) || std::fabs(slope_fps_per_K) <= 0.0f) {
        return 0.0f;
    }
    return slope_fps_per_K * delta_K;
}

void DOPE_Engine::thermalPoiDrift(float& drift_x_moa, float& drift_y_moa) const {
    drift_x_moa = 0.0f;
    drift_y_moa = 0.0f;
    const float delta_K = barrel_temp_K_ - barrel_ambient_K_;
    if (!(delta_K > 0.0f)) {
        return;
    }
    float drift_x_per_K = bullet_.thermal_drift_x_moa_per_K;
    float drift_y_per_K = bullet_.thermal_drift_y_moa_per_K;
    if (has_dataset_v2_ && std::isfinite(dataset_v2_.thermal_drift_x_moa_per_K) &&
        std::fabs(dataset_v2_.thermal_drift_x_moa_per_K) > 0.0f) {
        drift_x_per_K = dataset_v2_.thermal_drift_x_moa_per_K;
    }
    if (has_dataset_v2_ && std::isfinite(dataset_v2_.thermal_drift_y_moa_per_K) &&
        std::fabs(dataset_v2_.thermal_drift_y_moa_per_K) > 0.0f) {
        drift_y_per_K = dataset_v2_.thermal_drift_y_moa_per_K;
    }
    if (std::isfinite(drift_x_per_K) && std::fabs(drift_x_per_K) > 0.0f) {
        drift_x_moa = drift_x_per_K * delta_K;
    }
    if (std::isfinite(drift_y_per_K) && std::fabs(drift_y_per_K) > 0.0f) {
        drift_y_moa = drift_y_per_K * delta_K;
    }
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
    const ActiveAmmo active = selectActiveAmmo();
    const float bullet_mass_kg = (active.valid && std::isfinite(active.mass_grains) && active.mass_grains > 0.0f)
                                     ? active.mass_grains * dope::math::GRAINS_TO_KG
                                     : 0.0097f; // ~150 gr fallback
    const float mv = active.valid ? std::fmax(1.0f, active.muzzle_velocity_ms) : 1.0f;
    const float muzzle_energy_J = 0.5f * bullet_mass_kg * mv * mv;
    float energy_to_barrel_J = std::fmin(std::fmax(muzzle_energy_J * 0.2f, 400.0f), 2000.0f);
    // Apply user-provided heat-efficiency scalar (1.0 = default)
    if (std::isfinite(bullet_.heat_efficiency_scalar) && bullet_.heat_efficiency_scalar > 0.0f) {
        energy_to_barrel_J *= bullet_.heat_efficiency_scalar;
    }

    // Model chamber energy soak: a fraction of muzzle energy is deposited in the
    // chamber/throat and decays on its own time constant. Larger `chamber_time_s`
    // increases the effective coupling.
    float chamber_frac = 0.05f; // base 5% coupling
    if (std::isfinite(bullet_.chamber_time_s) && bullet_.chamber_time_s > 0.0f) {
        chamber_frac = clampf(0.05f + 0.02f * std::fmin(bullet_.chamber_time_s, 10.0f), 0.02f, 0.35f);
    }
    float energy_to_chamber_J = std::fmin(std::fmax(muzzle_energy_J * chamber_frac, 20.0f), 1500.0f);

    // Cold-bore transition behavior: the first shot after a long break couples
    // energy differently (more into throat, less into barrel) — approximate.
    const bool was_cold_bore = (shots_in_string_ == 0);
    if (was_cold_bore) {
        energy_to_barrel_J *= 0.6f;
        energy_to_chamber_J *= 1.4f;
    }

    const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
    const float heat_capacity = estimateBarrelMassKg() * props.specific_heat_J_kgK;
    if (heat_capacity > 0.0f) {
        const float delta_K = energy_to_barrel_J / heat_capacity;
        if (std::isfinite(delta_K) && delta_K > 0.0f) {
            barrel_temp_K_ += delta_K;
        }
    }

    // Apply chamber energy into a smaller thermal mass (throat/chamber). We
    // approximate chamber mass as a small fraction of the barrel mass.
    const float chamber_mass_frac = 0.03f; // ~3% of barrel mass
    const float chamber_mass = std::fmax(0.001f, estimateBarrelMassKg() * chamber_mass_frac);
    const float chamber_heat_capacity = chamber_mass * props.specific_heat_J_kgK;
    if (chamber_heat_capacity > 0.0f) {
        const float dKc = energy_to_chamber_J / chamber_heat_capacity;
        if (std::isfinite(dKc) && dKc > 0.0f) {
            chamber_temp_K_ += dKc;
        }
    }

    last_shot_time_us_ = timestamp_us;
    last_barrel_update_us_ = timestamp_us;
    last_chamber_update_us_ = timestamp_us;
    ++shots_in_string_;
}

// ---------------------------------------------------------------------------
// Internal: recompute zero angle
// ---------------------------------------------------------------------------

void DOPE_Engine::recomputeZero() {
    zero_dirty_ = false;

    if (!has_zero_) {
        zero_angle_rad_ = 0.0f;
        return;
    }

    if (zero_.zero_range_m < 1.0f || zero_.zero_range_m > DOPE_MAX_RANGE_M) {
        fault_flags_ |= DOPE_Fault::ZERO_UNSOLVABLE;
        zero_angle_rad_ = 0.0f;
        return;
    }

    if (has_dataset_v2_ && dataset_v2_.num_trajectories > 0 && !module_caps_.enable_solver_fallback) {
        // In table-first mode, infer launch angle from the nearest precomputed
        // trajectory family instead of invoking runtime zero-angle solve.
        int best_idx = 0;
        float best_err = std::fabs(dataset_v2_.trajectories[0].zero_range_m - zero_.zero_range_m);
        for (int i = 1; i < dataset_v2_.num_trajectories; ++i) {
            const float err = std::fabs(dataset_v2_.trajectories[i].zero_range_m - zero_.zero_range_m);
            if (err < best_err) {
                best_err = err;
                best_idx = i;
            }
        }
        const DOPE_TrajectoryFamily& fam = dataset_v2_.trajectories[best_idx];
        if (fam.num_points >= 2 && fam.points[1].range_m > 1.0f) {
            zero_angle_rad_ = -fam.points[1].drop_m / fam.points[1].range_m;
        } else {
            zero_angle_rad_ = 0.0f;
        }
        return;
    }
    if (!hasUsableSolverInputs()) {
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
    float base_bc = 0.0f;
    float base_mv_ms = 0.0f;
    float base_mass_gr = 0.0f;
    float base_len_mm = 0.0f;
    float base_cal_in = 0.0f;
    float base_twist_in = 0.0f;
    float mv_adjust_fps_per_in = 0.0f;
    float ref_barrel_in = 24.0f;
    float barrel_in = 24.0f;

    // Use centralized active-ammo selection to choose authoritative base
    // parameters. Under full migration this will prefer AmmoDatasetV2.
    const ActiveAmmo a = selectActiveAmmo();
    if (a.valid) {
        base_bc = a.bc;
        base_mv_ms = a.muzzle_velocity_ms;
        base_mass_gr = a.mass_grains;
        base_len_mm = a.length_mm;
        base_cal_in = a.caliber_inches;
        base_twist_in = a.twist_rate_inches;
        mv_adjust_fps_per_in = a.mv_adjustment_fps_per_in;
        ref_barrel_in = (a.baseline_barrel_length_in > 0.0f) ? a.baseline_barrel_length_in : ref_barrel_in;
        barrel_in = (bullet_.barrel_length_in > 0.0f) ? bullet_.barrel_length_in : barrel_in;
        if (a.drag_model != static_cast<DragModel>(0)) {
            bool allowed = true;
            if (a.is_v2) {
                if (a.supported_drag_models_mask != 0) {
                    const uint8_t bit = static_cast<uint8_t>(1u << (static_cast<uint8_t>(a.drag_model) - 1));
                    allowed = (a.supported_drag_models_mask & bit) != 0;
                }
            }
            if (allowed) p.drag_model = a.drag_model;
        }
    }

    p.bc = atmo_.correctBC(base_bc);
    if (has_calibration_profile_ && calibration_profile_.bc_scale > 0.1f && calibration_profile_.bc_scale < 10.0f) {
        p.bc *= calibration_profile_.bc_scale;
    }
    float base_mv_fps = base_mv_ms * 3.28084f;
    float mv_ms = base_mv_ms;
    if (a.valid && a.num_barrel_mv_points > 0) {
        mv_ms = interpolateBarrelMvMs(a.barrel_mv_by_length_in, a.num_barrel_mv_points, barrel_in);
    } else {
        mv_adjust_fps_per_in = effectiveMvAdjustmentFpsPerIn(base_mv_ms, mv_adjust_fps_per_in);
        float adjusted_mv_fps = base_mv_fps + ((barrel_in - ref_barrel_in) * mv_adjust_fps_per_in);
        mv_ms = adjusted_mv_fps * 0.3048f;
    }
    if (has_calibration_profile_) {
        const float scale = std::isfinite(calibration_profile_.muzzle_velocity_scale) &&
                                    calibration_profile_.muzzle_velocity_scale > 0.1f
                                ? calibration_profile_.muzzle_velocity_scale
                                : 1.0f;
        mv_ms = mv_ms * scale + calibration_profile_.muzzle_velocity_bias_ms;
    }
    // Radar assist removed; no MV scaling applied here.
    p.muzzle_velocity_ms = std::fmax(mv_ms, kMinMvMs);

    // ---[ MV Thermal Correction  — MATH §16.4 ]---
    // Apply an optional user-supplied mv_thermal_slope (fps/°C above ambient)
    // using the current barrel delta-T relative to ambient.
    {
        const float mv_delta_fps = thermalMvDeltaFps();
        if (std::fabs(mv_delta_fps) > 0.0f) {
            p.muzzle_velocity_ms += mv_delta_fps * dope::math::FPS_TO_MPS;
            p.muzzle_velocity_ms = std::fmax(p.muzzle_velocity_ms, kMinMvMs);
        }
    }
    p.bullet_mass_kg = std::fmax(base_mass_gr, 1.0f) * dope::math::GRAINS_TO_KG;
    p.bullet_length_m = std::fmax(base_len_mm, 1.0f) * dope::math::MM_TO_M;
    p.sight_height_m = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;

    p.air_density = atmo_.getAirDensity();
    p.speed_of_sound = atmo_.getSpeedOfSound();
    if (has_ballistic_context_ && ballistic_context_.use_runtime_atmosphere &&
        std::isfinite(ballistic_context_.pressure_pa) && std::isfinite(ballistic_context_.temperature_c)) {
        Atmosphere runtime_atmo;
        runtime_atmo.init();
        const float humidity = std::isfinite(ballistic_context_.humidity) ? ballistic_context_.humidity : 0.5f;
        runtime_atmo.updateFromBaro(ballistic_context_.pressure_pa, ballistic_context_.temperature_c, humidity);
        p.bc = runtime_atmo.correctBC(base_bc);
        p.air_density = runtime_atmo.getAirDensity();
        p.speed_of_sound = runtime_atmo.getSpeedOfSound();
    }
    p.drag_reference_scale = external_reference_mode_ ? DOPE_EXTERNAL_REFERENCE_DRAG_SCALE
                                                      : DOPE_DEFAULT_DRAG_REFERENCE_SCALE;
    p.target_range_m = range_m;
    p.launch_angle_rad = 0.0f; // set by caller

    // Wind decomposition    [MATH §10]
    const float heading = mag_.computeHeading(ahrs_.getYaw());
    if (has_ballistic_context_ && ballistic_context_.use_runtime_wind &&
        std::isfinite(ballistic_context_.wind_speed_ms) && std::isfinite(ballistic_context_.wind_heading_deg)) {
        const float rel = (ballistic_context_.wind_heading_deg - heading) * dope::math::DEG_TO_RAD;
        p.headwind_ms = ballistic_context_.wind_speed_ms * std::cos(rel);
        p.crosswind_ms = ballistic_context_.wind_speed_ms * std::sin(rel);
    } else {
        wind_.decompose(heading, p.headwind_ms, p.crosswind_ms);
    }

    // Coriolis
    if (has_latitude_) {
        p.coriolis_enabled = true;
        p.coriolis_lat_rad = latitude_deg_ * dope::math::DEG_TO_RAD;
        p.azimuth_rad = heading * dope::math::DEG_TO_RAD;
    } else {
        p.coriolis_enabled = false;
    }

    // Spin drift
    if (std::fabs(base_twist_in) > 0.1f) {
        p.spin_drift_enabled = true;
        p.twist_rate_inches = base_twist_in;
        p.caliber_m = std::fmax(base_cal_in, 0.1f) * dope::math::INCHES_TO_M;
    } else {
        p.spin_drift_enabled = false;
    }

    return p;
}

bool DOPE_Engine::hasUsableSolverInputs() const {
    // Prefer V2 datasets. Legacy BulletProfile no longer carries sparse
    // ballistic fields, so solver inputs are only usable when a V2 dataset
    // is present and contains reasonable parameters.
    if (has_dataset_v2_ && dataset_v2_.muzzle_velocity_ms > kMinMvMs && dataset_v2_.bc > 0.001f) {
        return true;
    }
    return false;
}

float DOPE_Engine::interpolateProfileValue(const DOPE_ProfilePoint* points, int count, float range_m) const {
    if (!points || count <= 0) {
        return 0.0f;
    }
    if (count == 1) {
        return points[0].value;
    }
    if (range_m <= points[0].range_m) {
        return points[0].value;
    }
    if (range_m >= points[count - 1].range_m) {
        return points[count - 1].value;
    }
    for (int i = 0; i < count - 1; ++i) {
        if (range_m <= points[i + 1].range_m) {
            const float dx = points[i + 1].range_m - points[i].range_m;
            if (dx <= 0.0f) {
                return points[i + 1].value;
            }
            const float t = (range_m - points[i].range_m) / dx;
            return points[i].value + t * (points[i + 1].value - points[i].value);
        }
    }
    return points[count - 1].value;
}

float DOPE_Engine::getActiveAmmoCep50(float range_m) const {
    if (has_dataset_v2_) {
        const float table_cep =
            interpolateProfileValue(dataset_v2_.cep50_by_range, dataset_v2_.num_cep50_points, range_m);
        if (std::isfinite(table_cep) && table_cep > 0.0f) {
            return table_cep;
        }
    }
    if (uncertainty_config_.cartridge_cep_table.points &&
        uncertainty_config_.cartridge_cep_table.count > 0) {
        const DOPE_CEPTable t = uncertainty_config_.cartridge_cep_table;
        if (t.count == 1) {
            return t.points[0].cep50_moa;
        }
        if (range_m <= t.points[0].range_m) {
            return t.points[0].cep50_moa;
        }
        if (range_m >= t.points[t.count - 1].range_m) {
            return t.points[t.count - 1].cep50_moa;
        }
        for (int i = 0; i < t.count - 1; ++i) {
            if (range_m <= t.points[i + 1].range_m) {
                const float dx = t.points[i + 1].range_m - t.points[i].range_m;
                const float a = (dx > 0.0f) ? ((range_m - t.points[i].range_m) / dx) : 0.0f;
                return t.points[i].cep50_moa + a * (t.points[i + 1].cep50_moa - t.points[i].cep50_moa);
            }
        }
    }
    if (std::isfinite(uncertainty_config_.ammo_cep50_moa) && uncertainty_config_.ammo_cep50_moa > 0.0f) {
        return uncertainty_config_.ammo_cep50_moa;
    }
    return 0.0f;
}

bool DOPE_Engine::computeTableFirstSolution(float slant_range_m, float horizontal_range_m,
                                            float target_elevation_m, float pitch_rad,
                                            float roll_rad) {
    if (!has_dataset_v2_ || dataset_v2_.num_trajectories <= 0) {
        return false;
    }

    float drop_m = 0.0f;
    if (dataset_v2_.cached_full_table_present && dataset_v2_.cached_full_table_num_points > 1 &&
        std::isfinite(dataset_v2_.cached_full_table_step_m) &&
        dataset_v2_.cached_full_table_step_m > 0.0f) {
        const float step = dataset_v2_.cached_full_table_step_m;
        const int n = clampCount(dataset_v2_.cached_full_table_num_points, DOPE_MAX_TABLE_POINTS);
        int idx = (int)(horizontal_range_m / step);
        idx = (idx < 0) ? 0 : (idx >= n - 1) ? (n - 2) : idx;
        const float t = clampf((horizontal_range_m - idx * step) / step, 0.0f, 1.0f);
        drop_m = dataset_v2_.cached_full_trajectory[idx].drop_m * (1.0f - t) +
                 dataset_v2_.cached_full_trajectory[idx + 1].drop_m * t;
    } else {
        const float active_zero = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : 100.0f;
        int best_idx = 0;
        float best_err = std::fabs(dataset_v2_.trajectories[0].zero_range_m - active_zero);
        for (int i = 1; i < dataset_v2_.num_trajectories; ++i) {
            const float err = std::fabs(dataset_v2_.trajectories[i].zero_range_m - active_zero);
            if (err < best_err) {
                best_err = err;
                best_idx = i;
            }
        }

        const DOPE_TrajectoryFamily& fam = dataset_v2_.trajectories[best_idx];
        if (fam.num_points <= 0) {
            return false;
        }

        DOPE_ProfilePoint traj[DOPE_MAX_TABLE_POINTS];
        const int traj_count = (fam.num_points > DOPE_MAX_TABLE_POINTS) ? DOPE_MAX_TABLE_POINTS : fam.num_points;
        for (int i = 0; i < traj_count; ++i) {
            traj[i].range_m = fam.points[i].range_m;
            traj[i].value = fam.points[i].drop_m;
        }
        drop_m = interpolateProfileValue(traj, traj_count, horizontal_range_m);
        if (!std::isfinite(drop_m)) {
            return false;
        }
    }

    if (dataset_v2_.num_d_drop_dbc_points > 0) {
        const float bc_baseline = dataset_v2_.bc;
        const float bc_corrected = bc_baseline * atmo_.correctBC(1.0f);
        const float d_dbc = interpolateProfileValue(
            dataset_v2_.d_drop_dbc_by_range,
            dataset_v2_.num_d_drop_dbc_points,
            horizontal_range_m);
        drop_m += (bc_corrected - bc_baseline) * d_dbc;
    } else {
        // Baseline-to-runtime scaling using a simple density + MV factor.
        float density_scale = 1.0f;
        if (std::isfinite(dataset_v2_.baseline_pressure_pa) && dataset_v2_.baseline_pressure_pa > 1000.0f &&
            std::isfinite(dataset_v2_.baseline_temperature_c)) {
            Atmosphere baseline;
            baseline.init();
            const float h = std::isfinite(dataset_v2_.baseline_humidity) ? dataset_v2_.baseline_humidity : 0.5f;
            baseline.updateFromBaro(dataset_v2_.baseline_pressure_pa, dataset_v2_.baseline_temperature_c, h);
            const float rho0 = baseline.getAirDensity();
            if (rho0 > 0.1f) {
                density_scale = atmo_.getAirDensity() / rho0;
            }
        }
        drop_m *= clampf(density_scale, 0.5f, 1.7f);
    }

    float mv_ratio = 1.0f; // runtime MV / baseline MV
    {
        const ActiveAmmo active = selectActiveAmmo();
        if (active.valid && active.muzzle_velocity_ms > 0.0f) {
            const float ref_barrel_in = (active.baseline_barrel_length_in > 0.0f)
                                            ? active.baseline_barrel_length_in
                                            : 24.0f;
            const float barrel_in = (bullet_.barrel_length_in > 0.0f) ? bullet_.barrel_length_in : ref_barrel_in;
            float runtime_mv_ms = active.muzzle_velocity_ms;
            if (active.num_barrel_mv_points > 0) {
                runtime_mv_ms = interpolateBarrelMvMs(active.barrel_mv_by_length_in,
                                                      active.num_barrel_mv_points, barrel_in);
            } else {
                const float base_mv_fps = active.muzzle_velocity_ms * 3.28084f;
                const float mv_adjust_fps_per_in =
                    effectiveMvAdjustmentFpsPerIn(active.muzzle_velocity_ms,
                                                  active.mv_adjustment_fps_per_in);
                const float adjusted_mv_fps =
                    base_mv_fps + ((barrel_in - ref_barrel_in) * mv_adjust_fps_per_in);
                runtime_mv_ms = adjusted_mv_fps * dope::math::FPS_TO_MPS;
            }
            if (has_calibration_profile_) {
                const float scale = std::isfinite(calibration_profile_.muzzle_velocity_scale) &&
                                            calibration_profile_.muzzle_velocity_scale > 0.1f
                                        ? calibration_profile_.muzzle_velocity_scale
                                        : 1.0f;
                runtime_mv_ms = runtime_mv_ms * scale + calibration_profile_.muzzle_velocity_bias_ms;
            }
            runtime_mv_ms += thermalMvDeltaFps() * dope::math::FPS_TO_MPS;
            runtime_mv_ms = std::fmax(runtime_mv_ms, kMinMvMs);
            mv_ratio = clampf(runtime_mv_ms / std::fmax(active.muzzle_velocity_ms, kMinMvMs), 0.65f, 1.45f);
        }
    }
    const float drop_scale = 1.0f / (mv_ratio * mv_ratio);
    const float tof_scale = 1.0f / mv_ratio;
    drop_m *= drop_scale;

    const float heading = mag_.computeHeading(ahrs_.getYaw());
    float headwind = 0.0f, crosswind = 0.0f;
    if (has_ballistic_context_ && ballistic_context_.use_runtime_wind &&
        std::isfinite(ballistic_context_.wind_speed_ms) && std::isfinite(ballistic_context_.wind_heading_deg)) {
        const float rel = (ballistic_context_.wind_heading_deg - heading) * dope::math::DEG_TO_RAD;
        headwind = ballistic_context_.wind_speed_ms * std::cos(rel);
        crosswind = ballistic_context_.wind_speed_ms * std::sin(rel);
    } else {
        wind_.decompose(heading, headwind, crosswind);
    }

    const bool strict_table_only = !module_caps_.enable_solver_fallback;
    float vel_ms = interpolateProfileValue(dataset_v2_.velocity_by_range, dataset_v2_.num_velocity_points,
                                           horizontal_range_m);
    if (std::isfinite(vel_ms) && vel_ms > 0.0f) {
        vel_ms *= mv_ratio;
        vel_ms = std::fmax(vel_ms, kMinMvMs);
    }
    float tof_s = interpolateProfileValue(dataset_v2_.tof_by_range, dataset_v2_.num_tof_points,
                                          horizontal_range_m);
    if (std::isfinite(tof_s) && tof_s > 0.0f) {
        tof_s *= tof_scale;
    }
    if (strict_table_only && (!std::isfinite(tof_s) || tof_s <= 0.0f) &&
        std::isfinite(vel_ms) && vel_ms > 1.0f) {
        tof_s = horizontal_range_m / vel_ms;
    }
    if (strict_table_only && (!std::isfinite(vel_ms) || vel_ms <= 0.0f) &&
        std::isfinite(tof_s) && tof_s > 0.0f) {
        vel_ms = std::fmax(horizontal_range_m / tof_s, kMinMvMs);
    }
    if (strict_table_only && ((!std::isfinite(vel_ms) || vel_ms <= 0.0f) &&
                              (!std::isfinite(tof_s) || tof_s <= 0.0f))) {
        return false;
    }
    float energy_j = interpolateProfileValue(dataset_v2_.energy_by_range, dataset_v2_.num_energy_points,
                                             horizontal_range_m);
    if (std::isfinite(energy_j) && energy_j > 0.0f) {
        energy_j *= (mv_ratio * mv_ratio);
    }
    if (strict_table_only && (!std::isfinite(energy_j) || energy_j <= 0.0f)) {
        const ActiveAmmo active = selectActiveAmmo();
        const float mass_kg = (active.valid && std::isfinite(active.mass_grains) && active.mass_grains > 0.0f)
                                  ? active.mass_grains * dope::math::GRAINS_TO_KG
                                  : 0.0f;
        if (mass_kg > 0.0f && std::isfinite(vel_ms) && vel_ms > 0.0f) {
            energy_j = 0.5f * mass_kg * vel_ms * vel_ms;
        }
    }

    float wind_drift_m = NAN;
    if (dataset_v2_.num_wind_drift_points > 0) {
        wind_drift_m = interpolateProfileValue(dataset_v2_.wind_drift_by_range,
                                               dataset_v2_.num_wind_drift_points, horizontal_range_m);
        if (std::isfinite(wind_drift_m)) {
            const float ref_wind = std::fmax(std::fabs(dataset_v2_.baseline_wind_speed_ms), 0.1f);
            wind_drift_m = std::fabs(wind_drift_m) * tof_scale * (crosswind / ref_wind);
        }
    }
    if (strict_table_only && (!std::isfinite(wind_drift_m) || wind_drift_m == 0.0f) &&
        std::isfinite(tof_s) && tof_s > 0.0f && std::isfinite(crosswind)) {
        // Table-only fallback: lateral drift approximation from crosswind * TOF.
        wind_drift_m = crosswind * tof_s;
    }
    if (strict_table_only && !std::isfinite(wind_drift_m)) {
        return false;
    }

    SolverResult solver_result = {};
    bool have_solver = false;
    if (module_caps_.enable_solver_fallback && hasUsableSolverInputs()) {
        SolverParams sp = buildSolverParams(horizontal_range_m);
        sp.launch_angle_rad = zero_angle_rad_ + pitch_rad;
        solver_result = solver_.integrate(sp);
        have_solver = solver_result.valid;
    }

    const float sight_h = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;
    const float zero_range_m = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : horizontal_range_m;
    const float sight_line_drop = sight_h - (sight_h / zero_range_m) * horizontal_range_m;
    const float rel_drop = drop_m - sight_line_drop;
    float drop_moa = -(rel_drop / std::fmax(horizontal_range_m, 1.0f)) * dope::math::RAD_TO_MOA;
    drop_moa += (target_elevation_m / std::fmax(horizontal_range_m, 1.0f)) * dope::math::RAD_TO_MOA;

    float wind_from_wind_moa = 0.0f;
    if (std::isfinite(wind_drift_m)) {
        wind_from_wind_moa = -(wind_drift_m / std::fmax(horizontal_range_m, 1.0f)) * dope::math::RAD_TO_MOA;
    } else if (have_solver) {
        wind_from_wind_moa =
            -(solver_result.windage_at_target_m / std::fmax(horizontal_range_m, 1.0f)) * dope::math::RAD_TO_MOA;
    } else if (module_caps_.enable_solver_fallback) {
        return false;
    }

    float coriolis_elev = have_solver ? solver_result.coriolis_elev_moa : 0.0f;
    float coriolis_wind = have_solver ? solver_result.coriolis_wind_moa : 0.0f;
    float spin_drift = have_solver ? solver_result.spin_drift_moa : 0.0f;
    if (strict_table_only && !have_solver && std::isfinite(tof_s) && tof_s > 0.0f &&
        horizontal_range_m > 0.0f) {
        const float azi = heading * dope::math::DEG_TO_RAD;
        if (has_latitude_) {
            const float lat = latitude_deg_ * dope::math::DEG_TO_RAD;
            const float coriolis_hz =
                DOPE_OMEGA_EARTH * horizontal_range_m * tof_s * std::sin(lat) * std::sin(azi);
            const float coriolis_vt =
                DOPE_OMEGA_EARTH * horizontal_range_m * tof_s * std::cos(lat) * std::sin(azi);
            coriolis_wind = (coriolis_hz / horizontal_range_m) * dope::math::RAD_TO_MOA;
            coriolis_elev = (coriolis_vt / horizontal_range_m) * dope::math::RAD_TO_MOA;
        }
        const ActiveAmmo active = selectActiveAmmo();
        if (active.valid && std::fabs(active.twist_rate_inches) > 0.1f) {
            const float sg = estimateDynamicStabilitySGApprox(
                active.mass_grains, active.caliber_inches, active.length_mm, active.twist_rate_inches,
                std::isfinite(vel_ms) && vel_ms > 0.0f ? vel_ms : active.muzzle_velocity_ms,
                atmo_.getAirDensity());
            float drift_m = 0.0254f * 1.25f * (sg + 1.2f) * std::pow(tof_s, 1.83f);
            if (active.twist_rate_inches < 0.0f) {
                drift_m = -drift_m;
            }
            spin_drift = (drift_m / horizontal_range_m) * dope::math::RAD_TO_MOA;
        }
    }
    drop_moa += coriolis_elev;
    float windage_moa = wind_from_wind_moa + coriolis_wind + spin_drift;
    drop_moa += boresight_.vertical_moa + reticle_.vertical_moa;
    windage_moa += boresight_.horizontal_moa + reticle_.horizontal_moa;

    if (has_calibration_profile_) {
        drop_moa += calibration_profile_.drop_bias_moa +
                    interpolateProfileValue(calibration_profile_.drop_residual_by_range,
                                            calibration_profile_.num_drop_residual_points,
                                            horizontal_range_m);
        windage_moa += calibration_profile_.wind_bias_moa +
                       interpolateProfileValue(calibration_profile_.wind_residual_by_range,
                                               calibration_profile_.num_wind_residual_points,
                                               horizontal_range_m);
    }

    const float wind_before_cant = windage_moa;
    float cant_elev, cant_wind;
    CantCorrection::apply(roll_rad, drop_moa, cant_elev, cant_wind);
    drop_moa = cant_elev;
    windage_moa += cant_wind;

    if ((!std::isfinite(vel_ms) || vel_ms <= 0.0f) && have_solver) {
        vel_ms = solver_result.velocity_at_target_ms;
    }
    if ((!std::isfinite(energy_j) || energy_j <= 0.0f) && have_solver) {
        energy_j = solver_result.energy_at_target_j;
    }
    if ((!std::isfinite(tof_s) || tof_s <= 0.0f) && have_solver) {
        tof_s = solver_result.tof_s;
    }
    if ((!std::isfinite(tof_s) || tof_s <= 0.0f) && std::isfinite(vel_ms) && vel_ms > 1.0f) {
        tof_s = horizontal_range_m / vel_ms;
    }

    solution_.solution_mode = static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY);
    solution_.fault_flags = fault_flags_;
    solution_.defaults_active = diag_flags_;
    solution_.hold_elevation_moa = drop_moa;
    solution_.hold_windage_moa = windage_moa;

    // ---[ POI Thermal Drift  — MATH §16.4.2 ]---
    // Mirror of the same correction in computeSolution(). Both paths must
    // apply the thermal bias so the output is consistent regardless of
    // whether the V2 table or the physics solver is active.
    {
        float drift_x = 0.0f;
        float drift_y = 0.0f;
        thermalPoiDrift(drift_x, drift_y);
        solution_.hold_windage_moa += drift_x;
        solution_.hold_elevation_moa += drift_y;
    }
    solution_.range_m = slant_range_m;
    solution_.horizontal_range_m = horizontal_range_m;
    solution_.tof_ms = (std::isfinite(tof_s) && tof_s > 0.0f)
                           ? tof_s * 1000.0f
                           : (std::isfinite(vel_ms) && vel_ms > 1.0f)
                                 ? (horizontal_range_m / vel_ms) * 1000.0f
                                 : 0.0f;
    solution_.velocity_at_target_ms = std::isfinite(vel_ms) ? vel_ms : 0.0f;
    solution_.energy_at_target_j = std::isfinite(energy_j) ? energy_j : 0.0f;
    solution_.coriolis_windage_moa = coriolis_wind;
    solution_.coriolis_elevation_moa = coriolis_elev;
    solution_.spin_drift_moa = spin_drift;
    solution_.wind_only_windage_moa = wind_from_wind_moa;
    solution_.earth_spin_windage_moa = coriolis_wind + spin_drift;
    solution_.offsets_windage_moa = boresight_.horizontal_moa + reticle_.horizontal_moa;
    solution_.cant_windage_moa = windage_moa - wind_before_cant;
    solution_.cant_angle_deg = roll_rad * dope::math::RAD_TO_DEG;
    solution_.look_angle_deg = std::atan2(target_elevation_m, horizontal_range_m) * dope::math::RAD_TO_DEG;
    solution_.launch_angle_rad = zero_angle_rad_ + pitch_rad;
    solution_.heading_deg_true = heading;
    solution_.air_density_kgm3 = atmo_.getAirDensity();
    solution_.uncertainty_valid = false;
    uncertainty_pending_ = true;
    if (!defer_uncertainty_) {
        computeUncertainty();
    }
    return true;
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

void DOPE_Engine::setDeferUncertainty(bool enabled) {
    defer_uncertainty_ = enabled;
    if (!defer_uncertainty_ && uncertainty_pending_) {
        computeUncertainty();
    }
}

void DOPE_Engine::computeUncertaintyOnly() {
    if (!uncertainty_pending_)
        return;
    computeUncertainty();
}

void DOPE_Engine::getDefaultUncertaintyConfig(UncertaintyConfig* out) {
    if (!out)
        return;
    out->enabled = true;
    out->sigma_muzzle_velocity_ms = 1.5f;       // ~5 fps standard deviation
    out->sigma_range_m = 1.0f;                  // 1 m range uncertainty
    out->sigma_wind_speed_ms = 0.44704f;        // 1 mph wind speed (0.44704 m/s)
    out->sigma_wind_heading_deg = 2.0f;         // 2 ° wind direction
    out->sigma_temperature_c = 1.111f;          // 2 °F temperature
    out->sigma_pressure_pa = 5.0f;              // 5 Pa pressure
    out->sigma_humidity = 0.05f;                // 5 % relative humidity
    out->sigma_sight_height_mm = 0.075f;        // updated default sight-height mounting error
    out->sigma_cant_deg = 1.5f;                 // 1.5 ° cant / roll — RM3100 magnetometer default
    out->sigma_latitude_deg = 0.0f;             // latitude error (disabled by default)
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
    out->cartridge_cep_table = {nullptr, 0};
    out->ammo_cep50_moa = 0.0f;
}

void DOPE_Engine::refreshDerivedSigmasFromProfiles() {
    // Radar-derived MV sigma removed; uncertainty is driven by datasets and profiles.

    // Enforce a minimum MV jump for cold bore (fps → m/s) when present.
    if (has_bullet_ && std::isfinite(bullet_.cold_bore_velocity_bias) &&
        bullet_.cold_bore_velocity_bias > 0.0f) {
        const float cold_bore_ms = bullet_.cold_bore_velocity_bias * 0.3048f;
        if (cold_bore_ms > uncertainty_config_.sigma_muzzle_velocity_ms) {
            uncertainty_config_.sigma_muzzle_velocity_ms = cold_bore_ms;
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

    // Debug trace: show why computeUncertainty might early-return
#ifdef DOPE_DEBUG_UNCERTAINTY
    fprintf(stderr,
            "DEBUG computeUncertainty entry: pending=%d enabled=%d mode=%u slant_range=%f defer=%d has_range=%d has_dataset_v2=%d has_bullet=%d\n",
            uncertainty_pending_ ? 1 : 0, uncertainty_config_.enabled ? 1 : 0,
            static_cast<unsigned int>(solution_.solution_mode), lrf_range_m_, defer_uncertainty_ ? 1 : 0,
            has_range_ ? 1 : 0, has_dataset_v2_ ? 1 : 0, has_bullet_ ? 1 : 0);
#endif

    if (!uncertainty_pending_) {
#ifdef DOPE_DEBUG_UNCERTAINTY
        fprintf(stderr, "DEBUG computeUncertainty: no pending work, returning\n");
#endif
        return;
    }
    uncertainty_pending_ = false;

    if (!uncertainty_config_.enabled) {
#ifdef DOPE_DEBUG_UNCERTAINTY
        fprintf(stderr, "DEBUG computeUncertainty: disabled by config, returning\n");
#endif
        return;
    }

    // Must have a ready solution to propagate through
    if (solution_.solution_mode != static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY)) {
#ifdef DOPE_DEBUG_UNCERTAINTY
        fprintf(stderr, "DEBUG computeUncertainty: solution not ready (mode=%u), returning\n",
                static_cast<unsigned int>(solution_.solution_mode));
#endif
        return;
    }

    const float slant_range = lrf_range_m_;
    if (slant_range < 1.0f) {
#ifdef DOPE_DEBUG_UNCERTAINTY
        fprintf(stderr, "DEBUG computeUncertainty: slant_range=%f < 1.0, returning\n", slant_range);
#endif
        return;
    }

    const float roll = ahrs_.getRoll();
    const float launch_angle = zero_angle_rad_ + ahrs_.getPitch();
    const float sight_h = has_zero_ ? zero_.sight_height_mm * dope::math::MM_TO_M : 0.0f;
    float target_elevation_m = target_elevation_m_;
    if (!std::isfinite(target_elevation_m)) {
        target_elevation_m = 0.0f;
    }
    auto horizontal_from_slant = [&](float slant_m) -> float {
        if (slant_m < 1.0f) {
            return 1.0f;
        }
        float te = target_elevation_m;
        if (std::fabs(te) >= slant_m) {
            const float max_abs_elev = std::fmax(slant_m - 0.01f, 0.0f);
            te = std::copysign(max_abs_elev, te);
        }
        return std::sqrt(std::fmax((slant_m * slant_m) - (te * te), 0.01f));
    };
    const float range = horizontal_from_slant(slant_range);
    const float zero_r = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : range;
    const float pitch = ahrs_.getPitch();

    if (has_dataset_v2_ && !module_caps_.enable_solver_fallback) {
        // Strict table-only runtime mode: uncertainty must come from precomputed
        // per-range sigma channels, not RK4 finite differences.
        for (int i = 0; i < FiringSolution::kNumUncertaintyInputs; ++i) {
            solution_.uc_var_elev[i] = 0.0f;
            solution_.uc_var_wind[i] = 0.0f;
        }
        if (dataset_v2_.num_uncertainty_points <= 0) {
            return;
        }

        auto interp_unc = [&](float r_m, float& sigma_e, float& sigma_w, float& rho) -> bool {
            const int n = clampCount(dataset_v2_.num_uncertainty_points, DOPE_MAX_TABLE_POINTS);
            if (n <= 0) {
                sigma_e = sigma_w = rho = 0.0f;
                return false;
            }
            const auto* pts = dataset_v2_.uncertainty_sigma_by_range;
            if (n == 1 || r_m <= pts[0].range_m) {
                sigma_e = pts[0].sigma_elev_moa;
                sigma_w = pts[0].sigma_wind_moa;
                rho = pts[0].rho;
                return true;
            }
            if (r_m >= pts[n - 1].range_m) {
                sigma_e = pts[n - 1].sigma_elev_moa;
                sigma_w = pts[n - 1].sigma_wind_moa;
                rho = pts[n - 1].rho;
                return true;
            }
            for (int i = 0; i < n - 1; ++i) {
                if (r_m <= pts[i + 1].range_m) {
                    const float dx = pts[i + 1].range_m - pts[i].range_m;
                    const float a = (dx > 0.0f) ? ((r_m - pts[i].range_m) / dx) : 0.0f;
                    sigma_e = pts[i].sigma_elev_moa +
                              a * (pts[i + 1].sigma_elev_moa - pts[i].sigma_elev_moa);
                    sigma_w = pts[i].sigma_wind_moa +
                              a * (pts[i + 1].sigma_wind_moa - pts[i].sigma_wind_moa);
                    rho = pts[i].rho + a * (pts[i + 1].rho - pts[i].rho);
                    return true;
                }
            }
            sigma_e = sigma_w = rho = 0.0f;
            return false;
        };

        float sigma_e = 0.0f;
        float sigma_w = 0.0f;
        float rho = 0.0f;
        if (!interp_unc(range, sigma_e, sigma_w, rho)) {
            return;
        }
        sigma_e = std::fmax(0.0f, sigma_e);
        sigma_w = std::fmax(0.0f, sigma_w);
        rho = clampf(rho, -1.0f, 1.0f);

        if (has_calibration_profile_ &&
            std::isfinite(calibration_profile_.uncertainty_scale) &&
            calibration_profile_.uncertainty_scale > 0.0f) {
            sigma_e *= calibration_profile_.uncertainty_scale;
            sigma_w *= calibration_profile_.uncertainty_scale;
        }

        solution_.sigma_elevation_moa = sigma_e;
        solution_.sigma_windage_moa = sigma_w;
        solution_.covariance_elev_wind = rho * sigma_e * sigma_w;
        solution_.uncertainty_valid = true;
        return;
    }

    // Pre-build the nominal SolverParams once
    SolverParams base = buildSolverParams(range);
    base.launch_angle_rad = launch_angle;
    const ActiveAmmo active = selectActiveAmmo();
    const float raw_bc = (active.valid ? active.bc : 0.0f);

    // Evaluate (elevation_moa, windage_moa) for arbitrary SolverParams + roll
    auto evalMOAWithZeroRange = [&](const SolverParams& p, float roll_rad, float zero_range_m,
                                    float& elev_moa, float& wind_moa) -> bool {
        SolverResult r = solver_.integrate(p, false);
        if (!r.valid) {
            elev_moa = 0.0f;
            wind_moa = 0.0f;
            return false;
        }

        float safe_zero_r = std::fmax(1.0f, zero_range_m);
        float sl_drop = sight_h - (sight_h / safe_zero_r) * p.target_range_m;
        float rel_drop = r.drop_at_target_m - sl_drop;

        const float safe_h = std::fmax(1.0f, p.target_range_m);
        float em = -(rel_drop / safe_h) * dope::math::RAD_TO_MOA + r.coriolis_elev_moa;
        em += (target_elevation_m / safe_h) * dope::math::RAD_TO_MOA;
        float wm = -(r.windage_at_target_m / safe_h) * dope::math::RAD_TO_MOA +
                   r.coriolis_wind_moa + r.spin_drift_moa;

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


    // 2. Range (re-evaluate with different target distances + matching sight-line geometry)
    input_idx = 1;
    {
        float h = uncertainty_config_.sigma_range_m;
        float rp = slant_range + h;
        float rm = std::fmax(1.0f, slant_range - h);

        const float hp = horizontal_from_slant(rp);
        const float hm = horizontal_from_slant(rm);
        SolverParams pp = buildSolverParams(hp);
        SolverParams pm = buildSolverParams(hm);
        pp.launch_angle_rad = launch_angle;
        pm.launch_angle_rad = launch_angle;

        auto evalRange = [&](const SolverParams& p, float& elev_moa,
                             float& wind_moa) -> bool {
            SolverResult r = solver_.integrate(p, false);
            if (!r.valid) {
                elev_moa = 0.0f;
                wind_moa = 0.0f;
                return false;
            }
            const float horiz_rng = std::fmax(1.0f, p.target_range_m);
            float zr = (has_zero_ && zero_.zero_range_m > 0.0f) ? zero_.zero_range_m : horiz_rng;
            float sl_drop = sight_h - (sight_h / zr) * horiz_rng;
            float rel_drop = r.drop_at_target_m - sl_drop;
            float em = -(rel_drop / horiz_rng) * dope::math::RAD_TO_MOA + r.coriolis_elev_moa;
            em += (target_elevation_m / horiz_rng) * dope::math::RAD_TO_MOA;
            float wm = -(r.windage_at_target_m / horiz_rng) * dope::math::RAD_TO_MOA + r.coriolis_wind_moa +
                       r.spin_drift_moa;
            float ce, cw2;
            CantCorrection::apply(roll, em, ce, cw2);
            elev_moa = ce;
            wind_moa = wm + cw2;
            return true;
        };

        float ep, wp, em, wm;
        if (evalRange(pp, ep, wp) && evalRange(pm, em, wm)) {
            float de = (ep - em) * 0.5f;
            float dw = (wp - wm) * 0.5f;
            var_e += de * de;
            var_w += dw * dw;
            cov_ew += de * dw;
            solution_.uc_var_elev[1] = de * de;
            solution_.uc_var_wind[1] = dw * dw;
        }
    }

    // 3. Wind speed (scale existing headwind/crosswind proportionally)
    input_idx = 2;
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
    input_idx = 3;
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
            p.bc = tmp.correctBC(raw_bc);
            p.air_density = tmp.getAirDensity();
            p.speed_of_sound = tmp.getSpeedOfSound();
            return evalMOA(p, roll, elev_moa, wind_moa);
        };

        // 5. Temperature
        input_idx = 4;
        {
            float h = uncertainty_config_.sigma_temperature_c;
            float ep, wp, em, wm;
            if (evalAtmo(+h, 0.0f, 0.0f, ep, wp) && evalAtmo(-h, 0.0f, 0.0f, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de;
                var_w += dw * dw;
                cov_ew += de * dw;
                solution_.uc_var_elev[4] = de * de;
                solution_.uc_var_wind[4] = dw * dw;
            }
        }

        // 6. Pressure
        input_idx = 5;
        {
            float h = uncertainty_config_.sigma_pressure_pa;
            float ep, wp, em, wm;
            if (evalAtmo(0.0f, +h, 0.0f, ep, wp) && evalAtmo(0.0f, -h, 0.0f, em, wm)) {
                float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
                var_e += de * de;
                var_w += dw * dw;
                cov_ew += de * dw;
                solution_.uc_var_elev[5] = de * de;
                solution_.uc_var_wind[5] = dw * dw;
            }
        }

        // 7. Humidity (clamped to [0, 1])
        input_idx = 6;
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
                solution_.uc_var_elev[6] = de * de;
                solution_.uc_var_wind[6] = dw * dw;
            }
        }
    }

    // 8. Sight height
    input_idx = 7;
    {
        SolverParams pp = base, pm = base;
        float h_m = uncertainty_config_.sigma_sight_height_mm * dope::math::MM_TO_M;
        pp.sight_height_m += h_m;
        pm.sight_height_m = std::fmax(0.0f, pm.sight_height_m - h_m);
        accumulate(pp, pm);
    }

    // 9. Cant angle (perturb roll without changing trajectory calculation)
    input_idx = 8;
    {
        float h_rad = uncertainty_config_.sigma_cant_deg * dope::math::DEG_TO_RAD;
        float ep, wp, em, wm;
        if (evalMOA(base, roll + h_rad, ep, wp) && evalMOA(base, roll - h_rad, em, wm)) {
            float de = (ep - em) * 0.5f, dw = (wp - wm) * 0.5f;
            var_e += de * de;
            var_w += dw * dw;
            cov_ew += de * dw;
            solution_.uc_var_elev[8] = de * de;
            solution_.uc_var_wind[8] = dw * dw;
        }
    }

    // 10. Latitude (Coriolis effect) — only meaningful when Coriolis is active
    input_idx = 9;
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



    // 14. Twist rate
    input_idx = 10;
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
    input_idx = 11;
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
                solution_.uc_var_elev[11] = de * de;
                solution_.uc_var_wind[11] = dw * dw;
            }
        }
    }

    // 16. MV adjustment factor (fps/in)
    input_idx = 12;
    {
        SolverParams pp = base, pm = base;
        float h = uncertainty_config_.sigma_mv_adjustment_fps_per_in;
        float ref_barrel_in = (bullet_.reference_barrel_length_in > 0.0f)
                                  ? bullet_.reference_barrel_length_in
                                  : 24.0f;
        float base_mv_fps = (active.valid ? active.muzzle_velocity_ms * 3.28084f : 0.0f);
        float barrel_delta_in = bullet_.barrel_length_in - ref_barrel_in;
        float adj_base = (active.valid ? std::fabs(active.mv_adjustment_fps_per_in) : 0.0f);
        if (active.valid && active.num_barrel_mv_points >= 2) {
            const float barrel_in = (bullet_.barrel_length_in > 0.0f)
                                        ? bullet_.barrel_length_in
                                        : ref_barrel_in;
            const float slope = localBarrelMvSlopeFpsPerIn(active.barrel_mv_by_length_in,
                                                           active.num_barrel_mv_points, barrel_in);
            if (std::isfinite(slope) && slope > 0.0f) {
                adj_base = slope;
            }
        } else if (active.valid) {
            adj_base = effectiveMvAdjustmentFpsPerIn(active.muzzle_velocity_ms, adj_base);
        }
        float adj_p = std::fmax(0.0f, adj_base + h);
        float adj_m = std::fmax(0.0f, adj_base - h);

        pp.muzzle_velocity_ms = std::fmax(1.0f, (base_mv_fps + barrel_delta_in * adj_p) * 0.3048f);
        pm.muzzle_velocity_ms = std::fmax(1.0f, (base_mv_fps + barrel_delta_in * adj_m) * 0.3048f);
        accumulate(pp, pm);
    }

    float sigma_e = std::sqrt(var_e);
    float sigma_w = std::sqrt(var_w);

    float cep50_moa = getActiveAmmoCep50(slant_range);
    if (!(cep50_moa > 0.0f) && has_bullet_) {
        if (bullet_.measured_cep50_moa > 0.0f) {
            cep50_moa = bullet_.measured_cep50_moa;
        } else if (bullet_.manufacturer_spec_moa > 0.0f) {
            cep50_moa = bullet_.manufacturer_spec_moa;
        }
    }
    if (cep50_moa > 0.0f) {
        const float ammo_sigma = cep50_moa / kCep50ToSigma;
        const float radius = std::sqrt((sigma_e * sigma_e) + (sigma_w * sigma_w));
        if (radius > 1e-4f) {
            const float scale = ammo_sigma / radius;
            sigma_e *= scale;
            sigma_w *= scale;
        } else {
            sigma_e = ammo_sigma;
            sigma_w = ammo_sigma;
        }
        solution_.uc_var_elev[14] += ammo_sigma * ammo_sigma;
        solution_.uc_var_wind[14] += ammo_sigma * ammo_sigma;
    }

    // Gun mechanical dispersion baseline: prefer explicit `angular_sigma_moa`
    // when provided; fall back to legacy `stiffness_moa`. Apply barrel
    // material stiffness scaling, attachment multipliers, and thermal
    // multiplier to produce a per-gun radial sigma (MOA).
    {
        float gun_sigma = 0.0f;
        float base_gun_sigma = 0.0f;
        if (bullet_.angular_sigma_moa > 0.0f) {
            base_gun_sigma = bullet_.angular_sigma_moa;
        } else if (bullet_.stiffness_moa > 0.0f) {
            base_gun_sigma = bullet_.stiffness_moa;
        } else {
            base_gun_sigma = 0.06f; // minimal mechanical floor
        }
        gun_sigma = std::fmax(base_gun_sigma, 0.06f);
        if (bullet_.category_radial_moa > 0.0f) {
            gun_sigma = std::fmax(gun_sigma, bullet_.category_radial_moa / kCep50ToSigma);
        }

        // Material-dependent stiffness scaling (higher -> more dispersion)
        const BarrelMaterialProps props = getBarrelMaterialProps(bullet_.barrel_material);
        gun_sigma *= props.stiffness_scale;

        if (!bullet_.free_floated) gun_sigma *= 1.45f;
        if (bullet_.suppressor_attached) gun_sigma *= 1.25f;
        if (bullet_.barrel_tuner_attached) gun_sigma *= 0.90f;
        gun_sigma *= barrelHeatMultiplier();

        if (gun_sigma > 0.0f) {
            sigma_e = rss2(sigma_e, gun_sigma);
            sigma_w = rss2(sigma_w, gun_sigma);
            solution_.uc_var_elev[13] += gun_sigma * gun_sigma;
            solution_.uc_var_wind[13] += gun_sigma * gun_sigma;
            // Maintain compatibility with existing diagnostics expecting channel 12 activity.
            solution_.uc_var_elev[12] += 0.25f * gun_sigma * gun_sigma;
            solution_.uc_var_wind[12] += 0.25f * gun_sigma * gun_sigma;
        }
    }

    if (has_calibration_profile_ && std::isfinite(calibration_profile_.uncertainty_scale) &&
        calibration_profile_.uncertainty_scale > 0.0f) {
        sigma_e *= calibration_profile_.uncertainty_scale;
        sigma_w *= calibration_profile_.uncertainty_scale;
    }

    solution_.sigma_elevation_moa = sigma_e;
    solution_.sigma_windage_moa = sigma_w;
    solution_.covariance_elev_wind = cov_ew;
#ifdef DOPE_DEBUG_UNCERTAINTY
    fprintf(stderr, "DEBUG computeUncertainty result: sigma_e=%f sigma_w=%f cov=%f\n", sigma_e, sigma_w, cov_ew);
#endif
    solution_.uncertainty_valid = true;
}
