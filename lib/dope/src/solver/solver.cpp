/**
 * @file solver.cpp
 * @brief Point-mass ballistic trajectory solver implementation.
 *
 * Uses an adaptive fourth-order Runge-Kutta (RK4) integrator.
 * All state stored in static trajectory table — zero heap allocation.
 *
 * Coordinate system:
 *   X = downrange (horizontal)
 *   Y = vertical (up positive)
 *   Z = lateral (right positive)
 */

#include "solver.h"
#include "../drag/drag_model.h"
#include "dope/dope_math_utils.h"
#include <cmath>
#include <cstring>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
bool SolverProfilingEnabled() {
    static const bool enabled = [] {
        const char* env = std::getenv("DOPE_SOLVER_PROFILE");
        return env && *env;
    }();
    return enabled;
}

FILE* SolverProfileStream() {
    static FILE* stream = [] {
        const char* path = std::getenv("DOPE_SOLVER_PROFILE_FILE");
        if (path && *path) {
            FILE* f = std::fopen(path, "w");
            if (f)
                return f;
        }
        return stderr;
    }();
    return stream;
}

float EstimateDynamicStabilitySG(const SolverParams& params, float velocity_ms) {
    // Miller-style SG estimate in imperial units:
    // SG = (30*m)/(t^2*d^3*l*(1+l^2)) * cbrt(v/2800) * (rho_std/rho)
    // m [gr], d [in], l [calibers], t [calibers/turn], v [fps].
    float sg = 1.5f;
    const float caliber_in = params.caliber_m / dope::math::INCHES_TO_M;
    const float length_in = params.bullet_length_m / dope::math::INCHES_TO_M;
    const float length_cal = (caliber_in > 1e-6f) ? (length_in / caliber_in) : 0.0f;
    const float twist_cal = (caliber_in > 1e-6f) ? (std::fabs(params.twist_rate_inches) / caliber_in) : 0.0f;
    const float mass_grains = params.bullet_mass_kg / dope::math::GRAINS_TO_KG;
    const float vel_fps = velocity_ms * 3.28084f;
    const float density_ratio = (params.air_density > 1e-6f)
        ? (DOPE_STD_AIR_DENSITY / params.air_density)
        : 1.0f;

    if (caliber_in > 0.05f && length_cal > 0.1f && twist_cal > 0.1f && mass_grains > 10.0f && vel_fps > 300.0f) {
        const float denom = (twist_cal * twist_cal)
            * (caliber_in * caliber_in * caliber_in)
            * length_cal
            * (1.0f + length_cal * length_cal);
        if (denom > 1e-6f) {
            float sg_raw = (30.0f * mass_grains / denom)
                * std::cbrt(vel_fps / 2800.0f)
                * density_ratio;
            if (std::isfinite(sg_raw) && sg_raw > 0.0f) {
                sg = sg_raw;
            }
        }
    }

    if (sg < 0.5f) sg = 0.5f;
    if (sg > 3.0f) sg = 3.0f;
    return sg;
}

float StabilityDragScale(float sg) {
    // Coupled-stability drag adjustment.
    // Low SG -> slightly higher drag, high SG -> slightly lower drag.
    // Bounded so this remains a small correction over BC-driven drag.
    float scale = 1.0f + DOPE_SG_DRAG_COUPLING_GAIN * (DOPE_SG_REFERENCE - sg);
    if (scale < DOPE_SG_DRAG_SCALE_MIN) scale = DOPE_SG_DRAG_SCALE_MIN;
    if (scale > DOPE_SG_DRAG_SCALE_MAX) scale = DOPE_SG_DRAG_SCALE_MAX;
    return scale;
}

} // namespace

void BallisticSolver::init() {
    std::memset(table_, 0, sizeof(table_));
    max_valid_range_ = 0;
}

float BallisticSolver::solveZeroAngle(SolverParams params, float zero_range_m) {
    if (zero_range_m < 1.0f || zero_range_m > DOPE_MAX_RANGE_M) {
        return NAN;
    }

    // Binary search for the launch angle that makes the bullet's trajectory
    // intersect the line of sight at the specified zero range.
    //
    // The sight is mounted above the bore, so the line of sight is a straight
    // line from the sight to the target. The barrel must be angled slightly
    // upward for the bullet to follow an arc that intersects this line.

    float lo = -5.0f * dope::math::DEG_TO_RAD;        // -5 degrees (bore pointing down)
    float hi = 5.0f * dope::math::DEG_TO_RAD;         // +5 degrees (bore pointing up)

    float sight_h = params.sight_height_m;

    // The line of sight (LOS) is a straight line from sight to target; with a
    // flat target plane, it crosses the bore axis at the zero range (y = 0).
    // We find the launch angle that yields y_bullet(zero_range_m) = 0.
    (void)sight_h; // sight height retained for compatibility; geometry simplified
    float target_drop = 0.0f;

    float best_angle = 0.0f;
    bool solved = false;

    for (uint32_t i = 0; i < DOPE_ZERO_MAX_ITERATIONS; ++i) {
        float mid = (lo + hi) * 0.5f;
        params.launch_angle_rad = mid;

        float drop = integrateToRange(params, zero_range_m, false);
        if (std::isnan(drop)) {
            // Bullet didn't reach. This can happen if the angle is too low,
            // causing the trajectory to terminate early. We need more angle.
            lo = mid;
            continue;
        }

        // If the drop is greater than the target drop, it means the bullet
        // hit too low. We need to increase the launch angle.
        if (drop > target_drop) {
            hi = mid;
        } else {
            lo = mid;
        }

        best_angle = mid;

        if (std::fabs(drop - target_drop) < DOPE_ZERO_TOLERANCE_M) {
            solved = true;
            break;
        }
    }

    // Final check: if the loop finished without converging, we might still
    // be close.
    if (!solved) {
        params.launch_angle_rad = best_angle;
        float final_drop = integrateToRange(params, zero_range_m, false);
        if (!std::isnan(final_drop) && std::fabs(final_drop - target_drop) < DOPE_ZERO_TOLERANCE_M) {
            solved = true;
        }
    }

    return solved ? best_angle : NAN;
}

SolverResult BallisticSolver::integrate(const SolverParams& params, bool fill_table) {
    SolverResult result;
    std::memset(&result, 0, sizeof(result));
    result.valid = false;

    if (params.target_range_m < 1.0f || params.target_range_m > DOPE_MAX_RANGE_M) {
        return result;
    }

    TrajectoryPoint final_tp = {};

    // Run integration; table fill is optional for perf-sensitive callers. Capture final state.
    float drop = integrateToRange(params, params.target_range_m, fill_table, &final_tp, nullptr);
    if (std::isnan(drop)) {
        return result;
    }

    int target_idx = static_cast<int>(params.target_range_m);
    if (target_idx < 0 || target_idx >= DOPE_TRAJ_TABLE_SIZE) {
        return result;
    }

    const TrajectoryPoint& tp = fill_table ? table_[target_idx] : final_tp;

    result.valid = true;
    result.drop_at_target_m = tp.drop_m;
    result.windage_at_target_m = tp.windage_m;
    result.tof_s = tp.tof_s;
    result.velocity_at_target_ms = tp.velocity_ms;
    result.energy_at_target_j = tp.energy_j;
    result.horizontal_range_m = params.target_range_m * std::cos(params.launch_angle_rad);

    // Compute spin drift. This uses the Litz approximation (drift ∝ TOF^1.83)
    // with a dynamic SG estimate from bullet geometry, mass, twist, velocity,
    // and air density. If required inputs are missing, it falls back to SG=1.5.
    result.spin_drift_moa = 0.0f;
    if (params.spin_drift_enabled && std::fabs(params.twist_rate_inches) > 0.1f) {
        // Litz approximation: drift_inches = 1.25 * (SG + 1.2) * TOF^1.83
        float sg = EstimateDynamicStabilitySG(params, tp.velocity_ms);
        float drift_m = 0.0254f * 1.25f * (sg + 1.2f) * std::pow(tp.tof_s, 1.83f); // [MATH §12]

        // Sign by twist direction: RH twist (positive) drifts right
        if (params.twist_rate_inches < 0.0f) drift_m = -drift_m;

        // Convert to MOA
        float range = params.target_range_m;
        if (range > 0.0f) {
            result.spin_drift_moa = (drift_m / range) * dope::math::RAD_TO_MOA;
        }
    }

    // Compute Coriolis and Eötvös effects. This is a simplified model that
    // assumes a constant velocity and horizontal firing angle, which is not
    // entirely accurate but provides a reasonable approximation for small arms
    // ranges.
    result.coriolis_elev_moa = 0.0f;
    result.coriolis_wind_moa = 0.0f;
    if (params.coriolis_enabled) {
        float lat = params.coriolis_lat_rad;
        float azi = params.azimuth_rad;
        float tof = tp.tof_s;
        float range = params.target_range_m;

        // Horizontal (windage) Coriolis deflection:    [MATH §13]
        // Δz ∝ ω × range × tof × sin(lat) × sin(azi)
        // Maximal for east/west shots, zero for north/south.
        float coriolis_hz = DOPE_OMEGA_EARTH * range * tof * std::sin(lat) * std::sin(azi); // [MATH §13]

        // Vertical (Eötvös) component:    [MATH §13]
        // Δy = ω × range × tof × cos(lat) × sin(azi)
        float coriolis_vt = DOPE_OMEGA_EARTH * range * tof * std::cos(lat) * std::sin(azi); // [MATH §13]

        if (range > 0.0f) {
            result.coriolis_wind_moa = (coriolis_hz / range) * dope::math::RAD_TO_MOA; // [MATH §13]
            result.coriolis_elev_moa = (coriolis_vt / range) * dope::math::RAD_TO_MOA; // [MATH §13]
        }
    }

    return result;
}

const TrajectoryPoint* BallisticSolver::getPointAt(int range_m) const {
    if (range_m < 0 || range_m > max_valid_range_ || range_m >= DOPE_TRAJ_TABLE_SIZE) {
        return nullptr;
    }
    return &table_[range_m];
}

float BallisticSolver::integrateToRange(const SolverParams& params, float range_m, bool fill_table,
                                        TrajectoryPoint* final_point, float* final_t) {
    const float half_bullet_mass_kg = 0.5f * params.bullet_mass_kg;

    const float max_step_m =
        (range_m >= DOPE_FAR_RANGE_THRESHOLD_M) ? DOPE_MAX_STEP_DISTANCE_FAR_M : DOPE_MAX_STEP_DISTANCE_M;

    const bool profile = SolverProfilingEnabled();
    using clock = std::chrono::high_resolution_clock;
    const auto solve_start = profile ? clock::now() : clock::time_point{};
    double time_accel_ms = 0.0;
    double time_table_ms = 0.0;
    float dt_min = std::numeric_limits<float>::max();
    float dt_max = 0.0f;

    // Initial conditions
    float vx = params.muzzle_velocity_ms * std::cos(params.launch_angle_rad);
    float vy = params.muzzle_velocity_ms * std::sin(params.launch_angle_rad);
    float vz = 0.0f; // no initial lateral velocity

    float x = 0.0f;  // downrange
    float y = 0.0f;  // vertical (bore axis is at y=0 at muzzle)
    float z = 0.0f;  // lateral

    float t = 0.0f;  // time of flight

    int last_range_index = 0;
    if (fill_table) {
        table_[0].drop_m = 0.0f;
        table_[0].windage_m = 0.0f;
        table_[0].velocity_ms = params.muzzle_velocity_ms;
        table_[0].tof_s = 0.0f;
        table_[0].energy_j = half_bullet_mass_kg *
                      params.muzzle_velocity_ms * params.muzzle_velocity_ms;
    }

    uint32_t iteration = 0;

    auto computeAcceleration = [&](float vxn, float vyn, float vzn,
                                   float& ax, float& ay, float& az) {
        // Relative velocity components accounting for wind    [MATH §11.2]
        float vx_rel = vxn + params.headwind_ms;  // [MATH §11.2]
        float vz_rel = vzn + params.crosswind_ms; // [MATH §11.2]
        float v_rel = std::sqrt(vx_rel * vx_rel + vyn * vyn + vz_rel * vz_rel); // [MATH §11.2]

        if (v_rel < 1.0f) {
            ax = 0.0f;
            ay = -DOPE_GRAVITY;
            az = 0.0f;
            return;
        }

        // Drag retardation magnitude — see §5.2 for the formula    [MATH §11.3]
        float decel = DragModelLookup::getDeceleration(v_rel, params.speed_of_sound,
                                                        params.bc, params.drag_model,
                                                        params.air_density);
        float drag_scale = params.drag_reference_scale;
        if (!std::isfinite(drag_scale) || drag_scale <= 0.0f) drag_scale = 1.0f;
        if (drag_scale < 0.2f) drag_scale = 0.2f;
        if (drag_scale > 2.0f) drag_scale = 2.0f;

        // Stability-coupled drag: SG modulates effective drag slightly.
        // This keeps BC as the primary drag driver while making geometry
        // (length/caliber/twist/mass) influence trajectory beyond spin drift.
        const float sg = EstimateDynamicStabilitySG(params, v_rel);
        const float stability_drag_scale = StabilityDragScale(sg);
        decel *= (drag_scale * stability_drag_scale);
        ax = -decel * (vx_rel / v_rel);             // [MATH §11.3]
        ay = -decel * (vyn  / v_rel) - DOPE_GRAVITY; // [MATH §11.3] gravity always downward
        az = -decel * (vz_rel / v_rel);             // [MATH §11.3]
    };

    auto log_profile = [&](float drop_value) {
        if (!profile)
            return;
        const auto solve_end = clock::now();
        const double total_ms = std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
        const float dt_min_out = (dt_min == std::numeric_limits<float>::max()) ? 0.0f : dt_min;
        FILE* out = SolverProfileStream();
        std::fprintf(out,
                 "SOLVER_PROFILE range_m=%.1f iter=%u dt_min=%.6f dt_max=%.6f total_ms=%.3f accel_ms=%.3f table_ms=%.3f fill_table=%d drop=%.4f\n",
                 range_m, iteration, dt_min_out, dt_max, total_ms, time_accel_ms, time_table_ms,
                 fill_table ? 1 : 0, drop_value);
        std::fflush(out);
    };

    while (x < range_m && iteration < DOPE_MAX_SOLVER_ITERATIONS) {
        iteration++;

        // Preserve previous state for interpolation.
        float prev_x = x;
        float prev_y = y;
        float prev_z = z;
        float prev_vx = vx;
        float prev_vy = vy;
        float prev_vz = vz;
        float prev_t = t;

        float v = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (v < DOPE_MIN_VELOCITY) break;

        // Adaptive timestep: smaller near transonic, larger at supersonic.    [MATH §11.4]
        // The constant 0.5 is a tuning parameter that balances performance
        // and stability. A smaller value would increase accuracy but slow
        // down the simulation.
        float mach = v / params.speed_of_sound; // [MATH §11.4]
        float dt;
        if (mach > 0.9f && mach < 1.2f) {
            dt = DOPE_DT_MIN; // [MATH §11.4] transonic region — use smallest step
        } else {
            // Scale dt with velocity — faster bullet covers more ground per step
            dt = 0.5f / v; // [MATH §11.4]
        }

        if (profile) {
            dt_min = std::min(dt_min, dt);
            dt_max = std::max(dt_max, dt);
        }

        // Bound per-step downrange travel for stability and table fidelity    [MATH §11.4]
        float dt_from_step = max_step_m / v; // [MATH §11.4]
        if (dt > dt_from_step) dt = dt_from_step;
        if (dt < DOPE_DT_MIN) dt = DOPE_DT_MIN;
        if (dt > DOPE_DT_MAX) dt = DOPE_DT_MAX;

        // --- RK4 integration ---
        float ax1, ay1, az1;
        const auto accel_start = profile ? clock::now() : clock::time_point{};
        computeAcceleration(vx, vy, vz, ax1, ay1, az1);

        float k1_vx = ax1;
        float k1_vy = ay1;
        float k1_vz = az1;
        float k1_x = vx;
        float k1_y = vy;
        float k1_z = vz;

        float vx_k2 = vx + 0.5f * dt * k1_vx;
        float vy_k2 = vy + 0.5f * dt * k1_vy;
        float vz_k2 = vz + 0.5f * dt * k1_vz;
        float ax2, ay2, az2;
        computeAcceleration(vx_k2, vy_k2, vz_k2, ax2, ay2, az2);

        float k2_vx = ax2;
        float k2_vy = ay2;
        float k2_vz = az2;
        float k2_x = vx_k2;
        float k2_y = vy_k2;
        float k2_z = vz_k2;

        float vx_k3 = vx + 0.5f * dt * k2_vx;
        float vy_k3 = vy + 0.5f * dt * k2_vy;
        float vz_k3 = vz + 0.5f * dt * k2_vz;
        float ax3, ay3, az3;
        computeAcceleration(vx_k3, vy_k3, vz_k3, ax3, ay3, az3);

        float k3_vx = ax3;
        float k3_vy = ay3;
        float k3_vz = az3;
        float k3_x = vx_k3;
        float k3_y = vy_k3;
        float k3_z = vz_k3;

        float vx_k4 = vx + dt * k3_vx;
        float vy_k4 = vy + dt * k3_vy;
        float vz_k4 = vz + dt * k3_vz;
        float ax4, ay4, az4;
        computeAcceleration(vx_k4, vy_k4, vz_k4, ax4, ay4, az4);

        if (profile) {
            const auto accel_end = clock::now();
            time_accel_ms += std::chrono::duration<double, std::milli>(accel_end - accel_start).count();
        }

        float k4_vx = ax4;
        float k4_vy = ay4;
        float k4_vz = az4;
        float k4_x = vx_k4;
        float k4_y = vy_k4;
        float k4_z = vz_k4;

        // RK4 integration step: pos += (dt/6)(k1 + 2k2 + 2k3 + k4)    [MATH §11.5]
        auto rk4_step = [&](float& pos, float& vel, float k1_p, float k1_v, float k2_p, float k2_v, float k3_p, float k3_v, float k4_p, float k4_v) {
            pos += (dt / 6.0f) * (k1_p + 2.0f * k2_p + 2.0f * k3_p + k4_p); // [MATH §11.5]
            vel += (dt / 6.0f) * (k1_v + 2.0f * k2_v + 2.0f * k3_v + k4_v); // [MATH §11.5]
        };

        rk4_step(x, vx, k1_x, k1_vx, k2_x, k2_vx, k3_x, k3_vx, k4_x, k4_vx);
        rk4_step(y, vy, k1_y, k1_vy, k2_y, k2_vy, k3_y, k3_vy, k4_y, k4_vy);
        rk4_step(z, vz, k1_z, k1_vz, k2_z, k2_vz, k3_z, k3_vz, k4_z, k4_vz);
        t += dt;

        // Fill trajectory table at each meter mark using linear interpolation
        // between the previous and current step states for better fidelity.
        if (fill_table) {
            const auto table_start = profile ? clock::now() : clock::time_point{};
            int current_range = static_cast<int>(x);
            float curr_speed = std::sqrt(vx * vx + vy * vy + vz * vz);
            float prev_speed = std::sqrt(prev_vx * prev_vx + prev_vy * prev_vy + prev_vz * prev_vz);
            while (last_range_index < current_range &&
                   last_range_index < DOPE_TRAJ_TABLE_SIZE - 1) {
                int next_idx = last_range_index + 1;
                float denom = (x - prev_x);
                float alpha = (denom > 1e-6f) ? ((static_cast<float>(next_idx) - prev_x) / denom) : 0.0f;
                if (alpha < 0.0f) alpha = 0.0f;
                if (alpha > 1.0f) alpha = 1.0f;

                float y_i = prev_y + alpha * (y - prev_y);
                float z_i = prev_z + alpha * (z - prev_z);
                float v_i = prev_speed + alpha * (curr_speed - prev_speed);
                float t_i = prev_t + alpha * (t - prev_t);

                table_[next_idx].drop_m = y_i;
                table_[next_idx].windage_m = z_i;
                table_[next_idx].velocity_ms = v_i;
                table_[next_idx].tof_s = t_i;
                table_[next_idx].energy_j = half_bullet_mass_kg * v_i * v_i;

                last_range_index = next_idx;
            }
            max_valid_range_ = last_range_index;

            if (profile) {
                const auto table_end = clock::now();
                time_table_ms += std::chrono::duration<double, std::milli>(table_end - table_start).count();
            }
        }

        // If we've passed the requested range, interpolate to that exact distance
        // for the returned drop value. Also capture final state if requested.
        if (x >= range_m) {
            float denom = (x - prev_x);
            float alpha = (denom > 1e-6f) ? ((range_m - prev_x) / denom) : 0.0f;
            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;
            float y_target = prev_y + alpha * (y - prev_y);

            if (final_point) {
                const float z_target = prev_z + alpha * (z - prev_z);
                const float vx_target = prev_vx + alpha * (vx - prev_vx);
                const float vy_target = prev_vy + alpha * (vy - prev_vy);
                const float vz_target = prev_vz + alpha * (vz - prev_vz);
                const float t_target = prev_t + alpha * (t - prev_t);
                const float v_target = std::sqrt(vx_target * vx_target + vy_target * vy_target + vz_target * vz_target);

                final_point->drop_m = y_target;
                final_point->windage_m = z_target;
                final_point->velocity_ms = v_target;
                final_point->tof_s = t_target;
                final_point->energy_j = half_bullet_mass_kg * v_target * v_target;
            }
            if (final_t) {
                *final_t = prev_t + alpha * (t - prev_t);
            }

            log_profile(y_target);
            return y_target;
        }
    }

    if (x < range_m) {
        if (final_point) {
            final_point->drop_m = y;
            final_point->windage_m = z;
            float v_here = std::sqrt(vx * vx + vy * vy + vz * vz);
            final_point->velocity_ms = v_here;
            final_point->tof_s = t;
            final_point->energy_j = half_bullet_mass_kg * v_here * v_here;
        }
        if (final_t) {
            *final_t = t;
        }
        log_profile(NAN);
        return NAN; // bullet didn't reach target range
    }

    if (final_point) {
        float v_here = std::sqrt(vx * vx + vy * vy + vz * vz);
        final_point->drop_m = y;
        final_point->windage_m = z;
        final_point->velocity_ms = v_here;
        final_point->tof_s = t;
        final_point->energy_j = half_bullet_mass_kg * v_here * v_here;
    }
    if (final_t) {
        *final_t = t;
    }

    log_profile(y);
    return y; // vertical drop at target range (exact match case)
}
