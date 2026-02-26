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
#include <cmath>
#include <cstring>

void BallisticSolver::init() {
    std::memset(table_, 0, sizeof(table_));
    max_valid_range_ = 0;
}

float BallisticSolver::solveZeroAngle(SolverParams params, float zero_range_m) {
    if (zero_range_m < 1.0f || zero_range_m > BCE_MAX_RANGE_M) {
        return NAN;
    }

    // Binary search for the launch angle that makes the bullet's trajectory
    // intersect the line of sight at the specified zero range.
    //
    // The sight is mounted above the bore, so the line of sight is a straight
    // line from the sight to the target. The barrel must be angled slightly
    // upward for the bullet to follow an arc that intersects this line.

    float lo = -5.0f * BCE_DEG_TO_RAD;        // -5 degrees (bore pointing down)
    float hi = 5.0f * BCE_DEG_TO_RAD;         // +5 degrees (bore pointing up)

    float sight_h = params.sight_height_m;

    // The line of sight (LOS) is a straight line. At range x, its height
    // relative to the bore axis is:
    //   y_los(x) = sight_h - (sight_h / zero_range_m) * x
    // This is a simplification assuming the target is at the same height as the
    // muzzle. The LOS starts at sight_h and slopes down to 0 at zero_range_m.
    //
    // We need to find the launch angle where the bullet's height y_bullet(x)
    // equals y_los(x) at x = zero_range_m.
    //
    // At zero_range_m, the LOS height is 0 relative to the target, but since
    // the sight is sight_h above the bore, the bullet must have a height of
    // -sight_h relative to the bore line to hit the target.
    //
    // So, we are looking for the launch angle where:
    //   drop_at_zero_range = -sight_height_m

    float target_drop = -sight_h;

    float best_angle = 0.0f;
    bool solved = false;

    for (uint32_t i = 0; i < BCE_ZERO_MAX_ITERATIONS; ++i) {
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

        if (std::fabs(drop - target_drop) < BCE_ZERO_TOLERANCE_M) {
            solved = true;
            break;
        }
    }

    // Final check: if the loop finished without converging, we might still
    // be close.
    if (!solved && std::fabs(integrateToRange(params, zero_range_m, false) - target_drop) < BCE_ZERO_TOLERANCE_M) {
        solved = true;
    }

    return solved ? best_angle : NAN;
}

SolverResult BallisticSolver::integrate(const SolverParams& params) {
    SolverResult result;
    std::memset(&result, 0, sizeof(result));
    result.valid = false;

    if (params.target_range_m < 1.0f || params.target_range_m > BCE_MAX_RANGE_M) {
        return result;
    }

    // Run full integration, filling the trajectory table
    float drop = integrateToRange(params, params.target_range_m, true);
    if (std::isnan(drop)) {
        return result;
    }

    int target_idx = static_cast<int>(params.target_range_m);
    if (target_idx < 0 || target_idx >= BCE_TRAJ_TABLE_SIZE) {
        return result;
    }

    const TrajectoryPoint& tp = table_[target_idx];

    result.valid = true;
    result.drop_at_target_m = tp.drop_m;
    result.windage_at_target_m = tp.windage_m;
    result.tof_s = tp.tof_s;
    result.velocity_at_target_ms = tp.velocity_ms;
    result.energy_at_target_j = tp.energy_j;
    result.horizontal_range_m = params.target_range_m * std::cos(params.launch_angle_rad);

    // Compute spin drift. This is a simplified model based on the Litz
    // approximation (drift ∝ TOF^1.83). The gyroscopic stability factor (SG)
    // is estimated at 1.5, which is a reasonable average but not precise for
    // all projectiles. A more rigorous model would calculate SG from bullet
    // geometry, twist rate, and velocity.
    result.spin_drift_moa = 0.0f;
    if (params.spin_drift_enabled && std::fabs(params.twist_rate_inches) > 0.1f) {
        // Litz approximation: drift ∝ TOF^1.83
        // SG (stability factor) approximation is embedded in the constant
        // drift_inches = 1.25 * (SG + 1.2) * TOF^1.83
        // Simplified: we use a caliber-and-twist dependent scaling
        float sg = 1.5f; // simplified stability factor estimate
        float drift_m = 0.0254f * 1.25f * (sg + 1.2f) * std::pow(tp.tof_s, 1.83f);

        // Sign by twist direction: RH twist (positive) drifts right
        if (params.twist_rate_inches < 0.0f) drift_m = -drift_m;

        // Convert to MOA
        float range = params.target_range_m;
        if (range > 0.0f) {
            result.spin_drift_moa = (drift_m / range) * BCE_RAD_TO_MOA;
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

        // Horizontal (windage) Coriolis deflection:
        // deflection = ω × v × tof × sin(lat)  (simplified)
        // More precisely for a bullet:
        // Δz = ω × range × tof × sin(lat)  (horizontal)
        float coriolis_hz = BCE_OMEGA_EARTH * range * tof * std::sin(lat);

        // Vertical (Eötvös) component:
        // Δy = ω × range × tof × cos(lat) × sin(azi)
        float coriolis_vt = BCE_OMEGA_EARTH * range * tof * std::cos(lat) * std::sin(azi);

        if (range > 0.0f) {
            result.coriolis_wind_moa = (coriolis_hz / range) * BCE_RAD_TO_MOA;
            result.coriolis_elev_moa = (coriolis_vt / range) * BCE_RAD_TO_MOA;
        }
    }

    return result;
}

const TrajectoryPoint* BallisticSolver::getPointAt(int range_m) const {
    if (range_m < 0 || range_m > max_valid_range_ || range_m >= BCE_TRAJ_TABLE_SIZE) {
        return nullptr;
    }
    return &table_[range_m];
}

float BallisticSolver::integrateToRange(const SolverParams& params, float range_m, bool fill_table) {
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
        table_[0].energy_j = 0.5f * params.bullet_mass_kg *
                              params.muzzle_velocity_ms * params.muzzle_velocity_ms;
    }

    uint32_t iteration = 0;

    auto computeAcceleration = [&](float vxn, float vyn, float vzn,
                                   float& ax, float& ay, float& az) {
        float vx_rel = vxn + params.headwind_ms;
        float vz_rel = vzn - params.crosswind_ms;
        float v_rel = std::sqrt(vx_rel * vx_rel + vyn * vyn + vz_rel * vz_rel);

        if (v_rel < 1.0f) {
            ax = 0.0f;
            ay = -BCE_GRAVITY;
            az = 0.0f;
            return;
        }

        float decel = DragModelLookup::getDeceleration(v_rel, params.speed_of_sound,
                                                        params.bc, params.drag_model,
                                                        params.air_density);
        float drag_scale = params.drag_reference_scale;
        if (!std::isfinite(drag_scale) || drag_scale <= 0.0f) drag_scale = 1.0f;
        if (drag_scale < 0.2f) drag_scale = 0.2f;
        if (drag_scale > 2.0f) drag_scale = 2.0f;
        decel *= drag_scale;
        ax = -decel * (vx_rel / v_rel);
        ay = -decel * (vyn / v_rel) - BCE_GRAVITY;
        az = -decel * (vz_rel / v_rel);
    };

    while (x < range_m && iteration < BCE_MAX_SOLVER_ITERATIONS) {
        iteration++;

        float v = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (v < BCE_MIN_VELOCITY) break;

        // Adaptive timestep: smaller near transonic, larger at supersonic.
        // The constant 0.5 is a tuning parameter that balances performance
        // and stability. A smaller value would increase accuracy but slow
        // down the simulation.
        float mach = v / params.speed_of_sound;
        float dt;
        if (mach > 0.9f && mach < 1.2f) {
            dt = BCE_DT_MIN; // transonic region — use smallest step
        } else {
            // Scale dt with velocity — faster bullet covers more ground per step
            dt = 0.5f / v;
        }

        // Bound per-step downrange travel for stability and table fidelity
        float dt_from_step = BCE_MAX_STEP_DISTANCE_M / v;
        if (dt > dt_from_step) dt = dt_from_step;
        if (dt < BCE_DT_MIN) dt = BCE_DT_MIN;
        if (dt > BCE_DT_MAX) dt = BCE_DT_MAX;

        // --- RK4 integration ---
        float ax1, ay1, az1;
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

        float k4_vx = ax4;
        float k4_vy = ay4;
        float k4_vz = az4;
        float k4_x = vx_k4;
        float k4_y = vy_k4;
        float k4_z = vz_k4;

        // RK4 integration step
        auto rk4_step = [&](float& pos, float& vel, float k1_p, float k1_v, float k2_p, float k2_v, float k3_p, float k3_v, float k4_p, float k4_v) {
            pos += (dt / 6.0f) * (k1_p + 2.0f * k2_p + 2.0f * k3_p + k4_p);
            vel += (dt / 6.0f) * (k1_v + 2.0f * k2_v + 2.0f * k3_v + k4_v);
        };

        rk4_step(x, vx, k1_x, k1_vx, k2_x, k2_vx, k3_x, k3_vx, k4_x, k4_vx);
        rk4_step(y, vy, k1_y, k1_vy, k2_y, k2_vy, k3_y, k3_vy, k4_y, k4_vy);
        rk4_step(z, vz, k1_z, k1_vz, k2_z, k2_vz, k3_z, k3_vz, k4_z, k4_vz);
        t += dt;

        // Fill trajectory table at each meter mark
        if (fill_table) {
            int current_range = static_cast<int>(x);
            while (last_range_index < current_range &&
                   last_range_index < BCE_TRAJ_TABLE_SIZE - 1) {
                last_range_index++;
                float v_current = std::sqrt(vx * vx + vy * vy + vz * vz);
                table_[last_range_index].drop_m = y;
                table_[last_range_index].windage_m = z;
                table_[last_range_index].velocity_ms = v_current;
                table_[last_range_index].tof_s = t;
                table_[last_range_index].energy_j = 0.5f * params.bullet_mass_kg *
                                                     v_current * v_current;
            }
            max_valid_range_ = last_range_index;
        }
    }

    if (x < range_m) {
        return NAN; // bullet didn't reach target range
    }

    return y; // vertical drop at target range
}
