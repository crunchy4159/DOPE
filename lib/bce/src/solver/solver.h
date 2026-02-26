/**
 * @file solver.h
 * @brief Point-mass ballistic trajectory solver.
 *
 * BCE SRS v1.3 — Section 11.1
 *
 * Integrates the point-mass equations of motion with adaptive timestep
 * fourth-order Runge-Kutta (RK4). Produces a 1-meter resolution trajectory table
 * stored in static memory. Also solves for the zero angle via binary search.
 */

#pragma once

#include "bce/bce_config.h"
#include "bce/bce_types.h"

/**
 * Per-meter trajectory record stored in the static table.
 */
struct TrajectoryPoint {
    float drop_m;           // Vertical drop from bore line (m, negative = below)
    float windage_m;        // Lateral deflection (m, positive = right)
    float velocity_ms;      // Velocity at this range
    float tof_s;            // Time of flight to this range (seconds)
    float energy_j;         // Kinetic energy at this range (joules)
};

/**
 * Solver input parameters — everything needed for one trajectory solution.
 */
struct SolverParams {
    float bc;                    // Ballistic coefficient (already atmosphere-corrected)
    DragModel drag_model;        // G1–G8
    float muzzle_velocity_ms;    // m/s
    float bullet_mass_kg;        // kg (converted from grains by caller)
    float sight_height_m;        // meters above bore axis

    float air_density;           // kg/m³
    float speed_of_sound;        // m/s
    float drag_reference_scale;  // 1.0 = legacy baseline, <1.0 reduces drag

    // Launch angle (radians above horizontal) — set by zero solver or manual
    float launch_angle_rad;

    // Target range
    float target_range_m;

    // Wind components (already decomposed into bore-axis frame)
    float headwind_ms;           // positive = into shooter
    float crosswind_ms;          // positive = right-to-left

    // Coriolis parameters (set to 0 if disabled)
    float coriolis_lat_rad;      // shooter latitude in radians
    float azimuth_rad;           // firing azimuth in radians
    bool  coriolis_enabled;

    // Spin drift
    float twist_rate_inches;     // signed: positive = RH
    float caliber_m;             // bullet caliber in meters
    bool  spin_drift_enabled;
};

/**
 * Result from a single trajectory integration.
 */
struct SolverResult {
    bool   valid;
    float  drop_at_target_m;
    float  windage_at_target_m;
    float  tof_s;
    float  velocity_at_target_ms;
    float  energy_at_target_j;
    float  horizontal_range_m;

    // Correction components in MOA
    float  coriolis_elev_moa;
    float  coriolis_wind_moa;
    float  spin_drift_moa;
};

class BallisticSolver {
public:
    void init();

    /**
     * Solve the zero angle for the given parameters.
     * This finds the launch angle that results in zero drop at zero_range_m
     * accounting for sight_height_m.
     *
     * @param params  Solver parameters (launch_angle_rad will be ignored/overwritten)
     * @param zero_range_m  Range to zero at (meters)
     * @return Zero angle in radians, or NAN if unsolvable
     */
    float solveZeroAngle(SolverParams params, float zero_range_m);

    /**
     * Integrate a full trajectory with the given parameters.
     * Populates the internal trajectory table and returns the result at target range.
     *
     * @param params  Complete solver parameters including launch_angle_rad
     * @return SolverResult at the target range
     */
    SolverResult integrate(const SolverParams& params);

    /**
     * Get a trajectory point at a specific range (meters).
     * Only valid after integrate() has been called.
     * @param range_m  Range in meters (0 to BCE_MAX_RANGE_M)
     * @return Pointer to trajectory point, or nullptr if out of range
     */
    const TrajectoryPoint* getPointAt(int range_m) const;

private:
    TrajectoryPoint table_[BCE_TRAJ_TABLE_SIZE];
    int max_valid_range_ = 0;

    /**
     * Run the RK4 integration.
     * Returns drop at specified range_m, or NAN if bullet didn't reach.
     */
    float integrateToRange(const SolverParams& params, float range_m, bool fill_table);
};
