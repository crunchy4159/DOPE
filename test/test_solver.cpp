/**
 * @file test_solver.cpp
 * @brief Unit tests for the ballistic solver.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/solver/solver.h"
#include "bce/bce_config.h"
#include <cmath>

class SolverTest : public ::testing::Test {
protected:
    BallisticSolver solver;

    void SetUp() override {
        solver.init();
    }

    // Helper: create typical .308 175gr SMK parameters
    SolverParams make308Params(float range_m) {
        SolverParams p = {};
        p.bc = 0.505f;                              // G1 BC
        p.drag_model = DragModel::G1;
        p.muzzle_velocity_ms = 792.0f;              // ~2600 fps
        p.bullet_mass_kg = 175.0f * BCE_GRAINS_TO_KG;
        p.sight_height_m = 0.0381f;                 // 1.5 inches
        p.air_density = BCE_STD_AIR_DENSITY;
        p.speed_of_sound = BCE_SPEED_OF_SOUND_15C;
        p.target_range_m = range_m;
        p.launch_angle_rad = 0.0f;
        p.headwind_ms = 0.0f;
        p.crosswind_ms = 0.0f;
        p.coriolis_enabled = false;
        p.spin_drift_enabled = false;
        return p;
    }
};

// Zero angle solver should find a small positive angle
TEST_F(SolverTest, ZeroAngleSmallPositive) {
    SolverParams p = make308Params(100.0f);
    float angle = solver.solveZeroAngle(p, 100.0f);
    EXPECT_FALSE(std::isnan(angle));
    EXPECT_GT(angle, 0.0f);
    EXPECT_LT(angle, 1.0f * BCE_DEG_TO_RAD); // less than 1 degree for 100m
}

// Zero angle at 200m should be larger than at 100m
TEST_F(SolverTest, ZeroAngleIncreasesWithRange) {
    SolverParams p = make308Params(100.0f);
    float angle_100 = solver.solveZeroAngle(p, 100.0f);
    float angle_200 = solver.solveZeroAngle(p, 200.0f);
    EXPECT_GT(angle_200, angle_100);
}

// Trajectory should show bullet dropping below bore line at long range
TEST_F(SolverTest, BulletDropsAtLongRange) {
    SolverParams p = make308Params(1000.0f);
    p.launch_angle_rad = 0.005f; // small launch angle

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_LT(result.drop_at_target_m, 0.0f); // negative = below bore line
}

// TOF should increase with range
TEST_F(SolverTest, TOFIncreasesWithRange) {
    SolverParams p500 = make308Params(500.0f);
    p500.launch_angle_rad = 0.005f;
    SolverResult r500 = solver.integrate(p500);

    SolverParams p1000 = make308Params(1000.0f);
    p1000.launch_angle_rad = 0.005f;
    SolverResult r1000 = solver.integrate(p1000);

    EXPECT_TRUE(r500.valid);
    EXPECT_TRUE(r1000.valid);
    EXPECT_GT(r1000.tof_s, r500.tof_s);
}

// Velocity should decrease with range
TEST_F(SolverTest, VelocityDecreasesWithRange) {
    SolverParams p = make308Params(1000.0f);
    p.launch_angle_rad = 0.005f;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_LT(result.velocity_at_target_ms, p.muzzle_velocity_ms);
    EXPECT_GT(result.velocity_at_target_ms, 0.0f);
}

// Energy should be positive and less than muzzle energy
TEST_F(SolverTest, EnergyDecreasesWithRange) {
    SolverParams p = make308Params(500.0f);
    p.launch_angle_rad = 0.003f;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);

    float muzzle_energy = 0.5f * p.bullet_mass_kg * p.muzzle_velocity_ms * p.muzzle_velocity_ms;
    EXPECT_LT(result.energy_at_target_j, muzzle_energy);
    EXPECT_GT(result.energy_at_target_j, 0.0f);
}

// .308 175gr at 1000m — rough sanity check against published data
// Expected: ~350 m/s remaining velocity, ~1.5s TOF
TEST_F(SolverTest, Sanity308At1000m) {
    SolverParams p = make308Params(1000.0f);

    // First solve the zero at 100m
    float zero_angle = solver.solveZeroAngle(p, 100.0f);
    EXPECT_FALSE(std::isnan(zero_angle));

    p.target_range_m = 1000.0f;
    p.launch_angle_rad = zero_angle;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);

    // Velocity at 1000m for .308 175gr should be roughly 300–400 m/s
    EXPECT_GT(result.velocity_at_target_ms, 200.0f);
    EXPECT_LT(result.velocity_at_target_ms, 500.0f);

    // TOF should be roughly 1.3–2.0 seconds
    EXPECT_GT(result.tof_s, 1.0f);
    EXPECT_LT(result.tof_s, 2.5f);
}

// Trajectory table should be populated after integrate
TEST_F(SolverTest, TrajectoryTablePopulated) {
    SolverParams p = make308Params(500.0f);
    p.launch_angle_rad = 0.003f;

    solver.integrate(p);

    const TrajectoryPoint* pt100 = solver.getPointAt(100);
    ASSERT_NE(pt100, nullptr);
    EXPECT_GT(pt100->velocity_ms, 0.0f);
    EXPECT_GT(pt100->tof_s, 0.0f);

    const TrajectoryPoint* pt500 = solver.getPointAt(500);
    ASSERT_NE(pt500, nullptr);
    EXPECT_LT(pt500->velocity_ms, pt100->velocity_ms);
}

// Out-of-range trajectory table access should return nullptr
TEST_F(SolverTest, TrajectoryTableOutOfRange) {
    SolverParams p = make308Params(100.0f);
    p.launch_angle_rad = 0.001f;
    solver.integrate(p);

    const TrajectoryPoint* pt = solver.getPointAt(5000);
    EXPECT_EQ(pt, nullptr);
}

// Crosswind should produce lateral deflection
TEST_F(SolverTest, CrosswindProducesWindage) {
    SolverParams p = make308Params(500.0f);
    p.launch_angle_rad = 0.003f;
    p.crosswind_ms = 5.0f; // 5 m/s crosswind

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_NE(result.windage_at_target_m, 0.0f);
}

// Invalid zero ranges should return NAN
TEST_F(SolverTest, ZeroAngleRejectsInvalidRange) {
    SolverParams p = make308Params(100.0f);
    EXPECT_TRUE(std::isnan(solver.solveZeroAngle(p, 0.0f)));
    EXPECT_TRUE(std::isnan(solver.solveZeroAngle(p, BCE_MAX_RANGE_M + 1.0f)));
}

// If projectile cannot reach zero range, solveZeroAngle should return NAN
TEST_F(SolverTest, ZeroAngleReturnsNaNWhenUnsolvable) {
    SolverParams p = make308Params(100.0f);
    p.muzzle_velocity_ms = 10.0f; // below solver minimum velocity threshold

    float angle = solver.solveZeroAngle(p, 100.0f);
    EXPECT_TRUE(std::isnan(angle));
}

// Test that spin drift produces rightward drift for RH twist
TEST_F(SolverTest, SpinDriftRightHandTwist) {
    SolverParams p = make308Params(1000.0f);
    p.launch_angle_rad = 0.005f;
    p.spin_drift_enabled = true;
    p.twist_rate_inches = 10.0f; // RH twist

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_GT(result.spin_drift_moa, 0.0f);
}

// Test that spin drift produces leftward drift for LH twist
TEST_F(SolverTest, SpinDriftLeftHandTwist) {
    SolverParams p = make308Params(1000.0f);
    p.launch_angle_rad = 0.005f;
    p.spin_drift_enabled = true;
    p.twist_rate_inches = -10.0f; // LH twist

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_LT(result.spin_drift_moa, -0.0f);
}

// Test Coriolis effect in Northern Hemisphere
TEST_F(SolverTest, CoriolisNorthernHemisphere) {
    SolverParams p = make308Params(1000.0f);
    p.launch_angle_rad = 0.005f;
    p.coriolis_enabled = true;
    p.coriolis_lat_rad = 45.0f * BCE_DEG_TO_RAD; // 45 deg N
    p.azimuth_rad = 90.0f * BCE_DEG_TO_RAD;      // Firing East

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_GT(result.coriolis_wind_moa, 0.0f); // Rightward drift
    EXPECT_GT(result.coriolis_elev_moa, 0.0f); // Upward drift (Eötvös)
}

// Test Coriolis effect in Southern Hemisphere
TEST_F(SolverTest, CoriolisSouthernHemisphere) {
    SolverParams p = make308Params(1000.0f);
    p.launch_angle_rad = 0.005f;
    p.coriolis_enabled = true;
    p.coriolis_lat_rad = -45.0f * BCE_DEG_TO_RAD; // 45 deg S
    p.azimuth_rad = 90.0f * BCE_DEG_TO_RAD;       // Firing East

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    EXPECT_LT(result.coriolis_wind_moa, 0.0f); // Leftward drift
    EXPECT_GT(result.coriolis_elev_moa, 0.0f); // Upward drift (Eötvös)
}

// Test that G7 drag model produces less drop than G1
TEST_F(SolverTest, G7vsG1Drag) {
    SolverParams p_g1 = make308Params(1000.0f);
    p_g1.launch_angle_rad = 0.005f;
    p_g1.drag_model = DragModel::G1;
    p_g1.bc = 0.505f;

    SolverParams p_g7 = make308Params(1000.0f);
    p_g7.launch_angle_rad = 0.005f;
    p_g7.drag_model = DragModel::G7;
    p_g7.bc = 0.257f; // Equivalent G7 BC for a 175gr SMK

    SolverResult r_g1 = solver.integrate(p_g1);
    SolverResult r_g7 = solver.integrate(p_g7);

    EXPECT_TRUE(r_g1.valid);
    EXPECT_TRUE(r_g7.valid);
    EXPECT_GT(r_g1.drop_at_target_m, r_g7.drop_at_target_m); // G1 drops more
}

// Test that higher air density increases drop
TEST_F(SolverTest, AirDensityEffect) {
    SolverParams p_low_density = make308Params(1000.0f);
    p_low_density.launch_angle_rad = 0.005f;
    p_low_density.air_density = 1.1f; // Low density

    SolverParams p_high_density = make308Params(1000.0f);
    p_high_density.launch_angle_rad = 0.005f;
    p_high_density.air_density = 1.3f; // High density

    SolverResult r_low = solver.integrate(p_low_density);
    SolverResult r_high = solver.integrate(p_high_density);

    EXPECT_TRUE(r_low.valid);
    EXPECT_TRUE(r_high.valid);
    EXPECT_LT(r_high.drop_at_target_m, r_low.drop_at_target_m); // More drop in denser air
}

// Test that an unsolvable zero returns NAN
TEST_F(SolverTest, UnsolvableZeroReturnsNAN) {
    SolverParams p = make308Params(100.0f);
    p.muzzle_velocity_ms = 100.0f; // Very low MV, may not reach 100m
    float angle = solver.solveZeroAngle(p, 2000.0f); // Very long range
    EXPECT_TRUE(std::isnan(angle));
}
