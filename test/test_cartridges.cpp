/**
 * @file test_cartridges.cpp
 * @brief Unit tests for popular cartridge configurations.
 *
 * Cartridge definitions in this file are validation references only.
 * They must not be treated as runtime lookup tables for BCE engine logic.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/solver/solver.h"
#include "bce/bce_config.h"
#include "bce/bce_math_utils.h"
#include <cmath>

class CartridgeTest : public ::testing::Test {
protected:
    BallisticSolver solver;

    void SetUp() override {
        solver.init();
    }

    // ref_barrel_in: the barrel length (inches) at which mv_ms was measured.
    // For SAAMI rifle calibers this is typically 24". For 9mm it is 4".
    SolverParams makeCartridgeParams(float range_m, float bc, DragModel model, float mv_ms, float mass_gr, float barrel_in, float mv_adjust, float ref_barrel_in = 24.0f) {
        SolverParams p = {};
        p.bc = bc;
        p.drag_model = model;

        // Adjust MV from reference barrel to actual barrel
        float base_mv_fps = mv_ms * bce::math::MPS_TO_FPS;
        float barrel_length_delta_in = barrel_in - ref_barrel_in;
        float adjusted_mv_fps = base_mv_fps + (barrel_length_delta_in * mv_adjust);
        p.muzzle_velocity_ms = adjusted_mv_fps * bce::math::FPS_TO_MPS;
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

// Test .223 Rem 55gr FMJ
TEST_F(CartridgeTest, 223Rem55gr) {
    SolverParams p = makeCartridgeParams(500.0f, 0.245f, DragModel::G1, 990.0f, 55.0f, 20.0f, 25.0f); // ~3250 fps from 20"
    
    float zero_angle = solver.solveZeroAngle(p, 100.0f);
    EXPECT_FALSE(std::isnan(zero_angle));
    p.launch_angle_rad = zero_angle;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);
    
    // Rough sanity check for .223 at 500 yards
    EXPECT_GT(result.velocity_at_target_ms, 400.0f); // Should be supersonic
    EXPECT_LT(result.tof_s, 1.0f);
}

// Test 6.5 Creedmoor 140gr ELD-M
TEST_F(CartridgeTest, 65Creedmoor140gr) {
    SolverParams p = makeCartridgeParams(1000.0f, 0.326f, DragModel::G7, 823.0f, 140.0f, 26.0f, 25.0f); // ~2700 fps from 26"

    float zero_angle = solver.solveZeroAngle(p, 100.0f);
    EXPECT_FALSE(std::isnan(zero_angle));
    p.launch_angle_rad = zero_angle;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);

    // Rough sanity check for 6.5 CM at 1000m
    // Expected to be near transonic by this distance depending on drag profile.
    EXPECT_GT(result.velocity_at_target_ms, 280.0f);
    EXPECT_LT(result.tof_s, 2.3f);
}

// Test .300 Win Mag 190gr BTHP
TEST_F(CartridgeTest, 300WinMag190gr) {
    SolverParams p = makeCartridgeParams(1200.0f, 0.533f, DragModel::G1, 884.0f, 190.0f, 24.0f, 30.0f); // ~2900 fps from 24"

    float zero_angle = solver.solveZeroAngle(p, 100.0f);
    EXPECT_FALSE(std::isnan(zero_angle));
    p.launch_angle_rad = zero_angle;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);

    // Rough sanity check for .300WM at 1200m
    EXPECT_GT(result.velocity_at_target_ms, 300.0f);
    EXPECT_LT(result.tof_s, 2.5f);
}

// Test 9mm 124gr FMJ from compact handgun barrel
// 365 m/s is the SAAMI reference MV measured from a 4" test barrel.
TEST_F(CartridgeTest, NineMm124grHandgun) {
    SolverParams p = makeCartridgeParams(100.0f, 0.150f, DragModel::G1, 365.0f, 124.0f, 4.0f, 12.0f, 4.0f); // 1197 fps from 4" (delta=0)

    float zero_angle = solver.solveZeroAngle(p, 25.0f);
    EXPECT_FALSE(std::isnan(zero_angle));
    p.launch_angle_rad = zero_angle;

    SolverResult result = solver.integrate(p);
    EXPECT_TRUE(result.valid);

    // Rough sanity check for 9mm handgun at 100m — starting at ~365 m/s (1197 fps)
    EXPECT_GT(result.velocity_at_target_ms, 270.0f);
    EXPECT_LT(result.tof_s, 1.2f);
}

// Test 9mm 124gr FMJ from PDW barrel and compare to handgun profile.
// Both profiles share the same SAAMI base MV (365 m/s at 4" reference).
// The PDW (8") should gain ~48 fps over the handgun (4") via barrel length alone.
TEST_F(CartridgeTest, NineMm124grPdwVsHandgun) {
    SolverParams handgun = makeCartridgeParams(100.0f, 0.150f, DragModel::G1, 365.0f, 124.0f, 4.0f, 12.0f, 4.0f);
    SolverParams pdw     = makeCartridgeParams(100.0f, 0.150f, DragModel::G1, 365.0f, 124.0f, 8.0f, 12.0f, 4.0f);

    float handgun_zero = solver.solveZeroAngle(handgun, 25.0f);
    float pdw_zero = solver.solveZeroAngle(pdw, 25.0f);
    EXPECT_FALSE(std::isnan(handgun_zero));
    EXPECT_FALSE(std::isnan(pdw_zero));

    handgun.launch_angle_rad = handgun_zero;
    pdw.launch_angle_rad = pdw_zero;

    SolverResult handgun_result = solver.integrate(handgun);
    SolverResult pdw_result = solver.integrate(pdw);
    EXPECT_TRUE(handgun_result.valid);
    EXPECT_TRUE(pdw_result.valid);

    // At the same distance, PDW profile should be faster and get there sooner.
    EXPECT_GT(pdw_result.velocity_at_target_ms, handgun_result.velocity_at_target_ms);
    EXPECT_LT(pdw_result.tof_s, handgun_result.tof_s);
}
