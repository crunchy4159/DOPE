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
#include <cmath>

class CartridgeTest : public ::testing::Test {
protected:
    BallisticSolver solver;

    void SetUp() override {
        solver.init();
    }

    SolverParams makeCartridgeParams(float range_m, float bc, DragModel model, float mv_ms, float mass_gr, float barrel_in, float mv_adjust) {
        SolverParams p = {};
        p.bc = bc;
        p.drag_model = model;
        
        // Adjust MV
        float base_mv_fps = mv_ms * 3.28084f;
        float barrel_length_delta_in = barrel_in - 24.0f;
        float adjusted_mv_fps = base_mv_fps + (barrel_length_delta_in * mv_adjust);
        p.muzzle_velocity_ms = adjusted_mv_fps * 0.3048f;

        p.bullet_mass_kg = mass_gr * BCE_GRAINS_TO_KG;
        p.sight_height_m = 0.0381f; // 1.5 inches
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
