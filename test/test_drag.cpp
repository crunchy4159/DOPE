/**
 * @file test_drag.cpp
 * @brief Unit tests for drag model lookup and interpolation.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/drag/drag_model.h"
#include "bce/bce_config.h"

// G1 Cd at Mach 0 should match the first table entry
TEST(DragTest, G1CdAtMachZero) {
    float cd = DragModelLookup::getCd(DragModel::G1, 0.0f);
    EXPECT_NEAR(cd, 0.2629f, 0.001f);
}

// G7 Cd at Mach 0 should match the first G7 entry
TEST(DragTest, G7CdAtMachZero) {
    float cd = DragModelLookup::getCd(DragModel::G7, 0.0f);
    EXPECT_NEAR(cd, 0.1198f, 0.001f);
}

// Cd should increase through transonic region
TEST(DragTest, G1TransonicRise) {
    float cd_sub = DragModelLookup::getCd(DragModel::G1, 0.8f);
    float cd_trans = DragModelLookup::getCd(DragModel::G1, 1.0f);
    EXPECT_GT(cd_trans, cd_sub);
}

// Cd should decrease at very high Mach
TEST(DragTest, G1SupersonicDecrease) {
    float cd_low = DragModelLookup::getCd(DragModel::G1, 1.5f);
    float cd_high = DragModelLookup::getCd(DragModel::G1, 3.0f);
    EXPECT_GT(cd_low, cd_high);
}

// Interpolation should produce smooth values between table points
TEST(DragTest, G1InterpolationSmooth) {
    float cd1 = DragModelLookup::getCd(DragModel::G1, 1.0f);
    float cd2 = DragModelLookup::getCd(DragModel::G1, 1.05f);
    float cd_mid = DragModelLookup::getCd(DragModel::G1, 1.025f);

    // Mid value should be between the two endpoints
    EXPECT_GE(cd_mid, std::min(cd1, cd2) - 0.001f);
    EXPECT_LE(cd_mid, std::max(cd1, cd2) + 0.001f);
}

// All G models should return positive Cd
TEST(DragTest, AllModelsPositiveCd) {
    DragModel models[] = {DragModel::G1, DragModel::G2, DragModel::G3, DragModel::G4,
                          DragModel::G5, DragModel::G6, DragModel::G7, DragModel::G8};

    for (auto model : models) {
        for (float mach = 0.0f; mach <= 5.0f; mach += 0.5f) {
            float cd = DragModelLookup::getCd(model, mach);
            EXPECT_GT(cd, 0.0f) << "Model " << static_cast<int>(model)
                                 << " Mach " << mach;
        }
    }
}

// Deceleration should be positive and increase with velocity
TEST(DragTest, DecelerationIncreasesWithVelocity) {
    float decel_slow = DragModelLookup::getDeceleration(
        300.0f, 340.0f, 0.505f, DragModel::G1, 1.225f);
    float decel_fast = DragModelLookup::getDeceleration(
        800.0f, 340.0f, 0.505f, DragModel::G1, 1.225f);

    EXPECT_GT(decel_slow, 0.0f);
    EXPECT_GT(decel_fast, decel_slow);
}

// Higher BC should produce less deceleration
TEST(DragTest, HigherBCLessDeceleration) {
    float decel_low_bc = DragModelLookup::getDeceleration(
        800.0f, 340.0f, 0.300f, DragModel::G1, 1.225f);
    float decel_high_bc = DragModelLookup::getDeceleration(
        800.0f, 340.0f, 0.600f, DragModel::G1, 1.225f);

    EXPECT_GT(decel_low_bc, decel_high_bc);
}

// Clamping: negative Mach should not crash
TEST(DragTest, NegativeMachClamps) {
    float cd = DragModelLookup::getCd(DragModel::G1, -1.0f);
    EXPECT_GT(cd, 0.0f);
}

// Clamping: very high Mach should return last table entry
TEST(DragTest, HighMachClamps) {
    float cd = DragModelLookup::getCd(DragModel::G1, 10.0f);
    EXPECT_GT(cd, 0.0f);
}
