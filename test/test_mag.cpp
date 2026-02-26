/**
 * @file test_mag.cpp
 * @brief Unit tests for magnetometer calibration and heading.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/mag/mag_calibration.h"
#include "bce/bce_config.h"
#include <cmath>

class MagTest : public ::testing::Test {
protected:
    MagCalibration mag;

    void SetUp() override {
        mag.init();
    }
};

// Identity calibration should not change readings
TEST_F(MagTest, IdentityCalibration) {
    float mx = 25.0f, my = 0.0f, mz = 40.0f;
    bool ok = mag.apply(mx, my, mz);

    EXPECT_TRUE(ok); // field is within valid range
    EXPECT_NEAR(mx, 25.0f, 0.001f);
    EXPECT_NEAR(my, 0.0f, 0.001f);
    EXPECT_NEAR(mz, 40.0f, 0.001f);
}

// Hard iron offset should be subtracted
TEST_F(MagTest, HardIronCorrection) {
    float hi[3] = {10.0f, 5.0f, -3.0f};
    float si[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // identity
    mag.setCalibration(hi, si);

    float mx = 35.0f, my = 5.0f, mz = 37.0f;
    mag.apply(mx, my, mz);

    EXPECT_NEAR(mx, 25.0f, 0.001f);
    EXPECT_NEAR(my, 0.0f, 0.001f);
    EXPECT_NEAR(mz, 40.0f, 0.001f);
}

// Disturbance should be detected for out-of-range field
TEST_F(MagTest, DisturbanceDetected) {
    // Very strong field (> 70 µT)
    float mx = 100.0f, my = 100.0f, mz = 100.0f;
    bool ok = mag.apply(mx, my, mz);

    EXPECT_FALSE(ok);
    EXPECT_TRUE(mag.isDisturbed());
}

// Very weak field should also be disturbed
TEST_F(MagTest, WeakFieldDisturbance) {
    float mx = 1.0f, my = 1.0f, mz = 1.0f;
    bool ok = mag.apply(mx, my, mz);

    EXPECT_FALSE(ok);
    EXPECT_TRUE(mag.isDisturbed());
}

// Heading at 0 yaw with 0 declination should be 0
TEST_F(MagTest, HeadingZero) {
    float heading = mag.computeHeading(0.0f);
    EXPECT_NEAR(heading, 0.0f, 0.001f);
}

// Declination should offset heading
TEST_F(MagTest, DeclinationOffset) {
    mag.setDeclination(10.0f); // 10° east
    float heading = mag.computeHeading(0.0f);
    EXPECT_NEAR(heading, 10.0f, 0.001f);
}

// Heading should wrap to 0–360 range
TEST_F(MagTest, HeadingWraps) {
    float heading = mag.computeHeading(-0.1f); // slightly negative yaw
    EXPECT_GE(heading, 0.0f);
    EXPECT_LT(heading, 360.0f);
}
