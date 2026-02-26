/**
 * @file test_ahrs.cpp
 * @brief Unit tests for AHRS (Madgwick + Mahony) filters.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/ahrs/ahrs_manager.h"
#include <cmath>

class AHRSTest : public ::testing::Test {
protected:
    AHRSManager ahrs;

    void SetUp() override {
        ahrs.init();
    }
};

// After init, quaternion should be identity
TEST_F(AHRSTest, InitialQuaternionIsIdentity) {
    Quaternion q = ahrs.getQuaternion();
    EXPECT_NEAR(q.w, 1.0f, 0.001f);
    EXPECT_NEAR(q.x, 0.0f, 0.001f);
    EXPECT_NEAR(q.y, 0.0f, 0.001f);
    EXPECT_NEAR(q.z, 0.0f, 0.001f);
}

// With pure gravity on Z-axis (device flat), pitch/roll should be ~0
TEST_F(AHRSTest, MadgwickFlatOrientation) {
    ahrs.setAlgorithm(AHRS_Algorithm::MADGWICK);

    // Feed accel pointing up (device flat, Z-up)
    for (int i = 0; i < 500; ++i) {
        ahrs.update(0.0f, 0.0f, 9.81f,  // accel
                    0.0f, 0.0f, 0.0f,    // gyro
                    0.0f, 0.0f, 0.0f,    // mag
                    false, 0.01f);
    }

    EXPECT_NEAR(ahrs.getPitch(), 0.0f, 0.1f);
    EXPECT_NEAR(ahrs.getRoll(), 0.0f, 0.1f);
}

// Same test with Mahony filter
TEST_F(AHRSTest, MahonyFlatOrientation) {
    ahrs.setAlgorithm(AHRS_Algorithm::MAHONY);

    for (int i = 0; i < 500; ++i) {
        ahrs.update(0.0f, 0.0f, 9.81f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }

    EXPECT_NEAR(ahrs.getPitch(), 0.0f, 0.1f);
    EXPECT_NEAR(ahrs.getRoll(), 0.0f, 0.1f);
}

// Quaternion should always be normalized
TEST_F(AHRSTest, QuaternionStaysNormalized) {
    for (int i = 0; i < 100; ++i) {
        ahrs.update(0.1f, 0.2f, 9.7f,
                    0.01f, -0.02f, 0.005f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);

        Quaternion q = ahrs.getQuaternion();
        float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        EXPECT_NEAR(norm, 1.0f, 0.001f);
    }
}

// After enough stable samples, isStable() should return true
TEST_F(AHRSTest, StabilityAfterConvergence) {
    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW + 10; ++i) {
        ahrs.update(0.0f, 0.0f, 9.81f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }
    EXPECT_TRUE(ahrs.isStable());
}

// Static detection: steady accel should report static
TEST_F(AHRSTest, StaticDetectionWhenStill) {
    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW + 10; ++i) {
        ahrs.update(0.0f, 0.0f, 9.81f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }
    EXPECT_TRUE(ahrs.isStatic());
}

// Static detection: varying accel should not be static
TEST_F(AHRSTest, NotStaticWhenMoving) {
    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW + 10; ++i) {
        float noise = (i % 2 == 0) ? 5.0f : -5.0f;
        ahrs.update(noise, noise, 9.81f + noise,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }
    EXPECT_FALSE(ahrs.isStatic());
}

// Bias correction should offset the readings
TEST_F(AHRSTest, GyroBiasCorrection) {
    // Set gyro bias
    float gyro_bias[3] = {0.01f, -0.02f, 0.005f};
    ahrs.setGyroBias(gyro_bias);

    // Feed exact bias values — should result in zero effective gyro
    for (int i = 0; i < 200; ++i) {
        ahrs.update(0.0f, 0.0f, 9.81f,
                    0.01f, -0.02f, 0.005f,  // same as bias → net zero
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }

    // Should converge to flat (same as no rotation)
    EXPECT_NEAR(ahrs.getPitch(), 0.0f, 0.15f);
    EXPECT_NEAR(ahrs.getRoll(), 0.0f, 0.15f);
}

// Stability should drop if the platform is dynamic/noisy after convergence
TEST_F(AHRSTest, StabilityDropsWhenDynamic) {
    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW + 10; ++i) {
        ahrs.update(0.0f, 0.0f, 9.81f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }
    EXPECT_TRUE(ahrs.isStable());

    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW + 10; ++i) {
        float noise = (i % 2 == 0) ? 5.0f : -5.0f;
        ahrs.update(noise, noise, 9.81f + noise,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    false, 0.01f);
    }

    EXPECT_FALSE(ahrs.isStable());
}
