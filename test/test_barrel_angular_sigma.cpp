/**
 * @file test_barrel_angular_sigma.cpp
 * @brief Focused test: `GunProfile.angular_sigma_moa` affects uncertainty output.
 */

#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "test_helpers_v2.h"
#include <cstring>
#include <cmath>

class BarrelAngularSigmaTest : public ::testing::Test {
protected:
    void SetUp() override {
        DOPE_Init();
        EnableSolverFallback();

        AmmoDatasetV2 ds = Make308V2();
        ds.mv_adjustment_fps_per_in = 25.0f;
        DOPE_SetAmmoDatasetV2(&ds);

        ZeroConfig z = {};
        z.zero_range_m = 100.0f;
        z.sight_height_mm = 38.1f;
        DOPE_SetZeroConfig(&z);

        // Basic atmosphere + latitude
        DOPE_SetLatitude(37.0f);
    }

    SensorFrame makeDefaultFrame(uint64_t timestamp_us) {
        SensorFrame f;
        std::memset(&f, 0, sizeof(f));
        f.timestamp_us = timestamp_us;
        f.accel_x = 0.0f; f.accel_y = 0.0f; f.accel_z = 9.81f;
        f.gyro_x = f.gyro_y = f.gyro_z = 0.0f;
        f.imu_valid = true;
        f.baro_pressure_pa = 101325.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true; f.baro_humidity_valid = true;
        f.mag_x = 25.0f; f.mag_y = 0.0f; f.mag_z = 40.0f; f.mag_valid = true;
        f.lrf_valid = false; f.encoder_valid = false;
        return f;
    }

    void stabilizeAHRS() {
        for (int i = 0; i < 200; ++i) {
            SensorFrame f = makeDefaultFrame(10000 * (i + 1));
            DOPE_Update(&f);
        }
    }

    void feedRange(float range_m) {
        SensorFrame f = makeDefaultFrame(10000 * 300);
        f.lrf_range_m = range_m;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        DOPE_Update(&f);
    }
};

TEST_F(BarrelAngularSigmaTest, AngularSigmaIncreasesOutputUncertainty) {
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    DOPE_SetUncertaintyConfig(&uc);

    // Baseline: angular_sigma_moa == 0
    GunProfile g = {};
    g.barrel_length_in = 24.0f;
    g.reference_barrel_length_in = 24.0f;
    g.angular_sigma_moa = 0.0f;
    g.free_floated = true;
    DOPE_SetGunProfile(&g);

    SensorFrame f0 = makeDefaultFrame(10000 * 400);
    f0.lrf_range_m = 500.0f; f0.lrf_timestamp_us = f0.timestamp_us; f0.lrf_valid = true;
    DOPE_Update(&f0);
    FiringSolution base = {};
    DOPE_GetSolution(&base);
    ASSERT_TRUE(base.uncertainty_valid);
    const float base_radius = std::sqrt(base.sigma_elevation_moa * base.sigma_elevation_moa + base.sigma_windage_moa * base.sigma_windage_moa);

    // Now set a nonzero angular sigma and expect overall uncertainty to increase
    g.angular_sigma_moa = 2.5f; // 2.5 MOA 1-sigma angular component
    DOPE_SetGunProfile(&g);

    SensorFrame f1 = makeDefaultFrame(10000 * 410);
    f1.lrf_range_m = 500.0f; f1.lrf_timestamp_us = f1.timestamp_us; f1.lrf_valid = true;
    DOPE_Update(&f1);
    FiringSolution with_angular = {};
    DOPE_GetSolution(&with_angular);
    ASSERT_TRUE(with_angular.uncertainty_valid);
    const float angular_radius = std::sqrt(with_angular.sigma_elevation_moa * with_angular.sigma_elevation_moa + with_angular.sigma_windage_moa * with_angular.sigma_windage_moa);

    EXPECT_GT(angular_radius, base_radius * 1.1f);
}
