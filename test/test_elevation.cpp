/*
 * @file test_elevation.cpp
 * @brief Unit test verifying target_elevation_m affects elevation hold as expected.
 */

#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "dope/dope_math_utils.h"
#include <cmath>
#include <cstring>

TEST(ElevationTest, ElevationShiftsHold) {
    DOPE_Init();

    // Minimal bullet profile
    BulletProfile bullet = {};
    bullet.bc = 0.5f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 800.0f;
    bullet.mass_grains = 150.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    DOPE_SetBulletProfile(&bullet);

    // Zero at 100 m
    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);

    // Feed baseline frames (range = 500 m)
    for (int i = 0; i < 120; ++i) {
        SensorFrame f;
        std::memset(&f, 0, sizeof(f));
        f.timestamp_us = (uint64_t)(i + 1) * 10000;
        f.accel_x = 0.0f; f.accel_y = 0.0f; f.accel_z = 9.81f; f.imu_valid = true;
        f.baro_pressure_pa = 101325.0f; f.baro_temperature_c = 15.0f; f.baro_valid = true;
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }

    FiringSolution sol0; DOPE_GetSolution(&sol0);

    // Now apply a 1.0 m positive target elevation and re-run
    DOPE_Init();
    DOPE_SetBulletProfile(&bullet);
    DOPE_SetZeroConfig(&zero);

    for (int i = 0; i < 120; ++i) {
        SensorFrame f;
        std::memset(&f, 0, sizeof(f));
        f.timestamp_us = (uint64_t)(i + 1) * 10000;
        f.accel_x = 0.0f; f.accel_y = 0.0f; f.accel_z = 9.81f; f.imu_valid = true;
        f.baro_pressure_pa = 101325.0f; f.baro_temperature_c = 15.0f; f.baro_valid = true;
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        f.target_elevation_m = 1.0f;
        DOPE_Update(&f);
    }

    FiringSolution sol1; DOPE_GetSolution(&sol1);

    // Expected change in MOA ~ (elevation_m / range_m) * RAD_TO_MOA
    const float expected_delta_moa = (1.0f / 500.0f) * dope::math::RAD_TO_MOA;
    const float actual_delta_moa = sol1.hold_elevation_moa - sol0.hold_elevation_moa;

    EXPECT_NEAR(actual_delta_moa, expected_delta_moa, 0.5f); // allow loose tolerance
}
