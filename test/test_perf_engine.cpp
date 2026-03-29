#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "test_helpers_v2.h"
#include <chrono>
#include <cmath>
#include <iostream>

namespace {

SensorFrame BuildFrame(uint64_t& now_us, float lrf_range_m) {
    SensorFrame frame = {};
    now_us += 10000;
    frame.timestamp_us = now_us;
    frame.accel_x = 0.0f; frame.accel_y = 0.0f; frame.accel_z = 9.81f;
    frame.gyro_x = 0.0f;  frame.gyro_y = 0.0f;  frame.gyro_z = 0.0f;
    frame.imu_valid = true;
    frame.mag_x = 25.0f; frame.mag_y = 0.0f; frame.mag_z = 40.0f;
    frame.mag_valid = true;
    frame.baro_pressure_pa = 101325.0f;
    frame.baro_temperature_c = 20.0f;
    frame.baro_humidity = 0.50f;
    frame.baro_valid = true;
    frame.baro_humidity_valid = true;
    frame.lrf_range_m = lrf_range_m;
    frame.lrf_timestamp_us = now_us;
    frame.lrf_valid = true;
    return frame;
}

void ConfigureDefaults() {
    DOPE_Init();
    EnableSolverFallback();

    AmmoDatasetV2 ds = Make308V2();
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);

    DOPE_SetWindManual(0.0f, 0.0f);
    DOPE_SetLatitude(37.0f);
}

} // namespace

TEST(Performance, Run100FramesTimingDefaultRange) {
    ConfigureDefaults();

    uint64_t now_us = 0;
    constexpr int kFrames = 100;
    constexpr float kRangeM = 400.0f;

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) {
        SensorFrame frame = BuildFrame(now_us, kRangeM);
        DOPE_Update(&frame);
    }
    auto end = std::chrono::steady_clock::now();

    FiringSolution sol = {};
    DOPE_GetSolution(&sol);

    const double total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    const double per_frame_ms = total_ms / static_cast<double>(kFrames);

    std::cout << "Run100 default_range_m=" << kRangeM
              << " total_ms=" << total_ms
              << " per_frame_ms=" << per_frame_ms << "\n";

    EXPECT_GT(total_ms, 0.0);
}

TEST(Performance, Run100FramesTimingLongRange) {
    if (std::getenv("DOPE_RUN_LONG_PERF") == nullptr) {
        GTEST_SKIP();
    }

    ConfigureDefaults();

    uint64_t now_us = 0;
    constexpr int kFrames = 100;
    constexpr float kRangeM = 2000.0f;

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) {
        SensorFrame frame = BuildFrame(now_us, kRangeM);
        DOPE_Update(&frame);
    }
    auto end = std::chrono::steady_clock::now();

    const double total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    const double per_frame_ms = total_ms / static_cast<double>(kFrames);

    std::cout << "Run100 long_range_m=" << kRangeM
              << " total_ms=" << total_ms
              << " per_frame_ms=" << per_frame_ms << "\n";

    EXPECT_GT(total_ms, 0.0);
}
