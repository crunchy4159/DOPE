/**
 * @file test_integration.cpp
 * @brief End-to-end integration tests for the DOPE V2 API.
 *
 * All tests use AmmoDatasetV2 + DOPE_SetAmmoDatasetV2.
 */

#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "dope/dope_config.h"
#include "dope/dope_math_utils.h"
#include "test_helpers_v2.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr float kMoaPerMil = 3.43774677f;
}

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        DOPE_Init();
        EnableSolverFallback();
    }

    SensorFrame makeDefaultFrame(uint64_t timestamp_us) {
        SensorFrame f;
        std::memset(&f, 0, sizeof(f));
        f.timestamp_us = timestamp_us;
        f.accel_x = 0.0f; f.accel_y = 0.0f; f.accel_z = 9.81f;
        f.gyro_x = 0.0f;  f.gyro_y = 0.0f;  f.gyro_z = 0.0f;
        f.imu_valid = true;
        f.baro_pressure_pa = 101325.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;
        f.mag_valid = false;
        f.lrf_valid = false;
        f.encoder_valid = false;
        return f;
    }

    void setDefault308() {
        AmmoDatasetV2 ds = Make308V2();
        DOPE_SetAmmoDatasetV2(&ds);
    }

    void setDefault308(float twist_rate_inches) {
        AmmoDatasetV2 ds = Make308V2(twist_rate_inches);
        DOPE_SetAmmoDatasetV2(&ds);
    }

    void setZero100m(float sight_height_mm = 38.1f) {
        ZeroConfig zero = {};
        zero.zero_range_m = 100.0f;
        zero.sight_height_mm = sight_height_mm;
        DOPE_SetZeroConfig(&zero);
    }
};

TEST_F(IntegrationTest, InitialModeIsIdle) {
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::IDLE);
}

TEST_F(IntegrationTest, NoBulletFaults) {
    SensorFrame f = makeDefaultFrame(1000);
    f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = 1000;
    for (int i = 0; i < 100; ++i) {
        f.timestamp_us = (uint64_t)(i + 1) * 10000;
        f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::FAULT);
    EXPECT_NE(DOPE_GetFaultFlags() & DOPE_Fault::NO_BULLET, 0u);
}

TEST_F(IntegrationTest, FullConfigProducesSolution) {
    AmmoDatasetV2 ds = Make308V2();
    ds.length_mm = 31.2f;
    DOPE_SetAmmoDatasetV2(&ds);
    setZero100m();

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }

    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution sol; DOPE_GetSolution(&sol);
    EXPECT_EQ(sol.solution_mode, static_cast<uint32_t>(DOPE_Mode::SOLUTION_READY));
    EXPECT_FLOAT_EQ(sol.range_m, 500.0f);
    EXPECT_GT(sol.tof_ms, 0.0f);
    EXPECT_GT(sol.velocity_at_target_ms, 0.0f);
    EXPECT_GT(sol.energy_at_target_j, 0.0f);
    EXPECT_GT(sol.air_density_kgm3, 0.0f);
}

TEST_F(IntegrationTest, RealtimeSolutionMatchesPrimaryHolds) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution full = {}; RealtimeSolution rt = {};
    DOPE_GetSolution(&full); DOPE_GetRealtimeSolution(&rt);
    EXPECT_EQ(rt.solution_mode, full.solution_mode);
    EXPECT_FLOAT_EQ(rt.hold_elevation_moa, full.hold_elevation_moa);
    EXPECT_FLOAT_EQ(rt.hold_windage_moa, full.hold_windage_moa);
    EXPECT_FLOAT_EQ(rt.range_m, full.range_m);
    EXPECT_FLOAT_EQ(rt.tof_ms, full.tof_ms);
    EXPECT_FLOAT_EQ(rt.velocity_at_target_ms, full.velocity_at_target_ms);
    EXPECT_FLOAT_EQ(rt.uncertainty_confidence, 0.682689f);
    if (full.uncertainty_valid) {
        const float expected_radius = std::sqrt(
            full.sigma_elevation_moa * full.sigma_elevation_moa +
            full.sigma_windage_moa * full.sigma_windage_moa);
        EXPECT_TRUE(rt.uncertainty_valid);
        EXPECT_NEAR(rt.uncertainty_radius_moa, expected_radius, 1e-5f);
    }
}

TEST_F(IntegrationTest, FederalFusion150gr500ydReferenceEnvelope) {
    AmmoDatasetV2 ds = MakeFedFusion150V2();
    DOPE_SetAmmoDatasetV2(&ds);
    ZeroConfig zero = {}; zero.zero_range_m = 91.44f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    DOPE_SetWindManual(0.0f, 0.0f);
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 457.2f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution sol; DOPE_GetSolution(&sol);
    const float velocity_fps = sol.velocity_at_target_ms * dope::math::MPS_TO_FPS;
    EXPECT_GT(velocity_fps, 1650.0f);
    EXPECT_LT(velocity_fps, 2000.0f);
}

TEST_F(IntegrationTest, PresetPair223Vs308HoldOrderingAt500Yd) {
    auto solve_at_500yd = [&](AmmoDatasetV2 ds) {
        DOPE_Init(); EnableSolverFallback();
        DOPE_SetAmmoDatasetV2(&ds);
        ZeroConfig zero = {}; zero.zero_range_m = 91.44f; zero.sight_height_mm = 38.1f;
        DOPE_SetZeroConfig(&zero);
        DOPE_SetWindManual(0.0f, 0.0f);
        for (int i = 0; i < 120; ++i) {
            SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
            f.lrf_valid = true; f.lrf_range_m = 457.2f; f.lrf_timestamp_us = f.timestamp_us;
            DOPE_Update(&f);
        }
        EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
        FiringSolution sol = {}; DOPE_GetSolution(&sol);
        return sol;
    };

    AmmoDatasetV2 p308 = Make308V2(10.0f);
    p308.mv_adjustment_fps_per_in = 25.0f;

    AmmoDatasetV2 p223 = Make223V2();

    const FiringSolution sol_308 = solve_at_500yd(p308);
    const FiringSolution sol_223 = solve_at_500yd(p223);
    EXPECT_GT(sol_308.hold_elevation_moa, sol_223.hold_elevation_moa);
    EXPECT_GT(sol_308.velocity_at_target_ms, sol_223.velocity_at_target_ms);
}

TEST_F(IntegrationTest, CoriolisDisabledWithoutLatitude) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 1000.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_NE(DOPE_GetDiagFlags() & DOPE_Diag::CORIOLIS_DISABLED, 0u);
    FiringSolution sol; DOPE_GetSolution(&sol);
    EXPECT_FLOAT_EQ(sol.coriolis_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(sol.coriolis_elevation_moa, 0.0f);
}

TEST_F(IntegrationTest, StaleLRFCausesFault) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    uint64_t future_time = 100 * 10000 + DOPE_LRF_STALE_US + 1000000;
    for (int i = 0; i < 10; ++i) {
        SensorFrame f = makeDefaultFrame(future_time + (uint64_t)i * 10000);
        f.lrf_valid = false;
        DOPE_Update(&f);
    }
    EXPECT_NE(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
}

TEST_F(IntegrationTest, LRFStaleCheckHandlesTimestampNearWrap) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    constexpr uint64_t near_wrap_lrf_ts = std::numeric_limits<uint64_t>::max() - 100;
    SensorFrame near_wrap = makeDefaultFrame(near_wrap_lrf_ts);
    near_wrap.lrf_valid = true; near_wrap.lrf_range_m = 500.0f;
    near_wrap.lrf_timestamp_us = near_wrap.timestamp_us;
    DOPE_Update(&near_wrap);
    SensorFrame shortly_after = makeDefaultFrame(near_wrap_lrf_ts + 50);
    shortly_after.lrf_valid = false;
    DOPE_Update(&shortly_after);
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_EQ(DOPE_GetDiagFlags() & DOPE_Diag::LRF_STALE, 0u);
}

TEST_F(IntegrationTest, DefaultOverridesApplied) {
    DOPE_DefaultOverrides ovr = {};
    ovr.use_altitude = true; ovr.altitude_m = 1500.0f;
    ovr.use_temperature = true; ovr.temperature_c = 30.0f;
    DOPE_SetDefaultOverrides(&ovr);
    EXPECT_EQ(DOPE_GetDiagFlags() & DOPE_Diag::DEFAULT_ALTITUDE, 0u);
}

TEST_F(IntegrationTest, InvalidZeroRangeFaults) {
    setDefault308();
    ZeroConfig zero = {}; zero.zero_range_m = 0.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::FAULT);
    EXPECT_NE(DOPE_GetFaultFlags() & DOPE_Fault::ZERO_UNSOLVABLE, 0u);
}

TEST_F(IntegrationTest, WindDefaultOverrideAffectsSolution) {
    setDefault308(0.0f); setZero100m();
    DOPE_DefaultOverrides ovr = {};
    ovr.use_wind = true; ovr.wind_speed_ms = 10.0f; ovr.wind_heading_deg = 90.0f;
    DOPE_SetDefaultOverrides(&ovr);
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_EQ(DOPE_GetDiagFlags() & DOPE_Diag::DEFAULT_WIND, 0u);
    FiringSolution sol; DOPE_GetSolution(&sol);
    EXPECT_NE(sol.hold_windage_moa, 0.0f);
}

TEST_F(IntegrationTest, WindFromRightRequiresRightHold) {
    setDefault308(0.0f); setZero100m();
    DOPE_SetWindManual(10.0f, 90.0f);
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution sol = {}; DOPE_GetSolution(&sol);
    EXPECT_GT(sol.hold_windage_moa, 0.0f);
}

TEST_F(IntegrationTest, NullCalibrationInputsAreSafe) {
    DOPE_SetIMUBias(nullptr, nullptr);
    DOPE_SetMagCalibration(nullptr, nullptr);
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
}

TEST_F(IntegrationTest, MechanicalOffsetsShiftHolds) {
    AmmoDatasetV2 ds = Make308V2(0.0f); // no spin drift
    DOPE_SetAmmoDatasetV2(&ds);
    setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    FiringSolution base; DOPE_GetSolution(&base);
    BoresightOffset boresight = {}; boresight.vertical_moa = 1.5f; boresight.horizontal_moa = -2.0f;
    DOPE_SetBoresightOffset(&boresight);
    DOPE_SetReticleMechanicalOffset(0.5f, 1.0f);
    for (int i = 0; i < 10; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(101 + i) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    FiringSolution shifted; DOPE_GetSolution(&shifted);
    EXPECT_NEAR(shifted.hold_elevation_moa - base.hold_elevation_moa, 2.0f, 0.05f);
    EXPECT_NEAR(shifted.hold_windage_moa - base.hold_windage_moa, -1.0f, 0.05f);
}

TEST_F(IntegrationTest, MagDisturbanceSetsDiagnostic) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.mag_valid = true; f.mag_x = 200.0f; f.mag_y = 200.0f; f.mag_z = 200.0f;
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_NE(DOPE_GetDiagFlags() & DOPE_Diag::MAG_SUPPRESSED, 0u);
}

TEST_F(IntegrationTest, BaroCalibrationRaisesLowPressureDensity) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.baro_pressure_pa = 90000.0f; f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f; f.baro_valid = true; f.baro_humidity_valid = true;
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    FiringSolution before; DOPE_GetSolution(&before);
    DOPE_CalibrateBaro();
    for (int i = 0; i < 20; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(101 + i) * 10000);
        f.baro_pressure_pa = 90000.0f; f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f; f.baro_valid = true; f.baro_humidity_valid = true;
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    FiringSolution after; DOPE_GetSolution(&after);
    EXPECT_GT(after.air_density_kgm3, before.air_density_kgm3);
}

TEST_F(IntegrationTest, DeclinationOffsetsHeading) {
    DOPE_SetMagDeclination(12.0f);
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    FiringSolution sol; DOPE_GetSolution(&sol);
    EXPECT_NEAR(sol.heading_deg_true, 12.0f, 0.5f);
}

TEST_F(IntegrationTest, TimestampJumpsAndRollbackRemainDeterministic) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 80; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    SensorFrame jump = makeDefaultFrame(10ull * 1000ull * 1000ull);
    jump.lrf_valid = true; jump.lrf_range_m = 500.0f; jump.lrf_timestamp_us = jump.timestamp_us;
    DOPE_Update(&jump);
    SensorFrame rollback = makeDefaultFrame(500000ull);
    rollback.lrf_valid = true; rollback.lrf_range_m = 500.0f; rollback.lrf_timestamp_us = rollback.timestamp_us;
    DOPE_Update(&rollback);
    for (int i = 0; i < 40; ++i) {
        SensorFrame f = makeDefaultFrame(600000ull + (uint64_t)i * 10000ull);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution sol; DOPE_GetSolution(&sol);
    EXPECT_TRUE(std::isfinite(sol.hold_elevation_moa));
    EXPECT_TRUE(std::isfinite(sol.hold_windage_moa));
}

TEST_F(IntegrationTest, NaNImuInputFlagsSensorInvalid) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true; f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    SensorFrame bad = makeDefaultFrame(2000000ull);
    bad.accel_x = std::numeric_limits<float>::quiet_NaN();
    bad.lrf_valid = true; bad.lrf_range_m = 500.0f; bad.lrf_timestamp_us = bad.timestamp_us;
    DOPE_Update(&bad);
    EXPECT_NE(DOPE_GetFaultFlags() & DOPE_Fault::SENSOR_INVALID, 0u);
}

TEST_F(IntegrationTest, InfRangeRejectedAndFlagged) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = std::numeric_limits<float>::infinity();
        f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::FAULT);
    EXPECT_NE(DOPE_GetFaultFlags() & DOPE_Fault::NO_RANGE, 0u);
    EXPECT_NE(DOPE_GetFaultFlags() & DOPE_Fault::SENSOR_INVALID, 0u);
}

TEST_F(IntegrationTest, RapidRangeValidityTransitionsRecover) {
    setDefault308(); setZero100m();
    for (int i = 0; i < 200; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = (i % 2 == 0); f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    SensorFrame final_good = makeDefaultFrame(3000000ull);
    final_good.lrf_valid = true; final_good.lrf_range_m = 500.0f;
    final_good.lrf_timestamp_us = final_good.timestamp_us;
    DOPE_Update(&final_good);
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution sol; DOPE_GetSolution(&sol);
    EXPECT_TRUE(std::isfinite(sol.hold_elevation_moa));
    EXPECT_TRUE(std::isfinite(sol.hold_windage_moa));
}

TEST_F(IntegrationTest, AtmosphericDeltaTriggersZeroRecompute) {
    AmmoDatasetV2 ds = Make308V2(10.0f, 0.462f, 2650.0f * dope::math::FPS_TO_MPS);
    ds.mass_grains = 168.0f;
    DOPE_SetAmmoDatasetV2(&ds);
    ZeroConfig zero = {}; zero.zero_range_m = 600.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    for (int i = 0; i < 140; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000ull);
        f.baro_pressure_pa = 101325.0f; f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f; f.baro_valid = true; f.baro_humidity_valid = true;
        f.lrf_valid = true; f.lrf_range_m = 600.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution baseline; DOPE_GetSolution(&baseline);
    EXPECT_LT(std::fabs(baseline.hold_elevation_moa), 0.75f);
    for (int i = 0; i < 140; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(200 + i + 1) * 10000ull);
        f.baro_pressure_pa = 76000.0f; f.baro_temperature_c = 35.0f;
        f.baro_humidity = 0.10f; f.baro_valid = true; f.baro_humidity_valid = true;
        f.lrf_valid = true; f.lrf_range_m = 600.0f; f.lrf_timestamp_us = f.timestamp_us;
        DOPE_Update(&f);
    }
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    FiringSolution after; DOPE_GetSolution(&after);
    EXPECT_LT(std::fabs(after.hold_elevation_moa), 0.75f);
}

TEST_F(IntegrationTest, Reference308168gr600mAnd800mTightEnvelope) {
    AmmoDatasetV2 ds = Make308G7V2();
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 50.8f;
    DOPE_SetAmmoDatasetV2(&ds);
    DOPE_SetZeroConfig(&zero);

    auto runAtRange = [&](float range_m, uint64_t ts_start) {
        for (int i = 0; i < 100; ++i) {
            SensorFrame f = makeDefaultFrame(ts_start + (uint64_t)(i + 1) * 10000);
            f.lrf_valid = true; f.lrf_range_m = range_m; f.lrf_timestamp_us = f.timestamp_us;
            DOPE_Update(&f);
        }
        EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
        FiringSolution sol; DOPE_GetSolution(&sol);
        return sol;
    };

    FiringSolution s600 = runAtRange(600.0f, 0);
    EXPECT_NEAR(s600.hold_elevation_moa / kMoaPerMil, 4.22, 0.05);
    EXPECT_NEAR(s600.tof_ms / 1000.0, 0.86, 0.02);

    FiringSolution s800 = runAtRange(800.0f, 2000000);
    EXPECT_NEAR(s800.hold_elevation_moa / kMoaPerMil, 6.70, 0.06);
    EXPECT_NEAR(s800.tof_ms / 1000.0, 1.23, 0.02);
}

TEST_F(IntegrationTest, ExternalReferenceModeReducesDropAndTof) {
    AmmoDatasetV2 ds = Make308V2(10.0f, 0.462f, 2650.0f * dope::math::FPS_TO_MPS);
    ds.mass_grains = 168.0f; ds.length_mm = 31.0f;
    DOPE_SetAmmoDatasetV2(&ds);
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    DOPE_SetWindManual(0.0f, 0.0f);
    DOPE_SetExternalReferenceMode(false);

    auto runAt800 = [&](uint64_t start_ts_us) {
        for (int i = 0; i < 140; ++i) {
            SensorFrame f = makeDefaultFrame(start_ts_us + (uint64_t)(i + 1) * 10000ull);
            f.baro_pressure_pa = 101325.0f; f.baro_temperature_c = 15.0f;
            f.baro_humidity = 0.5f; f.baro_valid = true; f.baro_humidity_valid = true;
            f.lrf_valid = true; f.lrf_range_m = 800.0f; f.lrf_timestamp_us = f.timestamp_us;
            DOPE_Update(&f);
        }
        EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
        FiringSolution sol; DOPE_GetSolution(&sol);
        return sol;
    };

    FiringSolution legacy = runAt800(0ull);
    DOPE_SetExternalReferenceMode(true);
    FiringSolution external_mode = runAt800(2000000ull);
    EXPECT_LT(std::fabs(external_mode.hold_elevation_moa), std::fabs(legacy.hold_elevation_moa));
    EXPECT_LT(external_mode.tof_ms, legacy.tof_ms);
    EXPECT_GT(external_mode.velocity_at_target_ms, legacy.velocity_at_target_ms);
}

// ---------------------------------------------------------------------------
// Zoom encoder / FOV tests
// ---------------------------------------------------------------------------

TEST_F(IntegrationTest, EncoderNoReadingDefaultsToZeroFOV) {
    EXPECT_FLOAT_EQ(DOPE_GetHFOV(), 0.0f);
    EXPECT_FLOAT_EQ(DOPE_GetVFOV(), 0.0f);
}

TEST_F(IntegrationTest, EncoderAt8mmProducesExpectedFOV) {
    SensorFrame f = makeDefaultFrame(10000);
    f.encoder_focal_length_mm = 8.0f; f.encoder_valid = true;
    DOPE_Update(&f);
    EXPECT_NEAR(DOPE_GetHFOV(), 42.92f, 0.1f);
    EXPECT_NEAR(DOPE_GetVFOV(), 32.80f, 0.1f);
}

TEST_F(IntegrationTest, EncoderAt50mmProducesNarrowFOV) {
    SensorFrame f = makeDefaultFrame(20000);
    f.encoder_focal_length_mm = 50.0f; f.encoder_valid = true;
    DOPE_Update(&f);
    EXPECT_NEAR(DOPE_GetHFOV(), 7.19f, 0.1f);
    EXPECT_NEAR(DOPE_GetVFOV(), 5.39f, 0.1f);
}

TEST_F(IntegrationTest, EncoderFOVDecreasesWithLongerFocalLength) {
    SensorFrame f8 = makeDefaultFrame(10000);
    f8.encoder_focal_length_mm = 8.0f; f8.encoder_valid = true;
    DOPE_Update(&f8);
    float hfov_8mm = DOPE_GetHFOV();
    SensorFrame f50 = makeDefaultFrame(20000);
    f50.encoder_focal_length_mm = 50.0f; f50.encoder_valid = true;
    DOPE_Update(&f50);
    EXPECT_GT(hfov_8mm, DOPE_GetHFOV());
}

TEST_F(IntegrationTest, EncoderNotValidDoesNotChangeFOV) {
    SensorFrame fValid = makeDefaultFrame(10000);
    fValid.encoder_focal_length_mm = 25.0f; fValid.encoder_valid = true;
    DOPE_Update(&fValid);
    float hfov_after_valid = DOPE_GetHFOV();
    EXPECT_GT(hfov_after_valid, 0.0f);
    SensorFrame fInvalid = makeDefaultFrame(20000);
    fInvalid.encoder_valid = false; fInvalid.encoder_focal_length_mm = 8.0f;
    DOPE_Update(&fInvalid);
    EXPECT_FLOAT_EQ(DOPE_GetHFOV(), hfov_after_valid);
}

TEST_F(IntegrationTest, EncoderInvalidFocalLengthRetainsPreviousFOV) {
    SensorFrame fValid = makeDefaultFrame(10000);
    fValid.encoder_focal_length_mm = 16.0f; fValid.encoder_valid = true;
    DOPE_Update(&fValid);
    float prev_hfov = DOPE_GetHFOV();
    EXPECT_GT(prev_hfov, 0.0f);
    SensorFrame fBad = makeDefaultFrame(20000);
    fBad.encoder_focal_length_mm = 0.0f; fBad.encoder_valid = true;
    DOPE_Update(&fBad);
    EXPECT_FLOAT_EQ(DOPE_GetHFOV(), prev_hfov);
}

TEST_F(IntegrationTest, EncoderFOVResetOnInit) {
    SensorFrame f = makeDefaultFrame(10000);
    f.encoder_focal_length_mm = 20.0f; f.encoder_valid = true;
    DOPE_Update(&f);
    EXPECT_GT(DOPE_GetHFOV(), 0.0f);
    DOPE_Init();
    EXPECT_FLOAT_EQ(DOPE_GetHFOV(), 0.0f);
    EXPECT_FLOAT_EQ(DOPE_GetVFOV(), 0.0f);
}
