/**
 * @file test_integration.cpp
 * @brief End-to-end integration tests for the BCE API.
 *
 * Tests the full pipeline: init → set profile → set zero → feed sensors → get solution.
 */

#include <gtest/gtest.h>
#include "bce/bce_api.h"
#include "bce/bce_config.h"
#include "bce/bce_math_utils.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr float kMoaPerMil = 3.43774677f;
}

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        BCE_Init();
    }

    SensorFrame makeDefaultFrame(uint64_t timestamp_us) {
        SensorFrame f;
        std::memset(&f, 0, sizeof(f));
        f.timestamp_us = timestamp_us;

        // Device flat, stationary
        f.accel_x = 0.0f;
        f.accel_y = 0.0f;
        f.accel_z = 9.81f;
        f.gyro_x = 0.0f;
        f.gyro_y = 0.0f;
        f.gyro_z = 0.0f;
        f.imu_valid = true;

        // Standard atmosphere
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

    void setDefault308BulletProfile(float twist_rate_inches = 10.0f) {
        BulletProfile bullet = {};
        bullet.bc = 0.505f;
        bullet.drag_model = DragModel::G1;
        bullet.muzzle_velocity_ms = 792.0f;
        bullet.mass_grains = 175.0f;
        bullet.caliber_inches = 0.308f;
        bullet.twist_rate_inches = twist_rate_inches;
        BCE_SetBulletProfile(&bullet);
    }

    void setZero100m(float sight_height_mm = 38.1f) {
        ZeroConfig zero = {};
        zero.zero_range_m = 100.0f;
        zero.sight_height_mm = sight_height_mm;
        BCE_SetZeroConfig(&zero);
    }
};

// After init, mode should be IDLE
TEST_F(IntegrationTest, InitialModeIsIdle) {
    EXPECT_EQ(BCE_GetMode(), BCE_Mode::IDLE);
}

// Without bullet profile, should fault
TEST_F(IntegrationTest, NoBulletFaults) {
    SensorFrame f = makeDefaultFrame(1000);
    f.lrf_valid = true;
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = 1000;

    // Feed enough frames for AHRS stability
    for (int i = 0; i < 100; ++i) {
        f.timestamp_us = (uint64_t)(i + 1) * 10000;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::FAULT);
    EXPECT_NE(BCE_GetFaultFlags() & BCE_Fault::NO_BULLET, 0u);
}

// With full configuration, mode should be SOLUTION_READY
TEST_F(IntegrationTest, FullConfigProducesSolution) {
    // Set bullet profile
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.length_mm = 31.2f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    // Set zero
    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    // Feed sensor frames with LRF
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    // Get solution
    FiringSolution sol;
    BCE_GetSolution(&sol);

    EXPECT_EQ(sol.solution_mode, static_cast<uint32_t>(BCE_Mode::SOLUTION_READY));
    EXPECT_FLOAT_EQ(sol.range_m, 500.0f);
    EXPECT_GT(sol.tof_ms, 0.0f);
    EXPECT_GT(sol.velocity_at_target_ms, 0.0f);
    EXPECT_GT(sol.energy_at_target_j, 0.0f);
    EXPECT_GT(sol.air_density_kgm3, 0.0f);
}

TEST_F(IntegrationTest, RealtimeSolutionMatchesPrimaryHolds) {
    setDefault308BulletProfile();
    setZero100m();

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    FiringSolution full = {};
    RealtimeSolution rt = {};
    BCE_GetSolution(&full);
    BCE_GetRealtimeSolution(&rt);

    EXPECT_EQ(rt.solution_mode, full.solution_mode);
    EXPECT_EQ(rt.fault_flags, full.fault_flags);
    EXPECT_EQ(rt.defaults_active, full.defaults_active);
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
    } else {
        EXPECT_FALSE(rt.uncertainty_valid);
        EXPECT_FLOAT_EQ(rt.uncertainty_radius_moa, 0.0f);
    }
}

// .308 Win 150gr Federal Fusion reference-style scenario at 500 yd
TEST_F(IntegrationTest, FederalFusion150gr500ydReferenceEnvelope) {

    BulletProfile bullet = {};
    bullet.bc = 0.414f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 2820.0f * bce::math::FPS_TO_MPS;
    bullet.mass_grains = 150.0f;
    bullet.length_mm = 28.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 91.44f;      // 100 yd
    zero.sight_height_mm = 38.1f;    // 1.5 in
    BCE_SetZeroConfig(&zero);

    // Federal reference conditions: 59 F, sea level, no wind
    BCE_SetWindManual(0.0f, 0.0f);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 457.2f; // 500 yd
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    FiringSolution sol;
    BCE_GetSolution(&sol);

    const float velocity_fps = sol.velocity_at_target_ms * bce::math::MPS_TO_FPS;

    // Published calculators vary with exact drag fit and implementation details.
    // Keep this as a realistic envelope centered near ~1821 fps.
    EXPECT_GT(velocity_fps, 1650.0f);
    EXPECT_LT(velocity_fps, 2000.0f);
}

// For the shipped GUI preset pair at 500 yd:
// .223 Rem 55gr (high MV) is expected to require slightly less elevation hold
// than .308 Win 175gr, despite the .308's higher BC.
TEST_F(IntegrationTest, PresetPair223Vs308HoldOrderingAt500Yd) {
    auto solve_at_500yd = [&](const BulletProfile& bullet) {
        BCE_Init();
        BCE_SetBulletProfile(&bullet);

        ZeroConfig zero = {};
        zero.zero_range_m = 91.44f;   // 100 yd
        zero.sight_height_mm = 38.1f; // 1.5 in
        BCE_SetZeroConfig(&zero);
        BCE_SetWindManual(0.0f, 0.0f);

        for (int i = 0; i < 120; ++i) {
            SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
            f.lrf_valid = true;
            f.lrf_range_m = 457.2f; // 500 yd
            f.lrf_timestamp_us = f.timestamp_us;
            BCE_Update(&f);
        }

        EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

        FiringSolution sol = {};
        BCE_GetSolution(&sol);
        return sol;
    };

    BulletProfile p308 = {};
    p308.bc = 0.505f;
    p308.drag_model = DragModel::G1;
    p308.muzzle_velocity_ms = 792.0f;
    p308.barrel_length_in = 24.0f;
    p308.reference_barrel_length_in = 24.0f; // .308 Win SAAMI reference
    p308.mv_adjustment_factor = 25.0f;
    p308.mass_grains = 175.0f;
    p308.length_mm = 31.2f;
    p308.caliber_inches = 0.308f;
    p308.twist_rate_inches = 10.0f;

    BulletProfile p223 = {};
    p223.bc = 0.245f;
    p223.drag_model = DragModel::G1;
    p223.muzzle_velocity_ms = 990.0f;   // measured from 24" SAAMI test barrel
    p223.barrel_length_in = 20.0f;
    p223.reference_barrel_length_in = 24.0f; // .223 Rem SAAMI reference
    p223.mv_adjustment_factor = 25.0f;
    p223.mass_grains = 55.0f;
    p223.length_mm = 19.0f;
    p223.caliber_inches = 0.223f;
    p223.twist_rate_inches = 12.0f;

    const FiringSolution sol_308 = solve_at_500yd(p308);
    const FiringSolution sol_223 = solve_at_500yd(p223);

    EXPECT_GT(sol_308.hold_elevation_moa, sol_223.hold_elevation_moa);
    EXPECT_GT(sol_308.velocity_at_target_ms, sol_223.velocity_at_target_ms);
}

// Coriolis should be disabled without latitude
TEST_F(IntegrationTest, CoriolisDisabledWithoutLatitude) {
    setDefault308BulletProfile();
    setZero100m();

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 1000.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_NE(BCE_GetDiagFlags() & BCE_Diag::CORIOLIS_DISABLED, 0u);

    FiringSolution sol;
    BCE_GetSolution(&sol);
    EXPECT_FLOAT_EQ(sol.coriolis_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(sol.coriolis_elevation_moa, 0.0f);
}

// LRF staleness should transition away from SOLUTION_READY
TEST_F(IntegrationTest, StaleLRFCausesFault) {
    setDefault308BulletProfile();
    setZero100m();

    // Feed frames with LRF
    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }
    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    // Now feed frames WITHOUT LRF, with timestamps far in the future
    // to trigger staleness
    uint64_t future_time = 100 * 10000 + BCE_LRF_STALE_US + 1000000;
    for (int i = 0; i < 10; ++i) {
        SensorFrame f = makeDefaultFrame(future_time + (uint64_t)i * 10000);
        f.lrf_valid = false;
        BCE_Update(&f);
    }

    // Should no longer be SOLUTION_READY
    EXPECT_NE(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
}

// Timestamp arithmetic near uint64 max should not falsely mark fresh LRF as stale
TEST_F(IntegrationTest, LRFStaleCheckHandlesTimestampNearWrap) {
    setDefault308BulletProfile();
    setZero100m();

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }
    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    constexpr uint64_t near_wrap_lrf_ts = std::numeric_limits<uint64_t>::max() - 100;

    SensorFrame near_wrap = makeDefaultFrame(near_wrap_lrf_ts);
    near_wrap.lrf_valid = true;
    near_wrap.lrf_range_m = 500.0f;
    near_wrap.lrf_timestamp_us = near_wrap.timestamp_us;
    BCE_Update(&near_wrap);

    SensorFrame shortly_after = makeDefaultFrame(near_wrap_lrf_ts + 50);
    shortly_after.lrf_valid = false;
    BCE_Update(&shortly_after);

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
    EXPECT_EQ(BCE_GetDiagFlags() & BCE_Diag::LRF_STALE, 0u);
}

// Default overrides should be reflected in solution
TEST_F(IntegrationTest, DefaultOverridesApplied) {
    BCE_DefaultOverrides ovr = {};
    ovr.use_altitude = true;
    ovr.altitude_m = 1500.0f;
    ovr.use_temperature = true;
    ovr.temperature_c = 30.0f;
    BCE_SetDefaultOverrides(&ovr);

    // Check that altitude default flag is cleared (since we set an override)
    uint32_t flags = BCE_GetDiagFlags();
    EXPECT_EQ(flags & BCE_Diag::DEFAULT_ALTITUDE, 0u);
}

// Invalid zero config should produce ZERO_UNSOLVABLE hard fault
TEST_F(IntegrationTest, InvalidZeroRangeFaults) {
    setDefault308BulletProfile();

    ZeroConfig zero = {};
    zero.zero_range_m = 0.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::FAULT);
    EXPECT_NE(BCE_GetFaultFlags() & BCE_Fault::ZERO_UNSOLVABLE, 0u);
}

// Wind default overrides should behave like startup manual wind
TEST_F(IntegrationTest, WindDefaultOverrideAffectsSolution) {
    setDefault308BulletProfile(0.0f); // isolate wind contribution
    setZero100m();

    BCE_DefaultOverrides ovr = {};
    ovr.use_wind = true;
    ovr.wind_speed_ms = 10.0f;
    ovr.wind_heading_deg = 90.0f;
    BCE_SetDefaultOverrides(&ovr);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
    EXPECT_EQ(BCE_GetDiagFlags() & BCE_Diag::DEFAULT_WIND, 0u);

    FiringSolution sol;
    BCE_GetSolution(&sol);
    EXPECT_NE(sol.hold_windage_moa, 0.0f);
}

// Crosswind sign convention: wind from the right should require right hold (+MOA).
TEST_F(IntegrationTest, WindFromRightRequiresRightHold) {
    setDefault308BulletProfile(0.0f); // isolate wind contribution
    setZero100m();

    BCE_SetWindManual(10.0f, 90.0f); // wind from right (east) when firing north

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    FiringSolution sol = {};
    BCE_GetSolution(&sol);
    EXPECT_GT(sol.hold_windage_moa, 0.0f);
}

// Null calibration pointers should be accepted safely
TEST_F(IntegrationTest, NullCalibrationInputsAreSafe) {
    BCE_SetIMUBias(nullptr, nullptr);
    BCE_SetMagCalibration(nullptr, nullptr);

    setDefault308BulletProfile();

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
}

// Boresight + reticle offsets should add directly to holds when cant is near zero
TEST_F(IntegrationTest, MechanicalOffsetsShiftHolds) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 0.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    FiringSolution base;
    BCE_GetSolution(&base);

    BoresightOffset boresight = {};
    boresight.vertical_moa = 1.5f;
    boresight.horizontal_moa = -2.0f;
    BCE_SetBoresightOffset(&boresight);
    BCE_SetReticleMechanicalOffset(0.5f, 1.0f);

    for (int i = 0; i < 10; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(101 + i) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    FiringSolution shifted;
    BCE_GetSolution(&shifted);

    EXPECT_NEAR(shifted.hold_elevation_moa - base.hold_elevation_moa, 2.0f, 0.05f);
    EXPECT_NEAR(shifted.hold_windage_moa - base.hold_windage_moa, -1.0f, 0.05f);
}

// Disturbed magnetometer should set MAG_SUPPRESSED diagnostic while still solving
TEST_F(IntegrationTest, MagDisturbanceSetsDiagnostic) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.mag_valid = true;
        f.mag_x = 200.0f;
        f.mag_y = 200.0f;
        f.mag_z = 200.0f;
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
    EXPECT_NE(BCE_GetDiagFlags() & BCE_Diag::MAG_SUPPRESSED, 0u);
}

// Baro calibration should move measured atmosphere toward standard reference
TEST_F(IntegrationTest, BaroCalibrationRaisesLowPressureDensity) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.baro_pressure_pa = 90000.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    FiringSolution before;
    BCE_GetSolution(&before);

    BCE_CalibrateBaro();

    for (int i = 0; i < 20; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(101 + i) * 10000);
        f.baro_pressure_pa = 90000.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    FiringSolution after;
    BCE_GetSolution(&after);

    EXPECT_GT(after.air_density_kgm3, before.air_density_kgm3);
}

// Magnetic declination should offset reported true heading
TEST_F(IntegrationTest, DeclinationOffsetsHeading) {
    BCE_SetMagDeclination(12.0f);

    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    FiringSolution sol;
    BCE_GetSolution(&sol);
    EXPECT_NEAR(sol.heading_deg_true, 12.0f, 0.5f);
}

// Large forward and backward timestamp discontinuities should not break solving
TEST_F(IntegrationTest, TimestampJumpsAndRollbackRemainDeterministic) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 80; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    SensorFrame jump = makeDefaultFrame(10ull * 1000ull * 1000ull);
    jump.lrf_valid = true;
    jump.lrf_range_m = 500.0f;
    jump.lrf_timestamp_us = jump.timestamp_us;
    BCE_Update(&jump);

    SensorFrame rollback = makeDefaultFrame(500000ull);
    rollback.lrf_valid = true;
    rollback.lrf_range_m = 500.0f;
    rollback.lrf_timestamp_us = rollback.timestamp_us;
    BCE_Update(&rollback);

    for (int i = 0; i < 40; ++i) {
        SensorFrame f = makeDefaultFrame(600000ull + (uint64_t)i * 10000ull);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
    FiringSolution sol;
    BCE_GetSolution(&sol);
    EXPECT_TRUE(std::isfinite(sol.hold_elevation_moa));
    EXPECT_TRUE(std::isfinite(sol.hold_windage_moa));
}

// Non-finite IMU inputs should be flagged invalid without crashing
TEST_F(IntegrationTest, NaNImuInputFlagsSensorInvalid) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    SensorFrame bad = makeDefaultFrame(2000000ull);
    bad.accel_x = std::numeric_limits<float>::quiet_NaN();
    bad.lrf_valid = true;
    bad.lrf_range_m = 500.0f;
    bad.lrf_timestamp_us = bad.timestamp_us;
    BCE_Update(&bad);

    EXPECT_NE(BCE_GetFaultFlags() & BCE_Fault::SENSOR_INVALID, 0u);
}

// Non-finite LRF range should be rejected and flagged invalid
TEST_F(IntegrationTest, InfRangeRejectedAndFlagged) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 100; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = true;
        f.lrf_range_m = std::numeric_limits<float>::infinity();
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::FAULT);
    EXPECT_NE(BCE_GetFaultFlags() & BCE_Fault::NO_RANGE, 0u);
    EXPECT_NE(BCE_GetFaultFlags() & BCE_Fault::SENSOR_INVALID, 0u);
}

// Rapid alternation of valid and missing LRF samples should recover deterministically
TEST_F(IntegrationTest, RapidRangeValidityTransitionsRecover) {
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 200; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000);
        f.lrf_valid = (i % 2 == 0);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    SensorFrame final_good = makeDefaultFrame(3000000ull);
    final_good.lrf_valid = true;
    final_good.lrf_range_m = 500.0f;
    final_good.lrf_timestamp_us = final_good.timestamp_us;
    BCE_Update(&final_good);

    EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    FiringSolution sol;
    BCE_GetSolution(&sol);
    EXPECT_TRUE(std::isfinite(sol.hold_elevation_moa));
    EXPECT_TRUE(std::isfinite(sol.hold_windage_moa));
}

// Atmospheric changes should trigger zero recomputation when deltas are significant
TEST_F(IntegrationTest, AtmosphericDeltaTriggersZeroRecompute) {

    BulletProfile bullet = {};
    bullet.bc = 0.462f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 2650.0f * bce::math::FPS_TO_MPS;
    bullet.mass_grains = 168.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 600.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    for (int i = 0; i < 140; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(i + 1) * 10000ull);
        f.baro_pressure_pa = 101325.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;
        f.lrf_valid = true;
        f.lrf_range_m = 600.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    ASSERT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);

    FiringSolution baseline;
    BCE_GetSolution(&baseline);
    EXPECT_LT(std::fabs(baseline.hold_elevation_moa), 0.75f);

    for (int i = 0; i < 140; ++i) {
        SensorFrame f = makeDefaultFrame((uint64_t)(200 + i + 1) * 10000ull);
        f.baro_pressure_pa = 76000.0f;
        f.baro_temperature_c = 35.0f;
        f.baro_humidity = 0.10f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;
        f.lrf_valid = true;
        f.lrf_range_m = 600.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        BCE_Update(&f);
    }

    ASSERT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
    FiringSolution after;
    BCE_GetSolution(&after);

    // If zero is recomputed for the new atmosphere, hold at zero range should remain near zero.
    EXPECT_LT(std::fabs(after.hold_elevation_moa), 0.75f);
}

// .308 Win 168gr @ 2650 fps benchmark envelope at 600m and 800m
TEST_F(IntegrationTest, Reference308168gr600mAnd800mTightEnvelope) {
    BulletProfile bullet = {};
    bullet.bc = 0.508f;
    bullet.drag_model = DragModel::G7;
    bullet.muzzle_velocity_ms = 838.2f;
    bullet.mass_grains = 168.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 50.8f;
    BCE_SetZeroConfig(&zero);

    auto runAtRange = [&](float range_m, uint64_t ts_start) {
        for (int i = 0; i < 100; ++i) {
            SensorFrame f = makeDefaultFrame(ts_start + (uint64_t)(i + 1) * 10000);
            f.lrf_valid = true;
            f.lrf_range_m = range_m;
            f.lrf_timestamp_us = f.timestamp_us;
            BCE_Update(&f);
        }

        EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
        FiringSolution sol;
        BCE_GetSolution(&sol);
        return sol;
    };

    // Check results at 600m
    FiringSolution s600 = runAtRange(600.0f, 0);
    double drop600_mil = s600.hold_elevation_moa / kMoaPerMil;
    double tof600_s = s600.tof_ms / 1000.0;

    EXPECT_NEAR(drop600_mil, 4.73, 0.05);
    EXPECT_NEAR(tof600_s, 0.86, 0.02);

    // Check results at 800m
    FiringSolution s800 = runAtRange(800.0f, 2000000);
    double drop800_mil = s800.hold_elevation_moa / kMoaPerMil;
    double tof800_s = s800.tof_ms / 1000.0;

    EXPECT_NEAR(drop800_mil, 7.24, 0.05);
    EXPECT_NEAR(tof800_s, 1.23, 0.02);
}

// External-reference mode should be opt-in and reduce modeled drag effects
TEST_F(IntegrationTest, ExternalReferenceModeReducesDropAndTof) {

    BulletProfile bullet = {};
    bullet.bc = 0.462f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 2650.0f * bce::math::FPS_TO_MPS;
    bullet.mass_grains = 168.0f;
    bullet.length_mm = 31.0f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    BCE_SetWindManual(0.0f, 0.0f);
    BCE_SetExternalReferenceMode(false);

    auto runAt800 = [&](uint64_t start_ts_us) {
        for (int i = 0; i < 140; ++i) {
            SensorFrame f = makeDefaultFrame(start_ts_us + (uint64_t)(i + 1) * 10000ull);
            f.baro_pressure_pa = 101325.0f;
            f.baro_temperature_c = 15.0f;
            f.baro_humidity = 0.5f;
            f.baro_valid = true;
            f.baro_humidity_valid = true;
            f.lrf_valid = true;
            f.lrf_range_m = 800.0f;
            f.lrf_timestamp_us = f.timestamp_us;
            BCE_Update(&f);
        }
        EXPECT_EQ(BCE_GetMode(), BCE_Mode::SOLUTION_READY);
        FiringSolution sol;
        BCE_GetSolution(&sol);
        return sol;
    };

    FiringSolution legacy = runAt800(0ull);

    BCE_SetExternalReferenceMode(true);
    FiringSolution external_mode = runAt800(2000000ull);

    EXPECT_LT(std::fabs(external_mode.hold_elevation_moa), std::fabs(legacy.hold_elevation_moa));
    EXPECT_LT(external_mode.tof_ms, legacy.tof_ms);
    EXPECT_GT(external_mode.velocity_at_target_ms, legacy.velocity_at_target_ms);
}

// ---------------------------------------------------------------------------
// Zoom encoder / FOV tests — SRS §7.5
// ---------------------------------------------------------------------------

// After init, with no encoder frame received, both FOV values must be zero.
TEST_F(IntegrationTest, EncoderNoReadingDefaultsToZeroFOV) {
    EXPECT_FLOAT_EQ(BCE_GetHFOV(), 0.0f);
    EXPECT_FLOAT_EQ(BCE_GetVFOV(), 0.0f);
}

// A valid encoder reading at 8 mm focal length should produce the expected
// horizontal and vertical FOV for the IMX477 (1/2.3" sensor).
//   HFOV = 2 * atan(6.287 / 16) ≈ 42.92°
//   VFOV = 2 * atan(4.712 / 16) ≈ 32.80°
TEST_F(IntegrationTest, EncoderAt8mmProducesExpectedFOV) {
    SensorFrame f = makeDefaultFrame(10000);
    f.encoder_focal_length_mm = 8.0f;
    f.encoder_valid = true;
    BCE_Update(&f);

    EXPECT_NEAR(BCE_GetHFOV(), 42.92f, 0.1f);
    EXPECT_NEAR(BCE_GetVFOV(), 32.80f, 0.1f);
}

// At maximum zoom (50 mm) the FOV should be much narrower.
//   HFOV = 2 * atan(6.287 / 100) ≈ 7.19°
//   VFOV = 2 * atan(4.712 / 100) ≈ 5.39°
TEST_F(IntegrationTest, EncoderAt50mmProducesNarrowFOV) {
    SensorFrame f = makeDefaultFrame(20000);
    f.encoder_focal_length_mm = 50.0f;
    f.encoder_valid = true;
    BCE_Update(&f);

    EXPECT_NEAR(BCE_GetHFOV(), 7.19f, 0.1f);
    EXPECT_NEAR(BCE_GetVFOV(), 5.39f, 0.1f);
}

// Longer focal length must yield smaller FOV (monotone relationship).
TEST_F(IntegrationTest, EncoderFOVDecreasesWithLongerFocalLength) {
    SensorFrame f8 = makeDefaultFrame(10000);
    f8.encoder_focal_length_mm = 8.0f;
    f8.encoder_valid = true;
    BCE_Update(&f8);
    float hfov_8mm = BCE_GetHFOV();
    float vfov_8mm = BCE_GetVFOV();

    SensorFrame f50 = makeDefaultFrame(20000);
    f50.encoder_focal_length_mm = 50.0f;
    f50.encoder_valid = true;
    BCE_Update(&f50);
    float hfov_50mm = BCE_GetHFOV();
    float vfov_50mm = BCE_GetVFOV();

    EXPECT_GT(hfov_8mm, hfov_50mm);
    EXPECT_GT(vfov_8mm, vfov_50mm);
}

// An encoder frame with encoder_valid = false must NOT change the stored FOV.
TEST_F(IntegrationTest, EncoderNotValidDoesNotChangeFOV) {
    // First establish a valid reading
    SensorFrame fValid = makeDefaultFrame(10000);
    fValid.encoder_focal_length_mm = 25.0f;
    fValid.encoder_valid = true;
    BCE_Update(&fValid);
    float hfov_after_valid = BCE_GetHFOV();
    EXPECT_GT(hfov_after_valid, 0.0f);

    // Now send a frame with encoder_valid = false — FOV must be unchanged
    SensorFrame fInvalid = makeDefaultFrame(20000);
    fInvalid.encoder_valid = false;
    fInvalid.encoder_focal_length_mm = 8.0f; // different value, should be ignored
    BCE_Update(&fInvalid);

    EXPECT_FLOAT_EQ(BCE_GetHFOV(), hfov_after_valid);
}

// A focal length below the minimum threshold with encoder_valid = true must
// retain the previously computed FOV.
TEST_F(IntegrationTest, EncoderInvalidFocalLengthRetainsPreviousFOV) {
    SensorFrame fValid = makeDefaultFrame(10000);
    fValid.encoder_focal_length_mm = 16.0f;
    fValid.encoder_valid = true;
    BCE_Update(&fValid);
    float prev_hfov = BCE_GetHFOV();
    EXPECT_GT(prev_hfov, 0.0f);

    // Sub-minimum focal length (< BCE_ENCODER_MIN_FOCAL_LENGTH_MM)
    SensorFrame fBad = makeDefaultFrame(20000);
    fBad.encoder_focal_length_mm = 0.0f;
    fBad.encoder_valid = true;
    BCE_Update(&fBad);

    EXPECT_FLOAT_EQ(BCE_GetHFOV(), prev_hfov);
}

// BCE_Init() must reset FOV to zero even after valid readings have been stored.
TEST_F(IntegrationTest, EncoderFOVResetOnInit) {
    SensorFrame f = makeDefaultFrame(10000);
    f.encoder_focal_length_mm = 20.0f;
    f.encoder_valid = true;
    BCE_Update(&f);
    EXPECT_GT(BCE_GetHFOV(), 0.0f);

    BCE_Init();
    EXPECT_FLOAT_EQ(BCE_GetHFOV(), 0.0f);
    EXPECT_FLOAT_EQ(BCE_GetVFOV(), 0.0f);
}
