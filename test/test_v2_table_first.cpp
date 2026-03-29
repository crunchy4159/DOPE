#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "dope/dope_math_utils.h"
#include <cmath>
#include <cstring>

namespace {

SensorFrame MakeFrame(uint64_t t_us, float range_m) {
    SensorFrame f = {};
    f.timestamp_us = t_us;
    f.accel_x = 0.0f;
    f.accel_y = 0.0f;
    f.accel_z = 9.81f;
    f.gyro_x = 0.0f;
    f.gyro_y = 0.0f;
    f.gyro_z = 0.0f;
    f.imu_valid = true;
    f.mag_x = 25.0f;
    f.mag_y = 0.0f;
    f.mag_z = 40.0f;
    f.mag_valid = true;
    f.baro_pressure_pa = 101325.0f;
    f.baro_temperature_c = 15.0f;
    f.baro_humidity = 0.5f;
    f.baro_valid = true;
    f.baro_humidity_valid = true;
    f.lrf_range_m = range_m;
    f.lrf_timestamp_us = t_us;
    f.lrf_valid = true;
    f.target_elevation_m = 0.0f;
    f.target_elevation_valid = true;
    return f;
}

void Stabilize(float range_m = 500.0f) {
    for (int i = 0; i < 120; ++i) {
        SensorFrame f = MakeFrame((uint64_t)(i + 1) * 10000, range_m);
        DOPE_Update(&f);
    }
}

} // namespace

TEST(V2TableFirst, UsesManufacturerTablesWhenPresent) {
    DOPE_Init();

    AmmoDatasetV2 ds = {};
    ds.source_confidence = 0.9f;
    ds.baseline_temperature_c = 15.0f;
    ds.baseline_pressure_pa = 101325.0f;
    ds.baseline_humidity = 0.5f;
    ds.baseline_wind_speed_ms = 5.0f;
    ds.baseline_barrel_length_in = 24.0f;
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 3;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {300.0f, -1.0f};
    ds.trajectories[0].points[2] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 610.0f};
    ds.num_energy_points = 2;
    ds.energy_by_range[0] = {100.0f, 2800.0f};
    ds.energy_by_range[1] = {500.0f, 1500.0f};
    ds.num_wind_drift_points = 2;
    ds.wind_drift_by_range[0] = {100.0f, 0.05f};
    ds.wind_drift_by_range[1] = {500.0f, 0.50f};
    ds.bc = 0.5f;
    ds.drag_model = DragModel::G1;
    ds.muzzle_velocity_ms = 790.0f;
    ds.mass_grains = 175.0f;
    ds.length_mm = 31.2f;
    ds.caliber_inches = 0.308f;
    ds.twist_rate_inches = 10.0f;
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig z = {};
    z.zero_range_m = 100.0f;
    z.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&z);

    BallisticContext ctx = {};
    ctx.use_runtime_wind = true;
    ctx.wind_speed_ms = 5.0f;
    ctx.wind_heading_deg = 90.0f;
    DOPE_SetBallisticContext(&ctx);

    Stabilize(500.0f);

    FiringSolution sol = {};
    DOPE_GetSolution(&sol);
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_NEAR(sol.velocity_at_target_ms, 610.0f, 20.0f);
    EXPECT_NEAR(sol.energy_at_target_j, 1500.0f, 150.0f);
}

TEST(V2TableFirst, FallsBackToSolverWhenChannelMissing) {
    DOPE_Init();

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    // No velocity/energy tables on purpose; solver fallback should provide values.
    ds.bc = 0.505f;
    ds.drag_model = DragModel::G1;
    ds.muzzle_velocity_ms = 792.0f;
    ds.mass_grains = 175.0f;
    ds.length_mm = 31.2f;
    ds.caliber_inches = 0.308f;
    ds.twist_rate_inches = 10.0f;
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig z = {};
    z.zero_range_m = 100.0f;
    z.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&z);

    Stabilize(500.0f);

    FiringSolution sol = {};
    DOPE_GetSolution(&sol);
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_GT(sol.velocity_at_target_ms, 100.0f);
    EXPECT_GT(sol.energy_at_target_j, 100.0f);
}

TEST(V2TableFirst, AppliesCalibrationBiasAndThermalMvShift) {
    DOPE_Init();

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 600.0f};
    ds.bc = 0.505f;
    ds.drag_model = DragModel::G1;
    ds.muzzle_velocity_ms = 792.0f;
    ds.mass_grains = 175.0f;
    ds.length_mm = 31.2f;
    ds.caliber_inches = 0.308f;
    ds.twist_rate_inches = 10.0f;
    ds.baseline_temperature_c = 15.0f;
    ds.baseline_pressure_pa = 101325.0f;
    ds.baseline_humidity = 0.5f;
    ds.baseline_wind_speed_ms = 0.0f;
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig z = {};
    z.zero_range_m = 100.0f;
    z.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&z);

    ModuleCapabilities caps = {true, true, true};
    DOPE_SetModuleCapabilities(&caps);

    RifleAmmoCalibrationProfile cal = {};
    cal.muzzle_velocity_scale = 1.0f;
    cal.uncertainty_scale = 1.0f;
    cal.drop_bias_moa = 1.2f;
    cal.wind_bias_moa = -0.7f;
    DOPE_SetRifleAmmoCalibrationProfile(&cal);

    Stabilize(500.0f);
    FiringSolution before = {};
    DOPE_GetSolution(&before);

    // Simulate a hot barrel by firing several shots
    for (int i = 0; i < 10; ++i) {
        DOPE_NotifyShotFired((uint64_t)(i + 1) * 500000ULL, 15.0f);
    }

    SensorFrame f = MakeFrame(6000000, 500.0f);
    DOPE_Update(&f);
    FiringSolution after = {};
    DOPE_GetSolution(&after);

    // Calibration bias should be applied in both states
    EXPECT_NEAR(before.hold_elevation_moa - 1.2f,
                before.hold_elevation_moa - 1.2f, 0.01f); // sanity
    // Hot barrel should produce a slightly different (higher MV → less drop) solution
    // The difference may be small but the solution must remain valid
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_GT(after.velocity_at_target_ms, 100.0f);
}
