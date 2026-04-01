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
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.80f};
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
    EXPECT_NEAR(sol.tof_ms, 800.0f, 80.0f);
}

TEST(V2TableFirst, TrajectoryPointsMatchDatasetAtBaselineConditions) {
    DOPE_Init();

    AmmoDatasetV2 ds = {};
    ds.source_confidence = 1.0f;
    ds.baseline_temperature_c = 15.0f;
    ds.baseline_pressure_pa = 101325.0f;
    ds.baseline_humidity = 0.5f;
    ds.baseline_wind_speed_ms = 5.0f;
    ds.baseline_barrel_length_in = 24.0f;
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 3;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {200.0f, -0.40f};
    ds.trajectories[0].points[2] = {300.0f, -1.20f};
    ds.num_velocity_points = 3;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {200.0f, 700.0f};
    ds.velocity_by_range[2] = {300.0f, 650.0f};
    ds.num_energy_points = 3;
    ds.energy_by_range[0] = {100.0f, 2800.0f};
    ds.energy_by_range[1] = {200.0f, 2200.0f};
    ds.energy_by_range[2] = {300.0f, 1800.0f};
    ds.num_tof_points = 3;
    ds.tof_by_range[0] = {100.0f, 0.13f};
    ds.tof_by_range[1] = {200.0f, 0.29f};
    ds.tof_by_range[2] = {300.0f, 0.48f};
    ds.num_wind_drift_points = 3;
    ds.wind_drift_by_range[0] = {100.0f, 0.03f};
    ds.wind_drift_by_range[1] = {200.0f, 0.09f};
    ds.wind_drift_by_range[2] = {300.0f, 0.20f};
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

    Stabilize(300.0f);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);

    TrajectoryPoint tp100 = {};
    TrajectoryPoint tp200 = {};
    TrajectoryPoint tp300 = {};
    ASSERT_TRUE(DOPE_GetTrajectoryPoint(100, &tp100));
    ASSERT_TRUE(DOPE_GetTrajectoryPoint(200, &tp200));
    ASSERT_TRUE(DOPE_GetTrajectoryPoint(300, &tp300));

    EXPECT_NEAR(tp100.drop_m, 0.0f, 1e-4f);
    EXPECT_NEAR(tp200.drop_m, -0.40f, 1e-4f);
    EXPECT_NEAR(tp300.drop_m, -1.20f, 1e-4f);
    EXPECT_NEAR(tp100.velocity_ms, 760.0f, 1e-3f);
    EXPECT_NEAR(tp200.velocity_ms, 700.0f, 1e-3f);
    EXPECT_NEAR(tp300.velocity_ms, 650.0f, 1e-3f);
    EXPECT_NEAR(tp100.tof_s, 0.13f, 1e-4f);
    EXPECT_NEAR(tp200.tof_s, 0.29f, 1e-4f);
    EXPECT_NEAR(tp300.tof_s, 0.48f, 1e-4f);
}

TEST(V2TableFirst, ShorterBarrelIncreasesDropAndTofWhenMvAdjMissing) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.source_confidence = 1.0f;
    ds.baseline_temperature_c = 15.0f;
    ds.baseline_pressure_pa = 101325.0f;
    ds.baseline_humidity = 0.5f;
    ds.baseline_barrel_length_in = 24.0f;
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 610.0f};
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.80f};
    ds.bc = 0.5f;
    ds.drag_model = DragModel::G1;
    ds.muzzle_velocity_ms = 790.0f;
    ds.mass_grains = 175.0f;
    ds.length_mm = 31.2f;
    ds.caliber_inches = 0.308f;
    ds.twist_rate_inches = 10.0f;
    // Intentionally unset/zero to verify engine fallback estimation path.
    ds.mv_adjustment_fps_per_in = 0.0f;
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig z = {};
    z.zero_range_m = 100.0f;
    z.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&z);

    GunProfile long_gun = {};
    long_gun.barrel_length_in = 24.0f;
    long_gun.reference_barrel_length_in = 24.0f;
    long_gun.barrel_material = BarrelMaterial::CMV;
    DOPE_SetGunProfile(&long_gun);
    Stabilize(500.0f);
    FiringSolution long_sol = {};
    DOPE_GetSolution(&long_sol);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);

    GunProfile short_gun = long_gun;
    short_gun.barrel_length_in = 18.0f;
    DOPE_SetGunProfile(&short_gun);
    Stabilize(500.0f);
    FiringSolution short_sol = {};
    DOPE_GetSolution(&short_sol);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);

    // Shorter barrel -> lower impact velocity, longer TOF, more elevation hold.
    EXPECT_LT(short_sol.velocity_at_target_ms, long_sol.velocity_at_target_ms);
    EXPECT_GT(short_sol.tof_ms, long_sol.tof_ms);
    EXPECT_GT(short_sol.hold_elevation_moa, long_sol.hold_elevation_moa);
}

TEST(V2TableFirst, BarrelMvProfileSupportsNonLinearPerInchGain) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.source_confidence = 1.0f;
    ds.baseline_temperature_c = 15.0f;
    ds.baseline_pressure_pa = 101325.0f;
    ds.baseline_humidity = 0.5f;
    ds.baseline_barrel_length_in = 24.0f;
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 610.0f};
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.80f};
    ds.bc = 0.5f;
    ds.drag_model = DragModel::G1;
    ds.muzzle_velocity_ms = 790.0f;
    ds.mass_grains = 175.0f;
    ds.length_mm = 31.2f;
    ds.caliber_inches = 0.308f;
    ds.twist_rate_inches = 10.0f;
    // Piecewise non-linear barrel MV calibration:
    // 18->24 inches: +40 fps (6.67 fps/in), 24->30 inches: +20 fps (3.33 fps/in).
    ds.num_barrel_mv_points = 3;
    ds.barrel_mv_by_length_in[0] = {18.0f, 775.368f}; // 2544 fps
    ds.barrel_mv_by_length_in[1] = {24.0f, 787.560f}; // 2584 fps
    ds.barrel_mv_by_length_in[2] = {30.0f, 793.656f}; // 2604 fps
    DOPE_SetAmmoDatasetV2(&ds);

    ZeroConfig z = {};
    z.zero_range_m = 100.0f;
    z.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&z);

    auto solve_at = [](float barrel_in) {
        GunProfile g = {};
        g.barrel_length_in = barrel_in;
        g.reference_barrel_length_in = 24.0f;
        g.barrel_material = BarrelMaterial::CMV;
        DOPE_SetGunProfile(&g);
        Stabilize(500.0f);
        FiringSolution s = {};
        DOPE_GetSolution(&s);
        return s;
    };

    const FiringSolution s18 = solve_at(18.0f);
    const FiringSolution s19 = solve_at(19.0f);
    const FiringSolution s24 = solve_at(24.0f);
    const FiringSolution s25 = solve_at(25.0f);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);

    const float dv_low = s19.velocity_at_target_ms - s18.velocity_at_target_ms;
    const float dv_high = s25.velocity_at_target_ms - s24.velocity_at_target_ms;
    EXPECT_GT(dv_low, dv_high);
}

TEST(V2TableFirst, DerivesMissingChannelsWithoutSolverFallback) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    // No velocity/energy tables on purpose; derive from TOF + mass.
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.85f};
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
    EXPECT_NEAR(sol.tof_ms, 850.0f, 80.0f);
    EXPECT_GT(sol.velocity_at_target_ms, 400.0f);
    EXPECT_GT(sol.energy_at_target_j, 800.0f);
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
    ds.mv_thermal_slope = 1.5f; // fps/K
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

    // Hot barrel should increase retained velocity and reduce elevation hold.
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    EXPECT_GT(after.velocity_at_target_ms, before.velocity_at_target_ms);
    EXPECT_LT(after.hold_elevation_moa, before.hold_elevation_moa);
}

TEST(V2TableFirst, UsesPrecomputedUncertaintyInStrictMode) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 600.0f};
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.85f};
    ds.num_uncertainty_points = 2;
    ds.uncertainty_sigma_by_range[0] = {100.0f, 0.30f, 0.35f, 0.10f};
    ds.uncertainty_sigma_by_range[1] = {500.0f, 0.90f, 1.10f, 0.20f};
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
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    ASSERT_TRUE(sol.uncertainty_valid);
    EXPECT_NEAR(sol.sigma_elevation_moa, 0.90f, 1e-3f);
    EXPECT_NEAR(sol.sigma_windage_moa, 1.10f, 1e-3f);
    EXPECT_NEAR(sol.covariance_elev_wind, 0.20f * 0.90f * 1.10f, 1e-3f);
}

TEST(V2TableFirst, StrictModeCoriolisRemainsLatitudeDependent) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 600.0f};
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.85f};
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

    // Bias heading to ~east/west so sin(azimuth) is non-zero and coriolis shows up.
    DOPE_SetMagDeclination(90.0f);

    DOPE_SetLatitude(45.0f);
    Stabilize(500.0f);
    FiringSolution north = {};
    DOPE_GetSolution(&north);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);

    DOPE_SetLatitude(-45.0f);
    Stabilize(500.0f);
    FiringSolution south = {};
    DOPE_GetSolution(&south);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);

    EXPECT_GT(std::fabs(north.coriolis_windage_moa), 1e-4f);
    EXPECT_GT(std::fabs(south.coriolis_windage_moa), 1e-4f);
    EXPECT_LT(north.coriolis_windage_moa * south.coriolis_windage_moa, 0.0f);
}

TEST(V2TableFirst, StrictModeCoriolisUsesLiveSensorHeading) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    ds.num_velocity_points = 2;
    ds.velocity_by_range[0] = {100.0f, 760.0f};
    ds.velocity_by_range[1] = {500.0f, 600.0f};
    ds.num_tof_points = 2;
    ds.tof_by_range[0] = {100.0f, 0.12f};
    ds.tof_by_range[1] = {500.0f, 0.85f};
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
    DOPE_SetLatitude(45.0f);

    // Start with near north/south heading (small sin(azimuth) => small coriolis).
    DOPE_SetMagDeclination(0.0f);
    Stabilize(500.0f);
    FiringSolution north = {};
    DOPE_GetSolution(&north);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    const float north_abs = std::fabs(north.coriolis_windage_moa);

    // Rotate heading using live sensor fusion path; coriolis should update.
    DOPE_SetMagDeclination(90.0f);
    for (int i = 0; i < 30; ++i) {
        SensorFrame f = MakeFrame(2000000 + (uint64_t)i * 10000ULL, 500.0f);
        DOPE_Update(&f);
    }

    FiringSolution east = {};
    DOPE_GetSolution(&east);
    ASSERT_EQ(DOPE_GetMode(), DOPE_Mode::SOLUTION_READY);
    const float east_abs = std::fabs(east.coriolis_windage_moa);
    EXPECT_GT(east_abs, north_abs + 1e-4f);
}

TEST(V2TableFirst, StrictModeFailsWhenEssentialChannelsMissing) {
    DOPE_Init();
    ModuleCapabilities caps = {false, true, true};
    DOPE_SetModuleCapabilities(&caps);

    AmmoDatasetV2 ds = {};
    ds.num_trajectories = 1;
    ds.trajectories[0].zero_range_m = 100.0f;
    ds.trajectories[0].num_points = 2;
    ds.trajectories[0].points[0] = {100.0f, 0.0f};
    ds.trajectories[0].points[1] = {500.0f, -4.0f};
    // Intentionally omit velocity and TOF channels.
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
    EXPECT_EQ(DOPE_GetMode(), DOPE_Mode::FAULT);

    FiringSolution sol = {};
    DOPE_GetSolution(&sol);
    EXPECT_NE((sol.fault_flags & DOPE_Fault::INVALID_AMMO), 0u);
}
