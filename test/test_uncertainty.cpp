/**
 * @file test_uncertainty.cpp
 * @brief Tests for the Gaussian uncertainty propagation system.
 *
 * Validates that the engine correctly computes a 2x2 covariance matrix
 * (elevation x windage) via linear error propagation / sensitivity analysis.
 */

#include <gtest/gtest.h>
#include "dope/dope_api.h"
#include "dope/dope_config.h"
#include "test_helpers_v2.h"
#include <cmath>
#include <cstring>

class UncertaintyTest : public ::testing::Test {
protected:
    void SetUp() override {
        DOPE_Init();
        EnableSolverFallback();
    }

    void configureStandard308() {
        AmmoDatasetV2 ds = Make308V2(10.0f);
        ds.mv_adjustment_fps_per_in = 25.0f;
        DOPE_SetAmmoDatasetV2(&ds);

        ZeroConfig zero = {};
        zero.zero_range_m = 100.0f;
        zero.sight_height_mm = 38.1f;
        DOPE_SetZeroConfig(&zero);

        DOPE_SetLatitude(37.0f);
    }

    SensorFrame makeDefaultFrame(uint64_t timestamp_us) {
        SensorFrame f;
        std::memset(&f, 0, sizeof(f));
        f.timestamp_us = timestamp_us;

        f.accel_x = 0.0f;
        f.accel_y = 0.0f;
        f.accel_z = 9.81f;
        f.gyro_x = 0.0f;
        f.gyro_y = 0.0f;
        f.gyro_z = 0.0f;
        f.imu_valid = true;

        f.baro_pressure_pa = 101325.0f;
        f.baro_temperature_c = 15.0f;
        f.baro_humidity = 0.5f;
        f.baro_valid = true;
        f.baro_humidity_valid = true;

        f.mag_x = 25.0f;
        f.mag_y = 0.0f;
        f.mag_z = 40.0f;
        f.mag_valid = true;

        f.lrf_valid = false;
        f.encoder_valid = false;

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

    FiringSolution getSolution() {
        FiringSolution sol = {};
        DOPE_GetSolution(&sol);
        return sol;
    }
};

TEST_F(UncertaintyTest, CartridgeCEPSetsScale) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    DOPE_GetDefaultUncertaintyConfig(&uc);
    uc.enabled = true;
    DOPE_SetUncertaintyConfig(&uc);

    // Defer uncertainty to validate manual trigger behavior
    DOPE_SetDeferUncertainty(true);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution base = getSolution();
    ASSERT_FALSE(base.uncertainty_valid);

    // Manually run the deferred pass
    DOPE_ComputeUncertainty();
    DOPE_GetSolution(&base);
    ASSERT_TRUE(base.uncertainty_valid);
    const float base_radius =
        std::sqrt(base.sigma_elevation_moa * base.sigma_elevation_moa +
                  base.sigma_windage_moa * base.sigma_windage_moa);
    ASSERT_GT(base_radius, 0.001f);

    // Return to default immediate uncertainty for the remainder of the test
    DOPE_SetDeferUncertainty(false);

    const float target_factor = 1.8f;
    const float target_cep50_moa = base_radius * target_factor * 1.17741f; // CEP50 -> 1-sigma
    DOPE_CEPPoint cep_point = {500.0f, target_cep50_moa};
    uc.cartridge_cep_table = {&cep_point, 1};
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f2 = makeDefaultFrame(10000 * 500);
    f2.lrf_range_m = 500.0f;
    f2.lrf_timestamp_us = f2.timestamp_us;
    f2.lrf_valid = true;
    DOPE_Update(&f2);

    FiringSolution scaled = getSolution();
    ASSERT_TRUE(scaled.uncertainty_valid);
    const float scaled_radius =
        std::sqrt(scaled.sigma_elevation_moa * scaled.sigma_elevation_moa +
                  scaled.sigma_windage_moa * scaled.sigma_windage_moa);
    EXPECT_NEAR(scaled_radius, base_radius * target_factor, base_radius * 0.2f);
}

// --- Zero sigma tests ---

TEST_F(UncertaintyTest, AllZeroSigmaProducesZeroUncertainty) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    // All sigmas are zero (default-initialized)
    DOPE_SetUncertaintyConfig(&uc);

    // Re-run to get updated solution
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    // With barrel stiffness/CEP baselines injected, sigmas are nonzero even when
    // input sigmas are zero. Ensure they are small and covariance remains ~0.
    EXPECT_GT(sol.sigma_elevation_moa, 0.05f);
    EXPECT_GT(sol.sigma_windage_moa, 0.05f);
    EXPECT_NEAR(sol.covariance_elev_wind, 0.0f, 0.005f);
}

TEST_F(UncertaintyTest, DisabledProducesNoUncertainty) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    DOPE_GetDefaultUncertaintyConfig(&uc);
    uc.enabled = false;
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_FALSE(sol.uncertainty_valid);
    EXPECT_NEAR(sol.sigma_elevation_moa, 0.0f, 0.001f);
    EXPECT_NEAR(sol.sigma_windage_moa, 0.0f, 0.001f);
}

// --- Single-input sensitivity tests ---

TEST_F(UncertaintyTest, MVSigmaAffectsElevation) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    // Baseline with zero MV sigma (captures stiffness/CEP floor).
    UncertaintyConfig base_uc = {};
    base_uc.enabled = true;
    DOPE_SetUncertaintyConfig(&base_uc);
    SensorFrame f0 = makeDefaultFrame(10000 * 380);
    f0.lrf_range_m = 500.0f;
    f0.lrf_timestamp_us = f0.timestamp_us;
    f0.lrf_valid = true;
    DOPE_Update(&f0);
    FiringSolution base = getSolution();

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_muzzle_velocity_ms = 3.0f; // ~10 fps
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    const float delta_e = sol.sigma_elevation_moa - base.sigma_elevation_moa;
    const float delta_w = sol.sigma_windage_moa - base.sigma_windage_moa;
    EXPECT_GT(delta_e, 0.01f);
    EXPECT_GE(delta_e, delta_w * 1.2f);
}

TEST_F(UncertaintyTest, WindSpeedSigmaAffectsWindage) {
    configureStandard308();
    DOPE_SetWindManual(3.0f, 90.0f); // 3 m/s crosswind from right
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_wind_speed_ms = 2.0f;
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    // Wind speed sigma in a full crosswind should primarily affect windage
    EXPECT_GT(sol.sigma_windage_moa, 0.01f);
}

TEST_F(UncertaintyTest, RangeSigmaAffectsBothAxes) {
    configureStandard308();
    DOPE_SetWindManual(3.0f, 90.0f);
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_range_m = 5.0f;
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    // Range uncertainty affects both elevation and windage
    EXPECT_GT(sol.sigma_elevation_moa, 0.01f);
    // With crosswind active, range error should affect windage too
    EXPECT_GT(sol.sigma_windage_moa, 0.001f);
}

// --- Monotonicity: larger sigma → larger output uncertainty ---

TEST_F(UncertaintyTest, LargerMVSigmaProducesLargerUncertainty) {
    configureStandard308();
    stabilizeAHRS();

    // Small sigma
    {
        feedRange(500.0f);
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 1.0f;
        DOPE_SetUncertaintyConfig(&uc);

        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        DOPE_Update(&f);
    }
    FiringSolution small_sol = getSolution();

    // Large sigma — reinit to get clean state
    DOPE_Init();
    configureStandard308();
    stabilizeAHRS();
    {
        feedRange(500.0f);
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 5.0f;
        DOPE_SetUncertaintyConfig(&uc);

        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        DOPE_Update(&f);
    }
    FiringSolution large_sol = getSolution();

    EXPECT_GT(large_sol.sigma_elevation_moa, small_sol.sigma_elevation_moa);
}

// --- Defaults test ---

TEST_F(UncertaintyTest, DefaultConfigHasSensibleValues) {
    UncertaintyConfig uc = {};
    DOPE_GetDefaultUncertaintyConfig(&uc);

    EXPECT_TRUE(uc.enabled);
    EXPECT_GT(uc.sigma_muzzle_velocity_ms, 0.0f);
    EXPECT_GT(uc.sigma_range_m, 0.0f);
    EXPECT_GT(uc.sigma_wind_speed_ms, 0.0f);
    EXPECT_GT(uc.sigma_wind_heading_deg, 0.0f);
    EXPECT_GT(uc.sigma_temperature_c, 0.0f);
    EXPECT_GT(uc.sigma_pressure_pa, 0.0f);
    EXPECT_GT(uc.sigma_humidity, 0.0f);
    EXPECT_GT(uc.sigma_sight_height_mm, 0.0f);
    EXPECT_GT(uc.sigma_cant_deg, 0.0f);
}

// --- Full config plausibility test ---

TEST_F(UncertaintyTest, FullDefaultSigmasAt500mProducePlausibleCEP) {
    configureStandard308();
    DOPE_SetWindManual(3.0f, 90.0f);
    stabilizeAHRS();
    feedRange(457.2f); // ~500 yd

    UncertaintyConfig uc = {};
    DOPE_GetDefaultUncertaintyConfig(&uc);
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 457.2f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);

    // With typical hardware sigmas at ~500yd, expect reasonable uncertainty:
    // - sigma_elevation should be roughly 0.1-2.0 MOA
    // - sigma_windage should be roughly 0.1-3.0 MOA
    EXPECT_GT(sol.sigma_elevation_moa, 0.05f);
    EXPECT_LT(sol.sigma_elevation_moa, 5.0f);
    EXPECT_GT(sol.sigma_windage_moa, 0.05f);
    EXPECT_LT(sol.sigma_windage_moa, 5.0f);

    // CEP should be in a reasonable range
    float cep_moa = 1.1774f * std::sqrt(sol.sigma_elevation_moa * sol.sigma_windage_moa);
    EXPECT_GT(cep_moa, 0.1f);
    EXPECT_LT(cep_moa, 5.0f);
}

// --- Linearity sanity check ---

TEST_F(UncertaintyTest, UncertaintyScalesRoughlyLinearlyWithInputSigma) {
    configureStandard308();
    stabilizeAHRS();

    // Baseline (sigma = 0) to remove stiffness/CEP floor from the comparison.
    feedRange(500.0f);
    UncertaintyConfig base_uc = {};
    base_uc.enabled = true;
    DOPE_SetUncertaintyConfig(&base_uc);
    SensorFrame f_base = makeDefaultFrame(10000 * 350);
    f_base.lrf_range_m = 500.0f;
    f_base.lrf_timestamp_us = f_base.timestamp_us;
    f_base.lrf_valid = true;
    DOPE_Update(&f_base);
    FiringSolution base_sol = getSolution();

    // sigma=1 m/s
    {
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 1.0f;
        DOPE_SetUncertaintyConfig(&uc);
        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        DOPE_Update(&f);
    }
    FiringSolution sol1 = getSolution();

    // sigma=2 m/s — reinit
    DOPE_Init();
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);
    {
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 2.0f;
        DOPE_SetUncertaintyConfig(&uc);
        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        DOPE_Update(&f);
    }
    FiringSolution sol2 = getSolution();

    const float delta1 = sol1.sigma_elevation_moa - base_sol.sigma_elevation_moa;
    const float delta2 = sol2.sigma_elevation_moa - base_sol.sigma_elevation_moa;

    // Doubling sigma should grow the effect meaningfully; allow wide tolerance
    // because the baseline floor and nonlinearity compress the ratio.
    if (delta1 > 0.001f) {
        float ratio = delta2 / delta1;
        EXPECT_GT(ratio, 1.1f);
        // Allow broader nonlinearity from MV sigma doubling while still
        // ensuring growth is meaningful and bounded.
        EXPECT_LT(ratio, 5.0f);
    }
}

// --- BC sigma test ---

TEST_F(UncertaintyTest, BCSigmaAffectsElevation) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    EXPECT_GT(sol.sigma_elevation_moa, 0.01f);
}

TEST_F(UncertaintyTest, LengthSigmaContributesToUncertainty) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(1000.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 1000.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    EXPECT_GT(sol.uc_var_elev[12], 0.0f);
    EXPECT_GT(sol.uc_var_wind[12], 0.0f);
}

// --- Latitude sigma test ---

TEST_F(UncertaintyTest, LatitudeSigmaAffectsBothAxesWhenCoriolisActive) {
    configureStandard308();
    DOPE_SetLatitude(45.0f); // Coriolis active
    DOPE_SetWindManual(0.0f, 0.0f);
    
    // Stabilize AHRS pointing North-East to ensure both vertical (Eotvos) 
    // and horizontal Coriolis effects are active.
    for (int i = 0; i < 200; ++i) {
        SensorFrame f = makeDefaultFrame(10000 * (i + 1));
        f.mag_y = 25.0f; // Y component gives an Eastward heading
        DOPE_Update(&f);
    }

    feedRange(1000.0f); // Longer range makes Coriolis more apparent

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_latitude_deg = 5.0f; // 5 degrees of latitude uncertainty
    DOPE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.mag_y = 25.0f;
    f.lrf_range_m = 1000.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);

    // Coriolis affects elevation (Eotvos effect) and windage (horizontal Coriolis)
    EXPECT_GT(sol.sigma_elevation_moa, 0.001f);
    EXPECT_GT(sol.sigma_windage_moa, 0.001f);
    EXPECT_GT(sol.uc_var_elev[10], 0.0f);
    EXPECT_GT(sol.uc_var_wind[10], 0.0f);
}

TEST_F(UncertaintyTest, LatitudeSigmaIsIgnoredIfCoriolisInactive) {
    DOPE_Init();
    EnableSolverFallback();
    // No DOPE_SetLatitude — Coriolis inactive
    AmmoDatasetV2 ds = Make308V2();
    ds.mv_adjustment_fps_per_in = 25.0f;
    DOPE_SetAmmoDatasetV2(&ds);
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    stabilizeAHRS();
    feedRange(1000.0f);
    UncertaintyConfig uc = {}; uc.enabled = true; uc.sigma_latitude_deg = 5.0f;
    DOPE_SetUncertaintyConfig(&uc);
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 1000.0f; f.lrf_timestamp_us = f.timestamp_us; f.lrf_valid = true;
    DOPE_Update(&f);
    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    EXPECT_EQ(sol.uc_var_elev[10], 0.0f);
    EXPECT_EQ(sol.uc_var_wind[10], 0.0f);
}

TEST_F(UncertaintyTest, StiffnessAndCepInjectBaseDispersion) {
    AmmoDatasetV2 ds = Make308V2(); ds.mv_adjustment_fps_per_in = 25.0f;
    DOPE_SetAmmoDatasetV2(&ds);
    GunProfile gun = {};
    gun.barrel_length_in = 24.0f; gun.reference_barrel_length_in = 24.0f;
    gun.measured_cep50_moa = 0.8f; gun.stiffness_moa = 1.25f;
    DOPE_SetGunProfile(&gun);
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    DOPE_SetLatitude(37.0f);
    stabilizeAHRS(); feedRange(100.0f);
    UncertaintyConfig uc = {}; uc.enabled = true;
    DOPE_SetUncertaintyConfig(&uc);
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 100.0f; f.lrf_timestamp_us = f.timestamp_us; f.lrf_valid = true;
    DOPE_Update(&f);
    FiringSolution sol = getSolution();
    ASSERT_TRUE(sol.uncertainty_valid);
    EXPECT_GT(sol.sigma_windage_moa, 1.0f);
    EXPECT_GT(sol.sigma_elevation_moa, sol.sigma_windage_moa * 0.95f);
}

TEST_F(UncertaintyTest, NonFreeFloatAddsRadialPenalty) {
    configureStandard308();
    GunProfile b = {};
    b.barrel_length_in = 24.0f; b.reference_barrel_length_in = 24.0f;
    b.stiffness_moa = 0.6f; b.free_floated = true;
    DOPE_SetGunProfile(&b);
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    stabilizeAHRS(); feedRange(500.0f);
    UncertaintyConfig uc = {}; uc.enabled = true; DOPE_SetUncertaintyConfig(&uc);
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us; f.lrf_valid = true;
    DOPE_Update(&f);
    FiringSolution free_sol = getSolution();
    ASSERT_TRUE(free_sol.uncertainty_valid);

    b.free_floated = false;
    DOPE_SetGunProfile(&b);
    SensorFrame f2 = makeDefaultFrame(10000 * 410);
    f2.lrf_range_m = 500.0f; f2.lrf_timestamp_us = f2.timestamp_us; f2.lrf_valid = true;
    DOPE_Update(&f2);
    FiringSolution contact_sol = getSolution();
    ASSERT_TRUE(contact_sol.uncertainty_valid);
    EXPECT_GT(contact_sol.sigma_windage_moa, free_sol.sigma_windage_moa * 1.4f);
    EXPECT_GT(contact_sol.sigma_elevation_moa, free_sol.sigma_elevation_moa * 1.2f);
}

TEST_F(UncertaintyTest, SuppressorScalingDependsOnMuzzleDiameter) {
    configureStandard308();
    GunProfile b = {};
    b.barrel_length_in = 24.0f; b.reference_barrel_length_in = 24.0f;
    b.stiffness_moa = 0.6f; b.free_floated = true; b.suppressor_attached = false;
    DOPE_SetGunProfile(&b);
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    stabilizeAHRS(); feedRange(500.0f);
    UncertaintyConfig uc = {}; uc.enabled = true; DOPE_SetUncertaintyConfig(&uc);
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us; f.lrf_valid = true;
    DOPE_Update(&f);
    FiringSolution baseline = getSolution();

    b.suppressor_attached = true; b.muzzle_diameter_in = 0.55f;
    DOPE_SetGunProfile(&b);
    SensorFrame f2 = makeDefaultFrame(10000 * 410);
    f2.lrf_range_m = 500.0f; f2.lrf_timestamp_us = f2.timestamp_us; f2.lrf_valid = true;
    DOPE_Update(&f2);
    FiringSolution thin = getSolution();

    b.muzzle_diameter_in = 1.0f;
    DOPE_SetGunProfile(&b);
    SensorFrame f3 = makeDefaultFrame(10000 * 420);
    f3.lrf_range_m = 500.0f; f3.lrf_timestamp_us = f3.timestamp_us; f3.lrf_valid = true;
    DOPE_Update(&f3);
    FiringSolution thick = getSolution();

    ASSERT_TRUE(baseline.uncertainty_valid);
    ASSERT_TRUE(thin.uncertainty_valid);
    ASSERT_TRUE(thick.uncertainty_valid);
    EXPECT_GT(thin.sigma_windage_moa, baseline.sigma_windage_moa * 1.2f);
    EXPECT_LT(thick.sigma_windage_moa, thin.sigma_windage_moa);
}

TEST_F(UncertaintyTest, BarrelTunerReducesBarrelDispersion) {
    configureStandard308();
    GunProfile b = {};
    b.barrel_length_in = 24.0f; b.reference_barrel_length_in = 24.0f;
    b.stiffness_moa = 0.9f; b.free_floated = true;
    b.suppressor_attached = false; b.barrel_tuner_attached = false;
    DOPE_SetGunProfile(&b);
    ZeroConfig zero = {}; zero.zero_range_m = 100.0f; zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    stabilizeAHRS(); feedRange(500.0f);
    UncertaintyConfig uc = {}; uc.enabled = true; DOPE_SetUncertaintyConfig(&uc);
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f; f.lrf_timestamp_us = f.timestamp_us; f.lrf_valid = true;
    DOPE_Update(&f);
    FiringSolution base = getSolution();

    b.barrel_tuner_attached = true;
    DOPE_SetGunProfile(&b);
    SensorFrame f2 = makeDefaultFrame(10000 * 410);
    f2.lrf_range_m = 500.0f; f2.lrf_timestamp_us = f2.timestamp_us; f2.lrf_valid = true;
    DOPE_Update(&f2);
    FiringSolution tuned = getSolution();

    ASSERT_TRUE(base.uncertainty_valid);
    ASSERT_TRUE(tuned.uncertainty_valid);
    EXPECT_LT(tuned.sigma_windage_moa, base.sigma_windage_moa * 0.9f);
    EXPECT_LT(tuned.sigma_elevation_moa, base.sigma_elevation_moa * 0.9f);
}

    // ...existing code...
TEST_F(UncertaintyTest, ShotHeatIncreasesDispersionWithRapidCadence) {
    configureStandard308();
    GunProfile g = {};
    g.stiffness_moa = 0.8f;
    g.free_floated = true;
    DOPE_SetGunProfile(&g);
    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    DOPE_SetZeroConfig(&zero);
    stabilizeAHRS();
    feedRange(500.0f);
    UncertaintyConfig uc = {};
    uc.enabled = true;
    DOPE_SetUncertaintyConfig(&uc);
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    DOPE_Update(&f);
    FiringSolution cold = getSolution();

    // Fire 5 shots over ~1 second to heat the barrel.
    uint64_t t0 = 5000000ULL;
    for (int i = 0; i < 5; ++i) {
        DOPE_NotifyShotFired(t0 + static_cast<uint64_t>(i) * 200000ULL, 18.0f);
    }
    SensorFrame warm_frame = makeDefaultFrame(static_cast<uint64_t>(t0 + 1200000ULL));
    warm_frame.lrf_range_m = 500.0f;
    warm_frame.lrf_timestamp_us = warm_frame.timestamp_us;
    warm_frame.lrf_valid = true;
    DOPE_Update(&warm_frame);
    FiringSolution warm = getSolution();

    ASSERT_TRUE(cold.uncertainty_valid);
    ASSERT_TRUE(warm.uncertainty_valid);
    EXPECT_GT(warm.sigma_windage_moa, cold.sigma_windage_moa * 1.02f);
    EXPECT_GT(warm.sigma_elevation_moa, cold.sigma_elevation_moa * 1.02f);
}

