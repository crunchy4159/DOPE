/**
 * @file test_uncertainty.cpp
 * @brief Tests for the Gaussian uncertainty propagation system.
 *
 * Validates that the engine correctly computes a 2x2 covariance matrix
 * (elevation x windage) via linear error propagation / sensitivity analysis.
 */

#include <gtest/gtest.h>
#include "bce/bce_api.h"
#include "bce/bce_config.h"
#include <cmath>
#include <cstring>

class UncertaintyTest : public ::testing::Test {
protected:
    void SetUp() override {
        BCE_Init();
    }

    void configureStandard308() {
        BulletProfile bullet = {};
        bullet.bc = 0.505f;
        bullet.drag_model = DragModel::G1;
        bullet.muzzle_velocity_ms = 792.0f;
        bullet.mass_grains = 175.0f;
        bullet.length_mm = 31.2f;
        bullet.caliber_inches = 0.308f;
        bullet.twist_rate_inches = 10.0f;
        bullet.barrel_length_in = 24.0f;
        bullet.reference_barrel_length_in = 24.0f;
        bullet.mv_adjustment_factor = 25.0f;
        BCE_SetBulletProfile(&bullet);

        ZeroConfig zero = {};
        zero.zero_range_m = 100.0f;
        zero.sight_height_mm = 38.1f;
        BCE_SetZeroConfig(&zero);

        BCE_SetLatitude(37.0f);
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
            BCE_Update(&f);
        }
    }

    void feedRange(float range_m) {
        SensorFrame f = makeDefaultFrame(10000 * 300);
        f.lrf_range_m = range_m;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        BCE_Update(&f);
    }

    FiringSolution getSolution() {
        FiringSolution sol = {};
        BCE_GetSolution(&sol);
        return sol;
    }
};

TEST_F(UncertaintyTest, CartridgeCEPSetsScale) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    BCE_GetDefaultUncertaintyConfig(&uc);
    uc.enabled = true;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution base = getSolution();
    ASSERT_TRUE(base.uncertainty_valid);
    const float base_radius =
        std::sqrt(base.sigma_elevation_moa * base.sigma_elevation_moa +
                  base.sigma_windage_moa * base.sigma_windage_moa);
    ASSERT_GT(base_radius, 0.001f);

    const float target_factor = 1.8f;
    const float target_cep50_moa = base_radius * target_factor * 1.17741f; // CEP50 -> 1-sigma
    BCE_CEPPoint cep_point = {500.0f, target_cep50_moa};
    uc.use_cartridge_cep_table = true;
    uc.cartridge_cep_table = {&cep_point, 1};
    uc.cartridge_cep_scale_floor = 1.0f;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f2 = makeDefaultFrame(10000 * 500);
    f2.lrf_range_m = 500.0f;
    f2.lrf_timestamp_us = f2.timestamp_us;
    f2.lrf_valid = true;
    BCE_Update(&f2);

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
    BCE_SetUncertaintyConfig(&uc);

    // Re-run to get updated solution
    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    EXPECT_NEAR(sol.sigma_elevation_moa, 0.0f, 0.001f);
    EXPECT_NEAR(sol.sigma_windage_moa, 0.0f, 0.001f);
    EXPECT_NEAR(sol.covariance_elev_wind, 0.0f, 0.001f);
}

TEST_F(UncertaintyTest, DisabledProducesNoUncertainty) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    BCE_GetDefaultUncertaintyConfig(&uc);
    uc.enabled = false;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

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

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_muzzle_velocity_ms = 3.0f; // ~10 fps
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    // MV mainly affects elevation (drop), should be nonzero
    EXPECT_GT(sol.sigma_elevation_moa, 0.01f);
    // MV should not significantly affect windage
    EXPECT_LT(sol.sigma_windage_moa, sol.sigma_elevation_moa * 0.5f);
}

TEST_F(UncertaintyTest, WindSpeedSigmaAffectsWindage) {
    configureStandard308();
    BCE_SetWindManual(3.0f, 90.0f); // 3 m/s crosswind from right
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_wind_speed_ms = 2.0f;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    // Wind speed sigma in a full crosswind should primarily affect windage
    EXPECT_GT(sol.sigma_windage_moa, 0.01f);
}

TEST_F(UncertaintyTest, RangeSigmaAffectsBothAxes) {
    configureStandard308();
    BCE_SetWindManual(3.0f, 90.0f);
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_range_m = 5.0f;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

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
        BCE_SetUncertaintyConfig(&uc);

        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        BCE_Update(&f);
    }
    FiringSolution small_sol = getSolution();

    // Large sigma — reinit to get clean state
    BCE_Init();
    configureStandard308();
    stabilizeAHRS();
    {
        feedRange(500.0f);
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 5.0f;
        BCE_SetUncertaintyConfig(&uc);

        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        BCE_Update(&f);
    }
    FiringSolution large_sol = getSolution();

    EXPECT_GT(large_sol.sigma_elevation_moa, small_sol.sigma_elevation_moa);
}

// --- Defaults test ---

TEST_F(UncertaintyTest, DefaultConfigHasSensibleValues) {
    UncertaintyConfig uc = {};
    BCE_GetDefaultUncertaintyConfig(&uc);

    EXPECT_TRUE(uc.enabled);
    EXPECT_GT(uc.sigma_muzzle_velocity_ms, 0.0f);
    EXPECT_GT(uc.sigma_bc_fraction, 0.0f);
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
    BCE_SetWindManual(3.0f, 90.0f);
    stabilizeAHRS();
    feedRange(457.2f); // ~500 yd

    UncertaintyConfig uc = {};
    BCE_GetDefaultUncertaintyConfig(&uc);
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 457.2f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

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

    // sigma=1 m/s
    feedRange(500.0f);
    {
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 1.0f;
        BCE_SetUncertaintyConfig(&uc);
        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        BCE_Update(&f);
    }
    FiringSolution sol1 = getSolution();

    // sigma=2 m/s — reinit
    BCE_Init();
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);
    {
        UncertaintyConfig uc = {};
        uc.enabled = true;
        uc.sigma_muzzle_velocity_ms = 2.0f;
        BCE_SetUncertaintyConfig(&uc);
        SensorFrame f = makeDefaultFrame(10000 * 400);
        f.lrf_range_m = 500.0f;
        f.lrf_timestamp_us = f.timestamp_us;
        f.lrf_valid = true;
        BCE_Update(&f);
    }
    FiringSolution sol2 = getSolution();

    // For linear propagation, doubling sigma should roughly double the output.
    // Allow 20% tolerance for nonlinearity.
    if (sol1.sigma_elevation_moa > 0.001f) {
        float ratio = sol2.sigma_elevation_moa / sol1.sigma_elevation_moa;
        EXPECT_GT(ratio, 1.6f);
        EXPECT_LT(ratio, 2.4f);
    }
}

// --- BC sigma test ---

TEST_F(UncertaintyTest, BCSigmaAffectsElevation) {
    configureStandard308();
    stabilizeAHRS();
    feedRange(500.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_bc_fraction = 0.05f; // 5% — large
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 500.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

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
    uc.sigma_length_mm = 1.0f;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 1000.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    EXPECT_GT(sol.uc_var_elev[12], 0.0f);
    EXPECT_GT(sol.uc_var_wind[12], 0.0f);
}

// --- Latitude sigma test ---

TEST_F(UncertaintyTest, LatitudeSigmaAffectsBothAxesWhenCoriolisActive) {
    configureStandard308();
    BCE_SetLatitude(45.0f); // Coriolis active
    BCE_SetWindManual(0.0f, 0.0f);
    
    // Stabilize AHRS pointing North-East to ensure both vertical (Eotvos) 
    // and horizontal Coriolis effects are active.
    for (int i = 0; i < 200; ++i) {
        SensorFrame f = makeDefaultFrame(10000 * (i + 1));
        f.mag_y = 25.0f; // Y component gives an Eastward heading
        BCE_Update(&f);
    }

    feedRange(1000.0f); // Longer range makes Coriolis more apparent

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_latitude_deg = 5.0f; // 5 degrees of latitude uncertainty
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.mag_y = 25.0f;
    f.lrf_range_m = 1000.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);

    // Coriolis affects elevation (Eotvos effect) and windage (horizontal Coriolis)
    EXPECT_GT(sol.sigma_elevation_moa, 0.001f);
    EXPECT_GT(sol.sigma_windage_moa, 0.001f);
    EXPECT_GT(sol.uc_var_elev[10], 0.0f);
    EXPECT_GT(sol.uc_var_wind[10], 0.0f);
}

TEST_F(UncertaintyTest, LatitudeSigmaIsIgnoredIfCoriolisInactive) {
    BCE_Init();
    // Configure standard bullet and zero, but omit BCE_SetLatitude
    BulletProfile bullet = {};
    bullet.bc = 0.505f;
    bullet.drag_model = DragModel::G1;
    bullet.muzzle_velocity_ms = 792.0f;
    bullet.mass_grains = 175.0f;
    bullet.length_mm = 31.2f;
    bullet.caliber_inches = 0.308f;
    bullet.twist_rate_inches = 10.0f;
    bullet.barrel_length_in = 24.0f;
    bullet.reference_barrel_length_in = 24.0f;
    bullet.mv_adjustment_factor = 25.0f;
    BCE_SetBulletProfile(&bullet);

    ZeroConfig zero = {};
    zero.zero_range_m = 100.0f;
    zero.sight_height_mm = 38.1f;
    BCE_SetZeroConfig(&zero);

    stabilizeAHRS();
    feedRange(1000.0f);

    UncertaintyConfig uc = {};
    uc.enabled = true;
    uc.sigma_latitude_deg = 5.0f;
    BCE_SetUncertaintyConfig(&uc);

    SensorFrame f = makeDefaultFrame(10000 * 400);
    f.lrf_range_m = 1000.0f;
    f.lrf_timestamp_us = f.timestamp_us;
    f.lrf_valid = true;
    BCE_Update(&f);

    FiringSolution sol = getSolution();
    EXPECT_TRUE(sol.uncertainty_valid);
    // Should have exactly 0 contribution from latitude (index 10)
    EXPECT_EQ(sol.uc_var_elev[10], 0.0f);
    EXPECT_EQ(sol.uc_var_wind[10], 0.0f);
}

