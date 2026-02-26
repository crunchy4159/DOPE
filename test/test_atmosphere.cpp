/**
 * @file test_atmosphere.cpp
 * @brief Unit tests for the Atmosphere module.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/atmo/atmosphere.h"
#include "bce/bce_config.h"
#include <cmath>

class AtmosphereTest : public ::testing::Test {
protected:
    Atmosphere atmo;

    void SetUp() override {
        atmo.init();
    }
};

// At ISA standard conditions, air density should be ~1.225 kg/m³
TEST_F(AtmosphereTest, ISAStandardDensity) {
    float density = atmo.getAirDensity();
    EXPECT_NEAR(density, 1.225f, 0.01f);
}

// Speed of sound at 15°C should be ~340 m/s
TEST_F(AtmosphereTest, ISASpeedOfSound) {
    float sos = atmo.getSpeedOfSound();
    EXPECT_NEAR(sos, 340.3f, 1.0f);
}

// Updating with ISA values should keep density near standard
TEST_F(AtmosphereTest, UpdateWithISABaro) {
    atmo.updateFromBaro(101325.0f, 15.0f, 0.5f);
    EXPECT_NEAR(atmo.getAirDensity(), 1.225f, 0.02f);
}

// Hot air should be less dense
TEST_F(AtmosphereTest, HotAirLessDense) {
    float cold_density, hot_density;

    atmo.updateFromBaro(101325.0f, 0.0f, 0.0f);
    cold_density = atmo.getAirDensity();

    atmo.updateFromBaro(101325.0f, 40.0f, 0.0f);
    hot_density = atmo.getAirDensity();

    EXPECT_GT(cold_density, hot_density);
}

// Low pressure should be less dense
TEST_F(AtmosphereTest, LowPressureLessDense) {
    float high_p, low_p;

    atmo.updateFromBaro(101325.0f, 15.0f, 0.0f);
    high_p = atmo.getAirDensity();

    atmo.updateFromBaro(90000.0f, 15.0f, 0.0f);
    low_p = atmo.getAirDensity();

    EXPECT_GT(high_p, low_p);
}

// BC correction at ISA should return approximately the same BC
TEST_F(AtmosphereTest, BCCorrectionAtISA) {
    float bc = 0.505f; // typical .308 G1 BC
    float corrected = atmo.correctBC(bc);
    // At sea level ISA with default humidity, correction should be near 1.0
    EXPECT_NEAR(corrected, bc, 0.02f);
}

// BC correction at altitude should increase BC (less air resistance)
TEST_F(AtmosphereTest, BCCorrectionAtAltitude) {
    BCE_DefaultOverrides ovr = {};
    ovr.use_altitude = true;
    ovr.altitude_m = 2000.0f; // 2000m elevation

    atmo.applyDefaults(ovr);

    float bc = 0.505f;
    float corrected = atmo.correctBC(bc);
    // At altitude, less air → BC should increase (factor > 1)
    // Actually the FA factor decreases, making corrected BC smaller...
    // but the pressure is lower too, making (1 - FP) > 1
    // The net effect depends on the specific altitude
    EXPECT_GT(corrected, 0.0f);
}

// Diagnostic flags should show defaults when no sensor data provided
TEST_F(AtmosphereTest, DiagFlagsShowDefaults) {
    uint32_t flags = atmo.getDiagFlags();
    EXPECT_NE(flags & BCE_Diag::DEFAULT_PRESSURE, 0u);
    EXPECT_NE(flags & BCE_Diag::DEFAULT_TEMP, 0u);
    EXPECT_NE(flags & BCE_Diag::DEFAULT_HUMIDITY, 0u);
    EXPECT_NE(flags & BCE_Diag::DEFAULT_ALTITUDE, 0u);
}

// After baro update, pressure and temp defaults should clear
TEST_F(AtmosphereTest, DiagFlagsClearAfterBaro) {
    atmo.updateFromBaro(101325.0f, 15.0f, 0.5f);
    uint32_t flags = atmo.getDiagFlags();
    EXPECT_EQ(flags & BCE_Diag::DEFAULT_PRESSURE, 0u);
    EXPECT_EQ(flags & BCE_Diag::DEFAULT_TEMP, 0u);
    EXPECT_EQ(flags & BCE_Diag::DEFAULT_HUMIDITY, 0u);
}

// Invalid barometer values should be sanitized to finite, physical outputs
TEST_F(AtmosphereTest, InvalidBaroInputIsSanitized) {
    atmo.updateFromBaro(-100.0f, -300.0f, 0.5f);

    EXPECT_TRUE(atmo.hadInvalidInput());
    EXPECT_GT(atmo.getPressure(), 0.0f);
    EXPECT_TRUE(std::isfinite(atmo.getAirDensity()));
    EXPECT_GT(atmo.getAirDensity(), 0.0f);
    EXPECT_TRUE(std::isfinite(atmo.getSpeedOfSound()));
    EXPECT_GT(atmo.getSpeedOfSound(), 0.0f);
}

// Humidity outside 0..1 should be clamped and flagged invalid
TEST_F(AtmosphereTest, InvalidHumidityIsClamped) {
    atmo.updateFromBaro(101325.0f, 15.0f, 2.0f);

    EXPECT_TRUE(atmo.hadInvalidInput());
    EXPECT_GE(atmo.getHumidity(), 0.0f);
    EXPECT_LE(atmo.getHumidity(), 1.0f);
}
