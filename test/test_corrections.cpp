/**
 * @file test_corrections.cpp
 * @brief Unit tests for wind, cant, spin drift, and Coriolis corrections.
 */

#include <gtest/gtest.h>
#include "../lib/bce/src/corrections/wind.h"
#include "../lib/bce/src/corrections/cant.h"
#include "bce/bce_config.h"
#include <cmath>

// ---------------------------------------------------------------------------
// Wind Tests
// ---------------------------------------------------------------------------

class WindTest : public ::testing::Test {
protected:
    WindCorrection wind;
};

// Pure headwind: wind from same direction as firing
TEST_F(WindTest, PureHeadwind) {
    wind.setWind(10.0f, 0.0f); // 10 m/s from north

    float headwind, crosswind;
    wind.decompose(0.0f, headwind, crosswind); // firing north

    EXPECT_NEAR(headwind, 10.0f, 0.1f);
    EXPECT_NEAR(crosswind, 0.0f, 0.1f);
}

// Pure tailwind: wind from behind
TEST_F(WindTest, PureTailwind) {
    wind.setWind(10.0f, 180.0f); // wind from south

    float headwind, crosswind;
    wind.decompose(0.0f, headwind, crosswind); // firing north

    EXPECT_NEAR(headwind, -10.0f, 0.1f); // negative = tailwind
    EXPECT_NEAR(crosswind, 0.0f, 0.1f);
}

// Pure crosswind from the right (90° relative)
TEST_F(WindTest, PureCrosswind) {
    wind.setWind(10.0f, 90.0f); // wind from east

    float headwind, crosswind;
    wind.decompose(0.0f, headwind, crosswind); // firing north

    EXPECT_NEAR(headwind, 0.0f, 0.1f);
    EXPECT_NEAR(std::fabs(crosswind), 10.0f, 0.1f);
}

// No wind should produce zero components
TEST_F(WindTest, ZeroWind) {
    float headwind, crosswind;
    wind.decompose(0.0f, headwind, crosswind); // no wind set

    EXPECT_FLOAT_EQ(headwind, 0.0f);
    EXPECT_FLOAT_EQ(crosswind, 0.0f);
}

// ---------------------------------------------------------------------------
// Cant Tests
// ---------------------------------------------------------------------------

// Zero cant should not change elevation
TEST(CantTest, ZeroCantNoChange) {
    float elev_out, wind_out;
    CantCorrection::apply(0.0f, 10.0f, elev_out, wind_out);

    EXPECT_NEAR(elev_out, 10.0f, 0.001f);
    EXPECT_NEAR(wind_out, 0.0f, 0.001f);
}

// 90° cant should turn all elevation into windage
TEST(CantTest, NinetyDegreeCant) {
    float elev_out, wind_out;
    float cant_90 = BCE_PI / 2.0f;
    CantCorrection::apply(cant_90, 10.0f, elev_out, wind_out);

    EXPECT_NEAR(elev_out, 0.0f, 0.01f);
    EXPECT_NEAR(wind_out, 10.0f, 0.01f);
}

// 45° cant should split equally
TEST(CantTest, FortyFiveDegreeCant) {
    float elev_out, wind_out;
    float cant_45 = BCE_PI / 4.0f;
    CantCorrection::apply(cant_45, 10.0f, elev_out, wind_out);

    float expected = 10.0f * std::cos(cant_45);
    EXPECT_NEAR(elev_out, expected, 0.01f);
    EXPECT_NEAR(wind_out, expected, 0.01f); // sin(45°) = cos(45°)
}

// Small cant should mostly preserve elevation
TEST(CantTest, SmallCantMostlyPreservesElevation) {
    float elev_out, wind_out;
    float cant_5deg = 5.0f * BCE_DEG_TO_RAD;
    CantCorrection::apply(cant_5deg, 30.0f, elev_out, wind_out);

    EXPECT_NEAR(elev_out, 30.0f, 0.5f); // barely changes
    EXPECT_GT(std::fabs(wind_out), 0.0f); // but adds a small windage error
    EXPECT_LT(std::fabs(wind_out), 5.0f);
}
