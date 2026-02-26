/**
 * @file wind.cpp
 * @brief Wind decomposition implementation.
 */

#include "wind.h"
#include <cmath>

void WindCorrection::setWind(float speed_ms, float heading_deg) {
    speed_ms_ = speed_ms;
    heading_deg_ = heading_deg;
    is_set_ = true;
}

void WindCorrection::decompose(float azimuth_deg, float& out_headwind, float& out_crosswind) const {
    if (!is_set_ || speed_ms_ < 0.001f) {
        out_headwind = 0.0f;
        out_crosswind = 0.0f;
        return;
    }

    // Angle between wind direction and firing direction
    // Wind heading = direction wind comes FROM
    // The relative angle: how the wind blows relative to the shooter's bore axis
    float angle_rad = (heading_deg_ - azimuth_deg) * BCE_DEG_TO_RAD;

    // headwind = wind_speed × cos(angle)
    //   positive = wind blowing into the shooter's face (retards bullet)
    // crosswind = wind_speed × sin(angle)
    //   positive = wind blowing from right to left (deflects bullet left)
    out_headwind = speed_ms_ * std::cos(angle_rad);
    out_crosswind = speed_ms_ * std::sin(angle_rad);
}
