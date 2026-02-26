/**
 * @file wind.h
 * @brief Wind decomposition — SRS §11.3
 *
 * Decomposes manual wind (speed + heading) into headwind and crosswind
 * components relative to the bullet's azimuth (firing direction).
 */

#pragma once

#include "bce/bce_config.h"

class WindCorrection {
public:
    /**
     * Set manual wind parameters.
     * @param speed_ms     Wind speed in m/s
     * @param heading_deg  Wind heading in degrees true (direction wind comes FROM)
     */
    void setWind(float speed_ms, float heading_deg);

    /**
     * Decompose wind into headwind and crosswind relative to firing azimuth.
     * @param azimuth_deg  Firing direction in degrees true
     * @param out_headwind  Output: headwind component (positive = into shooter face)
     * @param out_crosswind Output: crosswind component (positive = right to left)
     */
    void decompose(float azimuth_deg, float& out_headwind, float& out_crosswind) const;

    float getSpeed() const { return speed_ms_; }
    float getHeading() const { return heading_deg_; }
    bool isSet() const { return is_set_; }

private:
    float speed_ms_    = 0.0f;
    float heading_deg_ = 0.0f;
    bool  is_set_      = false;
};
