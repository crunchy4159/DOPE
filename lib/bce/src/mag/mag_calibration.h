/**
 * @file mag_calibration.h
 * @brief Magnetometer calibration, disturbance detection, and heading computation.
 *
 * BCE SRS v1.3 — Sections 7.2, 10
 */

#pragma once

#include "bce/bce_config.h"

class MagCalibration {
public:
    void init();

    /**
     * Set hard-iron offset and soft-iron correction matrix.
     * @param hard_iron  3-element offset vector to subtract.
     * @param soft_iron  9-element 3×3 matrix (row-major) for soft-iron correction.
     */
    void setCalibration(const float hard_iron[3], const float soft_iron[9]);

    /**
     * Set magnetic declination (east positive).
     */
    void setDeclination(float declination_deg);

    /**
     * Apply calibration to raw magnetometer readings.
     * Returns true if the field magnitude is within expected range.
     */
    bool apply(float& mx, float& my, float& mz) const;

    /**
     * Check if the last reading was disturbed (field outside expected range).
     */
    bool isDisturbed() const { return is_disturbed_; }

    /**
     * Compute true heading from calibrated mag and AHRS orientation.
     * @param yaw_rad  Yaw from AHRS (radians)
     * @return heading in degrees true (0–360)
     */
    float computeHeading(float yaw_rad) const;

    float getDeclination() const { return declination_deg_; }

private:
    float hard_iron_[3]  = {0, 0, 0};
    float soft_iron_[9]  = {1, 0, 0,  0, 1, 0,  0, 0, 1}; // identity
    float declination_deg_ = 0.0f;
    mutable bool is_disturbed_ = false;
};
