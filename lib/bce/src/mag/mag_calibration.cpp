/**
 * @file mag_calibration.cpp
 * @brief Magnetometer calibration implementation.
 */

#include "mag_calibration.h"
#include <cmath>
#include <cstring>

void MagCalibration::init() {
    std::memset(hard_iron_, 0, sizeof(hard_iron_));
    // Identity matrix for soft iron
    float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::memcpy(soft_iron_, identity, sizeof(soft_iron_));
    declination_deg_ = 0.0f;
    is_disturbed_ = false;
}

void MagCalibration::setCalibration(const float hard_iron[3], const float soft_iron[9]) {
    std::memcpy(hard_iron_, hard_iron, 3 * sizeof(float));
    std::memcpy(soft_iron_, soft_iron, 9 * sizeof(float));
}

void MagCalibration::setDeclination(float declination_deg) {
    declination_deg_ = declination_deg;
}

bool MagCalibration::apply(float& mx, float& my, float& mz) const {
    // Subtract hard iron
    float cx = mx - hard_iron_[0];
    float cy = my - hard_iron_[1];
    float cz = mz - hard_iron_[2];

    // Apply soft iron correction (3×3 matrix multiply)
    mx = soft_iron_[0] * cx + soft_iron_[1] * cy + soft_iron_[2] * cz;
    my = soft_iron_[3] * cx + soft_iron_[4] * cy + soft_iron_[5] * cz;
    mz = soft_iron_[6] * cx + soft_iron_[7] * cy + soft_iron_[8] * cz;

    // Check field magnitude for disturbance detection
    float field_mag = std::sqrt(mx * mx + my * my + mz * mz);
    is_disturbed_ = (field_mag < BCE_MAG_MIN_FIELD_UT || field_mag > BCE_MAG_MAX_FIELD_UT);

    return !is_disturbed_;
}

float MagCalibration::computeHeading(float yaw_rad) const {
    // Convert yaw to degrees and apply declination
    float heading = yaw_rad * BCE_RAD_TO_DEG + declination_deg_;

    // Normalize to 0–360
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    return heading;
}
