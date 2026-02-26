/**
 * @file ahrs_manager.cpp
 * @brief AHRS manager implementation.
 */

#include "ahrs_manager.h"
#include <cmath>
#include <cstring>

void AHRSManager::init() {
    madgwick_.reset();
    mahony_.reset();
    std::memset(accel_mag_buf_, 0, sizeof(accel_mag_buf_));
    buf_index_ = 0;
    sample_count_ = 0;
    is_static_ = false;
    std::memset(accel_bias_, 0, sizeof(accel_bias_));
    std::memset(gyro_bias_, 0, sizeof(gyro_bias_));
}

void AHRSManager::setAlgorithm(AHRS_Algorithm algo) {
    algorithm_ = algo;
}

AHRS_Interface* AHRSManager::activeFilter() {
    return (algorithm_ == AHRS_Algorithm::MAHONY)
        ? static_cast<AHRS_Interface*>(&mahony_)
        : static_cast<AHRS_Interface*>(&madgwick_);
}

const AHRS_Interface* AHRSManager::activeFilter() const {
    return (algorithm_ == AHRS_Algorithm::MAHONY)
        ? static_cast<const AHRS_Interface*>(&mahony_)
        : static_cast<const AHRS_Interface*>(&madgwick_);
}

void AHRSManager::update(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float mx, float my, float mz,
                          bool use_mag, float dt) {
    // Apply bias corrections
    ax -= accel_bias_[0];
    ay -= accel_bias_[1];
    az -= accel_bias_[2];
    gx -= gyro_bias_[0];
    gy -= gyro_bias_[1];
    gz -= gyro_bias_[2];

    // Update active filter
    activeFilter()->update(ax, ay, az, gx, gy, gz, mx, my, mz, use_mag, dt);

    // Update static detection
    updateStaticDetection(ax, ay, az);
}

void AHRSManager::setAccelBias(const float bias[3]) {
    accel_bias_[0] = bias[0];
    accel_bias_[1] = bias[1];
    accel_bias_[2] = bias[2];
}

void AHRSManager::setGyroBias(const float bias[3]) {
    gyro_bias_[0] = bias[0];
    gyro_bias_[1] = bias[1];
    gyro_bias_[2] = bias[2];
}

void AHRSManager::captureGyroBias(float gx, float gy, float gz) {
    gyro_bias_[0] = gx;
    gyro_bias_[1] = gy;
    gyro_bias_[2] = gz;
}

Quaternion AHRSManager::getQuaternion() const {
    return activeFilter()->getQuaternion();
}

float AHRSManager::getPitch() const {
    return const_cast<AHRS_Interface*>(activeFilter())->getPitch();
}

float AHRSManager::getRoll() const {
    return const_cast<AHRS_Interface*>(activeFilter())->getRoll();
}

float AHRSManager::getYaw() const {
    return const_cast<AHRS_Interface*>(activeFilter())->getYaw();
}

void AHRSManager::updateStaticDetection(float ax, float ay, float az) {
    float mag = std::sqrt(ax * ax + ay * ay + az * az);
    accel_mag_buf_[buf_index_] = mag;
    buf_index_ = (buf_index_ + 1) % BCE_AHRS_STATIC_WINDOW;

    if (sample_count_ < static_cast<uint32_t>(BCE_AHRS_STATIC_WINDOW)) {
        sample_count_++;
        is_static_ = false;
        return;
    }

    // Compute mean
    float sum = 0.0f;
    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW; ++i) {
        sum += accel_mag_buf_[i];
    }
    float mean = sum / static_cast<float>(BCE_AHRS_STATIC_WINDOW);

    // Compute variance
    float var = 0.0f;
    for (int i = 0; i < BCE_AHRS_STATIC_WINDOW; ++i) {
        float diff = accel_mag_buf_[i] - mean;
        var += diff * diff;
    }
    var /= static_cast<float>(BCE_AHRS_STATIC_WINDOW);

    is_static_ = (var < BCE_AHRS_STATIC_THRESHOLD);
}
