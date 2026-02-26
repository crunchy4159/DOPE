/**
 * @file ahrs_manager.h
 * @brief AHRS manager — owns both filters, handles bias, and static detection.
 *
 * Wraps the selectable filter behind a unified interface with bias correction,
 * static/dynamic detection via accel variance, and quaternion snapshot support.
 */

#pragma once

#include "madgwick.h"
#include "mahony.h"
#include "bce/bce_config.h"
#include "bce/bce_types.h"

class AHRSManager {
public:
    void init();
    void setAlgorithm(AHRS_Algorithm algo);

    /**
     * Feed raw (uncorrected) IMU and mag data. Applies biases internally.
     */
    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz,
                bool use_mag, float dt);

    void setAccelBias(const float bias[3]);
    void setGyroBias(const float bias[3]);

    /**
     * Capture current gyro readings as bias. Caller should ensure device is static.
     */
    void captureGyroBias(float gx, float gy, float gz);

    Quaternion getQuaternion() const;
    float getPitch() const;
    float getRoll() const;
    float getYaw() const;

    /** True if the device is approximately stationary. */
    bool isStatic() const { return is_static_; }

    /** True if AHRS has converged enough for a valid solution. */
    bool isStable() const {
        return sample_count_ >= BCE_AHRS_STATIC_WINDOW && is_static_;
    }

private:
    AHRS_Algorithm algorithm_ = AHRS_Algorithm::MADGWICK;
    MadgwickFilter madgwick_;
    MahonyFilter mahony_;

    float accel_bias_[3] = {0, 0, 0};
    float gyro_bias_[3]  = {0, 0, 0};

    // Static detection ring buffer
    float accel_mag_buf_[BCE_AHRS_STATIC_WINDOW] = {};
    int buf_index_ = 0;
    uint32_t sample_count_ = 0;
    bool is_static_ = false;

    AHRS_Interface* activeFilter();
    const AHRS_Interface* activeFilter() const;
    void updateStaticDetection(float ax, float ay, float az);
};
