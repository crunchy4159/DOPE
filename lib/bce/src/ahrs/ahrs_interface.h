/**
 * @file ahrs_interface.h
 * @brief Abstract AHRS (Attitude and Heading Reference System) interface.
 *
 * Both Madgwick and Mahony filters implement this interface.
 * The engine selects one at runtime via BCE_SetAHRSAlgorithm().
 */

#pragma once

#include <cstdint>
#include <cmath>

struct Quaternion {
    float w, x, y, z;

    void normalize() {
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0.0f) {
            float inv = 1.0f / norm;
            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        }
    }
};

class AHRS_Interface {
public:
    virtual ~AHRS_Interface() = default;

    /**
     * Update the filter with new IMU (and optionally magnetometer) data.
     *
     * @param ax, ay, az  Accelerometer (m/s²) — bias-corrected
     * @param gx, gy, gz  Gyroscope (rad/s) — bias-corrected
     * @param mx, my, mz  Magnetometer (µT) — calibrated. Pass 0 if invalid.
     * @param use_mag     Whether magnetometer data is valid this cycle
     * @param dt          Time step (seconds)
     */
    virtual void update(float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float mx, float my, float mz,
                        bool use_mag, float dt) = 0;

    /**
     * Reset the filter to identity quaternion.
     */
    virtual void reset() = 0;

    /** Get the current orientation quaternion. */
    virtual Quaternion getQuaternion() const = 0;

    /** Pitch angle in radians (nose up positive). */
    float getPitch() const {
        Quaternion q = getQuaternion();
        float sinp = 2.0f * (q.w * q.y - q.z * q.x);
        if (sinp > 1.0f) sinp = 1.0f;
        if (sinp < -1.0f) sinp = -1.0f;
        return std::asin(sinp);
    }

    /** Roll angle in radians (right wing down positive). */
    float getRoll() const {
        Quaternion q = getQuaternion();
        float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        return std::atan2(sinr_cosp, cosr_cosp);
    }

    /** Yaw angle in radians (clockwise from north positive). */
    float getYaw() const {
        Quaternion q = getQuaternion();
        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};
