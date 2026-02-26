/**
 * @file madgwick.cpp
 * @brief Madgwick gradient-descent AHRS filter implementation.
 */

#include "madgwick.h"
#include <cmath>

void MadgwickFilter::update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float mx, float my, float mz,
                            bool use_mag, float dt) {
    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

    // Rate of change of quaternion from gyroscope
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Normalize accelerometer
    float a_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (a_norm > 0.001f) {
        float a_inv = 1.0f / a_norm;
        ax *= a_inv;
        ay *= a_inv;
        az *= a_inv;

        if (use_mag) {
            // Normalize magnetometer
            float m_norm = std::sqrt(mx * mx + my * my + mz * mz);
            if (m_norm > 0.001f) {
                float m_inv = 1.0f / m_norm;
                mx *= m_inv;
                my *= m_inv;
                mz *= m_inv;

                // Auxiliary variables
                float _2q0 = 2.0f * q0;
                float _2q1 = 2.0f * q1;
                float _2q2 = 2.0f * q2;
                float _2q3 = 2.0f * q3;
                float q0q0 = q0 * q0;
                float q0q1 = q0 * q1;
                float q0q2 = q0 * q2;
                float q0q3 = q0 * q3;
                float q1q1 = q1 * q1;
                float q1q2 = q1 * q2;
                float q1q3 = q1 * q3;
                float q2q2 = q2 * q2;
                float q2q3 = q2 * q3;
                float q3q3 = q3 * q3;

                // Reference direction of Earth's magnetic field
                float hx = mx * (q0q0 + q1q1 - q2q2 - q3q3) +
                           2.0f * my * (q1q2 - q0q3) +
                           2.0f * mz * (q1q3 + q0q2);
                float hy = 2.0f * mx * (q1q2 + q0q3) +
                           my * (q0q0 - q1q1 + q2q2 - q3q3) +
                           2.0f * mz * (q2q3 - q0q1);
                float _2bx = std::sqrt(hx * hx + hy * hy);
                float _2bz = 2.0f * mx * (q1q3 - q0q2) +
                              2.0f * my * (q2q3 + q0q1) +
                              mz * (q0q0 - q1q1 - q2q2 + q3q3);

                // Gradient descent corrective step (6-axis: accel + mag)
                float s0 = -_2q2 * (2.0f * q1q3 - _2q0 * q2 - ax) +
                            _2q1 * (2.0f * q0q1 + _2q2 * q3 - ay) -
                            _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                            (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                            _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                float s1 = _2q3 * (2.0f * q1q3 - _2q0 * q2 - ax) +
                            _2q0 * (2.0f * q0q1 + _2q2 * q3 - ay) -
                            4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
                            _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                            (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                            (_2bx * q3 - 4.0f * _2bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                float s2 = -_2q0 * (2.0f * q1q3 - _2q0 * q2 - ax) +
                             _2q3 * (2.0f * q0q1 + _2q2 * q3 - ay) -
                             4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
                             (-4.0f * _2bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                             (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                             (_2bx * q0 - 4.0f * _2bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                float s3 = _2q1 * (2.0f * q1q3 - _2q0 * q2 - ax) +
                            _2q2 * (2.0f * q0q1 + _2q2 * q3 - ay) +
                            (-4.0f * _2bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                            (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                            _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

                // Normalize gradient step
                float s_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
                if (s_norm > 0.001f) {
                    float s_inv = 1.0f / s_norm;
                    s0 *= s_inv;
                    s1 *= s_inv;
                    s2 *= s_inv;
                    s3 *= s_inv;
                }

                // Apply feedback
                qDot0 -= beta_ * s0;
                qDot1 -= beta_ * s1;
                qDot2 -= beta_ * s2;
                qDot3 -= beta_ * s3;
            }
        } else {
            // IMU-only gradient descent (no magnetometer)
            float _2q0 = 2.0f * q0;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _4q0 = 4.0f * q0;
            float _4q1 = 4.0f * q1;
            float _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1;
            float _8q2 = 8.0f * q2;
            float q0q0 = q0 * q0;
            float q1q1 = q1 * q1;
            float q2q2 = q2 * q2;
            float q3q3 = q3 * q3;

            float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

            float s_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (s_norm > 0.001f) {
                float s_inv = 1.0f / s_norm;
                s0 *= s_inv;
                s1 *= s_inv;
                s2 *= s_inv;
                s3 *= s_inv;
            }

            qDot0 -= beta_ * s0;
            qDot1 -= beta_ * s1;
            qDot2 -= beta_ * s2;
            qDot3 -= beta_ * s3;
        }
    }

    // Integrate rate of change
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalize quaternion
    q_.w = q0;
    q_.x = q1;
    q_.y = q2;
    q_.z = q3;
    q_.normalize();
}
