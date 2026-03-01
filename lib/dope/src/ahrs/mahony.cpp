/**
 * @file mahony.cpp
 * @brief Mahony complementary AHRS filter implementation.
 */

#include "mahony.h"
#include <cmath>

void MahonyFilter::update(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float mx, float my, float mz,
                          bool use_mag, float dt) {
    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

    // Error is sum of cross products between estimated and measured directions    [MATH §6.6]
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    // Normalize accelerometer
    float a_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (a_norm > 0.001f) {
        float a_inv = 1.0f / a_norm;
        ax *= a_inv;
        ay *= a_inv;
        az *= a_inv;

        // Estimated gravity direction from quaternion    [MATH §6.7]
        float vx = 2.0f * (q1 * q3 - q0 * q2); // [MATH §6.7]
        float vy = 2.0f * (q0 * q1 + q2 * q3); // [MATH §6.7]
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // [MATH §6.7]

        // Cross product: accel × estimated_gravity    [MATH §6.8]
        ex += ay * vz - az * vy; // [MATH §6.8]
        ey += az * vx - ax * vz; // [MATH §6.8]
        ez += ax * vy - ay * vx; // [MATH §6.8]
    }

    if (use_mag) {
        float m_norm = std::sqrt(mx * mx + my * my + mz * mz);
        if (m_norm > 0.001f) {
            float m_inv = 1.0f / m_norm;
            mx *= m_inv;
            my *= m_inv;
            mz *= m_inv;

            // Reference direction of Earth's magnetic field (rotate mag to Earth frame)
            float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
            float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
            float bx = std::sqrt(hx * hx + hy * hy);
            float bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

            // Estimated magnetic field direction from quaternion
            float wx = bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2);
            float wy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3);
            float wz = bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2);

            // Cross product: mag × estimated_mag
            ex += my * wz - mz * wy;
            ey += mz * wx - mx * wz;
            ez += mx * wy - my * wx;
        }
    }

    // Apply integral feedback (if ki > 0)    [MATH §6.9]
    if (ki_ > 0.0f) {
        integral_fb_x_ += ki_ * ex * dt; // [MATH §6.9]
        integral_fb_y_ += ki_ * ey * dt; // [MATH §6.9]
        integral_fb_z_ += ki_ * ez * dt; // [MATH §6.9]
        gx += integral_fb_x_;
        gy += integral_fb_y_;
        gz += integral_fb_z_;
    }

    // Apply proportional feedback    [MATH §6.9]
    gx += kp_ * ex; // [MATH §6.9]
    gy += kp_ * ey; // [MATH §6.9]
    gz += kp_ * ez; // [MATH §6.9]

    // Integrate rate of change of quaternion    [MATH §6.5]
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz); // [MATH §6.5]
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy); // [MATH §6.5]
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx); // [MATH §6.5]
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx); // [MATH §6.5]

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    q_.w = q0;
    q_.x = q1;
    q_.y = q2;
    q_.z = q3;
    q_.normalize();
}
