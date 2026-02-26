/**
 * @file mahony.h
 * @brief Mahony complementary AHRS filter.
 *
 * Reference: R. Mahony, T. Hamel, J-M. Pflimlin, "Nonlinear
 *            Complementary Filters on the Special Orthogonal Group", 2008.
 */

#pragma once

#include "ahrs_interface.h"
#include "bce/bce_config.h"

class MahonyFilter : public AHRS_Interface {
public:
    MahonyFilter() { reset(); }

    void setGains(float kp, float ki) { kp_ = kp; ki_ = ki; }

    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz,
                bool use_mag, float dt) override;

    void reset() override {
        q_ = {1.0f, 0.0f, 0.0f, 0.0f};
        integral_fb_x_ = 0.0f;
        integral_fb_y_ = 0.0f;
        integral_fb_z_ = 0.0f;
    }

    Quaternion getQuaternion() const override { return q_; }

private:
    Quaternion q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    float kp_ = BCE_MAHONY_DEFAULT_KP;
    float ki_ = BCE_MAHONY_DEFAULT_KI;

    // Integral error terms for PI controller
    float integral_fb_x_ = 0.0f;
    float integral_fb_y_ = 0.0f;
    float integral_fb_z_ = 0.0f;
};
