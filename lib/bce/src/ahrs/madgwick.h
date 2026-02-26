/**
 * @file madgwick.h
 * @brief Madgwick gradient-descent AHRS filter.
 *
 * Reference: S. Madgwick, "An efficient orientation filter for inertial
 *            and inertial/magnetic sensor arrays", 2010.
 */

#pragma once

#include "ahrs_interface.h"
#include "bce/bce_config.h"

class MadgwickFilter : public AHRS_Interface {
public:
    MadgwickFilter() { reset(); }

    void setBeta(float beta) { beta_ = beta; }

    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz,
                bool use_mag, float dt) override;

    void reset() override {
        q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    }

    Quaternion getQuaternion() const override { return q_; }

private:
    Quaternion q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    float beta_ = BCE_MADGWICK_DEFAULT_BETA;
};
