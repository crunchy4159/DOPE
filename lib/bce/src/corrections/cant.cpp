/**
 * @file cant.cpp
 * @brief Cant correction implementation.
 */

#include "cant.h"
#include <cmath>

void CantCorrection::apply(float cant_angle_rad, float elevation_moa,
                           float& out_elev_moa, float& out_wind_moa) {
    // When canted, the vertical hold vector rotates in the sight plane.
    // The corrected vertical component is reduced by cos(θ),
    // and a spurious horizontal component of sin(θ) appears.
    out_elev_moa = elevation_moa * std::cos(cant_angle_rad);
    out_wind_moa = elevation_moa * std::sin(cant_angle_rad);
}
