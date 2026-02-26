/**
 * @file cant.h
 * @brief Cant (roll) correction — SRS §11.5
 *
 * Computes the POI shift due to rifle cant (roll angle).
 * When canted, the vertical hold produces both a vertical and
 * horizontal error at the target.
 */

#pragma once

#include "bce/bce_config.h"

class CantCorrection {
public:
    /**
     * Compute cant-corrected holds.
     *
     * When the rifle is canted by angle θ, the reticle's vertical axis
     * is no longer aligned with gravity. The elevation hold rotates:
     *   corrected_elev = elev × cos(θ)
     *   cant_windage   = elev × sin(θ)
     *
     * @param cant_angle_rad  Roll angle in radians (from AHRS)
     * @param elevation_moa   Uncorrected elevation hold (MOA)
     * @param out_elev_moa    Output: corrected elevation (MOA)
     * @param out_wind_moa    Output: additional windage from cant (MOA)
     */
    static void apply(float cant_angle_rad, float elevation_moa,
                      float& out_elev_moa, float& out_wind_moa);
};
