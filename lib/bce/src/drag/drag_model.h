/**
 * @file drag_model.h
 * @brief Drag coefficient lookup from standard G-model tables.
 *
 * Uses piecewise linear interpolation on the (Mach, Cd) table for
 * the selected drag model. Binary search for O(log n) Mach lookup.
 */

#pragma once

#include "bce/bce_types.h"

class DragModelLookup {
public:
    /**
     * Look up drag coefficient for a given Mach number and drag model.
     * @param model  G1–G8 drag model
     * @param mach   Mach number (≥ 0)
     * @return Drag coefficient Cd
     */
    static float getCd(DragModel model, float mach);

    /**
     * Compute the retardation (deceleration) of the projectile.
     *
     * a_drag = (Cd × ρ × A × v²) / (2 × m)
     *
     * But using the BC formulation:
     * a_drag = (Cd_standard / BC) × (ρ / ρ_std) × v² × (reference_area / reference_mass)
     *
     * Simplified with BC definition:
     * a_drag = (Cd_ref(mach) × v²) / (BC × ballistic_constant)
     *
     * @param velocity_ms     Current velocity (m/s)
     * @param speed_of_sound  Speed of sound (m/s) for Mach calculation
     * @param bc_corrected    Atmospherically-corrected BC
     * @param model           Drag model
     * @param air_density     Current air density (kg/m³)
     * @return Deceleration magnitude (m/s², positive)
     */
    static float getDeceleration(float velocity_ms, float speed_of_sound,
                                  float bc_corrected, DragModel model,
                                  float air_density);

private:
    static float interpolate(const struct DragPoint* table, int size, float mach);
};
