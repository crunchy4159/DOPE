/**
 * @file drag_model.cpp
 * @brief Drag coefficient lookup and deceleration computation.
 */

#include "drag_model.h"
#include "drag_tables.h"
#include "bce/bce_config.h"
#include <cmath>

float DragModelLookup::getCd(DragModel model, float mach) {
    if (mach < 0.0f) mach = 0.0f;

    switch (model) {
        case DragModel::G1: return interpolate(G1_TABLE, G1_TABLE_SIZE, mach);
        case DragModel::G2: return interpolate(G2_TABLE, G2_TABLE_SIZE, mach);
        case DragModel::G3: return interpolate(G3_TABLE, G3_TABLE_SIZE, mach);
        case DragModel::G4: return interpolate(G4_TABLE, G4_TABLE_SIZE, mach);
        case DragModel::G5: return interpolate(G5_TABLE, G5_TABLE_SIZE, mach);
        case DragModel::G6: return interpolate(G6_TABLE, G6_TABLE_SIZE, mach);
        case DragModel::G7: return interpolate(G7_TABLE, G7_TABLE_SIZE, mach);
        case DragModel::G8: return interpolate(G8_TABLE, G8_TABLE_SIZE, mach);
        default:            return interpolate(G1_TABLE, G1_TABLE_SIZE, mach);
    }
}

float DragModelLookup::interpolate(const DragPoint* table, int size, float mach) {
    // Clamp below minimum
    if (mach <= table[0].mach) return table[0].cd;
    // Clamp above maximum
    if (mach >= table[size - 1].mach) return table[size - 1].cd;

    // Binary search for the correct interval
    int lo = 0, hi = size - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (table[mid].mach <= mach) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    // Linear interpolation between table[lo] and table[hi]
    float frac = (mach - table[lo].mach) / (table[hi].mach - table[lo].mach);
    return table[lo].cd + frac * (table[hi].cd - table[lo].cd);
}

float DragModelLookup::getDeceleration(float velocity_ms, float speed_of_sound,
                                        float bc_corrected, DragModel model,
                                        float air_density) {
    if (velocity_ms < 1.0f) return 0.0f;
    if (bc_corrected < 0.001f) return 0.0f;

    float mach = velocity_ms / speed_of_sound;
    float cd = getCd(model, mach);

    // Classic ballistic drag formula using BC:
    //
    // The drag deceleration for a projectile with a given BC relative to
    // the standard G-model projectile is:
    //
    //   a = (Cd_ref(M) / BC) × (ρ / ρ_std) × v²  ×  (1 / reference_constant)
    //
    // Where the reference constant absorbs the standard projectile's
    // sectional density and reference area. For the G1/G7 system, the
    // standard ballistic constant maps BC in lb/in² to SI units:
    //
    //   K = 1 / (2 × 0.00068326)  ≈ 731.86  (for sectional density in lb/in²)
    //
    // But since we're applying the atmospheric BC correction already,
    // and the drag tables represent Cd_ref/Cd_std, the deceleration is:
    //
    //   a_drag = cd × (ρ / ρ_std) × v² / (BC × 0.00068326 × 2)
    //
    // Simplified constant: 1 / (2 * i_d) where i_d = BC in consistent units
    //
    // Using the formulation where BC is in (lb/in²) and velocity in m/s,
    // the conversion factor for the reference projectile is:
    //   a = cd * v^2 * ρ / (ρ_std * BC * 0.07030696)
    //
    // 0.07030696 = 2 * i_d_ref * sectional_density_conversion
    //
    // More practically, using the standard drag function approach:
    //   F_drag / m = (cd / bc_corrected) * (rho/rho_std) * v^2 * (1/C_bal)
    //
    // Where C_bal ≈ 731.86 converts BC (lb/in²) to SI retardation constant

    float density_ratio = air_density / BCE_STD_AIR_DENSITY;
    float decel = (cd * density_ratio * velocity_ms * velocity_ms) /
                  (bc_corrected * BCE_BALLISTIC_DRAG_CONSTANT);

    return decel;
}
