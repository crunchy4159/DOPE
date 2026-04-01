/**
 * @file test_helpers_v2.h
 * @brief Shared AmmoDatasetV2 helpers for DOPE V2 test suite.
 *
 * All tests use AmmoDatasetV2 + DOPE_SetAmmoDatasetV2. BulletProfile is gone.
 */
#pragma once
#include "dope/dope_api.h"
#include <cstring>

// ---------------------------------------------------------------------------
// Standard .308 Win 175gr Sierra MatchKing dataset (sparse fallback only).
// Provides BC + MV so the RK4 solver can run as fallback when no table data
// is present. Tests that need table-first behaviour should populate
// trajectories[] and velocity_by_range[] themselves.
// ---------------------------------------------------------------------------
inline AmmoDatasetV2 Make308V2(float twist_rate_inches = 10.0f,
                                float bc = 0.505f,
                                float mv_ms = 792.0f) {
    AmmoDatasetV2 d = {};
    d.source_confidence = 0.9f;
    d.baseline_temperature_c = 15.0f;
    d.baseline_pressure_pa   = 101325.0f;
    d.baseline_humidity      = 0.5f;
    d.baseline_barrel_length_in = 24.0f;
    d.baseline_wind_speed_ms = 0.0f;
    d.baseline_convention    = BaselineConvention::ICAO;

    // Sparse fallback params (used by RK4 when no trajectory table present)
    d.bc                  = bc;
    d.drag_model          = DragModel::G1;
    d.muzzle_velocity_ms  = mv_ms;
    d.mass_grains         = 175.0f;
    d.length_mm           = 31.2f;
    d.caliber_inches      = 0.308f;
    d.twist_rate_inches   = twist_rate_inches;
    d.mv_adjustment_fps_per_in = 25.0f;
    return d;
}

// .308 Win 168gr G7 variant
inline AmmoDatasetV2 Make308G7V2(float mv_ms = 838.2f) {
    AmmoDatasetV2 d = Make308V2(10.0f, 0.508f, mv_ms);
    d.drag_model = DragModel::G7;
    d.mass_grains = 168.0f;
    return d;
}

// .308 Win 150gr Federal Fusion
inline AmmoDatasetV2 MakeFedFusion150V2() {
    AmmoDatasetV2 d = Make308V2(10.0f, 0.414f, 2820.0f * 0.3048f);
    d.mass_grains = 150.0f;
    d.length_mm   = 28.0f;
    return d;
}

// .223 Rem 55gr
inline AmmoDatasetV2 Make223V2() {
    AmmoDatasetV2 d = {};
    d.source_confidence = 0.9f;
    d.baseline_temperature_c = 15.0f;
    d.baseline_pressure_pa   = 101325.0f;
    d.baseline_humidity      = 0.5f;
    d.baseline_barrel_length_in = 24.0f;
    d.baseline_wind_speed_ms = 0.0f;
    d.baseline_convention    = BaselineConvention::ICAO;
    d.bc                  = 0.245f;
    d.drag_model          = DragModel::G1;
    d.muzzle_velocity_ms  = 990.0f;
    d.mass_grains         = 55.0f;
    d.length_mm           = 19.0f;
    d.caliber_inches      = 0.223f;
    d.twist_rate_inches   = 12.0f;
    d.mv_adjustment_fps_per_in = 25.0f;
    return d;
}

// Enable solver fallback so RK4 runs for sparse datasets
inline void EnableSolverFallback() {
    ModuleCapabilities caps = {};
    caps.enable_solver_fallback    = true;
    caps.enable_shot_learning      = true;
    caps.enable_uncertainty_refine = true;
    DOPE_SetModuleCapabilities(&caps);
}
