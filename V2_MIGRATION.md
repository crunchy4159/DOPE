# DOPE V2 Migration Notes

## Breaking Direction
- The preferred runtime configuration path is now table-first via `AmmoDatasetV2`.
- `BulletProfile` remains available as sparse/legacy fallback, but DOPE 2.0 callers should migrate to:
  - `DOPE_SetAmmoDatasetV2`
  - `DOPE_SetBallisticContext`
  - `DOPE_SetRifleAmmoCalibrationProfile`
  - `DOPE_SetModuleCapabilities`
  - `DOPE_RecordShotObservation`
  - `DOPE_RecordRadarObservation`

## Runtime Precedence
1. Manufacturer dataset tables (trajectory/velocity/wind/energy/CEP).
2. Baseline-to-current atmospheric and wind scaling.
3. Rifle+ammo calibration profile corrections.
4. Solver fallback only for missing channels (if enabled by capabilities).

## Dataset import and precompute recommendations
- Datasets SHOULD declare their baseline convention via `baseline_convention` (e.g., `METRO` or `ICAO`) and populate explicit baseline fields (`baseline_pressure_pa`, `baseline_temperature_c`, `baseline_humidity`).
- Importers SHOULD precompute a dense cached trajectory (preferred 1 m step) and populate `cached_full_table_present` and `cached_full_trajectory` when possible. DOPE will use cached tables on the fast path to avoid runtime integration.
- If the importer cannot generate a dense cache, the dataset must be marked sparse and the engine may fall back to the solver for missing channels if `enable_solver_fallback` is enabled.

If a table channel is missing and solver fallback is disabled, output degrades deterministically and the channel is not synthesized.

## CEP/Uncertainty
- CEP normalization now resolves through one active path:
  - `AmmoDatasetV2.cep50_by_range` when present.
  - Else `UncertaintyConfig.cartridge_cep_table`.
  - Else `UncertaintyConfig.ammo_cep50_moa` scalar.

## Modularity
- Module capabilities gate optional behavior at runtime:
  - solver fallback
  - radar assist
  - shot-learning
  - deferred uncertainty refinement
