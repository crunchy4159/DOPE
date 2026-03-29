Handoff: Replace ProfileTrajectoryPoint → DOPE_TrajectoryPoint

Summary
- Replaced an undefined type `ProfileTrajectoryPoint` with the correct `DOPE_TrajectoryPoint`.

What I changed
- [lib/dope/src/solver/solver.h](lib/dope/src/solver/solver.h):
  - `SolverParams::trajectory_profile` now uses `DOPE_TrajectoryPoint` instead of `ProfileTrajectoryPoint`.
  - `BallisticSolver::interpolateSeries` declaration now accepts `const DOPE_TrajectoryPoint*`.

What I checked
- Grep/search: no other literal occurrences of `ProfileTrajectoryPoint` remain in the workspace.
- `lib/dope/include/dope/dope_types.h` defines `DOPE_TrajectoryPoint` and `DOPE_ProfilePoint`.
- `lib/dope/src/solver/solver.cpp` was inspected; it uses the internal runtime `TrajectoryPoint` table (`TrajectoryPoint table_[...]`) and did not require edits.
- `tools/native_gui/gui_main.cpp` and related GUI import code use `DOPE_TrajectoryPoint` and `DOPE_TrajectoryFamily` (no action required).

Possible failure modes and debug checklist
1. Missing/old references
   - Run: `git grep -n "ProfileTrajectoryPoint" || true` to confirm nothing remained.
2. Mismatched declaration vs implementation
   - If you see a compile error about `interpolateSeries` signature mismatch, search for any implementation or definition using the old signature:
     - `git grep -n "interpolateSeries" || true`
   - If an implementation exists using the old type, update it to `DOPE_TrajectoryPoint`.
3. Missing includes
   - If errors say `DOPE_TrajectoryPoint` is unknown in a TU, ensure the TU includes `dope_types.h` or `dope/dope_types.h` and that include paths are correct.
4. Name confusion
   - Be aware there are two similar structs:
     - `DOPE_TrajectoryPoint` (import/cache data: `.range_m`, `.drop_m`)
     - `TrajectoryPoint` (solver runtime table: `.drop_m`, `.windage_m`, `.velocity_ms`, etc.)
   - Verify callers use the intended type.
5. Rebuild requirement
   - After header changes, do a clean rebuild: object files that included the header must be recompiled.

Commands to run (PowerShell)

```powershell
# Activate venv
& .venv\Scripts\Activate.ps1
# Full test run (native)
python -m platformio test -e native
# Or run a clean build then tests
python -m platformio run -e native --target clean
python -m platformio test -e native
# Quick searches
git grep -n "ProfileTrajectoryPoint" || true
git grep -n "interpolateSeries" || true
```

If tests fail
- Capture the first failing compiler/linker error and paste it here.
- If compile errors complain about missing symbols or signature mismatches, run the grep steps above and recompile the TU that declares/defines the mismatched symbol.

Notes
- I edited only the header: [lib/dope/src/solver/solver.h](lib/dope/src/solver/solver.h). No other files were modified.
- PlatformIO was installed in the repo `.venv` during my checks so `python -m platformio` should work from the venv.

Next steps for the debugging agent
- Run the commands above to reproduce the failure (if any).
- If a failing TU or symbol is identified, update the implementation signatures or includes to match the header change and re-run tests.

If you want, I can:
- Open the failing translation unit and apply the fix.
- Run the tests here and iterate on errors.

-- End of handoff


Handoff Summary

Goal: Resolve remaining Uncertainty test failures after migrating ballistics to AmmoDatasetV2 and keeping GunProfile for hardware.
Failing tests: UncertaintyTest.AllZeroSigmaProducesZeroUncertainty, UncertaintyTest.BCSigmaAffectsElevation, UncertaintyTest.LengthSigmaContributesToUncertainty.
Repro commands (PowerShell):
Run single failing test:
Run all Uncertainty tests:
If debug prints missing, rebuild with the debug define:
Key files to inspect

dope_engine.cpp — computeUncertainty(), refreshDerivedSigmasFromProfiles(), selectActiveAmmo()
computeUncertainty around: dope_engine.cpp:1550-2040
refreshDerivedSigmasFromProfiles(): dope_engine.cpp:1423-1460
selectActiveAmmo(): dope_engine.cpp:240-320
API entrypoints: dope_api.cpp (mappings for DOPE_SetAmmoDatasetV2 / DOPE_SetGunProfile)
dope_api.cpp:1-120
GUI surface touched: gui_main.cpp (most g_state.bullet → g_state.ammo/g_state.gun edits)
gui_main.cpp
Tests: test_uncertainty.cpp (failing assertions & how tests set up dataset/config)
test_uncertainty.cpp:1-220
What to look at first (priority)

Reproduce a failing test with debug prints enabled. The code currently emits lines:
UNCERT: var_e=... var_w=... cov=... sigma_e=... sigma_w=... cep50=... has_bullet=...
Debug print location: computeUncertainty() guarded by #ifdef DOPE_DEBUG_UNCERTAINTY.
Verify getActiveAmmoCep50(slant_range) returns dataset CEP50 (from dataset_v2_.cep50_by_range) when present; confirm fallback logic that uses bullet_.measured_cep50_moa / bullet_.manufacturer_spec_moa only applies when dataset CEP50 absent.
Inspect refreshDerivedSigmasFromProfiles() to confirm it updates uncertainty_config_ fields (e.g., sigma_muzzle_velocity_ms, sigma_range_m) correctly based on dataset / gun profile and live sensor inputs.
Confirm selectActiveAmmo() returns is_v2=true and populated fields from dataset_v2_ (BC, mv, mass, length, mv_adjustment_fps_per_in). Ensure getActiveAmmoCep50() uses dataset_v2_.uncertainty_sigma_by_range or cep50_by_range as intended.
Check has_bullet_ usage: after migration, GunProfile (alias BulletProfile) should supply hardware dispersion (stiffness, measured CEP) only; ensure has_bullet_ is set when DOPE_SetGunProfile() is called by GUI/tests.
Immediate hypotheses

CEP50 scaling path: tests expect base dispersion injected even when input sigmas are zero (hardware or dataset CEP50). Logs show cep50=0 frequently; confirm dataset CEP50 lookup is empty or not used.
Per-input channels (indices 12 and 14) expected by tests: ensure solution_.uc_var_elev[12] (MV adjustment) and [14] (ammo dispersion) are populated as before.
computeUncertainty numeric stability: double accumulation was introduced — verify it didn’t change the per-input indexing or channel updates (some indices in code were adjusted during iteration).
Recommended debug steps

Run one failing test with debug prints to capture a full sequence of UNCERT: lines.
In computeUncertainty():
Log getActiveAmmoCep50() return and whether dataset num_cep50_points or uncertainty_sigma_by_range are nonzero.
Log has_bullet_ and bullet_ fields used for gun_sigma.
Confirm indices used to populate solution_.uc_var_elev[] match FiringSolution::kNumUncertaintyInputs mapping in dope_types.h.
In refreshDerivedSigmasFromProfiles():
Confirm dataset-derived sigmas are applied when has_dataset_v2_ is true and that uncertainty_config_.ammo_cep50_moa or cartridge_cep_table handling is correct.
If dataset CEP50 is intended but absent, check test setup in test_helpers_v2.h and test_uncertainty.cpp to ensure Make308V2() populates cep50_by_range (or set uc.ammo_cep50_moa manually in the test to isolate).
Run failing tests under debugger or capture full stderr/stdout to correlate UNCERT: traces with test assertions.
Cleanup & final steps

After fixes, rerun the full native test suite:
Remove DOPE_DEBUG_UNCERTAINTY prints and the temporary compile flag once tests pass.
Sweep for any remaining BulletProfile or g_state.bullet references and migrate to GunProfile / AmmoDatasetV2 as appropriate.
If you want, I can now:

Run a single failing test with the correct PowerShell invocation and collect its debug output, or
Re-run a grep to list all remaining BulletProfile/g_state.bullet occurrences and produce a patch.