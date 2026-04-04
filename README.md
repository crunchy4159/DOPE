# DOPE

**Digital Open-Source Precise Extrapolator**

DOPE 2.0 engine: a C++17 ballistic trajectory computation library targeting ESP32-P4 @ 400MHz and native desktop GUI for debugging and testing.

DOPE 2.0 runs a table-first hybrid path (`AmmoDatasetV2`) with deterministic solver fallback, runtime context scaling, and persistent rifle+ammo calibration. See `V2_MIGRATION.md`.

**Recent updates (2026-03-04):** LOS zero now aligns at bore crossing, Coriolis windage is azimuth-aware, RK4 per-metre outputs are interpolated, and drag retains density sensitivity alongside corrected BC.

--- 

XMR Donation Address: 8BWmYeEc8xQekZXC29ATz4aLagtw4y1U7JxFxrZFYyoaLDccPjTT6KRYvEVYeirr3M9p7ZQsvJSDeQUctB68wZPaDvZ1ifu

---

## What It Does

The engine ingests normalized sensor data, performs ballistic trajectory computation with atmospheric and Earth-rotation corrections, and produces a structured `FiringSolution`. It is platform-agnostic, display-agnostic, UI-agnostic, and vision-agnostic.

For margin/error modeling, the engine outputs analytic uncertainty primitives (sigma/covariance/per-input variance); presentation-layer confidence rings, heatmaps, or scores belong in the application/UI layer.

Barrel modeling covers stiffness/CEP hierarchy, free-float vs contact, suppressor and tuner multipliers, and heat/stringing. Feed shot events via `DOPE_NotifyShotFired(timestamp_us, ambient_temp_c)` to let the barrel-side dispersion grow with cadence and cool over time.

Note: The `GunProfile` now exposes `angular_sigma_moa` (1-σ mechanical angular dispersion in MOA). When provided the engine folds this angular component into the uncertainty model (see `MATH_REFERENCE.md §16.4`). A focused unit test `test_barrel_angular_sigma.cpp` verifies this behavior.

It does not render graphics, process camera/LiDAR data, or select targets.

## Architecture

```
┌───────────────────────────────┐
│        UI / Application       │
├───────────────────────────────┤
│   Ballistic Core Engine       │  ← This library
│  - AHRS (Madgwick / Mahony)   │
│  - Atmosphere (4-factor BC)   │
│  - Drag (G1–G8 tables)        │
│  - Solver (RK4 adaptive)      │
│  - Coriolis / Eötvös          │
│  - Spin drift / Cant          │
├───────────────────────────────┤
│       Sensor Drivers          │
└───────────────────────────────┘
```

## Project Structure

```
├── platformio.ini                # Build config (esp32sim + native + native_gui)
├── lib/dope/                     # DOPE engine library (platform-agnostic)
│   ├── include/dope/             # Public headers
│   │   ├── dope_api.h            # C-linkage entry points
│   │   ├── dope_types.h          # All data structures
│   │   ├── dope_config.h         # Compile-time constants
│   │   └── dope_math_utils.h     # Unit conversion & math constants
│   └── src/                      # Implementation
│       ├── ahrs/                 # Madgwick + Mahony filters
│       ├── atmo/                 # Atmospheric model & BC correction
│       ├── drag/                 # G1–G8 drag tables & lookup
│       ├── solver/               # Trajectory integrator & zero solver
│       ├── corrections/          # Wind, cant, spin drift, Coriolis
│       ├── mag/                  # Magnetometer calibration
│       ├── engine/               # Top-level orchestrator
│       └── dope_api.cpp          # C-linkage API wrapper
├── src/main.cpp                  # ESP32 app main (thin harness)
├── test/                         # GoogleTest suites (native env)
│   ├── test_ahrs.cpp
│   ├── test_atmosphere.cpp
│   ├── test_cartridges.cpp
│   ├── test_corrections.cpp
│   ├── test_drag.cpp
│   ├── test_integration.cpp
│   ├── test_mag.cpp
│   ├── test_main.cpp
│   ├── test_solver.cpp
│   └── test_uncertainty.cpp
├── tools/native_gui/             # Windows ImGui desktop harness
│   └── gui_main.cpp              # Full GUI implementation
└── DOPE SRS.md                   # Software Requirements Specification
```

## Engine Boundary (What Is "Real DOPE" vs Test/Tooling)

Treat `lib/dope/` as the actual DOPE ballistic engine. Everything outside that folder is integration code, host tooling, or validation scaffolding.

**Production engine (DOPE core):**
- `lib/dope/include/dope/` — public engine API/types/config/math-utils
- `lib/dope/src/` — solver, drag, atmosphere, corrections, AHRS, mag calibration, and orchestrator
- `lib/dope/src/dope_api.cpp` — C-linkage wrapper over core engine internals

**Not engine logic (test/harness/support code):**
- `test/` — GoogleTest validation suites and reference-envelope checks
- `tools/native_gui/gui_main.cpp` and `tools/native_gui/imgui_*` — desktop GUI harness for manual experiments
- `src/main.cpp` — thin firmware/native entry point
- `third_party/` — vendored dependencies (e.g., Dear ImGui)
- `scripts/` — developer launch helpers (`run_native_gui.ps1`, `run_native_gui.bat`)

Rule of thumb: if a change affects ballistic behavior in deployed firmware, it should be in `lib/dope/`; if it only affects how you run, visualize, or verify the engine, it belongs in harness/test/tooling code.

## Build

Requires [PlatformIO](https://platformio.org/).

### Desktop (native tests)

```bash
py -m platformio test -e native
```

### Desktop (native GUI)

Dear ImGui is vendored in `third_party/imgui/` — no setup step needed.

#### Windows

Build:
```bash
py -m platformio run -e native_gui
```
Launch: `.pio/build/native_gui/program.exe`  
Launch helper: `scripts\\run_native_gui.bat` or `./scripts/run_native_gui.ps1`

#### Linux

Prerequisites: GLFW3 dev libraries (`sudo apt install libglfw3-dev` on Debian/Ubuntu).  
First-time setup: `./scripts/setup_linux.sh`

Build:
```bash
py -m platformio run -e native_gui_linux
```
Launch: `.pio/build/native_gui_linux/program`  
Launch helper: `./scripts/run_native_gui.sh`

If PlatformIO is missing, install one of:

```bash
py -m pip install -U platformio
```

```bash
pip install -U platformio
```

GUI controls include:
- Dear ImGui desktop UI (Win32 + DirectX11)
- Bullet/zero/wind/latitude manual inputs
- Cartridge preset manager (add/update/apply/remove from current inputs)
- Rifle preset manager (add/update/apply/remove from current inputs)
- Shared profile library save/load (`dope_gui_profile_library.json` by default)
- Sensor validity toggles + baro/LRF fields
- `Apply Config`, `Step Update`, `Run 100`, `Reset Engine`
- Live solution panel (`mode`, `fault`, `diag`, holds, TOF, velocity, energy)
- Velocity fields display FPS (muzzle, wind, and velocity at target)
- Barometric pressure input is always `Pa` (even when `Imperial` is selected)
- Preset `Save` / `Load` (JSON via preset path field)

Dear ImGui is vendored at third_party/imgui/ (committed to the repo). To upgrade, delete the folder contents, re-run scripts/setup_imgui.ps1 with the desired version tag, and commit the result.

### ESP32-P4 (cross-compile)

```bash
pio run -e esp32p4
```

## API Quick Start (DOPE 2.0)

```cpp
#include "dope/dope_api.h"

// Initialize
DOPE_Init();

// Configure v2 ammo dataset (table-first)
AmmoDatasetV2 ammo = {};
ammo.num_trajectories = 1;
ammo.trajectories[0].zero_range_m = 100.0f;
ammo.trajectories[0].num_points = 2;
ammo.trajectories[0].points[0] = {100.0f, 0.0f};
ammo.trajectories[0].points[1] = {500.0f, -4.2f};
ammo.bc = 0.505f;                  // sparse fallback
ammo.drag_model = DragModel::G1;   // sparse fallback
ammo.muzzle_velocity_ms = 792.0f;  // sparse fallback
ammo.mass_grains = 175.0f;
ammo.length_mm = 31.2f;
ammo.caliber_inches = 0.308f;
ammo.twist_rate_inches = 10.0f;
DOPE_SetAmmoDatasetV2(&ammo);

// Set zero
ZeroConfig zero = {};
zero.zero_range_m = 100.0f;
zero.sight_height_mm = 38.1f;
DOPE_SetZeroConfig(&zero);

// Optional runtime context and calibration
BallisticContext ctx = {};
ctx.use_runtime_wind = true;
ctx.wind_speed_ms = 3.0f;
ctx.wind_heading_deg = 90.0f;
DOPE_SetBallisticContext(&ctx);

RifleAmmoCalibrationProfile cal = {};
cal.muzzle_velocity_scale = 1.0f;
cal.uncertainty_scale = 1.0f;
DOPE_SetRifleAmmoCalibrationProfile(&cal);

// Feed sensor data each cycle
DOPE_Update(&sensorFrame);

// Read solution
if (DOPE_GetMode() == DOPE_Mode::SOLUTION_READY) {
    RealtimeSolution rt;
    DOPE_GetRealtimeSolution(&rt);
    // rt.hold_elevation_moa, rt.hold_windage_moa, rt.uncertainty_radius_moa

    // Optional: full diagnostics payload
    FiringSolution sol;
    DOPE_GetSolution(&sol);
}
```

## Tweak Map (Where To Change What)

Use this quick map when tuning behavior:

- GUI startup defaults and synthetic desktop sensor feed:
    - `tools/native_gui/gui_main.cpp` (`ResetStateDefaults`, `BuildFrame`)
- GUI button actions and update cadence:
    - `tools/native_gui/gui_main.cpp` (`Apply Config`, `Step Update`, `Run 100` handlers)
- Public DOPE entry points used by app code:
    - `lib/dope/include/dope/dope_api.h`
    - `lib/dope/src/dope_api.cpp`
- Solver and trajectory behavior:
    - `lib/dope/src/solver/solver.cpp`
- Atmosphere and BC correction model:
    - `lib/dope/src/atmo/atmosphere.cpp`
- Drag tables and drag interpolation:
    - `lib/dope/src/drag/drag_model.cpp`

Suggested order for safe tuning:
1. Adjust GUI/input defaults in `tools/native_gui/gui_main.cpp`.
2. Validate with `pio test -e native`.
3. Then tune solver/atmo/drag internals and rerun tests.

## Cartridge Reference Data Scope

- Cartridge tables/presets are for validation and harness convenience only.
- DOPE engine internals do not look up named cartridge tables; they solve from the active `BulletProfile`, `ZeroConfig`, and sensor inputs.
- Current table-like cartridge presets live in:
    - GUI harness: `tools/native_gui/gui_main.cpp` (input prefill only)
    - Tests: `test/test_cartridges.cpp` and reference-envelope integration tests
- Presets can now carry range-dependent CEP50 accuracy tables (`cep_table_moa`) and a scale floor; when provided and enabled in the GUI, the engine scales propagated uncertainty to match the CEP envelope at each range.
- Keep this boundary intact: if cartridge presets are updated, use them to verify outputs, not as embedded runtime engine data.
- For crowd testing, share GUI profile-library JSON files and compare solver outputs under known conditions.

## Key Specifications

| Spec | Value |
|------|-------|
| Max range | 2500 m |
| 1000m solve time | < 8 ms (ESP32-P4) |
| 2500m solve time | < 15 ms (ESP32-P4) |
| AHRS update | < 1 ms |
| Memory model | Static only, zero heap after init |
| Drag models | G1–G8 (all) |
| AHRS filters | Madgwick + Mahony (selectable) |
| Units | SI internally |

## Operating Modes

| Mode | Description |
|------|-------------|
| `IDLE` | Insufficient data for solution |
| `SOLUTION_READY` | Valid firing solution available |
| `FAULT` | Required inputs missing or invalid |

Range ingestion is continuous; there is no internal `RANGING` mode.

## Units Policy

All internal calculations are SI (meters, m/s, Pascals, Celsius/Kelvin, radians). Imperial conversions are only used at boundaries where required for compatibility with reference algorithms.

## Defaults and Fault Philosophy

- ISA-consistent defaults are used when optional environmental inputs are missing (pressure `101325 Pa`, temperature `15 °C`, humidity `0.5`, wind `0 m/s`, altitude `0 m`)
- Hard faults are reserved for critical missing/invalid inputs (for example no valid range, missing BC/MV, unsolvable zero, AHRS instability)
- Non-critical gaps use defaults and set diagnostic flags rather than forcing `FAULT`

## License

GPL 3.0
