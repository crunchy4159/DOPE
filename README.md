# DOPE-ASS

**Digital Optic Precision Engine — Adaptive Smart Scope**

Ballistic Core Engine (BCE) implementing the DOPE SRS v1.31 draft (2026-02-25): a C++17 ballistic trajectory computation library targeting the ESP32-P4 @ 400MHz.

## What It Does

The engine ingests normalized sensor data, performs ballistic trajectory computation with atmospheric and Earth-rotation corrections, and produces a structured `FiringSolution`. It is platform-agnostic, display-agnostic, UI-agnostic, and vision-agnostic.

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
│  - Solver (RK2 adaptive)      │
│  - Coriolis / Eötvös          │
│  - Spin drift / Cant          │
├───────────────────────────────┤
│       Sensor Drivers          │
└───────────────────────────────┘
```

## Project Structure

```
├── platformio.ini                # Build config (esp32p4 + native)
├── lib/bce/                      # BCE library (platform-agnostic)
│   ├── include/bce/              # Public headers
│   │   ├── bce_api.h             # C-linkage entry points
│   │   ├── bce_types.h           # All data structures
│   │   └── bce_config.h          # Compile-time constants
│   └── src/                      # Implementation
│       ├── ahrs/                 # Madgwick + Mahony filters
│       ├── atmo/                 # Atmospheric model & BC correction
│       ├── drag/                 # G1–G8 drag tables & lookup
│       ├── solver/               # Trajectory integrator & zero solver
│       ├── corrections/          # Wind, cant
│       ├── mag/                  # Magnetometer calibration
│       ├── engine/               # Top-level orchestrator
│       └── bce_api.cpp           # C-linkage API wrapper
├── src/main.cpp                  # ESP32 app main (thin harness)
├── test/                         # GoogleTest suites (native env)
│   ├── test_ahrs.cpp
│   ├── test_atmosphere.cpp
│   ├── test_corrections.cpp
│   ├── test_drag.cpp
│   ├── test_integration.cpp
│   └── test_mag.cpp
└── DOPE-ASS SRS.md               # Software Requirements Specification
```

## Engine Boundary (What Is "Real DOPE" vs Test/Tooling)

Treat `lib/bce/` as the actual DOPE ballistic engine. Everything outside that folder is integration code, host tooling, or validation scaffolding.

**Production engine (DOPE core):**
- `lib/bce/include/bce/` — public engine API/types/config
- `lib/bce/src/` — solver, drag, atmosphere, corrections, AHRS, mag calibration, and orchestrator
- `lib/bce/src/bce_api.cpp` — C-linkage wrapper over core engine internals

**Not engine logic (test/harness/support code):**
- `test/` — GoogleTest validation suites and reference-envelope checks
- `src/gui_main.cpp` and `src/imgui_*` — desktop GUI harness for manual experiments
- `src/main.cpp` — thin app entry point/integration harness
- `third_party/` — vendored dependencies (e.g., Dear ImGui)
- `scripts/`, `run_native_gui.bat` — developer launch helpers

Rule of thumb: if a change affects ballistic behavior in deployed firmware, it should be in `lib/bce/`; if it only affects how you run, visualize, or verify the engine, it belongs in harness/test/tooling code.

## Build

Requires [PlatformIO](https://platformio.org/).

### Desktop (native tests)

```bash
pio test -e native
```

### Desktop (basic GUI test harness, Windows)

```bash
pio run -e native_gui
```

Then launch:

```bash
.pio/build/native_gui/program.exe
```

Or use the launcher helper (auto-detects PlatformIO command/module):

```bash
./scripts/run_native_gui.ps1
```

Or from CMD/Explorer:

```bat
run_native_gui.bat
```

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
- Shared profile library save/load (`bce_gui_profile_library.json` by default)
- Sensor validity toggles + baro/LRF fields
- `Apply Config`, `Step Update`, `Run 100`, `Reset Engine`
- Live solution panel (`mode`, `fault`, `diag`, holds, TOF, velocity, energy)
- Velocity fields display FPS (muzzle, wind, and velocity at target)
- Barometric pressure input is always `Pa` (even when `Imperial` is selected)
- Preset `Save` / `Load` (JSON via preset path field)

Vendored GUI dependency location:
- `third_party/imgui/`

### ESP32-P4 (cross-compile)

```bash
pio run -e esp32p4
```

## API Quick Start

```cpp
#include "bce/bce_api.h"

// Initialize
BCE_Init();

// Configure bullet
BulletProfile bullet = {};
bullet.bc = 0.505f;
bullet.drag_model = DragModel::G1;
bullet.muzzle_velocity_ms = 792.0f;
bullet.mass_grains = 175.0f;
bullet.caliber_inches = 0.308f;
bullet.twist_rate_inches = 10.0f;
BCE_SetBulletProfile(&bullet);

// Set zero
ZeroConfig zero = {};
zero.zero_range_m = 100.0f;
zero.sight_height_mm = 38.1f;
BCE_SetZeroConfig(&zero);

// Feed sensor data each cycle
BCE_Update(&sensorFrame);

// Read solution
if (BCE_GetMode() == BCE_Mode::SOLUTION_READY) {
    FiringSolution sol;
    BCE_GetSolution(&sol);
    // sol.hold_elevation_moa, sol.hold_windage_moa, etc.
}
```

## Tweak Map (Where To Change What)

Use this quick map when tuning behavior:

- GUI startup defaults and synthetic desktop sensor feed:
    - `src/gui_main.cpp` (`ResetStateDefaults`, `BuildFrame`)
- GUI button actions and update cadence:
    - `src/gui_main.cpp` (`Apply Config`, `Step Update`, `Run 100` handlers)
- Public BCE entry points used by app code:
    - `lib/bce/include/bce/bce_api.h`
    - `lib/bce/src/bce_api.cpp`
- Solver and trajectory behavior:
    - `lib/bce/src/solver/solver.cpp`
- Atmosphere and BC correction model:
    - `lib/bce/src/atmo/atmosphere.cpp`
- Drag tables and drag interpolation:
    - `lib/bce/src/drag/drag_model.cpp`

Suggested order for safe tuning:
1. Adjust GUI/input defaults in `src/gui_main.cpp`.
2. Validate with `pio test -e native`.
3. Then tune solver/atmo/drag internals and rerun tests.

## Cartridge Reference Data Scope

- Cartridge tables/presets are for validation and harness convenience only.
- BCE engine internals do not look up named cartridge tables; they solve from the active `BulletProfile`, `ZeroConfig`, and sensor inputs.
- Current table-like cartridge presets live in:
    - GUI harness: `src/gui_main.cpp` (input prefill only)
    - Tests: `test/test_cartridges.cpp` and reference-envelope integration tests
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

Proprietary. All rights reserved.
