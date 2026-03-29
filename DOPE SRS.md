# Software Requirements Specification

# DOPE — Digital Open-Source Precise Extrapolator

## Version 2.0 — DRAFT

**Date:** 2026-03-25  
**Language Target:** C++17  
**Primary Platform:** ESP32-P4 @ 400MHz

---

# 1. Introduction

## 1.1 Purpose

This document defines the requirements for **DOPE** (Digital Open-Source Precise Extrapolator), a C++ library targeting the ESP32-P4 microcontroller.

DOPE:

- Ingests normalized sensor data
    
- Performs ballistic trajectory computation
    
- Applies atmospheric and Earth-rotation corrections
    
- Produces a structured `FiringSolution`
    

DOPE is:

- Platform-agnostic
    
- Display-agnostic
    
- UI-agnostic
    
- Vision-agnostic
    

It does not render graphics, manage modes, process images, or select targets.

---

## 1.2 Architectural Boundary

DOPE:

- Consumes scalar numeric inputs
    
- Produces scalar numeric outputs
    

DOPE does **not**:

- Process camera frames
    
- Process LiDAR point clouds
    
- Perform target selection
    
- Store depth maps
    
- Manage user profiles
    
- Handle rendering
    

If LiDAR mapping or vision processing exists, it is handled by the Application layer. The Application selects one target point and passes a single range and orientation into DOPE.

DOPE computes one firing solution at a time.

For this repository, the production DOPE engine scope is `lib/dope/`; desktop GUI harness code (`tools/native_gui/gui_main.cpp`, `tools/native_gui/imgui_*`), test suites (`test/`), helper scripts, and third-party dependencies are verification/integration tooling and are not normative engine logic.

Segmentation (DOPE vs DOPE-ASS):

- DOPE (core engine) is a pure numeric engine that accepts sanitized, normalized numeric inputs (for example: orientation quaternions, scalar environment values, ranges, and calibration residuals) and deterministic metadata (dataset provenance). It performs ballistic math, table lookups, uncertainty propagation, and returns `FiringSolution` payloads. DOPE MUST NOT include hardware drivers, raw-sensor ingestion, protocol parsing (e.g., NMEA, UART framing), GNSS stack logic, or sensor-fusion algorithms. The engine's public API expects already-sanitized values supplied by the application.
- DOPE-ASS (application / host) is responsible for all sensor drivers, raw-sensor ingestion, protocol handling, calibration, and sensor-processing tasks (AHRS, magnetometer calibration/dip inversion, GNSS resolution). DOPE-ASS converts raw sensors and driver outputs into the normalized `SensorFrame` (see Section 7) and passes only those sanitized values into DOPE. DOPE-ASS also handles dataset import/validation, offline precompute jobs, persistence, and UI.

This strict separation ensures the engine remains lightweight, deterministic, and portable across targets; application-specific concerns (drivers, camera processing, data import, and GUI sensor presets) remain in DOPE-ASS where they can be platform-tailored.

---

# 2. System Architecture

```
┌───────────────────────────────┐
│        UI / Application       │
│  - Profiles                   │
│  - LiDAR mapping              │
│  - Target selection           │
│  - Rendering                  │
├───────────────────────────────┤
│   DOPE (Ballistic Engine)     │
│  - Atmosphere                 │
│  - Drag integration           │
│  - Coriolis / Eötvös          │
│  - Spin drift                 │
│  - Cant correction            │
├───────────────────────────────┤
│       Sensor Drivers /        │
│       Sensor Processing       │
│  - IMU (raw samples)          │
│  - Magnetometer (raw samples) │
│  - Barometer                  │
│  - LRF (raw driver/protocol)  │
│  - Encoder                    │
│  - AHRS / sensor fusion       |
|    (produces sanitized        |
| quaternion in `SensorFrame`)  │
└───────────────────────────────┘
```

---

# 3. Operating Modes

|Mode|Description|
|---|---|
|IDLE|Insufficient data for solution|
|SOLUTION_READY|Valid firing solution available|
|FAULT|Required inputs missing or invalid|

There is no internal RANGING mode. Range ingestion is continuous.

---

# 4. Units Policy

All internal calculations use SI units:

- Distance: meters
    
- Velocity: m/s
    
- Pressure: Pascals
    
- Temperature: Celsius (converted to Kelvin internally)
    
- Mass: kg
    
- Angles: radians internally
    

Imperial units are converted at the input boundary only when required by legacy reference algorithms.

---

# 5. Default Model

## 5.1 Internal ISA Defaults

If no overrides provided, the engine will use ISA fallback values as a safety net **only**. These fallbacks are intended for bench-testing, GUI demos, or partial sensor setups and are not a substitute for live sensor inputs in production use. When a fallback is used the engine sets diagnostic flags in `defaults_active` so the application can surface a warning; applications MAY treat fallback usage as a hard FAULT if they require strict sensor validity.

Fallback values (used only when no runtime sensor/override is present):

- Altitude: 0 m
- Pressure: 101325 Pa
- Temperature: 15 °C
- Humidity: 0.5
- Wind: 0 m/s
- Latitude: unset (GPS/GNSS used if fed by application; magnetometer-derived used if available; Coriolis disabled otherwise)

These values follow the International Standard Atmosphere (ISA) conventions and are purely fallbacks; production applications shall provide live sensor data or explicit `DOPE_SetDefaultOverrides()` values.

---

## 5.2 Default Override Mechanism

```cpp
struct DOPE_DefaultOverrides {
    bool use_altitude;
    float altitude_m;

    bool use_pressure;
    float pressure_pa;

    bool use_temperature;
    float temperature_c;

    bool use_humidity;
    float humidity_fraction;

    bool use_wind;
    float wind_speed_ms;
    float wind_heading_deg;

    bool use_latitude;
    float latitude_deg;
};
```

```cpp
void DOPE_SetDefaultOverrides(const DOPE_DefaultOverrides* defaults);
```

Application may load user profile defaults at startup.

---

# 6. Maximum Trajectory Range

Compile-time constant:

```cpp
constexpr uint32_t DOPE_MAX_RANGE_M = 2500;
```

- Supports current 2km LRF
    
- Allows stronger LRF upgrades
    
- Static memory allocation only
    
- No dynamic heap usage
    

Application may configure a lower operational limit.

---

# 7. Sensor Ingestion

All inputs enter through:

```cpp
void DOPE_Update(const SensorFrame* frame);
```

SensorFrame (sanitized input schema expected by DOPE):

```cpp
// DOPE expects sanitized, application-provided sensor outputs only.
// DOPE-ASS is responsible for raw drivers, protocol parsing, and
// sensor-processing (AHRS, calibration, disturbance detection).
struct SensorFrame {
        uint64_t timestamp_us;
        uint32_t flags; // bitmask: STALE, DISTURBED, FALLBACK_ACTIVE, etc.

        // Orientation (application-provided quaternion, optional)
        bool has_quaternion;
        float quaternion[4]; // w, x, y, z

        // Rangefinder (optional)
        bool has_range;
        float range_m;
        uint32_t range_confidence; // 0..100
        uint64_t range_timestamp_us;

        // Environment (optional)
        bool has_pressure;
        float pressure_pa;
        bool has_temperature_c;
        float temperature_c;
        bool has_humidity;
        float humidity_fraction;

        // Latitude (preferred single source supplied by DOPE-ASS)
        bool has_latitude;
        float latitude_deg;
        uint8_t latitude_source; // 0=unset,1=GNSS,2=manual,3=magnetometer_est

        // Optional calibration offsets supplied by DOPE-ASS
        bool has_imu_bias;
        float accel_bias[3];
        float gyro_bias[3];
};
```

Notes:
- DOPE consumes the `SensorFrame` structure and performs deterministic ballistic
    computation. All raw-sensor handling (UART framing, NMEA parsing, IMU/Mag
    fusion such as Mahony/Madgwick, magnetometer dip inversion, LRF protocol
    parsing, and disturbance detection) MUST be implemented in DOPE-ASS and
    provided to DOPE via the fields above.


---

## 7.1 IMU (ISM330DHCX)

- 3-axis accelerometer
    
- 3-axis gyroscope
    
- Mahony/Madgwick fusion
    
- Outputs quaternion
    
- Computes pitch, roll, yaw
    
- Detects dynamic vs static state
    
- Supports bias calibration
    

---

## 7.2 Magnetometer (RM3100)

- 3-axis magnetic field
    
- Hard iron + soft iron calibration
    
- Declination correction
    
- Disturbance detection
    
- Optional suppression if disturbed
    
- **Magnetic dip angle (inclination) measurement**
    
- **Autonomous latitude estimation from dip angle** (see §7.2.1; lowest-priority source after GPS/GNSS and manual)
    

### 7.2.1 Latitude Estimation via Magnetic Dip

The magnetic dip angle (inclination) ψ is related to geographic latitude φ by the dipole approximation:

```
tan(ψ) ≈ 2 × tan(φ)
```

DOPE computes the dip angle from the calibrated magnetometer vector (ratio of vertical to horizontal field components) and inverts this relation to produce an estimated latitude:

```
φ_estimated = atan(tan(ψ) / 2)
```

This estimate is:

- Autonomous — requires no GPS or manual entry
    
- Sufficient for Coriolis/Eötvös corrections at engagement ranges ≤ 2500 m
    
- Subject to ±1–5° typical error from crustal magnetic anomalies and non-dipole field components; worst-case ±10° in high-anomaly regions
    

Accuracy is adequate because Coriolis corrections are small at typical engagement ranges; a 10° latitude error produces < 0.1 MOA Coriolis error at 1000 m.

**Priority rule:** The magnetometer-derived latitude is used only when neither a GPS/GNSS feed nor a manually entered latitude is available. See priority hierarchy in §11.4.

**Manual entry as calibration:** When the user enters a known latitude manually at a known position, that value serves simultaneously as the live latitude input **and** as a reference calibration point for the magnetometer estimator. This allows the dip-angle model to be locally anchored, improving estimation accuracy in subsequent sessions at or near the same location.

If the magnetometer is disturbed (disturbance flag set), the latitude estimate is suppressed. The same disturbance gate that prevents heading from being updated also invalidates the dip-derived latitude.
    

---

## 7.3 Barometer (BMP581)

- Absolute pressure (Pa)
    
- Temperature (°C)
    
- Humidity optional
    

Computes:

```
ρ = P / (R_specific × T_kelvin)
```

Supports:

```cpp
void DOPE_CalibrateBaro(void);
```

---

## 7.4 Laser Rangefinder (JRT D09C)

- Continuous UART ingestion
    
- 1–16 Hz
    
- Range (meters)
    
- Timestamp
    
- Confidence
    

Engine behavior:

- Stores latest valid range
    
- Associates quaternion snapshot at receipt
    
- Flags stale if timestamp exceeded
    
- No RANGING mode
    

---

## 7.5 GPS / GNSS Receiver (optional)

Latitude may be fed directly from an external GPS/GNSS receiver by the Application layer. DOPE does not ingest raw NMEA or satellite data; the Application provides a single resolved latitude value via `DOPE_Update` or `DOPE_SetDefaultOverrides`.

When a GPS/GNSS latitude is available it takes the highest priority in the latitude hierarchy (see §11.4). A manually entered latitude should be used to verify/calibrate GPS accuracy at a known benchmark point.

---

## 7.6 Zoom Encoder

- Accepts focal length in mm
    
- Computes FOV
    
- Converts optional optical flow to angular velocity
    

---

# 8. Manual Inputs

Persist until changed.

|Parameter|Description|
|---|---|
|BC|Ballistic coefficient|
|Drag model|G1–G8|
|Muzzle velocity|m/s|
|Bullet mass|grains|
|Bullet length|mm|
|Caliber|inches|
|Twist rate|signed inches/turn|
|Zero range|m|
|Sight height|mm|
|Wind speed|m/s|
|Wind heading|deg true|
|Latitude|deg (see §11.4 — GPS/GNSS feed, manual entry, or magnetometer estimation)|
|Altitude override|m|
|Humidity|fraction|
|Barrel material|CMV, 416 stainless, carbon-wrapped (sets stiffness/thermal properties)|
|Barrel toggles|Free-float, suppressor, barrel tuner|
|Shot events|`DOPE_NotifyShotFired(timestamp_us, ambient_temp_c)` to drive barrel heat/stringing|

---

# 9. Zeroing Logic

Zero angle is automatically recomputed whenever:

- BC changes
    
- MV changes
    
- Drag model changes
    
- Zero range changes
    
- Sight height changes
    
- Atmospheric correction changes
    

Manual zero recompute API removed.

---

# 10. Calibration APIs

```cpp
void DOPE_SetIMUBias(const float accel_bias[3], const float gyro_bias[3]);
void DOPE_SetMagCalibration(const float hard_iron[3], const float soft_iron[9]);
void DOPE_SetBoresightOffset(const BoresightOffset* offset);
void DOPE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa);
void DOPE_CalibrateBaro(void);
void DOPE_CalibrateGyro(void);
```

These correct physical alignment errors.

---

# 11. Ballistic Solver

## 11.1 Model

- Point-mass trajectory
    
- Piecewise power-law drag
    
- Adaptive timestep integration
    
- 1m resolution trajectory table
    

---

## 11.2 Atmospheric Correction

Uses reference 4-factor model:

```
BC_corrected = BC × FA × (1 + FT - FP) × FR
```

Imperial conversions applied internally for compatibility.

---

## 11.3 Wind

Manual only (v1.3).

Decomposition:

```
headwind  = wind_speed × cos(angle)
crosswind = wind_speed × sin(angle)
```

---

## 11.4 Coriolis and Eötvös

Latitude required. Sources in priority order:

1. **GPS / GNSS** — latitude fed directly by the Application from an attached receiver. Highest accuracy; preempts all other sources.
2. **Manual entry** — latitude entered explicitly by the user via the Application UI or API. Overrides magnetometer estimation. When entered at a known geographic position, also serves as a reference calibration point for the dip-angle estimator.
3. **Magnetometer-derived** — latitude estimated from magnetic dip angle (see §7.2.1). Active only when no GPS feed and no manual entry are present, and the magnetometer is undisturbed.
4. **Unset** — none of the above available; Coriolis disabled.

If latitude is unset:

- Coriolis disabled
    
- Diagnostic bit set in `defaults_active`
    
- No FAULT
    

Earth rotation:

```
ω = 7.2921e-5 rad/s
```

Integrated each timestep.

The `FiringSolution` shall indicate the latitude source via a flag in `defaults_active` so the application layer can display an appropriate confidence indicator.

---

## 11.5 Cant

Computes POI shift due to roll angle.

Included in final hold values.

---

## 11.6 Spin Drift

Litz approximation:

```
drift ∝ TOF^1.83
```

Signed by twist direction.

---

# 12. FiringSolution Output

```cpp
struct FiringSolution {
    uint32_t solution_mode;
    uint32_t fault_flags;
    uint32_t defaults_active;

    float hold_elevation_moa;
    float hold_windage_moa;

    float range_m;
    float horizontal_range_m;
    float tof_ms;
    float velocity_at_target_ms;
    float energy_at_target_j;

    float coriolis_windage_moa;
    float coriolis_elevation_moa;
    float spin_drift_moa;
    float wind_only_windage_moa;
    float earth_spin_windage_moa;
    float offsets_windage_moa;
    float cant_windage_moa;

    float cant_angle_deg;
    float heading_deg_true;

    float air_density_kgm3;
};
```

No rendering instructions included.

---

# 13. Fault Philosophy

Hard FAULT only for:

- No valid range
    
- No bullet profile
    
- Missing MV
    
- Missing BC
    
- Zero unsolvable
    
- AHRS unstable
    
- Severe sensor invalid
    

Everything else uses defaults and sets diagnostic flags.

---

# 14. Performance Requirements

On ESP32-P4:

- 1000m solution < 8ms
    
- 2500m solution < 15ms
    
- AHRS update < 1ms
    
- Zero dynamic allocations after init
    
- Static buffers only
    

---

# 15. Hardware Reference (Prototype)

Reference build:

- ESP32-P4
    
- IMX477 camera
    
- Arducam 8–50mm lens
    
- JRT D09C LRF
    
- ISM330DHCX IMU
    
- RM3100 magnetometer
    
- BMP581 barometer
    
- 390x390 AMOLED
    
- I2C rotary encoder
    

DOPE remains hardware-agnostic.

---

# 16. Out of Scope

- Full 6DOF modeling
    
- Variable wind along path
    
- AI target detection
    
- Point cloud processing
    
- UI rendering
    
- Wireless communication
    
- Data logging
    

---

# 17. Summary

DOPE v1.3 is:

- Deterministic
    
- Physically consistent
    
- Expandable
    
- Cleanly layered
    
- Optimized for ESP32-P4
    
- Scalable to stronger sensors
    

It computes ballistic solutions only.

All intelligence above trajectory math lives outside it.

---

---

# 18. Latitude Source Architecture — Design Rationale

Coriolis and Eötvös corrections require latitude. DOPE supports three independent latitude sources with a well-defined priority hierarchy:

**GPS / GNSS (highest priority):** The Application layer may attach any GNSS receiver and pass resolved latitude through the existing scalar input interface (`DOPE_Update` / `DOPE_SetDefaultOverrides`). DOPE is hardware-agnostic — it does not ingest NMEA or satellite data directly. GPS provides the most accurate latitude (<0.001° typical) and is the preferred source when hardware permits.

**Manual entry (intermediate priority):** The user enters a known latitude at setup time. This is the intended primary workflow when GPS hardware is absent. Manual entry also serves a dual role: it **calibrates the magnetometer dip-angle estimator** by providing a ground-truth value at a known position, improving the accuracy of autonomous estimation in subsequent use at or near the same location.

**Magnetometer-derived (lowest autonomous source):** The RM3100 magnetometer, already present for heading, measures the magnetic dip angle ψ. The dipole approximation `tan(ψ) ≈ 2 × tan(φ)` inverts to yield an estimated latitude with ±1–5° typical accuracy (worst-case ±10° in high-anomaly regions). At engagement ranges ≤ 2500 m, a 10° latitude error produces < 0.1 MOA Coriolis error — inside the system uncertainty budget.

The RM3100 latitude uncertainty budget is **±1.5° flat** for uncertainty propagation purposes (conservative value covering typical mid-latitude deployments).

**Disabled:** If none of the above sources are available (no GPS, no manual entry, magnetometer disturbed or absent), Coriolis is silently disabled and a diagnostic bit is set in `defaults_active`. No FAULT is raised.

**Interaction with disturbance detection:** The magnetometer disturbance flag that gates heading output also gates the dip-derived latitude. A disturbed magnetometer contributes neither a valid heading nor a valid latitude estimate.

**GUI sensor presets:** The GUI exposes two separate hardware preset selectors for latitude uncertainty propagation:
- *GPS / GNSS Module* — framework ready; no presets yet (sources to be added as receivers are qualified).
- *Magnetometer (lat est.)* — PNI RM3100 at ±1.5° flat.

Selecting a hardware preset locks the manual σ field, identical to the LRF and thermometer preset patterns.

---

End of Document  
DOPE SRS v2.0

---

# 19. Runtime Solver Policy (Table‑First, Low‑Latency)

DOPE shall prefer manufacturer- or calibration-provided trajectory tables at runtime ("table‑first") and use lightweight augmentation for environmental and barrel-state differences. The adaptive RK4 integrator is retained as a deterministic fallback and as an offline/verification tool, but is not required on the real-time fast path when the following conditions are met:

- A valid `AmmoDatasetV2` (see §20) is active and contains either a `cached_full_trajectory` or dense per-range table covering the requested range.
- The dataset provides required channels for the engagement: drop, velocity (or energy), and wind drift (or a means to compute it from a supplied baseline). If any required channel is missing, the solver fallback may be used when `module_caps_.enable_solver_fallback` is set and `hasUsableSolverInputs()` is true.

Runtime behavior:

- Primary path: lookup table → apply scalars (density/MV/barrel-heat) → apply calibration residuals → return solution.
- Fallback path: when table data insufficient and solver fallback enabled → run `BallisticSolver` to compute missing values and (optionally) fill runtime cache.
- Uncertainty refinement may be deferred off the critical path (`defer_uncertainty_`) and executed asynchronously or at a lower cadence.

Performance targets (see §14) assume table-first lookups; solver fallback is budgeted per-target (1000 m < 8 ms on ESP32‑P4). Removing the RK4 fallback on lower-powered hardware (regular ESP32) is acceptable only if `AmmoDatasetV2` coverage is guaranteed for all operational ranges and corrections.

# 20. AmmoDatasetV2 Schema and Precompute Requirements

DOPE accepts `AmmoDatasetV2` as the preferred runtime ammo representation. Each dataset shall include explicit baseline metadata and optional dense/cached tables to enable table-first operation.

Required metadata (at minimum):

- `baseline_temperature_c`, `baseline_pressure_pa`, `baseline_humidity` — baseline environment used by the table.
- `baseline_convention` — one of `METRO`, `ICAO`, or `CUSTOM` to indicate the convention used by the dataset.

Recommended channels (provided by manufacturer or generated during import):

- `drop_by_range` (drop_m) — per-range drop values, or sparse points with interpolation.
- `velocity_by_range` or `energy_by_range` — used for TOF/energy; if absent, solver fallback or radar assist supplies velocity.
- `wind_drift_by_range` — baseline wind drift at a reference wind; runtime scales applied per actual wind.
- `cep50_by_range` and/or `uncertainty_sigma_by_range` — empirical uncertainty overrides per-range (preferred when available).

Precompute requirement:

- Implementations shall offer an import-time precompute that generates a cached trajectory covering the operational range (preferred 1 m step). The cache MAY be stored externally to avoid inflating in-memory static structures. If a cached table is provided, DOPE shall use it directly on the fast path.

Caching rules:

- If `cached_full_table_present` is true and `cached_source_checksum` matches, use cached values as authoritative and only apply scalars/residuals at runtime.
- If the dataset contains only sparse points, the importer must either (a) generate a dense cached table before first use (preferred), or (b) mark the dataset as sparse and allow solver fallback when queries fall in unsupported ranges.

# 21. Calibration Procedure & Residuals

Calibration samples are records of observed impact relative to the aim point at a known range and environmental state. A `calibration_sample` shall include: range, measured impact offsets (elevation and windage, MOA), timestamp, ambient temperature, barometric pressure, humidity, measurement method (manual/camera/radar), and optional radar MV.

Residuals are the per-range additive differences between measured mean impact (MOA) and the dataset-predicted impact (MOA). DOPE shall store residuals as a small per-range table (`drop_residual_by_range`, `wind_residual_by_range`) and apply them additively to the selected table-family at runtime.

Minimum protocol (recommended):

- Capture N ≥ 3 shots at each canonical calibration range (e.g., 100 m, 300 m, 600 m, 1000 m). Record mean and standard deviation per point.
- Record ambient barometer/temperature/humidity at shot time to anchor baseline correction.
- Compute and store residuals indexed by range; optionally fit a low-order spline to provide smooth interpolation.

Application shall expose a simple UI to supply measured hits (manual entry) and trigger residual recomputation.

# 22. Uncertainty Policy

Primary representation: 2D Gaussian (σ_elev, σ_wind, ρ). The engine will compute/propagate covariance internally for analytic propagation and expose both covariance and a circularized `uncertainty_radius_moa` (1‑sigma radial) for quick consumers.

Empirical overrides: when datasets provide `cep50_by_range` or `uncertainty_sigma_by_range`, DOPE shall prefer these empirical values for uncertainty reporting at the corresponding ranges and fall back to propagated covariance otherwise. CEP50 conversion uses the canonical constant (~1.17741).

Non-normality: engines that detect strong non-Gaussian residuals (sufficient sample size) may optionally store percentile tables per-range; clients may use those for visualization. The default engine contract remains Gaussian-based for simplicity and speed.

# 23. Loader & Runtime Scalars

At runtime DOPE shall apply the following inexpensive scalar adjustments to table values prior to returning a solution:

- `density_scale = runtime_density / baseline_density` — scales aerodynamic drop/drag contributions.
- `mv_scale` — scalar from radar or calibration applied to velocity/TOF/energy.
- `barrel_heat_multiplier` — multiplicative MV modifier derived from shot history and ambient temperature.

These scalars are O(1) arithmetic operations and are applied to the cached table entries or sparse interpolation outputs; they do not require re-integration.

# 24. Removing RK4 from the Real‑Time Path

Removing the RK4 integrator from the on-device real-time path is permissible under the following guarantees:

- All operational ranges and corrections required by the application are present in `AmmoDatasetV2` (or cached table) including drop, wind drift, and velocity/energy.
- A conservative uncertainty envelope is available for extrapolated ranges and outside validated data.
- A verification path exists (off-device or developer device) to re-run RK4 for new datasets and to generate full cached tables.

If these guarantees cannot be met, the RK4 fallback must remain available for correctness.

---

# 25. SRS Completion Checklist

This checklist helps track progress toward implementing and verifying the requirements in this SRS. Use the task boxes to indicate status (unchecked = pending, checked = complete). The application or project lead should update this file as work progresses.

- [ ] **DOPE-ASS Boundary Enforcement:** Implement `SensorFrame` builder in DOPE-ASS.
- [ ] **DOPE-ASS Boundary Enforcement:** Move fusion algorithms (AHRS Mahony/Madgwick) to DOPE-ASS code.
- [ ] **DOPE-ASS Boundary Enforcement:** Move raw sensor calibration flows (e.g., Mag hard/soft iron) into DOPE-ASS code.
- [ ] **DOPE-ASS Boundary Enforcement:** Replace direct driver references in `lib/dope/` with setter APIs and `DOPE_Update` calls.
- [ ] **GUI Refactor:** Review UI controls that expose hardware/protocol settings (LRF UART controls, GNSS options, magnetometer calibration flows).
- [ ] **GUI Refactor:** Remove driver/protocol controls from the GUI and replace them with "Sanitized Input" configuration panels that accept preprocessed values.
- [ ] **GUI Refactor:** Update hardware preset dialogs to reflect that presets are DOPE-ASS UI conveniences and not engine APIs.
- [ ] **Integration Testing:** Add tests to verify the GUI produces correct `SensorFrame` shapes expected by DOPE (`DOPE_Update`).

Notes:
- Mark items as complete here after code, tests, and documentation satisfy the requirement.
- Where applicable, link to implementation files, tests, or issues for traceability.


---