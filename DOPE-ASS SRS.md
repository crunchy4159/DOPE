# Software Requirements Specification

# DOPE — Digital Optical Performance Engine

## Version 1.6 — DRAFT

**Date:** 2026-02-27  
**Language Target:** C++17  
**Primary Platform:** ESP32-P4 @ 400MHz

---

# 1. Introduction

## 1.1 Purpose

This document defines the requirements for **DOPE** (Digital Optical Performance Engine), a C++ library targeting the ESP32-P4 microcontroller.

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

For this repository, the production DOPE engine scope is `lib/bce/`; desktop GUI harness code (`tools/native_gui/gui_main.cpp`, `tools/native_gui/imgui_*`), test suites (`test/`), helper scripts, and third-party dependencies are verification/integration tooling and are not normative engine logic.

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
│  - AHRS                       │
│  - Atmosphere                 │
│  - Drag integration           │
│  - Coriolis / Eötvös          │
│  - Spin drift                 │
│  - Cant correction            │
├───────────────────────────────┤
│       Sensor Drivers          │
│  - IMU                        │
│  - Magnetometer               │
│  - Barometer                  │
│  - LRF                        │
│  - Encoder                    │
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

If no overrides provided:

- Altitude: 0 m
    
- Pressure: 101325 Pa
    
- Temperature: 15 °C
    
- Humidity: 0.5
    
- Wind: 0 m/s
    
- Latitude: unset (GPS/GNSS used if fed by application; magnetometer-derived used if available; Coriolis disabled otherwise)
    

These values are ISA-consistent.

Defaults never trigger FAULT.

---

## 5.2 Default Override Mechanism

```cpp
struct BCE_DefaultOverrides {
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
void BCE_SetDefaultOverrides(const BCE_DefaultOverrides* defaults);
```

Application may load user profile defaults at startup.

---

# 6. Maximum Trajectory Range

Compile-time constant:

```cpp
constexpr uint32_t BCE_MAX_RANGE_M = 2500;
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
void BCE_Update(const SensorFrame* frame);
```

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
void BCE_CalibrateBaro(void);
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

Latitude may be fed directly from an external GPS/GNSS receiver by the Application layer. DOPE does not ingest raw NMEA or satellite data; the Application provides a single resolved latitude value via `BCE_Update` or `BCE_SetDefaultOverrides`.

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
void BCE_SetIMUBias(const float accel_bias[3], const float gyro_bias[3]);
void BCE_SetMagCalibration(const float hard_iron[3], const float soft_iron[9]);
void BCE_SetBoresightOffset(const BoresightOffset* offset);
void BCE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa);
void BCE_CalibrateBaro(void);
void BCE_CalibrateGyro(void);
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

**GPS / GNSS (highest priority):** The Application layer may attach any GNSS receiver and pass resolved latitude through the existing scalar input interface (`BCE_Update` / `BCE_SetDefaultOverrides`). DOPE is hardware-agnostic — it does not ingest NMEA or satellite data directly. GPS provides the most accurate latitude (<0.001° typical) and is the preferred source when hardware permits.

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
DOPE SRS v1.33