# Software Requirements Specification

# DOPE — Digital Optical Performance Engine

## Version 1.31 — DRAFT

**Date:** 2026-02-25  
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

For this repository, the production DOPE engine scope is `lib/bce/`; desktop GUI harness code (`src/gui_main.cpp`, `src/imgui_*`), test suites (`test/`), helper scripts, and third-party dependencies are verification/integration tooling and are not normative engine logic.

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
    
- Latitude: unset (Coriolis disabled)
    

These values are ISA-consistent.

Defaults never trigger FAULT.

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
void DOPE_SetDefaultOverrides(const DOPE_DefaultOverrides& defaults);
```

Application may load user profile defaults at startup.

---

# 6. Maximum Trajectory Range

Compile-time constant:

```cpp
#define DOPE_MAX_RANGE_M 2500
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
void DOPE_Update(const SensorFrame& frame);
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
void DOPE_CalibrateBaro();
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

## 7.5 Zoom Encoder

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
|Latitude|deg|
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
void DOPE_SetIMUBias(float accel_bias[3], float gyro_bias[3]);
void DOPE_SetMagCalibration(float hard_iron[3], float soft_iron[3]);
void DOPE_SetBoresightOffset(float vertical_moa, float horizontal_moa);
void DOPE_SetReticleMechanicalOffset(float vertical_moa, float horizontal_moa);
void DOPE_CalibrateBaro();
void DOPE_CalibrateGyro();
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

Latitude required.

If latitude unset:

- Coriolis disabled
    
- Diagnostic bit set
    
- No FAULT
    

Earth rotation:

```
ω = 7.2921e-5 rad/s
```

Integrated each timestep.

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

End of Document  
DOPE SRS v1.3