DOPE → DOPE-ASS Migration Checklist

Purpose

This document lists items removed from DOPE (the pure numeric engine) that must be implemented in DOPE-ASS (the application/host) following the SRS boundary enforcement.

Summary of items to move to DOPE-ASS

- AHRS sensor fusion algorithms (Mahony/Madgwick): implement fusion to produce sanitized orientation quaternion.
- Magnetometer calibration and disturbance detection: hard-iron/soft-iron calibration, local calibration anchoring, and disturbance gating.
- Magnetometer dip-angle inversion and autonomous latitude estimation: compute dip-derived latitude in DOPE-ASS if desired; provide `latitude_deg` and `latitude_source` to DOPE.
- LRF protocol handling and UART framing: continuous ingestion, parsing, timestamping, and confidence extraction for the JRT D09C or other LRFs.
- GNSS/GPS parsing and resolution: NMEA/receiver parsing and production of a single resolved latitude value.
- Encoder/zoom handling: focal-length conversion, FOV computation, and optical-flow → angular-velocity conversion.
- Calibration routines that operate on raw sensor data (e.g., `DOPE_CalibrateBaro`, `DOPE_CalibrateGyro`) — move to DOPE-ASS; DOPE retains setter APIs to accept calibration results (`DOPE_SetIMUBias`, `DOPE_SetMagCalibration`, etc.).
- Disturbance flags and gating logic that affect heading/latitude: compute in DOPE-ASS and pass boolean/flags to DOPE via `SensorFrame.flags`.
- Hardware/protocol UI controls and presets: GUI elements that configure drivers, UART params, or hardware presets belong to DOPE-ASS GUI.
- Any timing/throughput guarantees tied to sensor update loops (e.g., "AHRS update < 1ms") — enforce these in DOPE-ASS integrations rather than in the engine.

Suggested DOPE-ASS interfaces and responsibilities

- Provide a `SensorFrame` builder module that:
  - Ingests raw driver data and performs parsing.
  - Runs calibration and fusion algorithms.
  - Produces `SensorFrame` instances and handles timestamping, staleness, and flags.
- Provide a `CalibrationService` to run on-device calibration flows and export calibration results to DOPE via its setter APIs.
- Provide a `LRFAdapter` per supported rangefinder handling protocol conversions and confidence mapping.
- Provide a `GNSSAdapter` to resolve receiver outputs to a single latitude value and supply it to DOPE.
- Update GUI to present either raw-driver config panels (if drivers are present on that platform) or sanitized-input panels (if the application supplies preprocessed values to DOPE).

Integration checklist

- [ ] Implement `SensorFrame` builder in DOPE-ASS.
- [ ] Move fusion algorithms and calibration flows to DOPE-ASS code.
- [ ] Replace any direct driver references in `lib/dope/` with setter APIs and `DOPE_Update` calls.
- [ ] Update `tools/native_gui` to remove driver/protocol controls or mark them DOPE-ASS-only.
- [ ] Add integration tests verifying `SensorFrame` shapes consumed by DOPE.

Owner: DOPE-ASS integration team
Priority: High
