GUI TODO: Reflect DOPE/DOPE-ASS Boundary Changes

When the DOPE SRS boundary changes (e.g., moving sensor-processing responsibilities between DOPE and DOPE-ASS), update the native GUI so it remains aligned with DOPE-ASS responsibilities.

Action items:

- Review UI controls that expose hardware/protocol settings (LRF UART controls, GNSS options, magnetometer calibration flows).
- If DOPE prohibits sensor fusion or driver-level config, remove driver/protocol controls from the GUI and replace them with "Sanitized Input" configuration panels that accept preprocessed values (e.g., orientation quaternion, range_m, pressure_pa).
- Update hardware preset dialogs to reflect that presets are DOPE-ASS UI conveniences and not engine APIs.
- Add integration tests to verify the GUI produces `SensorFrame` shapes expected by DOPE (`DOPE_Update`).

Files likely to change:

- tools/native_gui/gui_main.cpp
- tools/native_gui/imgui_all.cpp
- tools/native_gui/dope_gui_preset.json

Owner: GUI maintainer
Priority: High when SRS boundary changes
 
Related migration checklist:

- See `DOPE_ASS_MIGRATION.md` in the repository root for a detailed list of
	items moved from DOPE into DOPE-ASS and suggested integration steps.
