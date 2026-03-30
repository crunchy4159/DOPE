# DOPE Precompute Tool

The DOPE precompute tool takes cartridge definitions from the human-editable `dope_gui_cartridges_v2.json`, reconstructs time of flight (if absent), performs spline fits, generates trajectories using the DOPE C++ Engine via a headless Native-binary, and evaluates the change in trajectory vs changes in ballistic coefficients (`d_drop_dbc`).

## Setup

1. Install Python 3.9+
2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Make sure you have PlatformIO installed globally and accessible on your path, as the precompute script compiles and invokes the internal simulation headless application.

## Workflow

1. Edit your cartridge definitions in `tools/native_gui/dope_gui_cartridges_v2.json`. You can now provide `tof_profile` entries. Note: `tools/native_gui/dope_gui_cartridges.json` (the older V1 version) has been removed.
2. If you have any new experimental external calibration conditions containing chronograph readings downrange across environments, save them in the precompute tool pipeline (Feature currently in progress for full struct integration).
3. From the command line, run the script against the target datasets:
   ```bash
   python dope_precompute.py ../native_gui/dope_gui_cartridges_v2.json --output-dir ./output
   ```
   **Arguments:**
   - `--grid-yards`: Custom table spacing interval (default 25yds).
   - `--max-range-yards`: The maximum precomputed range (default 1000yds). Capped at 128 elements.
   - `--output-dir`: Where to place `.dope` binaries and JSON readouts.

## Output

For each preset:
- `{Slug}.dope`: A precomputed binary memory block representing a complete `AmmoDatasetV2` struct. Load this onto your device SD card to skip JSON parsing latency.
- `{Slug}_precomputed.json`: The equivalent representation in raw JSON format to double-check output values before flashing to the device.

## Atmospheric Drift Engine (New v2 pipeline)

By pre-calculating the shift in drop per BC factor, DOPE can evaluate live atmospheric deltas (changes in temp/pressure compared to the zero-range environment) instantly, bypassing heavy iterative processing.
