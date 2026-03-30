import json
import argparse
import os
import struct
import zlib
import subprocess
import numpy as np
from scipy.interpolate import PchipInterpolator, CubicSpline
from scipy.integrate import quad

# Constants
DOPE_MAX_TABLE_POINTS = 128
DOPE_MAX_TRAJECTORY_FAMILIES = 8
DOPE_MAX_CALIBRATION_POINTS = 32

FPS_TO_MS = 0.3048
MS_TO_FPS = 1.0 / FPS_TO_MS
YARDS_TO_M = 0.9144
M_TO_YARDS = 1.0 / YARDS_TO_M
INCHES_TO_M = 0.0254

def pack_profile_point(range_m, value):
    return struct.pack('<ff', float(range_m), float(value))

def pack_trajectory_point(range_m, drop_m):
    return struct.pack('<ff', float(range_m), float(drop_m))

def run_headless_solver(bc, drag_model, mv, mass_kg, length_m, cal_m, temp_c, press_pa, hum, max_range_m, crosswind_ms=0.0):
    # This expects platformio to be built: pio run -e native
    # We execute the compiled binary: .pio/build/native/program.exe
    exe_path = os.path.join(os.path.dirname(__file__), '..', '..', '.pio', 'build', 'native_engine', 'program.exe')
    
    # ensure it exists
    if not os.path.exists(exe_path):
        print("Compiling native headless engine...")
        subprocess.run(["pio", "run", "-e", "native_engine"], cwd=os.path.join(os.path.dirname(__file__), '..', '..'), check=True, capture_output=True)

    cmd = [
        exe_path, "--headless",
        "--bc", str(bc),
        "--drag_model", str(drag_model),
        "--mv", str(mv),
        "--mass", str(mass_kg),
        "--length", str(length_m),
        "--caliber", str(cal_m),
        "--temp", str(temp_c),
        "--pressure", str(press_pa),
        "--humidity", str(hum),
        "--range", str(max_range_m),
        "--crosswind", str(crosswind_ms)
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Solver failed: {res.stderr}")
    return json.loads(res.stdout)

def precompute_cartridge(cart, v2_idx, grid_yards, max_range_yards, output_dir, calib=None):
    slug = cart.get('name', 'Unknown').replace(" ", "_").replace(".", "").replace("/", "")
    if calib:
        slug = f"{calib.get('gun_id', 'Gun')}_{slug}_{calib.get('env_id', 'env')}"
    
    print(f"Precomputing {cart.get('name')} -> {slug}")

    # STAGE 1: Extract and Normalize
    dm_str = cart.get('preferred_drag_model', 'G1')
    bc = cart.get('ballistic_coefficients', {}).get(dm_str, 0.5)
    dm_idx = int(dm_str.replace('G','')) if 'G' in dm_str else 1
    
    mv_ms = cart.get('velocity_profile', [{'distance_m':0, 'velocity_ms': 800}])[0]['velocity_ms']
    mass_kg = cart.get('mass_grains', 150.0) * 0.00006479891
    len_m = cart.get('length_inches', 1.0) * 0.0254
    cal_m = cart.get('caliber_inches', 0.3) * 0.0254
    
    vp = cart.get('velocity_profile', [])
    vp_ranges = np.array([pt['distance_m'] for pt in vp])
    vp_vels = np.array([pt['velocity_ms'] for pt in vp])
    
    # Extract trajectory_families directly
    tp_ranges = np.array([])
    tp_drops = np.array([])
    families = cart.get('trajectory_families', [])
    if len(families) > 0:
        tp = families[0].get('trajectory_profile', [])
        tp_ranges = np.array([pt['distance_m'] for pt in tp])
        tp_drops = np.array([pt['trajectory_m'] for pt in tp])

    # Default grid
    max_range_m = max_range_yards * YARDS_TO_M
    step_m = grid_yards * YARDS_TO_M
    
    out_ranges_m = np.arange(0, max_range_m + step_m/2, step_m)
    if len(out_ranges_m) > DOPE_MAX_TABLE_POINTS:
        out_ranges_m = out_ranges_m[:DOPE_MAX_TABLE_POINTS]
        
    out_count = len(out_ranges_m)

    # STAGE 2 & 3: Spline Fit
    if len(vp_ranges) >= 2:
        vel_spline = PchipInterpolator(vp_ranges, vp_vels)
        out_vels = vel_spline(out_ranges_m)
    else:
        out_vels = np.full(out_count, mv_ms)
        
    tof_profile = cart.get('tof_profile', [])
    if len(tof_profile) >= 2:
        tof_r = np.array([pt['distance_m'] for pt in tof_profile])
        tof_t = np.array([pt['tof_s'] for pt in tof_profile])
        tof_spline = PchipInterpolator(tof_r, tof_t)
        out_tofs = tof_spline(out_ranges_m)
    elif len(vp_ranges) >= 2:
        # Integrated 1/v
        out_tofs = np.zeros(out_count)
        for i in range(1, out_count):
            val, _ = quad(lambda x: 1.0 / vel_spline(x), 0, out_ranges_m[i], limit=100)
            out_tofs[i] = val
    else:
        out_tofs = np.zeros(out_count)

    if len(tp_ranges) >= 2:
        drop_spline = CubicSpline(tp_ranges, tp_drops, bc_type='natural')
        out_drops = drop_spline(out_ranges_m)
    else:
        out_drops = np.zeros(out_count)

    wind_drift_profile = cart.get('wind_drift_profile', [])
    if len(wind_drift_profile) >= 2:
        wd_r = np.array([pt['distance_m'] for pt in wind_drift_profile])
        wd_d = np.array([pt['wind_drift_m'] for pt in wind_drift_profile])
        wd_spline = PchipInterpolator(wd_r, wd_d)
        out_wind_drifts = np.abs(wd_spline(out_ranges_m))
        num_wind_drift_points = out_count
    else:
        out_wind_drifts = None
        num_wind_drift_points = 0

    # STAGE 4: BC Sensitivity Matrix using Headless Engine
    env_j = cart.get('environmental_conditions', {})
    temp_c = (env_j.get('temperature_f', 59.0) - 32.0) * 5.0 / 9.0
    press_pa = env_j.get('pressure_inhg', 29.921) * 3386.39
    hum = env_j.get('humidity_pct', 0.0)
    alt_m = env_j.get('altitude_ft', 0.0) * 0.3048

    print(" -> Solving Headless Solver Integrations...")
    try:
        sol_plus = run_headless_solver(bc * 1.05, dm_idx, mv_ms, mass_kg, len_m, cal_m, temp_c, press_pa, hum, out_ranges_m[-1])
        sol_minus = run_headless_solver(bc * 0.95, dm_idx, mv_ms, mass_kg, len_m, cal_m, temp_c, press_pa, hum, out_ranges_m[-1])
        
        # solver returns regular spacing 1 meter
        sp_ranges = np.array(sol_plus['range'])
        sp_drops = np.array(sol_plus['drop'])
        sm_drops = np.array(sol_minus['drop'])
        
        # interpolate bounds back onto our grid
        d_plus = np.interp(out_ranges_m, sp_ranges, sp_drops)
        d_minus = np.interp(out_ranges_m, sp_ranges, sm_drops)
        
        out_d_dbc = (d_plus - d_minus) / (bc * 0.10)

        if out_wind_drifts is None:
            sol_nom = run_headless_solver(bc, dm_idx, mv_ms, mass_kg, len_m, cal_m, temp_c, press_pa, hum, out_ranges_m[-1], crosswind_ms=4.4704)
            nom_windage = np.array(sol_nom['windage'])
            out_wind_drifts = np.abs(np.interp(out_ranges_m, np.array(sol_nom['range']), nom_windage))
            num_wind_drift_points = out_count

    except Exception as e:
        print(f" -> Warning: Headless solver failed ({e}), optional arrays will be zeroed")
        out_d_dbc = np.zeros(out_count)
        if out_wind_drifts is None:
            out_wind_drifts = np.zeros(out_count)
            num_wind_drift_points = 0

    # STAGE 5: Calibration Backsolve skipped here (stubbed for future)
    bc_scale = 1.0
    
    # Pack Binary Struct AmmoDatasetV2
    # Ensure ABI match with dope_types.h (this is manual padding/packing)
    # STAGE 5: Binary Packing
    bin_data = bytearray()
    bin_data += struct.pack('<f', 1.0) # confidence
    bin_data += struct.pack('<ffff', float(temp_c), float(press_pa), float(hum), float(alt_m))
    
    # baseline_convention (uint8_t) + 3 bytes padding
    conv_map = {"ICAO": 2, "Metro": 1, "Custom": 3}
    conv_val = conv_map.get(env_j.get('standard', 'ICAO'), 2)
    bin_data += struct.pack('<Bxxx', conv_val) 
    
    # baseline_barrel_length_in, wind_speed_ms
    bin_data += struct.pack('<ff', float(cart.get('reference_barrel_inches', 24.0)), 4.4704) # 10 mph in m/s

    # trajectories (DOPE_MAX_TRAJECTORY_FAMILIES = 8)
    # struct DOPE_TrajectoryFamily: float zero_range_m, DOPE_TrajectoryPoint points[128], int num_points, bool cached_table_present, float cached_table_step_m, int cached_table_num_points, DOPE_TrajectoryPoint cached_table[128], uint64_t cached_checksum
    
    for i in range(8):
        if i == 0:
            bin_data += struct.pack('<f', 100 * YARDS_TO_M)
            pts = bytearray()
            for r, d in zip(out_ranges_m, out_drops):
                pts += pack_trajectory_point(r, d)
            pts += bytes(8 * (128 - out_count))
            bin_data += pts
            bin_data += struct.pack('<i', out_count)
            bin_data += struct.pack('<B', 1) # cached present
            bin_data += bytes(3) # padding
            bin_data += struct.pack('<fi', step_m, out_count)
            bin_data += pts # cache duplicate
            bin_data += struct.pack('<Q', 0)
        else:
            bin_data += bytes(4 + 8*128 + 4 + 4 + 4 + 4 + 8*128 + 8)

    # num_trajectories
    bin_data += struct.pack('<i', 1)

    # velocity_by_range
    pts = bytearray()
    for r, v in zip(out_ranges_m, out_vels):
        pts += pack_profile_point(r, v)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<i', out_count)

    # wind_drift_by_range
    pts = bytearray()
    for r, w in zip(out_ranges_m, out_wind_drifts):
        pts += pack_profile_point(r, w)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<i', num_wind_drift_points)

    # energy_by_range
    pts = bytearray()
    pts += bytes(8 * 128)
    bin_data += pts + struct.pack('<i', 0)

    # cep50_by_range
    pts = bytearray()
    pts += bytes(8 * 128)
    bin_data += pts + struct.pack('<i', 0)

    # tof_by_range
    pts = bytearray()
    for r, t in zip(out_ranges_m, out_tofs):
        pts += pack_profile_point(r, t)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<i', out_count)

    # d_drop_dbc_by_range
    pts = bytearray()
    for r, d in zip(out_ranges_m, out_d_dbc):
        pts += pack_profile_point(r, d)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<i', out_count)
    
    # uncertainty_sigma_by_range (128 * 16 bytes = 2048 bytes), num_uncertainty_points
    bin_data += bytes(16 * 128)
    bin_data += struct.pack('<i', 0)

    # cached_full_table fallback metadata
    bin_data += struct.pack('<B', 1) # cached_full_table_present
    bin_data += bytes(3) # pad
    bin_data += struct.pack('<fi', step_m, out_count)
    
    pts = bytearray()
    for r, d in zip(out_ranges_m, out_drops):
        pts += pack_trajectory_point(r, d)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<Q', 0)
    
    # bc, dm, dmmask
    bin_data += struct.pack('<fBB', bc, dm_idx, (1 << (dm_idx - 1)))
    bin_data += bytes(2) # pad
    
    # mv, mass, len, cal, twist, mv_adj
    bin_data += struct.pack('<ffffff', mv_ms, cart.get('mass_grains', 150), len_m * 1000, cal_m / INCHES_TO_M, 10.0, 1.0)
    
    # thermals
    bin_data += struct.pack('<ffff', 1.0, 0.0, 0.0, 0.0)
    
    # Output Headers
    header = bytearray(b'DOPE')
    header += struct.pack('<HH', 2, 0) # V2.0
    header += struct.pack('<I', 1)     # flags: AmmoDatasetV2 present
    header += struct.pack('<I', len(bin_data)) # Size
    header += struct.pack('<I', 0) # No calib
    
    crc = zlib.crc32(bin_data)
    header += struct.pack('<I', crc)
    header += bytes(8) # reserved
    
    full_bin = header + bin_data
    
    os.makedirs(output_dir, exist_ok=True)
    out_bin = os.path.join(output_dir, f"{slug}.dope")
    with open(out_bin, 'wb') as f:
        f.write(full_bin)
        
    out_json = os.path.join(output_dir, f"{slug}_precomputed.json")
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump({
            "meta": {"slug": slug, "grid_yards": grid_yards, "count": out_count},
            "velocity": out_vels.tolist(),
            "drop": out_drops.tolist(),
            "tof": out_tofs.tolist(),
            "d_drop_dbc": out_d_dbc.tolist(),
            "wind_drift": out_wind_drifts.tolist()
        }, f, indent=2)

if __name__ == '__main__':
    parser = argparse.ArgumentParser("DOPE Precompute Tool")
    parser.add_argument("cartridge_json", help="Path to dope_gui_cartridges_v2.json")
    parser.add_argument("--grid-yards", type=int, default=25, help="Grid spacing (yards)")
    parser.add_argument("--max-range-yards", type=int, default=1000, help="Max distance")
    parser.add_argument("--output-dir", default=".", help="Output directory")
    args = parser.parse_args()

    with open(args.cartridge_json, 'r') as f:
        data = json.load(f)

    for i, cart in enumerate(data.get('ammo_v2', [])):
        try:
            precompute_cartridge(cart, i, args.grid_yards, args.max_range_yards, args.output_dir)
        except Exception as e:
            print(f"Skipping {cart.get('name')}: {e}")
