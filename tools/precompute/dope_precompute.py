import json
import argparse
import os
import struct
import zlib
import subprocess
import math
import sys
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.integrate import quad

# Constants
DOPE_MAX_TABLE_POINTS = 128
DOPE_MAX_BARREL_MV_POINTS = 16
DOPE_MAX_TRAJECTORY_FAMILIES = 8
DOPE_MAX_CALIBRATION_POINTS = 32

FPS_TO_MS = 0.3048
MS_TO_FPS = 1.0 / FPS_TO_MS
YARDS_TO_M = 0.9144
M_TO_YARDS = 1.0 / YARDS_TO_M
INCHES_TO_M = 0.0254
GRAINS_TO_KG = 0.00006479891
GRAINS_PER_POUND = 7000.0


def _as_float(value, default=0.0):
    try:
        f = float(value)
        if math.isfinite(f):
            return f
    except Exception:
        pass
    return float(default)


def _normalize_humidity_fraction(raw_h):
    h = _as_float(raw_h, 0.0)
    # Accept either fraction [0..1] or percent [0..100].
    if h > 1.0:
        h *= 0.01
    return float(np.clip(h, 0.0, 1.0))


def _extract_xy(points, x_key, y_key):
    if not points:
        return np.array([]), np.array([])
    rows = []
    for pt in points:
        x = _as_float(pt.get(x_key), float("nan"))
        y = _as_float(pt.get(y_key), float("nan"))
        if math.isfinite(x) and math.isfinite(y):
            rows.append((x, y))
    if not rows:
        return np.array([]), np.array([])
    rows.sort(key=lambda t: t[0])
    # De-dup x by keeping the last value.
    dedup = {}
    for x, y in rows:
        dedup[x] = y
    xs = np.array(sorted(dedup.keys()), dtype=float)
    ys = np.array([dedup[x] for x in xs], dtype=float)
    return xs, ys


def _integrate_tof_from_velocity_and_arc(out_ranges_m, out_vels, out_drops=None):
    n = len(out_ranges_m)
    out_tofs = np.zeros(n, dtype=float)
    if n < 2:
        return out_tofs

    # Protect against zero/negative velocity.
    vel = np.maximum(np.asarray(out_vels, dtype=float), 1.0)
    dx = np.diff(np.asarray(out_ranges_m, dtype=float))

    if out_drops is not None:
        drops = np.asarray(out_drops, dtype=float)
        ddx = np.diff(drops) / np.maximum(dx, 1e-6)
        ds = np.sqrt(1.0 + ddx * ddx) * dx
    else:
        ds = dx

    v_mid = 0.5 * (vel[1:] + vel[:-1])
    dt = ds / np.maximum(v_mid, 1.0)
    out_tofs[1:] = np.cumsum(dt)
    return out_tofs


def _integrate_tof_high_accuracy(out_ranges_m, out_vels, out_drops=None):
    ranges = np.asarray(out_ranges_m, dtype=float)
    vel = np.maximum(np.asarray(out_vels, dtype=float), 1.0)
    n = len(ranges)
    out_tofs = np.zeros(n, dtype=float)
    if n < 2:
        return out_tofs

    vel_interp = PchipInterpolator(ranges, vel, extrapolate=True)
    drop_deriv = None
    if out_drops is not None:
        drops = np.asarray(out_drops, dtype=float)
        if len(drops) == n:
            drop_interp = PchipInterpolator(ranges, drops, extrapolate=True)
            drop_deriv = drop_interp.derivative()

    for i in range(1, n):
        r = float(ranges[i])
        if drop_deriv is None:
            integrand = lambda x: 1.0 / max(float(vel_interp(x)), 1.0)
        else:
            integrand = lambda x: math.sqrt(1.0 + float(drop_deriv(x)) ** 2) / max(float(vel_interp(x)), 1.0)
        val, _ = quad(integrand, float(ranges[0]), r, epsabs=1e-10, epsrel=1e-8, limit=500)
        out_tofs[i] = max(val, 0.0)
    return out_tofs


def _compute_sectional_density_lb_in2(mass_grains, caliber_inches):
    m_gr = _as_float(mass_grains, 0.0)
    cal_in = _as_float(caliber_inches, 0.0)
    if m_gr <= 0.0 or cal_in <= 0.0:
        return 0.0
    weight_lb = m_gr / GRAINS_PER_POUND
    return weight_lb / (cal_in * cal_in)


def _evaluate_curve(x, y, xq, enforce_nonincreasing=False, enforce_nondecreasing=False):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    xq = np.asarray(xq, dtype=float)
    if len(x) == 0:
        return np.zeros_like(xq)
    if len(x) == 1:
        out = np.full_like(xq, y[0], dtype=float)
    else:
        if len(x) >= 3:
            interp = PchipInterpolator(x, y, extrapolate=False)
            out = interp(xq)
        else:
            out = np.interp(xq, x, y)

        # Linear extension outside bounds using end slopes (safer than spline extrapolation).
        left_mask = xq < x[0]
        right_mask = xq > x[-1]
        left_slope = (y[1] - y[0]) / max(x[1] - x[0], 1e-6)
        right_slope = (y[-1] - y[-2]) / max(x[-1] - x[-2], 1e-6)
        out[left_mask] = y[0] + left_slope * (xq[left_mask] - x[0])
        out[right_mask] = y[-1] + right_slope * (xq[right_mask] - x[-1])

        # Fill NaNs from interpolation boundary behavior.
        nan_mask = ~np.isfinite(out)
        if np.any(nan_mask):
            out[nan_mask] = np.interp(xq[nan_mask], x, y)

    if enforce_nonincreasing:
        for i in range(1, len(out)):
            if out[i] > out[i - 1]:
                out[i] = out[i - 1]
    if enforce_nondecreasing:
        for i in range(1, len(out)):
            if out[i] < out[i - 1]:
                out[i] = out[i - 1]
    return out

def pack_profile_point(range_m, value):
    return struct.pack('<ff', float(range_m), float(value))

def pack_trajectory_point(range_m, drop_m):
    return struct.pack('<ff', float(range_m), float(drop_m))

def pack_uncertainty_sigma_point(range_m, sigma_elev_moa, sigma_wind_moa, rho):
    return struct.pack('<ffff', float(range_m), float(sigma_elev_moa), float(sigma_wind_moa), float(rho))

def run_headless_solver(bc, drag_model, mv, mass_kg, length_m, cal_m, temp_c, press_pa, hum, max_range_m, crosswind_ms=0.0):
    # This expects platformio to be built: pio run -e native
    # We execute the compiled binary: .pio/build/native/program.exe
    exe_path = os.path.join(os.path.dirname(__file__), '..', '..', '.pio', 'build', 'native_engine', 'program.exe')
    
    # ensure it exists
    if not os.path.exists(exe_path):
        print("Compiling native headless engine...")
        build_cwd = os.path.join(os.path.dirname(__file__), '..', '..')
        try:
            subprocess.run(["pio", "run", "-e", "native_engine"], cwd=build_cwd, check=True, capture_output=True)
        except FileNotFoundError:
            subprocess.run([sys.executable, "-m", "platformio", "run", "-e", "native_engine"],
                           cwd=build_cwd, check=True, capture_output=True)

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
    
    mass_grains = _as_float(cart.get('mass_grains', 150.0), 150.0)
    caliber_inches = _as_float(cart.get('caliber_inches', 0.3), 0.3)
    mv_ms = _as_float(cart.get('muzzle_velocity_ms', 0.0), 0.0)
    vp = cart.get('velocity_profile', [])
    vp_ranges, vp_vels = _extract_xy(vp, 'distance_m', 'velocity_ms')
    if mv_ms <= 0.0:
        if len(vp_vels) > 0:
            mv_ms = float(vp_vels[0])
        else:
            mv_ms = 800.0

    mass_kg = mass_grains * GRAINS_TO_KG
    len_m = _as_float(cart.get('length_inches', 1.0), 1.0) * INCHES_TO_M
    cal_m = caliber_inches * INCHES_TO_M

    # Extract trajectory_families directly
    tp_ranges = np.array([])
    tp_drops = np.array([])
    families = cart.get('trajectory_families', [])
    if len(families) > 0:
        tp = families[0].get('trajectory_profile', [])
        tp_ranges, tp_drops = _extract_xy(tp, 'distance_m', 'trajectory_m')

    # Default grid
    max_range_m = max_range_yards * YARDS_TO_M
    step_m = grid_yards * YARDS_TO_M
    
    out_ranges_m = np.arange(0, max_range_m + step_m/2, step_m)
    if len(out_ranges_m) > DOPE_MAX_TABLE_POINTS:
        out_ranges_m = out_ranges_m[:DOPE_MAX_TABLE_POINTS]
        
    out_count = len(out_ranges_m)

    # STAGE 2 & 3: Fit curves from available manufacturer data.
    if len(vp_ranges) >= 2:
        out_vels = _evaluate_curve(vp_ranges, vp_vels, out_ranges_m, enforce_nonincreasing=True)
    else:
        out_vels = np.full(out_count, mv_ms)
    out_vels = np.maximum(np.asarray(out_vels, dtype=float), 1.0)

    if len(tp_ranges) >= 2:
        out_drops = _evaluate_curve(tp_ranges, tp_drops, out_ranges_m)
    else:
        out_drops = None

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
    temp_c = (_as_float(env_j.get('temperature_f', 59.0), 59.0) - 32.0) * 5.0 / 9.0
    # Expect explicit Pascals in v2 data; default to standard pressure if absent
    press_pa = _as_float(env_j.get('pressure_pa', 101325.0), 101325.0)
    hum = _normalize_humidity_fraction(env_j.get('humidity_pct', 0.0))
    alt_m = _as_float(env_j.get('altitude_ft', 0.0), 0.0) * 0.3048

    print(" -> Solving Headless Solver Integrations...")
    solver_nom = None
    try:
        solver_nom = run_headless_solver(bc, dm_idx, mv_ms, mass_kg, len_m, cal_m, temp_c, press_pa, hum, out_ranges_m[-1])
        sol_plus = run_headless_solver(bc * 1.05, dm_idx, mv_ms, mass_kg, len_m, cal_m, temp_c, press_pa, hum, out_ranges_m[-1])
        sol_minus = run_headless_solver(bc * 0.95, dm_idx, mv_ms, mass_kg, len_m, cal_m, temp_c, press_pa, hum, out_ranges_m[-1])
        
        # solver returns regular spacing 1 meter
        sp_ranges = np.array(sol_plus['range'])
        sp_drops = np.array(sol_plus['drop'])
        sm_drops = np.array(sol_minus['drop'])
        
        # interpolate bounds back onto our grid
        d_plus = np.interp(out_ranges_m, sp_ranges, sp_drops)
        d_minus = np.interp(out_ranges_m, sp_ranges, sm_drops)
        
        out_d_dbc = (d_plus - d_minus) / max(abs(bc) * 0.10, 1e-6)

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

    # If manufacturer drop table is sparse/missing, use solver nominal drop.
    if out_drops is None:
        if solver_nom is not None and 'drop' in solver_nom and 'range' in solver_nom:
            out_drops = np.interp(out_ranges_m, np.array(solver_nom['range']), np.array(solver_nom['drop']))
        else:
            out_drops = np.zeros(out_count)

    # If manufacturer velocity table is sparse/missing, use solver nominal velocity.
    if len(vp_ranges) < 2 and solver_nom is not None and 'velocity' in solver_nom and 'range' in solver_nom:
        out_vels = np.interp(out_ranges_m, np.array(solver_nom['range']), np.array(solver_nom['velocity']))
        out_vels = np.maximum(np.asarray(out_vels, dtype=float), 1.0)

    # TOF profile: prefer vendor TOF, else integrate ds/v using available drop arc.
    tof_profile = cart.get('tof_profile', [])
    tof_r, tof_t = _extract_xy(tof_profile, 'distance_m', 'tof_s')
    if len(tof_r) >= 2:
        out_tofs = _evaluate_curve(tof_r, tof_t, out_ranges_m, enforce_nondecreasing=True)
    else:
        # Accuracy-first TOF reconstruction from the final velocity + arc.
        # This is intentionally slower than finite-difference approximations.
        out_tofs = _integrate_tof_high_accuracy(out_ranges_m, out_vels, out_drops)
    out_tofs = np.maximum(np.asarray(out_tofs, dtype=float), 0.0)

    # Energy profile from velocity (and mass) if no explicit manufacturer channel.
    out_energy_j = 0.5 * mass_kg * np.square(out_vels)

    # Optional CEP50 channel.
    cep_profile = cart.get('cep50_profile', [])
    if len(cep_profile) < 2:
        cep_profile = cart.get('cartridge_cep_profile', [])
    cep_r, cep_v = _extract_xy(cep_profile, 'distance_m', 'cep50_moa')
    if len(cep_r) >= 2:
        out_cep50 = _evaluate_curve(cep_r, cep_v, out_ranges_m, enforce_nondecreasing=True)
        num_cep50_points = out_count
    else:
        out_cep50 = np.zeros(out_count, dtype=float)
        num_cep50_points = 0

    # Optional precomputed 2D uncertainty sigma channel.
    # Accept explicit profile first, then derive isotropic sigma from CEP50 if present.
    unc_profile = cart.get('uncertainty_sigma_profile', [])
    unc_rows = []
    for pt in unc_profile:
        r = _as_float(pt.get('distance_m'), float("nan"))
        se = _as_float(pt.get('sigma_elev_moa', pt.get('sigma_moa', float("nan"))), float("nan"))
        sw = _as_float(pt.get('sigma_wind_moa', pt.get('sigma_moa', float("nan"))), float("nan"))
        rho = _as_float(pt.get('rho', 0.0), 0.0)
        if math.isfinite(r) and math.isfinite(se) and math.isfinite(sw):
            unc_rows.append((r, max(se, 0.0), max(sw, 0.0), float(np.clip(rho, -1.0, 1.0))))
    # De-dup by range (keep last value), then sort.
    unc_map = {}
    for r, se, sw, rho in unc_rows:
        unc_map[r] = (se, sw, rho)
    unc_rows = [(r, *unc_map[r]) for r in sorted(unc_map.keys())]

    out_sigma_e = np.zeros(out_count, dtype=float)
    out_sigma_w = np.zeros(out_count, dtype=float)
    out_sigma_rho = np.zeros(out_count, dtype=float)
    num_uncertainty_points = 0

    if len(unc_rows) >= 2:
        ur = np.array([r for r, _, _, _ in unc_rows], dtype=float)
        ue = np.array([e for _, e, _, _ in unc_rows], dtype=float)
        uw = np.array([w for _, _, w, _ in unc_rows], dtype=float)
        uc = np.array([c for _, _, _, c in unc_rows], dtype=float)
        out_sigma_e = _evaluate_curve(ur, ue, out_ranges_m)
        out_sigma_w = _evaluate_curve(ur, uw, out_ranges_m)
        out_sigma_rho = np.clip(_evaluate_curve(ur, uc, out_ranges_m), -1.0, 1.0)
        out_sigma_e = np.maximum(out_sigma_e, 0.0)
        out_sigma_w = np.maximum(out_sigma_w, 0.0)
        num_uncertainty_points = out_count
    elif len(unc_rows) == 1:
        _, se, sw, rho = unc_rows[0]
        out_sigma_e = np.full(out_count, max(se, 0.0), dtype=float)
        out_sigma_w = np.full(out_count, max(sw, 0.0), dtype=float)
        out_sigma_rho = np.full(out_count, float(np.clip(rho, -1.0, 1.0)), dtype=float)
        num_uncertainty_points = out_count
    elif num_cep50_points > 0:
        # Convert CEP50 radius to isotropic per-axis 1-sigma using Rayleigh relation.
        # CEP50 ~= 1.17741 * sigma  => sigma ~= CEP50 / 1.17741
        sigma_iso = np.maximum(out_cep50 / 1.1774100225, 0.0)
        out_sigma_e = sigma_iso
        out_sigma_w = sigma_iso
        out_sigma_rho = np.zeros(out_count, dtype=float)
        num_uncertainty_points = out_count

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

    # barrel_mv_by_length_in[DOPE_MAX_BARREL_MV_POINTS], num_barrel_mv_points
    barrel_mv_profile = cart.get('barrel_mv_profile', []) if isinstance(cart.get('barrel_mv_profile', []), list) else []
    barrel_pts = []
    for p in barrel_mv_profile:
        if isinstance(p, dict):
            bl = _as_float(p.get('barrel_in'), 0.0)
            mv_fps = _as_float(p.get('mv_fps'), 0.0)
            if bl > 0.0 and mv_fps > 0.0:
                barrel_pts.append((bl, mv_fps * FPS_TO_MPS))
    barrel_pts.sort(key=lambda x: x[0])
    # de-dup by barrel length, keep last
    dedup = {}
    for bl, mv_ms in barrel_pts:
        dedup[bl] = mv_ms
    barrel_pts = [(bl, dedup[bl]) for bl in sorted(dedup.keys())][:DOPE_MAX_BARREL_MV_POINTS]
    for i in range(DOPE_MAX_BARREL_MV_POINTS):
        if i < len(barrel_pts):
            bin_data += struct.pack('<ff', float(barrel_pts[i][0]), float(barrel_pts[i][1]))
        else:
            bin_data += struct.pack('<ff', 0.0, 0.0)
    bin_data += struct.pack('<i', len(barrel_pts))

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
    for r, e in zip(out_ranges_m, out_energy_j):
        pts += pack_profile_point(r, e)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<i', out_count)

    # cep50_by_range
    pts = bytearray()
    for r, c in zip(out_ranges_m, out_cep50):
        pts += pack_profile_point(r, c)
    pts += bytes(8 * (128 - out_count))
    bin_data += pts + struct.pack('<i', num_cep50_points)

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
    
    # uncertainty_sigma_by_range, num_uncertainty_points
    pts = bytearray()
    for r, se, sw, rho in zip(out_ranges_m, out_sigma_e, out_sigma_w, out_sigma_rho):
        pts += pack_uncertainty_sigma_point(r, se, sw, rho)
    pts += bytes(16 * (128 - out_count))
    bin_data += pts + struct.pack('<i', num_uncertainty_points)

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
    bin_data += struct.pack('<ffffff', mv_ms, mass_grains, len_m * 1000, cal_m / INCHES_TO_M, 10.0, 1.0)
    
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
    sd_vendor = None
    for k in ("sectional_density", "sectional_density_lb_in2", "sd"):
        if k in cart and cart.get(k) is not None:
            sd_vendor = _as_float(cart.get(k), 0.0)
            break
    sd_computed = _compute_sectional_density_lb_in2(mass_grains, caliber_inches)
    sd_rel_err_pct = 0.0
    if sd_vendor is not None and sd_vendor > 0.0 and sd_computed > 0.0:
        sd_rel_err_pct = 100.0 * abs(sd_vendor - sd_computed) / sd_computed
        if sd_rel_err_pct > 5.0:
            print(f" -> Note: sectional density mismatch {sd_rel_err_pct:.2f}% (vendor={sd_vendor:.4f}, computed={sd_computed:.4f})")

    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump({
            "meta": {
                "slug": slug,
                "grid_yards": grid_yards,
                "count": out_count,
                "sectional_density_lb_in2_computed": sd_computed,
                "sectional_density_lb_in2_vendor": sd_vendor,
                "sectional_density_rel_error_pct": sd_rel_err_pct,
                "barrel_mv_profile_points": len(barrel_pts)
            },
            "velocity": out_vels.tolist(),
            "drop": out_drops.tolist(),
            "tof": out_tofs.tolist(),
            "energy": out_energy_j.tolist(),
            "cep50_moa": out_cep50.tolist(),
            "uncertainty_sigma_elev_moa": out_sigma_e.tolist(),
            "uncertainty_sigma_wind_moa": out_sigma_w.tolist(),
            "uncertainty_rho": out_sigma_rho.tolist(),
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
