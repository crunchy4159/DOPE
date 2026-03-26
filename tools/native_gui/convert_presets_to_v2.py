#!/usr/bin/env python3
import json, re, os

ROOT = os.path.dirname(__file__)
IN_PATH = os.path.join(ROOT, 'dope_gui_cartridges.json')
GUNS_PATH = os.path.join(ROOT, 'dope_gui_guns.json')
OUT_CARTRIDGES_V2 = os.path.join(ROOT, 'dope_gui_cartridges_v2.json')
OUT_GUNS_SANITIZED = os.path.join(ROOT, 'dope_gui_guns_sanitized.json')
OUT_GUNS_V2 = os.path.join(ROOT, 'dope_gui_guns_v2.json')

def strip_c_comments(s: str) -> str:
    # remove /* ... */
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    # remove //...\n
    s = re.sub(r'//.*?\n', '\n', s)
    # remove trailing commas before } or ]
    s = re.sub(r',\s*(?=[}\]])', '', s)
    return s

def load_json_with_comments(path):
    with open(path, 'r', encoding='utf-8') as f:
        raw = f.read()
    cleaned = strip_c_comments(raw)
    return json.loads(cleaned)

FPS_TO_MPS = 0.3048
GRAINS_TO_KG = 0.00006479891
INCH_TO_M = 0.0254

def convert_cartridge_to_v2(preset, idx):
    v2 = {}
    v2['name'] = preset.get('name')
    v2['tags'] = preset.get('tags', [])
    v2['cartridge_keys'] = preset.get('cartridge_keys', [])
    # drag model: map index -> string (G1..G8) if available
    d_idx = preset.get('drag_model_index')
    if isinstance(d_idx, int):
        v2['drag_model'] = f'G{d_idx+1}'
    # ballistic coefficient
    if 'bc' in preset:
        v2['ballistic_coefficient'] = preset['bc']
        if 'sigma_bc' in preset:
            v2['sigma_bc'] = preset['sigma_bc']
    # mass, caliber, length converted to SI
    if 'mass_grains' in preset:
        v2['mass_kg'] = round(preset['mass_grains'] * GRAINS_TO_KG, 9)
    if 'caliber_inches' in preset:
        v2['caliber_m'] = round(preset['caliber_inches'] * INCH_TO_M, 6)
    if 'length_mm' in preset:
        v2['length_m'] = round(preset['length_mm'] / 1000.0, 6)
    # velocity_profile: distance_inches -> distance_m
    if 'velocity_profile' in preset and isinstance(preset['velocity_profile'], list):
        vp = []
        for e in preset['velocity_profile']:
            di = e.get('distance_inches')
            vm = e.get('velocity_ms')
            if di is None or vm is None:
                continue
            vp.append({'distance_m': round(di * INCH_TO_M, 3), 'velocity_ms': round(vm, 3)})
        v2['velocity_profile'] = vp
    # trajectory_profile: distance_inches -> distance_m, trajectory_inches -> trajectory_m
    if 'trajectory_profile' in preset and isinstance(preset['trajectory_profile'], list):
        tp = []
        for e in preset['trajectory_profile']:
            di = e.get('distance_inches')
            tr = e.get('trajectory_inches')
            if di is None or tr is None:
                continue
            tp.append({'distance_m': round(di * INCH_TO_M, 3), 'trajectory_m': round(tr * INCH_TO_M, 4)})
        v2['trajectory_profile'] = tp
    # uncertainty: sd_mv_fps -> sigma_muzzle_velocity_ms
    if 'sd_mv_fps' in preset:
        v2['sigma_muzzle_velocity_ms'] = round(preset['sd_mv_fps'] * FPS_TO_MPS, 3)

    # preserve other numeric metadata if present
    for k in ['reference_barrel_inches', 'mv_adjustment_fps_per_in', 'cep_table_moa', 'cep_scale_floor', '_source', 'tags']:
        if k in preset:
            v2[k] = preset[k]

    # include provenance back to original index so nothing is lost
    v2['source_preset_index'] = idx
    return v2

def sanitize_gun(gun):
    g = dict(gun)
    # normalize numeric precision per project guidance
    if 'caliber_inches' in g:
        g['caliber_inches'] = round(float(g['caliber_inches']), 3)
    if 'barrel_length_in' in g:
        g['barrel_length_in'] = round(float(g['barrel_length_in']), 1)
    if 'muzzle_diameter_in' in g:
        g['muzzle_diameter_in'] = round(float(g['muzzle_diameter_in']), 3)
    if 'twist_rate_inches' in g:
        g['twist_rate_inches'] = round(float(g['twist_rate_inches']), 2)
    if 'cold_bore_velocity_bias' in g:
        g['cold_bore_velocity_bias'] = round(float(g['cold_bore_velocity_bias']), 2)
    if 'angular_sigma_moa' in g:
        g['angular_sigma_moa'] = round(float(g['angular_sigma_moa']), 2)
    return g

def main():
    print('Loading originals...')
    car = load_json_with_comments(IN_PATH)
    guns = load_json_with_comments(GUNS_PATH)

    presets = car.get('cartridge_presets', [])
    v2_list = []
    for i, p in enumerate(presets):
        v2_list.append(convert_cartridge_to_v2(p, i))

    out_car = {'generated_from': os.path.basename(IN_PATH), 'count': len(v2_list), 'ammo_v2': v2_list}
    with open(OUT_CARTRIDGES_V2, 'w', encoding='utf-8') as f:
        json.dump(out_car, f, indent=2, ensure_ascii=False)

    # sanitize guns
    sanitized = []
    for g in guns:
        sanitized.append(sanitize_gun(g))
    with open(OUT_GUNS_SANITIZED, 'w', encoding='utf-8') as f:
        json.dump(sanitized, f, indent=2, ensure_ascii=False)
    # also emit a v2-named copy so GUI can prefer *_v2.json files without overwriting originals
    with open(OUT_GUNS_V2, 'w', encoding='utf-8') as f:
        json.dump(sanitized, f, indent=2, ensure_ascii=False)

    # quick validation: read back
    with open(OUT_CARTRIDGES_V2, 'r', encoding='utf-8') as f:
        _ = json.load(f)
    with open(OUT_GUNS_SANITIZED, 'r', encoding='utf-8') as f:
        _ = json.load(f)

    print('Wrote:', OUT_CARTRIDGES_V2)
    print('Wrote:', OUT_GUNS_SANITIZED)
    print('Wrote:', OUT_GUNS_V2)

if __name__ == '__main__':
    main()
