import json
import shutil
from pathlib import Path

p = Path(__file__).resolve().parent / 'native_gui' / 'dope_gui_cartridges_v2.json'
if not p.exists():
    print('File not found:', p)
    raise SystemExit(1)

bak = p.with_suffix('.json.envbak')
shutil.copy2(p, bak)
print('Backup written to', bak)

data = json.loads(p.read_text(encoding='utf-8'))
if 'ammo_v2' not in data:
    print('No ammo_v2 key; nothing to do')
    raise SystemExit(0)

defaults = {
    "temperature_f": 59.0,
    "pressure_pa": 101325.0,
    "altitude_ft": 0.0,
    "humidity_pct": 0.0,
    "standard": "ICAO"
}

changed_entries = 0
ensured_profiles = 0
for entry in data['ammo_v2']:
    if 'environmental_conditions' not in entry or not isinstance(entry['environmental_conditions'], dict):
        entry['environmental_conditions'] = defaults.copy()
        changed_entries += 1
    # Ensure trajectory_families exists
    fams = entry.get('trajectory_families')
    if fams is None:
        # leave absent families as-is; but ensure key exists as empty list if there are no families
        entry['trajectory_families'] = []
    else:
        # For each family, ensure trajectory_profile key exists (as list)
        for fam in entry['trajectory_families']:
            if 'trajectory_profile' not in fam or not isinstance(fam['trajectory_profile'], list):
                fam['trajectory_profile'] = []
                ensured_profiles += 1

if changed_entries or ensured_profiles:
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
    print(f'Updated {changed_entries} entries with environmental_conditions; ensured {ensured_profiles} family profiles')
else:
    print('No changes needed')
