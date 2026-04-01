#!/usr/bin/env python3
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
JSON_PATH = ROOT / 'dope_gui_cartridges_v2.json'

def main():
    if not JSON_PATH.exists():
        print('file not found:', JSON_PATH)
        return 2
    bak = JSON_PATH.parent / (JSON_PATH.name + '.envfixbak')
    shutil.copy2(JSON_PATH, bak)
    with JSON_PATH.open('r', encoding='utf-8') as f:
        data = json.load(f)

    ammo = data.get('ammo_v2', [])
    metro_count = 0
    icao_count = 0
    changed = 0
    for entry in ammo:
        env = entry.setdefault('environmental_conditions', {})
        std = (env.get('standard') or '').strip()
        if std == 'Metro':
            updated = False
            if env.get('humidity_pct') != 78.0:
                env['humidity_pct'] = 78.0
                updated = True
            if env.get('pressure_pa') != 99992.0:
                env['pressure_pa'] = 99992.0
                updated = True
            if updated:
                changed += 1
            metro_count += 1
        elif std == 'ICAO':
            if env.get('humidity_pct') != 0.0:
                env['humidity_pct'] = 0.0
                changed += 1
            icao_count += 1

    with JSON_PATH.open('w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f'Backup written to: {bak}')
    print(f'Entries processed: {len(ammo)}; Metro: {metro_count}; ICAO: {icao_count}; modified: {changed}')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
