import json
from pathlib import Path

IN = Path(__file__).parent / "dope_gui_cartridges_v2.json"
BAK = Path(str(IN) + ".bak")
OUT = IN

with IN.open('r', encoding='utf-8') as f:
    data = json.load(f)

if 'ammo_v2' not in data:
    raise SystemExit('unexpected format')

# meters for yards
Y25 = 22.86
Y100 = 91.44
Y200 = 182.88

changed = 0
for entry in data['ammo_v2']:
    # skip if already annotated
    if 'zero_range_m' in entry and ('trajectory_families' in entry or 'available_zero_ranges' not in entry):
        continue
    ref = entry.get('reference_barrel_inches', None)
    # classify by reference barrel length heuristics
    if ref is None:
        # fallback: treat as medium
        zero = Y100
        avail = [Y100]
    else:
        try:
            r = float(ref)
        except Exception:
            r = 16.0
        if r <= 6.0:
            zero = Y25
            avail = [Y25]
        elif r <= 14.0:
            zero = Y100
            avail = [Y100]
        else:
            zero = Y100
            avail = [Y100, Y200]
    entry['zero_range_m'] = zero
    # add available_zero_ranges only for entries with more than one option
    if len(avail) > 1:
        entry['available_zero_ranges'] = avail
    # If available_zero_ranges present and no explicit trajectory_families,
    # create per-zero trajectory_families by copying the top-level trajectory_profile
    if 'available_zero_ranges' in entry and 'trajectory_families' not in entry:
        tpl = entry.get('trajectory_profile', [])
        tfam = []
        for zr in entry['available_zero_ranges']:
            fam = {'zero_range_m': zr, 'trajectory_profile': tpl}
            tfam.append(fam)
        entry['trajectory_families'] = tfam
    changed += 1

if changed == 0:
    print('No changes made; all entries already annotated.')
else:
    print(f'Annotated {changed} entries; backing up and writing file.')
    with BAK.open('w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)
    # write out pretty JSON to OUT
    with OUT.open('w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)

print('Done')
