import json
import os

def migrate():
    json_path = 'tools/native_gui/dope_gui_cartridges_v2.json'
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found.")
        return

    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # Conversion constants
    KG_TO_GRAINS = 15432.358352941
    M_TO_INCHES = 39.37007874

    for c in data.get('ammo_v2', []):
        name = c.get('name', 'Unknown')

        # 1. Convert units to Imperial
        if 'mass_kg' in c:
            c['mass_grains'] = round(c['mass_kg'] * KG_TO_GRAINS, 1)
            del c['mass_kg']
        if 'caliber_m' in c:
            c['caliber_inches'] = round(c['caliber_m'] * M_TO_INCHES, 3)
            del c['caliber_m']
        if 'length_m' in c:
            c['length_inches'] = round(c['length_m'] * M_TO_INCHES, 3)
            del c['length_m']

        # 2. Drag model & BC object
        if 'drag_model' in c:
            dm = c['drag_model']
            bc = c.get('ballistic_coefficient', 0.5)
            c['preferred_drag_model'] = dm
            c['ballistic_coefficients'] = {dm: bc}
            del c['drag_model']
            if 'ballistic_coefficient' in c:
                del c['ballistic_coefficient']

        # 3. Trajectory families migration
        if 'trajectory_families' not in c or not c['trajectory_families']:
            # Create from top-level profile if available
            tp = c.get('trajectory_profile', [])
            zr = c.get('zero_range_m', 0.0)
            if tp:
                c['trajectory_families'] = [{
                    "zero_range_m": zr,
                    "trajectory_profile": tp
                }]
        
        # 4. Remove top-level profile
        if 'trajectory_profile' in c:
            if not c['trajectory_families']:
                # Ensure we don't lose data
                c['trajectory_families'] = [{
                    "zero_range_m": c.get('zero_range_m', 0.0),
                    "trajectory_profile": c['trajectory_profile']
                }]
            del c['trajectory_profile']

        # 5. Environmental conditions
        # Default to ICAO standard (15C, 1013.25hPa, 0% humidity, 0m altitude)
        # Prefer explicit Pascals in migrated data
        env = {
            "temperature_f": 59.0,
            "pressure_pa": 101325.0,
            "altitude_ft": 0.0,
            "humidity_pct": 0.0,
            "standard": "ICAO"
        }

        # Specific override for Hornady 6.5 Creedmoor if we recognize it
        if "Hornady 6.5 Creedmoor 140gr ELD-M" in name:
            # Compute Pascal equivalent for provided inHg value and emit Pascals only
            env_inhg = 29.53
            env = {
                "temperature_f": 70.0,
                "pressure_pa": round(env_inhg * 3386.389, 3),
                "altitude_ft": 1000.0,
                "humidity_pct": 0.45,
                "standard": "Custom"
            }
            # Also ensure the wind_drift_profile I injected earlier is present
            if 'wind_drift_profile' not in c:
                c['wind_drift_profile'] = [
                    {'distance_m': 0.0, 'wind_drift_m': 0.0},
                    {'distance_m': 45.72, 'wind_drift_m': 0.003048},
                    {'distance_m': 91.44, 'wind_drift_m': 0.012192},
                    {'distance_m': 137.16, 'wind_drift_m': 0.02794},
                    {'distance_m': 182.88, 'wind_drift_m': 0.050546},
                    {'distance_m': 228.6, 'wind_drift_m': 0.079756},
                    {'distance_m': 274.32, 'wind_drift_m': 0.116332},
                    {'distance_m': 320.04, 'wind_drift_m': 0.160274},
                    {'distance_m': 365.76, 'wind_drift_m': 0.21209},
                    {'distance_m': 411.48, 'wind_drift_m': 0.272034},
                    {'distance_m': 457.2, 'wind_drift_m': 0.34036},
                    {'distance_m': 548.64, 'wind_drift_m': 0.503428},
                    {'distance_m': 640.08, 'wind_drift_m': 0.704342},
                    {'distance_m': 731.52, 'wind_drift_m': 0.94615},
                    {'distance_m': 822.96, 'wind_drift_m': 1.2319},
                    {'distance_m': 914.4, 'wind_drift_m': 1.564894}
                ]

        c['environmental_conditions'] = env

    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)

    print("Migration complete!")

if __name__ == '__main__':
    migrate()
