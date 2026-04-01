import json
from pathlib import Path


def ensure_gun_fields(gun: dict):
    # Ensure heat/accuracy fields exist; leave null so user can fill them
    gun.setdefault("barrel_material", gun.get("barrel_material", None))
    gun.setdefault("free_floated", gun.get("free_floated", None))
    gun.setdefault("angular_sigma_moa", gun.get("angular_sigma_moa", None))
    gun.setdefault("chamber_time", None)
    gun.setdefault("barrel_profile", {"wall_thickness_mm": None, "taper": None})
    gun.setdefault("ammo_temp", None)


def ensure_cartridge_fields(cartridge: dict):
    # Add ammo temperature field to cartridges
    cartridge.setdefault("ammo_temp", None)


def update_files(root: Path):
    guns_path = root / "tools" / "native_gui" / "dope_gui_guns_v2.json"
    carts_path = root / "tools" / "native_gui" / "dope_gui_cartridges_v2.json"

    # Update guns
    if guns_path.exists():
        data = json.loads(guns_path.read_text(encoding="utf-8"))
        changed = False
        for g in data:
            before = dict(g)
            ensure_gun_fields(g)
            if g != before:
                changed = True
        if changed:
            guns_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(f"Updated {guns_path}")
        else:
            print(f"No changes needed for {guns_path}")
    else:
        print(f"Guns file not found: {guns_path}")

    # Update cartridges
    if carts_path.exists():
        data = json.loads(carts_path.read_text(encoding="utf-8"))
        changed = False
        # expect top-level dict with key 'ammo_v2' (existing format)
        if isinstance(data, dict) and "ammo_v2" in data:
            for c in data["ammo_v2"]:
                before = dict(c)
                ensure_cartridge_fields(c)
                if c != before:
                    changed = True
            if changed:
                carts_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
                print(f"Updated {carts_path}")
            else:
                print(f"No changes needed for {carts_path}")
        else:
            print(f"Unexpected cartridges file structure: {carts_path}")
    else:
        print(f"Cartridges file not found: {carts_path}")


if __name__ == "__main__":
    repo_root = Path(__file__).resolve().parents[2]
    update_files(repo_root)
