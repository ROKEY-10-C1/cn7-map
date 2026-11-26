"""Replace an estimated parameter with a measured one.

    python3 map/wheel_cell/calibrate.py tire_radial_rate tire_test.csv
    python3 map/wheel_cell/calibrate.py pad_tire_friction --value 0.38 0.31 --source "coupon test 2026-12-02"

A CSV holds two columns (x, y), e.g. deflection mm and load N; the least-squares slope through
the origin becomes the rate. The value is written to measured_config.json (use it with
--config) and the entry in calibration.json is marked "measured" with its source.
"""
import argparse
import csv
import json
from datetime import date
from pathlib import Path

HERE = Path(__file__).resolve().parent
REGISTER = HERE/"calibration.json"
OVERRIDES = HERE/"measured_config.json"
# calibration.json key -> (cell_config section, key)
CONFIG_KEYS = {
    "tire_radial_rate": ("tire", "radial_rate_n_per_mm"),
    "tire_lateral_rate": ("tire", "lateral_rate_n_per_mm"),
    "tire_torsional_rate": ("tire", "torsional_rate_nm_per_deg"),
    "suspension_wheel_rate": ("suspension", "wheel_rate_n_per_mm"),
    "suspension_lateral_rate": ("suspension", "lateral_rate_n_per_mm"),
    "hub_bearing_drag": ("hub", "bearing_drag_nm"),
    "hub_radial_play": ("hub", "radial_play_mm"),
    "seat_force_threshold": ("insertion", "seat_force_n"),
    "nut_factor_K": ("nutrunner", "nut_factor"),
    "joint_rate_after_snug": ("nutrunner", "joint_rate_nm_per_deg"),
    "camera_angle_noise": ("sensing", "wheel_angle_noise_deg"),
    "servo_lag": ("actuators", "lag_s"),
    "gearbox_backlash": ("actuators", "backlash_deg"),
}


def slope(path):
    rows = [(float(a), float(b)) for a, b, *_ in csv.reader(open(path)) if a.strip() and a[0] not in "#xX"]
    return sum(x*y for x, y in rows)/sum(x*x for x, _ in rows), len(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("key")
    parser.add_argument("csv", nargs="?")
    parser.add_argument("--value", type=float, nargs="+")
    parser.add_argument("--source", default="")
    args = parser.parse_args()
    register = json.loads(REGISTER.read_text())
    if args.key not in register:
        parser.error(f"unknown key; choose from {sorted(k for k in register if not k.startswith('_'))}")
    if args.csv:
        value, n = slope(args.csv)
        source = args.source or f"{Path(args.csv).name}, {n} points, LSQ slope"
    elif args.value:
        value = args.value[0] if len(args.value) == 1 else args.value
        source = args.source or "entered value"
    else:
        parser.error("give a CSV or --value")
    entry = register[args.key]
    entry.update({"value": value, "status": "measured", "source": f"{source} ({date.today()})"})
    REGISTER.write_text(json.dumps(register, indent=2, ensure_ascii=False)+"\n")
    if args.key in CONFIG_KEYS:
        section, key = CONFIG_KEYS[args.key]
        overrides = json.loads(OVERRIDES.read_text()) if OVERRIDES.exists() else {}
        overrides.setdefault(section, {})[key] = value
        OVERRIDES.write_text(json.dumps(overrides, indent=2)+"\n")
        print(f"{args.key} = {value} -> {OVERRIDES.name} [{section}.{key}]; run with --config {OVERRIDES}")
    else:
        print(f"{args.key} = {value} recorded; this value is set in physical_data.py / station code")


if __name__ == "__main__":
    main()
