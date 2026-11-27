"""Model parameters for the live cell, in one place.

Values come from DEFAULTS, overridden by the JSON file named in the CELL_CONFIG environment
variable (run.sh / check_physics.sh accept --config FILE and set it), e.g.

    {"physics": {"hz": 480}}

There is one cell model (method 1 pick zone, hub on bearing and suspension, ring tire, camera
measurement); the file holds its numbers, not alternative models. Values that stand in for
measurements are listed in calibration.json.
"""
import copy
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent

DEFAULTS = {
    "physics": {
        "hz": 240,
        "position_iterations": 64,
        "velocity_iterations": 8,
        # Merge the wheel mounting-face triangles into convex pieces (same shape, ~half the
        # pieces). Off by default: contact points differ slightly from the triangle set.
        "merge_web_colliders": False,
    },
    "hub": {
        "bearing_drag_nm": 1.2,           # wheel bearing + brake pad drag torque (estimated)
        "radial_play_mm": .03,            # bearing radial play
        "stud_at_top": True,              # True: park every hub with one stud at 12 o'clock; False: random angle
    },
    "suspension": {                       # corner hangs on its rebound stop (car lifted)
        "wheel_rate_n_per_mm": 22.,       # CN7 wheel rate, ~22-25 N/mm (estimated)
        "damping_ns_per_m": 1800.,
        "lateral_rate_n_per_mm": 900.,    # knuckle / bushing compliance along the axle
        "droop_preload_mm": 6.,           # spring preload holding the hanging corner on its rebound stop
    },
    "tolerance": {                        # per-seed hub dimensions
        "pilot_diameter_mm": (-.03, 0.),  # h8-like range on the pilot
        "stud_position_mm": .05,          # true-position radius of each stud
        "pcd_mm": .05,
    },
    "tire": {
        # tread ring on springs around the rim
        "radial_rate_n_per_mm": 210.,     # 205/55 R16 at 2.3 bar (estimated)
        "lateral_rate_n_per_mm": 140.,
        "torsional_rate_nm_per_deg": 55.,
        "damping_ratio": .08,
    },
    "sensing": {
        # wheel-to-hub angle as the camera reports it, not ground truth
        "wheel_angle_noise_deg": .03,     # 1 sigma of the bolt-pattern angle from the depth image
        "center_noise_mm": .05,
        "update_hz": 30,                  # camera rate; the last frame is held between frames
        "latency_s": .033,                # one frame of processing delay
    },
}


def _merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def load():
    config = copy.deepcopy(DEFAULTS)
    path = os.environ.get("CELL_CONFIG")
    if path:
        _merge(config, json.loads(Path(path).read_text()))
    return config


CONFIG = load()


def calibration():
    path = HERE/"calibration.json"
    return json.loads(path.read_text()) if path.exists() else {}
