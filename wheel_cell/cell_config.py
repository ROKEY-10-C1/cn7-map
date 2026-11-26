"""Settings for the live cell, in one place.

Values come from DEFAULTS, overridden by the JSON file named in the CELL_CONFIG environment
variable (run.sh / check_physics.sh accept --config FILE and set it), e.g.

    {"physics": {"hz": 480}}

Defaults keep the cell's behaviour except one fix: the tip fixture parks at 0.255 m, clear of
the withdrawing mount arm (E12). The --jam offset and the post-seat press target are settings.
Parameters that stand in for measurements are listed in calibration.json.
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
    "fixture": {
        # Tip fixture parked beside the axle after the hand-off. At 0.24 m the upright beam
        # touched the withdrawing mount arm (Link_5 / camera) by 1-3 mm on every seed (E12).
        "park_y": .255,
    },
    "insertion": {
        # --jam: FL fixture offset. 2 mm is inside the 1.5 mm float + 0.7 mm hole clearance, so
        # whether FL jams depends on small trajectory differences (seed 42 jams, seeds 7 and 123
        # seat, with either park position). 3-4 mm push the car through the floating spindle and
        # jam other wheels or lose the dock instead.
        "jam_offset_mm": 2.,
        # After seating, hold the reached position instead of driving 6 mm past the hub face
        # (a standing press force on each corner can push the docked car sideways). Off: in the
        # current cycle the car does not drift, and holding early leaves up to 0.36° angle error.
        "hold_after_seat": False,
    },
    # Vehicle corner and wheel models (corner.py). Defaults keep the original model: hub
    # colliders fixed to the body, one rigid wheel body, true wheel angle. configs/realistic.json
    # switches all of them on.
    "corner": {
        "model": "fixed",                 # "fixed" / "bearing": knuckle on suspension, hub on its bearing
        "chamfers": True,                 # bearing model: lead-in on pilot and stud ends
    },
    "hub": {                              # bearing model only
        "random_angle": True,             # the stopped car parks its hubs at any angle
        "free_rotation": True,            # hub turns on its bearing, held by drag only
        "bearing_drag_nm": 1.2,           # wheel bearing + brake pad drag torque (estimated)
        "radial_play_mm": .03,            # bearing radial play
    },
    "suspension": {                       # bearing model only
        "enabled": True,
        "wheel_rate_n_per_mm": 22.,       # CN7 wheel rate, ~22-25 N/mm (estimated)
        "damping_ns_per_m": 1800.,
        "lateral_rate_n_per_mm": 900.,    # knuckle / bushing compliance along the axle
        "droop_preload_mm": 6.,           # spring preload holding the hanging corner on its rebound stop
    },
    "tolerance": {                        # bearing model only: per-seed hub dimensions
        "enabled": True,
        "pilot_diameter_mm": (-.03, 0.),  # h8-like range on the pilot
        "stud_position_mm": .05,          # true-position radius of each stud
        "pcd_mm": .05,
    },
    "tire": {
        "model": "rigid",                 # "rigid" / "ring": tread ring on springs around the rim
        "radial_rate_n_per_mm": 210.,     # 205/55 R16 at 2.3 bar (estimated)
        "lateral_rate_n_per_mm": 140.,
        "torsional_rate_nm_per_deg": 55.,
        "damping_ratio": .08,
    },
    "sensing": {
        "enabled": False,                 # wheel-to-hub angle from the camera instead of ground truth
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
