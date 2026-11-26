"""Settings for the live cell, in one place.

Values come from DEFAULTS, overridden by the JSON file named in the CELL_CONFIG environment
variable (run.sh / check_physics.sh accept --config FILE and set it), e.g.

    {"physics": {"hz": 480}}

The defaults reproduce the cell exactly as it runs without this file. Later integration steps
add their own sections here, each switched off by default.
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
