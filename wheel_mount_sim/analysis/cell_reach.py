"""Reach check for the accumulation-end pick in the wheel cell (left arm; the right one is its mirror).

For candidate pick points behind and to the -x side of the mount robot base (away from the fastening robot)
it solves the RS080N inverse kinematics (numpy FK of the URDF, bounded least squares, many seeds) for:
pick-above, pick, lift, the via point on the hub axis 0.45 m outboard, pre-insert 0.12 m and seated,
with the jaws across the flow (+-18 deg, the freedom the 72 deg bolt pattern leaves). It reports the
joint margins and in which direction J1 swings from the pick to the hub (must pass the -x side).
Run: ./tools/pxrpy.sh analysis/cell_reach.py
"""
import sys
from math import atan2, degrees
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent/"wheel_cell"))
import types  # noqa: E402
import pxr  # noqa: E402
if not hasattr(pxr, "PhysxSchema"):            # plain USD build: the schema is only needed to build stages
    pxr.PhysxSchema = sys.modules["pxr.PhysxSchema"] = types.ModuleType("pxr.PhysxSchema")
import scene as S  # noqa: E402
from process import BELT_Z, LINE_Y, MOUNT_BASE_X, MOUNT_HUB_X, ROBOT_BASE_Y, HUB_Z  # noqa: E402
from fitment import FRONT_TRACK  # noqa: E402

LO = np.array([-3.14, -1.83, -2.70, -6.28, -2.53, -6.28])
HI = np.array([3.14, 2.44, 2.35, 6.28, 2.53, 6.28])
BASE = np.array([MOUNT_BASE_X, ROBOT_BASE_Y, 0.])
YAW = 270.
RB = S.rot((0, 0, 1), YAW)
HUB = np.array([MOUNT_HUB_X, FRONT_TRACK/2, HUB_Z])
WHEEL_Z = BELT_Z+.1025          # flat tire: centre half the 205 mm width above the belt
rng = np.random.default_rng(3)


def fk(q):
    R6, p6 = S.link_poses(q)["link6"]
    return RB@R6, BASE+RB@p6


def err(q, Rt, pt):
    R, p = fk(q)
    dR = Rt@R.T
    return np.concatenate([(pt-p)*10, [dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]]])


def ik(R_G, o_G, warm=None):
    Rt, pt = S.flange_pose(R_G, o_G)
    seeds = ([warm] if warm is not None else [])+[rng.uniform(LO*.9, HI*.9) for _ in range(60)]
    best = None
    for s in seeds:
        r = least_squares(err, s, bounds=(LO, HI), args=(Rt, pt), xtol=1e-12, ftol=1e-12)
        if np.linalg.norm(r.fun) < 1e-5:
            d = 0. if warm is None else float(np.max(np.abs(r.x-warm)))
            if best is None or d < best[0]:
                best = (d, r.x)
            if warm is not None and d < .4:
                break
    return None if best is None else best[1]


def margin(q):
    return float(np.degrees(np.min(np.minimum(q-LO, HI-q))))


def azimuth(q):
    _, p = fk(q)
    return degrees(atan2(p[1]-BASE[1], p[0]-BASE[0]))


R_HUBM = S.R_MOUNT
YAWS = [270.]                               # cell base orientation (J1 zero toward the car, hard stop straight back)
DXS = [float(v) for v in (sys.argv[1:] or ["-.30", "-.40", "-.50", "-.60"])]


def solve_chain(R_pick, pick, hub):
    """Joint chain pick-above .. seat on one branch, J1 seeded from the wrist azimuth (az = zero - J1)."""
    zero = azimuth(np.zeros(6))
    az_pick = degrees(atan2(pick[1]-BASE[1], pick[0]-BASE[0]))
    j1 = np.radians(((zero-az_pick)+180) % 360-180)
    warm = np.array([j1, .3, -1.2, 0., -1.2, 0.])
    seq = []
    for name, R, o in (("above", R_pick, pick+[0, 0, .20]), ("pick", R_pick, pick), ("lift", R_pick, pick+[0, 0, .25]),
                       ("via", R_HUBM, hub+[0, .45, 0]), ("pre", R_HUBM, hub+[0, .12, 0]), ("seat", R_HUBM, hub)):
        if name == "via":                       # continue J1 the long way (through -x): wrap the expected value
            az_via = degrees(atan2(hub[1]+.45-BASE[1], hub[0]-BASE[0]))
            j1v = np.radians(((zero-az_via)+180) % 360-180)
            warm = warm.copy()
            warm[0] = j1v
        q = ik(R, o, warm)
        if q is None:
            return seq, name
        seq.append((name, q))
        warm = q
    return seq, None


for yaw in YAWS:
    RB[:] = S.rot((0, 0, 1), yaw)
    print(f"== base yaw {yaw:.0f} deg (cell value 270), J1 zero azimuth {azimuth(np.zeros(6)):.0f} deg")
    for dx in DXS:
        pick = np.array([MOUNT_BASE_X+dx, LINE_Y, WHEEL_Z])
        for jaw in (-18., 0., 18.):
            seq, fail = solve_chain(S.rot((0, 0, 1), 90.+jaw), pick, HUB)
            if fail:
                print(f"   pick dx {dx:+.2f} jaw {jaw:+.0f}: {fail} FAIL")
                continue
            q_lift, q_via = seq[2][1], seq[3][1]
            mids = [azimuth(q_lift+(q_via-q_lift)*t) for t in np.linspace(0, 1, 9)]
            print(f"   pick dx {dx:+.2f} jaw {jaw:+.0f}: J1 {degrees(q_lift[0]):6.1f} -> {degrees(q_via[0]):6.1f}, "
                  f"swing az {[round(a) for a in mids]}, through -x {any(abs(a) > 150 for a in mids)}, "
                  f"J1 margin {min(180-abs(degrees(q[0])) for _, q in seq):.1f}, J2/J3/J5 margin {min(margin(q[[1,2,4]]) if False else float(np.degrees(np.min(np.minimum(q[[1,2,4]]-LO[[1,2,4]], HI[[1,2,4]]-q[[1,2,4]])))) for _, q in seq):.1f} deg, J2/J3 at pick "
                  f"{degrees(seq[1][1][1]):.0f}/{degrees(seq[1][1][2]):.0f}")
