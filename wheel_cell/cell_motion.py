"""Joint keyframes for the fixed RS080N mount and fasten arms (numpy only, no USD).

World frame = cell frame (x along the line, +y = left side, z up). Each arm stands at
(base_x, side*ROBOT_BASE_Y, 0) and is turned to face the car (yaw 180 deg on the left side),
so targets are moved into the robot frame before IK. A plan is a list of (seconds, q).
"""
from math import ceil, pi

import numpy as np

from fitment import NUT_LENGTH, NUT_SEAT, PITCH, half_track
from process import (MOUNT_HUB_X, BELT_Z, FASTEN_BASE_X, HUB_Z, LINE_Y, MOUNT_BASE_X, NUT_CYCLE, NUT_DRIVE,
                     NUT_ENGAGE, NUT_FINAL, GRIP_AT, MOUNT_CLEAR, PICK_X, ROBOT_BASE_Y)

PARK = np.deg2rad([0, -30, -65, 0, -130, 0])
# PHL-W 63-080-S shoes sit 0.32 m out along the flange +Z (build.mount_gripper); the held wheel's
# axis is the flange Z, outboard face toward the flange.
GRIP_CENTRE = .32
# Mount: jaws open on the hub this long after the seat time (they close at process.GRIP_AT).
RELEASE_AT = .6
# Fasten: sockets take the five nuts at this offset from the tighten time.
NUT_PICK_AT = -2.0
# Compact five-spindle head of map_fasten (TOOL_LENGTH, flange to socket tip; e32662d, 2026-11-27).
HEAD_LENGTH = .235
NUT_ADVANCE = 5*PITCH
# Nut presenter pocket (socket tips meet it), beside the fastening arm, clear of the car and AMR.
MAGAZINE = (FASTEN_BASE_X+.90, 1.45, 1.135)
# Flange orientations as columns (x, y, z) in the world frame.
FACE_DOWN = np.array([[1., 0, 0], [0, -1, 0], [0, 0, -1]])
# Left-arm IK seed for picking: J1 turned round to the feed belt behind the robot, elbow up.
# The other branch (leaning back over the base) ends within 1-3 deg of the J2 limit.
PICK_SEED = np.deg2rad([-155, 19, -93, 180, 67, -205])


def seed_for(side, q):
    """Mirror a left-arm seed for the right arm (robot frames mirror about their x-z plane)."""
    return q if side > 0 else q*np.array([-1, 1, 1, -1, 1, -1])


def toward_car(side):
    return np.array([[1., 0, 0], [0, 0, -side], [0, side, 0]])


def base_pose(base_x, side):
    yaw = pi if side > 0 else 0.
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([base_x, side*ROBOT_BASE_Y, 0.]), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def flange_for(centre, rotation, reach):
    """Flange position that puts a point `reach` out along the flange +Z at `centre`."""
    return np.asarray(centre, float)-reach*rotation[:, 2]


def solve_plan(arm, base_x, side, keys):
    """keys: (t, None) for PARK, or (t, (flange_world, rotation_world)[, seed]). Returns [(t, q)]."""
    origin, yaw = base_pose(base_x, side)
    plan, previous = [], PARK
    for t, target, *seed in keys:
        if target is None:
            q = PARK
        else:
            position, rotation = target
            local = yaw.T@(np.asarray(position, float)-origin)
            start = seed_for(side, seed[0]) if seed else previous
            q = arm.solve(np.zeros(3), local, side, start, fallback=False, rotation=yaw.T@rotation)
            if q is None:
                raise ValueError(f"RS080N at x {base_x} side {side}: no IK on this branch at {t:.2f} s, {position}")
        if plan:
            dt = t-plan[-1][0]
            if dt <= 0 or np.any(abs(q-plan[-1][1]) > arm.speed*dt):
                need = max(abs(q-plan[-1][1])/arm.speed)
                raise ValueError(f"RS080N at x {base_x} side {side}: {need:.2f} s needed, {dt:.2f} s planned before {t:.2f} s")
        plan.append((t, q))
        previous = q
    return plan


def mount_keys(side, seat, wheel, car_x):
    """Pick the flat wheel at the conveyor stop, stand it up and hang it on the hub at `seat`."""
    pick = np.array([PICK_X, side*LINE_Y, BELT_Z+wheel.width/2])
    toward_base = np.array([MOUNT_BASE_X-PICK_X, side*(ROBOT_BASE_Y-LINE_Y), 0.])
    toward_base /= np.linalg.norm(toward_base)
    hub = np.array([MOUNT_HUB_X, side*half_track(car_x), HUB_Z])
    down, car = FACE_DOWN, toward_car(side)
    out = np.array([0., side, 0.])
    grip = lambda centre, rotation: (flange_for(centre, rotation, GRIP_CENTRE), rotation)
    return [(seat-8.5, None),
            (seat-6.0, grip(pick+(0, 0, .12)+.10*toward_base, down), PICK_SEED),
            (seat+GRIP_AT, grip(pick, down)),
            (seat-4.6, grip(pick+(0, 0, .15)+.35*toward_base, down)),
            (seat-2.6, grip(hub+.25*out, car)),
            (seat-1.2, grip(hub+.03*out, car)),
            (seat, grip(hub, car)),
            (seat+RELEASE_AT, grip(hub, car)),
            (seat+1.4, grip(hub+.15*out, car)),
            (seat+MOUNT_CLEAR, None)]


def fasten_keys(side, tight, car_x, hub_x):
    """Pick five nuts at the presenter, run them down on the hub (world x `hub_x`) at `tight`, park."""
    car = toward_car(side)
    out = half_track(car_x)+NUT_SEAT+NUT_LENGTH+NUT_ADVANCE+HEAD_LENGTH
    at = lambda y: ((hub_x, side*y, HUB_Z), car)
    mx, my, mz = MAGAZINE
    pocket = ((mx, side*(my+HEAD_LENGTH), mz), car)
    clear = ((mx, side*(my+HEAD_LENGTH+.10), mz), car)
    return [(tight-5.0, None), (tight-2.6, clear), (tight+NUT_PICK_AT, pocket), (tight-1.4, clear),
            (tight+1.94, at(out+.08)), (tight+NUT_ENGAGE, at(out)),
            (tight+NUT_ENGAGE+NUT_DRIVE, at(out-NUT_ADVANCE)),
            (tight+NUT_ENGAGE+NUT_DRIVE+NUT_FINAL, at(out-NUT_ADVANCE)),
            (tight+NUT_CYCLE, at(out+.08)), (tight+NUT_CYCLE+2.5, None)]


def samples(plan, rate=12, limit=48):
    """Joint-space linear samples between plan keys, for USD time samples."""
    out = [plan[0]]
    for (t0, q0), (t1, q1) in zip(plan, plan[1:]):
        n = 1 if np.allclose(q0, q1) else min(limit, ceil((t1-t0)*rate))
        out += [(t0+(t1-t0)*i/n, q0+(q1-q0)*i/n) for i in range(1, n+1)]
    return out
