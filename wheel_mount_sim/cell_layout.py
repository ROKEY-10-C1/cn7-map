"""Accumulation-end layout of the wheel cell feed belts (team scene map/scene/, factory override layer).

Changes against the original team cell (wheel_cell/, scene/), agreed on 2026-11-27:
- the pick zone in the middle of each station belt (V-stop, photo-eyes S1/S2, centering arms, lift bars) is gone;
- the station belt runs on to its end, past the mount robot on the side away from the fastening robot (-x);
- one fixed V end stop at the belt end. Tires come in flat (outboard face up) and accumulate against the stop,
  each pushed by the running rollers and the tire behind it, so the head tire always sits at the same place;
- the mount robot always picks at that place and turns to its -x side (no fastening robot there) to the hub,
  which is always at MOUNT_HUB_X (front wheel at dock A, rear wheel after the car indexes to B).

The pick point follows from the reach of the RS080N on the cell base (analysis/cell_reach.py): J1 has its hard
stop straight behind the robot, so it can only swing through -x, as the design wants; at 0.50 m to -x of the
base the J1 margin is >= 18.8 deg for every jaw angle (team criterion 15 deg, docs/RS080N_CELL_SAFETY.md).

USD part: tools/extend_belt_end.py writes the belt extension and the end stop into scene/overrides/factory.usda
(the hand-edit layer a scene rebuild never overwrites).
Live part: pre_build()/post_build() are called by wheel_cell/cell.py; they switch on whenever the opened stage
has the end stop (/World/Factory/LeftAccumEnd), otherwise the original pick zones stay.
"""
import sys
from math import radians, sin, cos, sqrt
from pathlib import Path

HERE = Path(__file__).resolve().parent
CELL = HERE.parent/"wheel_cell"
if str(CELL) not in sys.path:
    sys.path.append(str(CELL))

from process import BELT_Z, CN7_R16, LINE_Y, MOUNT_BASE_X  # noqa: E402

SCENE_DIR = HERE.parent/"scene"

TIRE_R = CN7_R16.radius                    # 0.316 m (205/55 R16)
PICK_DX = -.50                             # pick point to -x of the mount robot base (cell_reach.py)
PICK_X = MOUNT_BASE_X+PICK_DX              # head tire centre at the stop
V_HALF_ANGLE = 60.                         # deg, plate to flow axis (120 deg V, as the team V-stop)
APEX_X = PICK_X-TIRE_R/sin(radians(V_HALF_ANGLE))   # circle of radius R in the V sits R/sin(a) from the apex
PLATE = dict(start=.0, length=.30, thick=.024, z=BELT_Z+.10, height=.10)   # UHMW plate on each V leg
FRAME_END_X = APEX_X-.035                  # conveyor frame and stop bracket end
ROLLER_PITCH = .10347826                   # team feed belt (LeftFeed_roller0..)
ROLLER_X0 = .0017391304                    # team roller 0, the first one we continue from
GUIDE_START_X = PICK_X+.30                 # side guides (100 mm high) stop here: the jaws close across the flow
                                           # at the head tire (+-18 deg) and need the sides free
QUEUE_GAP = .90                            # initial spacing of the tires on the belt (process.QUEUE_GAP)
FIRST_AHEAD = .70                          # first tire put on the belt this far upstream of the pick point
SEATED_SPEED = .005                        # m/s: tire counts as stopped below this
SEATED_TIME = .5                           # s blocked eye + stopped before pick_ready
import os as _os
FIXED_SPIN = 36. if _os.environ.get("MOUNT_FIXED", "0") == "1" else None   # deg, tire spin on the belt
FEED_TIMEOUT = 25.                         # s for the head tire to reach the stop


def stop_plates(side):
    """(centre, yaw deg, scale) of the two V plates of one side, world frame."""
    a = radians(V_HALF_ANGLE)
    y0 = side*LINE_Y
    out = []
    for sgn in (-1, 1):
        mid = PLATE["start"]+PLATE["length"]/2
        # inner face on the V leg: shift the plate centre outward by half its thickness
        cx = APEX_X+mid*cos(a)-PLATE["thick"]/2*sin(a)
        cy = y0+sgn*(mid*sin(a)+PLATE["thick"]/2*cos(a))
        out.append(((cx, cy, PLATE["z"]), sgn*V_HALF_ANGLE, (PLATE["length"], PLATE["thick"], PLATE["height"])))
    return out


def new_roller_x():
    """Centres of the rollers that continue the team belt to -x, down to the frame end."""
    xs, k = [], 1
    while ROLLER_X0-k*ROLLER_PITCH > FRAME_END_X+.03:
        xs.append(ROLLER_X0-k*ROLLER_PITCH)
        k += 1
    return xs


# ------------------------------------------------------------------------------------------ live physics

def active(stage):
    return bool(stage.GetPrimAtPath("/World/Factory/LeftAccumEnd"))


_ORIGINAL = {}
_SIDES = {}


def pre_build(stage):
    """Before the cell builds its factory and stations: on our copy, swap the pick zone for the end stop and
    shorten the side guides. On any other stage restore the team modules (a Kit session can open both)."""
    import conveyor
    import handling
    _ORIGINAL.setdefault("PickZone", handling.PickZone)
    _ORIGINAL.setdefault("FEED_START_X", conveyor.FEED_START_X)
    _ORIGINAL.setdefault("PICK_X", conveyor.PICK_X)
    _SIDES.clear()
    if active(stage):
        handling.PickZone = EndStopZone
        conveyor.FEED_START_X = GUIDE_START_X
        # conveyor.py leaves the belt rollers at x <= PICK_X+0.33 without physics (the pick zone brings its own);
        # with no pick zone every roller down to the end stop must convey
        conveyor.PICK_X = -1e9
    else:
        handling.PickZone = _ORIGINAL["PickZone"]
        conveyor.FEED_START_X = _ORIGINAL["FEED_START_X"]
        conveyor.PICK_X = _ORIGINAL["PICK_X"]
    return active(stage)


def post_build(stage, stations):
    """After the stations exist: put the tires on the belt in queue order (front wheel first, it is mounted
    first at dock A), upstream of the stop so they convey in and accumulate."""
    if not active(stage):
        return
    from pxr import Gf
    for side in (1, -1):
        queue = sorted((s for s in stations if s.side == side), key=lambda s: -s.hub[0])
        for i, unit in enumerate(queue):
            x = PICK_X+FIRST_AHEAD+i*QUEUE_GAP
            for path in (unit.wheel, unit.tire):
                op = stage.GetPrimAtPath(path).GetAttribute("xformOp:translate")
                p = op.Get()
                op.Set(Gf.Vec3d(x, p[1], p[2]))
        _SIDES[side].order = [unit.wheel for unit in queue]
        if FIXED_SPIN is not None:
            # fixed setup (MOUNT_FIXED=1, no vision): every tire is put on the belt at the same spin, so the
            # taught grip yaw puts a hole on a stud (hub parked stud-at-top, cell_config)
            q = Gf.Quatd(cos(radians(FIXED_SPIN)/2), 0., 0., sin(radians(FIXED_SPIN)/2))
            for unit in queue:
                for path in (unit.wheel, unit.tire):
                    op = stage.GetPrimAtPath(path).GetAttribute("xformOp:orient")
                    op.Set(Gf.Quatf(q) if op.GetTypeName() == "quatf" else q)
        for unit in queue:
            # station.py holds an 'arriving' station until its tire passes the pick-zone inlet (axle+START_AHEAD);
            # the end stop has no inlet and steps every tire on its belt from the start
            unit.phase = "queued"


class _EndStop:
    """One per belt: the fixed V stop (UHMW plates from the USD copy, made colliders) and a diffuse photo-eye
    in the V that sees the head tire (the PLC's 'tire at pick' signal)."""

    def __init__(self, stage, side, guide_material):
        from conveyor import collider
        from handling import PhotoEye
        self.side = side
        label = "Left" if side > 0 else "Right"
        self.path = f"/World/Factory/{label}AccumEnd"
        for k in range(2):
            prim = stage.GetPrimAtPath(f"{self.path}/Plate_{k}")
            collider(prim, guide_material)
        bracket = stage.GetPrimAtPath(f"{self.path}/Bracket")
        if bracket:
            collider(bracket)
        seated = TIRE_R/sin(radians(V_HALF_ANGLE))-TIRE_R     # apex to tread of a seated tire
        root = f"/World/Live/AccumEnd{label}"
        self.eye = PhotoEye(stage, root+"/PhotoEye", (APEX_X+.01, side*LINE_Y, PLATE["z"]), (1, 0, 0),
                            seated-.01+.003, ignore=(self.path,))
        self.order = []
        self.picked = set()
        # the belt's roller drives (team feed rollers + our extension): the PLC stops them while the head tire sits
        # in the stop and restarts them when it is taken (accumulation conveyor 'zone full' logic). Rollers running
        # under a standing queue only rub the treads; stopped, the tires come to rest (and PhysX lets them sleep).
        from pxr import UsdPhysics
        self.drives = []
        factory = stage.GetPrimAtPath("/World/Factory")
        for prim in factory.GetChildren():
            n = prim.GetName()
            if n.startswith(label+"Feed_roller") and n.endswith("_Axle"):
                d = UsdPhysics.DriveAPI(prim, "angular")
                self.drives.append((d.GetTargetVelocityAttr(), d.GetTargetVelocityAttr().Get()))
        self.running = True

    def run(self, on):
        if on != self.running:
            for attr, rate in self.drives:
                attr.Set(rate if on else 0.)
            self.running = on

    def head(self):
        return next((w for w in self.order if w not in self.picked), None)


class EndStopZone:
    """Takes the place of handling.PickZone for one station (same constructor and the attributes the cell
    uses). Stations of one side share the belt end; a station is pick_ready when its tire is the head of the
    queue, the eye in the V is blocked and the tire has stopped for SEATED_TIME."""

    def __init__(self, stage, root, axle, side, roller_material, guide_material):
        if side not in _SIDES:
            _SIDES[side] = _EndStop(stage, side, guide_material)
        self.end = _SIDES[side]
        self.root = root
        self.wheel = root+"/Wheel"
        self.side = side
        self.axle = PICK_X                 # reference for the team checks' along_mm (station.py, check_physics.py)
        self.state = "queued"
        self.timer = 0.
        self.still = 0.
        self.contact_bodies = []           # the stop is static; tires touching each other in the queue is intended
        self.stopper_path = self.end.path
        self.s1 = self.s2 = self.end.eye
        self._last = None

    def step(self, dt):
        import omni.physx
        self.timer += dt
        pose = omni.physx.get_physx_interface().get_rigidbody_transformation(self.wheel)
        if not pose.get("ret_val"):
            return self.state
        p = pose["position"]
        speed = 0. if self._last is None else sqrt(sum((a-b)**2 for a, b in zip(p, self._last)))/dt
        self._last = tuple(p)
        is_head = self.end.head() == self.wheel
        if not is_head:
            self.state = "queued"
            return self.state
        if self.state == "queued":
            self.state, self.timer = "arriving", 0.
        blocked = self.end.eye.read()
        self.still = self.still+dt if (blocked and speed < SEATED_SPEED) else 0.
        if self.still >= SEATED_TIME:
            self.state = "pick_ready"
            self.end.run(False)                    # zone full: stop the belt
            self.along_mm = (p[0]-PICK_X)*1000
            self.across_mm = (p[1]-self.side*LINE_Y)*1000
        elif self.timer > FEED_TIMEOUT:
            self.state = "feed_failed"
        return self.state

    def picked(self):
        """Control calls this once the robot has lifted the tire off the belt: the next tire becomes the head."""
        self.end.picked.add(self.wheel)
        self.end.run(True)                         # head taken: run the belt, the next tire moves into the stop
