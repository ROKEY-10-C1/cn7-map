"""CN7 body underside collision, vehicle buffer stands, and the AMR buffer pick-up.

The body mesh (/AssetLibrary/Body) is an outer shell: its lowest parts are the side sills and the
bumper corners (car y +-0.65..0.91 m, z from 1.01 m); the floor pan is not modelled and stays the
Undertray box, the wheel openings stay the Wheelhouse arch proxies. The buffer stands carry the parked car at the sill ends (car x +-2.05 m), so the
sills are cut from the mesh into collision blocks: 0.1 m slices along the car, two bands across.
Slice edges fall on the stand edges, so a block never reaches below the mesh over a stand.
"""
from math import atan2, pi, sqrt

from pxr import Gf, Usd, UsdGeom

from conveyor import collider, quaternion
from physical_data import AMR_ACCELERATION, AMR_LIFT_MASS, AMR_LIFT_SPEED, AMR_MASS, AMR_SPEED_Y
from physical_data import AMR_TRACTION_COEFFICIENT, BODY_MASS
from fitment import ARCH_RADIUS
from process import AMR_BAY_Y, AMR_LIFT_CARRY, AMR_LIFT_ENTRY, STATIONS
from sim_util import cube, vec

SILL_INNER = .65          # car y where the sill starts (inboard of it the shell is at 1.5-1.6 m)
SILL_SPLIT = .78          # stand outer edge (y 0.68 + 0.10)
SLICE = .1
SLICE_START = -2.35       # slice edges at ..., -2.15, -2.05, -1.95, ... = the stand edges
BLOCK_HEIGHT = .10
BLOCK_GAP = .003
AISLE_X = -7.             # AMR lane beside the buffers
LIFT_DATUM = -.12         # hanger offset at lift joint position 0 (the car's height at the gate)


def body_points(stage):
    lib = stage.GetPrimAtPath("/AssetLibrary/Body")
    cache = UsdGeom.XformCache()
    inverse = cache.GetLocalToWorldTransform(lib).GetInverse()
    points = []
    for prim in Usd.PrimRange(lib, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            matrix = cache.GetLocalToWorldTransform(prim)*inverse
            points += [matrix.Transform(Gf.Vec3d(p)) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get() or ()]
    return points


def body_underside(stage, car_path):
    """Sill and bumper-corner collision blocks on the car body. Returns how many were made."""
    points = body_points(stage)
    # Over the wheel openings the lowest mesh points are the arch edge, not a sill; the arch is the
    # Wheelhouse proxy (fitment.ARCH_RADIUS + 25 mm liner) and the wheel has to pass under it.
    axles = sorted({axle for axle, _ in STATIONS.values()})
    reach = ARCH_RADIUS+.025
    count = 0
    for k in range(47):
        x0 = SLICE_START+k*SLICE
        if any(x0 < axle+reach and x0+SLICE > axle-reach for axle in axles):
            continue
        in_slice = [p for p in points if x0 <= p[0] < x0+SLICE]
        for side in (-1, 1):
            for band, (y0, y1) in enumerate(((SILL_INNER, SILL_SPLIT), (SILL_SPLIT, 1.))):
                zs = [p[2] for p in in_slice if y0 <= side*p[1] < y1]
                if not zs:
                    continue
                outer = y1 if band == 0 else max(side*p[1] for p in in_slice if side*p[1] >= y0)
                bottom = min(zs)
                # 3 mm short at each end: a lower neighbouring block flush with a stand's side face
                # held the car by friction against that face when the hanger lifted it.
                cube(stage, f"{car_path}/Sill_{'L' if side > 0 else 'R'}{band}_{k}",
                     (x0+SLICE/2, side*(y0+outer)/2, bottom+BLOCK_HEIGHT/2), (SLICE-2*BLOCK_GAP, outer-y0, BLOCK_HEIGHT))
                count += 1
    return count


def buffer_stands(stage):
    """The buffer stands are static colliders (the car rests on them, the hanger passes them)."""
    stands = [p for p in stage.Traverse() if p.GetName().startswith("Stand_") and "VehicleBuffer" in str(p.GetPath())]
    for prim in stands:
        collider(prim)
    return [str(p.GetPath()) for p in stands]


def move_rigidly(stage, paths, moved):
    """Apply one world transform change to several bodies (before the simulation starts)."""
    cache = UsdGeom.XformCache()
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        world = cache.GetLocalToWorldTransform(prim)*moved
        xform = UsdGeom.Xformable(prim)
        ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
        ops["xformOp:translate"].Set(world.ExtractTranslation())
        rotation = world.ExtractRotationQuat()
        if "xformOp:orient" in ops:
            ops["xformOp:orient"].Set(Gf.Quatf(rotation) if ops["xformOp:orient"].GetPrecision() ==
                                      UsdGeom.XformOp.PrecisionFloat else Gf.Quatd(rotation))
        else:
            xform.AddOrientOp().Set(Gf.Quatf(rotation))


class BufferRun:
    """AMR + hanger fetch a car from an inbound buffer slot: crab in under the parked car at the
    entry height, lift it off the stands to the carry height, crab back out to the lane.

    Uses the cell's Carrier bodies and its traction/lift drives with its own targets (the cell's
    gate -> dock logic is bypassed). Heights are hanger offsets from process.py (0 = docked)."""

    def __init__(self, carrier, slot_x, entry=AMR_LIFT_ENTRY, carry=AMR_LIFT_CARRY):
        self.c = carrier
        self.slot_x = slot_x
        self.entry = entry
        self.carry = carry
        self.phase = "lower"
        self.timer = 0.
        self.joint = 0.
        self.previous = None
        self.velocity = Gf.Vec3d(0.)
        self.previous_yaw = None
        self.carrying = False

    def lift_to(self, offset, dt):
        goal = offset-LIFT_DATUM
        step = AMR_LIFT_SPEED*dt
        self.joint += max(-step, min(step, goal-self.joint))
        load = (AMR_LIFT_MASS+(BODY_MASS if self.carrying else 0.))*9.81
        self.c.lift_drive.Set(self.joint+load/self.c.stiffness)
        return abs(goal-self.joint) < 1e-6

    def drive_to(self, api, x, dt):
        state = api.get_rigidbody_transformation(self.c.path)
        point = Gf.Vec3d(*state["position"])
        if self.previous is None:
            self.previous = point
        measured = (point-self.previous)/dt
        self.previous = point
        remaining = Gf.Vec3d(x-point[0], AMR_BAY_Y-point[1], 0.)
        speed = min(AMR_SPEED_Y, sqrt(2*AMR_ACCELERATION*remaining.GetLength()))
        goal = remaining.GetNormalized()*speed if remaining.GetLength() > 1e-6 else Gf.Vec3d(0.)
        delta = goal-self.velocity
        if delta.GetLength() > AMR_ACCELERATION*dt:
            delta = delta.GetNormalized()*(AMR_ACCELERATION*dt)
        self.velocity += delta
        mass = AMR_MASS+AMR_LIFT_MASS+(BODY_MASS if self.carrying else 0.)
        force = (self.velocity-Gf.Vec3d(measured[0], measured[1], 0.))*(mass*6.)
        limit = AMR_TRACTION_COEFFICIENT*mass*9.81
        if force.GetLength() > limit:
            force = force.GetNormalized()*limit
        self.c.force.Set(vec(force) if self.c.grounded else Gf.Vec3f(0.))
        front = Gf.Rotation(quaternion(state)).TransformDir(Gf.Vec3d(1, 0, 0))
        yaw = atan2(front[1], front[0])+pi/2            # heading -90 deg along the bay
        yaw = (yaw+pi) % (2*pi)-pi
        rate = 0. if self.previous_yaw is None else (yaw-self.previous_yaw)/dt
        self.previous_yaw = yaw
        torque = -3600*yaw-900*rate
        bound = limit*.31
        self.c.yaw_torque.Set(Gf.Vec3f(0., 0., max(-bound, min(bound, torque)) if self.c.grounded else 0.))
        return remaining.GetLength() < .006 and Gf.Vec3d(measured[0], measured[1], 0.).GetLength() < .025

    def step(self, api, dt):
        self.timer += dt
        if self.phase == "lower":
            self.c.set_locators(False)
            self.drive_to(api, AISLE_X, dt)
            if self.lift_to(self.entry, dt) and self.timer > .5:
                self.phase, self.timer = "enter", 0.
        elif self.phase == "enter":
            self.lift_to(self.entry, dt)
            if self.drive_to(api, self.slot_x, dt) and self.timer > 1.:
                self.phase, self.timer = "lift", 0.
        elif self.phase == "lift":
            self.drive_to(api, self.slot_x, dt)
            self.carrying = True
            if self.lift_to(self.carry, dt) and self.timer > .5:
                self.c.set_locators(True)     # up beside the sills and body ends: the car cannot slide
                self.phase, self.timer = "exit", 0.
        elif self.phase == "exit":
            self.lift_to(self.carry, dt)
            if self.drive_to(api, AISLE_X, dt) and self.timer > 1.:
                self.phase, self.timer = "done", 0.
        else:
            self.lift_to(self.carry, dt)
            self.drive_to(api, AISLE_X, dt)
        return self.phase

