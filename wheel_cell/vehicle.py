"""CN7 body underside collision, vehicle buffer stands, and the AMR buffer pick-up.

The body mesh (/AssetLibrary/Body) is an outer shell: its lowest parts are the side sills and the
bumper corners (car y +-0.65..0.91 m, z from 1.01 m); the floor pan is not modelled and stays the
Undertray box, the wheel openings stay the Wheelhouse arch proxies. The buffer stands carry the parked car at the sill ends (car x +-2.05 m), so the
sills are cut from the mesh into collision blocks: 0.1 m slices along the car, two bands across.
Slice edges fall on the stand edges, so a block never reaches below the mesh over a stand.
"""
from math import atan2, degrees, pi, radians, sqrt

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from conveyor import collider, quaternion
from physical_data import AMR_ACCELERATION, AMR_LIFT_MASS, AMR_LIFT_SPEED, AMR_MASS, AMR_SPEED_Y
from physical_data import AMR_TRACTION_COEFFICIENT, BODY_MASS
from fitment import ARCH_RADIUS
from process import AMR_BAY_Y, AMR_GATE_X, AMR_LIFT_CARRY, AMR_LIFT_ENTRY, STATIONS
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
    """AMR + hanger fetch the order's car from an inbound buffer slot and bring it to the gate:
    lower to the entry height with the locating blocks down, crab in under the parked car, lift it
    off the stands to the carry height (blocks up), crab back to the lane, turn from -90 to 0 deg at
    the lane end, crab to the gate and set the car down to the gate height. The cell's Carrier
    (gate -> dock) takes over from there.

    Uses the cell's Carrier bodies and drives with its own targets. Heights are hanger offsets
    from process.py (0 = docked). Contacts of the car or the hanger with the stands, other than the
    car resting on its own stands before the lift and the pads carrying it after, are recorded in
    `touches`."""

    TURN_RATE = 15.          # deg/s, in place at the lane end

    def __init__(self, carrier, slot_x, stands, entry=AMR_LIFT_ENTRY, carry=AMR_LIFT_CARRY):
        self.c = carrier
        self.slot_x = slot_x
        self.own_stands = stands
        self.entry = entry
        self.carry = carry
        self.phase = "lower"
        self.timer = 0.
        self.joint = 0.
        self.previous = None
        self.velocity = Gf.Vec3d(0.)
        self.previous_yaw = None
        self.heading_goal = -90.
        self.carrying = False
        self.touches = {}
        self.lift_off = None

    def lift_to(self, offset, dt):
        goal = offset-LIFT_DATUM
        step = AMR_LIFT_SPEED*dt
        self.joint += max(-step, min(step, goal-self.joint))
        load = (AMR_LIFT_MASS+(BODY_MASS if self.carrying else 0.))*9.81
        self.c.lift_drive.Set(self.joint+load/self.c.stiffness)
        return abs(goal-self.joint) < 1e-6

    def drive_to(self, api, x, y, dt):
        """Planar traction toward (x, y) at crab speed, heading held at heading_goal."""
        state = api.get_rigidbody_transformation(self.c.path)
        point = Gf.Vec3d(*state["position"])
        if self.previous is None:
            self.previous = point
        measured = (point-self.previous)/dt
        self.previous = point
        remaining = Gf.Vec3d(x-point[0], y-point[1], 0.)
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
        error = (atan2(front[1], front[0])-radians(self.heading_goal)+pi) % (2*pi)-pi
        rate = 0. if self.previous_yaw is None else (error-self.previous_yaw)/dt
        self.previous_yaw = error
        torque = -3600*error-900*rate
        bound = limit*.31
        self.c.yaw_torque.Set(Gf.Vec3f(0., 0., max(-bound, min(bound, torque)) if self.c.grounded else 0.))
        self.heading_error = degrees(error)
        return remaining.GetLength() < .006 and Gf.Vec3d(measured[0], measured[1], 0.).GetLength() < .025

    def step(self, api, dt):
        self.timer += dt
        lane = (AISLE_X, AMR_BAY_Y)
        if self.phase == "lower":
            self.c.set_locators(False)
            self.drive_to(api, *lane, dt)
            if self.lift_to(self.entry, dt) and self.timer > .5:
                self.phase, self.timer = "enter", 0.
        elif self.phase == "enter":
            self.lift_to(self.entry, dt)
            if self.drive_to(api, self.slot_x, AMR_BAY_Y, dt) and self.timer > 1.:
                self.phase, self.timer = "lift", 0.
        elif self.phase == "lift":
            self.drive_to(api, self.slot_x, AMR_BAY_Y, dt)
            self.carrying = True
            if self.lift_to(self.carry, dt) and self.timer > .5:
                self.c.set_locators(True)     # up beside the sills and body ends: the car cannot slide
                self.phase, self.timer = "exit", 0.
        elif self.phase == "exit":
            self.lift_to(self.carry, dt)
            if self.drive_to(api, *lane, dt) and self.timer > 1.:
                self.phase, self.timer = "turn", 0.
        elif self.phase == "turn":
            self.lift_to(self.carry, dt)
            self.heading_goal = min(0., self.heading_goal+self.TURN_RATE*dt)
            self.drive_to(api, *lane, dt)
            if self.heading_goal == 0. and abs(self.heading_error) < .3 and self.timer > 1.:
                self.phase, self.timer = "to_gate", 0.
        elif self.phase == "to_gate":
            self.lift_to(self.carry, dt)
            if self.drive_to(api, AMR_GATE_X, 0., dt) and self.timer > 1.:
                self.phase, self.timer = "set_down", 0.
        elif self.phase == "set_down":
            # Down to the gate height the dock approach starts from (car at -0.12).
            self.drive_to(api, AMR_GATE_X, 0., dt)
            if self.lift_to(LIFT_DATUM, dt) and self.timer > .5:
                self.phase, self.timer = "done", 0.
        return self.phase

    def observe(self, car_path, now, car_z, parked_z, pairs):
        """pairs: (collider, other, penetrating) for this step's contact reports."""
        if self.lift_off is None and car_z > parked_z+.005:
            self.lift_off = round(now, 2)
        hanger = (self.c.path+"/", self.c.lift_path+"/", self.c.locator_root+"/")
        for a, b, deep in pairs:
            if not deep:
                continue
            for x, y in ((a, b), (b, a)):
                stand = "/Stand_" in y and "VehicleBuffer" in y
                if x.startswith(car_path+"/") and (stand or y.startswith(hanger)):
                    intended = (y.startswith(self.own_stands) and self.phase in ("lower", "enter", "lift")) or (
                        "/LoadPad_" in y and self.phase not in ("lower", "enter"))
                    if not intended:
                        key = (self.phase, x.rsplit("/", 1)[1], "/".join(y.rsplit("/", 2)[-2:]))
                        self.touches[key] = self.touches.get(key, 0)+1
                elif x.startswith(hanger) and stand:
                    key = (self.phase, x.rsplit("/", 1)[1], "/".join(y.rsplit("/", 2)[-2:]))
                    self.touches[key] = self.touches.get(key, 0)+1


def place_at_buffer(stage, cell, slot):
    """Before the simulation starts: the order's car (with its knuckles and hubs) takes the parked
    car's place on inbound slot `slot`, slots nearer the lane are empty (fetched earlier), and the
    AMR + hanger wait in the lane heading along the bay. Returns (stand prefix, parked car z)."""
    buffers = [p for p in stage.Traverse() if p.GetName() == "Vehicle_0" and "VehicleBuffer" in str(p.GetPath())]
    inbound = next(p.GetParent() for p in buffers
                   if UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[1] > 0)
    root = str(inbound.GetPath())
    cache = UsdGeom.XformCache()
    slot_pose = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{root}/Vehicle_{slot}"))
    for k in range(slot+1):
        stage.GetPrimAtPath(f"{root}/Vehicle_{k}").SetActive(False)
    car = stage.GetPrimAtPath(cell.car_path)
    corners = [s.hub_path.rsplit("/", 1)[0]+part for s in cell.stations for part in ("/Knuckle", "/Hub")]
    move_rigidly(stage, [cell.car_path]+corners, cache.GetLocalToWorldTransform(car).GetInverse()*slot_pose)
    deck = stage.GetPrimAtPath(cell.carrier.path)
    start = cache.GetLocalToWorldTransform(deck)
    lane = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.))
    lane.SetTranslateOnly(Gf.Vec3d(AISLE_X, AMR_BAY_Y, start.ExtractTranslation()[2]))
    locators = [str(p.GetPath()) for p in stage.GetPrimAtPath(cell.carrier.locator_root).GetChildren()
                if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    move_rigidly(stage, [cell.carrier.path, cell.carrier.lift_path]+locators, start.GetInverse()*lane)
    # A machine waiting at rest must not fall asleep: a sleeping deck stops reporting the floor
    # contact the traction needs, and nothing would wake it.
    for path in [cell.car_path, cell.carrier.path, cell.carrier.lift_path]+locators:
        PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateSleepThresholdAttr(0.)
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(cell.car_path)).CreateThresholdAttr(0.)
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(cell.carrier.lift_path)).CreateThresholdAttr(0.)
    return f"{root}/Stand_{slot}_", slot_pose.ExtractTranslation()[2]
