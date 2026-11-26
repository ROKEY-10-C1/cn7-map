"""Vehicle corner and wheel models: hub on its bearing, suspension, dimensional tolerance,
rigid-ring tire, and the camera measurement model that replaces simulator ground truth.

Parameters come from cell_config.CONFIG; estimates are listed in calibration.json.
"""
from math import cos, degrees, sin, sqrt, tau
from random import Random

from pxr import Gf, PhysxSchema, UsdPhysics

from cell_config import CONFIG
from fitment import BOLT_CENTERS, FLANGE_RADIUS, OFFSET, PCD, PILOT, PILOT_LENGTH, STUD_DIAMETER, STUD_PROJECTION
from physical_data import TIRE_MASS, annulus_inertia, box_inertia
from process import CN7_R16, HUB_Z

PILOT_CHAMFER = (.0015, .0010)   # length, radial: 1.5 mm x ~34 deg lead-in on the pilot (drawing practice)
STUD_CHAMFER = (.0020, .0015)    # dog point on the stud end
HUB_MASS = 6.5       # hub, rotor hat and bearing inner race (estimated)
KNUCKLE_MASS = 11.   # knuckle, caliper bracket, half of the control arm (estimated)
HUB_CONTACT_OFFSET = .0008   # > 0.32 mm press travel per step at 76 mm/s, 240 Hz


def _helpers():
    # physics.py defines the shared USD helpers and imports this module lazily.
    import physics
    return physics


def lock(joint, dofs):
    for dof in dofs:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), dof)
        limit.CreateLowAttr(1.)
        limit.CreateHighAttr(-1.)


def travel(joint, dof, low, high):
    limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), dof)
    limit.CreateLowAttr(low)
    limit.CreateHighAttr(high)


class Tolerance:
    """Per-seed hub dimensions inside the drawing tolerances."""

    def __init__(self, rng):
        cfg = CONFIG["tolerance"]
        low, high = cfg["pilot_diameter_mm"]
        self.pilot = PILOT + rng.uniform(low, high)/1000
        pcd = PCD + rng.gauss(0., cfg["pcd_mm"]/2000)
        self.studs = []
        for x, y in BOLT_CENTERS:
            a = rng.uniform(0, tau)
            r = rng.uniform(0, cfg["stud_position_mm"]/1000)
            scale = pcd/PCD
            self.studs.append((x*scale + r*cos(a), y*scale + r*sin(a)))

    def as_dict(self):
        return {"pilot_mm": round(self.pilot*1000, 4),
                "studs_mm": [(round(x*1000, 3), round(y*1000, 3)) for x, y in self.studs]}


def chamfered_pin(stage, path, xy, z0, length, radius, chamfer, material):
    """Cylinder with a conical lead-in at its free end (+Z)."""
    P = _helpers()
    pin, convex_prism = P.pin, P.convex_prism
    tip, inset = chamfer
    pin(stage, path, xy, z0, length-tip, radius, material)
    points = [Gf.Vec3f(xy[0]+(rr)*cos(i*tau/24), xy[1]+(rr)*sin(i*tau/24), z)
              for z, rr in ((z0+length-tip, radius), (z0+length, radius-inset)) for i in range(24)]
    tip_path = convex_prism(stage, path+"Tip", points, 24, True, material)
    # The press moves the wheel 0.32 mm per step at 76 mm/s. With the precise 0.045 mm contact
    # offset the cone's slanted face was crossed between steps and the wheel tunnelled into the
    # hub (26-76 mm). A larger offset only creates the contacts earlier; the rest offset stays 0.
    PhysxSchema.PhysxCollisionAPI(stage.GetPrimAtPath(tip_path)).GetContactOffsetAttr().Set(HUB_CONTACT_OFFSET)
    return tip_path


def build_corner(stage, car_pose, root, name, axle, side, hub_y, steel, seed):
    """Knuckle on the suspension, hub on its bearing. Returns (hub body path, hub angle, tolerance)."""
    P = _helpers()
    body, constrain, drive, mass_properties, pin, rotate = (P.body, P.constrain, P.drive,
                                                            P.mass_properties, P.pin, P.rotate)
    rng = Random(f"{seed}-{name}-corner")
    hub_cfg, susp = CONFIG["hub"], CONFIG["suspension"]
    angle = rng.uniform(0., 72.)          # the stopped car parks its hubs at any angle
    tolerance = Tolerance(rng)
    local = Gf.Vec3d(axle, hub_y, HUB_Z)
    world = car_pose.Transform(local)
    car = stage.GetPrimAtPath(root+"/CN7")
    knuckle = body(stage, f"{root}/{name}/Knuckle", world, None, KNUCKLE_MASS)
    mass_properties(knuckle, (0., 0., 0.), box_inertia(KNUCKLE_MASS, (.18, .12, .30)))
    spring = constrain(UsdPhysics.Joint.Define(stage, f"{root}/{name}/Suspension"), car, knuckle, local)
    lock(spring, ("transX", "rotX", "rotY", "rotZ"))
    # Lifted car: the corner hangs at full droop, pressed onto the rebound stop by the
    # spring preload. Downward load (the wheel weight on the pilot) is taken by the stop;
    # only an upward push beyond preload + unsprung weight moves the hub.
    rate = susp["wheel_rate_n_per_mm"]*1000
    travel(spring, "transZ", 0., .09)
    drive(spring, "transZ", -susp["droop_preload_mm"]/1000, rate, susp["damping_ns_per_m"], 2e4)
    lateral = susp["lateral_rate_n_per_mm"]*1000
    travel(spring, "transY", -.004, .004)
    drive(spring, "transY", 0., lateral, 2*sqrt(lateral*KNUCKLE_MASS), 2e4)
    # Quaternion product: turn about the hub's own axle by `angle`, then lay the axle along Y.
    hub_q = rotate((1, 0, 0), -side*90).GetQuat()*rotate((0, 0, 1), angle).GetQuat()
    hub = body(stage, f"{root}/{name}/Hub", world, hub_q, HUB_MASS)
    mass_properties(hub, (0., 0., OFFSET-.03), annulus_inertia(HUB_MASS, .02, .14, .06))
    PhysxSchema.PhysxContactReportAPI.Apply(hub).CreateThresholdAttr(0.)
    path = str(hub.GetPath())
    pin(stage, path+"/Face", (0, 0), OFFSET-.012, .012, FLANGE_RADIUS, steel)
    chamfered_pin(stage, path+"/Pilot", (0, 0), OFFSET, PILOT_LENGTH, tolerance.pilot/2, PILOT_CHAMFER, steel)
    for i, center in enumerate(tolerance.studs):
        chamfered_pin(stage, path+f"/Stud_{i}", center, OFFSET, STUD_PROJECTION, STUD_DIAMETER/2, STUD_CHAMFER, steel)
    # A moving hub pressed by the wheel at 145 N lost the precise (0.045 mm) face contact and the
    # web slid through it to the end of its travel. A larger contact offset keeps the contacts alive;
    # the rest offset stays 0, so the seated position is unchanged.
    for part in hub.GetChildren():
        if part.HasAPI(UsdPhysics.CollisionAPI):
            PhysxSchema.PhysxCollisionAPI(part).GetContactOffsetAttr().Set(HUB_CONTACT_OFFSET)
    bearing = constrain(UsdPhysics.Joint.Define(stage, f"{root}/{name}/Bearing"), knuckle, hub, Gf.Vec3d(0.),
                        rot0=rotate((1, 0, 0), -side*90).GetQuat())
    bearing.CreateLocalRot1Attr(Gf.Quatf(rotate((0, 0, 1), -angle).GetQuat()))
    lock(bearing, ("transZ", "rotX", "rotY"))
    play = hub_cfg["radial_play_mm"]/2000
    for dof in ("transX", "transY"):
        travel(bearing, dof, -play, play)
    # Bearing seal and pad drag behaves like Coulomb friction: it holds the parked hub against
    # stray solver torques yet yields to a stud pushing on it. A steep viscous law reaching the
    # drag torque at 0.5 deg/s stands in for it.
    drive(bearing, "rotZ", 0., 0., hub_cfg["bearing_drag_nm"]/.5, 50.)
    return path, angle, tolerance


def ring_tire(stage, wheel_path, ring_path, position, orientation, materials, tread_builder):
    """Rigid-ring tire: tread ring on springs around the rim (radial, lateral, torsional, tilt).

    The rim body keeps the full wheel visual and the mounting-face colliders; the ring carries
    the tread colliders, so grippers and rollers act through the tire's own compliance.
    """
    P = _helpers()
    body, constrain, drive, mass_properties = P.body, P.constrain, P.drive, P.mass_properties
    cfg = CONFIG["tire"]
    ring = body(stage, ring_path, position, orientation, TIRE_MASS)
    mass_properties(ring, (0., 0., 0.), annulus_inertia(TIRE_MASS, CN7_R16.inner, CN7_R16.radius, CN7_R16.width))
    PhysxSchema.PhysxRigidBodyAPI(ring).CreateEnableCCDAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(ring).CreateThresholdAttr(0.)
    tread_builder(ring_path)
    joint = constrain(UsdPhysics.Joint.Define(stage, ring_path+"Mount"), stage.GetPrimAtPath(wheel_path), ring)
    zeta = cfg["damping_ratio"]
    radial = cfg["radial_rate_n_per_mm"]*1000
    lateral = cfg["lateral_rate_n_per_mm"]*1000
    for dof, rate in (("transX", radial), ("transY", radial), ("transZ", lateral)):
        travel(joint, dof, -.012, .012)
        drive(joint, dof, 0., rate, 2*zeta*sqrt(rate*TIRE_MASS), 5e4)
    inertia = annulus_inertia(TIRE_MASS, CN7_R16.inner, CN7_R16.radius, CN7_R16.width)
    torsion = cfg["torsional_rate_nm_per_deg"]
    for dof, rate, moment in (("rotZ", torsion, inertia[2]), ("rotX", torsion*4, inertia[0]), ("rotY", torsion*4, inertia[1])):
        travel(joint, dof, -6., 6.)
        # Angular drive units are per degree; critical damping uses the per-radian equivalent.
        drive(joint, dof, 0., rate, 2*zeta*sqrt(rate*degrees(1.)*moment)/degrees(1.), 5e3)
    return str(ring.GetPath())


class Sensor:
    """Sampled measurement: camera rate, one frame of latency and Gaussian noise."""

    def __init__(self, rng, sigma):
        cfg = CONFIG["sensing"]
        self.rng = rng
        self.sigma = sigma
        self.period = 1./cfg["update_hz"]
        self.latency = cfg["latency_s"]
        self.samples = []
        self.next = 0.

    def update(self, now, truth):
        if now + 1e-9 >= self.next:
            noisy = (tuple(v + self.rng.gauss(0., self.sigma) for v in truth) if isinstance(truth, tuple)
                     else truth + self.rng.gauss(0., self.sigma))
            self.samples.append((now, noisy))
            self.samples = self.samples[-8:]
            self.next = now + self.period

    def value(self, now):
        ready = [v for t, v in self.samples if t <= now - self.latency + 1e-9]
        return ready[-1] if ready else (self.samples[0][1] if self.samples else None)

    def fresh(self, now, since):
        """True once a frame captured after `since` has been processed."""
        return any(since <= t <= now - self.latency + 1e-9 for t, _ in self.samples)
