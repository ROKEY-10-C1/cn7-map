"""Pick zone at the end of each station conveyor (method 1 environment).

The tire arrives lying flat, outboard face up. Equipment, as on a tire-handling line:
  - (escapement for the next tire belongs on the feed line upstream; one tire per station here)
  - photo-eye S1 across the zone entry: zone rollers slow from 0.32 to 0.08 m/s
  - V-stop (UHMW plates, 120 deg, sprung): fixes the tire centre along the flow for any diameter
  - photo-eye S2 aimed into the V: tire in place -> zone rollers stop
  - centering arms with tip rollers: close together to centre the tire across the flow, then open
  - overhead camera above the pick point (ID, bolt-hole angle, defect check)

Photo-eyes are PhysX ray casts (a beam that is blocked or not), not simulator ground truth.
The zone logic below is the conveyor PLC; robot and gripper control are out of scope here.
"""
from math import cos, radians, sin, sqrt

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from conveyor import block, collider, powered_roller
from physical_data import ROLLER_RADIUS, box_inertia
from process import BELT_Z, CN7_R16, LINE_Y
from sim_util import body, constrain, drive, mass_properties, rotate

V_HALF_ANGLE = 60.          # deg between each V plate and the flow axis (120 deg opening)
V_SPRING = 40000.           # N/m buffer behind the V plates: ~2 mm under the slow-zone roller drag (estimated)
SLOW_SPEED = .08            # m/s after S1
LINE_SPEED = .32            # m/s nominal zone roller surface speed
ARM_FORCE = 150.            # N per centering arm (pneumatic, estimated)
ARM_STROKE = .10            # m of slide travel each side
ARM_OPEN = .07              # m between the open tip rollers and the tread
DROP = .30                  # m the centering arms drop below the conveyor when parked
LIFT_SPEED = .10            # m/s lift table speed
TIRE_TOP = BELT_Z + CN7_R16.width   # flat tire: tread width is its height


def _visual_cyl(stage, path, centre, axis, radius, height, colour):
    shape = UsdGeom.Cylinder.Define(stage, path)
    shape.CreateAxisAttr(axis)
    shape.CreateRadiusAttr(radius)
    shape.CreateHeightAttr(height)
    shape.AddTranslateOp().Set(Gf.Vec3d(*centre))
    shape.CreateDisplayColorAttr([colour])
    return shape


class PhotoEye:
    """Through-beam / diffuse photo-eye: a PhysX ray from the emitter; blocked when it hits
    anything within range other than the listed background prims."""

    def __init__(self, stage, path, origin, direction, length, ignore=()):
        self.origin = Gf.Vec3d(*origin)
        self.direction = Gf.Vec3d(*direction).GetNormalized()
        self.length = length
        self.ignore = tuple(ignore)
        housing = UsdGeom.Cube.Define(stage, path)
        housing.CreateSizeAttr(1.)
        housing.AddTranslateOp().Set(self.origin - self.direction*.03)
        housing.AddScaleOp().Set(Gf.Vec3d(.04, .04, .04))
        housing.CreateDisplayColorAttr([(.95, .75, .10)])
        beam = UsdGeom.BasisCurves.Define(stage, path+"_Beam")
        beam.CreateTypeAttr("linear")
        beam.CreatePointsAttr([Gf.Vec3f(self.origin), Gf.Vec3f(self.origin+self.direction*length)])
        beam.CreateCurveVertexCountsAttr([2])
        beam.CreateWidthsAttr([.002])
        beam.CreateDisplayColorAttr([(1., .2, .1)])
        self.blocked = False

    def read(self):
        from omni.physx import get_physx_scene_query_interface
        hit = get_physx_scene_query_interface().raycast_closest(
            tuple(self.origin), tuple(self.direction), self.length)
        what = str(hit.get("rigidBody") or hit.get("collision") or "")
        self.blocked = bool(hit.get("hit")) and not what.startswith(self.ignore)
        return self.blocked


class PickZone:
    def __init__(self, stage, root, axle, side, roller_material, guide_material):
        self.root = root
        self.side = side
        self.axle = axle
        self.state = "arriving"
        self.timer = 0.
        y0 = side*LINE_Y
        # Support frame, bed and the zone's own powered rollers (as before), motors kept for zone control.
        frame_bottom = BELT_Z-.087-.032
        for i, dx in enumerate((-.236, .236)):
            for j, dy in enumerate((-.226, .226)):
                block(stage, root+f"/Support_{i}_{j}", (axle+dx, y0+dy, frame_bottom/2), (.071, .071, frame_bottom))
        # Bed plate with slots where the lift bars and the pop-up stopper pass (as on the real
        # machine; a solid plate would have them cut through it).
        cuts = ((-.2425, -.1925), (.1925, .2425))   # 25 mm each side of the lift bars
        edges = [-.33] + [v for c in cuts for v in c] + [.33]
        for k in range(0, len(edges), 2):
            x0, x1 = edges[k], edges[k+1]
            block(stage, root+f"/TransferBed/Seg_{k//2}", (axle+(x0+x1)/2, y0, BELT_Z-.087), (x1-x0, .654, .064))
        cache = UsdGeom.XformCache()
        self.motors = []
        for i in range(7):
            path = root+f"/TransferRoller_{i}"
            roll = UsdGeom.Cylinder.Define(stage, path)
            roll.CreateAxisAttr("Y")
            roll.CreateRadiusAttr(ROLLER_RADIUS)
            roll.CreateHeightAttr(.58)
            roll.AddTranslateOp().Set(Gf.Vec3d(axle+(i-3)*.087, y0, BELT_Z-ROLLER_RADIUS))
            roll.CreateDisplayColorAttr([(.36, .40, .42)])
            self.motors.append(powered_roller(roll, path+"_Axle", -LINE_SPEED, roller_material, cache))
        self.rate_per_speed = self.motors[0].GetTargetVelocityAttr().Get()/-LINE_SPEED

        # V-stop: two UHMW plates on a sprung carriage. A circle of radius R in a V of half angle
        # a sits with its centre R/sin(a) from the apex, so the apex goes there before the axle.
        R = CN7_R16.radius
        apex_x = axle - R/sin(radians(V_HALF_ANGLE))
        z = BELT_Z + .10           # mid-height of the flat tire: full tread radius there
        self.vstop = body(stage, root+"/VStop", (apex_x, y0, z), None, 6.)
        mass_properties(self.vstop, (0., 0., 0.), box_inertia(6., (.12, .5, .12)))
        for k, sgn in enumerate((-1, 1)):
            plate = UsdGeom.Cube.Define(stage, root+f"/VStop/Plate_{k}")
            plate.CreateSizeAttr(1.)
            # Plate centre on the V leg, 0.17 m from the apex; leg direction (cos a, sgn*sin a).
            a = radians(V_HALF_ANGLE)
            plate.AddTranslateOp().Set(Gf.Vec3d(.17*cos(a)-.012*sin(a), sgn*(.17*sin(a)+.012*cos(a)), 0.))
            plate.AddRotateZOp().Set(sgn*V_HALF_ANGLE)
            plate.AddScaleOp().Set(Gf.Vec3d(.36, .024, .10))
            plate.CreateDisplayColorAttr([(.92, .92, .88)])
            collider(plate.GetPrim(), guide_material)
        buffer = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+"/VStopBuffer"), None, self.vstop,
                           (apex_x, y0, z))
        buffer.CreateAxisAttr("X")
        buffer.CreateLowerLimitAttr(-.015)
        buffer.CreateUpperLimitAttr(0.)
        drive(buffer, "linear", 0., V_SPRING, 2*sqrt(V_SPRING*6.), 400.)

        # Photo-eyes. S1 across the entry, S2 diffuse into the V just before the apex.
        ignore = (root+"/TransferRoller", root+"/TransferBed", root+"/VStop", root+"/Centering")
        s1_x = axle + .38
        # Emitter and reflector sit on the inner faces of the side guides (y0 +-0.3315 m, top
        # 0.60 m): a beam from outside them was always blocked by the guide itself.
        self.s1 = PhotoEye(stage, root+"/PhotoEyeS1", (s1_x, y0-side*.325, BELT_Z+.08), (0, side, 0), .65, ignore)
        # S2 sees the tire only once it sits in the V: range = seated tread distance + 3 mm.
        seated = R/sin(radians(V_HALF_ANGLE)) - R
        self.s2 = PhotoEye(stage, root+"/PhotoEyeS2", (apex_x+.01, y0, BELT_Z+.10), (1, 0, 0),
                           seated-.01+.003, ignore)

        # Centering arms: one each side across the flow, tip roller on the tread, pneumatic. They
        # are drop-away arms: up beside the tread only while centering, otherwise parked 0.30 m
        # lower under the conveyor - the car-side arm would stand in the wheel's insertion path
        # and the outboard one in the robot pedestal's path along the rail.
        self.arms = []
        self.risers = []
        for k, sgn in enumerate((-1, 1)):
            y = y0 + sgn*(R+.02+ARM_OPEN)
            carriage = body(stage, root+f"/CenteringLift_{k}", (axle, y, BELT_Z+.10), None, 2.)
            mass_properties(carriage, (0., 0., 0.), box_inertia(2., (.06, .06, .3)))
            rod = UsdGeom.Cube.Define(stage, root+f"/CenteringLift_{k}/Rod")
            rod.CreateSizeAttr(1.)
            rod.AddTranslateOp().Set(Gf.Vec3d(0., sgn*.05, -.17))
            rod.AddScaleOp().Set(Gf.Vec3d(.03, .03, .30))
            rod.CreateDisplayColorAttr([(.55, .58, .60)])
            riser = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+f"/CenteringLift_{k}Joint"), None, carriage,
                              (axle, y, BELT_Z+.10))
            riser.CreateAxisAttr("Z")
            riser.CreateLowerLimitAttr(-DROP)
            riser.CreateUpperLimitAttr(0.)
            self.risers.append(drive(riser, "linear", -DROP, 40000., 800., 600.))
            arm = body(stage, root+f"/Centering_{k}", (axle, y, BELT_Z+.10), None, 3.)
            mass_properties(arm, (0., 0., 0.), box_inertia(3., (.30, .06, .06)))
            beam = UsdGeom.Cube.Define(stage, root+f"/Centering_{k}/Beam")
            beam.CreateSizeAttr(1.)
            beam.AddTranslateOp().Set(Gf.Vec3d(0., sgn*.03, 0.))
            beam.AddScaleOp().Set(Gf.Vec3d(.30, .04, .05))
            beam.CreateDisplayColorAttr([(.20, .45, .55)])
            for j, dx in enumerate((-.10, .10)):
                tip = UsdGeom.Capsule.Define(stage, root+f"/Centering_{k}/Roller_{j}")
                tip.CreateAxisAttr("Z")
                tip.CreateRadiusAttr(.02)
                tip.CreateHeightAttr(.06)
                tip.AddTranslateOp().Set(Gf.Vec3d(dx, 0., 0.))
                tip.CreateDisplayColorAttr([(.08, .08, .08)])
                collider(tip.GetPrim(), guide_material)
            slide = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+f"/Centering_{k}Slide"), carriage, arm)
            slide.CreateAxisAttr("Y")
            # +travel moves the arm toward the conveyor centre for both arms.
            slide.CreateLowerLimitAttr(-ARM_STROKE if sgn > 0 else 0.)
            slide.CreateUpperLimitAttr(0. if sgn > 0 else ARM_STROKE)
            target = drive(slide, "linear", 0., ARM_FORCE/.02, 300., ARM_FORCE)
            self.arms.append((target, -sgn))

        # Supports: nothing floats. Stand for the V-stop and its buffer, cylinder housings for the
        # pop-up stopper and the centering arms, brackets for the photo-eyes (static colliders).
        grey, dark = (.30, .33, .35), (.18, .20, .22)
        block(stage, root+"/VStopStand", (apex_x-.07, y0, (z+.05)/2), (.06, .14, z+.05))
        _visual_cyl(stage, root+"/VStopBufferCyl", (apex_x-.035, y0, z), "X", .022, .07, dark)
        for k, sgn in enumerate((-1, 1)):
            # Lift cylinder under the arm's parked position, below the conveyor level.
            block(stage, root+f"/CenteringCylinder_{k}", (axle, y0+sgn*(R+.02+ARM_OPEN+.05), .12), (.07, .07, .20))
        block(stage, root+"/PhotoEyeS1Post", (s1_x, y0-side*.39, (BELT_Z+.10)/2), (.03, .03, BELT_Z+.10))

        # Lift table: two bars in the roller gaps under the tire's lower sidewall (a flat tire rests
        # on its sidewall ring, radius 0.22-0.32 m) lift it 55 mm, so the gripper's roller
        # flanges pass under the tread without touching the conveyor.
        self.lift_bars = []
        for k, dx in enumerate((-.2175, .2175)):
            bar = body(stage, root+f"/LiftBar_{k}", (axle+dx, y0, BELT_Z-.035), None, 1.5)
            # Bars run across the flow (y) and support the sidewall over +-0.20 m.
            mass_properties(bar, (0., 0., 0.), box_inertia(1.5, (.026, .40, .05)))
            shape = UsdGeom.Cube.Define(stage, root+f"/LiftBar_{k}/Pad")
            shape.CreateSizeAttr(1.)
            shape.AddScaleOp().Set(Gf.Vec3d(.026, .40, .05))
            shape.CreateDisplayColorAttr([(.20, .45, .55)])
            collider(shape.GetPrim(), guide_material)
            joint = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+f"/LiftBar_{k}Joint"), None, bar,
                              (axle+dx, y0, BELT_Z-.035))
            joint.CreateAxisAttr("Z")
            joint.CreateLowerLimitAttr(0.)
            joint.CreateUpperLimitAttr(.065)
            self.lift_bars.append(drive(joint, "linear", 0., 60000., 1500., 800.))
        self.contact_bodies = [str(self.vstop.GetPath())] + \
            [root+f"/Centering_{k}" for k in range(2)] + [root+f"/CenteringLift_{k}" for k in range(2)] + \
            [root+f"/LiftBar_{k}" for k in range(2)]
        for path in self.contact_bodies:
            PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)

        # Overhead camera on a gantry above the pick point, looking straight down.
        cam = UsdGeom.Camera.Define(stage, root+"/PickCamera")
        cam.AddTranslateOp().Set(Gf.Vec3d(axle, y0, BELT_Z+.95))
        cam.CreateFocalLengthAttr(1.93)       # D455 RGB: 1.93 mm lens
        cam.CreateHorizontalApertureAttr(3.9)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.1, 5.))
        # Mast downstream of the V-stop on the line centre, clear of the robot rail and the car side.
        post = UsdGeom.Cube.Define(stage, root+"/PickCameraMast")
        post.CreateSizeAttr(1.)
        post.AddTranslateOp().Set(Gf.Vec3d(axle-.55, y0, (BELT_Z+1.0)/2))
        post.AddScaleOp().Set(Gf.Vec3d(.05, .05, BELT_Z+1.0))
        post.CreateDisplayColorAttr([(.30, .33, .35)])
        bracket = UsdGeom.Cube.Define(stage, root+"/PickCameraArm")
        bracket.CreateSizeAttr(1.)
        bracket.AddTranslateOp().Set(Gf.Vec3d(axle-.275, y0, BELT_Z+1.0))
        bracket.AddScaleOp().Set(Gf.Vec3d(.55, .04, .04))
        bracket.CreateDisplayColorAttr([(.30, .33, .35)])
        self.stopper_path = str(self.vstop.GetPath())

    def set_speed(self, speed):
        for motor in self.motors:
            motor.GetTargetVelocityAttr().Set(-speed*self.rate_per_speed)

    def centering(self, closed):
        # Target past the tread: both arms push with the same limited force, so the tire ends
        # up centred between them whatever its diameter.
        for target, inward in self.arms:
            target.Set(inward*.095 if closed else 0.)

    def raise_arms(self, up):
        for target in self.risers:
            target.Set(0. if up else -DROP)

    def lift(self, up):
        # Goal only: step() ramps the bars at LIFT_SPEED, as a speed-controlled lift table.
        self.lift_goal = .065 if up else 0.     # bar top 30 mm above the roller tops when up

    def step(self, dt):
        """Zone PLC: S1 slow, S2 stop, centre, open, ready for pick."""
        self.timer += dt
        goal = getattr(self, "lift_goal", 0.)
        now = getattr(self, "lift_now", 0.)
        if now != goal:
            now += max(-LIFT_SPEED*dt, min(LIFT_SPEED*dt, goal-now))
            self.lift_now = now
            for target in self.lift_bars:
                target.Set(now)
        s1, s2 = self.s1.read(), self.s2.read()
        if self.state == "arriving" and s1:
            self.set_speed(SLOW_SPEED)
            self.state, self.timer = "slowing", 0.
        elif self.state == "slowing" and s2:
            self.state, self.timer = "settling", 0.
        elif self.state == "settling" and self.timer > .30:
            # Slow rollers keep it seated in the V for 0.3 s, then stop so it is not rubbed in place.
            self.set_speed(0.)
            self.raise_arms(True)
            self.state, self.timer = "arms_up", 0.
        elif self.state == "arms_up" and self.timer > .6:
            self.centering(True)
            self.state, self.timer = "centering", 0.
        elif self.state == "centering" and self.timer > 1.0:
            # Lift with the arms still closed so the tire cannot slide off centre on the bars.
            self.lift(True)
            self.state, self.timer = "lifting", 0.
        elif self.state == "lifting" and self.timer > .9:
            self.centering(False)
            self.state, self.timer = "opening", 0.
        elif self.state == "opening" and self.timer > .6:
            self.raise_arms(False)
            self.state, self.timer = "arms_down", 0.
        elif self.state == "arms_down" and self.timer > .7:
            self.state = "pick_ready"
        elif self.state in ("arriving", "slowing") and self.timer > 10.:
            self.state = "feed_failed"
        return self.state
