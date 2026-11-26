"""Method 1 wheel gripper on the mount robot flange (H2515), environment only.

Tool frame = robot Link_6 frame: +Z is the flange normal. Holding a wheel upright, the flange
sits below the tire and +Z points up at the wheel centre; the wheel axle lies along tool X with
the outboard face toward +X.

Parts (dimensions in metres, estimates marked in calibration.json):
  adapter + remote centre compliance block on the flange
  frame: base beam along Y and two uprights to the roller axes
  two grooved drive rollers (dia 66 mm, flanges 30 mm high, 225 mm between flanges) at 5 and
    7 o'clock of the wheel (349 mm from its centre); each a body on a revolute joint with a
    velocity drive (servo) whose high damping at zero speed is the holding brake
  pin arm: hinged at the base, outboard of the tire face; 90 deg fold puts it under the wheel
  centre pin: dia 66.5 mm collet on an axial slide (40 mm stroke) at the top of the arm

Default state: arm folded, pin retracted, rollers braked. Control comes later (ROS 2).
"""
from math import sqrt

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

from conveyor import collider
from physical_data import annulus_inertia, box_inertia
from process import CN7_R16
from sim_util import body, constrain, drive, mass_properties

WHEEL_CENTRE_Z = .512          # wheel centre above the flange when held
ROLLER_RADIUS = .033
ROLLER_Y = .1745               # 349 mm * sin 30 deg
ROLLER_Z = WHEEL_CENTRE_Z - .302   # 349 mm * cos 30 deg below the centre
ROLLER_LENGTH = .225
FLANGE_HEIGHT = .030
ARM_X = .135                   # pin arm plane: tire face (.1025) + 32 mm clearance
ARM_HINGE_Z = .09
PIN_RADIUS = .03325            # collet expanded in the 67.10 mm bore
PIN_TIP_X = .050               # 10 mm into the 14 mm centre web (face at .046-.060)
PIN_STROKE = .040
FRAME_MASS, ROLLER_MASS, ARM_MASS, PIN_MASS = 3.7, 1.2, .8, .6


def _box(stage, path, centre, size, colour, material=None, collide=True):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*centre))
    shape.AddScaleOp().Set(Gf.Vec3d(*size))
    shape.CreateDisplayColorAttr([colour])
    if collide:
        collider(shape.GetPrim(), material)
    return shape


def _cyl(stage, path, centre, axis, radius, height, colour, material=None, collide=True):
    shape = UsdGeom.Cylinder.Define(stage, path)
    shape.CreateAxisAttr(axis)
    shape.CreateRadiusAttr(radius)
    shape.CreateHeightAttr(height)
    shape.AddTranslateOp().Set(Gf.Vec3d(*centre))
    shape.CreateDisplayColorAttr([colour])
    if collide:
        collider(shape.GetPrim(), material)
    return shape


class WheelGripper:
    def __init__(self, stage, robot_path, flange_matrix, roller_material, carrier=False):
        root = robot_path+"/Gripper"
        self.root = root
        rot = flange_matrix.ExtractRotationQuat()
        origin = flange_matrix.ExtractTranslation()
        R = Gf.Rotation(rot)
        world = lambda local: origin + R.TransformDir(Gf.Vec3d(*local))
        steel, alu, blue, rubber = (.55, .58, .60), (.72, .74, .76), (.16, .42, .62), (.07, .07, .08)

        # Frame body, fixed to the flange.
        self.frame = body(stage, root+"/Frame", origin, rot, FRAME_MASS, light=True)
        mass_properties(self.frame, (0., 0., .10), box_inertia(FRAME_MASS, (.12, .42, .20)))
        f = root+"/Frame"
        _cyl(stage, f+"/Adapter", (0, 0, .01), "Z", .05, .02, steel)
        _box(stage, f+"/Compliance", (0, 0, .045), (.10, .10, .05), blue)
        _box(stage, f+"/BaseBeam", (0, 0, ARM_HINGE_Z), (.06, .42, .04), alu)
        for k, sgn in enumerate((-1, 1)):
            _box(stage, f+f"/Upright_{k}", (0, sgn*(ROLLER_Y+.015), (ARM_HINGE_Z+ROLLER_Z)/2),
                 (.05, .03, ROLLER_Z-ARM_HINGE_Z+.02), alu)
            _box(stage, f+f"/Servo_{k}", (-ROLLER_LENGTH/2-.05, sgn*ROLLER_Y, ROLLER_Z), (.08, .06, .06), blue)
        if carrier:
            # Demo: a kinematic body plays the robot flange path; the frame hangs on it through
            # the remote-centre-compliance unit (springs), so contacts stay force-limited.
            self.carrier = body(stage, root+"/FlangeTarget", origin, rot, 1., kinematic=True)
            rcc = constrain(UsdPhysics.Joint.Define(stage, root+"/Compliance"), self.carrier, self.frame)
            for dof, rate in (("transX", 6000.), ("transY", 6000.), ("transZ", 30000.)):
                limit = UsdPhysics.LimitAPI.Apply(rcc.GetPrim(), dof)
                limit.CreateLowAttr(-.004)
                limit.CreateHighAttr(.004)
                drive(rcc, dof, 0., rate, 2*sqrt(rate*12.), 2000.)
            for dof in ("rotX", "rotY", "rotZ"):
                limit = UsdPhysics.LimitAPI.Apply(rcc.GetPrim(), dof)
                limit.CreateLowAttr(-1.)
                limit.CreateHighAttr(1.)
                drive(rcc, dof, 0., 400., 20., 500.)
        else:
            self.carrier = None
            constrain(UsdPhysics.FixedJoint.Define(stage, root+"/FlangeMount"),
                      stage.GetPrimAtPath(robot_path+"/Link_6"), self.frame)
        # The gripper is part of the tool: never collides with the arm that carries it.
        links = [Sdf.Path(robot_path+f"/Link_{k}") for k in (4, 5, 6)]

        # Grooved drive rollers (servo + brake).
        self.rollers, self.roller_drives = [], []
        for k, sgn in enumerate((-1, 1)):
            local = (0., sgn*ROLLER_Y, ROLLER_Z)
            roller = body(stage, root+f"/DriveRoller_{k}", world(local), rot, ROLLER_MASS)
            mass_properties(roller, (0., 0., 0.), annulus_inertia(ROLLER_MASS, .02, ROLLER_RADIUS, ROLLER_LENGTH))
            PhysxSchema.PhysxRigidBodyAPI(roller).CreateAngularDampingAttr(0.)
            r = str(roller.GetPath())
            surface = UsdGeom.Capsule.Define(stage, r+"/Tread")
            surface.CreateAxisAttr("X")
            surface.CreateRadiusAttr(ROLLER_RADIUS)
            surface.CreateHeightAttr(ROLLER_LENGTH-2*ROLLER_RADIUS)
            surface.CreateDisplayColorAttr([rubber])
            collider(surface.GetPrim(), roller_material)
            for j, x in enumerate((-ROLLER_LENGTH/2-.006, ROLLER_LENGTH/2+.006)):
                _cyl(stage, r+f"/Flange_{j}", (x, 0, 0), "X", ROLLER_RADIUS+FLANGE_HEIGHT, .012, steel)
            axle = constrain(UsdPhysics.RevoluteJoint.Define(stage, root+f"/DriveRoller_{k}Axle"),
                             self.frame, roller, local)
            axle.CreateAxisAttr("X")
            motor = UsdPhysics.DriveAPI.Apply(axle.GetPrim(), "angular")
            motor.CreateTypeAttr("force")
            motor.CreateStiffnessAttr(0.)
            motor.CreateDampingAttr(.5)        # N·m per deg/s: holds like a brake at zero target
            motor.CreateMaxForceAttr(12.)      # N·m at the roller (estimated servo + gearbox)
            self.roller_drives.append(motor.CreateTargetVelocityAttr(0.))
            self.rollers.append(r)
            links.append(roller.GetPath())

        # Pin arm on a hinge at the base, outboard of the tire face. 0 deg = up (working),
        # -90 deg = folded flat under the wheel.
        hinge_local = (ARM_X, 0., ARM_HINGE_Z)
        self.arm = body(stage, root+"/PinArm", world(hinge_local), rot, ARM_MASS)
        mass_properties(self.arm, (0., 0., (WHEEL_CENTRE_Z-ARM_HINGE_Z)/2),
                        box_inertia(ARM_MASS, (.03, .04, WHEEL_CENTRE_Z-ARM_HINGE_Z)))
        _box(stage, root+"/PinArm/Bar", (0, 0, (WHEEL_CENTRE_Z-ARM_HINGE_Z)/2),
             (.03, .04, WHEEL_CENTRE_Z-ARM_HINGE_Z+.02), (.80, .45, .12))
        hinge = constrain(UsdPhysics.RevoluteJoint.Define(stage, root+"/PinArmHinge"), self.frame, self.arm,
                          hinge_local)
        hinge.CreateAxisAttr("Y")
        hinge.CreateLowerLimitAttr(0.)
        hinge.CreateUpperLimitAttr(90.)
        # Rotary actuator that locks at both ends of its 90 deg stroke (stiff hold).
        self.fold = drive(hinge, "angular", 90., 2000., 40., 600.)   # folded by default

        # Centre pin on an axial slide at the top of the arm.
        pin_local = (ARM_X, 0., WHEEL_CENTRE_Z)
        self.pin = body(stage, root+"/CentrePin", world(pin_local), rot, PIN_MASS)
        mass_properties(self.pin, (-.04, 0., 0.), (2e-4, 6e-4, 6e-4))
        length = ARM_X - PIN_TIP_X
        # Locating pin with a conical lead-in (10 mm long, radius 30.75 -> 20 mm), as on any
        # locating pin: with the RCC it slides into the 67.1 mm bore from a few mm off centre.
        lead = .010
        _cyl(stage, root+"/CentrePin/Body", (-(length-lead)/2, 0, 0), "X", PIN_RADIUS-.0025, length-lead,
             (.80, .45, .12))
        from math import cos as _c, sin as _s, tau as _tau
        ring = [Gf.Vec3f(x, r*_c(i*_tau/20), r*_s(i*_tau/20))
                for x, r in ((-length, .020), (-length+lead, PIN_RADIUS-.0025)) for i in range(20)]
        from sim_util import convex_prism
        convex_prism(stage, root+"/CentrePin/LeadIn", ring, 20, True, None)
        slide = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+"/CentrePinSlide"), self.arm, self.pin,
                          (0., 0., WHEEL_CENTRE_Z-ARM_HINGE_Z))
        slide.CreateAxisAttr("X")
        slide.CreateLowerLimitAttr(0.)
        slide.CreateUpperLimitAttr(PIN_STROKE)
        # Pneumatic slide held at its end of stroke (hard stop + cylinder force): it keeps the
        # wheel on the rollers between their flanges along the axle.
        self.pin_slide = drive(slide, "linear", PIN_STROKE, 2e5, 2000., 2500.)   # retracted by default
        # Expanding collet: three jaws at the pin tip (inside the 14 mm centre web) driven out
        # radially, as on an expanding mandrel; they hold the flat tire by friction on the bore.
        self.jaws = []
        self.lock = None
        tip = -(length) + .009   # jaw centre 9 mm above the tip: inside the 14 mm web when seated
        rest = PIN_RADIUS - .008  # retracted inside the pin body when idle (5 mm off the bore wall)
        for k in range(3):
            angle = 120.*k
            q = Gf.Rotation(Gf.Vec3d(1, 0, 0), angle)
            jaw_local = Gf.Vec3d(ARM_X+tip, 0., WHEEL_CENTRE_Z) + q.TransformDir(Gf.Vec3d(0, rest, 0))
            jaw = body(stage, root+f"/ColletJaw_{k}", world(jaw_local), rot*q.GetQuat(), .05)
            mass_properties(jaw, (0., 0., 0.), (1e-6, 1e-6, 1e-6))
            _box(stage, root+f"/ColletJaw_{k}/Pad", (0, 0, 0), (.012, .006, .02), (.30, .30, .32), roller_material)
            radial = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+f"/ColletJaw_{k}Slide"), self.pin, jaw,
                               Gf.Vec3d(tip, 0., 0.)+q.TransformDir(Gf.Vec3d(0, rest, 0)), rot0=q.GetQuat())
            radial.CreateAxisAttr("Y")
            radial.CreateLowerLimitAttr(0.)
            radial.CreateUpperLimitAttr(.009)
            self.jaws.append(drive(radial, "linear", 0., 1e5, 300., 300.))   # 300 N per jaw when expanded
            self.jaw_drives = getattr(self, "jaw_drives", []) + [UsdPhysics.DriveAPI(radial.GetPrim(), "linear")]
            links.append(jaw.GetPath())
        links += [self.arm.GetPath(), self.pin.GetPath(), self.frame.GetPath()]
        for part in [self.frame, self.arm, self.pin] + ([self.carrier] if self.carrier else []):
            UsdPhysics.FilteredPairsAPI.Apply(part.GetPrim()).CreateFilteredPairsRel().SetTargets(
                [p for p in links if p != part.GetPath()])
        for r in self.rollers:
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(r)).CreateFilteredPairsRel().SetTargets(
                [p for p in links if str(p) != r])

    # Actuator commands (the interface a ROS 2 driver will use later).
    def unfold_arm(self, up=True):
        self.fold.Set(0. if up else 90.)

    def extend_pin(self, out=True):
        self.pin_slide.Set(0. if out else PIN_STROKE)

    def expand_collet(self, grip=True):
        for target in self.jaws:
            target.Set(.009 if grip else 0.)   # past the bore wall: the force limit sets the grip
        if not grip and self.lock is not None:
            self.lock.GetJointEnabledAttr().Set(False)

    def collet_pressure(self, newtons):
        """Pneumatic pressure on the collet: 300 N per jaw to carry the flat tire, ~60 N to keep
        the wheel on the pin axially while the rollers turn it (a friction brake, not a lock)."""
        if self.lock is not None:
            self.lock.GetJointEnabledAttr().Set(False)
        for d in self.jaw_drives:
            d.GetMaxForceAttr().Set(newtons)
        for target in self.jaws:
            target.Set(.009 if newtons > 0 else 0.)

    def lock_collet(self, stage, api, wheel_path):
        """Fully expanded mandrel = rigid hold. Modelled as a fixed joint pin<->rim at the pose
        reached (friction on three 7 mm jaw strips is not resolved reliably by the contact solver);
        it is released together with the jaws."""
        from conveyor import quaternion
        pin = api.get_rigidbody_transformation(str(self.pin.GetPath()))
        rim = api.get_rigidbody_transformation(wheel_path)
        pr, rr = Gf.Rotation(quaternion(pin)), Gf.Rotation(quaternion(rim))
        inv = pr.GetInverse()
        joint = UsdPhysics.FixedJoint.Define(stage, self.root+"/ColletLock")
        joint.CreateBody0Rel().SetTargets([self.pin.GetPath()])
        joint.CreateBody1Rel().SetTargets([wheel_path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(inv.TransformDir(Gf.Vec3d(*rim["position"])-Gf.Vec3d(*pin["position"]))))
        q = (rr*inv).GetQuat()
        joint.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.))
        joint.CreateJointEnabledAttr(True)
        self.lock = joint

    def roller_speed(self, deg_per_s):
        for target in self.roller_drives:
            target.Set(deg_per_s)
