"""Scene: work table at conveyor height, CN7 front-left corner (hub, brake, wheelhouse), wheel + tire,
Kawasaki RS080N with the SCHUNK PHL-W 63-080-S tread clamp.

Frames
  world      z up. Seated wheel centre H = (0, 0, HUB_Z); the wheel axis points outboard along +y.
  hub local  z = outboard (+y world), x = +x world, y = -z world  (Q_HUB)
  grip G     the wheel frame the gripper is designed in: z = outboard, y = 'up' when mounted,
             shoes at +-45 deg from -y on the tread, jaws travel along x.
  flange F   tool body frame = robot link6 frame: z_F = -z_G (into the tool), x_F = x_G.
"""
from math import cos, pi, radians, sin, tau
from pathlib import Path

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

import params as P
from geom import (annulus_sector, bind, box, circle, cylinder, drive, extrude, joint, limit, lock, look, material,
                  per_deg, prism, rigid, xform)

HERE = Path(__file__).resolve().parent
ROBOT_USD = HERE/"assets/rs080n/rs080n_onrobot_rg2.usd"
WHEEL_USD = HERE.parent/"assets/vehicle/CN7_Wheel_205-55R16.usd"    # shared repo asset (read only)

H = np.array([0., 0., P.HUB_Z])
ROBOT_BASE = np.array([0., 1.85, 0.])
TABLE_C = np.array([1.05, 1.70])
TABLE_SIZE = (.80, .80)
KNUCKLE_MASS = 12.     # estimate, cast knuckle of a C-segment car (listed in PARAMS)

# ----------------------------------------------------------------------------- rotations (numpy, wxyz)


def rot(axis, deg):
    a = np.asarray(axis, float)/np.linalg.norm(axis)
    t = radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3)+sin(t)*K+(1-cos(t))*K@K


def quat(R):
    m = R
    w = np.sqrt(max(0., 1+m[0, 0]+m[1, 1]+m[2, 2]))/2
    x = np.sqrt(max(0., 1+m[0, 0]-m[1, 1]-m[2, 2]))/2
    y = np.sqrt(max(0., 1-m[0, 0]+m[1, 1]-m[2, 2]))/2
    z = np.sqrt(max(0., 1-m[0, 0]-m[1, 1]+m[2, 2]))/2
    x = np.copysign(x, m[2, 1]-m[1, 2])
    y = np.copysign(y, m[0, 2]-m[2, 0])
    z = np.copysign(z, m[1, 0]-m[0, 1])
    return np.array([w, x, y, z])


def gq(R):
    w, x, y, z = quat(R)
    return Gf.Quatd(float(w), float(x), float(y), float(z))


R_HUB = rot((1, 0, 0), -90)                 # hub local -> world
R_MOUNT = R_HUB@rot((0, 0, 1), 180)         # grip frame at the hub: y_G = +z world (shoes at the bottom)

# ----------------------------------------------------------------------------- tool geometry (grip frame)
SHOE_A = P.SHOE_ANGLE                       # from -y_G
CHORD_Y = -P.TIRE_R*cos(SHOE_A)            # jaw line, 223 mm under the wheel centre
TOUCH_X = P.TIRE_R*sin(SHOE_A)
BODY_L, BODY_W, BODY_H = P.GRIP["size"]
BODY_FRONT = P.TIRE_HALF_W+.050             # gripper face 50 mm outboard of the sidewall bulge
JAW_BLOCK = .025                            # base jaw height above the gripper face (estimate)
ADAPTER_T = .020
FLANGE_Z = BODY_FRONT+BODY_H+ADAPTER_T      # flange in G
S_TOUCH = .025                              # jaw opening (per side) at tread contact; 25 mm reserve to close
JAW_Q_RANGE = (-S_TOUCH, P.GRIP["stroke"]-S_TOUCH)
SHOE = dict(r_in=P.TIRE_R, pu=.010, backing=.008, z0=-.070, z1=.085, lip=(.106, .114), lip_r0=.290)
R_FG = np.diag([1., -1., -1.])              # flange axes in G
T_GF = np.array([0., CHORD_Y, FLANGE_Z])    # flange origin in G


def configure(shoe_deg=None):
    """Re-derive the grip geometry for another shoe angle (scenario replays of earlier designs)."""
    global SHOE_A, CHORD_Y, TOUCH_X, T_GF
    SHOE_A = radians(shoe_deg) if shoe_deg is not None else P.SHOE_ANGLE
    CHORD_Y = -P.TIRE_R*cos(SHOE_A)
    TOUCH_X = P.TIRE_R*sin(SHOE_A)
    T_GF = np.array([0., CHORD_Y, FLANGE_Z])


def g2f(p):
    """Point in grip frame -> flange frame."""
    return R_FG.T@(np.asarray(p, float)-T_GF)


def flange_pose(R_G, o_G):
    """World pose of link6 for a world pose of the grip frame (R_G, origin o_G = wheel centre)."""
    return R_G@R_FG, o_G+R_G@T_GF


# ----------------------------------------------------------------------------- build


def build(stage, seed_rng, mu_shoe=None, mu_hub=None, sync=True):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateTimeStepsPerSecondAttr(P.PHYSICS_HZ)
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(False)
    px.CreateBroadphaseTypeAttr("MBP")
    px.CreateEnableGPUDynamicsAttr(False)

    mu_s = mu_shoe or P.MU_SHOE_TREAD
    M = dict(rubber=material(stage, "/World/Mat/Rubber", 1.5, 1.5),
             pu=material(stage, "/World/Mat/PU", *mu_s),
             steel=material(stage, "/World/Mat/Steel", 1.5, 1.5),
             table=material(stage, "/World/Mat/TableSteel", *P.MU_TREAD_TABLE),
             alloy=material(stage, "/World/Mat/Alloy", *(mu_hub or P.MU_ALU_STEEL)),
             liner=material(stage, "/World/Mat/Liner", .5, .4))
    V = dict(steel=look(stage, "/World/Look/Steel", (.55, .56, .58), .35, .9),
             dark=look(stage, "/World/Look/Dark", (.12, .12, .13), .6, .3),
             table=look(stage, "/World/Look/Table", (.30, .33, .36), .7, .2),
             pu=look(stage, "/World/Look/PU", (.90, .55, .10), .7),
             alu=look(stage, "/World/Look/Alu", (.75, .76, .78), .35, .8),
             blue=look(stage, "/World/Look/Schunk", (.10, .30, .62), .45, .2),
             liner=look(stage, "/World/Look/Liner", (.06, .06, .06), .9),
             caliper=look(stage, "/World/Look/Caliper", (.55, .10, .10), .5, .3),
             floor=look(stage, "/World/Look/Floor", (.42, .42, .40), .9))

    # floor, light, table
    ground = box(stage, "/World/Floor", (0, 0, -.025), (12, 12, .05), phys=M["table"], vis=V["floor"])
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(900)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500)
    xform(stage, "/World/Sun", (0, 0, 0), gq(rot((1, .4, 0), -50)))
    tx, ty = TABLE_C
    box(stage, "/World/Table/Top", (tx, ty, P.TABLE_TOP-.025), (*TABLE_SIZE, .05), phys=M["table"], vis=V["table"])
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        box(stage, f"/World/Table/Leg{i}", (tx+dx*.36, ty+dy*.36, (P.TABLE_TOP-.05)/2), (.05, .05, P.TABLE_TOP-.05),
            collide=False, vis=V["dark"])

    hub_angle = corner(stage, M, V, seed_rng)
    spin = wheel(stage, M, V, seed_rng)
    robot(stage, M, V, sync)
    return dict(hub_angle=hub_angle, wheel_spin=spin)


def corner(stage, M, V, rng):
    """Knuckle on the suspension (car lifted: rebound stop), hub on a preloaded bearing, brake, wheelhouse."""
    hq = gq(R_HUB)
    knuckle = rigid(stage, "/World/Car/Knuckle", H, hq, KNUCKLE_MASS, iters=(64, 8))
    UsdPhysics.MassAPI(knuckle).CreateDiagonalInertiaAttr(Gf.Vec3f(.12, .12, .08))
    susp = joint(stage, "/World/Car/Suspension", "d6", None, knuckle.GetPath(), pos0=tuple(H),
                 rot1=Gf.Quatf(gq(R_HUB.T)))
    lock(susp, ("transX", "rotX", "rotY", "rotZ"))
    k = P.WHEEL_RATE
    limit(susp, "transZ", 0., .09)                      # 0 = on the rebound stop, up to 90 mm bump
    drive(susp, "transZ", -P.DROOP_PRELOAD/k, k, 2*.3*np.sqrt(k*KNUCKLE_MASS), 2e4)
    lat = 900e3                                          # bushing compliance along the axle (estimate)
    limit(susp, "transY", -.004, .004)
    drive(susp, "transY", 0., lat, 2*np.sqrt(lat*KNUCKLE_MASS), 2e4)
    # caliper on the knuckle, trailing side, inside the rim barrel (16 mm radial clearance)
    kp = "/World/Car/Knuckle"
    for i in range(6):                                   # 10 deg pieces: convex hulls stay true to the arc
        a = pi-radians(30)+radians(10)*i
        extrude(stage, kp+f"/Caliper_{i}", annulus_sector(.120, .163, a, a+radians(10), 2), -.050, .008,
                phys=M["steel"], vis=V["caliper"])

    h = float(rng.uniform(0, 72))
    hub = rigid(stage, "/World/Car/Hub", H, gq(R_HUB@rot((0, 0, 1), h)), P.HUB_MASS, iters=(64, 8))
    UsdPhysics.MassAPI(hub).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, P.OFFSET-.03))
    UsdPhysics.MassAPI(hub).CreateDiagonalInertiaAttr(Gf.Vec3f(.035, .035, .06))
    PhysxSchema.PhysxContactReportAPI.Apply(hub).CreateThresholdAttr(0.)
    hp = "/World/Car/Hub"
    co = P.CONTACT_OFFSET
    face_z = (P.OFFSET-.012, P.OFFSET)
    extrude(stage, hp+"/Face", circle(0, 0, P.FLANGE_R, 40), *face_z, phys=M["steel"], vis=V["steel"],
            offset=P.FACE_CONTACT_OFFSET)
    # pilot and studs as in the validated cell: native cylinders with convex lead-in cones, 0.8 mm contact offset
    r_out, L = P.PILOT/2, P.PILOT_LEN
    po = P.PILOT_CONTACT_OFFSET
    cylinder(stage, hp+"/Pilot", (0, 0), P.OFFSET, P.OFFSET+L-.0015, r_out, phys=M["steel"], vis=V["steel"], offset=po)
    prism(stage, hp+"/PilotTip", [(x, y, P.OFFSET+L-.0015) for x, y in circle(0, 0, r_out, 48)],
          [(x, y, P.OFFSET+L) for x, y in circle(0, 0, r_out-.001, 48)], phys=M["steel"], vis=V["steel"], offset=po)
    for i in range(5):
        a = i*tau/5
        cx, cy = P.PCD/2*cos(a), P.PCD/2*sin(a)
        cylinder(stage, hp+f"/Stud_{i}", (cx, cy), P.OFFSET, P.OFFSET+P.STUD_PROJ-.002, P.STUD_D/2,
                 phys=M["steel"], vis=V["steel"], offset=co)
        prism(stage, hp+f"/StudTip_{i}", [(x, y, P.OFFSET+P.STUD_PROJ-.002) for x, y in circle(cx, cy, P.STUD_D/2, 24)],
              [(x, y, P.OFFSET+P.STUD_PROJ) for x, y in circle(cx, cy, P.STUD_D/2-.0015, 24)],
              phys=M["steel"], vis=V["steel"], offset=co)
    rr, rt = P.ROTOR
    extrude(stage, hp+"/Rotor", annulus_sector(.085, rr, 0, tau, 40), -.030, -.030+rt, phys=M["steel"], vis=V["dark"])
    extrude(stage, hp+"/Hat", circle(0, 0, .085, 30), -.030+rt, face_z[0], collide=False, vis=V["dark"])
    brg = joint(stage, "/World/Car/Bearing", "revolute", knuckle.GetPath(), hub.GetPath(),
                rot1=Gf.Quatf(gq(rot((0, 0, 1), -h))))
    brg.CreateAxisAttr("Z")
    drive(brg, "angular", 0., 0., P.BEARING_DRAG/.5, 50.)   # 0.75 N m reached at 0.5 deg/s (viscous stand-in)

    # wheelhouse liner (static), in the hub frame: angle a in the x-z plane -> local (cos a, -sin a)
    wh = "/World/Car/Wheelhouse"
    xform(stage, wh, H, gq(R_HUB))
    a0, a1 = (radians(v) for v in P.ARCH_SPAN)
    n = 20
    for i in range(n):
        b0, b1 = a0+(a1-a0)*i/n, a0+(a1-a0)*(i+1)/n
        loop = [(r*cos(t), -r*sin(t)) for r, t in ((P.ARCH_R, b0), (P.ARCH_R+.025, b0), (P.ARCH_R+.025, b1), (P.ARCH_R, b1))]
        extrude(stage, wh+f"/Liner_{i}", loop[::-1], *P.ARCH_Z, phys=M["liner"], vis=V["liner"])
    # fender skin above the arch, flush with the liner's outboard edge
    for i in range(n):
        b0, b1 = a0+(a1-a0)*i/n, a0+(a1-a0)*(i+1)/n
        loop = [(r*cos(t), -r*sin(t)) for r, t in ((P.ARCH_R+.025, b0), (.60, b0), (.60, b1), (P.ARCH_R+.025, b1))]
        extrude(stage, wh+f"/Fender_{i}", loop[::-1], P.ARCH_Z[1]-.006, P.ARCH_Z[1], phys=M["liner"], vis=V["blue"])
    return h


def web_pieces():
    """Mounting pad as Delaunay triangles of the bore, the five lug holes and the pad outline (method of the
    validated cell, map/wheel_cell/wheel_geometry.plate_face). Triangles are convex, so PhysX's convex hull is
    exact and no hole or bore is partly filled."""
    from scipy.spatial import Delaunay
    rings = [circle(0, 0, .083, 96), circle(0, 0, P.BORE/2, 144)]
    rings += [circle(P.PCD/2*cos(k*tau/5), P.PCD/2*sin(k*tau/5), P.LUG_HOLE/2, 48) for k in range(5)]
    pts = np.array([p for ring in rings for p in ring])
    faces = Delaunay(pts).simplices
    centres = pts[faces].mean(axis=1)
    for ring in rings[1:]:
        loop = np.array(ring)
        c = loop.mean(axis=0)
        r_chord = np.linalg.norm(loop[0]-c)*cos(pi/len(loop))
        faces = faces[np.linalg.norm(centres-c, axis=1) > r_chord-1e-9]
        centres = pts[faces].mean(axis=1)
    out = []
    for f in faces:
        tri = pts[f]
        if np.cross(tri[1]-tri[0], tri[2]-tri[0]) < 0:
            tri = tri[::-1]
        out.append([tuple(p) for p in tri])
    return out


TREAD = ((.216, -.100), (.270, -P.TIRE_HALF_W), (.290, -.100), (P.TIRE_R-.004, -.081), (P.TIRE_R, -.062),
         (P.TIRE_R, .062), (P.TIRE_R-.004, .081), (.290, .100), (.270, P.TIRE_HALF_W), (.216, .100))


def visual_only(stage, path):
    """The referenced wheel asset is a rigid body with colliders of its own: strip both."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            prim.RemoveAPI(UsdPhysics.MassAPI)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def wheel(stage, M, V, rng):
    """Rim body (pad, barrel) + rigid-ring tire on its sidewall springs, lying flat, outboard face up."""
    spin = float(rng.uniform(0, 360))
    R0 = rot((0, 0, 1), spin)
    c = np.array([*TABLE_C, P.TABLE_TOP+P.TIRE_HALF_W+.002])
    rim = rigid(stage, "/World/Wheel", c, gq(R0), P.RIM_MASS, ccd=False, iters=(64, 8))
    UsdPhysics.MassAPI(rim).CreateDiagonalInertiaAttr(Gf.Vec3f(.17, .17, .27))
    UsdPhysics.MassAPI(rim).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    PhysxSchema.PhysxContactReportAPI.Apply(rim).CreateThresholdAttr(0.)
    wp = "/World/Wheel"
    ch = P.BORE_CHAMFER
    rb = P.BORE/2
    for i, tri in enumerate(web_pieces()):
        on_bore = [abs(np.hypot(x, y)-rb) < 1e-6 for x, y in tri]
        z0 = P.OFFSET
        if any(on_bore):
            # hub-side bore chamfer (ch x 45 deg): bore vertices pushed out by ch at the back face
            low = [((x*(rb+ch)/rb, y*(rb+ch)/rb) if f else (x, y)) for (x, y), f in zip(tri, on_bore)]
            p = prism(stage, wp+f"/PadCh_{i}", [(x, y, P.OFFSET) for x, y in low], [(x, y, P.OFFSET+ch) for x, y in tri],
                      phys=M["alloy"], offset=P.PAD_CONTACT_OFFSET)
            UsdGeom.Imageable(p).CreatePurposeAttr("guide")
            z0 = P.OFFSET+ch
        p = extrude(stage, wp+f"/Pad_{i}", tri, z0, P.OFFSET+P.WEB_T, phys=M["alloy"], offset=P.PAD_CONTACT_OFFSET)
        UsdGeom.Imageable(p).CreatePurposeAttr("guide")
    for i in range(24):
        a0, a1 = i*tau/24, (i+1)*tau/24
        p = extrude(stage, wp+f"/Barrel_{i}", annulus_sector(P.RIM_R-.024, P.RIM_R, a0, a1, 2), -P.RIM_W/2, P.RIM_W/2,
                    phys=M["alloy"])
        UsdGeom.Imageable(p).CreatePurposeAttr("guide")
    vis = stage.DefinePrim(wp+"/Visual", "Xform")
    vis.GetReferences().AddReference(str(WHEEL_USD))
    stage.OverridePrim(wp+"/Visual/Visuals/Tire").SetActive(False)
    visual_only(stage, wp+"/Visual")

    tire = rigid(stage, "/World/Tire", c, gq(R0), P.TIRE_MASS, ccd=False, iters=(64, 8))
    r_i, r_o, w = P.RIM_R, P.TIRE_R, 2*P.TIRE_HALF_W
    UsdPhysics.MassAPI(tire).CreateDiagonalInertiaAttr(Gf.Vec3f(*((3*(r_i**2+r_o**2)+w*w)/12*P.TIRE_MASS,)*2,
                                                                 (r_i**2+r_o**2)/2*P.TIRE_MASS))
    PhysxSchema.PhysxContactReportAPI.Apply(tire).CreateThresholdAttr(0.)
    tp = "/World/Tire"
    for i in range(32):
        a, b = i*tau/32, (i+1)*tau/32
        p = prism(stage, tp+f"/Tread_{i}", [(r*cos(a), r*sin(a), z) for r, z in TREAD],
                  [(r*cos(b), r*sin(b), z) for r, z in TREAD], phys=M["rubber"])
        UsdGeom.Imageable(p).CreatePurposeAttr("guide")
    tv = stage.DefinePrim(tp+"/Visual", "Xform")
    tv.GetReferences().AddReference(str(WHEEL_USD))
    for name in ("Barrel", "CenterWeb", "Spoke_0", "Spoke_1", "Spoke_2", "Spoke_3", "Spoke_4", "ValveStem"):
        stage.OverridePrim(tp+f"/Visual/Visuals/{name}").SetActive(False)
    visual_only(stage, tp+"/Visual")
    mount = joint(stage, "/World/TireMount", "d6", rim.GetPath(), tire.GetPath())
    z = P.TIRE_ZETA
    for dof, k in (("transX", P.TIRE_K_RADIAL), ("transY", P.TIRE_K_RADIAL), ("transZ", P.TIRE_K_LAT)):
        limit(mount, dof, -.015, .015)
        drive(mount, dof, 0., k, 2*z*np.sqrt(k*P.TIRE_MASS), 5e4)
    I_ax = (r_i**2+r_o**2)/2*P.TIRE_MASS
    I_tr = I_ax/2
    for dof, k, inertia in (("rotZ", P.TIRE_K_TORSION, I_ax), ("rotX", P.TIRE_K_TILT, I_tr), ("rotY", P.TIRE_K_TILT, I_tr)):
        limit(mount, dof, -8., 8.)
        drive(mount, dof, 0., per_deg(k), per_deg(2*z*np.sqrt(k*inertia)), 5e3)
    return spin


LINK6_HOME = (np.array([.15, 0., 2.795]), rot((0, 0, 1), -90))   # link6 at all-zero joints (USD probe)


def robot(stage, M, V, sync=True):
    rp = "/World/Robot"
    r = stage.DefinePrim(rp, "Xform")
    r.GetReferences().AddReference(str(ROBOT_USD))
    xform(stage, rp, tuple(ROBOT_BASE), gq(rot((0, 0, 1), 0)))
    for name in ("onrobot_rg2_base_link", "left_outer_knuckle", "left_inner_finger", "gripper_center",
                 "left_inner_knuckle", "right_inner_knuckle", "right_outer_knuckle", "right_inner_finger",
                 "link6/rs080n2rg2"):
        stage.OverridePrim(f"{rp}/{name}").SetActive(False)

    # tool body (gripper housing + adapter); its frame is the flange frame
    body_mass = P.GRIP["mass"]-2*1.6                     # SCHUNK mass includes the two base jaws (~1.6 kg each, est.)
    l6p, l6R = ROBOT_BASE+LINK6_HOME[0], LINK6_HOME[1]
    tool = rigid(stage, "/World/Tool", tuple(l6p), gq(l6R), body_mass+.55)
    tp = "/World/Tool"
    c = g2f((0, CHORD_Y, BODY_FRONT+BODY_H/2))
    box(stage, tp+"/Body", tuple(c), (BODY_L, BODY_W, BODY_H), phys=M["steel"], vis=V["blue"])
    box(stage, tp+"/Adapter", tuple(g2f((0, CHORD_Y, FLANGE_Z-ADAPTER_T/2))), (.22, .16, ADAPTER_T),
        phys=M["steel"], vis=V["alu"])
    j = joint(stage, "/World/ToolMount", "fixed", f"{rp}/link6", tool.GetPath())
    jaws = []
    for side, name in ((1, "R"), (-1, "L")):    # right first: the left slide mimics it
        jp = f"/World/Jaw{name}"
        origin_g = np.array([side*TOUCH_X, CHORD_Y, BODY_FRONT])
        jaw = rigid(stage, jp, tuple(l6p), gq(l6R), None)
        # base jaw block
        box(stage, jp+"/Base", tuple(g2f(origin_g+(0, 0, -JAW_BLOCK/2))), (.09, .12, JAW_BLOCK), phys=M["steel"], vis=V["dark"])
        # finger: aluminium plate from the jaw, outboard of the contact, back to the shoe
        f0, f1 = SHOE["z0"], BODY_FRONT-JAW_BLOCK
        # finger hangs under the shoe: its top stays below the arch end (-9.9 deg) at r < 365 mm
        low = -pi/2+side*(SHOE_A-P.SHOE_ARC/2)
        fx = side*(SHOE["r_in"]+SHOE["pu"]+SHOE["backing"]+.0125)*abs(cos(low))
        fy = CHORD_Y-.075
        fc = g2f((fx, fy, (f0+f1)/2))
        flen = f1-f0
        box(stage, jp+"/Finger", tuple(fc), (.025, .10, flen), phys=M["steel"], vis=V["alu"])
        finger_mass = .025*.10*flen*P.ALU_DENSITY
        # shoe: PU face on an aluminium backing, arc centred at +-45 deg below the centre
        centre = -pi/2+side*SHOE_A
        a0, a1 = centre-P.SHOE_ARC/2, centre+P.SHOE_ARC/2
        ri, rpu, rb = SHOE["r_in"], SHOE["r_in"]+SHOE["pu"], SHOE["r_in"]+SHOE["pu"]+SHOE["backing"]
        for k in range(4):
            b0, b1 = a0+(a1-a0)*k/4, a0+(a1-a0)*(k+1)/4
            for part, (r0, r1, m, v) in {"PU": (ri, rpu, M["pu"], V["pu"]), "Back": (rpu, rb, M["steel"], V["alu"])}.items():
                loop = annulus_sector(r0, r1, b0, b1, 2)
                prism(stage, jp+f"/{part}_{k}", [tuple(g2f((x, y, SHOE["z0"]))) for x, y in loop],
                      [tuple(g2f((x, y, SHOE["z1"]))) for x, y in loop], phys=m, vis=v)
            loop = annulus_sector(SHOE["lip_r0"], rb, b0, b1, 2)
            prism(stage, jp+f"/Lip_{k}", [tuple(g2f((x, y, SHOE["lip"][0]))) for x, y in loop],
                  [tuple(g2f((x, y, SHOE["lip"][1]))) for x, y in loop], phys=M["pu"], vis=V["pu"])
        shoe_vol = (rb**2-ri**2)/2*P.SHOE_ARC*(SHOE["z1"]-SHOE["z0"])
        UsdPhysics.MassAPI.Apply(jaw).CreateMassAttr(1.6+finger_mass+shoe_vol*1800.)
        pj = joint(stage, f"/World/Jaw{name}Slide", "prismatic", tool.GetPath(), jaw.GetPath())
        pj.CreateAxisAttr("X")
        lo, hi = JAW_Q_RANGE
        pj.CreateLowerLimitAttr(lo if side > 0 else -hi)
        pj.CreateUpperLimitAttr(hi if side > 0 else -lo)
        if not sync:
            # scenario 'nosync': each jaw on its own piston (half the catalogue force), no rack and pinion
            drive(pj, "linear", 0., 0., 1e6, P.GRIP["force"]/2, vel=0.)
        elif side > 0:
            # pneumatic piston on the right jaw; the rack and pinion carries the left jaw (mimic below), so
            # the drive limit is the catalogue gripping force (sum of both jaws)
            drive(pj, "linear", 0., 0., 1e6, P.GRIP["force"], vel=0.)
        else:
            mimic = PhysxSchema.PhysxMimicJointAPI.Apply(pj.GetPrim(), UsdPhysics.Tokens.rotX)
            mimic.GetReferenceJointRel().AddTarget("/World/JawRSlide")
            mimic.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
            mimic.GetGearingAttr().Set(1.)       # q_L = -q_R: symmetric rack-and-pinion synchronisation
            mimic.GetOffsetAttr().Set(0.)
        jaws.append((jp, side))
    for link in ("link6", "link5", "link4"):
        for b in [tp]+[j for j, _ in jaws]:
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(b)).CreateFilteredPairsRel().AddTarget(f"{rp}/{link}")
    return jaws
