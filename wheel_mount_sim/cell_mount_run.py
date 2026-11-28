"""Mount cell, our part only: the team cell map (scene/, belt end stop from cell_layout.py), the car standing at dock A, no AMR transfer.

Both mount robots (RS080N + SCHUNK PHL-W 63-080-S) run the wheel_mount_sim control on the cell layout, at the
same time: wait for pick_ready at the belt-end stop -> measure (vision) -> pick the head tire -> lift -> swing
through the -x side (J1 hard stop is straight back) standing the wheel up -> via point on the hub axis -> align
(vision) -> force-controlled insertion (flange F/T) -> release -> retract -> back over the pick point. The next
tire then moves into the stop. Car: kinematic at dock A (front hub at process.MOUNT_HUB_X), the AMR parked under
it; the AMR docking physics is not run.

--process (2026-11-28): the main process instead of the parked car. The AMR brings the car from the gate and docks
at A (floor clamps closed, lift up); both robots mount the front wheels; the AMR indexes 2.720 m to B with the
front wheels hanging on the studs; both robots mount the rear wheels from the same belt-end stops; the AMR goes on
to C and out the front. Fastening has no physics yet: the B->C and C->exit moves get probe completion signals.

Usage:  ~/isaacsim/python.sh wheel_mount_sim/cell_mount_run.py [--seed N] [--side both|left|right] [--basic]
                                                              [--no-video] [--gui] [--out DIR] [--process]
(the cycle-time profile is the default; --basic is the slow commissioning profile)
Writes output/cell/<run>/summary.json, events.jsonl and overview/left/hub_left .gif (+ .mp4 when available).
"""
import argparse
import json
import os
import sys
import time
from math import atan2, cos, degrees, sin, tau
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=42)
ap.add_argument("--side", choices=["both", "left", "right"], default="both")
ap.add_argument("--basic", action="store_true", help="slow commissioning profile (35 % joint speed, 2 s observe)")
ap.add_argument("--fast", action="store_true", help=argparse.SUPPRESS)          # default now
ap.add_argument("--speed", type=float, default=None, help="fraction of the RS080N joint speed limits")
ap.add_argument("--transfer-speed", type=float, default=None,
                help="joint-speed fraction while standing the wheel up (default .35, fast .45)")
ap.add_argument("--no-video", action="store_true")
ap.add_argument("--gui", action="store_true", help="open the Isaac Sim window and play live (stays open at the end)")
ap.add_argument("--out", default=None)
ap.add_argument("--stop-after", default=None, help="end after this event (e.g. gripped)")
ap.add_argument("--usd-writeback", action="store_true", help="write physics poses to USD every step (slow, 0.3x)")
ap.add_argument("--diag", action="store_true", help="per-step contact logging (slow): HIT lines, jam contacts")
ap.add_argument("--diag-awake", action="store_true", help="keep wheels awake (contact reports after release)")
ap.add_argument("--replay", default=None, metavar="RUN_DIR",
                help="play a finished run (its replay.npz) in the Isaac Sim window at exactly real time, no physics")
ap.add_argument("--t-max", type=float, default=None, help="simulated seconds before giving up (120, --process 300)")
ap.add_argument("--process", action="store_true",
                help="AMR docks at A, front wheels mounted, index to B, rear wheels mounted, C, exit")
ap.add_argument("--keep-going", action="store_true",
                help="--process demo: go on to B, C and the exit even when a wheel is not seated (results still logged)")
args = ap.parse_args()
if args.t_max is None:
    args.t_max = 300. if args.process else 120.
if args.replay:
    args.gui, args.no_video = True, True
FAST = not args.basic                  # cycle-time profile (docs/GOALS.md section 1) is the default
SPEED = args.speed or (.50 if FAST else .35)
if args.transfer_speed is None:
    args.transfer_speed = .45 if FAST else .35       # fastL10: 1.2 mm slip at .45 with the wheel tucked

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1600 if args.gui else 960, "height": 900 if args.gui else 540,
                     "renderer": "RayTracedLighting", "anti_aliasing": 1, "multi_gpu": False})

import carb  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# Physics results go to Fabric (read by the renderer) instead of being written back to USD every step: with the USD
# write-back the RTX scene delegate processed every body change and one step took 12 ms (0.3x real time) against
# 2.9 ms in the headless check app. Poses for control are read from PhysX directly (pose() below).


def use_fabric():
    """Called after the stage and the cell are built (enabling it before opening the stage crashed in Fabric)."""
    enable_extension("omni.physx.fabric")
    s = carb.settings.get_settings()
    s.set_bool("/physics/updateToUsd", False)
    s.set_bool("/physics/updateVelocitiesToUsd", False)
    s.set_bool("/physics/updateParticlesToUsd", False)
    s.set_bool("/physics/updateResidualsToUsd", False)


import omni.physics.tensors as tensors  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/"wheel_cell"))
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True

from physics import Cell, HZ, ROOT  # noqa: E402
from process import DOCK_X  # noqa: E402
import cell_layout as L  # noqa: E402
import params as P  # noqa: E402
import scene as S  # noqa: E402

DT = 1./HZ
OUT = Path(args.out) if args.out else HERE/"output"/"cell"/time.strftime("%Y%m%d-%H%M%S")
OUT.mkdir(parents=True, exist_ok=True)
VMAX = np.array([3.1416, 3.1416, 2.7925, 3.2289, 2.8798, 4.8869])   # URDF joint speed limits (rad/s)
LO = np.array([-3.14, -1.83, -2.70, -6.28, -2.53, -6.28])
HI = np.array([3.14, 2.44, 2.35, 6.28, 2.53, 6.28])
F_STOP = 400.                          # N: stop pushing (robot force monitoring, as run.py)
CENTER_NOISE = .00005                  # vision centre noise 1 sigma (as run.py)
NOISE_DEG = .03                        # vision angle noise 1 sigma (as run.py)
CAR_HALF_WIDTH = .9125                 # m, CN7 body incl. mirrors (cell:bodyWidthM 1.825)
R_TUCK = 1.00                          # m, wheel centre from the J1 axis while swinging: the swing passes straight
                                       # in front of the car, 2.50-0.91 = 1.59 m from the base; outer tread edge
                                       # 1.00+0.316 = 1.32 m leaves 0.27 m (at the pick radius 1.39 m it hit, dbg7)
SWING_SPEED = .70                      # J1-only swing with the wheel flat and tucked: tangential/centripetal load
                                       # at R 1.0 m stays < 60 N on a 5.5 kN shoe friction grip (stand-up stays at
                                       # --transfer-speed, the slip limit found standalone)
EMPTY_SPEED = .80                      # joint moves with the gripper empty
GUI_EVERY = 16                         # GUI redraw every 16 physics steps = 15 fps of simulated time: redrawing every
                                       # 4 steps ran the cell at ~0.08x real time (RTX render dominates)
FRAME_EVERY = 16                       # 15 frames per simulated second
rng = np.random.default_rng(args.seed)

# ---------------------------------------------------------------- scene and cell
ctx = omni.usd.get_context()
ctx.open_stage(str(L.SCENE_DIR/"wheel_cell.usda"))
for _ in range(30):                                    # let the stage and its textures load before building
    app.update()
stage = ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
cell = Cell(stage, args.seed, True, start="gate")
assert L.active(stage), "cell_layout did not switch on: no belt end stop in the scene"


def park_car_at_dock():
    """Car kinematic at dock A (car frame at DOCK_X: front hub at MOUNT_HUB_X, hub height HUB_Z), knuckles and
    hubs moved with it, the AMR deck and lift parked under it. The AMR docking is not simulated."""
    xc = UsdGeom.XformCache()
    car = stage.GetPrimAtPath(ROOT+"/CN7")
    before = xc.GetLocalToWorldTransform(car)
    movers = [ROOT+"/DockLift"]
    for name in ("FL", "FR", "RL", "RR"):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{ROOT}/{name}")):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() not in ("Wheel", "Tire"):
                movers.append(str(prim.GetPath()))
    loc = stage.GetPrimAtPath(ROOT+"/HangerLocator")
    if loc:
        movers += [str(p.GetPath()) for p in Usd.PrimRange(loc) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    worlds = {p: xc.GetLocalToWorldTransform(stage.GetPrimAtPath(p)) for p in movers}
    UsdGeom.Xformable(car).GetOrderedXformOps()[0].Set(Gf.Vec3d(DOCK_X, 0., 0.))
    xc.Clear()
    moved = before.GetInverse()*xc.GetLocalToWorldTransform(car)
    for path, world in worlds.items():
        prim = stage.GetPrimAtPath(path)
        w = world*moved
        prim.GetAttribute("xformOp:translate").Set(w.ExtractTranslation())
        if prim.GetAttribute("xformOp:orient"):
            prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(w.ExtractRotationQuat()) if
                                                    prim.GetAttribute("xformOp:orient").GetTypeName() == "quatf"
                                                    else Gf.Quatd(w.ExtractRotationQuat()))
    deck = stage.GetPrimAtPath(ROOT+"/DockedAMR")
    t = deck.GetAttribute("xformOp:translate").Get()
    deck.GetAttribute("xformOp:translate").Set(Gf.Vec3d(DOCK_X, t[1], t[2]))
    for path in (ROOT+"/CN7", ROOT+"/DockedAMR", ROOT+"/DockLift"):
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Set(True)
    cell.carrier.step = lambda api, dt: None          # no AMR traction, lift or clamp commands


if not args.process:                   # --process: the AMR brings the car and docks it (Carrier, clamps)
    park_car_at_dock()
if "--diag" in sys.argv:                 # contact reports cost time in the cell's per-step contact loop
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Live/MountSim")):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.GetName().startswith(("link", "Tool", "Jaw")):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
for s in cell.stations:
    if "--diag" in sys.argv:
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(s.tire)).CreateThresholdAttr(0.)
    if "--diag-awake" in sys.argv:
        for path in (s.wheel, s.tire):
            PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateSleepThresholdAttr(0.)


def camera(path, eye, target, focal=24.):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(.05, 100.))
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return path


hub_x = DOCK_X+cell.stations[0].hub[0]
cams = {"overview": camera("/World/MountCams/Overview", (-4.6, 0., 4.4), (0.1, 0., .7), 18.),
        "left": camera("/World/MountCams/Left", (-2.9, 5.6, 3.1), (-.15, 2.3, .8), 20.),
        "hub_left": camera("/World/MountCams/HubLeft", (hub_x+.9, 2.05, 1.25), (hub_x, .79, .94), 24.)}
if args.gui:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = cams["left"]

writers = {}
if not args.no_video:
    import omni.replicator.core as rep
    for k, path in cams.items():
        rp = rep.create.render_product(path, (960, 540))
        a = rep.AnnotatorRegistry.get_annotator("rgb")
        a.attach(rp)
        writers[k] = (a, [])

# ---------------------------------------------------------------- state
PROXIES = []
if not args.usd_writeback and not args.replay:
    use_fabric()
    # With Fabric the robots (articulations) render where PhysX has them, but the wheel bodies' referenced visual
    # stayed where the wheels started (speed7: hub empty, tire still on the belt). Each wheel visual is shown by a
    # render-only copy outside the body that render() moves to the PhysX pose (4 prims per frame).
    for s in cell.stations:
        vis = stage.GetPrimAtPath(s.wheel+"/Visual")
        if not vis:
            continue
        UsdGeom.Imageable(vis).MakeInvisible()
        proxy = UsdGeom.Xform.Define(stage, f"/World/WheelVisuals/{s.name}")
        proxy.GetPrim().GetReferences().AddInternalReference("/AssetLibrary/Wheel_16")
        ops = (proxy.AddTranslateOp(), proxy.AddOrientOp())
        PROXIES.append((s.wheel, ops))
    # Same for the gripper housing and the two jaws (separate rigid bodies on joints, not articulation links: with
    # Fabric they stayed invisible at their start pose, rec1). Their box/prism shapes are copied to a render-only
    # root with every physics schema removed, so the copies never collide.
    layer = stage.GetSessionLayer()
    for unit in ("Left", "Right"):
        for part in ("Tool", "JawR", "JawL"):
            body = stage.GetPrimAtPath(f"/World/Live/MountSim/{unit}/{part}")
            if not body:
                continue
            root = f"/World/GripperVisuals/{unit}_{part}"
            proxy = UsdGeom.Xform.Define(stage, root)
            for child in body.GetChildren():
                if not child.IsA(UsdGeom.Gprim) or not layer.GetPrimAtPath(child.GetPath()):
                    continue
                dst = Sdf.Path(f"{root}/{child.GetName()}")
                Sdf.CopySpec(layer, child.GetPath(), layer, dst)
                copy = stage.GetPrimAtPath(dst)
                for api in list(copy.GetAppliedSchemas()):
                    if api.startswith(("Physics", "Physx")):
                        copy.RemoveAppliedSchema(api)
                for prop in list(copy.GetAuthoredProperties()):
                    if prop.GetName().startswith(("physics:", "physx")):
                        copy.RemoveProperty(prop.GetName())
                UsdGeom.Imageable(child).MakeInvisible()
            ops = (proxy.AddTranslateOp(), proxy.AddOrientOp())
            PROXIES.append((str(body.GetPath()), ops))
cell.step()                                            # physics objects exist from here on
sim_view = tensors.create_simulation_view("numpy", cell.stage_id.ToLongInt())
sim_view.set_subspace_roots("/")
xc = UsdGeom.XformCache()
t_sim = [0.]
events = []
event_log = open(OUT/"events.jsonl", "w")


_LOCAL = {}


def _quat_R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose(path):
    """World pose (R, p) from PhysX for bodies and articulation links; for a collider/xform under a body, the body
    pose times the authored local offset; static prims from USD."""
    r = cell.api.get_rigidbody_transformation(path)
    if r.get("ret_val"):
        return _quat_R(r["rotation"]), np.array(r["position"], float)
    if path not in _LOCAL:
        prim = stage.GetPrimAtPath(path)
        body = prim.GetParent()
        while body and body.GetPath().pathString != "/" and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        xc.Clear()
        m = xc.GetLocalToWorldTransform(prim)
        Rm, pm = np.array(m.ExtractRotationMatrix()).T, np.array(m.ExtractTranslation())
        if body and body.HasAPI(UsdPhysics.RigidBodyAPI):
            mb = xc.GetLocalToWorldTransform(body)
            Rb, pb = np.array(mb.ExtractRotationMatrix()).T, np.array(mb.ExtractTranslation())
            _LOCAL[path] = (str(body.GetPath()), Rb.T@Rm, Rb.T@(pm-pb))
        else:
            _LOCAL[path] = (None, Rm, pm)
    body, Rl, pl = _LOCAL[path]
    if body is None:
        return Rl, pl
    Rb, pb = pose(body)
    return Rb@Rl, pb+Rb@pl


K_DIR = Path.home()/"isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/Kawasaki/rs080n"
IDX0 = np.array([0], dtype=np.int32)


class StopSide(Exception):
    pass


class MountSide:
    """One mount robot and its front station. `run()` is a generator: one yield per physics step."""

    def __init__(self, label, info, station):
        self.label, self.station = label, station
        self.side = info["side"]
        self.prefix = info["prefix"]
        self.av = sim_view.create_articulation_view(info["root"])
        meta = self.av.shared_metatype
        self.dofs = list(meta.dof_names)
        self.JR = self.dofs.index("JawRSlide")
        self.ti = list(meta.link_names).index("Tool")
        self.q = np.array(self.av.get_dof_positions()[0], float)
        self.jaw_v = 0.
        self.ik = LulaKinematicsSolver(robot_description_path=str(K_DIR/"rmpflow/rs080n_robot_description.yaml"),
                                       urdf_path=str(K_DIR/"rs080n_onrobot_rg2.urdf"))
        self.base = np.asarray(info["base"], float)
        self.ik.set_robot_base_pose(robot_position=self.base, robot_orientation=np.array(S.quat(S.rot((0, 0, 1), info["yaw"]))))
        self.bias = dict(R=np.eye(3), p=np.zeros(3))
        self.rng = np.random.default_rng([args.seed, self.side+2])
        self.phase = "wait_tire"
        self.trace = []
        self.result = "incomplete"
        # hub frame fixed to the car (knuckle side): z outboard; grip frame at the hub has y_G up (shoes below)
        self.contact = str(stage.GetPrimAtPath(ROOT+"/CN7").GetPath())+"/Contact_"+station.name
        self.R_mount = S.rot((1, 0, 0), -self.side*90.)@S.rot((0, 0, 1), 180. if self.side > 0 else 0.)

    # ---- low level
    def command(self):
        q = self.av.get_dof_positions().copy()
        q[0, :6] = self.q[:6]
        self.av.set_dof_position_targets(q.astype(np.float32), IDX0)
        v = np.zeros((1, len(self.dofs)), np.float32)
        v[0, self.JR] = self.jaw_v
        self.av.set_dof_velocity_targets(v, IDX0)

    def event(self, name, **kv):
        rel, tilt = self.seat()
        e = dict(t=round(t_sim[0], 3), side=self.label, event=name, gap_mm=round(float(rel[2])*1000, 3),
                 tilt_deg=round(float(tilt), 3), phase_err_deg=round(float(self.phase_error()), 3), **kv)
        events.append(e)
        event_log.write(json.dumps(e, default=float)+"\n")
        event_log.flush()
        print("EVENT", json.dumps(e, default=float), flush=True)
        if args.stop_after == name:
            raise StopSide(f"stop after {name}")

    def seat(self):
        Rh, oh = pose(self.station.hub_path)
        Rw, ow = pose(self.station.wheel)
        return Rh.T@(ow-oh), degrees(np.arccos(np.clip(Rh[:, 2]@Rw[:, 2], -1, 1)))

    def phase_error(self):
        Rh, _ = pose(self.station.hub_path)
        Rw, _ = pose(self.station.wheel)
        x = Rh.T@Rw[:, 0]
        return (degrees(atan2(x[1], x[0]))+36.) % 72.-36.

    def fk(self, q):
        p, R = self.ik.compute_forward_kinematics("link6", q)
        return np.asarray(R), np.asarray(p)

    def refine(self, q, Rt, pt, iters=6):
        q = np.asarray(q, float).copy()

        def err(R, p):
            dR = Rt@R.T
            return np.concatenate([pt-p, np.array([dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]])/2])
        for _ in range(iters):
            e = err(*self.fk(q))
            if np.linalg.norm(e[:3]) < 2e-6 and np.linalg.norm(e[3:]) < 1e-5:
                break
            J = np.zeros((6, 6))
            for i in range(6):
                dq = np.zeros(6)
                dq[i] = 1e-6
                J[:, i] = (e-err(*self.fk(q+dq)))/1e-6
            q = q+np.linalg.solve(J.T@J+1e-9*np.eye(6), J.T@e)
        return q

    def candidates(self, R_G, o_G, warm, n=40):
        Rf, of = S.flange_pose(R_G, o_G)
        Rf, of = self.bias["R"].T@Rf, of-self.bias["p"]
        out = []
        sweep = []                                   # warm start with J1 stepped round: every posture family seeded
        for j1 in np.radians(np.arange(-170., 171., 20.)) if n else ():
            s = np.array(warm, float).copy()
            s[0] = j1
            sweep.append(s)
        for s0 in [warm]+sweep+[self.rng.uniform(LO, HI) for _ in range(n)]:
            q, ok = self.ik.compute_inverse_kinematics("link6", of, S.quat(Rf), warm_start=s0, position_tolerance=2e-4,
                                                       orientation_tolerance=2e-3)
            if ok:
                q = np.asarray(q)
                if all(np.max(np.abs(q-c)) > 1e-3 for c in out):
                    out.append(q)
        good = []
        for q in out:
            q = self.refine(q, Rf, of)
            R, pp = self.fk(q)
            ang = np.degrees(np.arccos(np.clip((np.trace(Rf@R.T)-1)/2, -1, 1)))
            if np.linalg.norm(pp-of) < 1e-3 and ang < .5:          # Lula 'ok' alone let a 180 deg flip through
                good.append(q)
            else:
                self.rejected = getattr(self, "rejected", 0)+1
        return good, (Rf, of)

    def solve(self, R_G, o_G, warm, key=None):
        qs, _ = self.candidates(R_G, o_G, warm, n=0)
        close = [q for q in qs if np.max(np.abs(q-warm)) < .5]
        if close and key is None:
            return close[0]
        qs, _ = self.candidates(R_G, o_G, warm)
        if not qs:
            raise StopSide(f"IK failed at {np.round(o_G, 3)} ({self.phase})")
        return min(qs, key=key or (lambda q: float(np.max(np.abs(q-warm)))))

    def swing_key(self, q0):
        """Joint-space path from q0 must swing through the -x side of the base (J1 hard stop straight back, the
        fastening robot on +x): prefer the solution whose interpolated flange path has the lowest mean x."""
        def key(q):
            # same posture family as at the pick: elbow up (J3 < 0) and the upper arm near upright (|J2| < 60 deg);
            # an over-the-shoulder solution swung the wheel into the floor (dbg3, t 19.5 s)
            family = 0. if (q[2] < 0 and abs(q[1]) < np.radians(60)) else 100.
            xs = [self.fk(q0+(q-q0)*t)[1][0] for t in np.linspace(0, 1, 9)]
            return family+float(np.mean(xs))+.01*float(np.max(np.abs(q-q0)))
        return key

    def sample(self):
        """0.1 s trace: wheel and tire in the grip frame, jaw positions and efforts, flange speed."""
        RG, oG = self.grip_frame()
        _, ow = pose(self.station.wheel)
        _, ot = pose(self.station.tire)
        jq = self.av.get_dof_positions()[0]
        jf = self.av.get_dof_projected_joint_forces()[0] if hasattr(self.av, "get_dof_projected_joint_forces") else [0]*8
        rel, tilt = self.seat()
        self.trace.append(dict(t=round(t_sim[0], 2), phase=self.phase, gap_mm=round(float(rel[2])*1000, 3),
                               radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3), tilt=round(float(tilt), 3),
                               wheel_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ow-oG)],
                               tire_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ot-oG)],
                               jaw_mm=[round(float(jq[self.JR])*1000, 2), round(float(jq[self.JR+1])*1000, 2)],
                               jaw_N=round(float(jf[self.JR]), 1), q_deg=[round(float(v), 1) for v in np.degrees(jq[:6])]))

    def wait(self, seconds):
        for _ in range(int(seconds/DT)):
            self.command()
            yield

    def move_joints(self, q_goal, scale=None):
        scale = scale or SPEED
        q0 = self.q[:6].copy()
        T = max(float(np.max(np.abs(q_goal-q0)/(VMAX*scale))), .4)*1.875
        n = int(T/DT)
        for i in range(1, n+1):
            s = i/n
            s = 10*s**3-15*s**4+6*s**5
            self.q[:6] = q0+(q_goal-q0)*s
            self.command()
            yield

    def move_line(self, R_G, o0, o1, speed, stop=None):
        Lm = float(np.linalg.norm(np.asarray(o1)-np.asarray(o0)))
        n = max(int(Lm/speed*60), 1)
        q_prev = self.q[:6].copy()
        sub_n = int(HZ/60)
        for i in range(1, n+1):
            o = np.asarray(o0)+(np.asarray(o1)-np.asarray(o0))*i/n
            q_next = self.solve(R_G, o, q_prev)
            for j in range(1, sub_n+1):
                self.q[:6] = q_prev+(q_next-q_prev)*j/sub_n
                self.command()
                yield
                if stop and stop():
                    self.stopped_at = o
                    return
            q_prev = q_next
        self.stopped_at = np.asarray(o1)

    def ft_world(self):
        w = self.av.get_link_incoming_joint_force()[0, self.ti]
        Rt, _ = pose(self.prefix+"/Tool")
        return Rt@np.asarray(w[:3], float)

    def vision_target(self, gap):
        """Vision at the hub: wheel pose relative to the hub (+ noise) -> grip-frame target that puts the measured
        wheel on the hub axis, parallel, holes on studs, `gap` out from the seat. Also re-measures the flange bias."""
        st = self.station
        Rh, oh = pose(st.hub_path)
        Rw, ow = pose(st.wheel)
        ow = ow+self.rng.normal(0, CENTER_NOISE, 3)
        Rw = Rw@S.rot(self.rng.normal(0, 1, 3), self.rng.normal(0, NOISE_DEG))
        RG, oG = self.grip_frame()
        R_GW, p_GW = RG.T@Rw, RG.T@(ow-oG)
        x = Rh.T@Rw[:, 0]
        phi = degrees(atan2(x[1], x[0]))
        Rw_des = Rh@S.rot((0, 0, 1), phi-((phi+36.) % 72.-36.))
        ow_des = oh+Rh@np.array([0, 0, gap])
        RG_t = Rw_des@R_GW.T
        Rf, of = pose(self.prefix+"/Arm/link6")
        Rfk, pfk = self.fk(self.q[:6])
        self.bias["p"] = of-pfk
        self.bias["R"] = Rf@Rfk.T
        return RG_t, ow_des-RG_t@p_GW

    def grip(self, close):
        v_close, v_open = P.GRIP["stroke"]/P.GRIP["t_close"], P.GRIP["stroke"]/P.GRIP["t_open"]
        self.jaw_v = -v_close if close else v_open

    def grip_frame(self):
        Rf, of = pose(self.prefix+"/Arm/link6")
        RG = Rf@S.R_FG.T
        return RG, of-RG@S.T_GF

    def force_insert(self, R_G, o_start, axis, v, expect):
        """Admittance insertion on the flange F/T sensor, as run.force_insert (145 N set point, 25 N/mm lateral,
        seat = >= 90 % force and < 0.02 mm in 0.3 s, then the press force is ramped down)."""
        Kl, Fset = P.LATERAL_COMPLIANCE, P.PRESS_FORCE
        st = dict(s=0., lat=np.zeros(3), Ff=np.zeros(3), f_ax=0., q_prev=self.q[:6].copy())
        tare = self.ft_world()
        sub_n = int(HZ/60)
        still_n = int(P.SEAT_STILL[0]*60)
        gaps, seated = [], False

        def cycle(setpoint):
            F = -(self.ft_world()-tare)
            st["Ff"] = st["Ff"]+(F-st["Ff"])*.15
            f_ax = float(st["Ff"]@axis)
            st["f_ax"] = f_ax
            f_lat = st["Ff"]-f_ax*axis
            n_lat = np.linalg.norm(f_lat)
            f_lat = f_lat*max(n_lat-10., 0.)/n_lat if n_lat > 1e-9 else f_lat
            st["s"] += v/60*float(np.clip(1-f_ax/setpoint, -.5, 1.))
            goal = np.clip(f_lat/Kl, -.005, .005)
            st["lat"] = st["lat"]+np.clip(goal-st["lat"], -.0005/60, .0005/60)
            o = o_start-axis*st["s"]+st["lat"]
            q_next = self.solve(R_G, o, st["q_prev"])
            for j in range(1, sub_n+1):
                self.q[:6] = st["q_prev"]+(q_next-st["q_prev"])*j/sub_n
                self.command()
                yield
            st["q_prev"] = q_next
            st["o"] = o

        for _ in range(int(15.*60)):
            yield from cycle(Fset)
            gaps.append(self.seat()[0][2])
            if (st["f_ax"] >= P.SEAT_FORCE_RATIO*Fset and len(gaps) > still_n
                    and max(gaps[-still_n:])-min(gaps[-still_n:]) < P.SEAT_STILL[1]):
                # the robot knows the travel to the seat from the vision at the hub: stopping well short of it
                # under full force is a jam, not a seat
                seated = st["s"] >= expect-.003
                break
        top = sorted(getattr(self, "now_contacts", {}).items(), key=lambda kv: -kv[1][0])[:10]
        self.event("force_insert_end", seated=seated, expected_mm=round(expect*1000, 2), contacts_N_sepmm=top, travel_mm=round(st["s"]*1000, 2),
                   lateral_mm=[round(float(x)*1000, 3) for x in st["lat"]], f_axial_N=round(st["f_ax"], 1))
        if seated:
            for _ in range(int((.4 if FAST else .6)*60)):
                yield from cycle(P.RELEASE_FORCE)
            self.event("press_relieved", f_axial_N=round(st["f_ax"], 1))
        self.stopped_at = st["o"]
        self.seated = seated

    # ---- the cycle
    def run(self):
        try:
            yield from self._run()
        except StopSide as exc:
            self.result = f"stopped: {exc}"
            print("ERROR", self.label, exc, flush=True)
        while True:
            self.command()
            yield

    def _run(self):
        st = self.station
        # wait over the pick point while the tire conveys in (cycle steady state: the robot ends each cycle there)
        pre_pick = np.array([L.PICK_X, self.side*L.LINE_Y, L.BELT_Z+.1025+.20])
        q_pre = self.solve(S.rot((0, 0, 1), 90.), pre_pick, self.q[:6], key=self.swing_key(self.q[:6]))
        yield from self.move_joints(q_pre, scale=EMPTY_SPEED)
        while st.phase != "pick_ready":
            if st.phase in ("feed_failed",):
                raise StopSide(f"feed {st.phase}")
            self.command()
            yield
        self.event("pick_ready", along_mm=round(getattr(st.zone, "along_mm", 0.), 2),
                   across_mm=round(getattr(st.zone, "across_mm", 0.), 2))
        t0 = t_sim[0]
        # vision at the pick point (overhead camera): wheel centre and spin; hub stud angle at the car
        Rw, ow = pose(st.wheel)
        ow = ow+self.rng.normal(0, CENTER_NOISE, 3)*np.array([1, 1, 0])
        spin = degrees(atan2(Rw[1, 0], Rw[0, 0]))+self.rng.normal(0, NOISE_DEG)
        Rh, _ = pose(st.hub_path)
        Rk, _ = pose(self.contact)
        xh = Rk.T@Rh[:, 0]
        hub_deg = degrees(atan2(xh[1], xh[0]))+self.rng.normal(0, NOISE_DEG)
        # grip yaw against the holes: the wheel x axis ends up at (180 - g) in the knuckle frame on the left
        # (R_mount has a 180 deg turn about the hub axis) and at -g on the right, and must sit on a stud
        g = ((180. if self.side > 0 else 0.)-hub_deg) % 72.
        # the 5 holes leave 5 grip yaws; take the one with the jaws closest to across the flow (world y)
        yaws = sorted((spin+g+72.*k for k in range(5)), key=lambda a: abs((a-90.+90.) % 180.-90.))
        self.event("measured", wheel_spin_deg=round(spin, 3), hub_deg=round(hub_deg, 3), grip_yaw_deg=round(g, 3),
                   jaw_axis_deg=round(yaws[0] % 180., 2))
        self.phase = "approach"
        above = ow+np.array([0, 0, .20])
        R_pick, q_above = None, None
        for a in yaws[:3]:
            try:
                R_pick = S.rot((0, 0, 1), a)
                q_above = self.solve(R_pick, above, self.q[:6], key=self.swing_key(self.q[:6]))
                break
            except StopSide:
                continue
        if q_above is None:
            raise StopSide("no pick pose")
        yield from self.move_joints(q_above, scale=EMPTY_SPEED)
        self.phase = "descend"
        yield from self.move_line(R_pick, above, ow, .25 if FAST else .10)
        RG, oG = self.grip_frame()
        _, ow_now = pose(st.wheel)
        _, ot_now = pose(st.tire)
        Rf, of = pose(self.prefix+"/Arm/link6")
        self.event("descended", wheel_in_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ow_now-oG)],
                   tire_in_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ot_now-oG)],
                   link6_vs_fk_mm=round(float(np.linalg.norm(self.fk(self.q[:6])[1]-of))*1000, 2),
                   link6_vs_target_mm=round(float(np.linalg.norm(S.flange_pose(R_pick, ow)[1]-of))*1000, 2),
                   z_G_world=[round(float(v), 3) for v in RG[:, 2]], x_G_world=[round(float(v), 3) for v in RG[:, 0]])
        self.phase = "close"
        self.grip(True)
        yield from self.wait(P.GRIP["t_close"]+(.2 if FAST else .6))
        jq = self.av.get_dof_positions()[0]
        self.event("gripped", jaw_mm=[round(float(jq[self.JR])*1000, 2), round(float(jq[self.dofs.index("JawLSlide")])*1000, 2)])
        RG, oG = self.grip_frame()
        _, ow_ = pose(st.wheel)
        w_in_G0 = RG.T@(ow_-oG)
        self.phase = "lift"
        up = ow+np.array([0, 0, .25])
        clear = ow+np.array([0, 0, .05])                    # straight up off the belt and the next tire first
        yield from self.move_line(R_pick, ow, clear, .10)
        st.zone.picked()                                   # the next tire moves into the stop
        self.event("lifted")

        self.phase = "transfer"
        Rh, oh = pose(st.hub_path)
        axis = Rh[:, 2]
        via = oh+axis*.45
        pre = oh+axis*.12
        q_via = self.solve(self.R_mount, via, self.q[:6], key=self.swing_key(self.q[:6]))
        # 1) tuck the flat wheel in to R_TUCK, 2) swing on J1 only through the -x side, 3) stand it up to the via
        #    (dbg4: swinging and standing up in one joint move passed J5 through 0; dbg7: swinging at the pick
        #    radius put the tread into the car side)
        d = (up-self.base)[:2]
        tuck = np.array([*(self.base[:2]+d/np.linalg.norm(d)*R_TUCK), up[2]])
        # all the way up first: pulling in has a +x part toward the next tire on the belt (205 mm tall); the
        # diagonal 'up and in' from 50 mm hit it and pushed the jaws 5 mm open (fastL8, 16 mm slip)
        yield from self.move_line(R_pick, clear, up, .25 if FAST else .10)
        yield from self.move_line(R_pick, up, tuck, .40 if FAST else .15)
        self.q_tuck = self.q[:6].copy()
        q_swing = self.q[:6].copy()
        q_swing[0] = q_via[0]
        yield from self.move_joints(q_swing, scale=SWING_SPEED if FAST else args.transfer_speed)
        self.event("swung", j1_deg=round(float(np.degrees(q_via[0])), 1))
        # 2) stand the wheel up to the via point on the hub axis
        yield from self.move_joints(q_via, scale=args.transfer_speed)
        yield from self.move_line(self.R_mount, via, pre, .30 if FAST else .15)
        RG, oG = self.grip_frame()
        _, ow_ = pose(st.wheel)
        self.event("at_hub", transfer_slip_mm=round(float(np.linalg.norm(RG.T@(ow_-oG)-w_in_G0))*1000, 2))
        yield from self.wait(.1 if FAST else .5)

        self.phase = "align"
        gap_pre = .12
        RG_t, oG_t = self.R_mount, pre
        # two vision passes also in the fast profile: one pass left 0.064-0.074 mm at the pilot (radial clearance
        # 0.09 mm) and jammed the bore chamfer on the pilot tip at 9 mm in 2 of 4 runs (fastL8, fastL10)
        for it in range(2):
            Rh, oh = pose(st.hub_path)
            Rw, ow = pose(st.wheel)
            ow = ow+self.rng.normal(0, CENTER_NOISE, 3)
            Rw = Rw@S.rot(self.rng.normal(0, 1, 3), self.rng.normal(0, NOISE_DEG))
            RG, oG = self.grip_frame()
            R_GW, p_GW = RG.T@Rw, RG.T@(ow-oG)
            x = Rh.T@Rw[:, 0]
            phi = degrees(atan2(x[1], x[0]))
            Rw_des = Rh@S.rot((0, 0, 1), phi-((phi+36.) % 72.-36.))
            ow_des = oh+Rh@np.array([0, 0, gap_pre])
            RG_t = Rw_des@R_GW.T
            oG_t = ow_des-RG_t@p_GW
            Rf, of = pose(self.prefix+"/Arm/link6")
            Rfk, pfk = self.fk(self.q[:6])
            self.bias["p"] = of-pfk
            self.bias["R"] = Rf@Rfk.T
            yield from self.move_joints(self.solve(RG_t, oG_t, self.q[:6]), scale=.2 if FAST else .08)
            yield from self.wait(.10 if FAST else .4)
            rel, _ = self.seat()
            self.event(f"aligned_{it}", radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3))
        Rh, oh = pose(st.hub_path)
        Rw, ow = pose(st.wheel)
        holes = [Rh.T@(ow+Rw@np.array([P.PCD/2*cos(k*tau/5), P.PCD/2*sin(k*tau/5), 0])-oh) for k in range(5)]
        studs = [Rh.T@(pose(f"{st.hub_path}/Stud_{i}")[1]-oh) for i in range(5)]
        self.event("hole_check", hole_to_stud_mm=[round(float(min(np.hypot(*(h[:2]-s[:2])) for s in studs))*1000, 3)
                                                  for h in holes])
        self.phase = "insert"
        axis = Rh[:, 2]
        near = oG_t-axis*(gap_pre-.032)
        tare = self.ft_world()
        yield from self.move_line(RG_t, oG_t, near, .100 if FAST else .020,
                                  stop=lambda: np.linalg.norm(self.ft_world()-tare) > F_STOP)
        expect = float((np.asarray(self.stopped_at)-(oG_t-axis*gap_pre))@axis)      # travel left to the seat
        yield from self.force_insert(RG_t, np.asarray(self.stopped_at), axis, v=.020 if FAST else .010, expect=expect)
        # jam recovery: stopped short of the seat under full force -> back off out of the pilot (still on the studs),
        # measure again and press again. Vision noise (0.05 mm 1 sigma per axis) is of the order of the 0.09 mm
        # radial pilot clearance, so a single measurement jams the bore chamfer on the pilot tip now and then.
        for attempt in range(1, 3):
            if self.seated:
                break
            gap_now = float(self.seat()[0][2])
            self.event("jam_retry", attempt=attempt)
            back = np.asarray(self.stopped_at)+axis*.005
            yield from self.move_line(RG_t, np.asarray(self.stopped_at), back, .010)
            yield from self.wait(.1)
            gap_back = float(self.seat()[0][2])
            RG_t, oG_t = self.vision_target(gap_back)
            yield from self.move_joints(self.solve(RG_t, oG_t, self.q[:6]), scale=.2 if FAST else .08)
            yield from self.wait(.1)
            yield from self.force_insert(RG_t, oG_t, axis, v=.010, expect=gap_back)
        yield from self.wait(.1 if FAST else .3)
        stop_at = np.asarray(self.stopped_at)
        self.event("inserted")
        self.phase = "release"
        self.grip(False)
        yield from self.wait(.90 if FAST else P.GRIP["t_open"]+.3)
        self.phase = "retract"
        out = oh+axis*.45+(stop_at-oh-axis*((stop_at-oh)@axis))
        off_studs = stop_at+axis*.04                       # slow until the shoes are clear of the tread
        yield from self.move_line(RG_t, stop_at, off_studs, .10 if FAST else .05)
        yield from self.move_line(RG_t, off_studs, out, .50 if FAST else .10)
        self.event("retracted")
        self.phase = "return"
        self.bias = dict(R=np.eye(3), p=np.zeros(3))
        q_back = self.q_tuck.copy()                       # tuck at the hub side, swing back, then over the pick
        q_back[0] = self.q[0]
        yield from self.move_joints(q_back, scale=EMPTY_SPEED)
        # swing back and reach out over the pick point in one move: the empty gripper (half width 0.27 m) sweeps
        # past the car front at <= (1.00+1.39)/2+0.27 = 1.47 m from the base, car side at 1.59 m
        yield from self.move_joints(self.solve(R_pick, above, self.q_tuck), scale=EMPTY_SPEED)
        self.event("cycle_end", cycle_s=round(t_sim[0]-t0, 2))
        self.phase = "observe"
        yield from self.wait(2.)
        rel, tilt = self.seat()
        ok = rel[2] < P.MOUNTED[0] and tilt < P.MOUNTED[1] and np.hypot(rel[0], rel[1]) < .002
        self.result = "mounted" if ok else "not_mounted"
        top = sorted(getattr(self, "last_hang", {}).items(), key=lambda kv: -kv[1][0])[:14]
        Rh, _ = pose(st.hub_path)
        Rw, _ = pose(st.wheel)
        lean = Rw[:, 2]                                     # wheel axis in the world: z > 0 = outer end up
        self.event("final", result=self.result, radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3),
                   rel_mm=[round(float(v)*1000, 3) for v in rel], lean_axis=[round(float(v), 4) for v in lean],
                   contacts_N_sepmm=top)
        self.phase = "done"


# ---------------------------------------------------------------- run
front = {s.side: s for s in cell.stations if s.hub[0] > 0}
sides = [(lab, info) for lab, info in cell.mount_arms.items()
         if args.side == "both" or args.side == lab.lower()]
rear = {s.side: s for s in cell.stations if s.hub[0] < 0}
units = [MountSide(lab+("_front" if args.process else ""), info, front[info["side"]]) for lab, info in sides]
gens = [u.run() for u in units]
for u in units:
    u.command()
proc = dict(done=not args.process, result="incomplete")


def proc_event(name, **kv):
    e = dict(t=round(t_sim[0], 3), side="AMR", event=name, **kv)
    events.append(e)
    event_log.write(json.dumps(e, default=float)+"\n")
    event_log.flush()
    print("EVENT", json.dumps(e, default=float), flush=True)


def amr_x():
    return round(float(cell.api.get_rigidbody_transformation(cell.carrier.path)["position"][0]), 4)


def run_units(us):
    gs = [u.run() for u in us]
    while not all(u.phase == "done" or u.result.startswith("stopped") for u in us):
        for g in gs:
            next(g)
        yield


def carried(tag, us):
    """Hung wheels after an AMR move: still on the studs (gap, radial offset and tilt to the hub)."""
    for u in us:
        rel, tilt = u.seat()
        u.event(tag, radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3))


HANG_GAP = .010    # m: the process goes on while each wheel still hangs on the pilot (12 mm) and the studs. The
                   # params.MOUNTED limit (2 mm, 2 deg) is kept as each unit's result; the cell itself (car parked at
                   # A, no --process) releases at 2.1-2.2 mm / 1.5 deg on seed 42 (base1, 2026-11-28).


def hung(tag, us):
    for u in us:
        if u.result.startswith("stopped"):
            raise StopSide(f"{tag} {u.label}: {u.result}")
        rel, _ = u.seat()
        if rel[2] > HANG_GAP and not args.keep_going:
            raise StopSide(f"{tag} {u.label}: wheel off the hub, gap {rel[2]*1000:.1f} mm")


def index(completed, tag):
    while not cell.advance_amr(completed=completed, robots_clear=True, load_secured=True):
        if t_sim[0] > args.t_max-1:
            raise StopSide(f"index to {tag} refused (stop {cell.amr_stop}, interference {cell.interference[:1]})")
        yield
    proc_event(f"index_{tag}", amr_x=amr_x())
    while cell.amr_index_pending:
        yield
    proc_event(f"stopped_{tag}", amr_x=amr_x(), stop=cell.amr_stop)


def process_run():
    try:
        while not cell.carrier.ready:              # gate -> dock A, clamps closed, lift at the work height
            yield
        proc_event("docked_A", amr_x=amr_x())
        yield from run_units(units)
        hung("front", units)
        yield from index({"front_mounted"}, "B")
        carried("carried_to_B", units)
        rear_units = [MountSide(lab+"_rear", info, rear[info["side"]]) for lab, info in sides]
        front_units = list(units)
        units.extend(rear_units)
        yield from run_units(rear_units)
        hung("rear", rear_units)
        # fastening has no physics here: probe completion signals for the two remaining moves
        yield from index({"front_fastened", "rear_mounted"}, "C")
        carried("carried_to_C", front_units+rear_units)
        yield from index({"rear_fastened"}, "exit")
        carried("carried_to_exit", front_units+rear_units)
        proc["result"] = "done"
    except StopSide as exc:
        proc["result"] = f"stopped: {exc}"
        print("ERROR process", exc, flush=True)
    proc["done"] = True
    while True:
        yield


if args.process:
    gens = [process_run()]


def sync_proxies():
    for body, (op_t, op_q) in PROXIES:
        r = cell.api.get_rigidbody_transformation(body)
        if r.get("ret_val"):
            x, y, z, w = r["rotation"]
            op_t.Set(Gf.Vec3d(*r["position"]))
            op_q.Set(Gf.Quatf(float(w), float(x), float(y), float(z)))


def render():
    sync_proxies()
    if writers:
        rep.orchestrator.step(rt_subframes=1, pause_timeline=False)
    else:
        app.update()
    for k, (a, frames) in writers.items():
        d = a.get_data()
        if d is not None and getattr(d, "size", 0):
            frames.append(np.asarray(d)[:, :, :3].copy())


# ---------------------------------------------------------------- recording / real-time replay
# The live cell runs at ~0.4-0.7x real time on this PC (PhysX alone ~0.9x: 4 wheels x 545 convex tread pieces), so
# the window cannot show the robots at true speed while computing. Every run records the moving bodies at 30 Hz of
# simulated time (replay.npz); --replay plays that file without physics, frame chosen by the wall clock (1.0x).
REC_EVERY = 8                                          # 240 Hz / 8 = 30 Hz
REC_PATHS = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Live/MountSim"))
             if p.HasAPI(UsdPhysics.RigidBodyAPI)]
for s in cell.stations:
    REC_PATHS += [s.wheel, s.tire]
if args.process:                       # the whole moving cell: car, AMR deck and lift, locators, clamps, hub corners
    REC_PATHS += [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT))
                  if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()) not in REC_PATHS]
rec_t, rec_pose = [], []


def record():
    row = np.zeros((len(REC_PATHS), 7))
    for i, path in enumerate(REC_PATHS):
        r = cell.api.get_rigidbody_transformation(path)
        if r.get("ret_val"):
            row[i, :3] = r["position"]
            row[i, 3:] = r["rotation"]                  # x, y, z, w
        else:
            row[i, 6] = np.nan
    rec_t.append(t_sim[0])
    rec_pose.append(row)


def replay(run_dir):
    data = np.load(Path(run_dir)/"replay.npz", allow_pickle=False)
    times, poses, paths = data["t"], data["pose"], [str(x) for x in data["paths"]]
    xc.Clear()
    tracked = set(paths)
    setters = []
    for i, path in enumerate(paths):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        parent = prim.GetParent()
        while parent.GetPath().pathString != "/" and parent.GetPath().pathString not in tracked:
            parent = parent.GetParent()
        moving_parent = paths.index(parent.GetPath().pathString) if parent.GetPath().pathString in tracked else None
        static_parent = xc.GetLocalToWorldTransform(prim.GetParent())
        scale = Gf.Transform(xc.GetLocalTransformation(prim)[0]).GetScale()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        setters.append((i, moving_parent, static_parent, Gf.Matrix4d().SetScale(scale), xf.AddTransformOp()))

    def world(row):
        x, y, z, w = row[3:]
        return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(float(w), float(x), float(y), float(z))), Gf.Vec3d(*(float(v) for v in row[:3])))

    def show(k):
        with Sdf.ChangeBlock():
            for i, mp, sp, S_, op in setters:
                if np.isnan(poses[k, i, 6]):
                    continue
                parent_w = world(poses[k, mp]) if mp is not None else sp
                op.Set(S_*world(poses[k, i])*parent_w.GetInverse())

    SNAPS = [float(v) for v in os.environ.get("REPLAY_SNAP", "").split(",") if v]   # debug screenshots (s)
    show(0)
    for _ in range(30):
        app.update()
    print(f"REPLAY {run_dir}: {times[-1]:.1f} s at real time", flush=True)
    while app.is_running():
        wall0 = time.perf_counter()
        shown, k = 0, -1
        snapped = set()
        while app.is_running():
            k_now = int(np.searchsorted(times, time.perf_counter()-wall0, side="right"))-1
            if k_now >= len(times)-1:
                show(len(times)-1)
                break
            if k_now != k:
                k = k_now
                show(k)
                shown += 1
                for ts in SNAPS:
                    if times[k] >= ts and ts not in snapped:
                        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                        capture_viewport_to_file(get_active_viewport(), str(Path(run_dir)/f"replay_{ts:04.1f}s.png"))
                        snapped.add(ts)
            app.update()
        wall = time.perf_counter()-wall0
        print(f"REPLAY done: sim {times[-1]:.1f} s in wall {wall:.1f} s (x{times[-1]/wall:.2f}), "
              f"{shown} of {len(times)} frames drawn; again in 3 s (close the window to exit)", flush=True)
        if os.environ.get("REPLAY_ONCE"):
            break
        t_end = time.perf_counter()+3.
        while app.is_running() and time.perf_counter() < t_end:
            app.update()


if args.replay:
    replay(args.replay)
    app.close()
    sys.exit(0)

step_i = 0
if writers:
    for _ in range(3):
        render()
    print("FRAMES warmup", {k: len(f) for k, (_, f) in writers.items()}, flush=True)
prof = dict(control=0., physics=0., diag=0., render=0.)
wall0 = time.perf_counter()
try:
    while t_sim[0] < args.t_max and not (proc["done"] if args.process else
                                         all(u.phase == "done" or u.result.startswith("stopped") for u in units)):
        t0_ = time.perf_counter()
        for gen in gens:
            next(gen)
        tac = time.perf_counter()
        prof["control"] += tac-t0_
        cell.prepare_step(DT)
        ta = time.perf_counter()
        cell.sim.simulate(DT, cell.seconds)
        cell.sim.fetch_results()
        tb = time.perf_counter()
        cell.finish_step(DT)
        prof["prepare"] = prof.get("prepare", 0.)+ta-tac
        prof["phx"] = prof.get("phx", 0.)+tb-ta
        prof["finish"] = prof.get("finish", 0.)+time.perf_counter()-tb
        t_sim[0] += DT
        step_i += 1
        if step_i % REC_EVERY == 0:
            record()
        tic = time.perf_counter()
        prof["physics"] += tic-tac
        if args.diag and any(u.phase in ("transfer", "approach", "lift", "insert", "release", "retract") for u in units):
            heads, data = cell.sim.get_contact_report()[:2]
            for u in units:
                u.now_contacts = {}
            for h in heads:
                if not h.num_contact_data:
                    continue
                c0, c1 = (str(PhysicsSchemaTools.intToSdfPath(v)) for v in (h.collider0, h.collider1))
                for u in units:
                    if u.phase in ("insert", "release", "retract", "observe") and (c0.startswith(u.station.wheel) or c1.startswith(u.station.wheel)
                                                or c0.startswith(u.station.tire) or c1.startswith(u.station.tire)):
                        f = sum(np.linalg.norm(data[i].impulse) for i in range(h.contact_data_offset,
                                h.contact_data_offset+h.num_contact_data))*HZ
                        sep = min(data[i].separation for i in range(h.contact_data_offset,
                                  h.contact_data_offset+h.num_contact_data))
                        k = c0.rsplit("/", 1)[-1]+" | "+c1.rsplit("/", 1)[-1]
                        u.now_contacts[k] = (round(f, 1), round(sep*1000, 3))
            for u in units:
                if u.now_contacts and u.phase in ("release", "retract", "observe"):
                    u.last_hang = dict(u.now_contacts)
            for h in heads:
                if not h.num_contact_data:
                    continue
                a0, a1 = (str(PhysicsSchemaTools.intToSdfPath(v)) for v in (h.collider0, h.collider1))
                ours = [u for u in units if any(x.startswith((u.prefix, u.station.wheel, u.station.tire)) for x in (a0, a1))]
                if not ours or all(x.startswith(("/World/Live/MountSim", ours[0].station.wheel, ours[0].station.tire))
                                   for x in (a0, a1)):
                    continue
                f = sum(np.linalg.norm(data[i].impulse) for i in range(h.contact_data_offset,
                                                                         h.contact_data_offset+h.num_contact_data))*HZ
                if f > 50. and not ("/Tire/" in a0 and "/Tire/" in a1) and "AccumEnd" not in a0+a1 \
                        and not ("/Wheel/" in a0+a1 and "/Hub/" in a0+a1) and ours[0].phase != "insert":
                    print("HIT", round(t_sim[0], 3), ours[0].phase, a0, a1, round(f, 1), flush=True)
        if step_i % 24 == 0:
            for u in units:
                if u.phase not in ("wait_tire", "done"):
                    u.sample()
        tr = time.perf_counter()
        prof["diag"] += tr-tic
        if (writers and step_i % FRAME_EVERY == 0) or (args.gui and step_i % GUI_EVERY == 0):
            render()
        prof["render"] += time.perf_counter()-tr
        if args.gui:
            # play at real time: the window shows the robots at the speed of the simulated clock
            ahead = t_sim[0]-(time.perf_counter()-wall0)
            if ahead > 0:
                time.sleep(ahead)
        if step_i % (5*HZ) == 0:
            wall = time.perf_counter()-wall0
            print("SPEED", f"sim {t_sim[0]:.1f}s wall {wall:.1f}s x{t_sim[0]/wall:.2f}",
                  {k: round(v, 1) for k, v in prof.items()}, flush=True)
except Exception as exc:
    print("ERROR", exc, flush=True)
    import traceback
    traceback.print_exc()

# the next tire of each side, after the front one was taken
queue = {}
for s in cell.stations:
    if s.hub[0] < 0:
        p = np.array(cell.api.get_rigidbody_transformation(s.wheel)["position"])
        queue[s.name] = dict(phase=s.phase, along_mm=round(float(p[0]-L.PICK_X)*1000, 2),
                             across_mm=round(float(p[1]-s.side*L.LINE_Y)*1000, 2))
for u in units:
    (OUT/f"trace_{u.label}.json").write_text(json.dumps(u.trace))
summary = dict(results={u.label: u.result for u in units}, process=proc["result"] if args.process else None,
               seed=args.seed, fast=FAST, speed=SPEED,
               sim_time_s=round(t_sim[0], 2), next_tires=queue, events=events)
(OUT/"summary.json").write_text(json.dumps(summary, indent=1, default=float))
if rec_t:
    np.savez_compressed(OUT/"replay.npz", t=np.array(rec_t), pose=np.array(rec_pose, np.float32),
                        paths=np.array(REC_PATHS))
event_log.close()
if writers:
    from PIL import Image
    for k, (_, frames) in writers.items():
        if not frames:
            continue
        try:
            import imageio.v2 as imageio
            imageio.mimwrite(OUT/f"{k}.mp4", frames, fps=15, quality=7)
        except Exception as exc:
            print("mp4 skipped:", exc)
        small = [Image.fromarray(fr).resize((400, 225)).quantize(96) for fr in frames[::3]]
        small[0].save(OUT/f"{k}.gif", save_all=True, append_images=small[1:], duration=200, loop=0, optimize=True)
        Image.fromarray(frames[len(frames)//2]).save(OUT/f"{k}_mid.png")
        Image.fromarray(frames[-1]).save(OUT/f"{k}_end.png")
print("RESULT", json.dumps(summary["results"]), "next", json.dumps(queue), OUT, flush=True)
if args.gui:
    print("GUI: close the Isaac Sim window to exit", flush=True)
    while app.is_running():
        app.update()
cell.close()
app.close()
