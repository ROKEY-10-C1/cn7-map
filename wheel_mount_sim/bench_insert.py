"""Insertion bench: many copies of one hub corner + RS080N + PHL-W gripper holding a wheel, in ONE PhysX scene,
each copy with its own start misalignment and/or control settings. Runs only the end of the mount cycle:
aligned 120 mm in front of the hub -> force insertion (mount_control.MountSide.insert) -> release/retract -> judge.

Geometry: the standalone corner/wheel/robot of scene.py (bore 144-gon: 0.072 mm radial pilot clearance, a little
tighter than the cell's 0.08 mm). Robot base relative to the hub as the cell's left robot to FL at dock A, so the
joint configurations are the cell's. Final checks stay in the cell (mount_control.py).

    ~/isaacsim/python.sh wheel_mount_sim/bench_insert.py --set offsets --out wheel_mount_sim/output/bench/b1
    ~/isaacsim/python.sh wheel_mount_sim/bench_insert.py --variants my.json      # list of {"offset_mm": [dx, dy], "cfg": {...}}
"""
import argparse
import json
import os
import sys
import time
from math import cos, radians, sin
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("--set", default="offsets", help="built-in variant set: offsets | lat | single")
ap.add_argument("--variants", default=None, help="JSON file: list of {offset_mm, cfg, label}")
ap.add_argument("--spacing", type=float, default=5.)
ap.add_argument("--observe", type=float, default=1.5)
ap.add_argument("--t-max", type=float, default=25.)
ap.add_argument("--out", default=None)
ap.add_argument("--trace", action="store_true", help="40 Hz trace of every cell (trace_E*.json)")
ap.add_argument("--gui", action="store_true", help="window: all cells side by side, drawn while computing")
args = ap.parse_args()

os.environ.setdefault("MOUNT_PERCEPTION", "teach")     # no vision: jam retry by fixed back-off, as the fixed cell
os.environ.setdefault("MOUNT_TEACH_FILE", "/tmp/claude-1000/bench_teach_unused.json")
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1600, "height": 900})
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import mount_control as MC  # noqa: E402

MC._late_imports()
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402

S, P, process = MC.S, MC.P, MC.process
OUT = Path(args.out) if args.out else HERE/"output"/"bench"/time.strftime("ins_%H%M%S")
OUT.mkdir(parents=True, exist_ok=True)
P_GW = np.array([0., -.0005, .0013])                # wheel centre in the grip frame at the hub (cell traces)
HUB_W = np.array([0., 0., P.HUB_Z])                 # scene.H
# cell: left robot base (MOUNT_BASE_X, ROBOT_BASE_Y), yaw 270; FL hub (MOUNT_HUB_X, half track, HUB_Z)
BASE_REL = np.array([process.MOUNT_BASE_X-process.MOUNT_HUB_X, process.ROBOT_BASE_Y-.7925, 0.])
YAW = 270.


def variants():
    if args.variants:
        return json.loads(Path(args.variants).read_text())
    if args.set == "single":
        return [dict(label="0", offset_mm=[0, 0], cfg={})]
    if args.set == "offsets":                       # baseline control, misalignment 0..0.3 mm in 4 directions
        out = []
        for r in (0., .1, .2, .3):
            for a in (0, 90, 180, 270):
                out.append(dict(label=f"r{r:.1f}_a{a}", offset_mm=[r*cos(radians(a)), r*sin(radians(a))], cfg={}))
        return out
    if args.set == "lat":                           # lateral admittance settings at 0.2 mm misalignment
        out = []
        for rate in (.0005, .002, .005, .010):
            for slow in (1., .5):
                for a in (90, 270):
                    out.append(dict(label=f"rate{rate*1000:.1f}_slow{slow}_a{a}", offset_mm=[.2*cos(radians(a)), .2*sin(radians(a))],
                                    cfg=dict(pilot_lat_rate=rate, lat_rate=rate, pilot_slow=slow, pilot_deadband=5.,
                                             lat_deadband=5.)))
        return out
    raise SystemExit(f"unknown set {args.set}")


VARS = variants()
N = len(VARS)

# ---------------------------------------------------------------- template (one corner + wheel + robot)
tmpl = Usd.Stage.CreateInMemory()
S.configure(None)
M_V = None


def build_template(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.Xform.Define(stage, "/World")
    from geom import material, look
    M = dict(rubber=material(stage, "/World/Mat/Rubber", 1.5, 1.5), pu=material(stage, "/World/Mat/PU", *P.MU_SHOE_TREAD),
             steel=material(stage, "/World/Mat/Steel", 1.5, 1.5), table=material(stage, "/World/Mat/TableSteel", *P.MU_TREAD_TABLE),
             alloy=material(stage, "/World/Mat/Alloy", *P.MU_ALU_STEEL), liner=material(stage, "/World/Mat/Liner", .5, .4))
    V = {k: look(stage, f"/World/Look/{k}", (.5, .5, .5), .5) for k in
         ("steel", "dark", "table", "pu", "alu", "blue", "liner", "caliper", "floor")}
    S.corner(stage, M, V, np.random.default_rng(0))
    S.wheel(stage, M, V, np.random.default_rng(0))
    S.robot(stage, M, V, True, rp="/World/Mount/Arm", base=HUB_W*0+BASE_REL, yaw=YAW, prefix="/World/Mount", q0=MC.PARK)


import cell_mount  # noqa: E402  (PARK)
MC.PARK = cell_mount.PARK
build_template(tmpl)
# rim/tire push-out speed cap now comes from scene.wheel (params.WHEEL_MAX_DEPEN)
tpath = OUT/"template.usda"
tmpl.GetRootLayer().Export(str(tpath))

# ---------------------------------------------------------------- bench stage: N references, one physics scene
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.)
scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
px.CreateTimeStepsPerSecondAttr(P.PHYSICS_HZ)
px.CreateSolverTypeAttr("TGS")
px.CreateEnableCCDAttr(True)
px.CreateEnableStabilizationAttr(False)
px.CreateBroadphaseTypeAttr("MBP")
px.CreateEnableGPUDynamicsAttr(False)
cols = int(np.ceil(np.sqrt(N)))
offs = []
for i in range(N):
    off = np.array([(i % cols)*args.spacing, (i//cols)*args.spacing, 0.])
    offs.append(off)
    e = UsdGeom.Xform.Define(stage, f"/Env_{i}")
    e.GetPrim().GetReferences().AddReference(str(tpath), "/World")
    e.AddTranslateOp().Set(Gf.Vec3d(*off))
# scene.py authors its bodies in world coordinates (!resetXformStack!), so the Env offset does not reach them:
# shift those prims, and the world-side anchor of joints that have no body there
for i in range(1, N):
    off = Gf.Vec3d(*offs[i])
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"/Env_{i}")):
        xf = UsdGeom.Xformable(prim) if prim.IsA(UsdGeom.Xformable) else None
        if xf and xf.GetResetXformStack():
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(op.Get()+type(op.Get())(off))
                    break
                if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                    m = op.Get()
                    m.SetTranslateOnly(m.ExtractTranslation()+off)
                    op.Set(m)
                    break
        if prim.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(prim)
            for rel, attr in ((j.GetBody0Rel(), j.GetLocalPos0Attr()), (j.GetBody1Rel(), j.GetLocalPos1Attr())):
                if not rel.GetTargets() and attr.HasAuthoredValue():
                    attr.Set(attr.Get()+Gf.Vec3f(*offs[i]))
if args.gui:
    from pxr import UsdLux
    UsdLux.DomeLight.Define(stage, "/Dome").CreateIntensityAttr(1000)
    sun = UsdLux.DistantLight.Define(stage, "/Sun")
    sun.CreateIntensityAttr(2500)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))
    floor = UsdGeom.Cube.Define(stage, "/Floor")
    floor.CreateSizeAttr(1.)
    ext = (cols-1)*args.spacing
    UsdGeom.Xformable(floor).AddTranslateOp().Set(Gf.Vec3d(ext/2-.6, ext/2+.8, -.01))
    UsdGeom.Xformable(floor).AddScaleOp().Set(Gf.Vec3f(ext+6, ext+6, .02))
    floor.CreateDisplayColorAttr([(.35, .36, .38)])
    cam = UsdGeom.Camera.Define(stage, "/BenchCam")
    cam.CreateFocalLengthAttr(18.)
    cam.CreateClippingRangeAttr(Gf.Vec2f(.05, 200.))
    rows_ = int(np.ceil(N/cols))
    cx, cy = (cols-1)*args.spacing/2, (rows_-1)*args.spacing/2
    eye = Gf.Vec3d(cx+2.2, cy-ext*.55-3.6, 3.4+ext*.35)
    m = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(cx-.3, cy+.6, .8), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
for _ in range(5):
    app.update()

api = omni.physx.get_physx_interface()
sim = omni.physx.get_physx_simulation_interface()
stage_id = UsdUtils.StageCache.Get().Insert(stage)
sim.attach_stage(stage_id.ToLongInt())
sim.simulate(1./P.PHYSICS_HZ, 0.)
sim.fetch_results()
if os.environ.get("BENCH_DEBUG"):
    xc0 = UsdGeom.XformCache()
    for i in range(N):
        hp = f"/Env_{i}/Car/Hub"
        r = api.get_rigidbody_transformation(hp)
        print("DBG", i, "physx", r.get("ret_val"), [round(v, 4) for v in r.get("position", [])],
              "usd", list(xc0.GetLocalToWorldTransform(stage.GetPrimAtPath(hp)).ExtractTranslation()), flush=True)


class BenchEnv:
    """The parts of mount_control.CellEnv that MountSide uses."""

    def __init__(self):
        self.HZ = P.PHYSICS_HZ
        self.DT = 1./self.HZ
        self.t = 0.
        self.seed = 0
        self.stage = stage
        self.ROOT = "/Env_0"
        self.out = OUT
        self.units, self.events, self.listeners = [], [], []
        self.stop_after = None
        self.teach, self.teach_dirty = {}, False
        self.car_dx = 0.
        self.diag = False
        self.sim_view = MC.tensors.create_simulation_view("numpy", stage_id.ToLongInt())
        self.sim_view.set_subspace_roots("/")
        self.xc = UsdGeom.XformCache()
        self._local = {}
        self.log = open(OUT/"events.jsonl", "w")

    pose = MC.CellEnv.pose

    @property
    def cell(self):
        return SimpleNamespace(api=api, sim=sim)

    def car_offset_mm(self):
        return [[0., 0., 0.], 0.]

    def log_event(self, e):
        self.events.append(e)
        self.log.write(json.dumps(e, default=float)+"\n")


env = BenchEnv()
prof = MC.Profile()
units = []
for i, var in enumerate(VARS):
    st = SimpleNamespace(name=f"E{i}", side=1, hub_path=f"/Env_{i}/Car/Hub", wheel=f"/Env_{i}/Wheel",
                         tire=f"/Env_{i}/Tire", hub=(1., 0., 0.), zone=None, phase="idle")
    info = dict(side=1, prefix=f"/Env_{i}/Mount", root=f"/Env_{i}/Mount/Arm", base=offs[i]+BASE_REL, yaw=YAW)
    u = MC.MountSide(env, f"E{i}", info, st, prof)
    u.cfg = dict(var.get("cfg", {}))
    u.var = var
    units.append(u)
bodies = env.sim_view.create_rigid_body_view([p for u in units for p in (u.station.wheel, u.station.tire)])


def put_wheel(i, R, o):
    data = np.array(bodies.get_transforms(), np.float32)
    vel = np.array(bodies.get_velocities(), np.float32)
    q = S.quat(R)                                    # w, x, y, z
    for k in (2*i, 2*i+1):
        data[k, :3] = o
        data[k, 3:] = [q[1], q[2], q[3], q[0]]
        vel[k] = 0.
    idx = np.array([2*i, 2*i+1], np.int32)
    bodies.set_transforms(data, idx)
    bodies.set_velocities(vel, idx)


def program(i, u):
    var = u.var
    try:
        Rh, oh = env.pose(u.station.hub_path)
        Rw, ow = Rh.copy(), oh+Rh[:, 2]*.12          # wheel aligned (holes on studs), 120 mm out
        RG = u.R_mount
        oG = ow-RG@P_GW
        q = u.solve(RG, oG, MC.PARK)
        qa = np.array(u.av.get_dof_positions(), np.float32)
        qa[0, :6] = q
        jl = u.dofs.index("JawLSlide")
        qa[0, u.JR], qa[0, jl] = .012, -.012
        u.av.set_dof_positions(qa, u.IDX0)
        u.av.set_dof_velocities(np.zeros_like(qa), u.IDX0)
        u.av.set_dof_position_targets(qa, u.IDX0)
        u.q = np.array(qa[0], float)
        put_wheel(i, Rw, ow)
        u.command()
        yield
        Rf, of = env.pose(u.prefix+"/Arm/link6")
        Rfk, pfk = u.fk(u.q[:6])
        u.bias = dict(R=Rf@Rfk.T, p=of-pfk)
        u.grip(True)
        still, last = 0, None
        for _ in range(int(2.*env.HZ)):             # hold the wheel in place until the jaws stop on the tread
            put_wheel(i, Rw, ow)
            jq = float(u.av.get_dof_positions()[0][u.JR])
            still = still+1 if (last is not None and abs(jq-last) < 2e-5) else 0
            last = jq
            u.command()
            yield
            if still >= 12:
                break
        yield from u.wait(MC.GRIP_HOLD)
        # exact alignment (as a noise-free vision pass) plus the variant's misalignment in the hub plane
        Rh, oh = env.pose(u.station.hub_path)
        Rw_, ow_ = env.pose(u.station.wheel)
        RGm, oGm = u.grip_frame()
        R_GW, p_GW = RGm.T@Rw_, RGm.T@(ow_-oGm)
        x = Rh.T@Rw_[:, 0]
        phi = np.degrees(np.arctan2(x[1], x[0]))
        Rw_des = Rh@S.rot((0, 0, 1), phi-((phi+36.) % 72.-36.))
        RG_t = Rw_des@R_GW.T
        oG_t = oh+Rh@np.array([0, 0, .12])-RG_t@p_GW
        Rf, of = env.pose(u.prefix+"/Arm/link6")
        Rfk, pfk = u.fk(u.q[:6])
        u.bias = dict(R=Rf@Rfk.T, p=of-pfk)
        dx, dy = var.get("offset_mm", [0, 0])
        oG_t = oG_t+Rh@np.array([dx, dy, 0.])/1000
        yield from u.move_joints(u.solve(RG_t, oG_t, u.q[:6]), scale=.2)
        yield from u.wait(.1)
        rel, tilt = u.seat()
        u.event("bench_aligned", radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3), tilt=round(float(tilt), 3))
        u.RG_t, u.oG_t, u.gap_pre = RG_t, oG_t, .12
        if MC.LASER:                                 # taught position is off by the variant -> lasers correct it
            yield from u.laser_align()
        u.t_ins0 = env.t
        yield from u.insert()
        u.t_ins = env.t-u.t_ins0
        yield from u.release_retract()
        yield from u.judge(args.observe)
    except MC.StopSide as exc:
        u.result = f"stopped: {exc}"
        u.fail_reason = exc.reason
    u.phase = "done"


for i, u in enumerate(units):
    u.load(program(i, u))
wall0 = time.perf_counter()
steps = 0
while env.t < args.t_max and any(u.phase != "done" for u in units):
    for u in units:
        u.tick()
    sim.simulate(env.DT, env.t)
    sim.fetch_results()
    env.t += env.DT
    steps += 1
    if env.diag:
        MC.CellEnv._diag_contacts(env)
    if os.environ.get("BENCH_RCC_DEBUG") and steps % 60 == 0:
        for u in units[:1]:
            if u.rcc:
                x = np.asarray(u.av.get_dof_positions()[0], float)
                rel, _ = u.seat()
                print("RCCDBG", round(env.t, 2), u.phase, [round(float(x[i])*1000, 3) for i in u.rcc],
                      "radial", round(float(np.hypot(rel[0], rel[1]))*1000, 3), "gap", round(float(rel[2])*1000, 2), flush=True)
    if args.trace and steps % 6 == 0:
        for u in units:
            if u.phase not in ("done", "wait_tire", "idle"):
                u.sample()
                tr = u.trace[-1]
                tr["f_ax"] = round(float(-(u.ft_world()-getattr(u, "tare_free", np.zeros(3)))@u.axis), 1) \
                    if hasattr(u, "axis") else None
    if args.gui and steps % 8 == 0:
        if steps < 80:
            from omni.kit.viewport.utility import get_active_viewport
            get_active_viewport().camera_path = "/BenchCam"
        app.update()
    if steps % (5*env.HZ) == 0:
        print(f"SPEED sim {env.t:.1f}s wall {time.perf_counter()-wall0:.1f}s", flush=True)
wall = time.perf_counter()-wall0

rows = []
for i, u in enumerate(units):
    ev = [e for e in env.events if e.get("side") == u.label]
    g = lambda n: [e for e in ev if e["event"] == n]
    fi, fin, al = g("force_insert_end"), g("final"), g("bench_aligned")
    rows.append(dict(label=u.var.get("label", str(i)), offset_mm=u.var.get("offset_mm"), cfg=u.var.get("cfg", {}),
                     result=u.result, first_seated=bool(fi and fi[0]["seated"]), jams=len(g("jam_retry")),
                     aligned_radial_mm=al[0]["radial_mm"] if al else None,
                     insert_s=round(getattr(u, "t_ins", float("nan")), 2),
                     final_gap_mm=fin[0]["gap_mm"] if fin else None, final_tilt_deg=fin[0]["tilt_deg"] if fin else None))
summary = dict(n=N, sim_s=round(env.t, 2), wall_s=round(wall, 1), x_realtime=round(env.t/wall, 3),
               env_seconds_per_wall_s=round(N*env.t/wall, 2), rows=rows)
(OUT/"summary.json").write_text(json.dumps(summary, indent=1, default=float))
print(f"BENCH N={N} sim {env.t:.1f}s wall {wall:.1f}s  (x{env.t/wall:.2f} per env, {N*env.t/wall:.2f} env-s per s)")
for r in rows:
    print("ROW", json.dumps(r, default=float))
env.log.close()
if args.trace:
    for u in units:
        (OUT/f"trace_{u.label}.json").write_text(json.dumps(u.trace))
if args.gui and not os.environ.get("BENCH_NO_HOLD"):
    print("GUI: close the window to exit", flush=True)
    while app.is_running():
        app.update()
app.close()
