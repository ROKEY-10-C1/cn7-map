"""Wheel mount simulation: pick the flat wheel from the work table, stand it up, mount it on the CN7 hub.

Usage:  ~/isaacsim/python.sh run.py [--seed N] [--mu-shoe S D] [--out DIR] [--no-video]
Writes output/<run>/log.csv, summary.json and (unless --no-video) wide.mp4 / hub.mp4 / *.gif.
"""
import argparse
import csv
import json
import time
from math import atan2, degrees
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--mu-shoe", type=float, nargs=2, default=None)
ap.add_argument("--mu-hub", type=float, nargs=2, default=None, help="alloy-steel static/dynamic at the hub")
ap.add_argument("--out", default=None)
ap.add_argument("--no-video", action="store_true")
ap.add_argument("--gui", action="store_true", help="open the Isaac Sim window and play the run live (stays open at the end)")
ap.add_argument("--debug-stop", default=None)
ap.add_argument("--scenario", default="nominal",
                choices=["nominal", "shoe45", "shoe20", "nosync", "joint_transfer", "novision", "position_insert"],
                help="replay one earlier failure: only that one cause is put back")
ap.add_argument("--stop-after", default=None, help="end the run after this event (e.g. lifted, aligned_1)")
ap.add_argument("--speed", type=float, default=.35, help="fraction of the RS080N joint speed limits")
ap.add_argument("--fast", action="store_true", help="cycle-time profile (docs/GOALS.md section 1)")
ap.add_argument("--transfer-speed", type=float, default=.35,
                help="joint-speed fraction while standing the wheel up (grip retention limits it)")
args = ap.parse_args()
FAST = args.fast
if FAST and args.speed == .35:
    args.speed = .50

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1600 if args.gui else 960, "height": 900 if args.gui else 540, "renderer": "RayTracedLighting",
                     "anti_aliasing": 1, "multi_gpu": False})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver  # noqa: E402
from omni.physx import get_physx_simulation_interface  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom  # noqa: E402

import params as P  # noqa: E402
import scene as S  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = Path(args.out) if args.out else HERE/"output"/time.strftime("%Y%m%d-%H%M%S")
OUT.mkdir(parents=True, exist_ok=True)
DT = 1./P.PHYSICS_HZ
FRAME_EVERY = 16                       # 240 / 16 = 15 frames per second of simulated time
VMAX = np.array([3.1416, 3.1416, 2.7925, 3.2289, 2.8798, 4.8869])   # URDF joint speed limits (rad/s)
F_STOP = 400.                          # N: stop pushing (robot force-monitoring threshold, estimate)
DEBUG = args.debug_stop is not None
CENTER_NOISE = .00005                  # vision centre noise 1 sigma (estimate, as in the cell)
NOISE_DEG = .03                        # vision angle noise 1 sigma (estimate, as in the cell)

world = World(stage_units_in_meters=1., physics_dt=DT, rendering_dt=DT, physics_prim_path="/World/PhysicsScene")
stage = omni.usd.get_context().get_stage()
rng = np.random.default_rng(args.seed)
SC = args.scenario
S.configure({"shoe45": 45., "shoe20": 70.}.get(SC))          # angle from the bottom: 45 / 70 / (80 default)
info = S.build(stage, rng, tuple(args.mu_shoe) if args.mu_shoe else None, tuple(args.mu_hub) if args.mu_hub else None,
               sync=SC != "nosync")

# cameras
def camera(path, eye, target, focal=24.):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(.05, 100.))
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return path


cams = {"wide": camera("/World/CamWide", (3.1, 3.3, 2.0), (.45, .85, .75)),
        "hub": camera("/World/CamHub", (.95, .95, .62), tuple(S.H), focal=28.),
        "pick": camera("/World/CamPick", (S.TABLE_C[0]+.95, S.TABLE_C[1]-.85, 1.15), (S.TABLE_C[0], S.TABLE_C[1], .62),
                       focal=24.)}

if args.gui:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = cams["wide"]          # switch with the viewport camera menu

robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="rs080n"))
world.reset()
names = robot.dof_names
ARM = [names.index(f"joint{i}") for i in range(1, 7)]
JAW = [names.index("JawRSlide"), names.index("JawLSlide")]
JAW_DRIVEN = [JAW[0]] if SC != "nosync" else JAW
print("DOFS", names, flush=True)

K = Path.home()/"isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/Kawasaki/rs080n"
ik = LulaKinematicsSolver(robot_description_path=str(K/"rmpflow/rs080n_robot_description.yaml"),
                          urdf_path=str(K/"rs080n_onrobot_rg2.urdf"))
ik.set_robot_base_pose(robot_position=S.ROBOT_BASE, robot_orientation=np.array([1., 0, 0, 0]))

# ---------------------------------------------------------------- recording
writers = {}
if not args.no_video:
    import omni.replicator.core as rep
    for k, path in cams.items():
        rp = rep.create.render_product(path, (960, 540))
        a = rep.AnnotatorRegistry.get_annotator("rgb")
        a.attach(rp)
        writers[k] = (a, [])

contacts = {}


def on_contact(headers, data):
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
        c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
        imp = np.zeros(3)
        for i in range(h.contact_data_offset, h.contact_data_offset+h.num_contact_data):
            imp += np.array([data[i].impulse[0], data[i].impulse[1], data[i].impulse[2]])
        if DEBUG and phase[0] == "insert" and ("/World/Wheel" in a0+a1 or "/World/Tire" in a0+a1) and h.num_contact_data:
            tot = sum(np.linalg.norm([data[i].impulse[0], data[i].impulse[1], data[i].impulse[2]])
                      for i in range(h.contact_data_offset, h.contact_data_offset+h.num_contact_data))/DT
            if tot > 30 and not ("Jaw" in a0+a1 and "Tire" in a0+a1):
                d = data[h.contact_data_offset]
                Rh_, oh_ = pose("/World/Car/Hub")
                pl = Rh_.T@(np.array([d.position[0], d.position[1], d.position[2]])-oh_)
                print("BIG", round(t_sim[0], 4), a0.split("/")[-1], c0.split("/")[-1], a1.split("/")[-1], c1.split("/")[-1],
                      round(tot, 1), "p_hub_mm", np.round(pl*1000, 2), flush=True)
        if False:
            d = data[h.contact_data_offset]
            Rh_, oh_ = pose("/World/Car/Hub")
            pl = Rh_.T@(np.array([d.position[0], d.position[1], d.position[2]])-oh_)
            nl = Rh_.T@np.array([d.normal[0], d.normal[1], d.normal[2]])
            print("CPT", round(t_sim[0], 3), c0.split("/")[-1], c1.split("/")[-1], "p_hub_mm", np.round(pl*1000, 2),
                  "n", np.round(nl, 2), "sep_mm", round(d.separation*1000, 3), "imp", round(np.linalg.norm([d.impulse[0], d.impulse[1], d.impulse[2]]), 3), flush=True)
        key = tuple(sorted((a0, a1)))
        f = contacts.setdefault(key, [np.zeros(3), set()])
        f[0] = f[0]+imp/DT
        f[1].update((c0.split("/")[-1], c1.split("/")[-1]))


sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)

# ---------------------------------------------------------------- state helpers
xc = UsdGeom.XformCache()


def pose(path):
    xc.Clear()
    m = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    R = np.array(m.ExtractRotationMatrix()).T          # Gf is row-vector: transpose to column form
    return R, np.array(m.ExtractTranslation())


def phase_error():
    """Wheel hole angle relative to the nearest stud, in the hub body frame (deg)."""
    Rh, _ = pose("/World/Car/Hub")
    Rw, _ = pose("/World/Wheel")
    x = Rh.T@Rw[:, 0]
    return (degrees(atan2(x[1], x[0]))+36.) % 72.-36.


def seat():
    """Wheel origin in the hub frame (z = axial gap, 0 = seated) and axis tilt (deg)."""
    Rh, oh = pose("/World/Car/Hub")
    Rw, ow = pose("/World/Wheel")
    rel = Rh.T@(ow-oh)
    tilt = degrees(np.arccos(np.clip(Rh[:, 2]@Rw[:, 2], -1, 1)))
    return rel, tilt


log_rows = []
F_VEC = np.zeros(3)
events = []
t_sim = [0.]
step_count = [0]
phase = ["start"]
peaks = {}


def jaw_state():
    q = robot.get_joint_positions()
    e = robot.get_measured_joint_efforts()
    return q[JAW], e[JAW]


def step():
    contacts.clear()
    render = (bool(writers) and step_count[0] % FRAME_EVERY == 0) or (args.gui and step_count[0] % 4 == 0)
    world.step(render=render)
    step_count[0] += 1
    t_sim[0] += DT
    if render:
        for k, (a, frames) in writers.items():
            d = a.get_data()
            if d is not None and getattr(d, "size", 0):
                frames.append(np.asarray(d)[:, :, :3].copy())
    # force summaries
    def total(match_a, match_b):
        s = np.zeros(3)
        for (a, b), (f, _) in contacts.items():
            if (match_a in a and match_b in b) or (match_a in b and match_b in a):
                s += f
        return float(np.linalg.norm(s))
    if DEBUG and phase[0] in ("close", "insert") and step_count[0] % 12 == 0:
        for (a, b), (f, cols) in contacts.items():
            if ("Hub" in a+b and phase[0] == "insert" and seat()[0][2] < .0135):
                print("CONTACT", round(t_sim[0], 3), a, b, np.round(f, 1), sorted(c for c in cols if not c.startswith("Pad"))[:8], flush=True)
    vec = np.zeros(3)
    for (a, b), (f, _) in contacts.items():
        if ("/World/Wheel" in a and "/World/Car/Hub" in b) or ("/World/Wheel" in b and "/World/Car/Hub" in a):
            vec += f
    F_VEC[:] = vec
    f_hub = total("/World/Wheel", "/World/Car/Hub")
    f_shoe = total("/World/Tire", "/World/Jaw")
    f_liner = total("/World/T", "/World/Car/Wheelhouse")+total("/World/Wheel", "/World/Car/Wheelhouse")
    f_cal = total("/World/Wheel", "/World/Car/Knuckle")
    f_tool_car = total("/World/Jaw", "/World/Car")+total("/World/Tool", "/World/Car")
    for k, v in (("hub", f_hub), ("shoe", f_shoe), ("wheelhouse", f_liner), ("caliper", f_cal), ("tool_car", f_tool_car)):
        if v > peaks.get(k, (0, ""))[0]:
            peaks[k] = (v, phase[0])
    if step_count[0] % 4 == 0:
        rel, tilt = seat()
        qj, ej = jaw_state()
        Rt, ot = pose("/World/Tire")
        Rw, ow = pose("/World/Wheel")
        log_rows.append(dict(t=round(t_sim[0], 4), phase=phase[0], gap_mm=rel[2]*1000, radial_mm=np.hypot(rel[0], rel[1])*1000,
                             tilt_deg=tilt, phase_err_deg=phase_error(), F_hub=f_hub, F_shoe=f_shoe, F_wheelhouse=f_liner,
                             F_caliper=f_cal, F_tool_car=f_tool_car, jawR_mm=qj[0]*1000, jawL_mm=qj[1]*1000,
                             effR=ej[0], effL=ej[1], wheel_z=ow[2], tire_off_mm=np.linalg.norm(Rw.T@(ot-ow))*1000,
                             hub_y_mm=pose("/World/Car/Hub")[1][1]*1000, knuckle_y_mm=pose("/World/Car/Knuckle")[1][1]*1000,
                             wheel_y_mm=ow[1]*1000))
    return f_hub


q_cmd = np.zeros(len(names))
jaw_vel = np.zeros(len(JAW_DRIVEN))


def command():
    robot.apply_action(ArticulationAction(joint_positions=q_cmd[ARM], joint_indices=np.array(ARM)))
    robot.apply_action(ArticulationAction(joint_velocities=jaw_vel, joint_indices=np.array(JAW_DRIVEN)))


LO = np.array([-3.14, -1.83, -2.70, -6.28, -2.53, -6.28])
HI = np.array([3.14, 2.44, 2.35, 6.28, 2.53, 6.28])
rng_ik = np.random.default_rng(7)


BIAS = dict(R=np.eye(3), p=np.zeros(3))      # actual flange vs model FK, measured at the hub


def solve(R_G, o_G, warm):
    Rf, of = S.flange_pose(R_G, o_G)
    Rf, of = BIAS["R"].T@Rf, of-BIAS["p"]
    seeds = [warm, np.array([0., -.2, -1.7, -1.507, 0, 0])]+[rng_ik.uniform(LO, HI) for _ in range(40)]
    best = None
    for s0 in seeds:
        q, ok = ik.compute_inverse_kinematics("link6", of, S.quat(Rf), warm_start=s0, position_tolerance=2e-4,
                                              orientation_tolerance=2e-3)
        if ok:
            q = np.asarray(q)
            d = float(np.max(np.abs(q-warm)))
            if best is None or d < best[0]:
                best = (d, q)
            if d < .5:
                break
    if best is None:
        raise RuntimeError(f"IK failed at {o_G} ({phase[0]})")
    return refine(best[1], Rf, of)


def fk(q):
    p, R = ik.compute_forward_kinematics("link6", q)
    return np.asarray(R), np.asarray(p)


def pose_error(R, p, Rt, pt):
    dR = Rt@R.T
    ang = np.array([dR[2, 1]-dR[1, 2], dR[0, 2]-dR[2, 0], dR[1, 0]-dR[0, 1]])/2
    return np.concatenate([pt-p, ang])


def refine(q, Rt, pt, iters=6):
    """Damped Newton on the Lula forward kinematics: Lula's own tolerance leaves ~0.6 mm at the wheel."""
    q = np.asarray(q, float).copy()
    for _ in range(iters):
        R, p = fk(q)
        e = pose_error(R, p, Rt, pt)
        if np.linalg.norm(e[:3]) < 2e-6 and np.linalg.norm(e[3:]) < 1e-5:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            R2, p2 = fk(q+dq)
            J[:, i] = (pose_error(R, p, Rt, pt)-pose_error(R2, p2, Rt, pt))/1e-6
        q = q+np.linalg.solve(J.T@J+1e-9*np.eye(6), J.T@e)
    return q


def wait(seconds):
    for _ in range(int(seconds/DT)):
        command()
        step()


def move_joints(q_goal, scale=None):
    scale = scale or args.speed
    q0 = q_cmd[ARM].copy()
    T = max(float(np.max(np.abs(q_goal-q0)/(VMAX*scale))), .4)*1.875     # quintic peak speed factor
    n = int(T/DT)
    for i in range(1, n+1):
        s = i/n
        s = 10*s**3-15*s**4+6*s**5
        q_cmd[ARM] = q0+(q_goal-q0)*s
        command()
        step()


F_HIST = []


def filtered(f, n=12):
    """50 ms moving average of the hub contact force (robot force monitoring is filtered too)."""
    F_HIST.append(f)
    del F_HIST[:-n]
    return sum(F_HIST)/len(F_HIST)


def move_line(R_G, o0, o1, speed, stop=None):
    """Straight line of the wheel centre; IK at 60 Hz, joint targets interpolated in between."""
    L = float(np.linalg.norm(np.asarray(o1)-np.asarray(o0)))
    n = max(int(L/speed*60), 1)
    q_prev = q_cmd[ARM].copy()
    for i in range(1, n+1):
        o = np.asarray(o0)+(np.asarray(o1)-np.asarray(o0))*i/n
        q_next = solve(R_G, o, q_prev)
        sub_n = int(1/60/DT)
        for j in range(1, sub_n+1):
            q_cmd[ARM] = q_prev+(q_next-q_prev)*j/sub_n
            command()
            f = step()
            if stop and stop(f):
                return o
        q_prev = q_next
    return np.asarray(o1)


def force_insert(R_G, o_start, axis, v=.010, t_max=15.):
    """Admittance insertion with the flange F/T sensor: advance along -axis while the filtered axial force is under
    the set point (145 N), lateral compliance 25 N/mm. Seated when the force holds >= 90 % of the set point and the
    wheel stops (< 0.02 mm in 0.3 s). Then the press force is ramped down before the jaws open."""
    K, Fset = P.LATERAL_COMPLIANCE, P.PRESS_FORCE
    s, lat = 0., np.zeros(3)
    Ff = np.zeros(3)
    bodies = list(robot._articulation_view.body_names)
    ti = bodies.index("Tool")

    def ft_world():
        """Flange F/T sensor: incoming-joint force of the tool link, rotated to world."""
        w6 = robot.get_measured_joint_forces()[ti]
        Rt, _ = pose("/World/Tool")
        return Rt@np.asarray(w6[:3], float)
    tare = ft_world()
    q_prev = q_cmd[ARM].copy()
    sub_n = int(1/60/DT)
    gaps, seated, f_ax = [], False, 0.
    still_n = int(P.SEAT_STILL[0]*60)

    def cycle(setpoint):
        nonlocal s, lat, Ff, q_prev, f_ax
        F = -(ft_world()-tare)                            # contact force acting on the wheel (+axis = outboard)
        Ff = Ff+(F-Ff)*.15                                # ~100 ms low-pass at 60 Hz (force-control bandwidth)
        f_ax = float(Ff@axis)
        f_lat = Ff-f_ax*axis
        n_lat = np.linalg.norm(f_lat)
        f_lat = f_lat*max(n_lat-10., 0.)/n_lat if n_lat > 1e-9 else f_lat   # 10 N dead band
        s += v/60*float(np.clip(1-f_ax/setpoint, -.5, 1.))
        goal = np.clip(f_lat/K, -.005, .005)
        lat = lat+np.clip(goal-lat, -.0005/60, .0005/60)  # lateral correction rate <= 0.5 mm/s
        o = o_start-axis*s+lat
        q_next = solve(R_G, o, q_prev)
        for j in range(1, sub_n+1):
            q_cmd[ARM] = q_prev+(q_next-q_prev)*j/sub_n
            command()
            step()
        q_prev = q_next
        return o

    for _ in range(int(t_max*60)):
        o = cycle(Fset)
        gaps.append(seat()[0][2])
        if (f_ax >= P.SEAT_FORCE_RATIO*Fset and len(gaps) > still_n
                and max(gaps[-still_n:])-min(gaps[-still_n:]) < P.SEAT_STILL[1]):
            seated = True
            break
    event("force_insert_end", seated=seated, travel_mm=round(s*1000, 2), lateral_mm=[round(float(v)*1000, 3) for v in lat],
          f_axial_N=round(f_ax, 1))
    if seated:
        for _ in range(int(.6*60)):                       # ramp the press force down before opening the jaws
            o = cycle(P.RELEASE_FORCE)
        event("press_relieved", f_axial_N=round(f_ax, 1))
    return o


def grip(close):
    v_close, v_open = P.GRIP["stroke"]/P.GRIP["t_close"], P.GRIP["stroke"]/P.GRIP["t_open"]
    v = -v_close if close else v_open
    jaw_vel[:] = (v,) if len(jaw_vel) == 1 else (v, -v)


class StopRun(Exception):
    pass


def event(name, **kv):
    rel, tilt = seat()
    e = dict(t=round(t_sim[0], 3), event=name, gap_mm=round(float(rel[2])*1000, 3), tilt_deg=round(float(tilt), 3),
             phase_err_deg=round(float(phase_error()), 3), **kv)
    events.append(e)
    print("EVENT", json.dumps(e, default=float), flush=True)
    if args.stop_after and name == args.stop_after:
        wait(1.0)                                          # one more second of footage
        raise StopRun(name)


# ---------------------------------------------------------------- sequence
result = "incomplete"
try:
    q_cmd[ARM] = robot.get_joint_positions()[ARM]
    grip(close=False)
    phase[0] = "settle"
    wait(1.0)
    _, w0 = pose("/World/Wheel")
    event("settled", wheel_height_mm=round((w0[2]-P.TABLE_TOP)*1000, 2))

    # vision: wheel spin on the table and hub stud angle (truth + noise)
    Rw, ow = pose("/World/Wheel")
    spin = degrees(atan2(Rw[1, 0], Rw[0, 0]))+rng.normal(0, NOISE_DEG)
    Rh, _ = pose("/World/Car/Hub")
    Rk, _ = pose("/World/Car/Knuckle")
    xh = Rk.T@Rh[:, 0]
    hub_deg = degrees(atan2(xh[1], xh[0]))+rng.normal(0, NOISE_DEG)
    g = (36.-hub_deg) % 72.                                  # grip yaw relative to the holes (see scene notes)
    R_pick = S.rot((0, 0, 1), spin+g)
    event("measured", wheel_spin_deg=round(spin, 3), hub_deg=round(hub_deg, 3), grip_yaw_deg=round(g, 3))

    T_CYCLE0 = [t_sim[0]]
    phase[0] = "approach"
    above = ow+np.array([0, 0, .20])
    move_joints(solve(R_pick, above, q_cmd[ARM]))
    phase[0] = "descend"
    move_line(R_pick, above, ow, .25 if FAST else .10)
    Rf, of = pose("/World/Robot/link6")
    Rt_, ot_ = pose("/World/Tool")
    fk_p, fk_R = ik.compute_forward_kinematics("link6", q_cmd[ARM])
    RG = Rf@S.R_FG.T
    oG = of-RG@S.T_GF
    _, ow_now = pose("/World/Wheel")
    event("descended", wheel_in_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ow_now-oG)],
          link6_vs_fk_mm=round(float(np.linalg.norm(np.asarray(fk_p)-of))*1000, 2),
          tool_vs_link6_mm=round(float(np.linalg.norm(ot_-of))*1000, 2),
          link6_z_world=[round(float(v), 3) for v in Rf[:, 2]])
    phase[0] = "close"
    grip(close=True)
    wait(P.GRIP["t_close"]+(.2 if FAST else .6))
    qj, ej = jaw_state()
    Rt0, ot0 = pose("/World/Tire")
    Rj0, oj0 = pose("/World/JawR")
    if args.debug_stop == "gripped":
        raise RuntimeError("debug stop")
    event("gripped", jaw_mm=[round(v*1000, 2) for v in qj], jaw_force_N=[round(v, 1) for v in ej])

    Rf_, of_ = pose("/World/Robot/link6")
    RG_ = Rf_@S.R_FG.T
    _, ow_ = pose("/World/Wheel")
    W_IN_G0 = [RG_.T@(ow_-(of_-RG_@S.T_GF))]
    phase[0] = "lift"
    up = ow+np.array([0, 0, .25])
    move_line(R_pick, ow, up, .25 if FAST else .10)
    wait(.1 if FAST else .3)
    Rt1, ot1 = pose("/World/Tire")
    Rj1, oj1 = pose("/World/JawR")
    slip = np.linalg.norm(Rj1.T@(ot1-oj1)-Rj0.T@(ot0-oj0))*1000
    event("lifted", slip_mm=round(slip, 3))

    phase[0] = "transfer"
    R_m = S.R_MOUNT
    pre = S.H+np.array([0, .12, 0])
    if SC == "joint_transfer":
        move_joints(solve(R_m, pre, q_cmd[ARM]))           # earlier design: one joint-space move to the hub
    else:
        via = S.H+np.array([0, .45, 0])                   # on the hub axis, well outboard of the fender
        move_joints(solve(R_m, via, q_cmd[ARM]), scale=args.transfer_speed)
        move_line(R_m, via, pre, .30 if FAST else .15)
    Rf_, of_ = pose("/World/Robot/link6")
    RG_ = Rf_@S.R_FG.T
    _, ow_ = pose("/World/Wheel")
    w_in_G = RG_.T@(ow_-(of_-RG_@S.T_GF))
    event("at_hub", transfer_slip_mm=round(float(np.linalg.norm(w_in_G-W_IN_G0[0]))*1000, 2),
          wheel_in_G_mm=[round(float(v)*1000, 1) for v in w_in_G])
    wait(.5)

    phase[0] = "align"
    # vision at the hub: wheel pose relative to the hub (bore centre, face normal, hole angle) + noise;
    # the robot knows its own flange pose from the encoders. Re-target the grip frame so the measured wheel
    # lands on the hub axis, parallel, with the holes on the studs. Two passes.
    gap_pre = .12
    for it in range(0 if SC == "novision" else (1 if FAST else 2)):
        Rh, oh = pose("/World/Car/Hub")
        Rw, ow = pose("/World/Wheel")
        ow = ow+rng.normal(0, CENTER_NOISE, 3)
        Rw = Rw@S.rot(rng.normal(0, 1, 3), rng.normal(0, NOISE_DEG))
        Rf, of = pose("/World/Robot/link6")
        RG = Rf@S.R_FG.T
        oG = of-RG@S.T_GF
        R_GW, p_GW = RG.T@Rw, RG.T@(ow-oG)
        x = Rh.T@Rw[:, 0]
        phi = degrees(atan2(x[1], x[0]))
        Rw_des = Rh@S.rot((0, 0, 1), phi-((phi+36.) % 72.-36.))
        ow_des = oh+Rh@np.array([0, 0, gap_pre])
        RG_t = Rw_des@R_GW.T
        oG_t = ow_des-RG_t@p_GW
        # the flange sits ~0.8 mm off the model forward kinematics (drive sag, model): measure that bias
        Rfk, pfk = fk(q_cmd[ARM])
        BIAS["p"] = of-pfk
        BIAS["R"] = Rf@Rfk.T
        move_joints(solve(RG_t, oG_t, q_cmd[ARM]), scale=.2 if FAST else .08)
        wait(.15 if FAST else .4)
        rel, tilt = seat()
        event(f"aligned_{it}", wheel_in_G_mm=[round(float(v)*1000, 1) for v in p_GW],
              radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3))
    if SC == "novision":
        RG_t, oG_t = R_m, pre                              # trust the grip: no measurement at the hub
    Rh, oh = pose("/World/Car/Hub")
    Rw, ow = pose("/World/Wheel")
    from math import cos as _c, sin as _s, tau as _t
    holes = [Rh.T@(ow+Rw@np.array([P.PCD/2*_c(k*_t/5), P.PCD/2*_s(k*_t/5), 0])-oh) for k in range(5)]
    studs = []
    for i in range(5):
        xc.Clear()
        m = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(f"/World/Car/Hub/Stud_{i}"))
        studs.append(Rh.T@(np.array(m.ExtractTranslation())-oh))
    event("hole_check", hole_to_stud_mm=[round(float(min(np.hypot(*(h[:2]-s[:2])) for s in studs))*1000, 3) for h in holes],
          stud_r_mm=[round(float(np.hypot(*s[:2]))*1000, 2) for s in studs], hole_r_mm=[round(float(np.hypot(*h[:2]))*1000, 2) for h in holes])
    phase[0] = "insert"
    axis = Rh[:, 2]
    seated = oG_t-axis*(gap_pre+.001)                        # 1 mm over-travel, stopped by force or seat
    # free approach to 2 mm before the stud tips, then force-controlled insertion
    near = oG_t-axis*(gap_pre-.032)
    halt = lambda f: filtered(f) > F_STOP
    stop_at = move_line(RG_t, oG_t, near, .050 if FAST else .020, stop=halt)
    if SC == "position_insert":
        stop_at = move_line(RG_t, np.asarray(stop_at), seated, .005,
                            stop=lambda f: filtered(f) > F_STOP or seat()[0][2] < .0002)
    else:
        stop_at = force_insert(RG_t, np.asarray(stop_at), axis, v=.020 if FAST else .010)
    wait(.3)
    if args.debug_stop == "inserted":
        raise RuntimeError("debug stop")
    event("inserted", stop_mm_from_seat=round(float((stop_at-seated)@axis)*1000-1, 2))
    R_m2 = RG_t

    phase[0] = "release"
    grip(close=False)
    # shoes clear the tread after ~10 mm of jaw travel: start retracting then (fast) or after full opening
    wait(.90 if FAST else P.GRIP["t_open"]+.3)
    phase[0] = "retract"
    cur = np.asarray(stop_at)
    out = S.H+axis*.45+(cur-S.H-axis*((cur-S.H)@axis))   # back along the hub axis to the transfer via point
    move_line(R_m2, cur, out, .20 if FAST else .05)
    event("retracted")
    phase[0] = "return"
    move_joints(solve(R_pick, above, q_cmd[ARM]))         # back over the pick point: end of one wheel cycle
    event("cycle_end", cycle_s=round(t_sim[0]-T_CYCLE0[0], 2))
    phase[0] = "observe"
    wait(2.0)
    rel, tilt = seat()
    gap_ok, tilt_ok = rel[2] < P.MOUNTED[0], tilt < P.MOUNTED[1]
    result = "mounted" if gap_ok and tilt_ok and np.hypot(rel[0], rel[1]) < .002 else "not_mounted"
    event("final", result=result, radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3))
except StopRun as exc:
    result = f"stopped after {exc}"
except Exception as exc:                                   # keep whatever was recorded
    result = f"error: {exc}"
    print("ERROR", exc, flush=True)

# ---------------------------------------------------------------- outputs
with open(OUT/"log.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()) if log_rows else ["t"])
    w.writeheader()
    w.writerows(log_rows)
summary = dict(result=result, scenario=SC, seed=args.seed, mu_shoe=args.mu_shoe or P.MU_SHOE_TREAD, scene=info, events=events,
               peak_forces_N={k: [round(v, 1), ph] for k, (v, ph) in peaks.items()}, sim_time_s=round(t_sim[0], 2),
               params={k: v["value"] for k, v in P.P.items()})
(OUT/"summary.json").write_text(json.dumps(summary, indent=1, default=str))
if writers:
    import imageio.v2 as imageio
    from PIL import Image
    for k, (_, frames) in writers.items():
        if not frames:
            continue
        try:
            imageio.mimwrite(OUT/f"{k}.mp4", frames, fps=15, quality=7)
        except Exception as exc:
            print("mp4 failed", exc)
        small = [Image.fromarray(fr).resize((480, 270)) for fr in frames[::2]]
        small[0].save(OUT/f"{k}.gif", save_all=True, append_images=small[1:], duration=133, loop=0)
print("RESULT", result, OUT, flush=True)
if args.gui:
    print("GUI: close the Isaac Sim window to exit", flush=True)
    while app.is_running():
        world.step(render=True)
app.close()
