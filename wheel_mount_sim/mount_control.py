"""Mount control in the team cell, shared by phase 1 and phase 2.

Phase 1 (cell_mount_run.py, kept as it was) has everything in one script. This module is the same control split
in two parts so the phase-2 ROS 2 run (cell_mount_ros2.py) can drive it step by step:

- CellEnv: the team cell map with our belt end and mount robots, the car standing kinematic at a dock (A, or B
  after the fake index move), cameras/recording, PhysX stepping at 240 Hz. This is the "robot controller and
  plant" side of a real cell: joint servo, 240 Hz F/T force loop and gripper stay here (not ROS).
- MountSide: one mount robot (RS080N + SCHUNK PHL-W 63-080-S). Its cycle is cut into steps (generators, one
  yield per physics step) that a sequencer can call one after the other:
      prepare -> wait_tire -> measure_pick -> pick -> transfer -> align -> insert -> release_retract -> go_home
  run_cycle() chains them exactly as phase 1 does.

Split from phase 1 with identical events (seed 42, TS-22); since then it carries the 11/28 fixes (pick wrist family,
grip check, jam retry backs off the pilot, pilot-zone lateral rate). Run alone:
    ~/isaacsim/python.sh wheel_mount_sim/mount_control.py [--seed N] [--side both|left|right] [--no-video] [--out DIR]
"""
import json
import os
import sys
import time
from math import atan2, cos, degrees, sin, tau
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

VMAX = np.array([3.1416, 3.1416, 2.7925, 3.2289, 2.8798, 4.8869])   # URDF joint speed limits (rad/s)
LO = np.array([-3.14, -1.83, -2.70, -6.28, -2.53, -6.28])
HI = np.array([3.14, 2.44, 2.35, 6.28, 2.53, 6.28])
F_STOP = 400.                          # N: stop pushing (robot force monitoring, as run.py)
CENTER_NOISE = .00005                  # vision centre noise 1 sigma (as run.py)
NOISE_DEG = .03                        # vision angle noise 1 sigma (as run.py)
R_TUCK = 1.00                          # m, wheel centre from the J1 axis while swinging (cell_mount_run.py)
SWING_SPEED = .70                      # J1-only swing, wheel flat and tucked
EMPTY_SPEED = .80                      # joint moves with the gripper empty
GUI_EVERY = 16                         # GUI redraw every 16 physics steps
FRAME_EVERY = 16                       # 15 video frames per simulated second
REC_EVERY = 8                          # replay.npz at 30 Hz
PILOT_ZONE = (.006, .014)              # m to the seat: the bore chamfer runs onto the pilot tip (jams at 9 mm)
PILOT_SLOW = float(os.environ.get("MOUNT_PILOT_SLOW", "1"))   # speed factor there (1 = phase 1)
PILOT_LAT_RATE = float(os.environ.get("MOUNT_PILOT_LAT_RATE", ".005"))  # m/s lateral admittance rate there
PILOT_DEADBAND = float(os.environ.get("MOUNT_PILOT_DEADBAND", "3"))     # N lateral deadband there (10 elsewhere)
GRIP_HOLD = 1.2                        # s on the tread before lifting (first cycle had 1.25 s)
GRIP_MIN_N = 200.                      # N on the jaw drive before lifting (good grips 430-1760 N)
GRIP_TIMEOUT = 3.                      # s for the jaws to reach the tread
RETRY_CLEAR = .014                     # m: jam retry backs off until the wheel is this far from the seat
RETRY_BACK_MAX = .024                  # m of flange travel at most (studs stay engaged: tips at ~76 mm)
RELEASE_PRESS = float(os.environ.get("MOUNT_RELEASE_PRESS", "0"))   # N while the jaws open (0 = phase 1)
K_DIR = Path.home()/"isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/Kawasaki/rs080n"


class Profile:
    """Speed profile. Default = cycle-time profile (docs/GOALS.md section 1); basic = slow commissioning."""

    def __init__(self, basic=False, speed=None, transfer_speed=None):
        self.fast = not basic
        self.speed = speed or (.50 if self.fast else .35)
        self.transfer = transfer_speed if transfer_speed is not None else (.45 if self.fast else .35)


class StopSide(Exception):
    """A mount step failed. `reason` is one of pick|ik|jam|release|timeout|feed (the /Mount fail code)."""

    def __init__(self, msg, reason="ik"):
        super().__init__(msg)
        self.reason = reason


def _late_imports():
    """Omniverse modules can only be imported after SimulationApp exists."""
    g = globals()
    if "tensors" in g:
        return
    import carb
    import omni.physics.tensors as tensors
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    sys.path.insert(0, str(HERE.parent/"wheel_cell"))
    sys.path.insert(0, str(HERE))
    sys.dont_write_bytecode = True
    import physics
    import process
    import cell_layout
    import params
    import scene
    g.update(carb=carb, tensors=tensors, omni=omni, enable_extension=enable_extension,
             LulaKinematicsSolver=LulaKinematicsSolver, Gf=Gf, PhysicsSchemaTools=PhysicsSchemaTools,
             PhysxSchema=PhysxSchema, Sdf=Sdf, Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics,
             physics=physics, process=process, L=cell_layout, P=params, S=scene)


def _quat_R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


# ====================================================================================== cell / plant side

class CellEnv:
    def __init__(self, app, seed=42, gui=False, video=True, out=None, diag=False, diag_awake=False,
                 usd_writeback=False, replay=False, stop_after=None, car_dynamic=None):
        _late_imports()
        self.app, self.seed, self.gui, self.diag = app, seed, gui, diag
        self.stop_after = stop_after
        self.DT = 1./physics.HZ
        self.HZ = physics.HZ
        self.ROOT = physics.ROOT
        self.out = Path(out) if out else HERE/"output"/"cell"/time.strftime("%Y%m%d-%H%M%S")
        self.out.mkdir(parents=True, exist_ok=True)
        self.t = 0.
        self.events = []
        self.event_log = open(self.out/"events.jsonl", "w")
        self.listeners = []                # callables(event dict): phase 2 publishes /Mount from here
        self.units = []
        self.car_dx = 0.                   # car shift from dock A (fake index move)
        # car body dynamic on the lift pads (AMR deck and lift kinematic = held by the locked dock clamps,
        # assets/README.md 2026-11-28); default kinematic as phase 1
        self.car_dynamic = (os.environ.get("MOUNT_CAR_DYNAMIC", "0") == "1") if car_dynamic is None else car_dynamic

        ctx = omni.usd.get_context()
        ctx.open_stage(str(L.SCENE_DIR/"wheel_cell.usda"))
        for _ in range(30):                                # let the stage and its textures load before building
            app.update()
        stage = self.stage = ctx.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        self.cell = cell = physics.Cell(stage, seed, True, start="gate")
        assert L.active(stage), "cell_layout did not switch on: no belt end stop in the scene"
        self._park_car_at_dock()
        if diag:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Live/MountSim")):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.GetName().startswith(("link", "Tool", "Jaw")):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        no_sleep = os.environ.get("MOUNT_NO_SLEEP", "0") == "1"
        if no_sleep:                       # mount arms never sleep (articulation root)
            for info in cell.mount_arms.values():
                prim = stage.GetPrimAtPath(info["root"])
                if prim:
                    PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateSleepThresholdAttr(0.)
        for s in cell.stations:
            if diag:
                PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(s.tire)).CreateThresholdAttr(0.)
            if diag_awake or no_sleep:
                for path in (s.wheel, s.tire):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateSleepThresholdAttr(0.)

        hub_x = process.DOCK_X+cell.stations[0].hub[0]
        self.cams = {"overview": self._camera("/World/MountCams/Overview", (-4.6, 0., 4.4), (0.1, 0., .7), 18.),
                     "left": self._camera("/World/MountCams/Left", (-2.9, 5.6, 3.1), (-.15, 2.3, .8), 20.),
                     "hub_left": self._camera("/World/MountCams/HubLeft", (hub_x+.9, 2.05, 1.25), (hub_x, .79, .94), 24.)}
        if replay:                         # replay-only views (no extra prims in computing runs)
            px, ly, bz = L.PICK_X, L.LINE_Y, L.BELT_Z
            self.cams.update({
                "pick_left": self._camera("/World/MountCams/PickLeft", (px-1.1, ly+1.3, bz+1.2), (px+.1, ly, bz+.2), 20.),
                "pick_right": self._camera("/World/MountCams/PickRight", (px-1.1, -ly-1.3, bz+1.2), (px+.1, -ly, bz+.2), 20.),
                "hub_right": self._camera("/World/MountCams/HubRight", (hub_x+.9, -2.05, 1.25), (hub_x, -.79, .94), 24.)})
            # close-ups of the hub face, pilot and studs: low and in front of the tire, looking up into the gap
            # between the wheel pad and the hub face (MOUNT_ZOOM="dx,dy,dz,focal" to adjust)
            zx, zy, zz, zf = (float(v) for v in os.environ.get("MOUNT_ZOOM", ".12,.52,.27,30").split(","))
            for st in cell.stations:
                if st.hub[0] <= 0:
                    continue
                hy = st.side*abs(st.hub[1])
                tgt = (hub_x, hy, process.HUB_Z)
                lab = "zoom_left" if st.side > 0 else "zoom_right"
                self.cams[lab] = self._camera(f"/World/MountCams/{lab}", (hub_x+zx, hy+st.side*zy, process.HUB_Z+zz),
                                              tgt, zf)
                from pxr import UsdLux
                lamp = UsdLux.SphereLight.Define(stage, f"/World/MountCams/{lab}_lamp")
                lamp.CreateRadiusAttr(.05)
                lamp.CreateIntensityAttr(float(os.environ.get("MOUNT_ZOOM_LAMP", "60000")))
                UsdGeom.Xformable(lamp).AddTranslateOp().Set(Gf.Vec3d(hub_x+zx+.05, hy+st.side*(zy-.08), process.HUB_Z+zz+.10))
        self.replay_mode = replay
        if gui:
            from omni.kit.viewport.utility import get_active_viewport
            get_active_viewport().camera_path = self.cams["left"]
        self.writers = {}
        if video:
            import omni.replicator.core as rep
            self.rep = rep
            only = [c for c in os.environ.get("MOUNT_CAMS", "").split(",") if c]
            for k, path in self.cams.items():
                if only and k not in only:
                    continue
                rp = rep.create.render_product(path, (960, 540))
                a = rep.AnnotatorRegistry.get_annotator("rgb")
                a.attach(rp)
                self.writers[k] = (a, [])
        self.proxies = []
        if not usd_writeback and not replay:
            self._use_fabric()
            self._make_proxies()
        cell.step()                                        # physics objects exist from here on
        self.sim_view = tensors.create_simulation_view("numpy", cell.stage_id.ToLongInt())
        self.car_offset_mm()                               # reference pose of the car
        self.sim_view.set_subspace_roots("/")
        self.xc = UsdGeom.XformCache()
        self._local = {}
        self.rec_paths = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Live/MountSim"))
                          if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        for s in cell.stations:
            self.rec_paths += [s.wheel, s.tire]
        self.rec_t, self.rec_pose = [], []
        self.step_i = 0
        self.prof = dict(control=0., physics=0., diag=0., render=0.)
        self.wall0 = None

    # ---- setup helpers
    def _car_movers(self):
        stage, ROOT = self.stage, self.ROOT
        movers = [ROOT+"/DockLift"]
        for name in ("FL", "FR", "RL", "RR"):
            for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{ROOT}/{name}")):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() not in ("Wheel", "Tire"):
                    movers.append(str(prim.GetPath()))
        loc = stage.GetPrimAtPath(ROOT+"/HangerLocator")
        if loc:
            movers += [str(p.GetPath()) for p in Usd.PrimRange(loc) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        return movers

    def _park_car_at_dock(self):
        """Car kinematic at dock A (car frame at DOCK_X: front hub at MOUNT_HUB_X, hub height HUB_Z), knuckles and
        hubs moved with it, the AMR deck and lift parked under it. The AMR docking is not simulated."""
        stage, ROOT = self.stage, self.ROOT
        xc = UsdGeom.XformCache()
        car = stage.GetPrimAtPath(ROOT+"/CN7")
        before = xc.GetLocalToWorldTransform(car)
        movers = self._car_movers()
        worlds = {p: xc.GetLocalToWorldTransform(stage.GetPrimAtPath(p)) for p in movers}
        UsdGeom.Xformable(car).GetOrderedXformOps()[0].Set(Gf.Vec3d(process.DOCK_X, 0., 0.))
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
        deck.GetAttribute("xformOp:translate").Set(Gf.Vec3d(process.DOCK_X, t[1], t[2]))
        for path in ((ROOT+"/DockedAMR", ROOT+"/DockLift") if self.car_dynamic else
                     (ROOT+"/CN7", ROOT+"/DockedAMR", ROOT+"/DockLift")):
            UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Set(True)
        self.cell.carrier.step = lambda api, dt: None     # no AMR traction, lift or clamp commands

    def _camera(self, path, eye, target, focal=24.):
        cam = UsdGeom.Camera.Define(self.stage, path)
        cam.CreateFocalLengthAttr(focal)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.05, 100.))
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
        UsdGeom.Xformable(cam).AddTransformOp().Set(m)
        return path

    def _use_fabric(self):
        """After the stage and the cell are built (enabling it before opening the stage crashed in Fabric)."""
        enable_extension("omni.physx.fabric")
        s = carb.settings.get_settings()
        s.set_bool("/physics/updateToUsd", False)
        s.set_bool("/physics/updateVelocitiesToUsd", False)
        s.set_bool("/physics/updateParticlesToUsd", False)
        s.set_bool("/physics/updateResidualsToUsd", False)

    def _make_proxies(self):
        """Render-only copies of the wheels and the gripper bodies, moved to the PhysX pose (Fabric, TS-20)."""
        stage = self.stage
        for s in self.cell.stations:
            vis = stage.GetPrimAtPath(s.wheel+"/Visual")
            if not vis:
                continue
            UsdGeom.Imageable(vis).MakeInvisible()
            proxy = UsdGeom.Xform.Define(stage, f"/World/WheelVisuals/{s.name}")
            proxy.GetPrim().GetReferences().AddInternalReference("/AssetLibrary/Wheel_16")
            ops = (proxy.AddTranslateOp(), proxy.AddOrientOp())
            self.proxies.append((s.wheel, ops))
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
                self.proxies.append((str(body.GetPath()), ops))

    # ---- state
    def pose(self, path):
        """World pose (R, p) from PhysX for bodies and articulation links; for a collider/xform under a body, the
        body pose times the authored local offset; static prims from USD."""
        r = self.cell.api.get_rigidbody_transformation(path)
        if r.get("ret_val"):
            return _quat_R(r["rotation"]), np.array(r["position"], float)
        if path not in self._local:
            prim = self.stage.GetPrimAtPath(path)
            body = prim.GetParent()
            while body and body.GetPath().pathString != "/" and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                body = body.GetParent()
            self.xc.Clear()
            m = self.xc.GetLocalToWorldTransform(prim)
            Rm, pm = np.array(m.ExtractRotationMatrix()).T, np.array(m.ExtractTranslation())
            if body and body.HasAPI(UsdPhysics.RigidBodyAPI):
                mb = self.xc.GetLocalToWorldTransform(body)
                Rb, pb = np.array(mb.ExtractRotationMatrix()).T, np.array(mb.ExtractTranslation())
                self._local[path] = (str(body.GetPath()), Rb.T@Rm, Rb.T@(pm-pb))
            else:
                self._local[path] = (None, Rm, pm)
        body, Rl, pl = self._local[path]
        if body is None:
            return Rl, pl
        Rb, pb = self.pose(body)
        return Rb@Rl, pb+Rb@pl

    def car_offset_mm(self):
        """Car body displacement from where it stood after parking (+ index moves), mm and yaw deg."""
        r = self.cell.api.get_rigidbody_transformation(self.ROOT+"/CN7")
        p = np.array(r["position"], float)
        if not hasattr(self, "_car0"):
            self._car0 = p.copy()
        d = (p-self._car0-np.array([self.car_dx, 0., 0.]))*1000
        R = _quat_R(r["rotation"])
        return [round(float(v), 4) for v in d], round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 5)

    def log_event(self, e):
        self.events.append(e)
        self.event_log.write(json.dumps(e, default=float)+"\n")
        self.event_log.flush()
        print("EVENT", json.dumps(e, default=float), flush=True)
        for fn in self.listeners:
            fn(e)

    def stations(self, front):
        """{side: station} of the front (hub x > 0 in the car frame) or rear wheels."""
        return {s.side: s for s in self.cell.stations if (s.hub[0] > 0) == front}

    # ---- fake index move A -> B (the AMR team owns the real one)
    def car_move(self, dx, T=None):
        """Generator (tick it as a control): move the kinematic car, AMR deck and lift by dx along world x with a
        quintic profile over T s (process.INDEX_TRAVEL, 6 s). Knuckles, hubs and the wheels already on the hubs
        are dynamic and follow through their joints and contacts. Assumption standing in for the AMR transfer."""
        T = T or process.INDEX_TRAVEL
        paths = ([] if self.car_dynamic else [self.ROOT+"/CN7"])+[self.ROOT+"/DockedAMR", self.ROOT+"/DockLift"]
        view = self.sim_view.create_rigid_body_view(paths)
        x0 = np.array(view.get_transforms(), float)
        idx = np.arange(len(paths), dtype=np.int32)
        n = int(T/self.DT)
        self.car_moving = True
        for i in range(1, n+1):
            s = i/n
            s = 10*s**3-15*s**4+6*s**5
            x = x0.copy()
            x[:, 0] += dx*s
            view.set_kinematic_targets(x.astype(np.float32), idx)
            yield
        self.car_dx += dx
        self.car_moving = False

    # ---- stepping
    def tick(self, controls):
        """One 240 Hz step: controls (callables) -> PhysX -> record/diag/sample/render."""
        if self.wall0 is None:
            self.wall0 = time.perf_counter()
        cell, prof = self.cell, self.prof
        t0_ = time.perf_counter()
        for c in controls:
            c()
        tac = time.perf_counter()
        prof["control"] += tac-t0_
        cell.prepare_step(self.DT)
        ta = time.perf_counter()
        cell.sim.simulate(self.DT, cell.seconds)
        cell.sim.fetch_results()
        tb = time.perf_counter()
        cell.finish_step(self.DT)
        prof["prepare"] = prof.get("prepare", 0.)+ta-tac
        prof["phx"] = prof.get("phx", 0.)+tb-ta
        prof["finish"] = prof.get("finish", 0.)+time.perf_counter()-tb
        self.t += self.DT
        self.step_i += 1
        if self.step_i % REC_EVERY == 0:
            self.record()
        tic = time.perf_counter()
        prof["physics"] += tic-tac
        if self.diag and any(u.phase in ("transfer", "approach", "lift", "insert", "release", "retract")
                             for u in self.units):
            self._diag_contacts()
        if self.step_i % 24 == 0:
            for u in self.units:
                if u.phase not in ("wait_tire", "done", "idle"):
                    u.sample()
        tr = time.perf_counter()
        prof["diag"] += tr-tic
        if (self.writers and self.step_i % FRAME_EVERY == 0) or (self.gui and self.step_i % GUI_EVERY == 0):
            self.render()
        prof["render"] += time.perf_counter()-tr
        if self.gui:
            ahead = self.t-(time.perf_counter()-self.wall0)
            if ahead > 0:
                time.sleep(ahead)
        if self.step_i % (5*self.HZ) == 0:
            wall = time.perf_counter()-self.wall0
            print("SPEED", f"sim {self.t:.1f}s wall {wall:.1f}s x{self.t/wall:.2f}",
                  {k: round(v, 1) for k, v in prof.items()}, flush=True)

    def _diag_contacts(self):
        HZ, units = self.HZ, self.units
        heads, data = self.cell.sim.get_contact_report()[:2]
        for u in units:
            u.now_contacts = {}
        for h in heads:
            if not h.num_contact_data:
                continue
            c0, c1 = (str(PhysicsSchemaTools.intToSdfPath(v)) for v in (h.collider0, h.collider1))
            for u in units:
                if u.phase in ("insert", "release", "retract", "observe") and (
                        c0.startswith(u.station.wheel) or c1.startswith(u.station.wheel)
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
                print("HIT", round(self.t, 3), ours[0].phase, a0, a1, round(f, 1), flush=True)

    def sync_proxies(self):
        for body, (op_t, op_q) in self.proxies:
            r = self.cell.api.get_rigidbody_transformation(body)
            if r.get("ret_val"):
                x, y, z, w = r["rotation"]
                op_t.Set(Gf.Vec3d(*r["position"]))
                op_q.Set(Gf.Quatf(float(w), float(x), float(y), float(z)))

    def render(self):
        self.sync_proxies()
        if self.writers:
            self.rep.orchestrator.step(rt_subframes=1, pause_timeline=False)
        else:
            self.app.update()
        for k, (a, frames) in self.writers.items():
            d = a.get_data()
            if d is not None and getattr(d, "size", 0):
                frames.append(np.asarray(d)[:, :, :3].copy())

    def warmup(self):
        if self.writers:
            for _ in range(3):
                self.render()
            print("FRAMES warmup", {k: len(f) for k, (_, f) in self.writers.items()}, flush=True)

    def record(self):
        row = np.zeros((len(self.rec_paths), 7))
        for i, path in enumerate(self.rec_paths):
            r = self.cell.api.get_rigidbody_transformation(path)
            if r.get("ret_val"):
                row[i, :3] = r["position"]
                row[i, 3:] = r["rotation"]                  # x, y, z, w
            else:
                row[i, 6] = np.nan
        self.rec_t.append(self.t)
        self.rec_pose.append(row)

    # ---- end of run
    def finish(self, extra=None):
        """trace/summary/replay.npz/videos into self.out; returns the summary dict."""
        out = self.out
        L_ = L
        queue = {}
        for s in self.cell.stations:
            if s.hub[0] < 0:
                p = np.array(self.cell.api.get_rigidbody_transformation(s.wheel)["position"])
                queue[s.name] = dict(phase=s.phase, along_mm=round(float(p[0]-L_.PICK_X)*1000, 2),
                                     across_mm=round(float(p[1]-s.side*L_.LINE_Y)*1000, 2))
        for u in self.units:
            (out/f"trace_{u.label}.json").write_text(json.dumps(u.trace))
        u0 = self.units[0] if self.units else None
        summary = dict(results={u.label: u.result for u in self.units}, seed=self.seed,
                       fast=u0.prof.fast if u0 else None, speed=u0.prof.speed if u0 else None,
                       sim_time_s=round(self.t, 2), next_tires=queue, events=self.events)
        if extra:
            summary.update(extra)
        (out/"summary.json").write_text(json.dumps(summary, indent=1, default=float))
        if self.rec_t:
            np.savez_compressed(out/"replay.npz", t=np.array(self.rec_t), pose=np.array(self.rec_pose, np.float32),
                                paths=np.array(self.rec_paths))
        self.event_log.close()
        if self.writers:
            from PIL import Image
            for k, (_, frames) in self.writers.items():
                if not frames:
                    continue
                try:
                    import imageio.v2 as imageio
                    imageio.mimwrite(out/f"{k}.mp4", frames, fps=15, quality=7)
                except Exception as exc:
                    print("mp4 skipped:", exc)
                small = [Image.fromarray(fr).resize((400, 225)).quantize(96) for fr in frames[::3]]
                small[0].save(out/f"{k}.gif", save_all=True, append_images=small[1:], duration=200, loop=0,
                              optimize=True)
                Image.fromarray(frames[len(frames)//2]).save(out/f"{k}_mid.png")
                Image.fromarray(frames[-1]).save(out/f"{k}_end.png")
        print("RESULT", json.dumps(summary["results"]), "next", json.dumps(queue), out, flush=True)
        return summary


# ====================================================================================== one mount robot

class MountSide:
    """One mount robot. Each step method is a generator (one yield per physics step); `tick()` advances the
    current program by one step and holds the last command when none is loaded."""

    def __init__(self, env, label, info, station, prof):
        self.env, self.label, self.station, self.prof = env, label, station, prof
        self.side = info["side"]
        self.prefix = info["prefix"]
        self.av = env.sim_view.create_articulation_view(info["root"])
        meta = self.av.shared_metatype
        self.dofs = list(meta.dof_names)
        self.JR = self.dofs.index("JawRSlide")
        self.ti = list(meta.link_names).index("Tool")
        self.q = np.array(self.av.get_dof_positions()[0], float)
        self.jaw_v = 0.
        self.ik = LulaKinematicsSolver(robot_description_path=str(K_DIR/"rmpflow/rs080n_robot_description.yaml"),
                                       urdf_path=str(K_DIR/"rs080n_onrobot_rg2.urdf"))
        self.base = np.asarray(info["base"], float)
        self.ik.set_robot_base_pose(robot_position=self.base,
                                    robot_orientation=np.array(S.quat(S.rot((0, 0, 1), info["yaw"]))))
        self.bias = dict(R=np.eye(3), p=np.zeros(3))
        self.rng = np.random.default_rng([env.seed, self.side+2])
        self.phase = "wait_tire"
        self.trace = []
        self.result = "incomplete"
        self.fail_reason = None
        self.results = {}                  # wheel name -> mounted | not_mounted | stopped: ...
        self.mounted_station = None        # station whose wheel this robot left on a hub
        self.R_mount = S.rot((1, 0, 0), -self.side*90.)@S.rot((0, 0, 1), 180. if self.side > 0 else 0.)
        self.IDX0 = np.array([0], dtype=np.int32)
        self._prog = None
        env.units.append(self)

    @property
    def contact(self):
        return self.env.ROOT+"/CN7/Contact_"+self.station.name

    def pose(self, path):
        return self.env.pose(path)

    # ---- program runner (phase 2 sequencer loads one step at a time)
    def load(self, gen):
        self._prog = gen

    def busy(self):
        return self._prog is not None

    def tick(self):
        if self._prog is None:
            self.command()
            return
        try:
            next(self._prog)
        except StopIteration:
            self._prog = None
            self.command()

    # ---- low level
    def command(self):
        q = self.av.get_dof_positions().copy()
        q[0, :6] = self.q[:6]
        self.av.set_dof_position_targets(q.astype(np.float32), self.IDX0)
        v = np.zeros((1, len(self.dofs)), np.float32)
        v[0, self.JR] = self.jaw_v
        self.av.set_dof_velocity_targets(v, self.IDX0)

    def event(self, name, **kv):
        rel, tilt = self.seat()
        e = dict(t=round(self.env.t, 3), side=self.label, event=name, gap_mm=round(float(rel[2])*1000, 3),
                 tilt_deg=round(float(tilt), 3), phase_err_deg=round(float(self.phase_error()), 3),
                 wheel=self.station.name, **kv)
        self.env.log_event(e)
        if self.env.stop_after == name:
            raise StopSide(f"stop after {name}", "stop")

    def seat(self):
        Rh, oh = self.pose(self.station.hub_path)
        Rw, ow = self.pose(self.station.wheel)
        return Rh.T@(ow-oh), degrees(np.arccos(np.clip(Rh[:, 2]@Rw[:, 2], -1, 1)))

    def phase_error(self):
        Rh, _ = self.pose(self.station.hub_path)
        Rw, _ = self.pose(self.station.wheel)
        x = Rh.T@Rw[:, 0]
        return (degrees(atan2(x[1], x[0]))+36.) % 72.-36.

    # ---- planning (IK / trajectories): the phase-2 planning node would own these
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
            q, ok = self.ik.compute_inverse_kinematics("link6", of, S.quat(Rf), warm_start=s0,
                                                       position_tolerance=2e-4, orientation_tolerance=2e-3)
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
            raise StopSide(f"IK failed at {np.round(o_G, 3)} ({self.phase})", "ik")
        return min(qs, key=key or (lambda q: float(np.max(np.abs(q-warm)))))

    def swing_key(self, q0):
        """Prefer the solution whose interpolated flange path has the lowest mean x (swing through -x; J1 hard
        stop straight back, fastening robot on +x), same posture family as at the pick."""
        def key(q):
            family = 0. if (q[2] < 0 and abs(q[1]) < np.radians(60)) else 100.
            xs = [self.fk(q0+(q-q0)*t)[1][0] for t in np.linspace(0, 1, 9)]
            return family+float(np.mean(xs))+.01*float(np.max(np.abs(q-q0)))
        return key

    def pick_key(self, q0, closest=False):
        """Pick postures: same arm family as swing_key and the wrist NOT flipped (J5 < 0, as at the grip). The
        pre-pick pose came out with the flipped wrist (J4 +-180, J5 +74 deg) for some seeds; the joint move to the
        grip pose then turned J4 through 180 deg and swept the jaws over the tire (sw0_s1, sw0_s3 Right: jaws forced
        to 25-55 mm, link6 225 mm off target, wheel left on the belt)."""
        base = self.swing_key(q0)

        def key(q):
            wrist = 0. if q[4] < 0 else 100.
            if closest:
                family = 0. if (q[2] < 0 and abs(q[1]) < np.radians(60)) else 100.
                return wrist+family+float(np.max(np.abs(q-q0)))
            return wrist+base(q)
        return key

    # ---- sensing
    def sample(self):
        """0.1 s trace: wheel and tire in the grip frame, jaw positions and efforts, flange speed."""
        RG, oG = self.grip_frame()
        _, ow = self.pose(self.station.wheel)
        _, ot = self.pose(self.station.tire)
        jq = self.av.get_dof_positions()[0]
        jf = self.av.get_dof_projected_joint_forces()[0] if hasattr(self.av, "get_dof_projected_joint_forces") else [0]*8
        rel, tilt = self.seat()
        self.trace.append(dict(t=round(self.env.t, 2), phase=self.phase, gap_mm=round(float(rel[2])*1000, 3),
                               radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3), tilt=round(float(tilt), 3),
                               wheel_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ow-oG)],
                               tire_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ot-oG)],
                               jaw_mm=[round(float(jq[self.JR])*1000, 2), round(float(jq[self.JR+1])*1000, 2)],
                               jaw_N=round(float(jf[self.JR]), 1),
                               q_deg=[round(float(v), 1) for v in np.degrees(jq[:6])]))

    def ft_world(self):
        w = self.av.get_link_incoming_joint_force()[0, self.ti]
        Rt, _ = self.pose(self.prefix+"/Tool")
        return Rt@np.asarray(w[:3], float)

    def grip_frame(self):
        Rf, of = self.pose(self.prefix+"/Arm/link6")
        RG = Rf@S.R_FG.T
        return RG, of-RG@S.T_GF

    def vision_target(self, gap):
        """Vision at the hub: wheel pose relative to the hub (+ noise) -> grip-frame target that puts the measured
        wheel on the hub axis, parallel, holes on studs, `gap` out from the seat. Also re-measures the flange bias."""
        st = self.station
        Rh, oh = self.pose(st.hub_path)
        Rw, ow = self.pose(st.wheel)
        ow = ow+self.rng.normal(0, CENTER_NOISE, 3)
        Rw = Rw@S.rot(self.rng.normal(0, 1, 3), self.rng.normal(0, NOISE_DEG))
        RG, oG = self.grip_frame()
        R_GW, p_GW = RG.T@Rw, RG.T@(ow-oG)
        x = Rh.T@Rw[:, 0]
        phi = degrees(atan2(x[1], x[0]))
        Rw_des = Rh@S.rot((0, 0, 1), phi-((phi+36.) % 72.-36.))
        ow_des = oh+Rh@np.array([0, 0, gap])
        RG_t = Rw_des@R_GW.T
        Rf, of = self.pose(self.prefix+"/Arm/link6")
        Rfk, pfk = self.fk(self.q[:6])
        self.bias["p"] = of-pfk
        self.bias["R"] = Rf@Rfk.T
        return RG_t, ow_des-RG_t@p_GW

    # ---- motion primitives (robot controller side)
    def wait(self, seconds):
        for _ in range(int(seconds/self.env.DT)):
            self.command()
            yield

    def move_joints(self, q_goal, scale=None):
        scale = scale or self.prof.speed
        q0 = self.q[:6].copy()
        T = max(float(np.max(np.abs(q_goal-q0)/(VMAX*scale))), .4)*1.875
        n = int(T/self.env.DT)
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
        sub_n = int(self.env.HZ/60)
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

    def grip(self, close):
        v_close, v_open = P.GRIP["stroke"]/P.GRIP["t_close"], P.GRIP["stroke"]/P.GRIP["t_open"]
        self.jaw_v = -v_close if close else v_open

    def force_insert(self, R_G, o_start, axis, v, expect):
        """Admittance insertion on the flange F/T sensor (145 N set point, 25 N/mm lateral, seat = >= 90 % force
        and < 0.02 mm in 0.3 s, then the press force is ramped down). 240 Hz loop in the robot controller."""
        FAST = self.prof.fast
        Kl, Fset = P.LATERAL_COMPLIANCE, P.PRESS_FORCE
        st = dict(s=0., lat=np.zeros(3), Ff=np.zeros(3), f_ax=0., q_prev=self.q[:6].copy())
        tare = self.ft_world()
        sub_n = int(self.env.HZ/60)
        still_n = int(P.SEAT_STILL[0]*60)
        gaps, seated = [], False

        def cycle(setpoint):
            F = -(self.ft_world()-tare)
            st["Ff"] = st["Ff"]+(F-st["Ff"])*.15
            f_ax = float(st["Ff"]@axis)
            st["f_ax"] = f_ax
            f_lat = st["Ff"]-f_ax*axis
            left = expect-st["s"]                          # travel to the seat known from vision
            in_pilot = PILOT_ZONE[0] < left < PILOT_ZONE[1]
            n_lat = np.linalg.norm(f_lat)
            dead, rate = (PILOT_DEADBAND, PILOT_LAT_RATE) if in_pilot else (10., .0005)
            f_lat = f_lat*max(n_lat-dead, 0.)/n_lat if n_lat > 1e-9 else f_lat
            slow = PILOT_SLOW if in_pilot else 1.
            st["s"] += v*slow/60*float(np.clip(1-f_ax/setpoint, -.5, 1.))
            goal = np.clip(f_lat/Kl, -.005, .005)
            st["lat"] = st["lat"]+np.clip(goal-st["lat"], -rate/60, rate/60)
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
                # stopping well short of the travel known from vision under full force is a jam, not a seat
                seated = st["s"] >= expect-.003
                break
        top = sorted(getattr(self, "now_contacts", {}).items(), key=lambda kv: -kv[1][0])[:10]
        self.event("force_insert_end", seated=seated, car_mm_yaw=self.env.car_offset_mm(), expected_mm=round(expect*1000, 2), contacts_N_sepmm=top,
                   travel_mm=round(st["s"]*1000, 2), lateral_mm=[round(float(x)*1000, 3) for x in st["lat"]],
                   f_axial_N=round(st["f_ax"], 1))
        if seated:
            for _ in range(int((.4 if FAST else .6)*60)):
                yield from cycle(max(RELEASE_PRESS, P.RELEASE_FORCE))
            self.event("press_relieved", f_axial_N=round(st["f_ax"], 1))
        self.stopped_at = st["o"]
        self.seated = seated

    def hold_press(self, R_G, o_start, axis, force, seconds, v=.010, s_max=.002):
        """Axial force control only (no lateral): keep pressing the wheel on the hub with `force` N for `seconds`
        while something else happens (jaws opening). Travel limited to s_max past o_start, so once the shoes are
        off the tire the flange cannot run on toward the car."""
        st = dict(s=0., Ff=np.zeros(3), q_prev=self.q[:6].copy())
        tare = getattr(self, "tare_free", None)
        if tare is None:
            tare = np.zeros(3)
        sub_n = int(self.env.HZ/60)
        f_log = []
        for _ in range(int(seconds*60)):
            F = -(self.ft_world()-tare)
            st["Ff"] = st["Ff"]+(F-st["Ff"])*.15
            f_ax = float(st["Ff"]@axis)
            f_log.append(f_ax)
            st["s"] = float(np.clip(st["s"]+v/60*float(np.clip(1-f_ax/force, -.5, 1.)), -.002, s_max))
            o = o_start-axis*st["s"]
            q_next = self.solve(R_G, o, st["q_prev"])
            for j in range(1, sub_n+1):
                self.q[:6] = st["q_prev"]+(q_next-st["q_prev"])*j/sub_n
                self.command()
                yield
            st["q_prev"] = q_next
        self.stopped_at = o_start-axis*st["s"]
        self.hold_log = dict(f_first=round(f_log[0], 1) if f_log else 0., f_last=round(f_log[-1], 1) if f_log else 0.,
                             travel_mm=round(st["s"]*1000, 2))

    # ================================================================== cycle steps (sequencer calls these)
    def prepare(self):
        """Empty gripper over the pick point (cycle steady state: the robot ends each cycle there)."""
        pre_pick = np.array([L.PICK_X, self.side*L.LINE_Y, L.BELT_Z+.1025+.20])
        q_pre = self.solve(S.rot((0, 0, 1), 90.), pre_pick, self.q[:6], key=self.pick_key(self.q[:6]))
        yield from self.move_joints(q_pre, scale=EMPTY_SPEED)

    def wait_tire(self, timeout=None):
        st = self.station
        self.phase = "wait_tire"
        t_end = None if timeout is None else self.env.t+timeout
        while st.phase != "pick_ready":
            if st.phase in ("feed_failed",):
                raise StopSide(f"feed {st.phase}", "feed")
            if t_end is not None and self.env.t > t_end:
                raise StopSide("tire wait timeout", "timeout")
            self.command()
            yield
        self.event("pick_ready", along_mm=round(getattr(st.zone, "along_mm", 0.), 2),
                   across_mm=round(getattr(st.zone, "across_mm", 0.), 2))

    def measure_pick(self):
        """Vision at the pick point (overhead camera): wheel centre and spin; hub stud angle at the car. Not a
        generator (no time passes). Sets the grip yaws for pick()."""
        st = self.station
        self.t0 = self.env.t
        Rw, ow = self.pose(st.wheel)
        ow = ow+self.rng.normal(0, CENTER_NOISE, 3)*np.array([1, 1, 0])
        spin = degrees(atan2(Rw[1, 0], Rw[0, 0]))+self.rng.normal(0, NOISE_DEG)
        Rh, _ = self.pose(st.hub_path)
        Rk, _ = self.pose(self.contact)
        xh = Rk.T@Rh[:, 0]
        hub_deg = degrees(atan2(xh[1], xh[0]))+self.rng.normal(0, NOISE_DEG)
        # the wheel x axis ends up at (180 - g) in the knuckle frame on the left (R_mount has a 180 deg turn about
        # the hub axis) and at -g on the right, and must sit on a stud
        g = ((180. if self.side > 0 else 0.)-hub_deg) % 72.
        # the 5 holes leave 5 grip yaws; take the one with the jaws closest to across the flow (world y)
        self.yaws = sorted((spin+g+72.*k for k in range(5)), key=lambda a: abs((a-90.+90.) % 180.-90.))
        self.ow_pick = ow
        self.event("measured", wheel_spin_deg=round(spin, 3), hub_deg=round(hub_deg, 3), grip_yaw_deg=round(g, 3),
                   jaw_axis_deg=round(self.yaws[0] % 180., 2))

    def pick(self):
        FAST = self.prof.fast
        st = self.station
        ow = self.ow_pick
        self.phase = "approach"
        above = ow+np.array([0, 0, .20])
        R_pick, q_above = None, None
        for a in self.yaws[:3]:
            try:
                R_pick = S.rot((0, 0, 1), a)
                q_above = self.solve(R_pick, above, self.q[:6], key=self.pick_key(self.q[:6], closest=True))
                break
            except StopSide:
                continue
        if q_above is None:
            raise StopSide("no pick pose", "pick")
        self.R_pick, self.above = R_pick, above
        self.event("approach_plan", dq_deg=[round(float(v), 1) for v in np.degrees(q_above-self.q[:6])])
        yield from self.move_joints(q_above, scale=EMPTY_SPEED)
        self.phase = "descend"
        yield from self.move_line(R_pick, above, ow, .25 if FAST else .10)
        RG, oG = self.grip_frame()
        _, ow_now = self.pose(st.wheel)
        _, ot_now = self.pose(st.tire)
        Rf, of = self.pose(self.prefix+"/Arm/link6")
        self.event("descended", wheel_in_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ow_now-oG)],
                   tire_in_G_mm=[round(float(v)*1000, 1) for v in RG.T@(ot_now-oG)],
                   link6_vs_fk_mm=round(float(np.linalg.norm(self.fk(self.q[:6])[1]-of))*1000, 2),
                   link6_vs_target_mm=round(float(np.linalg.norm(S.flange_pose(R_pick, ow)[1]-of))*1000, 2),
                   z_G_world=[round(float(v), 3) for v in RG[:, 2]], x_G_world=[round(float(v), 3) for v in RG[:, 0]])
        self.phase = "close"
        self.grip(True)
        # wait for the jaws to stop on the tread, then hold: a fixed t_close+0.2 s was enough only when the jaws
        # started near the tire (first cycle). After a release the jaws are 55 mm open, closing took 0.75 s of the
        # 1.29 s and the rear wheel was lifted 0.54 s after contact with 4-580 N (car42 RL: lifted 24 mm only)
        t_c0, last, still = self.env.t, None, 0
        while self.env.t-t_c0 < GRIP_TIMEOUT:
            jq = float(self.av.get_dof_positions()[0][self.JR])
            still = still+1 if (last is not None and abs(jq-last) < 2e-5) else 0
            last = jq
            self.command()
            yield
            if still >= 12:                               # 0.05 s without jaw motion
                break
        t_contact = self.env.t-t_c0
        yield from self.wait(GRIP_HOLD if FAST else GRIP_HOLD+.4)
        # grip check before lifting: some second-cycle grips stopped at jaw 0.33-0.35 mm with 0-8 N on the drive
        # (carNS42 RL, car1 RR; good grips 0.19-0.24 mm, 430-1760 N) and the wheel came up only 23 mm, tilted
        for attempt in range(1, 3):
            jf = float(self.av.get_dof_projected_joint_forces()[0][self.JR])
            if abs(jf) >= GRIP_MIN_N:
                break
            jq = float(self.av.get_dof_positions()[0][self.JR])
            self.event("regrip", attempt=attempt, jaw_N=round(jf, 1), jaw_mm=round(jq*1000, 2))
            self.grip(False)
            q_open = jq+.010
            while float(self.av.get_dof_positions()[0][self.JR]) < q_open:
                self.command()
                yield
            self.grip(True)
            yield from self.wait(GRIP_HOLD)
        jf = float(self.av.get_dof_projected_joint_forces()[0][self.JR])
        if abs(jf) < GRIP_MIN_N:
            raise StopSide(f"no grip force ({jf:.0f} N)", "pick")
        jq = self.av.get_dof_positions()[0]
        self.event("gripped", close_s=round(t_contact, 3), jaw_mm=[round(float(jq[self.JR])*1000, 2),
                                      round(float(jq[self.dofs.index("JawLSlide")])*1000, 2)])
        RG, oG = self.grip_frame()
        _, ow_ = self.pose(st.wheel)
        self.w_in_G0 = RG.T@(ow_-oG)
        self.phase = "lift"
        self.up = ow+np.array([0, 0, .25])
        self.clear = ow+np.array([0, 0, .05])              # straight up off the belt and the next tire first
        yield from self.move_line(R_pick, ow, self.clear, .10)
        _, ow_up = self.pose(st.wheel)
        rise = float(ow_up[2]-ow[2])
        if rise < .03:                                     # gripper went up 50 mm without the wheel
            self.event("pick_lost", rise_mm=round(rise*1000, 1))
            raise StopSide(f"wheel not lifted ({rise*1000:.1f} mm)", "pick")
        st.zone.picked()                                   # the next tire moves into the stop
        self.event("lifted", rise_mm=round(rise*1000, 1))

    def transfer(self):
        FAST = self.prof.fast
        st = self.station
        self.phase = "transfer"
        Rh, oh = self.pose(st.hub_path)
        axis = Rh[:, 2]
        via = oh+axis*.45
        self.pre = pre = oh+axis*.12
        q_via = self.solve(self.R_mount, via, self.q[:6], key=self.swing_key(self.q[:6]))
        # 1) tuck the flat wheel in to R_TUCK, 2) swing on J1 only through the -x side, 3) stand it up to the via
        d = (self.up-self.base)[:2]
        tuck = np.array([*(self.base[:2]+d/np.linalg.norm(d)*R_TUCK), self.up[2]])
        # all the way up first: pulling in has a +x part toward the next tire on the belt (fastL8)
        yield from self.move_line(self.R_pick, self.clear, self.up, .25 if FAST else .10)
        yield from self.move_line(self.R_pick, self.up, tuck, .40 if FAST else .15)
        self.q_tuck = self.q[:6].copy()
        q_swing = self.q[:6].copy()
        q_swing[0] = q_via[0]
        yield from self.move_joints(q_swing, scale=SWING_SPEED if FAST else self.prof.transfer)
        self.event("swung", j1_deg=round(float(np.degrees(q_via[0])), 1))
        yield from self.move_joints(q_via, scale=self.prof.transfer)
        yield from self.move_line(self.R_mount, via, pre, .30 if FAST else .15)
        RG, oG = self.grip_frame()
        _, ow_ = self.pose(st.wheel)
        self.event("at_hub", transfer_slip_mm=round(float(np.linalg.norm(RG.T@(ow_-oG)-self.w_in_G0))*1000, 2))
        yield from self.wait(.1 if FAST else .5)

    def align(self):
        """Two vision passes at the hub (one left 0.064-0.074 mm and jammed at 9 mm in 2 of 4 runs)."""
        FAST = self.prof.fast
        st = self.station
        self.phase = "align"
        self.gap_pre = gap_pre = .12
        RG_t, oG_t = self.R_mount, self.pre
        for it in range(2):
            Rh, oh = self.pose(st.hub_path)
            Rw, ow = self.pose(st.wheel)
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
            Rf, of = self.pose(self.prefix+"/Arm/link6")
            Rfk, pfk = self.fk(self.q[:6])
            self.bias["p"] = of-pfk
            self.bias["R"] = Rf@Rfk.T
            yield from self.move_joints(self.solve(RG_t, oG_t, self.q[:6]), scale=.2 if FAST else .08)
            yield from self.wait(.10 if FAST else .4)
            rel, _ = self.seat()
            self.event(f"aligned_{it}", radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3))
        self.RG_t, self.oG_t = RG_t, oG_t
        Rh, oh = self.pose(st.hub_path)
        Rw, ow = self.pose(st.wheel)
        holes = [Rh.T@(ow+Rw@np.array([P.PCD/2*cos(k*tau/5), P.PCD/2*sin(k*tau/5), 0])-oh) for k in range(5)]
        studs = [self._stud_local(st, i) for i in range(5)]
        self.event("hole_check", hole_to_stud_mm=[round(float(min(np.hypot(*(h[:2]-s[:2])) for s in studs))*1000, 3)
                                                  for h in holes])

    def _stud_local(self, st, i):
        """Stud position in the hub frame from the authored USD (static offset under the hub body)."""
        key = (st.hub_path, i)
        cache = self.env.__dict__.setdefault("_stud", {})
        if key not in cache:
            xc = UsdGeom.XformCache()
            mh = xc.GetLocalToWorldTransform(self.env.stage.GetPrimAtPath(st.hub_path))
            ms = xc.GetLocalToWorldTransform(self.env.stage.GetPrimAtPath(f"{st.hub_path}/Stud_{i}"))
            rel = ms*mh.GetInverse()
            cache[key] = np.array(rel.ExtractTranslation())
        return cache[key]

    def insert(self):
        FAST = self.prof.fast
        st = self.station
        self.phase = "insert"
        Rh, oh = self.pose(st.hub_path)
        axis = Rh[:, 2]
        self.oh, self.axis = oh, axis
        RG_t, oG_t, gap_pre = self.RG_t, self.oG_t, self.gap_pre
        near = oG_t-axis*(gap_pre-.032)
        tare = self.ft_world()
        self.tare_free = tare
        yield from self.move_line(RG_t, oG_t, near, .100 if FAST else .020,
                                  stop=lambda: np.linalg.norm(self.ft_world()-tare) > F_STOP)
        expect = float((np.asarray(self.stopped_at)-(oG_t-axis*gap_pre))@axis)      # travel left to the seat
        yield from self.force_insert(RG_t, np.asarray(self.stopped_at), axis, v=.020 if FAST else .010, expect=expect)
        # jam recovery: back off out of the pilot (still on the studs), measure again and press again
        for attempt in range(1, 3):
            if self.seated:
                break
            self.event("jam_retry", attempt=attempt)
            # back off until the bore is off the pilot (tip at 12 mm): backing the flange 5 mm left the wheel at
            # 8.5-11.8 mm (the tire stretched in the shoes), still on the pilot, and it re-jammed at the same gap
            o_now = np.asarray(self.stopped_at)
            gap_back = float(self.seat()[0][2])
            backed = 0.
            while gap_back < RETRY_CLEAR and backed < RETRY_BACK_MAX-1e-9:
                yield from self.move_line(RG_t, o_now, o_now+axis*.004, .010)
                o_now, backed = o_now+axis*.004, backed+.004
                yield from self.wait(.05)
                gap_back = float(self.seat()[0][2])
            self.event("jam_backed", backed_mm=round(backed*1000, 1), gap_back_mm=round(gap_back*1000, 2))
            if gap_back < RETRY_CLEAR:
                raise StopSide(f"wheel stuck on the pilot at {gap_back*1000:.1f} mm", "jam")
            RG_t, oG_t = self.vision_target(gap_back)
            yield from self.move_joints(self.solve(RG_t, oG_t, self.q[:6]), scale=.2 if FAST else .08)
            yield from self.wait(.1)
            yield from self.force_insert(RG_t, oG_t, axis, v=.010, expect=gap_back)
        self.RG_t = RG_t
        if not self.seated:                                # still jammed after the retries: hold it, alarm
            raise StopSide("not seated after jam retries", "jam")
        yield from self.wait(.1 if FAST else .3)
        self.stop_at = np.asarray(self.stopped_at)
        self.event("inserted")

    def release_retract(self):
        FAST = self.prof.fast
        self.phase = "release"
        oh, axis, stop_at, RG_t = self.oh, self.axis, self.stop_at, self.RG_t
        self.grip(False)
        if RELEASE_PRESS:
            # keep pressing through the outboard shoe lip while the jaws open (wheel tipped 1.6 deg in the first
            # 0.2 s of opening when the press had been relieved to 18 N, sw0_s2 Right)
            yield from self.hold_press(RG_t, stop_at, axis, RELEASE_PRESS, .90 if FAST else P.GRIP["t_open"]+.3)
            stop_at = self.stop_at
            self.event("released_pressing", **self.hold_log)
        else:
            yield from self.wait(.90 if FAST else P.GRIP["t_open"]+.3)
        self.mounted_station = self.station
        self.phase = "retract"
        out = oh+axis*.45+(stop_at-oh-axis*((stop_at-oh)@axis))
        off_studs = stop_at+axis*.04                       # slow until the shoes are clear of the tread
        yield from self.move_line(RG_t, stop_at, off_studs, .10 if FAST else .05)
        yield from self.move_line(RG_t, off_studs, out, .50 if FAST else .10)
        self.event("retracted")

    def go_home(self):
        """Tuck at the hub side, swing back, then over the pick point in one move."""
        self.phase = "return"
        self.bias = dict(R=np.eye(3), p=np.zeros(3))
        q_back = self.q_tuck.copy()
        q_back[0] = self.q[0]
        yield from self.move_joints(q_back, scale=EMPTY_SPEED)
        yield from self.move_joints(self.solve(self.R_pick, self.above, self.q_tuck), scale=EMPTY_SPEED)
        self.event("cycle_end", cycle_s=round(self.env.t-self.t0, 2))

    def judge(self, observe=2.):
        """Wait, then judge the wheel on the hub (MOUNTED limits from params) and log 'final'."""
        st = self.station
        self.phase = "observe"
        yield from self.wait(observe)
        rel, tilt = self.seat()
        ok = rel[2] < P.MOUNTED[0] and tilt < P.MOUNTED[1] and np.hypot(rel[0], rel[1]) < .002
        self.result = "mounted" if ok else "not_mounted"
        self.results[st.name] = self.result
        if not ok and self.fail_reason is None:
            self.fail_reason = "release"
        top = sorted(getattr(self, "last_hang", {}).items(), key=lambda kv: -kv[1][0])[:14]
        Rw, _ = self.pose(st.wheel)
        lean = Rw[:, 2]                                     # wheel axis in the world: z > 0 = outer end up
        self.event("final", result=self.result, car_mm_yaw=self.env.car_offset_mm(), radial_mm=round(float(np.hypot(rel[0], rel[1]))*1000, 3),
                   rel_mm=[round(float(v)*1000, 3) for v in rel], lean_axis=[round(float(v), 4) for v in lean],
                   contacts_N_sepmm=top)
        self.phase = "done"

    def run_cycle(self, judge=True):
        """Phase-1 order of the steps."""
        yield from self.prepare()
        yield from self.wait_tire()
        self.measure_pick()
        yield from self.pick()
        yield from self.transfer()
        yield from self.align()
        yield from self.insert()
        yield from self.release_retract()
        yield from self.go_home()
        if judge:
            yield from self.judge()

    def guarded(self, gen):
        """Run a step program; a StopSide ends the cycle with result 'stopped: ...' and a fail reason, then the
        robot holds its last command."""
        try:
            yield from gen
        except StopSide as exc:
            self.result = f"stopped: {exc}"
            self.fail_reason = exc.reason
            self.results[self.station.name] = self.result
            print("ERROR", self.label, exc, flush=True)
            self.event("failed", reason=exc.reason, message=str(exc))


def replay(env, run_dir):
    """Play a finished run (its replay.npz) in the window at exactly real time, no physics."""
    app, stage, xc = env.app, env.stage, env.xc
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
        return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(float(w), float(x), float(y), float(z))),
                           Gf.Vec3d(*(float(v) for v in row[:3])))

    def show(k):
        with Sdf.ChangeBlock():
            for i, mp, sp, S_, op in setters:
                if np.isnan(poses[k, i, 6]):
                    continue
                parent_w = world(poses[k, mp]) if mp is not None else sp
                op.Set(S_*world(poses[k, i])*parent_w.GetInverse())

    show(0)
    for _ in range(30):
        app.update()
    print(f"REPLAY {run_dir}: {times[-1]:.1f} s at real time", flush=True)
    while app.is_running():
        wall0 = time.perf_counter()
        k = -1
        while app.is_running():
            k_now = int(np.searchsorted(times, time.perf_counter()-wall0, side="right"))-1
            if k_now >= len(times)-1:
                show(len(times)-1)
                break
            if k_now != k:
                k = k_now
                show(k)
            app.update()
        if os.environ.get("REPLAY_ONCE"):
            break
        t_end = time.perf_counter()+3.
        while app.is_running() and time.perf_counter() < t_end:
            app.update()


def replay_video(env, run_dir, t0=0., t1=None, fps=15, tag="video"):
    """Render a finished run's replay.npz headless through the env cameras (no physics): <run>/<tag>_<cam>.gif/.mp4
    and a mid-frame PNG. Same poses as the computing run, so it never changes results."""
    from PIL import Image
    stage, xc = env.stage, env.xc
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
        mp = paths.index(parent.GetPath().pathString) if parent.GetPath().pathString in tracked else None
        sp = xc.GetLocalToWorldTransform(prim.GetParent())
        scale = Gf.Transform(xc.GetLocalTransformation(prim)[0]).GetScale()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        setters.append((i, mp, sp, Gf.Matrix4d().SetScale(scale), xf.AddTransformOp()))

    def world(row):
        x, y, z, w = row[3:]
        return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(float(w), float(x), float(y), float(z))),
                           Gf.Vec3d(*(float(v) for v in row[:3])))

    def show(k):
        with Sdf.ChangeBlock():
            for i, mp, sp, S_, op in setters:
                if np.isnan(poses[k, i, 6]):
                    continue
                parent_w = world(poses[k, mp]) if mp is not None else sp
                op.Set(S_*world(poses[k, i])*parent_w.GetInverse())
    t1 = times[-1] if t1 is None else t1
    ks = [int(np.searchsorted(times, t)) for t in np.arange(t0, t1, 1./fps)]
    ks = [min(k, len(times)-1) for k in ks]
    show(ks[0])
    for _ in range(20):
        env.app.update()
    for _ in range(3):
        env.render()
    for _, frames in env.writers.values():
        frames.clear()
    stamps = []
    for k in ks:
        show(k)
        env.render()
        stamps.append(float(times[k]))
    out = Path(run_dir)
    for cam, (_, frames) in env.writers.items():
        if not frames:
            continue
        try:
            import imageio.v2 as imageio
            imageio.mimwrite(out/f"{tag}_{cam}.mp4", frames, fps=fps, quality=7)
        except Exception as exc:
            print("mp4 skipped:", exc)
        small = [Image.fromarray(fr).resize((480, 270)).quantize(128) for fr in frames[::2]]
        small[0].save(out/f"{tag}_{cam}.gif", save_all=True, append_images=small[1:], duration=int(2000/fps),
                      loop=0, optimize=True)
        Image.fromarray(frames[len(frames)//2]).save(out/f"{tag}_{cam}_mid.png")
        print("VIDEO", out/f"{tag}_{cam}.gif", len(frames), "frames", f"{t0:.1f}-{t1:.1f} s", flush=True)


# ====================================================================================== phase-1 equivalent run

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--side", choices=["both", "left", "right"], default="both")
    ap.add_argument("--basic", action="store_true")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--transfer-speed", type=float, default=None)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--stop-after", default=None)
    ap.add_argument("--diag", action="store_true")
    ap.add_argument("--replay", default=None, metavar="RUN_DIR")
    ap.add_argument("--replay-video", default=None, metavar="RUN_DIR",
                    help="render RUN_DIR/replay.npz headless to GIF/MP4 (MOUNT_CAMS=cam1,cam2 to choose views)")
    ap.add_argument("--t0", type=float, default=0.)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--tag", default="video")
    ap.add_argument("--t-max", type=float, default=None)
    ap.add_argument("--car-dynamic", action="store_true", help="car body dynamic on the lift pads (AMR/lift held)")
    ap.add_argument("--car", action="store_true",
                    help="whole car: front wheels at dock A, fake index move A->B (+wheelbase), rear wheels at B")
    args = ap.parse_args()
    if args.replay:
        args.gui, args.no_video = True, True
    if args.replay_video:
        args.no_video = False
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "width": 1600 if args.gui else 960,
                         "height": 900 if args.gui else 540, "renderer": "RayTracedLighting", "anti_aliasing": 1,
                         "multi_gpu": False})
    env = CellEnv(app, args.seed, gui=args.gui, video=not args.no_video, diag=args.diag,
                  replay=bool(args.replay or args.replay_video), stop_after=args.stop_after,
                  car_dynamic=True if args.car_dynamic else None,
                  out=args.out or (str(Path(args.replay_video)/"_replay_tmp") if args.replay_video else None))
    if args.replay_video:
        replay_video(env, args.replay_video, args.t0, args.t1, tag=args.tag)
        app.close()
        return
    if args.replay:
        replay(env, args.replay)
        app.close()
        return
    prof = Profile(args.basic, args.speed, args.transfer_speed)
    front = env.stations(front=True)
    units = [MountSide(env, lab, info, front[info["side"]], prof) for lab, info in env.cell.mount_arms.items()
             if args.side == "both" or args.side == lab.lower()]
    t_max = args.t_max or (200. if args.car else 120.)
    for u in units:
        u.command()
    env.warmup()

    def sequence():
        """Phase-1 sequencer (phase 2 moves this into a ROS 2 node): front pair at A, index, rear pair at B."""
        for u in units:
            u.load(u.guarded(u.run_cycle()))
        while any(u.busy() for u in units):
            yield
        if not args.car:
            return
        if not all(r == "mounted" for u in units for r in u.results.values()):
            env.log_event(dict(t=round(env.t, 3), side="Cell", event="index_held",
                               reason="front wheel not mounted", wheels={k: v for u in units for k, v in u.results.items()}))
            return
        env.log_event(dict(t=round(env.t, 3), side="Cell", event="index_start", dx_m=process.WHEELBASE))
        yield from env.car_move(process.WHEELBASE)
        env.log_event(dict(t=round(env.t, 3), side="Cell", event="index_done", car_dx_m=round(env.car_dx, 4)))
        rear = env.stations(front=False)
        for u in units:
            u.station = rear[u.side]
            u.result, u.fail_reason = "incomplete", None
            u.load(u.guarded(u.run_cycle()))
        while any(u.busy() for u in units):
            yield

    seq = sequence()
    seq_done = [False]

    def seq_tick():
        if not seq_done[0]:
            try:
                next(seq)
            except StopIteration:
                seq_done[0] = True
    try:
        while env.t < t_max and not seq_done[0]:
            env.tick([seq_tick]+[u.tick for u in units])
    except Exception as exc:
        print("ERROR", exc, flush=True)
        import traceback
        traceback.print_exc()
    env.finish(dict(wheels={k: v for u in units for k, v in u.results.items()}))
    if args.gui:
        while app.is_running():
            app.update()
    env.cell.close()
    app.close()


if __name__ == "__main__":
    main()
