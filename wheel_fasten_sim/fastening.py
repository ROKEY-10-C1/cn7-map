"""FL 체결. Task가 USD 관측을 읽고 Controller가 매 물리 프레임에 명령을 갱신한다.

거리 m, 시간 s, 계산 각도 rad, USD drive 각도 deg. USD 쿼터니언 wxyz는
회전 행렬로 변환한다. 행렬의 열은 로컬 축의 World 방향이다.
"""
from dataclasses import dataclass
from math import tau
from pathlib import Path

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from fitment import NUT_SEAT, OFFSET, PITCH, WEB_THICKNESS
from robot import Arm

ASSETS = Path(__file__).resolve().parent / "assets"
TRAVEL_HALF_WIDTH = 1.06
TRAVEL_MARGIN = .05


@dataclass(frozen=True)
class Pose:
    xyz: np.ndarray
    rotation: np.ndarray

    def point(self, local):
        return self.xyz + self.rotation @ local


@dataclass(frozen=True)
class Observation:
    base: Pose
    flange: Pose
    tcp: Pose
    hub: Pose
    wheel: Pose
    tips: np.ndarray
    seats: np.ndarray
    nuts: tuple
    rotors: tuple


@dataclass(frozen=True)
class Settings:
    stand_off: float = .10
    control_period: float = 1 / 30
    entry_error: float = .0007
    axis_error_deg: float = .5
    pattern_error_deg: float = .5
    seat_error: float = .0007
    turn_error: float = .15
    wheel_gap: float = .0065
    wheel_tilt_deg: float = 2.7
    stud_protrusion: float = .003
    spindle_deg_s: float = 720.
    feed_lateral_error: float = .001
    motion_timeout: float = 20.
    phase_timeout: float = 6.


# 관측 및 USD 핸들. Pose 읽기만 바꾸면 비전/TF 관측을 연결할 수 있다.
class FastenTask:
    def __init__(self, stage, corner="FL", robot_path=None, load=None):
        if corner not in ("FL", "FR", "RL", "RR"):
            raise ValueError(f"Unknown corner {corner}")
        self.stage = stage
        self.corner = corner
        side = "Left" if corner.endswith("L") else "Right"
        self.side = 1 if side == "Left" else -1
        self.robot = robot_path or f"/World/{side}FastenRobot"
        self.hub = f"/World/Live/{corner}/Hub"
        self.wheel = f"/World/Live/{corner}/Wheel"
        self.flange = self.robot + "/link_6"
        self.tcp = self.flange + "/TCP"
        self.axes = tuple(f"{self.robot}/Head/Axis_{i}" for i in range(5))
        self.nuts = (tuple(path + "/Nut" for path in self.axes) if load is None else load[0])
        self.rotors = tuple(path + "/Rotor" for path in self.axes)
        for path in (self.robot, self.flange, self.tcp, self.hub, self.wheel, *self.nuts, *self.rotors):
            self.prim(path)
        if not self.prim(self.robot).HasAPI(UsdPhysics.ArticulationRootAPI):
            raise ValueError(f"Missing articulation {self.robot}")
        for path in (self.flange, self.hub, self.wheel, *self.nuts, *self.rotors):
            if not self.prim(path).HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(f"Missing rigid body {path}")
        self.arm_drives = [self.drive(f"{self.robot}/Joints/joint_{i}").GetTargetPositionAttr()
                           for i in range(1, 7)]
        self.bearings = [UsdPhysics.RevoluteJoint.Get(stage, p + "/Bearing") for p in self.axes]
        self.spindles = [self.drive(p + "/Bearing").CreateTargetVelocityAttr(0.) for p in self.axes]
        hold_paths = [p + "/SocketRetention" for p in self.axes] if load is None else load[1]
        self.holds = [UsdPhysics.FixedJoint.Get(stage, p) for p in hold_paths]
        self.offsets = np.array([list(b.GetLocalPos0Attr().Get()) for b in self.bearings])
        self.hold_offsets = np.array([list(h.GetLocalPos0Attr().Get()) for h in self.holds])
        for i, (bearing, hold) in enumerate(zip(self.bearings, self.holds)):
            self.check_joint(bearing, self.flange, self.rotors[i])
            self.check_joint(hold, self.rotors[i], self.nuts[i])
            if bearing.GetAxisAttr().Get() != "Z":
                raise ValueError(f"Unexpected spindle axis {bearing.GetPath()}")

        # 스터드 중심은 실린더, 끝단은 모따기 메시의 로컬 extent에서 읽는다.
        self.studs = []
        for i in range(5):
            stud = UsdGeom.Cylinder.Get(stage, f"{self.hub}/Stud_{i}")
            tip = UsdGeom.Mesh.Get(stage, f"{self.hub}/Stud_{i}Tip")
            if not stud or not tip or stud.GetAxisAttr().Get() != "Z":
                raise ValueError(f"Unexpected stud geometry {self.hub}/Stud_{i}")
            for shape in (stud, tip):
                collision = UsdPhysics.CollisionAPI(shape.GetPrim())
                if not collision or not collision.GetCollisionEnabledAttr().Get():
                    raise ValueError(f"Missing active stud collider {shape.GetPath()}")
            extent = tip.GetExtentAttr().Get()
            if not extent:
                extent = UsdGeom.Boundable.ComputeExtentFromPlugins(tip, Usd.TimeCode.Default())
            self.studs.append((str(stud.GetPath()), str(tip.GetPath()), float(extent[1][2])))

    def prim(self, path):
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError(f"Missing prim {path}")
        return prim

    def drive(self, path):
        drive = UsdPhysics.DriveAPI.Get(self.prim(path), "angular")
        if not drive or not self.prim(path).IsA(UsdPhysics.RevoluteJoint):
            raise ValueError(f"Missing revolute drive {path}")
        return drive

    @staticmethod
    def check_joint(joint, body0, body1):
        if (not joint or not joint.GetJointEnabledAttr().Get()
                or list(map(str, joint.GetBody0Rel().GetTargets())) != [body0]
                or list(map(str, joint.GetBody1Rel().GetTargets())) != [body1]):
            raise ValueError(f"Invalid joint endpoints {joint}")

    def get_world_pose(self, path, cache):
        matrix = cache.GetLocalToWorldTransform(self.prim(path))
        quat = matrix.ExtractRotationQuat()
        rotation = Rotation.from_quat([quat.GetReal(), *quat.GetImaginary()], scalar_first=True)
        return Pose(np.array(matrix.ExtractTranslation()), rotation.as_matrix())

    def observe(self):
        cache = UsdGeom.XformCache()
        hub = self.get_world_pose(self.hub, cache)
        outward = hub.rotation[:, 2]
        tips = []
        for stud, tip, top_z in self.studs:
            center = self.get_world_pose(stud, cache).xyz
            end = self.get_world_pose(tip, cache).point([0., 0., top_z])
            tips.append(center + outward * ((end-center) @ outward))
        tips = np.array(tips)
        # 시스템 모델의 안착면을 사용하고, 반경 방향으로는 각 스터드의 실측 위치를 유지한다.
        seats = tips + np.outer(NUT_SEAT - (tips-hub.xyz) @ outward, outward)
        return Observation(
            self.get_world_pose(self.robot + "/base_link", cache),
            self.get_world_pose(self.flange, cache), self.get_world_pose(self.tcp, cache),
            hub, self.get_world_pose(self.wheel, cache), tips, seats,
            tuple(self.get_world_pose(p, cache) for p in self.nuts),
            tuple(self.get_world_pose(p, cache) for p in self.rotors),
        )


# 제어. 업데이트 한 번은 명령만 기록한다. 물리 스텝은 호출자가 소유한다.
class FastenController:
    def __init__(self, task, config=Settings(), *, autostart=True, return_to_park=True):
        self.task = task
        self.config = config
        self.arm = Arm(ASSETS / "rs080n.urdf")
        self.park = np.deg2rad([attr.Get() for attr in task.arm_drives])
        self.q = self.park.copy()
        self.state = "SETTLE" if autostart else "IDLE"
        self.elapsed = 0.
        self.until_control = 0.
        self.message = ""
        self.metrics = {}
        self.motion = []
        self.mapping = None
        self.locked = []
        self.load = None
        self.operation = "fasten"
        self.return_to_park = return_to_park
        self.link_boxes = None

    def request_fasten(self, corner):
        """외부 공정이 로봇별 체결을 요청한다. 목표 변경은 완료 상태에서만 허용한다."""
        if corner[-1:] != self.task.corner[-1]:
            raise ValueError("Fastening request is on the opposite side")
        if self.state == "IDLE" and corner == self.task.corner:
            self.enter("SETTLE")
            return
        if self.state != "DONE" or self.operation != "reload" or self.load is None:
            raise RuntimeError("Fastening requires a completed reload")
        self.task = FastenTask(self.task.stage, corner, self.task.robot, self.load)
        self.load = None
        self.metrics = {}
        self.locked = []
        self.operation = "fasten"
        self.return_to_park = True
        self.enter("SETTLE")

    def request_reload(self):
        if self.state != "DONE" or self.operation != "fasten":
            raise RuntimeError("Reload requires completed fastening and verified retraction")
        self.operation = "reload"
        self.metrics = {}
        self.enter("RELOAD_PLAN")

    def result(self):
        return {"state": self.state, "operation": self.operation, "corner": self.task.corner,
                "message": self.message, **self.metrics}


    def enter(self, state):
        self.state = state
        self.elapsed = 0.
        self.until_control = 0.

    def command_arm(self, q):
        for attr, angle in zip(self.task.arm_drives, np.rad2deg(q)):
            attr.Set(float(angle))
        self.q = q.copy()

    def speed(self, values):
        for attr, value in zip(self.task.spindles, values):
            attr.Set(float(value))

    def solve(self, obs, position, rotation, fallback=False):
        # World 목표를 관측한 베이스 좌표로 변환한다. PARK/URDF 관절 부호는 USD와 동일하다.
        base = obs.base
        q = self.arm.solve(np.zeros(3), base.rotation.T @ (position-base.xyz), side=self.task.side,
                           previous=self.q, fallback=fallback, rotation=base.rotation.T @ rotation)
        if q is None:
            raise RuntimeError("Pose IK failed on the current branch")
        # 같은 플랜지 자세의 회전 후보 중 현재 손목에서 가장 가까운 값을 쓴다.
        wrist = [q[5]+turn*tau for turn in (-1, 0, 1)
                 if self.arm.lower[5] <= q[5]+turn*tau <= self.arm.upper[5]]
        q[5] = min(wrist, key=lambda angle: abs(angle-self.q[5]))
        return q

    def plan_motion(self, obs, goal, *, transit=False, start=None, allow_detour=True):
        """현재 자세에서 직접 연결을 우선하고, 필요한 구간에만 접은 자세를 넣는다."""
        task = self.task
        cache = UsdGeom.XformCache()
        if self.link_boxes is None:
            self.link_boxes = []
            for i in range(1, 7):
                mesh = UsdGeom.Mesh.Get(task.stage, f"{task.robot}/link_{i}/Housing")
                low, high = np.array(mesh.GetExtentAttr().Get())
                self.link_boxes.append(((low+high)/2, (high-low)/2))
        side = task.side
        label = "Left" if side > 0 else "Right"
        obstacles = []
        for prim in task.prim(f"/World/{label}NutMagazine").GetChildren():
            shape = UsdGeom.Boundable(prim)
            if shape:
                bbox = Gf.BBox3d(Gf.Range3d(*(Gf.Vec3d(p) for p in shape.GetExtentAttr().Get())),
                                 cache.GetLocalToWorldTransform(prim)).ComputeAlignedRange()
                obstacles.append((np.array(bbox.GetMin()), np.array(bbox.GetMax())))
        car = task.get_world_pose("/World/Live/CN7", cache).xyz
        obstacles.append((car+[-2.46, -.93, .55], car+[2.46, .93, 2.02]))
        # 로터와 장전 너트까지 포함하는 플랜지 기준 공구 외곽.
        radius = np.max(np.abs(task.offsets[:, :2]), axis=0)+.026
        tool_end = float(np.max((task.offsets+task.hold_offsets)[:, 2]))
        tool = (np.array([0., 0., tool_end/2]), np.r_[radius, tool_end/2])

        def clear(q):
            frames = self.arm.fk(np.zeros(3), q)
            bounds = (*zip(frames, self.link_boxes), (frames[-1], tool))
            for (position, rotation, _), (center, half) in bounds:
                world_rotation = obs.base.rotation @ rotation
                middle = obs.base.point(position+rotation @ center)
                extent = np.abs(world_rotation) @ half
                low, high = middle-extent, middle+extent
                if transit and side*middle[1]-extent[1] < TRAVEL_HALF_WIDTH+TRAVEL_MARGIN:
                    return False
                for lower, upper in obstacles:
                    if (np.all(high > lower) and np.all(low < upper)
                            and boxes_overlap(middle, world_rotation, half, lower, upper)):
                        return False
            return True

        def segment(start, end):
            steps = max(1, int(np.ceil(np.max(np.abs(end-start))/np.deg2rad(2.))))
            return all(clear(start+(end-start)*(i/steps)) for i in range(steps+1))

        start = self.q if start is None else start
        if segment(start, goal):
            return [goal]
        if not allow_detour or not clear(start) or not clear(goal):
            raise RuntimeError(f"Blocked path endpoint or insertion for {task.corner} {self.state}")
        # J1을 중간 방향으로 돌리면서 팔을 로봇 쪽으로 접는 후보를 검사한다.
        candidates = []
        for elbow in (-65., -95., -45.):
            for wrist in (-90., -115., -60.):
                fold = np.deg2rad([0., -35., elbow, 0., wrist, 0.])
                for yaw in ((start[0]+goal[0])/2, start[0], goal[0]):
                    fold[0] = yaw
                    if np.all(fold >= self.arm.lower) and np.all(fold <= self.arm.upper):
                        candidates.append(fold.copy())
        candidates.sort(key=lambda q: np.linalg.norm(q-start)+np.linalg.norm(goal-q))
        for waypoint in candidates:
            if segment(start, waypoint) and segment(waypoint, goal):
                return [waypoint, goal]
        raise RuntimeError(f"No clear joint path for {task.corner} {self.state}")

    def queue_motion(self, goals):
        self.motion = list(goals)
        self.motion_start = self.q.copy()
        self.motion_time = 0.

    def move(self, dt):
        if not self.motion:
            return True
        start, goal = self.motion_start, self.motion[0]
        duration = max(1.6, float(np.max(1.5*np.abs(goal-start)/(.55*self.arm.speed))))
        self.motion_time += dt
        t = min(1., self.motion_time/duration)
        self.command_arm(start + (goal-start)*t*t*(3.-2.*t))
        if self.motion_time >= duration + .35:
            self.motion.pop(0)
            self.motion_start = goal.copy()
            self.motion_time = 0.
        return not self.motion

    def target_rotation(self, obs):
        # 허브 +Z는 바깥쪽, 공구 +Z는 스터드를 향한다. 72도 대칭을 이용해 가까운 위상을 선택한다.
        outward = obs.hub.rotation[:, 2]
        nominal = obs.hub.rotation @ np.diag([1., -1., -1.])
        a = (self.task.offsets-self.task.offsets.mean(axis=0)) @ nominal.T
        choices = []
        for shift in range(5):
            ids = np.roll(np.arange(5), shift)
            b = obs.tips[ids]-obs.tips.mean(axis=0)
            angles = np.arctan2(np.cross(a, b) @ outward, np.sum(a*b, axis=1))
            phase = np.angle(np.exp(1j*angles).mean())
            choices.append((abs(phase), phase, ids))
        _, phase, self.mapping = min(choices, key=lambda choice: choice[0])
        self.metrics["stud_mapping"] = self.mapping.tolist()
        self.metrics["pattern_phase_deg"] = float(np.rad2deg(phase))
        self.hub_tool_rotation = obs.hub.rotation.T @ (
            Rotation.from_rotvec(outward*phase).as_matrix() @ nominal)

    def flange_goal(self, obs, positions):
        rotation = obs.hub.rotation @ self.hub_tool_rotation
        offset = (self.task.offsets+self.task.hold_offsets).mean(axis=0)
        return positions.mean(axis=0) - rotation @ offset, rotation

    def feed(self, obs, positions):
        # 실제 너트 위치의 오차를 작게 보정하여 드라이브 추종 오차를 상쇄한다.
        goal, rotation = self.flange_goal(obs, positions)
        error = positions.mean(axis=0) - np.mean([p.xyz for p in obs.nuts], axis=0)
        commanded = obs.base.point(self.arm.fk(np.zeros(3), self.q)[-1][0])
        correction = np.clip(commanded-obs.flange.xyz, -.0015, .0015)
        if np.linalg.norm(error) < .004:
            goal += correction
        # IK에 넘기는 목표는 현재 플랜지에서 프레임당 최대 4 mm 이동한다.
        delta = goal-obs.flange.xyz
        goal = obs.flange.xyz + delta*min(1., .004/max(np.linalg.norm(delta), 1e-9))
        self.command_arm(self.solve(obs, goal, rotation))

    def errors(self, obs, positions):
        outward = obs.hub.rotation[:, 2]
        actual = np.array([p.xyz for p in obs.nuts])
        axis_error = np.rad2deg(np.arccos(np.clip(obs.tcp.rotation[:, 2] @ -outward, -1, 1)))
        a, b = actual-actual.mean(axis=0), positions-positions.mean(axis=0)
        phase = np.max(np.abs(np.rad2deg(np.arctan2(np.cross(a,b) @ outward, np.sum(a*b, axis=1)))))
        position = float(np.max(np.linalg.norm(actual-positions, axis=1)))
        self.metrics.update(position_error_mm=1000*position, axis_error_deg=float(axis_error),
                            pattern_error_deg=float(phase))
        return position, axis_error, phase

    def wheel_ready(self, obs):
        outward = obs.hub.rotation[:, 2]
        gap = (obs.wheel.xyz-obs.hub.xyz) @ outward
        tilt = np.rad2deg(np.arccos(np.clip(obs.wheel.rotation[:, 2] @ outward, -1, 1)))
        # 휠이 기울어져도 각 스터드가 웹 바깥으로 실제 노출되어야 한다.
        normal = obs.wheel.rotation[:, 2]
        web = obs.wheel.point([0., 0., OFFSET+WEB_THICKNESS])
        protrusion = np.min((obs.tips-web) @ normal) / max(outward @ normal, .01)
        self.metrics.update(wheel_gap_mm=float(1000*gap), wheel_tilt_deg=float(tilt),
                            min_stud_protrusion_mm=float(1000*protrusion))
        if (abs(gap) > self.config.wheel_gap or tilt > self.config.wheel_tilt_deg
                or protrusion < self.config.stud_protrusion):
            raise RuntimeError(f"Wheel outside seating window: gap {gap*1000:.3f} mm, tilt {tilt:.3f} deg")

    @staticmethod
    def phases(obs):
        # 플랜지에 대한 로터의 상대 X축으로 실제 스핀을 측정한다. 팔 회전은 누적에서 제외한다.
        axes = np.array([obs.flange.rotation.T @ rotor.rotation[:, 0] for rotor in obs.rotors])
        return np.arctan2(axes[:, 1], axes[:, 0])

    @staticmethod
    def fixed_joint(stage, path, parent, child, frame, pose):
        joint = UsdPhysics.FixedJoint.Define(stage, path)
        joint.CreateJointEnabledAttr(False)
        joint.CreateBody0Rel().SetTargets([parent])
        joint.CreateBody1Rel().SetTargets([child])
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        local = frame.rotation.T @ (pose.xyz-frame.xyz)
        w, x, y, z = Rotation.from_matrix(frame.rotation.T @ pose.rotation).as_quat(scalar_first=True)
        joint.CreateLocalPos0Attr(Gf.Vec3f(*local))
        joint.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.))
        return joint.GetJointEnabledAttr()

    def lock(self, obs):
        # 새 관절을 모두 비활성 상태로 준비한다. 교대 중에는 물리 스텝을 진행하지 않는다.
        task = self.task
        paths = [f"/World/FastenedJoints/{task.corner}/Nut_{i}" for i in range(5)]
        clamp = f"/World/FastenedJoints/{task.corner}/WheelClamp"
        if any(task.stage.GetPrimAtPath(path) for path in (*paths, clamp)):
            raise RuntimeError("A hub attachment already exists")
        try:
            enabled = [self.fixed_joint(task.stage, path, task.hub, nut, obs.hub, pose)
                       for path, nut, pose in zip(paths, task.nuts, obs.nuts)]
            # 시스템 너트 체결 후의 휠 고정력을 허브-림 강체 관절로 모사한다.
            # 기존 상대 자세에서 붙잡으므로 림을 이동시키거나 물리 나사산을 계산하지 않는다.
            enabled.append(self.fixed_joint(task.stage, clamp, task.hub, task.wheel, obs.hub, obs.wheel))
            for hold in task.holds:
                if not hold.CreateJointEnabledAttr().Set(False):
                    raise RuntimeError("Socket retention could not be released")
            for attr in enabled:
                if not attr.Set(True):
                    raise RuntimeError("Hub joint could not be enabled")
            self.locked = [obs.hub.rotation.T @ (pose.xyz-obs.hub.xyz) for pose in obs.nuts]
        except Exception:
            for hold in task.holds:
                hold.CreateJointEnabledAttr().Set(True)
            for path in (*paths, clamp):
                task.stage.RemovePrim(path)
            raise

    def update(self, dt):
        """물리 스텝마다 호출. DONE 또는 FAILED는 호출자가 소비하는 종결 상태다."""
        if self.state in ("IDLE", "DONE", "FAILED"):
            return self.state
        self.elapsed += dt
        try:
            timeout = (self.config.motion_timeout if self.state in ("APPROACH", "PARK", "RUN_DOWN", "RELOAD_MOVE", "RELOAD_CLEAR")
                       else self.config.phase_timeout)
            if self.elapsed > timeout:
                raise RuntimeError(f"{self.state} timeout")
            if self.operation == "reload":
                self.update_reload(dt)
                return self.state
            if self.state in ("APPROACH", "PARK"):
                if self.move(dt):
                    self.enter("ALIGN" if self.state == "APPROACH" else "VERIFY")
                return self.state

            # 회전 위상만 매 물리 스텝 추적해 ±π 경계를 넘는 회전도 센다.
            turning = self.state in ("RUN_DOWN", "SEAT")
            if turning:
                obs = self.task.observe()
                phase = self.phases(obs)
                self.turns += np.angle(np.exp(1j*(phase-self.last_phase)))/tau
                self.last_phase = phase
            self.until_control -= dt
            if self.until_control > 0.:
                return self.state
            self.until_control = self.config.control_period
            if not turning:
                obs = self.task.observe()
            self.control(obs)
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
            self.speed(np.zeros(5))
            self.message = f"{self.state}: {error}"
            self.enter("FAILED")
        return self.state

    def control(self, obs):
        s = self.config
        outward = obs.hub.rotation[:, 2]
        if self.state == "SETTLE":
            if self.elapsed < .35:
                return
            self.wheel_ready(obs)
            for nut, rotor, offset in zip(obs.nuts, obs.rotors, self.task.hold_offsets):
                if np.linalg.norm(nut.xyz-rotor.point(offset)) > s.entry_error:
                    raise RuntimeError("Loaded nut pose does not match its retention")
            self.target_rotation(obs)
            goal, rot = self.flange_goal(obs, obs.tips + outward*s.stand_off)
            self.approach = self.solve(obs, goal, rot, fallback=True)
            self.queue_motion(self.plan_motion(obs, self.approach))
            self.enter("APPROACH")
            return

        tips, seats = obs.tips[self.mapping], obs.seats[self.mapping]
        if self.state in ("ALIGN", "ENGAGE"):
            self.wheel_ready(obs)
            positions = tips + outward*(s.stand_off if self.state == "ALIGN" else 0.)
            position, axis, phase = self.errors(obs, positions)
            if position <= s.entry_error and axis <= s.axis_error_deg and phase <= s.pattern_error_deg:
                if self.state == "ALIGN":
                    self.enter("ENGAGE")
                else:
                    self.required = ((tips-seats) @ outward)/PITCH
                    if np.any(self.required <= 0):
                        raise RuntimeError("Invalid pitch travel")
                    self.turns = np.zeros(5)
                    self.last_phase = self.phases(obs)
                    self.enter("RUN_DOWN")
            else:
                self.feed(obs, positions)
        elif self.state == "RUN_DOWN":
            self.wheel_ready(obs)
            error = np.array([pose.xyz for pose in obs.nuts])-tips
            radial = error-np.outer(error @ outward, outward)
            if (np.max(np.linalg.norm(radial, axis=1)) > s.feed_lateral_error
                    or obs.tcp.rotation[:, 2] @ -outward < np.cos(np.deg2rad(s.axis_error_deg))):
                raise RuntimeError("Lost spindle alignment during run-down")
            remaining = self.required-self.turns
            self.speed(np.clip(remaining*1800., 0., s.spindle_deg_s))
            advance = np.clip(np.min(self.turns/self.required), 0., 1.)
            self.feed(obs, tips+(seats-tips)*advance)
            if np.max(np.abs(remaining)) <= .025:
                self.speed(np.zeros(5))
                self.enter("SEAT")
        elif self.state == "SEAT":
            self.wheel_ready(obs)
            self.feed(obs, seats)
            position, axis, phase = self.errors(obs, seats)
            self.metrics.update(seat_error_mm=1000*position, turns=self.turns.tolist(),
                                required_turns=self.required.tolist())
            if (self.elapsed > .25 and position <= s.seat_error and axis <= s.axis_error_deg
                    and phase <= s.pattern_error_deg and np.max(np.abs(self.required-self.turns)) <= s.turn_error):
                self.lock(obs)
                self.enter("RETRACT")
        elif self.state == "RETRACT":
            clearance = float((obs.tcp.xyz-seats.mean(axis=0)) @ outward)
            minimum = .115 if self.return_to_park else .29
            if clearance >= minimum:
                self.metrics["retract_clearance_mm"] = clearance*1000
                if self.return_to_park:
                    self.queue_motion(self.plan_motion(obs, self.park))
                    self.enter("PARK")
                else:
                    self.enter("VERIFY")
            else:
                self.feed(obs, tips + outward*(s.stand_off if self.return_to_park else .27))
        elif self.state == "VERIFY":
            shifts = [np.linalg.norm(obs.hub.rotation.T @ (nut.xyz-obs.hub.xyz)-old)
                      for nut, old in zip(obs.nuts, self.locked)]
            self.metrics["nut_drift_mm"] = float(max(shifts)*1000)
            for i, hold in enumerate(self.task.holds):
                if hold.GetJointEnabledAttr().Get():
                    raise RuntimeError("Socket retention remains enabled")
                self.task.check_joint(
                    UsdPhysics.FixedJoint.Get(self.task.stage, f"/World/FastenedJoints/{self.task.corner}/Nut_{i}"),
                    self.task.hub, self.task.nuts[i])
            self.task.check_joint(
                UsdPhysics.FixedJoint.Get(self.task.stage, f"/World/FastenedJoints/{self.task.corner}/WheelClamp"),
                self.task.hub, self.task.wheel)
            if max(shifts) > s.seat_error:
                raise RuntimeError("Nut moved relative to the hub after retreat")
            if self.return_to_park and not self.parked(obs):
                return
            self.metrics["attached_nuts"] = 5
            self.enter("DONE")

    def parked(self, obs):
        flange, rotation, _ = self.arm.fk(np.zeros(3), self.park)[-1]
        pose_error = Rotation.from_matrix((obs.base.rotation @ rotation).T @ obs.flange.rotation)
        return np.linalg.norm(obs.flange.xyz-obs.base.point(flange)) <= .005 and pose_error.magnitude() <= .02

    # 재장전은 동일 로봇의 별도 요청이다. 박스는 기존 USD의 실제 포트 위치에서 관측한다.
    def update_reload(self, dt):
        if self.state in ("RELOAD_MOVE", "RELOAD_CLEAR") and not self.move(dt):
            return
        obs = self.task.observe()
        if self.state == "RELOAD_PLAN":
            side = "Left" if self.task.side > 0 else "Right"
            cache = UsdGeom.XformCache()
            outlets = [UsdGeom.Cylinder.Get(self.task.stage, f"/World/{side}NutMagazine/Outlet_{i}")
                       for i in range(5)]
            if not all(outlets):
                raise RuntimeError("Missing magazine outlets")
            tops = np.array([self.task.get_world_pose(str(outlet.GetPath()), cache).point(
                [0., 0., outlet.GetHeightAttr().Get()/2]) for outlet in outlets])
            self.reload_outlets = tops
            self.reload_tcp = tops.mean(axis=0) + np.array([0., 0., .015])
            self.reload_rotation = np.diag([1., -1., -1.])
            reach = np.linalg.norm(obs.tcp.xyz-obs.flange.xyz)
            pick = self.reload_tcp-self.reload_rotation[:, 2]*reach
            pre = pick + np.array([0., 0., .12])
            self.reload_pre = self.solve(obs, pre, self.reload_rotation, fallback=True)
            self.reload_at = self.arm.solve(np.zeros(3), obs.base.rotation.T @ (pick-obs.base.xyz),
                                            side=self.task.side, previous=self.reload_pre, fallback=False,
                                            rotation=obs.base.rotation.T @ self.reload_rotation)
            if self.reload_at is None:
                raise RuntimeError("Magazine insertion IK failed")
            approach = self.plan_motion(obs, self.reload_pre, transit=True)
            insertion = self.plan_motion(obs, self.reload_at, transit=True, start=self.reload_pre,
                                         allow_detour=False)
            self.queue_motion([*approach, *insertion])
            self.enter("RELOAD_MOVE")
        elif self.state == "RELOAD_MOVE":
            error = np.linalg.norm(obs.tcp.xyz-self.reload_tcp)
            angle = Rotation.from_matrix(self.reload_rotation.T @ obs.tcp.rotation).magnitude()
            if error > .002 or angle > np.deg2rad(1.):
                raise RuntimeError(f"Magazine alignment {error*1000:.2f} mm / {np.rad2deg(angle):.2f} deg")
            anchors = np.array([rotor.point(offset) for rotor, offset in zip(obs.rotors, self.task.hold_offsets)])
            radial = np.linalg.norm(anchors[:, :2]-self.reload_outlets[:, :2], axis=1)
            if np.max(radial) > .002 or np.min(anchors[:, 2]-self.reload_outlets[:, 2]) <= 0.:
                raise RuntimeError("Socket pattern does not match the magazine outlets")
            self.metrics["magazine_error_mm"] = float(error*1000)
            self.metrics["magazine_pattern_error_mm"] = float(np.max(radial)*1000)
            self.load = self.spawn_batch(obs)
            self.enter("RELOAD_CHECK")
        elif self.state == "RELOAD_CHECK":
            if self.elapsed < .25:
                return
            self.verify_batch(obs)
            self.queue_motion(self.plan_motion(obs, self.reload_pre, transit=True, allow_detour=False))
            self.enter("RELOAD_CLEAR")
        elif self.state == "RELOAD_CLEAR":
            self.verify_batch(obs)
            self.metrics["loaded_nuts"] = 5
            self.enter("DONE")

    def spawn_batch(self, obs):
        task = self.task
        stage = task.stage
        prefix = f"/World/ReloadedNuts/{task.robot.rsplit('/', 1)[-1]}"
        batch = 1
        while stage.GetPrimAtPath(f"{prefix}/Batch_{batch}"):
            batch += 1
        root = f"{prefix}/Batch_{batch}"
        nuts, holds = [], []
        try:
            for i, (rotor, offset) in enumerate(zip(obs.rotors, task.hold_offsets)):
                path = f"{root}/Nut_{i}"
                nut = UsdGeom.Xform.Define(stage, path).GetPrim()
                nut.GetReferences().AddReference(str(ASSETS / "FastenNut.usda"), "/Nut")
                set_initial_pose(nut, rotor.point(offset), rotor.rotation)
                for part, material in (("Hex", "Zinc"), ("Cone", "MachinedAlloy")):
                    stage.GetPrimAtPath(f"{path}/Visual/{part}").CreateRelationship("material:binding").SetTargets(
                        [f"/Looks/{material}"])
                joint = UsdPhysics.FixedJoint.Define(stage, f"{root}/Retention_{i}")
                joint.CreateBody0Rel().SetTargets([task.rotors[i]])
                joint.CreateBody1Rel().SetTargets([path])
                joint.CreateLocalPos0Attr(Gf.Vec3f(*offset))
                joint.CreateExcludeFromArticulationAttr(True)
                joint.CreateCollisionEnabledAttr(False)
                nuts.append(path)
                holds.append(str(joint.GetPath()))
        except Exception:
            stage.RemovePrim(root)
            raise
        return tuple(nuts), tuple(holds)

    def verify_batch(self, obs):
        cache = UsdGeom.XformCache()
        for i, (nut, hold) in enumerate(zip(*self.load)):
            self.task.check_joint(UsdPhysics.FixedJoint.Get(self.task.stage, hold), self.task.rotors[i], nut)
            pose = self.task.get_world_pose(nut, cache)
            if np.linalg.norm(pose.xyz-obs.rotors[i].point(self.task.hold_offsets[i])) > self.config.entry_error:
                raise RuntimeError(f"Reload retention position error for nut {i}")


def boxes_overlap(center, rotation, half, lower, upper):
    """회전 상자와 고정 상자의 분리축 검사. 긴 링크의 월드 AABB 과대 판정을 줄인다."""
    other_half = (upper-lower)/2
    delta = (upper+lower)/2-center
    axes = np.eye(3)
    candidates = (*rotation.T, *np.cross(rotation.T[:, None, :], axes[None, :, :]).reshape(-1, 3))
    for axis in candidates:
        if abs(axis @ delta) > np.abs(axis @ rotation) @ half + np.abs(axis) @ other_half:
            return False
    return True


def set_initial_pose(prim, xyz, rotation):
    """새로 생성한 물체에 초기 자세를 기록한다."""
    w, x, y, z = Rotation.from_matrix(rotation).as_quat(scalar_first=True)
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*xyz))
    prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(w, x, y, z))


# 독립 시험의 공정 인터록. 통합 셀에서는 로봇별 요청과 결과를 직접 사용한다.
class FastenCell:
    def __init__(self, stage, through):
        import omni.physx
        from physical_data import AMR_LIFT_MASS, AMR_MASS, BODY_MASS, WHEEL_MASS
        self.physics = omni.physx.get_physx_interface()
        self.stage = stage
        self.through = through
        self.controllers = [FastenController(FastenTask(stage, corner), return_to_park=through == "front")
                            for corner in ("FL", "FR")]
        self.state = "FRONT"
        self.message = ""
        self.metrics = {}
        self.elapsed = 0.
        self.deck = "/World/Live/DockedAMR"
        self.car = "/World/Live/CN7"
        self.lift = "/World/Live/DockLift"
        self.force_api = PhysxSchema.PhysxForceAPI(stage.GetPrimAtPath(self.deck))
        self.mass = AMR_MASS+AMR_LIFT_MASS+BODY_MASS+4*WHEEL_MASS
        world = stage.GetDefaultPrim()
        self.index = world.GetAttribute("cell:initialStopX").Get()
        self.final = world.GetAttribute("cell:finalStopX").Get()
        self.gap = world.GetAttribute("cell:frontWheelSeatGapM").Get()
        self.velocity = np.zeros(2)
        self.arrived_for = 0.
        self.clearances = None
        self.transport_holds = []

    def result(self):
        return {"state": self.state, "message": self.message, **self.metrics,
                "robots": [robot.result() for robot in self.controllers]}

    def enter(self, state):
        self.state = state
        self.elapsed = 0.

    def mount_rear(self):
        # 장착 모사: 기존 림/링 타이어 접촉 구조를 현재 후륜 허브에 한 번 생성한다.
        # 생성 이후엔 허브와 림 사이의 접촉 및 현가를 PhysX가 계산한다.
        cache = UsdGeom.XformCache()
        reader = self.controllers[0].task
        for side in ("L", "R"):
            source = f"/World/Live/F{side}"
            rear = f"/World/Live/R{side}"
            hub = reader.get_world_pose(rear+"/Hub", cache)
            for part in ("Wheel", "Tire", "TireMount"):
                old, new = Sdf.Path(source+"/"+part), Sdf.Path(rear+"/"+part)
                base = self.stage.DefinePrim(new)
                base.GetReferences().AddInternalReference(old)
                if part == "TireMount":
                    joint = UsdPhysics.Joint(base)
                    joint.GetBody0Rel().SetTargets([rear+"/Wheel"])
                    joint.GetBody1Rel().SetTargets([rear+"/Tire"])
                    continue
                set_initial_pose(base, hub.point([0., 0., self.gap]), hub.rotation)
                rigid = UsdPhysics.RigidBodyAPI(base)
                rigid.CreateVelocityAttr().Set(Gf.Vec3f(0.))
                rigid.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.))
                # 장착면/트레드의 물리 재질은 차축 외부에 있어 참조 대상에서 빠진다.
                for prim in Usd.PrimRange(self.stage.GetPrimAtPath(old), Usd.TraverseInstanceProxies()):
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    material = prim.GetRelationship("material:binding:physics")
                    if material:
                        dest = self.stage.GetPrimAtPath(prim.GetPath().ReplacePrefix(old, new))
                        dest.CreateRelationship(material.GetName()).SetTargets(material.GetTargets())

    def snapshot_payload(self):
        # USD 렌더 동기화 주기 대신 동일 PhysX 스텝의 강체 좌표를 비교한다.
        from conveyor import quaternion
        matrices = {}
        for path in {path for pair in self.payload for path in pair}:
            state = self.physics.get_rigidbody_transformation(path)
            if not state.get("ret_val"):
                raise RuntimeError(f"Missing PhysX payload body {path}")
            matrix = Gf.Matrix4d().SetRotate(quaternion(state))
            matrix.SetTranslateOnly(Gf.Vec3d(*state["position"]))
            matrices[path] = matrix
        return {child: matrices[child]*matrices[parent].GetInverse() for parent, child in self.payload}

    def begin_move(self):
        # 가결합 유지 조건을 통과한 후륜을 현재 자세에서 이송용 관절로 유지한다.
        # C에서 해제하고 다시 물리 안정화한 뒤 체결한다.
        reader = self.controllers[0].task
        cache = UsdGeom.XformCache()
        for corner in ("RL", "RR"):
            root = f"/World/Live/{corner}"
            parent, child = root+"/Hub", root+"/Wheel"
            frame, wheel = (reader.get_world_pose(path, cache) for path in (parent, child))
            hold = FastenController.fixed_joint(self.stage, f"/World/TransportHolds/{corner}",
                                              parent, child, frame, wheel)
            if not hold.Set(True):
                raise RuntimeError(f"Could not retain {corner} for transport")
            self.transport_holds.append(hold)
        self.payload = [(self.deck, self.lift), (self.lift, self.car)]
        for corner in ("FL", "FR", "RL", "RR"):
            root = f"/World/Live/{corner}"
            self.payload.append((root+"/Hub", root+"/Wheel"))
            self.payload.append((root+"/Wheel", root+"/Tire"))
        for robot in self.controllers:
            self.payload.extend((robot.task.hub, path) for path in robot.task.nuts)
        self.reference_payload = self.snapshot_payload()
        self.max_payload_mm = {"car_lift": 0., "lift_deck": 0., "wheel_hub": 0., "tire_rim": 0., "nut_hub": 0.}
        self.max_wheel_angle = 0.
        self.previous = self.deck_pose().xyz.copy()
        self.last_yaw = 0.
        self.enter("INDEX_RELOAD")

    def deck_pose(self):
        return self.controllers[0].task.get_world_pose(self.deck, UsdGeom.XformCache())

    def drive_deck(self, dt):
        from physical_data import AMR_ACCELERATION, AMR_SPEED_X, AMR_TRACTION_COEFFICIENT
        pose = self.deck_pose()
        if abs(pose.xyz[2]) > .025 or pose.rotation[2, 2] < np.cos(np.deg2rad(3.)):
            raise RuntimeError("AMR is outside the grounded chassis envelope")
        point = pose.xyz[:2]
        actual = (point-self.previous[:2])/dt
        self.previous = pose.xyz.copy()
        delta = np.array([self.final, 0.])-point
        distance = np.linalg.norm(delta)
        if distance < .006 and np.linalg.norm(actual) < .025:
            self.arrived_for += dt
        else:
            self.arrived_for = 0.
        desired = delta*min(3., AMR_SPEED_X/max(distance, 1e-9),
                            np.sqrt(2*AMR_ACCELERATION/max(distance, 1e-9)))
        change = desired-self.velocity
        self.velocity += change*min(1., AMR_ACCELERATION*dt/max(np.linalg.norm(change), 1e-9))
        force = (self.velocity-actual)*self.mass*6.
        limit = AMR_TRACTION_COEFFICIENT*self.mass*9.81
        force *= min(1., limit/max(np.linalg.norm(force), 1e-9))
        yaw = np.arctan2(pose.rotation[1, 0], pose.rotation[0, 0])
        torque = np.clip(-3600*yaw-900*(yaw-self.last_yaw)/dt, -.31*limit, .31*limit)
        self.last_yaw = yaw
        self.force_api.GetForceAttr().Set(Gf.Vec3f(*force, 0.))
        self.force_api.GetTorqueAttr().Set(Gf.Vec3f(0., 0., torque))
        self.metrics["amr_x_m"] = float(point[0])
        self.metrics["amr_y_m"] = float(point[1])
        self.metrics["amr_speed_m_s"] = float(np.linalg.norm(actual))
        return self.arrived_for >= .25

    def verify_payload(self):
        current = self.snapshot_payload()
        for path, before in self.reference_payload.items():
            after = current[path]
            error = 1000*(after.ExtractTranslation()-before.ExtractTranslation()).GetLength()
            category = ("car_lift" if path == self.car else "lift_deck" if path == self.lift else
                        "wheel_hub" if path.endswith("/Wheel") else
                        "tire_rim" if path.endswith("/Tire") else "nut_hub")
            self.max_payload_mm[category] = max(self.max_payload_mm[category], error)
            if category == "wheel_hub":
                angle = np.linalg.norm(Rotation.from_matrix(np.array(
                    after.ExtractRotationMatrix()*before.ExtractRotationMatrix().GetTranspose())).as_rotvec())
                self.max_wheel_angle = max(self.max_wheel_angle, np.rad2deg(angle))
            if error > {"car_lift": 20., "lift_deck": 15., "wheel_hub": 5., "tire_rim": 12., "nut_hub": .7}[category]:
                raise RuntimeError(f"Index payload shifted: {path} {error:.3f} mm")
        if self.max_wheel_angle > 3.:
            raise RuntimeError(f"Wheel rotated relative to its hub during index: {self.max_wheel_angle:.3f} deg")

    def reload_clearance(self):
        # AMR 선형 경로 전체를 덮는 보수적인 횡방향 분리 검사.
        cache = UsdGeom.XformCache()
        if self.clearances is None:
            self.clearances = []
            for robot in self.controllers:
                label = "Left" if robot.task.side > 0 else "Right"
                for root in (robot.task.robot, f"/World/{label}NutMagazine"):
                    for prim in Usd.PrimRange(self.stage.GetPrimAtPath(root)):
                        shape = UsdGeom.Boundable(prim)
                        if not shape or "/Nut/" in str(prim.GetPath()):
                            continue
                        ext = shape.GetExtentAttr().Get()
                        if not ext:
                            ext = UsdGeom.Boundable.ComputeExtentFromPlugins(shape, Usd.TimeCode.Default())
                        if ext:
                            low, high = (Gf.Vec3d(point) for point in ext)
                            self.clearances.append((robot.task.side, prim, (low+high)/2, (high-low)/2))
        # 차폭, 휠 외곽 및 여유를 포함한 통과 영역은 ±1.06 m다.
        clearance = float("inf")
        for side, prim, center, half in self.clearances:
            xform = cache.GetLocalToWorldTransform(prim)
            # 상자 중심의 Y 투영에서 세 축의 반폭 기여분을 빼면 가장 가까운 모서리다.
            half_y = sum(abs(xform[i][1])*half[i] for i in range(3))
            lateral = side*xform.Transform(center)[1]-half_y
            clearance = min(clearance, lateral-TRAVEL_HALF_WIDTH)
        for robot in self.controllers:
            if robot.load:
                for path in robot.load[0]:
                    p = robot.task.get_world_pose(path, cache).xyz
                    clearance = min(clearance, robot.task.side*p[1]-.026-TRAVEL_HALF_WIDTH)
        self.metrics["reload_lane_clearance_mm"] = min(
            self.metrics.get("reload_lane_clearance_mm", float("inf")), clearance*1000)
        if clearance < TRAVEL_MARGIN:
            raise RuntimeError(f"Reload arm entered AMR travel corridor: {clearance*1000:.1f} mm")

    def update(self, dt):
        if self.state in ("DONE", "FAILED"):
            return self.state
        self.elapsed += dt
        try:
            if self.elapsed > 65.:
                raise RuntimeError("Process stage timeout")
            if self.state == "FRONT" and self.through != "front" and "rear_mounted" not in self.metrics:
                self.mount_rear()
                self.metrics["rear_mounted"] = True
            if self.state == "INDEX_RELOAD":
                self.reload_clearance()
                self.verify_payload()
                self.metrics.update(index_relative_drift_mm=self.max_payload_mm,
                                    index_wheel_rotation_deg=self.max_wheel_angle)
                arrived = self.drive_deck(dt)
                if self.metrics["amr_speed_m_s"] > .025 and any(r.state != "DONE" for r in self.controllers):
                    self.metrics["index_reload_overlap_s"] = self.metrics.get("index_reload_overlap_s", 0.)+dt
                if arrived:
                    self.metrics.setdefault("index_arrival_s", self.elapsed)
            else:
                arrived = False
                if self.state == "REAR":
                    self.drive_deck(dt)
                    if (np.hypot(self.metrics["amr_x_m"]-self.final, self.metrics["amr_y_m"]) > .006
                            or self.metrics["amr_speed_m_s"] > .025):
                        raise RuntimeError("AMR left the stationary C envelope during rear fastening")
            for robot in self.controllers:
                robot.update(dt)
                if robot.state == "FAILED":
                    raise RuntimeError(f"{robot.task.corner} {robot.message}")
            if not all(robot.state == "DONE" for robot in self.controllers):
                return self.state
            if self.state == "FRONT":
                self.metrics["front"] = [robot.result() for robot in self.controllers]
                if self.through == "front":
                    self.metrics["front_safe"] = True
                    self.enter("DONE")
                else:
                    if np.linalg.norm(self.deck_pose().xyz[:2]-[self.index, 0.]) > .006:
                        raise RuntimeError("AMR is outside stop B tolerance")
                    for robot in self.controllers:
                        rear = "R"+robot.task.corner[-1]
                        # 후륜 검증엔 아직 소켓이 비어 있으므로 후륜과 허브 자세만 읽는다.
                        reader = robot.task
                        cache = UsdGeom.XformCache()
                        root = f"/World/Live/{rear}"
                        hub, wheel = (reader.get_world_pose(root+"/"+part, cache) for part in ("Hub", "Wheel"))
                        if (np.linalg.norm(wheel.xyz-hub.xyz) > .0065
                                or wheel.rotation[:, 2] @ hub.rotation[:, 2] < np.cos(np.deg2rad(2.7))):
                            raise RuntimeError(f"Rear wheel is not retained before indexing: {rear}")
                    self.reload_clearance()
                    self.metrics["front_safe"] = True
                    for robot in self.controllers:
                        robot.request_reload()
                    if self.through == "reload":
                        self.enter("RELOAD_ONLY")
                    else:
                        self.begin_move()
            elif self.state == "RELOAD_ONLY" or (self.state == "INDEX_RELOAD" and arrived):
                self.metrics["reload"] = [robot.result() for robot in self.controllers]
                if self.through in ("reload", "move"):
                    self.stop_drive()
                    self.enter("DONE")
                else:
                    self.metrics["arrived_c"] = True
                    for hold in self.transport_holds:
                        if not hold.Set(False):
                            raise RuntimeError("Could not release rear transport hold")
                    for robot in self.controllers:
                        robot.request_fasten("R"+robot.task.corner[-1])
                    self.enter("REAR")
            elif self.state == "REAR":
                self.metrics["rear"] = [robot.result() for robot in self.controllers]
                self.metrics["rear_safe"] = True
                self.stop_drive()
                self.enter("DONE")
        except (RuntimeError, ValueError) as error:
            self.stop_drive()
            for robot in self.controllers:
                robot.speed(np.zeros(5))
            self.message = f"{self.state}: {error}"
            self.enter("FAILED")
        return self.state

    def stop_drive(self):
        self.force_api.GetForceAttr().Set(Gf.Vec3f(0.))
        self.force_api.GetTorqueAttr().Set(Gf.Vec3f(0.))
