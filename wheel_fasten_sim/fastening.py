"""FL 체결. Task가 USD 관측을 읽고 Controller가 매 물리 프레임에 명령을 갱신한다.

거리 m, 시간 s, 계산 각도 rad, USD drive 각도 deg. USD 쿼터니언 wxyz는
회전 행렬로 변환한다. 행렬의 열은 로컬 축의 World 방향이다.
"""
from dataclasses import dataclass
from math import tau
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from fitment import NUT_SEAT, OFFSET, PITCH, WEB_THICKNESS
from robot import Arm

ASSETS = Path(__file__).resolve().parent / "assets"


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
    def __init__(self, stage, corner="FL", robot_path=None):
        if corner not in ("FL", "FR", "RL", "RR"):
            raise ValueError(f"Unknown corner {corner}")
        self.stage = stage
        self.corner = corner
        side = "Left" if corner.endswith("L") else "Right"
        self.robot = robot_path or f"/World/{side}FastenRobot"
        self.hub = f"/World/Live/{corner}/Hub"
        self.wheel = f"/World/Live/{corner}/Wheel"
        self.flange = self.robot + "/link_6"
        self.tcp = self.flange + "/TCP"
        self.axes = tuple(f"{self.robot}/Head/Axis_{i}" for i in range(5))
        self.nuts = tuple(path + "/Nut" for path in self.axes)
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
        self.holds = [UsdPhysics.FixedJoint.Get(stage, p + "/SocketRetention") for p in self.axes]
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
    def __init__(self, task, config=Settings()):
        self.task = task
        self.config = config
        self.arm = Arm(ASSETS / "rs080n.urdf")
        self.park = np.deg2rad([attr.Get() for attr in task.arm_drives])
        self.q = self.park.copy()
        self.state = "SETTLE"
        self.elapsed = 0.
        self.until_control = 0.
        self.message = ""
        self.metrics = {}
        self.motion = []
        self.mapping = None
        self.locked = []

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
        q = self.arm.solve(np.zeros(3), base.rotation.T @ (position-base.xyz), side=1,
                           previous=self.q, fallback=fallback, rotation=base.rotation.T @ rotation)
        if q is None:
            raise RuntimeError("Pose IK failed on the current branch")
        return q

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

    def lock(self, obs):
        # 새 관절을 모두 비활성 상태로 준비한다. 교대 중에는 물리 스텝을 진행하지 않는다.
        task = self.task
        paths = [f"/World/FastenedJoints/{task.corner}/Nut_{i}" for i in range(5)]
        if any(task.stage.GetPrimAtPath(path) for path in paths):
            raise RuntimeError("A hub attachment already exists")
        enabled = []
        try:
            for path, nut_path, pose in zip(paths, task.nuts, obs.nuts):
                joint = UsdPhysics.FixedJoint.Define(task.stage, path)
                enabled.append(joint.CreateJointEnabledAttr(False))
                joint.CreateBody0Rel().SetTargets([task.hub])
                joint.CreateBody1Rel().SetTargets([nut_path])
                joint.CreateExcludeFromArticulationAttr(True)
                joint.CreateCollisionEnabledAttr(False)
                local = obs.hub.rotation.T @ (pose.xyz-obs.hub.xyz)
                w, x, y, z = Rotation.from_matrix(obs.hub.rotation.T @ pose.rotation).as_quat(scalar_first=True)
                joint.CreateLocalPos0Attr(Gf.Vec3f(*local))
                joint.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
                joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
                joint.CreateLocalRot1Attr(Gf.Quatf(1.))
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
            for path in paths:
                task.stage.RemovePrim(path)
            raise

    def update(self, dt):
        """물리 스텝마다 호출. DONE 또는 FAILED는 호출자가 소비하는 종결 상태다."""
        if self.state in ("DONE", "FAILED"):
            return self.state
        self.elapsed += dt
        try:
            timeout = (self.config.motion_timeout if self.state in ("APPROACH", "PARK", "RUN_DOWN")
                       else self.config.phase_timeout)
            if self.elapsed > timeout:
                raise RuntimeError(f"{self.state} timeout")
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
            upright = np.zeros(6)
            upright[0] = self.park[0]
            facing = upright.copy()
            facing[0] = self.approach[0]
            self.queue_motion([upright, facing, self.approach])
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
            # 너트에서 축 방향으로 충분히 떨어진 후에만 관절 경유 자세로 PARK 복귀한다.
            clearance = float((obs.tcp.xyz-seats.mean(axis=0)) @ outward)
            if clearance >= .09:
                facing = np.zeros(6)
                facing[0] = self.approach[0]
                upright = np.zeros(6)
                upright[0] = self.park[0]
                self.queue_motion([facing, upright, self.park])
                self.metrics["retract_clearance_mm"] = clearance*1000
                self.enter("PARK")
            else:
                self.feed(obs, tips + outward*s.stand_off)
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
            if max(shifts) > s.seat_error:
                raise RuntimeError("Nut moved relative to the hub after retreat")
            park_flange, park_rot, _ = self.arm.fk(np.zeros(3), self.park)[-1]
            if (np.linalg.norm(obs.flange.xyz-obs.base.point(park_flange)) > .005
                    or Rotation.from_matrix((obs.base.rotation @ park_rot).T @ obs.flange.rotation).magnitude() > .02):
                return
            self.metrics["attached_nuts"] = 5
            self.enter("DONE")
