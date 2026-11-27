"""Isaac Sim 좌전륜 체결 시험. 기본 실행은 TaskController를 시작한다."""

import argparse
import sys
from math import cos, degrees, sin, tau
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

# 체결 시험 진입점. 이 파일을 Isaac Sim의 python.sh로 실행한다.
# run()은 USD 장면과 재생 상태를 관리하고 TaskController는 체결 공정을 제어한다.
# Python에서 PhysX 드라이브에 목표값을 직접 쓴다.
from isaacsim import SimulationApp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "wheel_cell"))

from fitment import BOLT_CENTERS, NUT_LENGTH, NUT_SEAT, OFFSET, PITCH
from robot import Arm

STAGE_PATH = HERE / "scene" / "fasten_pose_check.usda"
ROBOT_PATH = "/World/FastenRobot"
HUB_PATH = "/World/Car/Visuals/FL"
ROBOT_URDF = HERE / "assets" / "rs080n.urdf"
TOOL_LENGTH = .464
SPINDLE_RADIUS = .0255
SPINDLE_MASS = 5.8
SPINDLE_STROKE = .050
SPINDLES = tuple(f"/World/Head/Axis_{i}" for i in range(5))


# 실행 환경. 장면을 열고 재생·정지·재시작에 맞춰 체결 작업의 수명을 관리한다.
class CycleInterrupted(Exception):
    """타임라인 정지나 창 닫기에 맞춰 실행 중인 체결을 중단한다."""


def run(app, headless, captures):
    """장면과 재생 상태를 관리한다. 로봇의 단계별 작업은 TaskController가 맡는다."""
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Sdf, Usd

    context = omni.usd.get_context()
    if not context.open_stage(str(STAGE_PATH)):
        raise RuntimeError("Could not open the fastening pose check stage")
    while context.get_stage_loading_status()[2]:
        app.update()

    stage = context.get_stage()
    dt = 1 / 120
    sim = SimulationContext(physics_dt=dt, rendering_dt=1 / 60, stage_units_in_meters=1,
                            physics_prim_path="/World/Physics")
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_end_time(60.)
    timeline.set_looping(False)
    timeline.commit()
    restart = False
    stopped = False

    def on_stop(_event):
        nonlocal stopped
        stopped = True

    stop_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
        int(omni.timeline.TimelineEventType.STOP), on_stop
    )
    if not headless:
        import omni.ui as ui

        # Restart plays automatically after the stopped scene has been restored.
        def restart_cycle():
            nonlocal restart
            restart = True
            timeline.stop()

        set_camera_view(eye=[4.4, 5.4, 3.0], target=[1.3, 1.5, 1.1])
        window = ui.Window("CN7 Fastening", width=350, height=85)
        with window.frame, ui.VStack(spacing=4):
            status = ui.Label("Ready")
            with ui.HStack(spacing=4):
                ui.Button("Play", clicked_fn=timeline.play)
                ui.Button("Pause", clicked_fn=timeline.pause)
                ui.Button("Restart", clicked_fn=restart_cycle)

    def is_open():
        return app.is_running() and context.get_stage() == stage

    def step(frame):
        if not headless and (frame % 2 == 0 or not timeline.is_playing()):
            sim.render()
        while not timeline.is_playing() and not stopped and not restart and is_open():
            if not headless:
                status.text = "Paused"
            sim.render()
        if stopped or restart or not is_open():
            raise CycleInterrupted
        if not headless:
            status.text = "Running"
        sim.step(render=False)

    session = stage.GetSessionLayer()
    # 관절 명령과 안착 조인트는 임시 레이어에 쓴다. 재시작 때 비우면 너트가 소켓에 고정된 초기 상태로 돌아간다.
    cycle_layer = Sdf.Layer.CreateAnonymous("fasten_cycle.usda")
    session.subLayerPaths.insert(0, cycle_layer.identifier)
    last_result = 0
    timeline.play()
    timeline.commit()
    try:
        while is_open():
            try:
                sim.initialize_physics()
                with Usd.EditContext(stage, cycle_layer):
                    last_result = TaskController(app, stage, dt, step, captures).run()
            except CycleInterrupted:
                if headless:
                    return 1
            except (RuntimeError, ValueError) as exc:
                if headless:
                    raise
                last_result = 1
                print(f"FAIL {exc}", flush=True)
            if headless or not is_open():
                return last_result
            if not stopped and not restart:
                timeline.pause()
                timeline.commit()
                status.text = "Complete · Restart to repeat" if last_result == 0 else "Failed · Restart to retry"
                while not stopped and not restart and is_open():
                    if timeline.is_playing():
                        timeline.pause()
                    sim.render()
            if not is_open():
                break
            if not timeline.is_stopped():
                timeline.stop()
                timeline.commit()
            sim.render()
            cycle_layer.Clear()
            stopped = False
            status.text = "Ready"
            if restart:
                restart = False
                timeline.play()
                timeline.commit()
            while not timeline.is_playing() and is_open():
                if restart:
                    restart = False
                    timeline.play()
                    timeline.commit()
                sim.render()
            stopped = False
    finally:
        stop_subscription.unsubscribe()
        if context.get_stage() == stage:
            timeline.stop()
            timeline.commit()
        session.subLayerPaths.remove(cycle_layer.identifier)
    return last_result


# 체결 공정. run()에 작업 순서가 있고, 각 메서드에 해당 단계의 제어와 판정이 있다.
class TaskController:
    """체결 한 사이클의 상태를 관리한다. 재시작할 때 새 인스턴스를 만든다."""

    def __init__(self, app, stage, dt, step, captures):
        self.app = app
        self.stage = stage
        self.dt = dt
        self.step = step
        self.captures = captures

    def run(self):
        # 체결 순서와 단계 사이에서 넘기는 측정값은 여기에 모아 둔다.
        self.prepare()
        self.capture("loaded_sockets", [2.12, 2.45, 3.70], [1.8, 2.80, 3.10])
        approach_ok = self.approach()
        self.align()
        start, turns = self.spin()
        seated = self.seat(start, turns)
        self.release()
        passed = self.retract(seated, approach_ok)
        self.capture("seated_nuts", [1.66, 1.10, 1.07], [1.36, .85, .94])
        return 0 if passed else 1

    def prepare(self):
        """휠·베이스의 월드 자세를 읽고 준비 자세의 IK를 계산한다."""
        from isaacsim.core.prims import SingleXFormPrim

        wheel = SingleXFormPrim("/World/Wheel", reset_xform_properties=False)
        base = SingleXFormPrim(f"{ROBOT_PATH}/base_link", reset_xform_properties=False)
        self.tcp = SingleXFormPrim(f"{ROBOT_PATH}/link_6/TCP", reset_xform_properties=False)
        self.wheel_xyz, wheel_wxyz = wheel.get_world_pose()
        self.base_xyz, base_wxyz = base.get_world_pose()
        print(f"Wheel GT: xyz_m={self.wheel_xyz} quaternion_wxyz={wheel_wxyz}", flush=True)
        print(f"Robot base: xyz_m={self.base_xyz} quaternion_wxyz={base_wxyz}", flush=True)

        # 휠과 허브가 기울어질 수 있으므로 체결기 방향은 허브(스터드) 축으로 잡는다.
        hub = SingleXFormPrim(HUB_PATH, reset_xform_properties=False)
        self.hub_xyz, hub_wxyz = hub.get_world_pose()
        self.outward = Rotation.from_quat(hub_wxyz, scalar_first=True).apply([0, 0, 1])
        # 좌전륜 바깥 방향은 월드 +Y, 베이스는 월드에 대해 회전하지 않은 배치다.
        if not np.allclose(self.outward, [0, 1, 0], atol=1e-5) or not np.allclose(
            np.abs(base_wxyz), [1, 0, 0, 0], atol=1e-5
        ):
            raise ValueError("The approach solver expects the left-side fixture orientation")

        # 너트 외측 기준면에서 80 mm 바깥이 TCP 목표다. 공구 길이 464 mm를 더해 플랜지 목표를 구한다.
        self.target = self.hub_xyz + self.outward * (NUT_SEAT + NUT_LENGTH + .08)
        self.flange_target = self.target + self.outward * TOOL_LENGTH
        self.arm = Arm(ROBOT_URDF)
        self.angles = self.arm.solve(self.base_xyz, self.flange_target, side=1)
        print(f"Target TCP: xyz_m={self.target} joint_deg={np.rad2deg(self.angles)}", flush=True)

        self.arm_drives = [
            self.stage.GetPrimAtPath(f"{ROBOT_PATH}/Joints/joint_{i}").GetAttribute(
                "drive:angular:physics:targetPosition"
            )
            for i in range(1, 7)
        ]
        self.nuts = [SingleXFormPrim(root + "/Nut", reset_xform_properties=False) for root in SPINDLES]

    def command_arm(self, radians):
        # IK는 라디안으로 계산한다. PhysX 각도 드라이브에는 도 단위로 직접 전달한다.
        for drive, degrees in zip(self.arm_drives, np.rad2deg(radians)):
            drive.Set(float(degrees))

    def approach(self):
        """초기 자세에서 준비 자세로 이동하고 TCP 도착 오차를 측정한다."""
        # 팔을 세운 상태에서 먼저 베이스를 돌린다. 그다음 최종 준비 자세로 팔을 접는다.
        # 중간 자세의 접촉 여부는 별도 검증이 필요하다.
        turned = np.zeros_like(self.angles)
        turned[0] = self.angles[0]
        for start, goal in ((np.zeros_like(turned), turned), (turned, self.angles)):
            for frame in range(720):
                fraction = min((frame + 1) * self.dt / 3, 1)
                blend = fraction**2 * (3 - 2 * fraction)
                self.command_arm(start + (goal - start) * blend)
                self.step(frame)

        actual, quat = self.tcp.get_world_pose()
        actual_axis = Rotation.from_quat(quat, scalar_first=True).apply([0, 0, 1])
        position_error = np.linalg.norm(actual - self.target)
        axis_error = np.degrees(np.arccos(np.clip(actual_axis @ -self.outward, -1, 1)))
        passed = position_error <= .015 and axis_error <= 2
        print(f"{'PASS' if passed else 'FAIL'} TCP xyz_m={actual} "
              f"position_error_mm={position_error * 1000:.2f} axis_error_deg={axis_error:.2f}", flush=True)
        return passed

    def nut_positions(self):
        return np.array([nut.get_world_pose()[0] for nut in self.nuts])

    def correct_arm(self, delta):
        # 현재 명령값의 플랜지 위치에 실측 오차로 구한 보정량을 더하고 새 IK를 푼다.
        commanded = self.arm.fk(self.base_xyz, self.angles)[-1][0]
        self.angles = self.arm.solve(self.base_xyz, commanded + delta, 1, self.angles)
        self.command_arm(self.angles)

    def move_nuts(self, goal, frames):
        # 12스텝마다 너트 평균 위치를 읽어 플랜지 목표를 최대 6 mm 보정한다.
        for frame in range(frames):
            if frame % 12 == 0:
                error = goal.mean(axis=0) - self.nut_positions().mean(axis=0)
                delta = error * min(1., .006 / max(np.linalg.norm(error), 1e-9))
                self.correct_arm(delta)
            self.step(frame)

    def align(self):
        """스터드 끝점에 접근하고 다섯 너트의 진입 위치를 확인한다."""
        from pxr import UsdGeom

        bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        tips = []
        for i in range(len(SPINDLES)):
            # 스터드 원점은 허브와 같으므로 메시 경계 상자의 +Y 끝을 읽는다.
            stud = self.stage.GetPrimAtPath(f"{HUB_PATH}/Stud_{i}")
            box = bounds.ComputeWorldBound(stud).ComputeAlignedRange()
            tip = np.array(box.GetMidpoint())
            tip[1] = box.GetMax()[1]
            tips.append(tip)
        self.tips = np.array(tips)
        self.seats = self.tips.copy()
        self.seats[:, 1] = (self.hub_xyz + self.outward * NUT_SEAT)[1]
        self.turns_required = np.mean(self.tips[:, 1] - self.seats[:, 1]) / PITCH
        self.move_nuts(self.tips, 360)
        alignment = np.linalg.norm(self.nut_positions() - self.tips, axis=1)
        print("Engagement errors mm:", alignment * 1000, flush=True)
        if np.max(alignment) > .0007:
            raise RuntimeError("Stud entry alignment exceeds the 0.7 mm radial clearance")

    def phases(self):
        # 너트 로컬 X축을 월드 XZ 평면에 투영해 회전 위상을 읽는다.
        rotations = [Rotation.from_quat(nut.get_world_pose()[1], scalar_first=True) for nut in self.nuts]
        axes = np.array([rotation.apply([1, 0, 0]) for rotation in rotations])
        return np.arctan2(axes[:, 2], axes[:, 0])

    def spin(self):
        """스핀들 5개를 동시에 돌리고 실측 회전량에 따라 팔을 전진시킨다."""
        from pxr import UsdPhysics

        speeds = [
            UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath(root + "/Bearing"), "angular")
            .CreateTargetVelocityAttr(0.) for root in SPINDLES
        ]
        last = self.phases()
        turns = np.zeros(len(SPINDLES))
        start = self.nut_positions()
        # 각 스핀들에 목표 속도 720도/초를 준다.
        for speed in speeds:
            speed.Set(720.)
        for frame in range(1800):
            self.step(frame)
            phase = self.phases()
            turns += np.angle(np.exp(1j * (phase - last))) / tau
            last = phase
            if frame % 12 == 0:
                # 가장 느린 너트의 실측 회전수 × 피치가 팔의 축 방향 이송량이다.
                advance = min(self.turns_required, float(turns.min())) * PITCH
                goal = self.tips - self.outward * advance
                self.correct_arm(goal.mean(axis=0) - self.nut_positions().mean(axis=0))
            if turns.min() >= self.turns_required:
                break
        for speed in speeds:
            speed.Set(0.)
        return start, turns

    def seat(self, start, turns):
        """마지막 위치를 보정하고 너트별 회전수와 안착 오차를 검증한다."""
        self.move_nuts(self.seats, 120)
        end = self.nut_positions()
        errors = np.linalg.norm(end - self.seats, axis=1)
        for i, error in enumerate(errors):
            travel_mm = (start[i] - end[i]) @ self.outward * 1000
            print(f"Nut {i+1}: turns={turns[i]:.3f} travel_mm={travel_mm:.2f} "
                  f"seat_error_mm={error*1000:.3f}", flush=True)
        if not (np.all(np.abs(turns - self.turns_required) < .15) and np.max(errors) < .0007):
            raise RuntimeError("Five-nut joint-based seating check failed")
        return end

    def release(self):
        """안착 위치에서 너트를 월드 조인트로 고정하고 소켓 고정을 해제한다."""
        from pxr import Gf, UsdPhysics

        for root, nut in zip(SPINDLES, self.nuts):
            xyz, quat = nut.get_world_pose()
            lock = UsdPhysics.FixedJoint.Define(self.stage, root + "/SeatedJoint")
            lock.CreateBody1Rel().SetTargets([root + "/Nut"])
            lock.CreateLocalPos0Attr(Gf.Vec3f(*xyz.tolist()))
            lock.CreateLocalRot0Attr(Gf.Quatf(float(quat[0]), Gf.Vec3f(*quat[1:].tolist())))
            lock.CreateExcludeFromArticulationAttr(True)
            UsdPhysics.Joint.Get(self.stage, root + "/SocketRetention").CreateJointEnabledAttr(False)

    def retract(self, seated, approach_ok):
        """준비 자세로 후퇴한 뒤 너트 잔류 위치와 TCP 이격 거리를 확인한다."""
        retract = self.arm.solve(self.base_xyz, self.flange_target, 1, self.angles)
        for frame in range(240):
            fraction = min(1., (frame + 1) / 180)
            self.command_arm(self.angles + (retract - self.angles) * fraction)
            self.step(frame)
        movement = np.linalg.norm(self.nut_positions() - seated, axis=1)
        retreat = (self.tcp.get_world_pose()[0] - self.seats.mean(axis=0)) @ self.outward
        passed = approach_ok and movement.max() < .0007 and retreat > .075
        print(f"{'PASS' if passed else 'FAIL'} retained after retreat: "
              f"max_drift_mm={movement.max()*1000:.3f} tcp_clearance_mm={retreat*1000:.1f}", flush=True)
        return passed

    def capture(self, name, eye, look_at):
        if self.captures is None:
            return
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        self.captures.mkdir(parents=True, exist_ok=True)
        set_camera_view(eye=eye, target=look_at)
        viewport = get_active_viewport()
        viewport.set_texture_resolution((1280, 720))
        for _ in range(40):
            self.app.update()
        image = self.captures / (name+".png")
        image.unlink(missing_ok=True)
        capture_viewport_to_file(viewport, str(image))
        for _ in range(300):
            self.app.update()
            if image.exists():
                return
        raise RuntimeError(f"Capture did not finish: {image}")



def build_robot(mesh_dir):
    """Kawasaki URDF와 공식 STL을 고정 베이스 PhysX 관절 모델로 변환한다."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
    from sim_util import body, constrain, drive

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage, "/RS080N").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    arm = Arm(ROBOT_URDF)
    frames = arm.fk((0, 0, 0), np.zeros(6))
    data = ET.parse(ROBOT_URDF).getroot()
    links = ["base_link"] + [joint.find("child").get("link") for joint in arm.joints]
    bodies = []
    stl = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])

    def quaternion(matrix):
        x, y, z, w = Rotation.from_matrix(matrix).as_quat()
        return Gf.Quatd(w, Gf.Vec3d(x, y, z))

    for index, name in enumerate(links):
        link = data.find(f"./link[@name='{name}']")
        mass = float(link.find("inertial/mass").get("value"))
        position, rot, _ = frames[index - 1] if index else (np.zeros(3), np.eye(3), None)
        path = f"{root.GetPath()}/{'link_'+str(index) if index else name}"
        rigid = body(stage, path, position, quaternion(rot), mass)
        bodies.append(rigid)
        visual = link.find("visual")
        filename = Path(visual.find("geometry/mesh").get("filename")).name
        triangles = np.fromfile(mesh_dir / filename, dtype=stl, offset=84)["vertices"]
        origin = visual.find("origin")
        points = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")).apply(
            triangles.reshape(-1, 3)
        ) + np.fromstring(origin.get("xyz"), sep=" ")
        vertices, indices = np.unique(points, axis=0, return_inverse=True)
        shape = UsdGeom.Mesh.Define(stage, path + "/Housing")
        points_attr = shape.CreatePointsAttr(vertices.tolist())
        shape.CreateFaceVertexCountsAttr([3] * len(triangles))
        shape.CreateFaceVertexIndicesAttr(indices.tolist())
        shape.CreateSubdivisionSchemeAttr("none")
        shape.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(points_attr.Get()))
        shape.CreateDisplayColorAttr([Gf.Vec3f(.84, .85, .86)])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("convexHull")

    joint_root = str(root.GetPath()) + "/Joints"
    constrain(UsdPhysics.FixedJoint.Define(stage, joint_root + "/root_joint"), None, bodies[0])
    for i, (joint_spec, xyz, rot, axis) in enumerate(zip(arm.joints, arm.xyz, arm.rpy, arm.axes), 1):
        # USD의 Z축을 URDF의 +Z 또는 -Z에 맞춰 IK 각도의 부호를 보존한다.
        axis_frame = np.diag([1., axis[2], axis[2]])
        joint = constrain(UsdPhysics.RevoluteJoint.Define(stage, f"{joint_root}/joint_{i}"),
                          bodies[i-1], bodies[i], xyz, rot0=quaternion(rot @ axis_frame))
        joint.CreateLocalRot1Attr(Gf.Quatf(quaternion(axis_frame)))
        joint.CreateAxisAttr("Z")
        limits = joint_spec.find("limit")
        joint.CreateLowerLimitAttr(degrees(float(limits.get("lower"))))
        joint.CreateUpperLimitAttr(degrees(float(limits.get("upper"))))
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateMaxJointVelocityAttr(
            degrees(float(limits.get("velocity")))
        )
        # 가속도 드라이브 계수는 시뮬레이션 제어용이다.
        drive(joint, "angular", 0., 3500., 130., 1.e6)
        UsdPhysics.DriveAPI.Get(joint.GetPrim(), "angular").GetTypeAttr().Set("acceleration")
    output = HERE / "assets" / "RS080N.usdc"
    if not stage.GetRootLayer().Export(str(output)):
        raise RuntimeError(f"Could not save {output}")
    print(f"BUILT {output}", flush=True)


# 헤드 생성 옵션. 시험 맵에서 참조하는 fasten_head.usdc를 다시 만든다.
def build_thread_flank(stage, path, a, b):
    # M12×1.5 암나사의 한 구간을 볼록 충돌 형상으로 근사한다.
    from pxr import UsdGeom, UsdPhysics

    points = []
    for angle in (a,b):
        z = PITCH*angle/tau
        for radius, dz in ((.00545,0.),(.00645,-.00048),(.00645,.00048)):
            points.append((radius*cos(angle),radius*sin(angle),z+dz))
    mesh = UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3,3,4,4,4])
    mesh.CreateFaceVertexIndicesAttr([0,2,1,3,4,5,0,1,4,3,1,2,5,4,2,0,3,5])
    mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(False)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


def build_head():
    """플랜지에 연결할 스핀들 5개와 소켓·너트를 USD 레이어로 생성한다."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from sim_util import body, constrain, drive, mass_properties
    from visuals import ALLOY, ROBOT_YELLOW
    from wheel_geometry import nut, profile

    head_layer = STAGE_PATH.with_name("fasten_head.usdc")
    stage = Usd.Stage.Open(str(STAGE_PATH))
    stage.MuteLayer(str(head_layer))
    layer = Sdf.Layer.CreateAnonymous()
    stage.GetSessionLayer().subLayerPaths.append(layer.identifier)
    stage.SetEditTarget(layer)
    flange = stage.GetPrimAtPath(f"{ROBOT_PATH}/link_6")
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(flange)
    tool = str(flange.GetPath()) + "/Tool"
    plate_z = .16
    profile(stage, tool+"/FloatingPlate", [(.03,.145),(.103,.145),(.103,.175),(.03,.175)], ALLOY)
    # 중앙 D405와 반력 핀은 장비 배치를 보기 위한 근사 형상이다.
    camera = UsdGeom.Cube.Define(stage, tool+"/D405_Envelope")
    camera.CreateSizeAttr(1.)
    camera.AddTranslateOp().Set(Gf.Vec3d(0., 0., .35))
    camera.AddScaleOp().Set(Gf.Vec3f(.042, .042, .023))
    camera.CreateDisplayColorAttr([Gf.Vec3f(.15, .15, .16)])
    for i, x in enumerate((-.096, .096)):
        profile(stage, f"{tool}/ReactionPin_{i}", [(.001,.175),(.010,.175),(.010,.395),(.001,.400)], ALLOY, 24)
    UsdPhysics.MassAPI(flange).CreateMassAttr(5 * SPINDLE_MASS + 4.)
    mass_properties(flange, (0., 0., .21), (.80, .80, .20))
    for i, (root, (x, y)) in enumerate(zip(SPINDLES, BOLT_CENTERS)):
        # 허브 볼트 원주에 맞춰 스핀들 5개를 배치한다. 플랜지 +Z는 허브 쪽을 향한다.
        # QST50-150CT 하우징의 외경 51 mm와 길이 464 mm를 기준으로 배치한다.
        housing = UsdGeom.Cylinder.Define(stage, f"{tool}/QST50_150CT_{i}")
        housing.CreateRadiusAttr(SPINDLE_RADIUS)
        housing.CreateHeightAttr(.30)
        housing.AddTranslateOp().Set(Gf.Vec3d(x, -y, plate_z + .06))
        housing.CreateDisplayColorAttr([Gf.Vec3f(.20, .22, .23)])
        housing.GetPrim().SetCustomDataByKey("axialStrokeM", SPINDLE_STROKE)
        offset = Gf.Vec3d(x, -y, TOOL_LENGTH - .048)
        rotor = body(stage, root+"/Rotor", mat.Transform(offset), mat.ExtractRotationQuat(), .12)
        mass_properties(rotor, (0,0,.024), (4e-5,4e-5,2e-5))
        # 스핀들과 중공 소켓의 표시 메시를 만든다. 이 두 메시에는 외형 콜라이더를 적용하지 않는다.
        profile(stage, root+"/Rotor/Spindle", [(.003,-.008),(.011,-.008),(.011,.008),(.003,.008)], ALLOY, 48)
        profile(stage, root+"/Rotor/Socket", [(.0125,.008),(.016,.008),(.016,.048),(.0125,.048)], ROBOT_YELLOW, 24)
        # 스핀들마다 로컬 Z축 회전 관절과 독립 속도 드라이브를 만든다.
        bearing = constrain(UsdPhysics.RevoluteJoint.Define(stage,root+"/Bearing"), flange, rotor, offset)
        bearing.CreateAxisAttr("Z")
        drive(bearing,"angular",0.,0.,.15,2.)
        # 너트를 별도 강체로 만들고 원추형 안착면이 스터드를 향하게 배치한다.
        seat = .060
        nut_body = body(stage,root+"/Nut",mat.Transform(offset+Gf.Vec3d(0,0,seat)),
                        mat.ExtractRotationQuat(),.045)
        mass_properties(nut_body,(0,0,-.012),(3e-6,3e-6,3e-6))
        visual = UsdGeom.Xform.Define(stage,root+"/Nut/Visual")
        visual.AddRotateXOp().Set(180)
        nut(stage,str(visual.GetPath()))
        for k in range(48):
            build_thread_flank(stage, f"{visual.GetPath()}/InternalThread/Flank_{k}",tau*k/24,tau*(k+1)/24)
        # 초기에는 너트를 소켓에 고정해 함께 회전시킨다. 조립 시험에서 안착하면 이 고정을 해제한다.
        hold = constrain(UsdPhysics.FixedJoint.Define(stage,root+"/SocketRetention"),
                         rotor,nut_body,(0,0,seat))
        hold.CreateExcludeFromArticulationAttr(True)
    if not layer.Export(str(head_layer)):
        raise RuntimeError(f"Could not save {head_layer}")
    print(f"BUILT {head_layer}", flush=True)



# 접촉 시험 옵션. 암나사·수나사의 물리 접촉만으로 생기는 축 이동을 측정한다.
def check_thread_contact(app):
    """맞물린 스터드와 너트를 회전시켜 나사면 접촉만으로 생기는 전진량을 검사한다."""
    import omni.usd
    from isaacsim.core.api import SimulationContext
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    ctx = omni.usd.get_context()
    if not ctx.open_stage(str(STAGE_PATH)):
        raise RuntimeError(f"Could not open {STAGE_PATH}")
    while ctx.get_stage_loading_status()[2]:
        app.update()
    stage = ctx.get_stage()
    cache = UsdGeom.XformCache()
    hub = stage.GetPrimAtPath(HUB_PATH)
    hub_frame = cache.GetLocalToWorldTransform(hub)
    axis = np.array(hub_frame.TransformDir(Gf.Vec3d(0,0,1)).GetNormalized())
    # 나사 접촉쌍만 남기고, 로봇 이송과 중력을 제거한 상태로 시험한다.
    for path in (ROBOT_PATH, "/World/Wheel"):
        stage.GetPrimAtPath(path).SetActive(False)
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    scene = UsdPhysics.Scene.Get(stage,"/World/Physics")
    scene.CreateGravityMagnitudeAttr(0)

    def collider(prim, approximation):
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        phys = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        phys.CreateContactOffsetAttr(.00005)
        phys.CreateRestOffsetAttr(0.)

    bodies = []
    hinges = []
    for i, root in enumerate(SPINDLES):
        stud = stage.GetPrimAtPath(f"{hub.GetPath()}/Stud_{i}")
        thread = stage.GetPrimAtPath(f"{hub.GetPath()}/Thread_{i}")
        for part in (stud, thread):
            collider(part,"none")
        pts = np.array(UsdGeom.Mesh(stud).GetPointsAttr().Get())
        center = (pts.min(axis=0)+pts.max(axis=0))/2
        # 암나사 돌기가 수나사 골 사이에 오도록 미리 맞물린 위치를 정한다.
        center[2] = OFFSET+.00125+14.5*PITCH
        origin = hub_frame.Transform(Gf.Vec3d(*center.tolist()))
        path = f"/World/Contact/Nut_{i}"
        item = UsdGeom.Xform.Define(stage,path)
        item.AddTranslateOp().Set(origin)
        rot = Gf.Quatf(hub_frame.ExtractRotationQuat())
        item.AddOrientOp().Set(rot)
        prim = item.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(.045)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(3e-6))
        geometry = stage.DefinePrim(path+"/Geometry","Xform")
        geometry.GetReferences().AddInternalReference(f"{root}/Nut/Visual")
        UsdGeom.Xformable(geometry).ClearXformOpOrder()
        for part in Usd.PrimRange(geometry):
            if part.HasAPI(UsdPhysics.CollisionAPI):
                collider(part,"convexHull")
        joint = UsdPhysics.Joint.Define(stage,path+"/Guide")
        joint.CreateBody1Rel().SetTargets([path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(origin))
        joint.CreateLocalRot0Attr(rot)
        # 가이드가 축을 맞추고, 축 방향 이동과 축 회전은 자유롭게 둔다.
        for dof in ("transX","transY","rotX","rotY"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(),dof)
            limit.CreateLowAttr(1.)
            limit.CreateHighAttr(-1.)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(),"rotZ")
        drive.CreateDampingAttr(.001)
        drive.CreateMaxForceAttr(.12)
        hinges.append(drive.CreateTargetVelocityAttr(0.))
        bodies.append(prim)

    stage.GetPrimAtPath("/World/Head").SetActive(False)
    sim = SimulationContext(physics_dt=1/480, stage_units_in_meters=1,physics_prim_path="/World/Physics")
    sim.initialize_physics()
    sim.play()

    def poses():
        transforms = UsdGeom.XformCache()
        return [transforms.GetLocalToWorldTransform(p) for p in bodies]

    def measure():
        mats = poses()
        xyz = np.array([m.ExtractTranslation() for m in mats])
        local = [m*hub_frame.GetInverse() for m in mats]
        directions = [m.TransformDir(Gf.Vec3d(1,0,0)) for m in local]
        return xyz@axis,np.array([np.arctan2(v[1],v[0]) for v in directions])

    for _ in range(120):
        sim.step(render=False)
    start,last = measure()
    for speed in hinges:
        speed.Set(-360.)
    turns = np.zeros(5)
    for frame in range(480*12):
        sim.step(render=False)
        if frame%4 == 0:
            end,phase = measure()
            turns -= np.angle(np.exp(1j*(phase-last)))/tau
            last = phase
            if turns.min()>=8:
                break
    advance = start-end
    for i in range(5):
        print(f"CONTACT nut {i+1}: turns={turns[i]:.3f} advance_mm={advance[i]*1000:.3f} "
              f"pitch_error_mm={(advance[i]-turns[i]*PITCH)*1000:.3f}",flush=True)
    passed = bool(np.all(turns>=8) and np.all(np.abs(advance-turns*PITCH)<.0005))
    print("PASS" if passed else "FAIL","rotation-only thread contact",flush=True)
    return passed



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Check motion and exit")
    parser.add_argument("--captures", type=Path, help="Save loaded and seated close-ups")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--build-robot", type=Path, metavar="STL_DIR", help="Kawasaki STL로 로봇 USD 생성")
    modes.add_argument("--build-head", action="store_true", help="5축 체결 헤드 USD 생성")
    modes.add_argument("--check-thread-contact", action="store_true", help="나사면 접촉 시험")
    args = parser.parse_args()
    headless = args.headless or args.build_robot is not None or args.build_head or args.check_thread_contact
    app = SimulationApp({"headless": headless, "renderer": "RayTracedLighting",
                         "multi_gpu": False, "max_gpu_count": 1})
    try:
        if args.build_robot is not None:
            build_robot(args.build_robot)
            return 0
        if args.build_head:
            build_head()
            return 0
        if args.check_thread_contact:
            return 0 if check_thread_contact(app) else 1
        return run(app, args.headless, args.captures)
    finally:
        # USD 생성은 동기 Export로 끝난다. 생성 모드는 렌더러와 캡처 종료를 기다릴 필요가 없다.
        app.close(skip_cleanup=args.build_robot is not None or args.build_head)


if __name__ == "__main__":
    raise SystemExit(main())
