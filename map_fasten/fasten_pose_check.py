"""Isaac Sim 좌전륜 체결 시험. 기본 실행은 TaskController를 시작한다."""

import argparse
import json
from enum import Enum
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
TOOL_LENGTH = 0.235
SPINDLES = tuple(f"/World/Head/Axis_{i}" for i in range(5))

# 초기 시험 기준. 실제 공차/PhysX 결과에 맞추어 조정한다.
CENTER_TOL = .0005          # m, 축에 수직한 중심 오차
STANDOFF_TOL = .002         # m, 준비 거리 오차
AXIS_TOL_DEG = .5
PATTERN_TOL_DEG = .3
ENTRY_TOL = .0007          # m, 개별 너트 진입 위치
SEAT_TOL = .0007           # m
TURN_TOL = .15             # rev
RETRACT_CLEARANCE = .075   # m


class FastenState(Enum):
    """체결 공정 번호. 각 단계의 완료 조건은 TaskController.run()과 메서드 참조."""
    IDLE = 0           # 시작 대기
    DETECT = 1         # 휠/허브/스터드 좌표, 너트 5개 장전 확인
    CENTER_ALIGN = 2   # 공구 중심과 허브 중심 정렬 (0.5 mm 이하)
    AXIS_ALIGN = 3     # 공구 축과 허브 축 정렬 (0.5도 이하)
    PATTERN_ALIGN = 4  # 5개 스터드와 너트의 원주상 각도 정렬 (0.3도 이하)
    APPROACH = 5       # 너트를 스터드 끝점까지 이동 (개별 오차 0.7 mm 이하)
    ENGAGE = 6         # 진입 위치 검증 (실제 나사 물림 아님)
    FASTENING = 7      # 스핀들 회전 및 피치 기반 로봇 전진
    SEATING_CHECK = 8  # 너트 5개 안착 위치 및 회전수 확인
    LOCK = 9           # 안착 너트를 World Fixed Joint로 고정 (모사)
    RELEASE = 10       # 소켓-너트 고정 Joint 해제
    RETRACT = 11       # 로봇 후퇴 및 너트 잔류 확인
    COMPLETE = 12      # 한 휠의 체결 모사 종료
    RELOAD = 13        # 재장전 필요 (자동 장전 미구현)
    ERROR = 99         # 오류 발생, 공정 중단


# 출력용 설명: 터미널에서는 한글, Isaac Sim UI에서는 영문 상태명 사용.
STEP_NAMES = {
    FastenState.IDLE: "시작 대기",
    FastenState.DETECT: "휠·허브 좌표 및 너트 장전 확인",
    FastenState.CENTER_ALIGN: "공구와 허브 중심 정렬",
    FastenState.AXIS_ALIGN: "공구와 허브 중심축 정렬",
    FastenState.PATTERN_ALIGN: "볼트·너트 5축 회전각 정렬",
    FastenState.APPROACH: "너트를 스터드 끝점까지 이동",
    FastenState.ENGAGE: "너트 진입 위치 확인 (물림 모사)",
    FastenState.FASTENING: "5축 동시 체결 회전",
    FastenState.SEATING_CHECK: "너트 안착 확인",
    FastenState.LOCK: "너트 고정 (World Joint 모사)",
    FastenState.RELEASE: "소켓과 너트 분리",
    FastenState.RETRACT: "공구 후퇴 및 너트 유지 확인",
    FastenState.COMPLETE: "한 휠 체결 모사 완료",
    FastenState.RELOAD: "너트 재장전 대기",
    FastenState.ERROR: "공정 오류",
}


def format_step_report(state, event, metrics, nuts, **extra):
    """한 상태당 필요한 수치만 출력. 상세 JSON은 --json-log 옵션으로 선택한다."""
    prefix = f"[STEP {state.value:02d}] {STEP_NAMES[state]}"
    label = {"ENTER": "시작", "PASS": "통과", "READY": "준비", "PENDING": "대기",
             "FAIL": "실패", "INTERRUPTED": "중단"}.get(event, event)
    m = metrics
    def count(key):
        return sum(bool(n.get(key)) for n in nuts)
    def mm(key):
        return ", ".join(f"{v:.3f}" for v in m[key])
    detail = ""
    if event in ("PASS", "FAIL", "PENDING"):
        if state == FastenState.DETECT and "hub_xyz_m" in m:
            xyz = ", ".join(f"{v:.4f}" for v in m["hub_xyz_m"])
            detail = f"허브 XYZ=({xyz}) m | 장전 {count('loaded')}/5"
        elif state == FastenState.CENTER_ALIGN and "center_error_mm" in m:
            detail = f"중심 오차 {m['center_error_mm']:.3f}/{CENTER_TOL*1000:.1f} mm | 준비 거리 오차 {m['standoff_error_mm']:.3f}/{STANDOFF_TOL*1000:.1f} mm"
        elif state == FastenState.AXIS_ALIGN and "axis_error_deg" in m:
            detail = f"축 오차 {m['axis_error_deg']:.3f}/{AXIS_TOL_DEG:.1f} deg"
        elif state == FastenState.PATTERN_ALIGN and "max_pattern_error_deg" in m:
            detail = f"최대 각도 오차 {m['max_pattern_error_deg']:.3f}/{PATTERN_TOL_DEG:.1f} deg"
        elif state in (FastenState.APPROACH, FastenState.ENGAGE) and "entry_error_mm" in m:
            detail = f"너트 1~5 오차 [{mm('entry_error_mm')}] mm | 최대 {max(m['entry_error_mm']):.3f}/{ENTRY_TOL*1000:.1f} mm"
            if state == FastenState.ENGAGE:
                detail += " | 실제 나사 물림 미검증"
        elif state == FastenState.FASTENING and "turns" in m:
            detail = f"회전수 [{', '.join(f'{v:.3f}' for v in m['turns'])}] rev | 목표 {m['required_turns']:.3f} rev"
        elif state == FastenState.SEATING_CHECK and "seat_error_mm" in m:
            detail = f"안착 {count('seated')}/5 | 최대 위치 오차 {max(m['seat_error_mm']):.3f}/{SEAT_TOL*1000:.1f} mm | 최대 회전 오차 {max(m['turn_error_rev']):.3f}/{TURN_TOL:.2f} rev | 토크 미검증"
        elif state == FastenState.LOCK:
            detail = f"고정 {count('locked')}/5 | World Joint (허브 결합 아님)"
        elif state == FastenState.RELEASE:
            detail = f"분리 {count('released')}/5 | 남은 장전 {count('loaded')}/5"
        elif state == FastenState.RETRACT and "nut_drift_mm" in m:
            detail = f"이격 {m['tcp_clearance_mm']:.3f} mm (기준 > {RETRACT_CLEARANCE*1000:.0f}) | 너트 최대 변위 {max(m['nut_drift_mm']):.3f} mm"
        elif state == FastenState.COMPLETE:
            detail = f"안착 {count('seated')}/5 | 분리 {count('released')}/5 | 나사 물림·토크 미검증"
        elif state == FastenState.RELOAD:
            detail = f"장전 {count('loaded')}/5 | 자동 재장전 미구현"
        elif state == FastenState.ERROR:
            detail = f"실패 단계: {extra.get('failed_state', '?')} | 원인: {extra.get('reason', '?')}"
    return f"{prefix} : {label}" + (f" | {detail}" if detail else "")


# 실행 환경. 장면을 열고 재생·정지·재시작에 맞춰 체결 작업의 수명을 관리한다.
class CycleInterrupted(Exception):
    """타임라인 정지나 창 닫기에 맞춰 실행 중인 체결을 중단한다."""


def run(app, headless, captures, json_log=False):
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
    if not stage.GetPrimAtPath(SPINDLES[0] + "/Bearing"):
        raise RuntimeError("Run --build-head to regenerate the compact fastening head USD")

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
        window = ui.Window("CN7 Fastening", width=520, height=120)
        with window.frame, ui.VStack(spacing=4):
            status = ui.Label("Ready")
            with ui.HStack(spacing=4):
                ui.Button("Play", clicked_fn=timeline.play)
                ui.Button("Pause", clicked_fn=timeline.pause)
                ui.Button("Restart", clicked_fn=restart_cycle)

    def is_open():
        return app.is_running() and context.get_stage() == stage

    active_status = "IDLE"

    def state_changed(state, event):
        nonlocal active_status
        active_status = f"STEP {state.value:02d} | {state.name} | {event}"
        if not headless:
            status.text = active_status

    def step(frame):
        if not headless and (frame % 2 == 0 or not timeline.is_playing()):
            sim.render()
        while not timeline.is_playing() and not stopped and not restart and is_open():
            if not headless:
                status.text = f"Paused | {active_status}"
            sim.render()
        if stopped or restart or not is_open():
            raise CycleInterrupted
        if not headless:
            status.text = active_status
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
                    last_result = TaskController(app, stage, dt, step, captures, state_changed, json_log=json_log).run()
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
                status.text = "COMPLETE · Reload required · Restart resets scene" if last_result == 0 else f"Failed · {active_status} · Restart to retry"
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
    """명시적 공정 상태 + 완료 조건. 기존 단독 시험용 blocking 실행 방식 유지.

    ENGAGE/LOCK은 좌표 및 Joint 기반 모사이며 실제 나사 물림/토크가 아니다.
    RELOAD는 필요한 상태만 보고한다. 자동 재장전 장치는 아직 없다.
    """

    def __init__(self, app, stage, dt, step, captures, on_state=None, json_log=False):
        self.app, self.stage, self.dt = app, stage, dt
        self.step, self.captures = step, captures
        self.on_state = on_state
        self.json_log = json_log
        self.state = FastenState.IDLE
        self.completed = []
        self.metrics = {}
        self.nut_status = [dict(loaded=False, aligned=False, engaged_model=False,
                                seated=False, locked=False, released=False)
                           for _ in SPINDLES]
        self.rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        self.speeds = []
        self.reload_required = False
        self.cycle_complete = False

    def report(self, event, **extra):
        payload = dict(state=self.state.name, event=event, metrics=self.metrics,
                       nuts=self.nut_status, **extra)
        print(format_step_report(self.state, event, self.metrics, self.nut_status, **extra), flush=True)
        #if self.json_log:
            #print("FASTEN_STATE " + json.dumps(payload, ensure_ascii=False), flush=True)
        if self.on_state:
            self.on_state(self.state, event)

    def transition(self, state, action):
        self.state = state
        self.metrics = {}
        self.report("ENTER")
        result = action()   # 조건 불충족 시 예외: 이후 상태로 진입하지 않는다.
        if result is False:
            raise RuntimeError(f"{state.name} condition failed")
        self.completed.append(state.name)
        self.report("PASS")
        return result

    def require(self, condition, reason):
        if not condition:
            raise RuntimeError(reason)

    def run(self):
        try:
            self.report("READY")
            # STEP 01: GT 좌표 및 장전 확인. 조건 불충족 시 즉시 ERROR.
            self.transition(FastenState.DETECT, self.prepare)
            self.capture("loaded_sockets", [2.12, 2.45, 3.70], [1.8, 2.80, 3.10])
            # STEP 02: 안전 준비 위치에서 중심 오차와 접근 거리 확인.
            self.transition(FastenState.CENTER_ALIGN, self.center_align)
            # STEP 03: 공구 축과 허브 축이 반대 방향으로 평행한지 확인.
            self.transition(FastenState.AXIS_ALIGN, self.axis_align)
            # STEP 04: 72도 대칭을 고려해 볼트와 너트 5개 패턴 정렬.
            self.transition(FastenState.PATTERN_ALIGN, self.pattern_align)
            # STEP 05: 너트별 스터드 진입점까지 이동 및 오차 판정.
            self.transition(FastenState.APPROACH, self.align)
            # STEP 06: 기하학적 진입 확인. 실제 나사산 물림은 미구현.
            self.transition(FastenState.ENGAGE, self.engage)
            # STEP 07: 실측 회전수로 피치 기반 전진량 계산.
            start, turns = self.transition(FastenState.FASTENING, self.spin)
            # STEP 08: 너트별 안착 위치와 회전수 오차 검사.
            seated = self.transition(FastenState.SEATING_CHECK, lambda: self.seat(start, turns))
            # STEP 09: 안착된 너트를 World Joint로 고정 (고정 시험 전용).
            self.transition(FastenState.LOCK, self.lock)
            # STEP 10: 소켓의 유지 Joint를 해제하고 분리 여부 확인.
            self.transition(FastenState.RELEASE, self.release)
            # STEP 11: 공구 후퇴 후 이격 거리 및 너트 잔류 확인.
            self.transition(FastenState.RETRACT, lambda: self.retract(seated))
            self.capture("seated_nuts", [1.66, 1.10, 1.07], [1.36, .85, .94])
            self.state = FastenState.COMPLETE
            self.metrics = {"completion_mode": "joint_based_simulation",
                            "physical_thread_engagement_verified": False,
                            "torque_verified": False}
            self.cycle_complete = True
            self.completed.append(self.state.name)
            self.report("PASS")
            self.state = FastenState.RELOAD
            self.reload_required = True
            self.metrics = {"implemented": False, "required": True, "loaded_count": 0}
            self.report("PENDING", reason="Single-wheel test complete; automatic reload not implemented")
            return 0   # 한 휠의 모사 체결 성공. 재장전 성공이 아님.
        except CycleInterrupted:
            self.report("INTERRUPTED")
            raise
        except Exception as exc:
            failed_state = self.state.name
            self.state = FastenState.ERROR
            self.report("FAIL", failed_state=failed_state, reason=str(exc))
            raise
        finally:
            self.stop_spindles()

    def prepare(self):
        from isaacsim.core.prims import SingleXFormPrim
        from pxr import UsdPhysics, UsdGeom

        wheel = SingleXFormPrim("/World/Wheel", reset_xform_properties=False)
        base = SingleXFormPrim(f"{ROBOT_PATH}/base_link", reset_xform_properties=False)
        self.tcp = SingleXFormPrim(f"{ROBOT_PATH}/link_6/TCP", reset_xform_properties=False)
        self.wheel_xyz, wheel_wxyz = wheel.get_world_pose()
        self.base_xyz, base_wxyz = base.get_world_pose()
        hub = SingleXFormPrim(HUB_PATH, reset_xform_properties=False)
        self.hub_xyz, hub_wxyz = hub.get_world_pose()
        for pose in (self.wheel_xyz, wheel_wxyz, self.base_xyz, base_wxyz, self.hub_xyz, hub_wxyz):
            self.require(np.all(np.isfinite(pose)), "Invalid ground truth pose")
        self.outward = Rotation.from_quat(hub_wxyz, scalar_first=True).apply([0, 0, 1])
        # 원래 Arm FK와 고정 FL 시험 씬의 제한을 명시적으로 유지한다.
        self.require(np.allclose(self.outward, [0, 1, 0], atol=1e-5) and
                     np.allclose(np.abs(base_wxyz), [1, 0, 0, 0], atol=1e-5),
                     "Only fixed left-front fixture and unrotated robot base are supported")
        self.target = self.hub_xyz + self.outward * (NUT_SEAT + NUT_LENGTH + .08)
        self.flange_target = self.target + self.outward * TOOL_LENGTH
        self.arm = Arm(ROBOT_URDF)
        self.angles = self.arm.solve(self.base_xyz, self.flange_target, side=1)
        self.arm_drives = [self.stage.GetPrimAtPath(f"{ROBOT_PATH}/Joints/joint_{i}")
                           .GetAttribute("drive:angular:physics:targetPosition") for i in range(1, 7)]
        self.require(all(bool(d) for d in self.arm_drives), "Missing arm drive")
        self.nuts = [SingleXFormPrim(root + "/Nut", reset_xform_properties=False) for root in SPINDLES]
        self.speeds = []
        for i, root in enumerate(SPINDLES):
            bearing = self.stage.GetPrimAtPath(root + "/Bearing")
            self.require(bool(bearing) and bearing.HasAPI(UsdPhysics.DriveAPI, "angular"),
                         f"Missing spindle drive {root}")
            self.speeds.append(UsdPhysics.DriveAPI.Get(bearing, "angular").CreateTargetVelocityAttr(0.))
            hold = UsdPhysics.FixedJoint.Get(self.stage, root + "/SocketRetention")
            loaded = (bool(hold) and hold.GetJointEnabledAttr().Get() and
                      list(map(str, hold.GetBody0Rel().GetTargets())) == [root + "/Rotor"] and
                      list(map(str, hold.GetBody1Rel().GetTargets())) == [root + "/Nut"])
            self.nut_status[i]["loaded"] = bool(loaded)
        self.require(all(n["loaded"] for n in self.nut_status), "Initial five nuts are not retained")
        # 기존 고정 FL 씬의 스터드 끝점. 기울어진 실시간 허브에는 적용 불가.
        bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        tips = []
        for i in range(len(SPINDLES)):
            stud = self.stage.GetPrimAtPath(f"{HUB_PATH}/Stud_{i}")
            self.require(bool(stud), f"Missing stud {i}")
            box = bounds.ComputeWorldBound(stud).ComputeAlignedRange()
            self.require(not box.IsEmpty(), f"Empty stud bound {i}")
            tip = np.array(box.GetMidpoint())
            tip[1] = box.GetMax()[1]
            tips.append(tip)
        self.tips = np.array(tips)
        self.seats = self.tips.copy()
        self.seats[:, 1] = (self.hub_xyz + self.outward * NUT_SEAT)[1]
        self.turns_required = float(np.mean((self.tips - self.seats) @ self.outward) / PITCH)
        self.require(np.isfinite(self.turns_required) and self.turns_required > 0, "Invalid fastening travel")
        self.metrics = {"source": "ground_truth", "wheel_xyz_m": self.wheel_xyz.tolist(),
                        "hub_xyz_m": self.hub_xyz.tolist(), "target_tcp_m": self.target.tolist(),
                        "wheel_hub_lateral_offset_mm": float(np.linalg.norm(self.planar(self.wheel_xyz-self.hub_xyz))*1000),
                        "required_turns": self.turns_required}

    def planar(self, vector):
        vector = np.asarray(vector)
        return vector - np.expand_dims(vector @ self.outward, -1) * self.outward

    def command_arm(self, radians):
        for drive, angle in zip(self.arm_drives, np.rad2deg(radians)):
            drive.Set(float(angle))

    def solve_pose(self, target, rotation, seed):
        # Arm.solve()은 고정 방향만 지원한다. 같은 FK/Jacobian을 이용해 목표 자세를 확장.
        q = np.array(seed, dtype=float, copy=True)
        for _ in range(150):
            frames = self.arm.fk(self.base_xyz, q)
            p, r, _ = frames[-1]
            ep = np.asarray(target) - p
            er = Rotation.from_matrix(rotation @ r.T).as_rotvec()
            if np.linalg.norm(ep) < .0001 and np.linalg.norm(er) < .0008:
                return q
            jac = np.array([np.r_[np.cross(axis, p-origin), .3*axis]
                            for origin, _, axis in frames]).T
            dq = jac.T @ np.linalg.solve(jac @ jac.T + .0004*np.eye(6), np.r_[ep, .3*er])
            q = np.clip(q + np.clip(dq, -.25, .25), self.arm.lower, self.arm.upper)
        raise RuntimeError("Pose IK failed from current joint branch")

    def correct_arm(self, delta, rotation=None):
        commanded = self.arm.fk(self.base_xyz, self.angles)[-1][0]
        desired = self.rotation if rotation is None else rotation
        self.angles = self.solve_pose(commanded + delta, desired, self.angles)
        self.rotation = desired
        self.command_arm(self.angles)

    def settle(self, frames):
        for frame in range(frames):
            self.step(frame)

    def tcp_errors(self):
        xyz, quat = self.tcp.get_world_pose()
        axis = Rotation.from_quat(quat, scalar_first=True).apply([0, 0, 1])
        delta = self.target - xyz
        return (float(np.linalg.norm(self.planar(delta))), float(abs(delta @ self.outward)),
                float(np.degrees(np.arccos(np.clip(axis @ -self.outward, -1, 1)))))

    def update_tcp_metrics(self):
        c, z, a = self.tcp_errors()
        self.metrics.update(center_error_mm=c*1000, standoff_error_mm=z*1000, axis_error_deg=a)
        return c, z, a

    def center_align(self):
        # 기존 접근 궤적: 베이스 회전 후 팔 접기. 안전거리에서 중심 정렬.
        turned = np.zeros_like(self.angles)
        turned[0] = self.angles[0]
        for start, goal in ((np.zeros_like(turned), turned), (turned, self.angles)):
            for frame in range(720):
                fraction = min((frame + 1) * self.dt / 3, 1)
                blend = fraction**2 * (3 - 2*fraction)
                self.command_arm(start + (goal-start)*blend)
                self.step(frame)
        for _ in range(30):
            c,z,_ = self.update_tcp_metrics()
            if c <= CENTER_TOL and z <= STANDOFF_TOL:
                return
            xyz,_ = self.tcp.get_world_pose()
            delta = self.target-xyz
            delta *= min(1., .003/max(np.linalg.norm(delta), 1e-9))
            self.correct_arm(delta)
            self.settle(24)
        c,z,_ = self.update_tcp_metrics()
        self.require(c <= CENTER_TOL and z <= STANDOFF_TOL, "Center alignment timed out")

    def axis_align(self):
        for _ in range(20):
            c,z,a = self.update_tcp_metrics()
            if c <= CENTER_TOL and z <= STANDOFF_TOL and a <= AXIS_TOL_DEG:
                return
            xyz,quat = self.tcp.get_world_pose()
            actual = Rotation.from_quat(quat, scalar_first=True).as_matrix()
            axis = actual[:,2]
            cross = np.cross(axis, -self.outward)
            norm = np.linalg.norm(cross)
            if norm > 1e-9:
                angle = np.arctan2(norm, np.clip(axis @ -self.outward,-1,1))
                correction = Rotation.from_rotvec(cross/norm*min(angle,.035)).as_matrix()
                desired = correction @ self.rotation
            else:
                self.require(axis @ -self.outward > 0, "Tool axis reversed")
                desired = self.rotation
            delta = self.target-xyz
            delta *= min(1., .003/max(np.linalg.norm(delta),1e-9))
            self.correct_arm(delta, desired)
            self.settle(24)
        c,z,a = self.update_tcp_metrics()
        self.require(c <= CENTER_TOL and z <= STANDOFF_TOL and a <= AXIS_TOL_DEG,
                     "Axis alignment timed out")

    def nut_positions(self):
        return np.array([nut.get_world_pose()[0] for nut in self.nuts])

    def pattern_angles(self, actual, target):
        # 중심 이동을 제거한 뒤 동일 평면에 투영해 각 축의 signed phase를 계산한다.
        a = self.planar(actual - actual.mean(axis=0))
        b = self.planar(target - target.mean(axis=0))
        self.require(np.all(np.linalg.norm(a,axis=1)>1e-6) and
                     np.all(np.linalg.norm(b,axis=1)>1e-6), "Degenerate bolt pattern")
        return np.arctan2(np.cross(a,b) @ self.outward, np.sum(a*b,axis=1))

    def pattern_align(self):
        # 5축 대칭. cyclic 대응만 허용하여 불필요한 72도 회전을 피한다.
        actual = self.nut_positions()
        choices = []
        for shift in range(len(SPINDLES)):
            ids = np.roll(np.arange(len(SPINDLES)), shift)
            angles = self.pattern_angles(actual, self.tips[ids])
            phase = np.angle(np.mean(np.exp(1j*angles)))
            residual = np.max(np.abs(np.angle(np.exp(1j*(angles-phase)))))
            choices.append((abs(phase)+residual, ids))
        _,ids = min(choices,key=lambda v:v[0])
        self.tips, self.seats = self.tips[ids].copy(), self.seats[ids].copy()
        self.metrics["stud_mapping"] = ids.tolist()
        for _ in range(30):
            angles = self.pattern_angles(self.nut_positions(),self.tips)
            phase = float(np.angle(np.mean(np.exp(1j*angles))))
            c,z,a = self.update_tcp_metrics()
            max_deg = float(np.max(np.abs(np.rad2deg(angles))))
            self.metrics.update(pattern_error_deg=float(np.rad2deg(phase)), max_pattern_error_deg=max_deg)
            if max_deg <= PATTERN_TOL_DEG and c <= CENTER_TOL and z <= STANDOFF_TOL and a <= AXIS_TOL_DEG:
                return
            correction = Rotation.from_rotvec(self.outward*np.clip(phase,-.035,.035)).as_matrix()
            xyz,_ = self.tcp.get_world_pose()
            delta = self.target-xyz
            delta *= min(1., .003/max(np.linalg.norm(delta),1e-9))
            self.correct_arm(delta, correction @ self.rotation)
            self.settle(24)
        angles = self.pattern_angles(self.nut_positions(),self.tips)
        c,z,a = self.update_tcp_metrics()
        self.metrics["max_pattern_error_deg"] = float(np.max(np.abs(np.rad2deg(angles))))
        self.require(self.metrics["max_pattern_error_deg"] <= PATTERN_TOL_DEG and c <= CENTER_TOL
                     and z <= STANDOFF_TOL and a <= AXIS_TOL_DEG, "Pattern alignment timed out")

    def move_nuts(self, goal, frames):
        for frame in range(frames):
            if frame % 12 == 0:
                error = goal.mean(axis=0) - self.nut_positions().mean(axis=0)
                delta = error * min(1., .006 / max(np.linalg.norm(error), 1e-9))
                self.correct_arm(delta)
            self.step(frame)

    def align(self):
        self.move_nuts(self.tips, 360)
        errors = np.linalg.norm(self.nut_positions()-self.tips,axis=1)
        self.metrics["entry_error_mm"] = (errors*1000).tolist()
        for n,e in zip(self.nut_status,errors):
            n["aligned"] = bool(e <= ENTRY_TOL)
        self.require(all(n["aligned"] for n in self.nut_status), "Stud entry position error exceeds 0.7 mm")

    def engage(self):
        # 기본 맵에는 활성 나사산 접촉이 없다. 진입 좌표만 검사한다.
        errors = np.linalg.norm(self.nut_positions()-self.tips,axis=1)
        self.metrics = {"mode":"geometric_entry_only", "physical_contact_verified":False,
                        "entry_error_mm":(errors*1000).tolist()}
        for n,e in zip(self.nut_status,errors):
            n["engaged_model"] = bool(n["loaded"] and e<=ENTRY_TOL)
        self.require(all(n["engaged_model"] for n in self.nut_status), "Geometric entry check failed")

    def phases(self):
        rotations = [Rotation.from_quat(nut.get_world_pose()[1], scalar_first=True) for nut in self.nuts]
        axes = np.array([r.apply([1,0,0]) for r in rotations])
        return np.arctan2(axes[:,2],axes[:,0])

    def stop_spindles(self):
        for speed in self.speeds:
            speed.Set(0.)

    def spin(self):
        last = self.phases()
        turns = np.zeros(len(SPINDLES))
        start = self.nut_positions()
        try:
            for speed in self.speeds:
                speed.Set(720.)
            for frame in range(1800):
                self.step(frame)
                phase = self.phases()
                turns += np.angle(np.exp(1j*(phase-last)))/tau
                last = phase
                if frame%12 == 0:
                    advance = min(self.turns_required, float(turns.min()))*PITCH
                    goal = self.tips-self.outward*advance
                    self.correct_arm(goal.mean(axis=0)-self.nut_positions().mean(axis=0))
                if turns.min() >= self.turns_required:
                    break
            self.metrics = {"turns":turns.tolist(),"required_turns":self.turns_required,
                            "mode":"pitch_based_arm_feed"}
            self.require(np.all(turns >= self.turns_required), "Fastening rotation timeout")
            return start,turns
        finally:
            self.stop_spindles()

    def seat(self,start,turns):
        self.move_nuts(self.seats,120)
        end = self.nut_positions()
        errors = np.linalg.norm(end-self.seats,axis=1)
        turn_errors = np.abs(turns-self.turns_required)
        self.metrics = {"seat_error_mm":(errors*1000).tolist(),"turn_error_rev":turn_errors.tolist(),
                        "travel_mm":((start-end)@self.outward*1000).tolist(),"torque_verified":False}
        for n,e,t in zip(self.nut_status,errors,turn_errors):
            n["seated"] = bool(e<SEAT_TOL and t<TURN_TOL)
        self.require(all(n["seated"] for n in self.nut_status), "Five-nut joint-based seating check failed")
        return end

    def lock(self):
        from pxr import Gf, UsdPhysics
        # 고정 FL 시험에 한해 World 고정. 차량 이동을 위한 Hub 결합은 미구현.
        # LOCK과 RELEASE 사이에는 physics step을 진행하지 않는다.
        for i,(root,nut) in enumerate(zip(SPINDLES,self.nuts)):
            xyz,quat = nut.get_world_pose()
            lock = UsdPhysics.FixedJoint.Define(self.stage,root+"/SeatedJoint")
            lock.CreateBody1Rel().SetTargets([root+"/Nut"])
            lock.CreateLocalPos0Attr(Gf.Vec3f(*xyz.tolist()))
            lock.CreateLocalRot0Attr(Gf.Quatf(float(quat[0]),Gf.Vec3f(*quat[1:].tolist())))
            lock.CreateExcludeFromArticulationAttr(True)
            lock.CreateJointEnabledAttr(True)
            self.nut_status[i]["locked"] = bool(lock.GetJointEnabledAttr().Get() and
                not lock.GetBody0Rel().GetTargets() and
                list(map(str,lock.GetBody1Rel().GetTargets())) == [root+"/Nut"])
        self.metrics = {"mode":"world_fixed_joint", "hub_attachment":False}
        self.require(all(n["locked"] for n in self.nut_status), "SeatedJoint creation failed")

    def release(self):
        from pxr import UsdPhysics
        for i,root in enumerate(SPINDLES):
            hold = UsdPhysics.FixedJoint.Get(self.stage,root+"/SocketRetention")
            hold.CreateJointEnabledAttr(False)
            released = not hold.GetJointEnabledAttr().Get()
            self.nut_status[i]["released"] = bool(released)
            self.nut_status[i]["loaded"] = not released
        self.require(all(n["released"] for n in self.nut_status), "SocketRetention release failed")

    def retract(self,seated):
        retract = self.solve_pose(self.flange_target,self.rotation,self.angles)
        for frame in range(240):
            fraction = min(1.,(frame+1)/180)
            self.command_arm(self.angles+(retract-self.angles)*fraction)
            self.step(frame)
        movement = np.linalg.norm(self.nut_positions()-seated,axis=1)
        retreat = float((self.tcp.get_world_pose()[0]-self.seats.mean(axis=0))@self.outward)
        self.metrics = {"nut_drift_mm":(movement*1000).tolist(),"tcp_clearance_mm":retreat*1000}
        self.require(movement.max()<SEAT_TOL and retreat>RETRACT_CLEARANCE,
                     "Retraction or nut retention check failed")

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
    from tempfile import TemporaryDirectory

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from sim_util import body, constrain, drive, mass_properties, tensor_properties
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
    UsdGeom.Xform.Define(stage, tool)
    parts = []

    def fixed_mesh(name, xy, radius, start, end, mass, color):
        path = tool + "/" + name
        profile(stage, path, [(.001, start), (radius, start), (radius, end), (.001, end)],
                color, 48, xy=xy)
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        height = end - start
        transverse = mass * (3 * radius**2 + height**2) / 12
        parts.append((mass, (*xy, (start + end) / 2),
                      (transverse, transverse, mass * radius**2 / 2)))

    rotor_z = TOOL_LENGTH - .048
    fixed_mesh("FlangeAdapter", (0., 0.), .055, 0., .030, .8, ALLOY)
    fixed_mesh("MainPlate", (0., 0.), .081, .030, .110, 2., ALLOY)
    for i, (root, (x, y)) in enumerate(zip(SPINDLES, BOLT_CENTERS)):
        # 플랜지 +Z는 허브를 향한다. 기존 스터드 순서와 Y 부호를 유지한다.
        fixed_mesh(f"Housing_{i}", (x, -y), .020, .110, rotor_z - .007,
                   .8, (.20, .22, .23))
        offset = Gf.Vec3d(x, -y, rotor_z)
        rotor = body(stage, root+"/Rotor", mat.Transform(offset), mat.ExtractRotationQuat(), .12)
        mass_properties(rotor, (0,0,.024), (4e-5,4e-5,2e-5))
        # 스핀들과 중공 소켓의 표시 메시를 만든다. 이 두 메시에는 외형 콜라이더를 적용하지 않는다.
        profile(stage, root+"/Rotor/Spindle", [(.003,-.009),(.013,-.009),(.013,.009),(.003,.009)], ALLOY, 48)
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
    # 기존 링크 형상의 로컬 경계 상자로 링크 자체의 질량 분포를 근사한다.
    flange_mass = UsdPhysics.MassAPI(flange)
    link_mass = flange_mass.GetMassAttr().Get()
    bounds = UsdGeom.Mesh.Get(stage, str(flange.GetPath()) + "/Housing").GetExtentAttr().Get()
    low, high = np.asarray(bounds)
    size = high - low
    parts.append((link_mass, (low + high) / 2,
                  link_mass / 12 * (np.sum(size**2) - size**2)))
    total_mass = sum(mass for mass, _, _ in parts)
    center = sum(mass * np.asarray(pos) for mass, pos, _ in parts) / total_mass
    inertia = np.zeros((3, 3))
    for mass, pos, diagonal in parts:
        delta = np.asarray(pos) - center
        inertia += np.diag(diagonal) + mass * (delta @ delta * np.eye(3) - np.outer(delta, delta))
    flange_mass.CreateMassAttr(total_mass)
    tensor_properties(flange, center, inertia)

    validate_head(stage, flange)
    with TemporaryDirectory(dir=head_layer.parent) as directory:
        candidate = Path(directory) / head_layer.name
        if not layer.Export(str(candidate)):
            raise RuntimeError(f"Could not export {candidate}")
        candidate.replace(head_layer)
    print(f"BUILT {head_layer}", flush=True)


def validate_head(stage, flange):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    flange_path = str(flange.GetPath())
    tool = stage.GetPrimAtPath(flange_path + "/Tool")
    fixed_names = ["FlangeAdapter", "MainPlate"] + [f"Housing_{i}" for i in range(5)]
    for name in fixed_names:
        if not UsdGeom.Mesh.Get(stage, tool.GetPath().AppendChild(name)):
            raise RuntimeError(f"Missing fixed tool mesh: {name}")
    if any(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in Usd.PrimRange(tool)):
        raise RuntimeError("Fixed tool meshes must belong to link_6")
    cache = UsdGeom.XformCache()
    tcp = stage.GetPrimAtPath(flange_path + "/TCP")
    flange_frame = cache.GetLocalToWorldTransform(flange)
    tip = cache.GetLocalToWorldTransform(tcp).ExtractTranslation()
    if (tip - flange_frame.Transform(Gf.Vec3d(0, 0, TOOL_LENGTH))).GetLength() > 1e-6:
        raise RuntimeError("TCP does not match the socket tip")
    for root in SPINDLES:
        rotor = root + "/Rotor"
        nut_body = root + "/Nut"
        for path in (rotor, nut_body):
            if not stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f"Missing rigid body: {path}")
        for name in ("Spindle", "Socket"):
            if not UsdGeom.Mesh.Get(stage, rotor + "/" + name):
                raise RuntimeError(f"Missing rotor mesh: {rotor}/{name}")
        bearing = UsdPhysics.RevoluteJoint.Get(stage, root + "/Bearing")
        hold = UsdPhysics.FixedJoint.Get(stage, root + "/SocketRetention")
        if not bearing or bearing.GetAxisAttr().Get() != "Z" or not bearing.GetPrim().HasAPI(
            UsdPhysics.DriveAPI, "angular"
        ):
            raise RuntimeError(f"Missing Z-axis bearing drive: {root}")
        for joint, path0, path1 in ((bearing, flange_path, rotor), (hold, rotor, nut_body)):
            if (not joint or list(map(str, joint.GetBody0Rel().GetTargets())) != [path0]
                    or list(map(str, joint.GetBody1Rel().GetTargets())) != [path1]):
                raise RuntimeError(f"Invalid joint bodies: {root}")
            frame0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path0))
            frame1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path1))
            anchor0 = frame0.Transform(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
            anchor1 = frame1.Transform(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
            rot0 = frame0.ExtractRotation() * Gf.Rotation(Gf.Quatd(joint.GetLocalRot0Attr().Get()))
            rot1 = frame1.ExtractRotation() * Gf.Rotation(Gf.Quatd(joint.GetLocalRot1Attr().Get()))
            if (anchor0 - anchor1).GetLength() > 1e-6 or any(
                (rot0.TransformDir(axis) - rot1.TransformDir(axis)).GetLength() > 1e-6
                for axis in (Gf.Vec3d.XAxis(), Gf.Vec3d.ZAxis())
            ):
                raise RuntimeError(f"Mismatched world joint frames: {joint.GetPath()}")


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
    parser.add_argument("--json-log", action="store_true", help="상세 상태 JSON도 함께 출력")
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
        return run(app, args.headless, args.captures, json_log=args.json_log)
    finally:
        # USD 생성은 동기 Export로 끝난다. 생성 모드는 렌더러와 캡처 종료를 기다릴 필요가 없다.
        app.close(skip_cleanup=args.build_robot is not None or args.build_head)


if __name__ == "__main__":
    raise SystemExit(main())
