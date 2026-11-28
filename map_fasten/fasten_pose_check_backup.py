"""Isaac Sim 고정 차량 좌측 전·후륜 연속 체결 및 모사 재장전 시험."""

import argparse
import json
from enum import Enum
import sys
from math import cos, degrees, isfinite, sin, tau
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
WHEEL_TARGETS = {"FL": (HUB_PATH, "/World/Wheel"),
                 "RL": ("/World/Car/Visuals/RL", "/World/WheelRear")}
DUAL_BASE = (0., 2.45, 0.)  # 정적 시험용 베이스; 양쪽 목표의 IK 검증 후 사용
RELOAD_BOX = "/World/NutReloadBox"
RELOAD_PORT = RELOAD_BOX + "/ReloadTCP"
RELOAD_XY = (0., 1.65)
RELOAD_TOP = .65
RELOAD_DOCK_GAP = .025  # 소켓 TCP와 상판 간격; 너트 12mm 돌출 여유 포함
RELOAD_APPROACH = .20
RELOAD_POS_TOL = .001
RELOAD_AXIS_TOL = 1.0
RELOAD_HOLD_SECONDS = .5
NUT_ROTOR_OFFSET = .060  # 기존 build_head()와 동일


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
    RELOAD = 13        # 박스 접근·정렬·신규 너트 결합·후퇴 (공급 모사)
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
    FastenState.RELOAD: "너트 재장전",
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
            physical = m.get(
                "physical_thread_engagement_verified",
                False
            )

            mode = (
                "물리 나사 물림 검증 완료"
                if physical
                else "Joint 기반 체결 모사"
            )

            detail = (
                f"안착 {count('seated')}/5 | "
                f"분리 {count('released')}/5 | "
                f"{mode} | 최종 토크 미검증"
            )
        elif state == FastenState.RELOAD:
            detail = f"장전 {count('loaded')}/5 | {m.get('mode', '박스 장전 모사')}"
        elif state == FastenState.ERROR:
            detail = f"실패 단계: {extra.get('failed_state', '?')} | 원인: {extra.get('reason', '?')}"
    return f"{prefix} : {label}" + (f" | {detail}" if detail else "")


# 실행 환경. 장면을 열고 재생·정지·재시작에 맞춰 체결 작업의 수명을 관리한다.
class CycleInterrupted(Exception):
    """타임라인 정지나 창 닫기에 맞춰 실행 중인 체결을 중단한다."""


def setup_dual_scene(stage):
    """고정 차량 시험: 원본 USD/USDC는 건드리지 않고 세션 레이어에만 구성한다."""
    from pxr import Gf, UsdGeom, UsdPhysics

    # 기존 헤드의 Rotor/Nut는 월드 위치에 작성되어 있어 베이스와 함께 평행 이동.
    base_attr = stage.GetPrimAtPath(ROBOT_PATH).GetAttribute("xformOp:translate")
    delta = np.array(DUAL_BASE) - np.array(base_attr.Get())
    base_attr.Set(Gf.Vec3d(*DUAL_BASE))
    stage.GetPrimAtPath(ROBOT_PATH + "/Joints/root_joint").GetAttribute(
        "physics:localPos0").Set(Gf.Vec3f(*DUAL_BASE))
    for root in SPINDLES:
        for part in ("Rotor", "Nut"):
            attr = stage.GetPrimAtPath(root + "/" + part).GetAttribute("xformOp:translate")
            if not attr or attr.Get() is None:
                raise RuntimeError(f"Missing initial head transform: {root}/{part}")
            attr.Set(Gf.Vec3d(*(np.array(attr.Get())+delta).tolist()))

    # 후륜은 실제 차량 에셋의 RL 허브 중심에서 기존 휠 에셋을 재사용.
    rear_hub = stage.GetPrimAtPath(WHEEL_TARGETS["RL"][0])
    if not rear_hub:
        raise RuntimeError("Rear hub RL is missing from the car asset")
    rear_pose = UsdGeom.XformCache().GetLocalToWorldTransform(rear_hub)
    rear = stage.DefinePrim(WHEEL_TARGETS["RL"][1], "Xform")
    rear.GetReferences().AddInternalReference("/World/Wheel")
    UsdGeom.Xformable(rear).MakeMatrixXform().Set(rear_pose)
    rear.GetAttribute("physics:rigidBodyEnabled").Set(False)

    # 박스는 고정 Collider, 상단 원은 장전 구역을 표시하는 시각 형상.
    x,y = RELOAD_XY
    UsdGeom.Xform.Define(stage, RELOAD_BOX)
    box = UsdGeom.Cube.Define(stage, RELOAD_BOX+"/Body")
    box.CreateSizeAttr(1.)
    box.AddTranslateOp().Set(Gf.Vec3d(x,y,RELOAD_TOP/2))
    box.AddScaleOp().Set(Gf.Vec3d(.42,.36,RELOAD_TOP))
    box.CreateDisplayColorAttr([Gf.Vec3f(.24,.29,.34)])
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    disk = UsdGeom.Cylinder.Define(stage, RELOAD_BOX+"/ReloadPort")
    disk.CreateAxisAttr("Z")
    disk.CreateRadiusAttr(.102)
    disk.CreateHeightAttr(.003)
    disk.AddTranslateOp().Set(Gf.Vec3d(x,y,RELOAD_TOP+.0015))
    disk.CreateDisplayColorAttr([Gf.Vec3f(.12,.45,.72)])
    port = UsdGeom.Xform.Define(stage, RELOAD_PORT)
    port.AddTranslateOp().Set(Gf.Vec3d(x,y,RELOAD_TOP+RELOAD_DOCK_GAP))
    port.AddRotateXOp().Set(180.)  # 공구 +Z는 아래쪽
    print(f"[SETUP] base={DUAL_BASE}, rear={tuple(rear_pose.ExtractTranslation())}, reload={RELOAD_XY}", flush=True)


def run(app, headless, captures, json_log=False, wheel_order=("RL", "FL"), physical_threads=False):
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
    substeps = 4 if physical_threads else 1

    sim = SimulationContext(
        physics_dt=dt / substeps,
        rendering_dt=1 / 60,
        stage_units_in_meters=1,
        physics_prim_path="/World/Physics"
    )
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_end_time(300.)
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

        set_camera_view(eye=[4.4, 5.4, 3.5], target=[0., 1.5, 1.1])
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

    def state_changed(state, event, wheel_id):
        nonlocal active_status
        active_status = f"{wheel_id} | STEP {state.value:02d} | {state.name} | {event}"
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

        for _ in range(substeps):
            sim.step(render=False)

    session = stage.GetSessionLayer()
    # 관절 명령과 안착 조인트는 임시 레이어에 쓴다. 재시작 때 비우면 너트가 소켓에 고정된 초기 상태로 돌아간다.
    cycle_layer = Sdf.Layer.CreateAnonymous("fasten_cycle.usda")
    setup_layer = Sdf.Layer.CreateAnonymous("fasten_dual_setup.usda")
    session.subLayerPaths.insert(0, setup_layer.identifier)
    session.subLayerPaths.insert(0, cycle_layer.identifier)
    try:
        with Usd.EditContext(stage, setup_layer):
            setup_dual_scene(stage)
    except Exception:
        session.subLayerPaths.remove(cycle_layer.identifier)
        session.subLayerPaths.remove(setup_layer.identifier)
        stop_subscription.unsubscribe()
        raise
    last_result = 0
    timeline.play()
    timeline.commit()
    try:
        while is_open():
            try:
                sim.initialize_physics()
                with Usd.EditContext(stage, cycle_layer):
                    last_result = TaskController(app, stage, dt, step, captures, state_changed,
                                                 json_log=json_log, wheel_order=wheel_order,
                                                 physical_threads=physical_threads).run()
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
                status.text = "COMPLETE · Reloaded · Restart resets scene" if last_result == 0 else f"Failed · {active_status} · Restart to retry"
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
        session.subLayerPaths.remove(setup_layer.identifier)
    return last_result


# 체결 공정. run()에 작업 순서가 있고, 각 메서드에 해당 단계의 제어와 판정이 있다.
class TaskController:
    """명시적 공정 상태 + 완료 조건. 기존 단독 시험용 blocking 실행 방식 유지.

    ENGAGE/LOCK은 좌표 및 Joint 기반 모사이며 실제 나사 물림/토크가 아니다.
    RELOAD는 박스에서 신규 너트를 생성·결합하는 공급 모사이다.
    """

    def __init__(self, app, stage, dt, step, captures, on_state=None, json_log=False, wheel_order=("RL", "FL"), physical_threads=False):
        self.app, self.stage, self.dt = app, stage, dt
        self.step, self.captures = step, captures
        self.on_state = on_state
        self.json_log = json_log
        self.physical_threads = physical_threads
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
        self.wheel_order = tuple(wheel_order)
        self.target_wheel = self.wheel_order[0]
        self.wheel_status = {w: "WAITING" for w in self.wheel_order}
        self.nut_paths = [root+"/Nut" for root in SPINDLES]
        self.hold_paths = [root+"/SocketRetention" for root in SPINDLES]
        self.batch_id = 0
        self.finished_nuts = {}  # 체결 완료 너트의 경로와 안착 위치 저장
        self.angles = np.zeros(6)  # 현재 명령 관절각; 다음 휠에서도 유지

    def report(self, event, **extra):
        payload = dict(state=self.state.name, event=event, metrics=self.metrics,
                       nuts=self.nut_status, wheel=self.target_wheel, wheel_status=self.wheel_status, **extra)
        print(f"[{self.target_wheel}] " + format_step_report(self.state, event, self.metrics, self.nut_status, **extra), flush=True)
        if self.json_log:
            print("FASTEN_STATE " + json.dumps(payload, ensure_ascii=False), flush=True)
        if self.on_state:
            self.on_state(self.state, event, self.target_wheel)

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
        """상위 공정: 대상 휠 변경 → STEP 01~12 → STEP 13 → 다음 휠."""
        try:
            for wheel_id in self.wheel_order:
                self.target_wheel = wheel_id
                self.wheel_status[wheel_id] = "FASTENING"
                self.completed = []
                self.state = FastenState.IDLE
                self.metrics = {}
                self.report("READY")
                self.run_fastening()
                self.reload_required = True
                self.transition(FastenState.RELOAD, self.reload)
                self.verify_finished_nuts()
            self.cycle_complete = True
            self.state = FastenState.IDLE
            self.metrics = {"loaded_count": 5}
            self.report("READY")
            print(f"[SIDE COMPLETE] {self.wheel_status} | 장전 5/5 | 고정 차량 체결 모사", flush=True)
            return 0
        except CycleInterrupted:
            self.report("INTERRUPTED")
            raise
        except Exception as exc:
            failed_state = self.state.name
            if self.wheel_status[self.target_wheel] != "COMPLETE":
                self.wheel_status[self.target_wheel] = "ERROR"
            self.state = FastenState.ERROR
            self.report("FAIL", failed_state=failed_state, reason=str(exc))
            raise
        finally:
            self.stop_spindles()

    def run_fastening(self):
        """하위 공정: 하나의 휠에 대해 기존 STEP 01~12를 재사용."""
        self.transition(FastenState.DETECT, self.prepare)
        self.capture(f"{self.target_wheel}_loaded_sockets", self.hub_xyz + [1., 2., 1.5], self.hub_xyz)
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
        if not self.physical_threads:
            self.transition(FastenState.LOCK, self.lock)
        # STEP 10: 소켓의 유지 Joint를 해제하고 분리 여부 확인.
        self.transition(FastenState.RELEASE, self.release)
        # STEP 11: 공구 후퇴 후 이격 거리 및 너트 잔류 확인.
        self.transition(FastenState.RETRACT, lambda: self.retract(seated))
        self.capture(f"{self.target_wheel}_seated_nuts", self.hub_xyz + [.30, .40, .13], self.hub_xyz)
        self.finished_nuts[self.target_wheel] = (tuple(self.nut_paths), seated.copy())
        self.verify_finished_nuts()
        self.state = FastenState.COMPLETE
        self.metrics = {
            "completion_mode": (
                "physical_thread_rundown"
                if self.physical_threads
                else "joint_based_simulation"
            ),
            "physical_thread_engagement_verified": (
                self.physical_threads
                and all(
                    n["engaged_model"]
                    for n in self.nut_status
                )
            ),
            "torque_verified": False
        }
        self.wheel_status[self.target_wheel] = "COMPLETE"
        self.completed.append(self.state.name)
        self.report("PASS")

    def prepare(self):
        from isaacsim.core.prims import SingleXFormPrim
        from pxr import UsdPhysics, UsdGeom

        self.hub_path, wheel_path = WHEEL_TARGETS[self.target_wheel]
        self.require(bool(self.stage.GetPrimAtPath(wheel_path)), f"Missing wheel: {wheel_path}")
        wheel = SingleXFormPrim(wheel_path, reset_xform_properties=False)
        base = SingleXFormPrim(f"{ROBOT_PATH}/base_link", reset_xform_properties=False)
        self.tcp = SingleXFormPrim(f"{ROBOT_PATH}/link_6/TCP", reset_xform_properties=False)
        self.wheel_xyz, wheel_wxyz = wheel.get_world_pose()
        self.base_xyz, base_wxyz = base.get_world_pose()
        hub = SingleXFormPrim(self.hub_path, reset_xform_properties=False)
        self.hub_xyz, hub_wxyz = hub.get_world_pose()
        for pose in (self.wheel_xyz, wheel_wxyz, self.base_xyz, base_wxyz, self.hub_xyz, hub_wxyz):
            self.require(np.all(np.isfinite(pose)), "Invalid ground truth pose")
        self.outward = Rotation.from_quat(hub_wxyz, scalar_first=True).apply([0, 0, 1])
        # 원래 Arm FK와 고정 좌측 시험 씬의 제한을 명시적으로 유지한다.
        self.require(np.allclose(self.outward, [0, 1, 0], atol=1e-5) and
                     np.allclose(np.abs(base_wxyz), [1, 0, 0, 0], atol=1e-5),
                     "Only fixed left-side fixtures and unrotated robot base are supported")
        self.target = self.hub_xyz + self.outward * (NUT_SEAT + NUT_LENGTH + .08)
        self.flange_target = self.target + self.outward * TOOL_LENGTH
        self.arm = Arm(ROBOT_URDF)
        self.rotation = np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
        self.goal_angles = self.arm.solve(self.base_xyz, self.flange_target, side=1)
        self.arm_drives = [self.stage.GetPrimAtPath(f"{ROBOT_PATH}/Joints/joint_{i}")
                           .GetAttribute("drive:angular:physics:targetPosition") for i in range(1, 7)]
        self.require(all(bool(d) for d in self.arm_drives), "Missing arm drive")
        self.nuts = [SingleXFormPrim(path, reset_xform_properties=False) for path in self.nut_paths]
        self.nut_status = [dict(loaded=False, aligned=False, engaged_model=False,
                                seated=False, locked=False, released=False) for _ in SPINDLES]
        self.speeds = []
        for i, root in enumerate(SPINDLES):
            bearing = self.stage.GetPrimAtPath(root + "/Bearing")
            self.require(bool(bearing) and bearing.HasAPI(UsdPhysics.DriveAPI, "angular"),
                         f"Missing spindle drive {root}")
            self.speeds.append(UsdPhysics.DriveAPI.Get(bearing, "angular").CreateTargetVelocityAttr(0.))
            hold = UsdPhysics.FixedJoint.Get(self.stage, self.hold_paths[i])
            loaded = (bool(hold) and hold.GetJointEnabledAttr().Get() and
                      list(map(str, hold.GetBody0Rel().GetTargets())) == [root + "/Rotor"] and
                      list(map(str, hold.GetBody1Rel().GetTargets())) == [self.nut_paths[i]])
            self.nut_status[i]["loaded"] = bool(loaded)
        self.require(all(n["loaded"] for n in self.nut_status), "Initial five nuts are not retained")
        # 고정 좌측 씬의 스터드 끝점. 기울어진 실시간 허브에는 적용 불가.
        bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        tips = []
        for i in range(len(SPINDLES)):
            stud = self.stage.GetPrimAtPath(f"{self.hub_path}/Stud_{i}")
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
        self.verify_loaded()  # 장전 관계와 실제 너트 위치를 함께 검증

    def planar(self, vector):
        vector = np.asarray(vector)
        return vector - np.expand_dims(vector @ self.outward, -1) * self.outward

    def command_arm(self, radians):
        for drive, angle in zip(self.arm_drives, np.rad2deg(radians)):
            drive.Set(float(angle))

    def solve_pose(self, target, rotation, seed):
        # Arm.solve()은 고정 방향만 지원한다. 같은 FK/Jacobian을 이용해 목표 자세를 확장.
        q = np.clip(np.array(seed, dtype=float, copy=True), self.arm.lower, self.arm.upper)
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
        # 실제 이전 명령 자세에서 공통 경유 자세를 거쳐 다음 휠 준비 자세로 이동.
        self.move_via_upright(self.goal_angles)
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
        # 물리 체결 시 스터드 끝에서 5mm 떨어진 위치
        gap = .005 if self.physical_threads else 0.
        entry = self.tips + self.outward * gap

        self.move_nuts(entry, 360)

        # 물리 체결 모드에서 추가 정밀 정렬
        if self.physical_threads:
            LATERAL_TOL = .0001
            AXIAL_TOL = .0002
            ANGLE_TOL = .1

            for attempt in range(60):
                actual = self.nut_positions()
                error = entry - actual

                lateral = np.linalg.norm(
                    self.planar(error), axis=1
                )

                axial = np.abs(error @ self.outward)

                _, quat = self.tcp.get_world_pose()

                actual_rotation = Rotation.from_quat(
                    quat, scalar_first=True
                ).as_matrix()

                actual_axis = actual_rotation[:, 2]
                target_axis = -self.outward

                cross = np.cross(
                    actual_axis, target_axis
                )

                cross_norm = np.linalg.norm(cross)

                angle = np.arctan2(
                    cross_norm,
                    np.clip(
                        actual_axis @ target_axis,
                        -1., 1.
                    )
                )

                axis_error = np.degrees(angle)

                angles = self.pattern_angles(
                    actual, self.tips
                )

                phase = float(
                    np.angle(np.mean(np.exp(1j * angles)))
                )

                print(
                    f"[ALIGN {attempt+1:02d}] "
                    f"중심 오차={lateral.max()*1000:.3f} mm | "
                    f"축 방향={axial.max()*1000:.3f} mm | "
                    f"각도={axis_error:.3f} deg",
                    flush=True
                )

                if (
                    lateral.max() <= LATERAL_TOL
                    and axial.max() <= AXIAL_TOL
                    and axis_error <= ANGLE_TOL
                ):
                    break

                # 중심축 보정
                correction = np.eye(3)

                if cross_norm > 1e-9:
                    correction = Rotation.from_rotvec(
                        cross / cross_norm
                        * min(angle, .01)
                    ).as_matrix()

                # 볼트 패턴 보정
                pattern_correction = Rotation.from_rotvec(
                    self.outward * np.clip(phase, -.01, .01)
                ).as_matrix()

                desired_rotation = (
                    pattern_correction
                    @ correction
                    @ self.rotation
                )

                # 중심 위치 보정
                delta = error.mean(axis=0)

                delta *= min(
                    1.,
                    .001 / max(np.linalg.norm(delta), 1e-9)
                )

                self.correct_arm(
                    delta, desired_rotation
                )

                self.settle(24)

            else:
                raise RuntimeError(
                    "Physical thread fine alignment failed"
                )

        errors = np.linalg.norm(
            self.nut_positions() - entry,
            axis=1
        )

        self.metrics["entry_error_mm"] = (
            errors * 1000
        ).tolist()

        for nut, error in zip(self.nut_status, errors):
            nut["aligned"] = bool(error <= ENTRY_TOL)

        self.require(
            all(n["aligned"] for n in self.nut_status),
            "Stud entry alignment failed"
        )

    def engage(self):
        if self.physical_threads:
            return self.enable_thread_contact()
        # 기본 맵에는 활성 나사산 접촉이 없다. 진입 좌표만 검사한다.
        errors = np.linalg.norm(self.nut_positions()-self.tips,axis=1)
        self.metrics = {"mode":"geometric_entry_only", "physical_contact_verified":False,
                        "entry_error_mm":(errors*1000).tolist()}
        for n,e in zip(self.nut_status,errors):
            n["engaged_model"] = bool(n["loaded"] and e<=ENTRY_TOL)
        self.require(all(n["engaged_model"] for n in self.nut_status), "Geometric entry check failed")


    def enable_thread_contact(self):
        from pxr import Gf, PhysxSchema, Usd, UsdPhysics
        from isaacsim.core.prims import SingleXFormPrim

        self.thread_guides = []
        self.thread_springs = []

        start = self.nut_positions().copy()

        # =========================================
        # 1. 실제 너트 위치 기준으로 직동 관절 생성
        # =========================================
        for i, root in enumerate(SPINDLES):

            path = self.nut_paths[i]
            rotor_path = root + "/Rotor"

            rotor = SingleXFormPrim(
                rotor_path,
                reset_xform_properties=False
            )

            rotor_xyz, rotor_quat = rotor.get_world_pose()
            nut_xyz, nut_quat = self.nuts[i].get_world_pose()

            rotor_rotation = Rotation.from_quat(
                rotor_quat, scalar_first=True
            )

            nut_rotation = Rotation.from_quat(
                nut_quat, scalar_first=True
            )

            # Rotor 좌표계 기준 실제 너트 위치
            local_position = rotor_rotation.inv().apply(
                np.asarray(nut_xyz)
                - np.asarray(rotor_xyz)
            )

            # Rotor 좌표계 기준 실제 너트 자세
            local_rotation = (
                rotor_rotation.inv() * nut_rotation
            ).as_quat(scalar_first=True)

            hold = UsdPhysics.FixedJoint.Get(
                self.stage,
                self.hold_paths[i]
            )

            guide = UsdPhysics.PrismaticJoint.Define(
                self.stage,
                path + "/ThreadGuide"
            )

            guide.CreateJointEnabledAttr(False)

            guide.CreateBody0Rel().SetTargets(
                [rotor_path]
            )

            guide.CreateBody1Rel().SetTargets(
                [path]
            )

            guide.CreateLocalPos0Attr(
                Gf.Vec3f(*local_position.tolist())
            )

            guide.CreateLocalPos1Attr(
                Gf.Vec3f(0.0)
            )

            guide.CreateLocalRot0Attr(
                Gf.Quatf(
                    float(local_rotation[0]),
                    Gf.Vec3f(
                        *local_rotation[1:].tolist()
                    )
                )
            )

            guide.CreateLocalRot1Attr(
                Gf.Quatf(1.0)
            )

            # 로컬 Z축 방향으로만 전진 허용
            guide.CreateAxisAttr("Z")
            guide.CreateLowerLimitAttr(-.004)
            guide.CreateUpperLimitAttr(.032)

            guide.CreateCollisionEnabledAttr(False)
            guide.CreateExcludeFromArticulationAttr(True)

            # 위치 기반 강제 이동 비활성화
            spring = UsdPhysics.DriveAPI.Apply(
                guide.GetPrim(), "linear"
            )

            spring.CreateStiffnessAttr(0.0)
            spring.CreateDampingAttr(0.0)
            spring.CreateMaxForceAttr(0.0)

            # Fixed Joint -> Prismatic Joint
            hold.CreateJointEnabledAttr(False)
            guide.CreateJointEnabledAttr(True)

            self.thread_guides.append(guide)
            self.thread_springs.append(spring)

        # =========================================
        # 2. 관절 전환 자체의 안정성 확인
        # =========================================
        self.settle(12)

        current = self.nut_positions()

        drift = np.linalg.norm(
            current - start,
            axis=1
        )

        print(
            "[GUIDE CHECK] 관절 전환 후 변위(mm):",
            np.round(drift * 1000, 3),
            flush=True
        )

        self.require(
            np.all(drift < .0005),
            "Prismatic joint transition unstable"
        )

        # =========================================
        # 3. 나사산 Collider 활성화
        # =========================================
        def enable(prim, approximation):

            self.require(
                bool(prim),
                "Missing thread collider"
            )

            UsdPhysics.CollisionAPI.Apply(
                prim
            ).CreateCollisionEnabledAttr(True)

            UsdPhysics.MeshCollisionAPI.Apply(
                prim
            ).CreateApproximationAttr(approximation)

            config = PhysxSchema.PhysxCollisionAPI.Apply(
                prim
            )

            config.CreateContactOffsetAttr(.00005)
            config.CreateRestOffsetAttr(0.0)

        # 차량과 휠의 기존 충돌체를 임시 비활성화
        # 나사산 Collider는 이후 기존 코드에서 활성화
        disabled_colliders = []

        for prim in self.stage.Traverse():
            path = str(prim.GetPath())

            if not path.startswith((
                "/World/Car/",
                "/World/Wheel/",
                "/World/WheelRear/"
            )):
                continue

            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision = UsdPhysics.CollisionAPI(prim)

                if collision.GetCollisionEnabledAttr().Get():
                    collision.CreateCollisionEnabledAttr(False)
                    disabled_colliders.append(path)

        print(
            f"[COLLISION FILTER] "
            f"기존 충돌체 {len(disabled_colliders)}개 비활성화",
            flush=True
        )

        for path in disabled_colliders:
            print(f"  OFF: {path}", flush=True)


        from pxr import UsdGeom

        stud_parts = []
        nut_parts = []

        for i, path in enumerate(self.nut_paths):

            for name in ("Stud", "Thread"):
                prim = self.stage.GetPrimAtPath(
                    f"{self.hub_path}/{name}_{i}"
                )
                stud_parts.append(prim)

            thread = self.stage.GetPrimAtPath(
                path + "/Visual/InternalThread"
            )

            for part in Usd.PrimRange(thread):
                if part.HasAPI(UsdPhysics.CollisionAPI):
                    nut_parts.append(part)

        def disable(parts):
            for prim in parts:
                UsdPhysics.CollisionAPI.Apply(
                    prim
                ).CreateCollisionEnabledAttr(False)

        # 실제 Mesh의 체결축 방향 범위 측정
        def axial_bounds(parts):
            cache = UsdGeom.XformCache()
            values = []

            for prim in parts:
                mesh = UsdGeom.Mesh(prim)

                if not mesh:
                    continue

                matrix = cache.GetLocalToWorldTransform(prim)

                for point in mesh.GetPointsAttr().Get():
                    xyz = np.array(
                        matrix.Transform(Gf.Vec3d(point))
                    )

                    values.append(xyz @ self.outward)

            return min(values), max(values)

        stud_min, stud_max = axial_bounds(stud_parts)
        nut_min, nut_max = axial_bounds(nut_parts)

        print(
            f"[GEOMETRY CHECK] 실제 축 방향 간격: "
            f"{(nut_min - stud_max)*1000:.3f} mm",
            flush=True
        )

        def check_motion(label):
            before = self.nut_positions().copy()

            self.settle(12)

            delta = self.nut_positions() - before

            axial = delta @ self.outward

            lateral = np.linalg.norm(
                self.planar(delta), axis=1
            )

            drift = np.linalg.norm(delta, axis=1)

            print(
                f"\n[{label}]",
                flush=True
            )

            print(
                "축 방향 변위(mm):",
                np.round(axial * 1000, 3),
                flush=True
            )

            print(
                "측면 변위(mm):",
                np.round(lateral * 1000, 3),
                flush=True
            )

            self.require(
                np.all(drift < .0005),
                f"{label}: unstable"
            )

        # TEST A: 나사산 충돌체 모두 OFF
        disable(stud_parts)
        disable(nut_parts)

        check_motion("TEST A - NO THREAD COLLISION")

        # TEST B: 스터드만 ON
        for prim in stud_parts:
            enable(prim, "none")

        check_motion("TEST B - STUD ONLY")

        # TEST C: 너트만 ON
        disable(stud_parts)

        for prim in nut_parts:
            enable(prim, "convexHull")

        check_motion("TEST C - NUT ONLY")

        # TEST D: 스터드 + 너트 ON
        for prim in stud_parts:
            enable(prim, "none")

        check_motion("TEST D - BOTH ENABLED")



        self.metrics = {
            "mode": "physical_threads",
            "guide_type": "prismatic",
            "axial_compliance_force_n": 2.0
        }


    def spin_physical(self):
        """힘 기반 최초 물림 + 고속 런다운 + 저속 안착."""

        from pxr import Gf, PhysxSchema, UsdPhysics

        # =============================================
        # 1. 체결 파라미터
        # =============================================
        PUSH_FORCE = 2.0          # N / nut
        ENGAGE_TORQUE = 0.02      # N·m
        RUNDOWN_TORQUE = 0.08     # N·m
        SEATING_TORQUE = 0.02     # N·m

        ENGAGE_RPM = 60
        RUNDOWN_RPM = 240
        SEATING_RPM = 30

        MAX_LINEAR_SPEED = 0.010  # m/s
        ENGAGE_DEPTH = 0.007      # 스터드 끝 기준 7 mm
        SLOW_DISTANCE = 0.003     # 안착 3 mm 전 감속
        STOP_DISTANCE = 0.0003    # 안착 0.3 mm 이내 정지

        PITCH_TOL = 0.0005
        ENGAGE_TIMEOUT = 12.0
        TOTAL_TIMEOUT = 40.0

        PUSH = 0
        VERIFY = 1
        RUNDOWN = 2
        SEATED = 3

        count = len(SPINDLES)
        zero = Gf.Vec3f(0.0)

        # =============================================
        # 2. 초기 상태 및 측정 변수
        # =============================================
        start = self.nut_positions()
        tcp_start = np.array(self.tcp.get_world_pose()[0])

        last_phase = self.phases()
        turns = np.zeros(count)

        states = np.full(count, PUSH)
        slowed = np.zeros(count, dtype=bool)

        free_start = start.copy()
        free_turn_base = np.zeros(count)
        peak_rpm = np.zeros(count)

        forces = []
        motors = []
        angular_limits = []
        saved_attrs = []

        push_vector = Gf.Vec3f(
            *(-self.outward * PUSH_FORCE).tolist()
        )
        reaction_vector = Gf.Vec3f(
            *(self.outward * PUSH_FORCE).tolist()
        )

        def save_attr(attr, value):
            saved_attrs.append((attr, attr.Get()))
            attr.Set(value)

        def force_attr(prim):
            api = PhysxSchema.PhysxForceAPI.Apply(prim)
            api.CreateModeAttr("force")
            api.CreateWorldFrameEnabledAttr(True)
            return api.CreateForceAttr(zero)

        def set_push(i, enabled):
            nut_force, rotor_force = forces[i]

            nut_force.Set(
                push_vector if enabled else zero
            )
            rotor_force.Set(
                reaction_vector if enabled else zero
            )

        def set_motor(i, rpm, torque):
            motors[i].GetMaxForceAttr().Set(
                float(torque)
            )

            # USD 각속도 단위: deg/s
            speed_deg = float(rpm * 6)

            for attr in angular_limits[i]:
                attr.Set(float(max(30, rpm) * 6))

            self.speeds[i].Set(speed_deg)

        def update_measurement():
            nonlocal last_phase

            phase = self.phases()
            delta = np.angle(
                np.exp(1j * (phase - last_phase))
            ) / tau

            turns[:] += delta
            last_phase = phase

            actual_rpm = np.abs(delta) * 60 / self.dt
            peak_rpm[:] = np.maximum(
                peak_rpm, actual_rpm
            )

            return self.nut_positions()

        # =============================================
        # 3. 실제 물리 제어 준비
        # =============================================
        try:
            required_travel = (
                start - self.seats
            ) @ self.outward

            self.require(
                np.all(required_travel > 0)
                and np.all(required_travel < 0.031),
                "Insufficient axial floating stroke"
            )

            # 기존 축 방향 위치 스프링 비활성화
            for spring in self.thread_springs:
                spring.GetStiffnessAttr().Set(0.0)
                spring.GetDampingAttr().Set(0.0)
                spring.GetMaxForceAttr().Set(0.0)

            for i, root in enumerate(SPINDLES):
                nut = self.stage.GetPrimAtPath(
                    self.nut_paths[i]
                )
                rotor = self.stage.GetPrimAtPath(
                    root + "/Rotor"
                )
                bearing = self.stage.GetPrimAtPath(
                    root + "/Bearing"
                )

                # 너트 추력과 소켓 측 반력
                forces.append((
                    force_attr(nut),
                    force_attr(rotor)
                ))

                nut_phys = (
                    PhysxSchema.PhysxRigidBodyAPI.Apply(nut)
                )
                rotor_phys = (
                    PhysxSchema.PhysxRigidBodyAPI.Apply(rotor)
                )

                save_attr(
                    nut_phys.CreateMaxLinearVelocityAttr(),
                    MAX_LINEAR_SPEED
                )

                caps = [
                    nut_phys.CreateMaxAngularVelocityAttr(),
                    rotor_phys.CreateMaxAngularVelocityAttr()
                ]

                for attr in caps:
                    save_attr(attr, ENGAGE_RPM * 6.0)

                angular_limits.append(caps)

                # 실제 스핀들의 토크 제한
                motor = UsdPhysics.DriveAPI.Get(
                    bearing, "angular"
                )

                save_attr(
                    motor.GetMaxForceAttr(),
                    ENGAGE_TORQUE
                )

                motors.append(motor)

                set_push(i, True)
                set_motor(
                    i, ENGAGE_RPM, ENGAGE_TORQUE
                )

            print(
                "[THREAD] 최초 물림 시작 | "
                f"{PUSH_FORCE} N | "
                f"{ENGAGE_TORQUE} N·m | "
                f"{ENGAGE_RPM} RPM",
                flush=True
            )

            # =========================================
            # 4. 너트별 독립 물리 제어
            # =========================================
            for frame in range(
                int(TOTAL_TIMEOUT / self.dt)
            ):
                self.step(frame)

                positions = update_measurement()
                elapsed = (frame + 1) * self.dt

                depth = (
                    self.tips - positions
                ) @ self.outward

                remaining = (
                    positions - self.seats
                ) @ self.outward

                backout = (
                    positions - start
                ) @ self.outward

                tcp_now = np.array(
                    self.tcp.get_world_pose()[0]
                )

                self.require(
                    np.linalg.norm(tcp_now - tcp_start)
                    < 0.001,
                    "Robot TCP moved during thread contact"
                )


                if (
                    frame % 60 == 0
                    or np.any(backout >= 0.004)
                ):
                    lateral_error = np.linalg.norm(
                        self.planar(positions - self.tips),
                        axis=1
                    )

                    print(
                        f"\n[THREAD DEBUG] "
                        f"시간={elapsed:.3f} sec",
                        flush=True
                    )

                    for i in range(count):
                        print(
                            f"NUT {i+1} | "
                            f"후퇴={backout[i]*1000:.3f} mm | "
                            f"진입={depth[i]*1000:.3f} mm | "
                            f"중심오차={lateral_error[i]*1000:.3f} mm | "
                            f"회전={turns[i]:.3f} rev | "
                            f"최대RPM={peak_rpm[i]:.1f}",
                            flush=True
                        )

                if np.any(backout >= 0.004):
                    failed_ids = (
                        np.where(backout >= 0.004)[0] + 1
                    ).tolist()

                    raise RuntimeError(
                        f"Nut backed out: {failed_ids}"
                    )


                self.require(
                    np.all(remaining > -SEAT_TOL),
                    "Nut passed seating position"
                )

                for i in range(count):

                    # ---------------------------------
                    # A. 최초 나사 물림
                    # ---------------------------------
                    if states[i] == PUSH:

                        if depth[i] >= ENGAGE_DEPTH:
                            set_push(i, False)

                            free_start[i] = positions[i]
                            free_turn_base[i] = turns[i]

                            states[i] = VERIFY

                            print(
                                f"[NUT {i+1}] "
                                "진입 깊이 확보, 축힘 제거",
                                flush=True
                            )

                        elif elapsed >= ENGAGE_TIMEOUT:
                            raise RuntimeError(
                                f"Nut {i+1}: engagement timeout"
                            )

                    # ---------------------------------
                    # B. 자력 전진 및 피치 검증
                    # ---------------------------------
                    elif states[i] == VERIFY:

                        free_turns = (
                            turns[i] - free_turn_base[i]
                        )

                        advance = float(
                            (free_start[i] - positions[i])
                            @ self.outward
                        )

                        error = abs(
                            advance - free_turns * PITCH
                        )

                        if free_turns >= 1.0:

                            self.require(
                                advance > 0.001
                                and error < PITCH_TOL,
                                f"Nut {i+1}: pitch mismatch"
                            )

                            self.nut_status[i][
                                "engaged_model"
                            ] = True

                            states[i] = RUNDOWN

                            set_motor(
                                i,
                                RUNDOWN_RPM,
                                RUNDOWN_TORQUE
                            )

                            print(
                                f"[NUT {i+1}] "
                                "물리 물림 확인 | "
                                f"피치 오차 {error*1000:.3f} mm "
                                f"| {RUNDOWN_RPM} RPM",
                                flush=True
                            )

                    # ---------------------------------
                    # C. 고속 런다운 및 안착
                    # ---------------------------------
                    elif states[i] == RUNDOWN:

                        if (
                            remaining[i] <= SLOW_DISTANCE
                            and not slowed[i]
                        ):
                            set_motor(
                                i,
                                SEATING_RPM,
                                SEATING_TORQUE
                            )

                            slowed[i] = True

                            print(
                                f"[NUT {i+1}] "
                                "안착 직전 감속",
                                flush=True
                            )

                        if remaining[i] <= STOP_DISTANCE:

                            seat_error = np.linalg.norm(
                                positions[i] - self.seats[i]
                            )

                            self.require(
                                seat_error < SEAT_TOL,
                                f"Nut {i+1}: seating error"
                            )

                            set_motor(
                                i, 0, SEATING_TORQUE
                            )

                            states[i] = SEATED

                            print(
                                f"[NUT {i+1}] 안착 위치 도달 | "
                                f"오차 {seat_error*1000:.3f} mm",
                                flush=True
                            )

                if np.all(states == SEATED):
                    break

            # =========================================
            # 5. 최종 물리 검증
            # =========================================
            self.require(
                np.all(states == SEATED),
                "Physical fastening timeout"
            )

            # 스핀들 정지 후 0.5초 안정화
            for frame in range(int(0.5 / self.dt)):
                self.step(frame)
                positions = update_measurement()

            end = self.nut_positions()

            free_turns = turns - free_turn_base
            advance = (
                free_start - end
            ) @ self.outward

            pitch_error = np.abs(
                advance - free_turns * PITCH
            )

            seat_errors = np.linalg.norm(
                end - self.seats, axis=1
            )

            self.metrics = {
                "mode": "physical_threads",
                "free_turns": free_turns.tolist(),
                "free_advance_mm":
                    (advance * 1000).tolist(),
                "pitch_error_mm":
                    (pitch_error * 1000).tolist(),
                "physical_seat_error_mm":
                    (seat_errors * 1000).tolist(),
                "peak_rpm": peak_rpm.tolist(),
                "elapsed_seconds": float(elapsed),
                "physical_contact_verified": bool(
                    np.all(free_turns >= 1)
                    and np.all(pitch_error < PITCH_TOL)
                ),
                "torque_verified": False
            }

            print(
                "\n[PHYSICAL THREAD RESULT]",
                flush=True
            )

            for i in range(count):
                print(
                    f"NUT {i+1} | "
                    f"회전 {free_turns[i]:.2f} rev | "
                    f"전진 {advance[i]*1000:.2f} mm | "
                    f"피치오차 {pitch_error[i]*1000:.3f} mm | "
                    f"안착오차 {seat_errors[i]*1000:.3f} mm | "
                    f"최대속도 {peak_rpm[i]:.1f} RPM",
                    flush=True
                )

            self.require(
                self.metrics["physical_contact_verified"],
                "Physical pitch verification failed"
            )

            self.require(
                np.all(seat_errors < SEAT_TOL),
                "Physical seating verification failed"
            )

            return free_start, free_turns

        finally:
            # 실패하거나 중단되어도 추력과 회전 명령 제거
            for nut_force, rotor_force in forces:
                nut_force.Set(zero)
                rotor_force.Set(zero)

            self.stop_spindles()

            # 다음 재장전 및 일반 로봇 이동을 위해 복원
            for attr, old_value in reversed(saved_attrs):
                if old_value is None:
                    attr.Clear()
                else:
                    attr.Set(old_value)


    
    def phases(self):
        rotations = [Rotation.from_quat(nut.get_world_pose()[1], scalar_first=True) for nut in self.nuts]
        axes = np.array([r.apply([1,0,0]) for r in rotations])
        return np.arctan2(axes[:,2],axes[:,0])

    def stop_spindles(self):
        for speed in self.speeds:
            speed.Set(0.)

    def spin(self):
        if self.physical_threads:
            return self.spin_physical()
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
        if self.physical_threads:
            end = self.nut_positions()
            errors = np.linalg.norm(end-self.seats, axis=1)
            self.metrics = {"physical_seat_error_mm": (errors*1000).tolist()}
            for nut, error in zip(self.nut_status, errors):
                nut["seated"] = bool(error < SEAT_TOL)
            self.require(np.all(errors < SEAT_TOL), "Physical nuts did not seat")
            return end
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
        # 고정 차량 시험에 한해 World 고정. 차량 이동을 위한 Hub 결합은 미구현.
        # LOCK과 RELEASE 사이에는 physics step을 진행하지 않는다.
        for i,(root,nut) in enumerate(zip(SPINDLES,self.nuts)):
            xyz,quat = nut.get_world_pose()
            lock = UsdPhysics.FixedJoint.Define(self.stage, f"/World/FastenedJoints/{self.target_wheel}/Nut_{i}")
            lock.CreateBody1Rel().SetTargets([self.nut_paths[i]])
            lock.CreateLocalPos0Attr(Gf.Vec3f(*xyz.tolist()))
            lock.CreateLocalRot0Attr(Gf.Quatf(float(quat[0]),Gf.Vec3f(*quat[1:].tolist())))
            lock.CreateExcludeFromArticulationAttr(True)
            lock.CreateJointEnabledAttr(True)
            self.nut_status[i]["locked"] = bool(lock.GetJointEnabledAttr().Get() and
                not lock.GetBody0Rel().GetTargets() and
                list(map(str,lock.GetBody1Rel().GetTargets())) == [self.nut_paths[i]])
        self.metrics = {"mode":"world_fixed_joint", "hub_attachment":False}
        self.require(all(n["locked"] for n in self.nut_status), "SeatedJoint creation failed")

    def release(self):
        from pxr import UsdPhysics
        if self.physical_threads:
            for guide in self.thread_guides:
                guide.CreateJointEnabledAttr(False)
        for i,root in enumerate(SPINDLES):
            hold = UsdPhysics.FixedJoint.Get(self.stage,self.hold_paths[i])
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
        self.angles = retract.copy()  # 다음 장전/체결 이동의 시작점

    # ── STEP 13: 고정식 박스에서 신규 너트 장전 (공급 동작 모사) ─────────
    def move_joints(self, goal, seconds=3.):
        """현재 명령 관절각에서 목표까지 smoothstep, 충분한 안정화 포함."""
        start = self.angles.copy()
        goal = np.asarray(goal, dtype=float)
        # smoothstep 최대 속도 = 1.5 * 이동각 / 시간. URDF 속도 한계 여유 적용.
        seconds = max(seconds, float(np.max(1.5*np.abs(goal-start)/(.7*self.arm.speed))))
        frames = max(1, int(np.ceil(seconds/self.dt)))
        for frame in range(frames):
            t = (frame+1)/frames
            self.command_arm(start+(goal-start)*(t*t*(3-2*t)))
            self.step(frame)
        self.angles = goal.copy()
        self.settle(120)

    def move_via_upright(self, goal):
        """기존 수직 준비 자세를 중간 경유점으로 재사용. 충돌 계획은 별도 검증 필요."""
        upright = np.zeros(6)
        upright[0] = self.angles[0]
        if np.linalg.norm(self.angles-upright) > 1e-8:
            self.move_joints(upright)
        upright[0] = goal[0]
        if np.linalg.norm(self.angles-upright) > 1e-8:
            self.move_joints(upright)
        self.move_joints(goal)

    def reload_log(self, number, text):
        print(f"[{self.target_wheel}][STEP 13-{number}] {text}", flush=True)

    def reload_pose(self, tcp_xyz, rotation, seed):
        """장전 TCP 위치 → 플랜지 위치. 공구 +Z는 박스 상면 방향."""
        flange = np.asarray(tcp_xyz)-rotation[:,2]*TOOL_LENGTH
        return self.solve_pose(flange, rotation, seed)

    def plan_reload(self, dock, rotation):
        # 회전각 자유인 원형 포트: 4개 yaw 후보와 seed에서 연속 도달 자세 탐색.
        above = dock+np.array([0.,0.,RELOAD_APPROACH])
        choices = []
        for yaw in (0.,90.,180.,270.):
            desired = Rotation.from_euler("z",yaw,degrees=True).as_matrix() @ rotation
            for seed in (np.zeros(6), self.angles, *np.random.default_rng(41).uniform(-2.4,2.4,(6,6))):
                try:
                    pre = self.reload_pose(above,desired,seed)
                    at = self.reload_pose(dock,desired,pre)
                    back = self.reload_pose(above,desired,at)
                    score = np.linalg.norm(pre) + 2*np.linalg.norm(at-pre) + np.linalg.norm(back-pre)
                    choices.append((score,desired,pre,at,back))
                except RuntimeError:
                    continue
        self.require(bool(choices), "Reload box IK unreachable")
        return min(choices,key=lambda item:item[0])[1:]

    def check_reload_pose(self, goal, rotation):
        xyz,quat = self.tcp.get_world_pose()
        error = np.asarray(xyz)-goal
        actual = Rotation.from_quat(quat,scalar_first=True).as_matrix()
        axis = float(np.degrees(np.arccos(np.clip(actual[:,2]@rotation[:,2],-1,1))))
        lateral = float(np.linalg.norm(error[:2]))
        height = float(abs(error[2]))
        self.metrics.update(reload_xy_error_mm=lateral*1000, reload_z_error_mm=height*1000,
                            reload_axis_error_deg=axis)
        return lateral<=RELOAD_POS_TOL and height<=RELOAD_POS_TOL and axis<=RELOAD_AXIS_TOL

    def spawn_nuts(self):
        """실제 Rotor 월드 자세에서 신규 강체 생성. 기존 체결 너트는 변경하지 않는다."""
        from isaacsim.core.prims import SingleXFormPrim
        from pxr import Gf, UsdGeom, UsdPhysics
        from sim_util import body, constrain, mass_properties
        self.batch_id += 1
        batch = f"/World/ReloadedNuts/Batch_{self.batch_id:03d}"
        UsdGeom.Xform.Define(self.stage,batch)
        paths, holds = [], []
        for i,root in enumerate(SPINDLES):
            rotor = self.stage.GetPrimAtPath(root+"/Rotor")
            xyz,quat = SingleXFormPrim(root+"/Rotor",reset_xform_properties=False).get_world_pose()
            rot = Rotation.from_quat(quat,scalar_first=True)
            origin = np.asarray(xyz)+rot.apply([0,0,NUT_ROTOR_OFFSET])
            q = Gf.Quatd(float(quat[0]),Gf.Vec3d(*map(float,quat[1:])))
            path = batch+f"/Nut_{i}"
            self.require(not self.stage.GetPrimAtPath(path), f"Duplicate nut: {path}")
            nut = body(self.stage,path,origin,q,.045)
            mass_properties(nut,(0,0,-.012),(3e-6,3e-6,3e-6))
            visual = self.stage.DefinePrim(path+"/Visual","Xform")
            visual.GetReferences().AddInternalReference(root+"/Nut/Visual")
            joint_path = batch+f"/Retention_{i}"
            hold = constrain(UsdPhysics.FixedJoint.Define(self.stage,joint_path),rotor,nut,(0,0,NUT_ROTOR_OFFSET))
            hold.CreateExcludeFromArticulationAttr(True)
            hold.CreateJointEnabledAttr(True)
            paths.append(path); holds.append(joint_path)
        self.nut_paths, self.hold_paths = paths, holds
        self.nuts = [SingleXFormPrim(path,reset_xform_properties=False) for path in paths]
        self.nut_status = [dict(loaded=False,aligned=False,engaged_model=False,seated=False,locked=False,released=False) for _ in paths]
        self.settle(60)
        self.verify_loaded()

    def verify_loaded(self):
        from pxr import UsdPhysics
        from isaacsim.core.prims import SingleXFormPrim
        errors = []
        for i,root in enumerate(SPINDLES):
            hold = UsdPhysics.FixedJoint.Get(self.stage,self.hold_paths[i])
            xyz,q = SingleXFormPrim(root+"/Rotor",reset_xform_properties=False).get_world_pose()
            expected = np.asarray(xyz)+Rotation.from_quat(q,scalar_first=True).apply([0,0,NUT_ROTOR_OFFSET])
            error = float(np.linalg.norm(self.nuts[i].get_world_pose()[0]-expected))
            errors.append(error*1000)
            self.nut_status[i]["loaded"] = bool(hold and hold.GetJointEnabledAttr().Get()
                and list(map(str,hold.GetBody0Rel().GetTargets()))==[root+"/Rotor"]
                and list(map(str,hold.GetBody1Rel().GetTargets()))==[self.nut_paths[i]]
                and error<=ENTRY_TOL)
        self.metrics.update(loaded_count=sum(n["loaded"] for n in self.nut_status),load_errors_mm=errors)
        self.require(all(n["loaded"] for n in self.nut_status), "Reload retention/position verification failed")

    def verify_finished_nuts(self):
        from isaacsim.core.prims import SingleXFormPrim
        for wheel_id,(paths,seated) in self.finished_nuts.items():
            positions = np.array([SingleXFormPrim(p,reset_xform_properties=False).get_world_pose()[0] for p in paths])
            drift = float(np.max(np.linalg.norm(positions-seated,axis=1)))
            self.require(drift<SEAT_TOL,f"Previously seated {wheel_id} nuts moved: {drift*1000:.3f} mm")

    def reload(self):
        from isaacsim.core.prims import SingleXFormPrim
        from pxr import UsdPhysics
        self.stop_spindles()
        self.require(all(not n["loaded"] for n in self.nut_status), "Reload requires five empty sockets")
        self.require(all(not UsdPhysics.FixedJoint.Get(self.stage,p).GetJointEnabledAttr().Get()
                         for p in self.hold_paths), "Old retention still enabled")
        self.metrics = {"mode":"박스 장전 모사 (공급기 미모델링)","physical_feed_verified":False}
        port = SingleXFormPrim(RELOAD_PORT,reset_xform_properties=False)
        dock,quat = port.get_world_pose()
        dock = np.asarray(dock,dtype=float)
        rotation = Rotation.from_quat(quat,scalar_first=True).as_matrix()
        desired,pre,_,_ = self.plan_reload(dock,rotation)
        self.reload_log(1,"박스 상단 접근")
        self.move_via_upright(pre)
        above = dock+np.array([0.,0.,RELOAD_APPROACH])
        self.require(self.check_reload_pose(above,desired), "Reload approach pose error")
        self.reload_log(2,"원형 장전 구역 중심·축 정렬 및 하강")
        # Cartesian 직선 경유점 IK. 손목 branch는 이전 관절각에서 이어서 계산.
        for fraction in np.linspace(0.,1.,9)[1:]:
            q = self.reload_pose(above+(dock-above)*fraction,desired,self.angles)
            self.move_joints(q,.25)
        self.reload_log(3,"장전 위치 및 자세 유지 확인")
        for frame in range(max(1,int(RELOAD_HOLD_SECONDS/self.dt))):
            self.require(self.check_reload_pose(dock,desired), "Reload docking position/axis out of tolerance")
            self.step(frame)
        self.metrics["dock_errors"] = {k:self.metrics[k] for k in ("reload_xy_error_mm","reload_z_error_mm","reload_axis_error_deg")}
        self.reload_log(4,"신규 너트 5개 생성 및 Fixed Joint 장착")
        self.spawn_nuts()
        self.reload_log(5,"장전 상태 확인 및 공구 후퇴")
        for fraction in np.linspace(0.,1.,9)[1:]:
            q = self.reload_pose(dock+(above-dock)*fraction,desired,self.angles)
            self.move_joints(q,.25)
        self.require(self.check_reload_pose(above,desired), "Reload withdrawal pose error")
        self.verify_loaded()
        self.verify_finished_nuts()
        self.reload_required = False
        self.metrics["batch_id"] = self.batch_id
        self.rotation = desired

    def capture(self, name, eye, look_at):
        if self.captures is None:
            return
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        self.captures.mkdir(parents=True, exist_ok=True)
        set_camera_view(eye=list(eye), target=list(look_at))
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
class ThreadContactView:
    """접촉 시험의 카메라와 표시만 관리하고, 물리 제어는 시험 루프에 맡긴다."""

    def __init__(self, app, sim, ctx, stage, hub_frame, bodies, camera_state):
        import carb.settings
        import omni.timeline
        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport
        from omni.physx.bindings._physx import SETTING_DISPLAY_COLLIDERS
        from pxr import Gf, UsdGeom, UsdPhysics

        self.app, self.sim, self.ctx, self.stage = app, sim, ctx, stage
        self.timeline = omni.timeline.get_timeline_interface()
        self.viewport = get_active_viewport()
        self.settings = carb.settings.get_settings()
        self.collider_setting = SETTING_DISPLAY_COLLIDERS
        self.original_display = self.settings.get_as_int(SETTING_DISPLAY_COLLIDERS)
        self.zoomed = False
        self.show_colliders = False
        self.finished = False
        self.restart_requested = False
        self.camera_state = camera_state
        # 확대 시 외피만 숨겨 실제 PhysX 콜라이더를 드러낸다.
        self.shells = [
            UsdGeom.Imageable(prim) for prim in bodies[0].GetChild("Geometry").GetChildren()
            if prim.IsA(UsdGeom.Gprim) and not prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        hub_center = hub_frame.Transform(Gf.Vec3d(0, 0, OFFSET + .016))
        nut_center = UsdGeom.XformCache().GetLocalToWorldTransform(bodies[0]).ExtractTranslation()
        nut_center += hub_frame.TransformDir(Gf.Vec3d(0, 0, -.003))
        for name, target, offset, focal in (
            ("Hub", hub_center, (.12, .13, .25), 27.),
            ("Thread", nut_center, (.044, .020, .010), 36.),
        ):
            camera = UsdGeom.Camera.Define(stage, f"/World/ContactViews/{name}")
            eye = target + hub_frame.TransformDir(Gf.Vec3d(*offset))
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
                eye, target, Gf.Vec3d(0, 0, 1)
            ).GetInverse())
            camera.CreateFocalLengthAttr(focal)
            camera.CreateClippingRangeAttr(Gf.Vec2f(.001, 10.))

        self.window = ui.Window("Thread Contact | FL", width=440, height=105)
        with self.window.frame, ui.VStack(spacing=4):
            self.status = ui.Label("나사면 접촉 시험 · 너트 5개")
            with ui.HStack(spacing=4):
                self.pause_button = ui.Button("Pause", clicked_fn=self.toggle_pause)
                self.restart_button = ui.Button("Restart", enabled=False, clicked_fn=self.request_restart)
                ui.Button("Hub view", clicked_fn=lambda: self.set_view(False))
                ui.Button("Thread zoom", clicked_fn=lambda: self.set_view(True))
                self.collider_button = ui.Button("Colliders: Off", clicked_fn=self.toggle_colliders)
        if self.camera_state:
            for path, viewpoint in self.camera_state["cameras"].items():
                UsdGeom.Camera.Define(stage, path).SetFromCamera(viewpoint)
            self.viewport.camera_path = self.camera_state["active"]
            self.zoomed = self.camera_state["zoomed"]
            self.show_colliders = self.camera_state["colliders"]
            self.update_colliders()
        else:
            self.set_view(False)

    def remember_camera(self):
        from pxr import UsdGeom

        active = self.viewport.camera_path
        paths = ("/World/ContactViews/Hub", "/World/ContactViews/Thread", active)
        cameras = {}
        for path in paths:
            camera = UsdGeom.Camera.Get(self.stage, path)
            if camera:
                cameras[path] = camera.GetCamera()
        self.camera_state.update(active=active, cameras=cameras,
                                 zoomed=self.zoomed, colliders=self.show_colliders)

    def set_view(self, zoomed):
        from pxr import Sdf

        self.zoomed = zoomed
        self.viewport.camera_path = Sdf.Path(f"/World/ContactViews/{'Thread' if zoomed else 'Hub'}")
        self.show_colliders = zoomed
        self.update_colliders()

    def toggle_colliders(self):
        self.show_colliders = not self.show_colliders
        self.update_colliders()

    def update_colliders(self):
        self.settings.set_int(self.collider_setting, 2 if self.show_colliders else 0)
        self.collider_button.text = "Colliders: On" if self.show_colliders else "Colliders: Off"
        for shell in self.shells:
            if self.zoomed and self.show_colliders:
                shell.MakeInvisible()
            else:
                shell.MakeVisible()

    def toggle_pause(self):
        if self.finished:
            return
        if self.timeline.is_playing():
            self.timeline.pause()
        else:
            self.timeline.play()
        self.timeline.commit()

    def is_open(self):
        return self.app.is_running() and self.ctx.get_stage() == self.stage

    def step(self, frame):
        # 480 Hz 물리 스텝은 그대로 실행하고 8스텝마다 화면만 갱신한다.
        if frame % 8 == 0:
            self.sim.render()
        while self.is_open() and not self.timeline.is_playing() and not self.timeline.is_stopped():
            self.pause_button.text = "Resume"
            self.sim.render()
        if not self.is_open() or self.timeline.is_stopped():
            raise CycleInterrupted
        self.pause_button.text = "Pause"
        self.sim.step(render=False)

    def hold_result(self, passed):
        self.finished = True
        self.timeline.pause()
        self.timeline.commit()
        self.pause_button.enabled = False
        self.restart_button.enabled = True
        self.status.text = f"{'PASS' if passed else 'FAIL'} · 최종 자세"
        while self.is_open() and not self.restart_requested:
            self.sim.render()

    def request_restart(self):
        self.restart_requested = True

    def close(self):
        self.settings.set_int(self.collider_setting, self.original_display)
        self.window.destroy()


def check_thread_contact(app, gui=False, approach=False, push_force_n=2., spin_torque_nm=.02):
    camera_state = {}
    while True:
        result = run_thread_contact(app, gui, approach, push_force_n, spin_torque_nm, camera_state)
        if result is not None:
            return result


def run_thread_contact(app, gui, approach, push_force_n, spin_torque_nm, camera_state):
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
    loads = []
    approach_offset = .012 if approach else 0.
    for i, root in enumerate(SPINDLES):
        stud = stage.GetPrimAtPath(f"{hub.GetPath()}/Stud_{i}")
        thread = stage.GetPrimAtPath(f"{hub.GetPath()}/Thread_{i}")
        for part in (stud, thread):
            collider(part,"none")
        pts = np.array(UsdGeom.Mesh(stud).GetPointsAttr().Get())
        center = (pts.min(axis=0)+pts.max(axis=0))/2
        # 암나사 돌기가 수나사 골 사이에 오도록 미리 맞물린 위치를 정한다.
        center[2] = OFFSET+.00125+14.5*PITCH + approach_offset
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
        if approach:
            # 원뿔 시트와 만나는 단면을 유지하며 육각 외형의 축 방향 높이만 반으로 줄인다.
            hex_mesh = UsdGeom.Mesh.Get(stage, path + "/Geometry/Hex")
            points = hex_mesh.GetPointsAttr()
            vertices = points.Get()
            base_z = min(point[2] for point in vertices)
            points.Set([Gf.Vec3f(x, y, base_z + .5 * (z - base_z)) for x, y, z in vertices])
            hex_mesh.GetExtentAttr().Set(UsdGeom.PointBased.ComputeExtent(points.Get()))
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
        if approach:
            # 월드 기준 스터드 축을 따라 힘과 토크를 직접 가한다. 속도 상한만 설정한다.
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rigid.CreateMaxLinearVelocityAttr(.010)
            rigid.CreateMaxAngularVelocityAttr(360.)
            load = PhysxSchema.PhysxForceAPI.Apply(prim)
            load.CreateModeAttr("force")
            load.CreateWorldFrameEnabledAttr(True)
            loads.append((load.CreateForceAttr(Gf.Vec3f(0)), load.CreateTorqueAttr(Gf.Vec3f(0))))
        else:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(),"rotZ")
            drive.CreateDampingAttr(.001)
            drive.CreateMaxForceAttr(.12)
            hinges.append(drive.CreateTargetVelocityAttr(0.))
        bodies.append(prim)

    stage.GetPrimAtPath("/World/Head").SetActive(False)
    sim = SimulationContext(physics_dt=1/480, stage_units_in_meters=1,physics_prim_path="/World/Physics")
    if gui:
        import omni.timeline

        # GUI 타임라인이 물리 시험의 제한 시간보다 먼저 끝나지 않도록 한다.
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_end_time(60.)
        timeline.set_current_time(0.)
        timeline.commit()
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

    view = ThreadContactView(app, sim, ctx, stage, hub_frame, bodies, camera_state) if gui else None

    def step(frame):
        if view is None:
            sim.step(render=False)
        else:
            view.step(frame)

    try:
        for frame in range(120):
            step(frame)
        start,last = measure()
        for speed in hinges:
            speed.Set(-360.)
        if approach:
            push = Gf.Vec3f(*(-axis * push_force_n).tolist())
            torque = Gf.Vec3f(*(-axis * spin_torque_nm).tolist())
            print(f"LOAD per nut: push_n={push_force_n:g} spin_nm={spin_torque_nm:g}", flush=True)
            if view is not None:
                view.status.text = f"너트당 추력 {push_force_n:g} N · 회전 토크 {spin_torque_nm:g} N·m"
            for force_attr, torque_attr in loads:
                force_attr.Set(push)
                torque_attr.Set(torque)
            initial, _ = measure()
            for frame in range(480 * 12):
                step(frame)
                if frame % 4 == 0:
                    reached, _ = measure()
                    if np.all(initial - reached >= approach_offset):
                        break
            advance_mm = (initial - reached) * 1000
            print(f"PUSH advance_mm={advance_mm}", flush=True)
            for force_attr, _ in loads:
                force_attr.Set(Gf.Vec3f(0))
            if not np.all(advance_mm >= approach_offset * 1000):
                print("FAIL push: one or more nuts did not reach the engagement depth", flush=True)
                if view is not None:
                    view.hold_result(False)
                    if view.restart_requested and view.is_open():
                        reset_thread_contact(sim, view, ctx)
                        return None
                return False
            # 축힘을 끈 뒤 회전 토크만 유지해 나사 피치와 실제 전진량을 비교한다.
            print("LOAD axial push off; spin torque retained", flush=True)
            start, last = measure()
        turns = np.zeros(5)
        for frame in range(480*12):
            step(frame)
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
        if view is not None:
            view.hold_result(passed)
            if view.restart_requested and view.is_open():
                # 시험 중 생성한 너트·조인트와 물리 상태를 버리고 원본 장면을 다시 연다.
                reset_thread_contact(sim, view, ctx)
                return None
        return passed
    except CycleInterrupted:
        print("Contact test interrupted", flush=True)
        return False
    finally:
        for force_attr, torque_attr in loads:
            if force_attr and torque_attr:
                force_attr.Set(Gf.Vec3f(0))
                torque_attr.Set(Gf.Vec3f(0))
        if view is not None:
            view.close()


def reset_thread_contact(sim, view, ctx):
    from isaacsim.core.api import SimulationContext

    view.remember_camera()
    sim.stop()
    view.timeline.commit()
    SimulationContext.clear_instance()
    ctx.close_stage()



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-log", action="store_true", help="상세 상태 JSON도 함께 출력")
    parser.add_argument("--wheel-order", nargs="+", choices=("RL","FL"), default=["RL","FL"],
                        help="체결 대상 및 순서 (기본 RL FL)")
    parser.add_argument("--gui", action="store_true", help="나사면 접촉 시험을 GUI에서 관찰")
    parser.add_argument("--physical-threads", action="store_true", help="전체 공정에서 나사산 접촉으로 체결 검증")
    parser.add_argument("--push-to-engage", action="store_true", help="간격을 두고 축 방향으로 밀어 물림을 시험")
    parser.add_argument("--push-force-n", type=float, help="초기 물림 시험의 너트당 축 방향 힘 (기본 2 N)")
    parser.add_argument("--spin-torque-nm", type=float, help="초기 물림 시험의 너트당 회전 토크 (기본 0.02 N·m)")
    parser.add_argument("--headless", action="store_true", help="Check motion and exit")
    parser.add_argument("--captures", type=Path, help="Save loaded and seated close-ups")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--build-robot", type=Path, metavar="STL_DIR", help="Kawasaki STL로 로봇 USD 생성")
    modes.add_argument("--build-head", action="store_true", help="5축 체결 헤드 USD 생성")
    modes.add_argument("--check-thread-contact", action="store_true", help="나사면 접촉 시험")
    args = parser.parse_args()
    if len(set(args.wheel_order)) != len(args.wheel_order):
        parser.error("Duplicate wheel in --wheel-order")
    if args.gui and (not args.check_thread_contact or args.headless):
        parser.error("--gui requires --check-thread-contact and cannot be used with --headless")
    if args.push_to_engage and not args.check_thread_contact:
        parser.error("--push-to-engage requires --check-thread-contact")
    for value in (args.push_force_n, args.spin_torque_nm):
        if value is not None and (not args.push_to_engage or not isfinite(value) or value <= 0.):
            parser.error("Force and torque must be positive, finite values and require --push-to-engage")
    if args.physical_threads and (args.check_thread_contact or args.build_head or args.build_robot):
        parser.error("--physical-threads runs the full robot cycle")
    headless = args.headless or args.build_robot is not None or args.build_head or (
        args.check_thread_contact and not args.gui
    )
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
            return 0 if check_thread_contact(
                app, gui=args.gui, approach=args.push_to_engage,
                push_force_n=args.push_force_n if args.push_force_n is not None else 2.,
                spin_torque_nm=args.spin_torque_nm if args.spin_torque_nm is not None else .02,
            ) else 1
        return run(app, args.headless, args.captures, json_log=args.json_log, wheel_order=args.wheel_order,
                   physical_threads=args.physical_threads)
    finally:
        # USD 생성은 동기 Export로 끝난다. 생성 모드는 렌더러와 캡처 종료를 기다릴 필요가 없다.
        app.close(skip_cleanup=args.build_robot is not None or args.build_head)


if __name__ == "__main__":
    raise SystemExit(main())
