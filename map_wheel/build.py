"""map_wheel: 타이어를 집어 FL 허브에 끼우는 동작만 보는 작은 장면 (한 코너).

전체 셀(wheel_cell)에서 필요한 것만 남김:
  - 도크에 고정된 CN7 1대 (차체, FL 허브는 비어 있고 나머지 세 바퀴는 끼워진 상태), 받침 4개
  - 두산 H2515 1대 (레일 없이 고정 받침), 손은 wheel_cell/gripper.py 설계 (아래 롤러 2개 + 가운데 핀)
  - 타이어 받침대 1개 (타이어를 세워 둠)
좌표·치수는 셀과 같다 (월드 원점 = 도크, 허브 높이 process.HUB_Z). 그래서 여기서 맞춘 동작을 셀로 옮길 수 있다.

동작 (키프레임, 재생 시간 BENCH_END s): 대기 → 받침대 밑으로 → 롤러로 들어 올림·핀 넣기 → 허브 앞 10 cm 정렬
→ 5 cm/s로 삽입 → 놓기(손을 2 cm 내림) → 빠짐 → 대기. 직선 구간은 IK로 촘촘히 풀어 툴이 직선으로 감.
물리(PhysX 접촉·허브 볼트 끼움)는 다음 단계.

실행 (Isaac 또는 usd-core가 있는 파이썬):
  ~/isaacsim/python.sh map_wheel/build.py         → map_wheel/scene/mount_bench.usda
  열기: Isaac Sim File > Open map_wheel/scene/mount_bench.usda, Play
필요: scene/shared/asset_library.usdc, materials.usdc (wheel_cell/build.sh가 만든 것, 하위 레이어로 씀)
"""
import argparse
import sys
from math import ceil
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/"wheel_cell"))

from build import (AMBER, BRUSHED, DARK, FPS, RUBBER, box, cylinder, frame, robot_model,  # noqa: E402
                   vector, visibility, xform)
from fitment import half_track  # noqa: E402
from process import AXLE, CN7_R16, HUB_Z, STATIONS  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402
from robot import Arm  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from visuals import ALLOY, CONCRETE, LIB, PANEL  # noqa: E402
from wheel_geometry import wheelhouse  # noqa: E402

CORNER = "FL"
SIDE = STATIONS[CORNER][1]                      # +1: 차 왼쪽
HUB = np.array([AXLE, half_track(AXLE), HUB_Z])  # 허브 장착면 중심 (FL)
ROBOT_BASE = np.array([1.0, 1.7, .68])          # H2515 받침 위 (IK로 고름: 집는 자리·허브 둘 다 닿음)
PICK = np.array([1.9, 1.7, 1.10])               # 받침대 위 타이어 (장착면 중심, 축은 +y). 손목이 바닥 쪽으로 안 내려가게 높임
PARK = np.array([1.45, 2.25, 1.05])             # 대기 자세 (차 바깥쪽, 툴 자세 같음)
# 툴(플랜지) 자세: 타이어를 세워서 아래에서 받침. x = 바퀴 축(바깥쪽 +y), z = 위 (gripper.py 좌표)
TOOL_R = np.column_stack([(0, SIDE, 0), (-SIDE, 0, 0), (0, 0, 1)])
WHEEL_IN_TOOL = np.array([.046, 0., .512])      # 장착면 중심 (gripper.py: 바퀴 중심 .512 위, 핀 끝 x .050)
ROLLER_Y, ROLLER_Z, ROLLER_RADIUS, ROLLER_LENGTH = .1745, .210, .033, .225   # gripper.py
ARM_X, ARM_HINGE_Z, PIN_RADIUS = .135, .09, .03325
STAND_X = .277                                  # 받침대 롤러: 바퀴 중심에서 x ±, 4·8시 방향 (그리퍼 롤러 바깥)
STAND_Z = -.16
APPROACH = .10                                  # 허브 앞 정렬 거리
INSERT_SPEED = .05                              # m/s
STEP = 1/12                                     # 직선 구간 IK 간격 (s)

# (시각 s, 장착면 중심 위치 또는 None=대기 자세, 직선 이동 여부)
WAYPOINTS = [
    (0., PARK, False),
    (3., PICK+(0, 0, -.08), False),             # 받침대 밑 (그리퍼 롤러가 타이어 아래)
    (4.5, PICK, True),                          # 롤러 닿음
    (5.5, PICK+(0, 0, .03), True),              # 들어 올림 (받침대에서 뜸) → 핀 넣음
    (7., PICK+(0, 0, .15), True),
    (10., HUB+(0, SIDE*.30, .03), False),
    (12., HUB+(0, SIDE*APPROACH, 0), True),     # 허브 앞 10 cm, 축 정렬
    (12.+APPROACH/INSERT_SPEED, HUB, True),     # 삽입
    (15.5, HUB+(0, 0, -.02), True),             # 놓기: 핀 빼고 손 2 cm 내림 (바퀴는 허브 볼트에 걸림)
    (17., HUB+(0, SIDE*.35, -.02), True),       # 빠짐 (롤러가 타이어 밑에서 바깥으로)
    (19.5, PARK, False),
]
GRAB, RELEASE = 5.0, 15.0                       # 바퀴가 손에 붙어 있는 구간
BENCH_END = 21.


def solve(arm, base, wheel, seed):
    """장착면 중심 위치 → 6축 (TOOL_R 자세). 실패하면 None."""
    target = np.asarray(wheel) - TOOL_R @ WHEEL_IN_TOOL
    seeds = [seed] + list(np.random.default_rng(41).uniform(-2.4, 2.4, (30, 6)))
    for s in seeds:
        q = s.copy()
        for _ in range(200):
            frames = arm.fk(base, q)
            p, r, _ = frames[-1]
            err = np.r_[target-p, .3*Rotation.from_matrix(TOOL_R @ r.T).as_rotvec()]
            if np.linalg.norm(err[:3]) < 2e-4 and np.linalg.norm(err[3:]) < 6e-4:
                return q
            jac = np.array([np.r_[np.cross(a, p-o), .3*a] for o, _, a in frames]).T
            q = np.clip(q+np.clip(jac.T @ np.linalg.solve(jac @ jac.T+4e-4*np.eye(6), err), -.25, .25),
                        arm.lower, arm.upper)
    return None


def plan(arm):
    """[(시각, q)] — 직선 구간은 STEP마다 IK, 나머지는 관절 보간."""
    home = solve(arm, ROBOT_BASE, PARK, np.deg2rad([-4., 99., 67., 0., -166., 94.]))
    out, prev_t, prev_q = [(0., home)], 0., home
    previous_wheel = PARK
    for t, wheel, straight in WAYPOINTS[1:]:
        if straight:
            start = np.asarray(previous_wheel)
            n = max(1, ceil((t-prev_t)/STEP))
            samples, q = [], prev_q
            for i in range(1, n+1):
                q = solve(arm, ROBOT_BASE, start+(np.asarray(wheel)-start)*i/n, q)
                if q is None:
                    raise ValueError(f"IK 실패: {t:.1f} s 구간 {i}/{n}")
                samples.append((prev_t+(t-prev_t)*i/n, q))
        else:
            q = solve(arm, ROBOT_BASE, wheel, prev_q)
            if q is None:
                raise ValueError(f"IK 실패: {t:.1f} s {wheel}")
            samples = [(t, q)]
        for (ta, qa), (tb, qb) in zip([(prev_t, prev_q)]+samples[:-1], samples):
            need = np.max(np.abs(qb-qa)/arm.speed)
            if need > tb-ta+1e-9:
                raise ValueError(f"관절 속도 초과 {ta:.2f}-{tb:.2f} s: {need:.2f} s 필요")
        out += samples
        prev_t, prev_q = t, q
        previous_wheel = wheel
    return out


def key(stage, arm, groups, samples):
    """관절 각과 링크 자세를 키프레임으로 (관절 보간 구간은 12 Hz로 쪼갬)."""
    root = ROBOT_ROOT
    base = ROBOT_BASE-np.array(ROBOT_OFFSET)

    def pose(t, q):
        for i, angle in enumerate(q):
            stage.GetPrimAtPath(root+f"/Joint_{i+1}").CreateAttribute(
                "cell:jointTargetDeg", Sdf.ValueTypeNames.Double).Set(float(np.degrees(angle)), frame(t))
        for (pos, orient), (p, rot, _) in zip(groups, arm.fk(base, q)):
            pos.Set(vector(p), frame(t))
            x, y, z, w = Rotation.from_matrix(rot).as_quat()
            orient.Set(Gf.Quatf(w, x, y, z), frame(t))

    (t0, q0), rest = samples[0], samples[1:]
    pose(t0, q0)
    for t, q in rest:
        n = min(48, ceil((t-t0)*12)) if np.linalg.norm(q-q0) > 1e-6 else 1
        for i in range(1, n+1):
            pose(t0+(t-t0)*i/n, q0+(q-q0)*i/n)
        t0, q0 = t, q


def gripper(stage, flange):
    """gripper.py 모양만 (물리 없음): 어댑터, 바닥 빔, 기둥, 홈 롤러 2개, 핀 암, 센터 핀."""
    cylinder(stage, flange+"/Adapter", (0, 0, .03), .06, .06, BRUSHED)
    box(stage, flange+"/BaseBeam", (0, 0, ARM_HINGE_Z), (.08, 2*ROLLER_Y+.05, .04), PANEL)
    for j in (-1, 1):
        box(stage, flange+f"/Upright_{j+1}", (0, j*ROLLER_Y, (ARM_HINGE_Z+ROLLER_Z)/2), (.05, .04, ROLLER_Z-ARM_HINGE_Z),
            PANEL)
        cylinder(stage, flange+f"/Roller_{j+1}", (0, j*ROLLER_Y, ROLLER_Z), ROLLER_RADIUS, ROLLER_LENGTH, RUBBER, "X")
        for k in (-1, 1):
            cylinder(stage, flange+f"/RollerFlange_{j+1}_{k+1}", (k*ROLLER_LENGTH/2, j*ROLLER_Y, ROLLER_Z),
                     ROLLER_RADIUS+.015, .01, BRUSHED, "X")
    box(stage, flange+"/PinArm", (ARM_X, 0, (ARM_HINGE_Z+.53)/2), (.03, .05, .53-ARM_HINGE_Z), AMBER)
    cylinder(stage, flange+"/CentrePin", ((ARM_X+.05)/2, 0, WHEEL_IN_TOOL[2]), PIN_RADIUS, ARM_X-.05, ALLOY, "X")


def wheel_at(stage, path, pos, quat):
    item = UsdGeom.Xform.Define(stage, path)
    item.AddTranslateOp().Set(vector(pos))
    item.AddOrientOp().Set(quat)
    item.GetPrim().GetReferences().AddInternalReference(LIB+f"/Wheel_{CN7_R16.inches}")
    item.GetPrim().SetInstanceable(True)
    return item


ROBOT_ROOT = "/World/MountRobot"
ROBOT_OFFSET = (ROBOT_BASE[0], ROBOT_BASE[1]-SIDE*(1.57+.11), 0.)   # robot_model은 레일 y(RAIL_Y+.11)에 받침을 둠


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=HERE/"scene"/"mount_bench.usda")
    args = ap.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    shared = HERE.parent/"scene"/"shared"

    stage = Usd.Stage.CreateNew(str(args.out))
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(frame(BENCH_END))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    import os
    for name in ("asset_library.usdc", "materials.usdc"):
        stage.GetRootLayer().subLayerPaths.append(os.path.relpath(shared/name, args.out.parent))
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # 바닥, 조명, 카메라
    box(stage, "/World/Floor", (1.2, 1.0, -.05), (7., 6., .1), CONCRETE)
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(900.)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500.)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-40., 0., 35.))
    for name, eye, look, focal in (("Bench", (4.6, 4.4, 2.4), (1.3, 1.1, .8), 24.),
                                   ("Hub", (1.36, 3.2, 1.05), (1.36, .8, .94), 35.),
                                   ("Side", (4.2, 1.6, 1.0), (1.2, 1.3, .9), 28.)):
        cam = UsdGeom.Camera.Define(stage, f"/World/{name}")
        cam.CreateFocalLengthAttr(focal)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.05, 200.))
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look), Gf.Vec3d(0, 0, 1)).GetInverse()
        cam.AddTransformOp().Set(m)

    # 차: 도크 위치 그대로, 받침 4개 (AMR·행거 대신)
    car = "/World/CN7"
    UsdGeom.Xform.Define(stage, car)
    shell = UsdGeom.Xform.Define(stage, car+"/Shell")
    shell.GetPrim().GetReferences().AddInternalReference(LIB+"/Body")
    shell.GetPrim().SetInstanceable(True)
    box(stage, car+"/Undertray", (0, 0, 1.035), (3.43, 1.02, .115), DARK)
    for i, x in enumerate((-1.65, 1.65)):
        box(stage, f"/World/CarStand_{i}", (x, 0, .49), (.30, .60, .98), AMBER)
    hub_q = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -SIDE*90).GetQuat())
    for name, (x, side) in STATIONS.items():
        y = half_track(x)
        link, _ = xform(stage, car+f"/{name}", (x, side*y, HUB_Z))
        link.AddRotateXOp().Set(-side*90)
        link.GetPrim().GetReferences().AddInternalReference(LIB+f"/Corner_{name}")
        wheelhouse(stage, car+f"/Wheelhouse_{name}", x, side, HUB_Z, y)
        if name != CORNER:
            q = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -side*90).GetQuat())
            wheel_at(stage, car+f"/Wheel_{name}", (x, side*y, HUB_Z), q)

    # 타이어 받침대: 롤러 2개(4·8시)에 세워 둠, 축은 +y
    mid_y = PICK[1]-SIDE*WHEEL_IN_TOOL[0]
    for j in (-1, 1):
        x, z = PICK[0]+j*STAND_X, PICK[2]+STAND_Z
        box(stage, f"/World/TireStand/Post_{j+1}", (x, mid_y, (z-.045)/2), (.06, .10, z-.045), BRUSHED)
        cylinder(stage, f"/World/TireStand/Roller_{j+1}", (x, mid_y, z), .04, .24, RUBBER, "Y")
    box(stage, "/World/TireStand/Base", (PICK[0], mid_y, .02), (2*STAND_X+.2, .40, .04), AMBER)

    # 로봇 + 그리퍼
    arm = Arm(HERE.parent/"wheel_cell"/"assets"/"h2515.urdf")
    item, _ = xform(stage, ROBOT_ROOT, ROBOT_OFFSET)
    groups = robot_model(stage, ROBOT_ROOT, SIDE, arm, True)     # 장착 로봇 받침 (레일 y + .11)
    for extra in ("GuideAdapter", "GuideGripper", "GuideJaw_0", "GuideJaw_2"):   # 셀의 가이드 손 → 아래 그리퍼로 바꿈
        p = stage.GetPrimAtPath(ROBOT_ROOT+"/Joint_6/"+extra)
        if p:
            stage.RemovePrim(p.GetPath())
    gripper(stage, ROBOT_ROOT+"/Joint_6/Gripper")
    samples = plan(arm)
    # robot_model은 링크를 루트 기준으로 둠 → FK 기준점도 루트 기준
    key(stage, arm, groups, samples)

    # 바퀴: 받침대 위 → 손 → 허브 (보이는 구간만 바꿈)
    wheel_q = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -SIDE*90).GetQuat())
    on_stand = wheel_at(stage, "/World/Tire_OnStand", PICK, wheel_q)
    vis = UsdGeom.Imageable(on_stand.GetPrim()).CreateVisibilityAttr("inherited")
    vis.Set("inherited", 0)
    vis.Set("invisible", frame(GRAB))
    held_m = np.column_stack([(0, -1, 0), (0, 0, -1), (1, 0, 0)])   # 바퀴 x→-yf(월드 x), z→xf(축)
    x, y, z, w = Rotation.from_matrix(held_m).as_quat()
    held = wheel_at(stage, ROBOT_ROOT+"/Joint_6/HeldTire", WHEEL_IN_TOOL, Gf.Quatf(w, x, y, z))
    visibility(held, GRAB, RELEASE)
    mounted = wheel_at(stage, car+f"/Wheel_{CORNER}", HUB, wheel_q)
    visibility(mounted, RELEASE, BENCH_END+10)
    stage.GetRootLayer().Save()
    print(args.out, f"{len(samples)} 자세, {BENCH_END:.0f} s")


if __name__ == "__main__":
    main()
