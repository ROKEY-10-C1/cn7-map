"""독립 FL 시험: SimulationApp → USD 세션 → World 물리 스텝 → Task 관측 → Controller.

제어와 물리 상태는 런타임 세션에서만 변경된다. GUI는 결괏값을 확인하도록
최종 프레임을 유지한다. --headless는 자동 검증용이다.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "multi_gpu": False})
    try:
        return run(app, args.headless)
    finally:
        app.close()


def run(app, headless):
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import PhysxSchema
    sys.path.insert(0, str(HERE.parent / "wheel_cell"))
    from fastening import FastenController, FastenTask

    context = omni.usd.get_context()
    if not context.open_stage(str(HERE / "wheel_fasten.usda")):
        raise RuntimeError("Could not load wheel_fasten.usda")
    while context.get_stage_loading_status()[2]:
        app.update()
    stage = context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    scene = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/World/Physics"))
    hz = scene.GetTimeStepsPerSecondAttr().Get()
    dt = 1. / hz
    world = World(physics_dt=dt, rendering_dt=1./60., physics_prim_path="/World/Physics",
                  set_defaults=False)
    world.get_physics_context().enable_fabric(False)
    if not headless:
        set_camera_view(eye=[6.5, 5.6, 3.2], target=[3.1, 1.0, 1.15])
    world.reset()
    controller = FastenController(FastenTask(stage, "FL"))
    previous = None
    frame = 0
    while app.is_running() and world.is_playing():
        state = controller.update(dt)
        if state != previous:
            print(f"FL {frame*dt:6.2f}s {state}", flush=True)
            previous = state
        if state in ("DONE", "FAILED"):
            break
        # 화면을 그리는 프레임에서도 컨트롤러 갱신당 물리는 정확히 한 번만 진행한다.
        world.step(render=False)
        if not headless and frame % max(1, hz//60) == 0:
            world.render()
        frame += 1

    world.pause()
    print("RESULT", json.dumps({"state": controller.state, "message": controller.message,
                                **controller.metrics}, ensure_ascii=False), flush=True)
    if not headless:
        while app.is_running():
            app.update()
    return 0 if controller.state == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
