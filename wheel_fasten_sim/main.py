"""독립 B→C 시험: SimulationApp → USD 세션 → World 물리 스텝 → Task 관측 → Controller.

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
    parser.add_argument("--through", choices=("FL", "FR", "front", "reload", "move", "full"), default="full")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "multi_gpu": False})
    try:
        return run(app, args.headless, args.through)
    finally:
        app.close()


def run(app, headless, through):
    import omni.usd
    from isaacsim.core.api import World
    sys.path.insert(0, str(HERE.parent / "wheel_cell"))

    context = omni.usd.get_context()
    wait_for_play = False
    while True:
        code, restart = run_session(app, headless, through, context, wait_for_play)
        if not restart:
            return code
        World.clear_instance()
        context.close_stage()
        # SimulationApp.is_running()는 스테이지가 닫힌 동안 False다.
        # 다음 루프에서 먼저 원본을 열고, 새 스테이지에서 Play를 기다린다.
        wait_for_play = True


def run_session(app, headless, through, context, wait_for_play):
    import carb
    import omni.timeline
    from isaacsim.core.api import World
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import PhysxSchema
    from fastening import FastenCell, FastenController, FastenTask

    if not context.open_stage(str(HERE / "wheel_fasten.usda")):
        raise RuntimeError("Could not load wheel_fasten.usda")
    while context.get_stage_loading_status()[2]:
        app.update()
    stage = context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    timeline = omni.timeline.get_timeline_interface()
    if wait_for_play:
        print("READY_FOR_PLAY", flush=True)
        player = carb.settings.get_settings()
        player.set_bool("/app/player/playSimulations", False)
        try:
            while app.is_running() and not timeline.is_playing():
                app.update()
        finally:
            player.set_bool("/app/player/playSimulations", True)
        if not app.is_running():
            return 1, False
    scene = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/World/Physics"))
    hz = scene.GetTimeStepsPerSecondAttr().Get()
    dt = 1. / hz
    world = World(physics_dt=dt, rendering_dt=1./60., physics_prim_path="/World/Physics",
                  set_defaults=False)
    world.get_physics_context().enable_fabric(False)
    if not headless:
        set_camera_view(eye=[10., 7.5, 5.7], target=[4.1, 0., 1.0])
    world.reset()
    controller = (FastenController(FastenTask(stage, through)) if through in ("FL", "FR")
                  else FastenCell(stage, through))
    previous = None
    frame = 0
    while app.is_running():
        if not world.is_playing():
            if headless or world.is_stopped():
                break
            app.update()
            continue
        state = controller.update(dt)
        if state != previous:
            print(f"{through} {frame*dt:6.2f}s {state}", flush=True)
            previous = state
        if state in ("DONE", "FAILED"):
            break
        # 화면을 그리는 프레임에서도 컨트롤러 갱신당 물리는 정확히 한 번만 진행한다.
        world.step(render=False)
        if not headless and frame % max(1, hz//60) == 0:
            world.render()
        frame += 1

    stopped = world.is_stopped()
    terminal = controller.state in ("DONE", "FAILED")
    if terminal:
        world.pause()
    result = controller.result()
    if not terminal:
        result.update(state="STOPPED", message="Simulation stopped before completion")
    print("RESULT", json.dumps(result, ensure_ascii=False), flush=True)
    if not headless:
        while app.is_running():
            if stopped or world.is_stopped():
                return int(controller.state != "DONE"), True
            if world.is_playing():
                world.pause()
            app.update()
    return int(controller.state != "DONE"), False


if __name__ == "__main__":
    raise SystemExit(main())
