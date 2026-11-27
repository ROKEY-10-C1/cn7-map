"""map_wheel 장면만 Isaac Sim에서 여는 실행기 (전체 셀 wheel_cell/과 따로).

Isaac Python 환경에서 실행 (run.sh가 부름). 장면을 열고 카메라를 고른 뒤 타임라인을 반복 재생한다.
  --camera Bench|Hub|Side   --once (반복 안 함)   --smoke-frames N (창 없이 N번 갱신 후 종료, 확인용)
"""
import argparse
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    from isaacsim import SimulationApp

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=Path, default=HERE/"scene"/"mount_bench.usda")
    parser.add_argument("--camera", choices=("Bench", "Hub", "Side"), default="Bench")
    parser.add_argument("--once", action="store_true", help="한 번만 재생")
    parser.add_argument("--smoke-frames", type=int, default=0)
    args = parser.parse_args()
    if not args.stage.is_file():
        parser.error(f"{args.stage} 없음 → 먼저 map_wheel/build.py")
    app = SimulationApp({"headless": bool(args.smoke_frames), "width": 1600, "height": 900,
                         "window_width": 1600, "window_height": 960, "renderer": "RaytracedLighting"})
    try:
        import omni.timeline
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Sdf

        context = omni.usd.get_context()
        context.open_stage(str(args.stage.resolve()))
        while context.get_stage_loading_status()[2]:
            app.update()
        stage = context.get_stage()
        for path in ("/World/CN7", "/World/MountRobot", "/World/TireStand", f"/World/{args.camera}"):
            if not stage or not stage.GetPrimAtPath(path):
                raise RuntimeError(f"장면에 {path} 없음: {args.stage}")
        viewport = get_active_viewport()
        if viewport:
            viewport.camera_path = Sdf.Path(f"/World/{args.camera}")
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0.)
        timeline.set_end_time(stage.GetEndTimeCode()/stage.GetTimeCodesPerSecond())
        timeline.set_looping(not args.once)
        timeline.play()
        print(f"map_wheel: {args.stage.name} 재생, 카메라 {args.camera}, "
              f"{stage.GetEndTimeCode()/stage.GetTimeCodesPerSecond():.1f} s", flush=True)
        frames = 0
        while app.is_running():
            app.update()
            frames += 1
            if args.smoke_frames and frames >= args.smoke_frames:
                print(f"map_wheel smoke ok: {frames} frames, t={timeline.get_current_time():.2f} s", flush=True)
                break
    finally:
        app.close()


if __name__ == "__main__":
    main()
