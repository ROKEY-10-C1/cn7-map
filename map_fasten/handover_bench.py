"""CN7 handover-based fastening bench (Isaac Sim 5.1).

Place this file in cn7-map/map_fasten/ and run with Isaac Sim python.sh.
Builds the same corner physics as wheel_cell/handover.py, adds the existing
RS080N and fastening head, and measures the released-wheel state. No fastening
motion is commanded yet. All edits are session-only unless --export is used.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO / "wheel_cell"))
sys.dont_write_bytecode = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth-mm", type=float, choices=[0.3, 6.0], default=0.3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--export", type=Path, help="Export the INITIAL bench USD, before physics steps")
    parser.add_argument("--report", type=Path, help="Write measured post-release state as JSON")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting", "multi_gpu": False, "max_gpu_count": 1})
    cell = sim = None
    try:
        import omni.usd
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.utils.viewports import set_camera_view
        from physics import Cell, HZ, ROOT
        from handover import place, measure, stud_centres
        source = REPO / "scene" / "wheel_cell.usda"
        fasten = HERE / "scene" / "fasten_pose_check.usda"
        for path in (source, fasten):
            if not path.is_file():
                raise FileNotFoundError(path)
        context = omni.usd.get_context()
        if not context.open_stage(str(source)):
            raise RuntimeError(f"Could not open {source}")
        while context.get_stage_loading_status()[2]:
            app.update()
        stage = context.get_stage()
        session = stage.GetSessionLayer()
        # Add the existing RS080N/head composition without editing the original USD.
        layer = Sdf.Layer.FindOrOpen(str(fasten))
        if not layer:
            raise RuntimeError(f"Could not open layer {fasten}")
        session.subLayerPaths.insert(0, layer.identifier)
        stage.SetEditTarget(session)

        def deactivate(path):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsActive():
                prim.SetActive(False)

        # Remove the old idealized vehicle/wheel and second physics scene.
        for path in ("/World/Car", "/World/Wheel", "/World/Physics", "/World/WhiteFloor"):
            deactivate(path)
        # Same constructor and initial placement as the handover test.
        # manual=False avoids attaching a second PhysX stage; SimulationContext owns stepping.
        cell = Cell(stage, args.seed, robots=False, manual=False, start="gate")
        car_world = place(stage, cell, args.depth_mm / 1000.)
        stations = {s.name: s for s in cell.stations}
        # Keep all four physical corners. Remove unrelated production equipment only
        # after construction, since Cell needs the original replay source prims.
        keep = {"CN7", "Physics", "Floor", *stations.keys()}
        for child in list(stage.GetPrimAtPath(ROOT).GetChildren()):
            if child.GetName() not in keep:
                deactivate(str(child.GetPath()))
        for station in stations.values():
            corner = stage.GetPrimAtPath(f"{ROOT}/{station.name}")
            keep_parts = {"Knuckle", "Hub", "Suspension", "Bearing", "Wheel", "Tire", "TireMount"}
            for child in list(corner.GetChildren()):
                if child.GetName() not in keep_parts:
                    deactivate(str(child.GetPath()))
            # Display the actual physical hub/stud colliders, not the nominal replay studs.
            visual = stage.GetPrimAtPath(f"{ROOT}/CN7/Visual/{station.name}")
            if visual:
                for part in list(visual.GetChildren()):
                    if part.GetName().startswith(("Stud_", "Thread_", "Pilot", "Face")):
                        UsdGeom.Imageable(part).MakeInvisible()
            hub = stage.GetPrimAtPath(station.hub_path)
            for part in hub.GetChildren():
                if part.HasAPI(UsdPhysics.CollisionAPI):
                    UsdGeom.Imageable(part).MakeVisible()
        # Factory geometry is no longer needed for the independent bench. Keep Orders
        # and AssetLibrary as composition sources (Orders is already invisible).
        deactivate("/World/Factory")
        # Verify required bodies and joints before starting physics. A bad reference
        # should fail explicitly rather than produce a misleading PASS.
        required = [f"{ROOT}/CN7", f"{ROOT}/Physics", "/World/FastenRobot/base_link", "/World/FastenRobot/link_6/TCP"]
        for s in stations.values():
            required.extend([s.hub_path, s.wheel, s.tire, f"{ROOT}/{s.name}/Knuckle", f"{ROOT}/{s.name}/Bearing", f"{ROOT}/{s.name}/Suspension", f"{ROOT}/{s.name}/TireMount"])
        for i in range(5):
            required.extend([f"/World/Head/Axis_{i}/{name}" for name in ("Rotor", "Bearing", "Nut", "SocketRetention")])
        for path in required:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsActive():
                raise RuntimeError(f"Missing active prim: {path}")
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            if not joint.GetJointEnabledAttr().Get():
                continue
            for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()):
                for target in rel.GetTargets():
                    body = stage.GetPrimAtPath(target)
                    if not body or not body.IsActive():
                        raise RuntimeError(f"Dangling joint target: {prim.GetPath()} -> {target}")
        print(f"BENCH_READY depth_mm={args.depth_mm} seed={args.seed} physics_hz={HZ}", flush=True)
        if args.export:
            path = args.export.resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            if not stage.Flatten(False).Export(str(path)):
                raise RuntimeError(f"Could not export {path}")
            print(f"INITIAL_USD {path}", flush=True)
        # No controller is run: the handover test measures gravity/contact only.
        dt = 1.0 / HZ
        sim = SimulationContext(physics_dt=dt, rendering_dt=1/60, stage_units_in_meters=1,
                                physics_prim_path=f"{ROOT}/Physics")
        if not args.headless:
            set_camera_view(eye=[4.4, 5.4, 3.0], target=[1.3, 1.5, 1.1])
        sim.initialize_physics()
        sim.play()
        for i in range(max(1, round(args.seconds * HZ))):
            if not app.is_running():
                return 1
            sim.step(render=not args.headless and i % max(1, round(HZ/60)) == 0)
        # Use the same measurement functions and tolerance generation as handover.py.
        report = {"seed": args.seed, "release_depth_mm": args.depth_mm, "seconds": args.seconds,
                  "physics_hz": HZ, "stations": {}}
        for name, station in stations.items():
            result = measure(cell, station, stud_centres(args.seed, name), car_world.GetInverse(), Gf.Vec3d(station.hub))
            report["stations"][name] = result
            wheel = result["wheel"]
            print(f"HANDOVER {name} gap_mm={wheel['axial_gap_at_centre_mm']:.3f} "
                  f"tilt_deg={wheel['tilt_deg']:.3f} phase_deg={wheel['bolt_phase_error_deg']:.3f} "
                  f"stud_out_mm={[s['stud_out_of_web_mm'] for s in result['studs']]}", flush=True)
        if args.report:
            path = args.report.resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"REPORT {path}", flush=True)
        sim.pause()
        if not args.headless:
            print("Observation complete. Timeline paused; close the window to exit.", flush=True)
            while app.is_running() and context.get_stage() == stage:
                sim.render()
        return 0
    finally:
        if sim is not None:
            sim.stop()
        if cell is not None:
            cell.close()
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
