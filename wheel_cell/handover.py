"""Handover state for the fastening robot: the mount robot has put each wheel on its hub
and withdrawn. What does the wheel look like when the fastening robot arrives?

Stationary car (kinematic, moved to the origin, so world = car frame), the cell's own corner
model: hub on its bearing (drag, radial play), corner on the suspension, per-seed hub
tolerances (pilot diameter, PCD, stud positions), CAD web colliders, ring tire on springs.
Each of the four wheels starts concentric with its hub, bolt pattern on the studs, its
mounting face `depth` off the hub face (as the mount robot let go), at rest. Gravity then
acts alone for `seconds`; the settled state is measured in the hub frame and the car frame.

    ./handover.sh                               # depths 6 and 0.3 mm, seeds 42 7 123 2024
    ./handover.sh --depths-mm 6 --seeds 42 --seconds 10
"""
import argparse
import json
import os
from datetime import datetime
from math import acos, atan2, degrees, sqrt
from pathlib import Path
from random import Random
import sys

import omni.kit.app

for _i, _arg in enumerate(sys.argv[:-1]):
    if _arg == "--config":
        os.environ["CELL_CONFIG"] = str(Path(sys.argv[_i+1]).resolve())
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics

sys.dont_write_bytecode = True
root = Path(__file__).resolve().parent
sys.path.insert(0, str(root))

from conveyor import quaternion
from corner import Tolerance
from fitment import BOLT_CENTERS, OFFSET, STUD_PROJECTION, WEB_THICKNESS
from physical_data import RIM_MASS, TIRE_MASS, WHEEL_MASS
from physics import Cell, HZ, ROOT


def frame(api, path):
    data = api.get_rigidbody_transformation(path)
    if not data.get("ret_val"):
        raise RuntimeError(f"no rigid body at {path}")
    m = Gf.Matrix4d().SetRotate(quaternion(data))
    m.SetTranslateOnly(Gf.Vec3d(*data["position"]))
    return m


def vec(v, scale=1.):
    return [round(float(x)*scale, 4) for x in v]


def stud_centres(seed, name):
    """Hub stud centres (hub frame, m) exactly as corner.build_corner drew them."""
    rng = Random(f"{seed}-{name}-corner")
    rng.uniform(0., 72.)
    return Tolerance(rng)


def place(stage, cell, depth):
    """Car to the origin (kinematic), every wheel and tire concentric on its hub at `depth`."""
    car = stage.GetPrimAtPath(ROOT+"/CN7")
    cache = UsdGeom.XformCache()
    before = cache.GetLocalToWorldTransform(car)
    UsdGeom.Xformable(car).GetOrderedXformOps()[0].Set(Gf.Vec3d(0.))
    UsdPhysics.RigidBodyAPI(car).GetKinematicEnabledAttr().Set(True)
    cache.Clear()
    moved = before.GetInverse()*cache.GetLocalToWorldTransform(car)
    for station in cell.stations:
        for path in (station.hub_path.rsplit("/", 1)[0]+"/Knuckle", station.hub_path):
            prim = stage.GetPrimAtPath(path)
            world = cache.GetLocalToWorldTransform(prim)*moved
            prim.GetAttribute("xformOp:translate").Set(world.ExtractTranslation())
            if prim.GetAttribute("xformOp:orient"):
                prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(world.ExtractRotationQuat()))
    cache.Clear()
    for station in cell.stations:
        hub = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(station.hub_path))
        start = hub.Transform(Gf.Vec3d(0., 0., depth))
        q = Gf.Quatf(hub.ExtractRotationQuat())
        for path in (station.wheel, station.tire):
            prim = stage.GetPrimAtPath(path)
            prim.GetAttribute("xformOp:translate").Set(start)
            prim.GetAttribute("xformOp:orient").Set(q)
            # Never sleep: a sleeping body stops reporting the contacts it rests on.
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr(0.)
    cache.Clear()
    return cache.GetLocalToWorldTransform(car)


def measure(cell, station, tolerance, car_inv, design_hub):
    api = cell.api
    hub = frame(api, station.hub_path)
    wheel = frame(api, station.wheel)
    tire = frame(api, station.tire)
    hub_inv = hub.GetInverse()
    rel = wheel*hub_inv                                 # wheel in hub frame
    centre = rel.ExtractTranslation()
    axis = rel.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
    tilt = degrees(acos(max(-1., min(1., axis[2]))))
    # Direction of the tilt in the car frame: where the outboard axis end points, off the hub axis.
    hub_axis_car = (hub*car_inv).TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
    wheel_axis_car = (wheel*car_inv).TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
    lean = wheel_axis_car-hub_axis_car
    x_dir = rel.TransformDir(Gf.Vec3d(1, 0, 0))
    phase = degrees(atan2(x_dir[1], x_dir[0]))
    phase = (phase+36.) % 72.-36.
    studs = []
    for i, ((sx, sy), (hx, hy)) in enumerate(zip(tolerance.studs, BOLT_CENTERS)):
        tip_hub = Gf.Vec3d(sx, sy, OFFSET+STUD_PROJECTION)
        base_hub = Gf.Vec3d(sx, sy, OFFSET)
        # Wheel lug hole: mounting face (inboard) and outer web face (nut side) centres.
        face_hub = rel.Transform(Gf.Vec3d(hx, hy, OFFSET))
        outer_hub = rel.Transform(Gf.Vec3d(hx, hy, OFFSET+WEB_THICKNESS))
        studs.append({
            "index": i,
            "stud_tip_car_mm": vec((hub*car_inv).Transform(tip_hub), 1000),
            "stud_axis_car": vec(hub_axis_car),
            "stud_tip_hub_mm": vec(tip_hub, 1000),
            "hole_outer_centre_car_mm": vec((hub*car_inv).Transform(outer_hub), 1000),
            "face_gap_mm": round((face_hub[2]-OFFSET)*1000, 3),
            "stud_out_of_web_mm": round((tip_hub[2]-outer_hub[2])*1000, 3),
            "hole_to_stud_radial_mm": round(1000*sqrt((outer_hub[0]-sx)**2+(outer_hub[1]-sy)**2), 3),
        })
    tire_rel = tire*wheel.GetInverse()
    return {
        "hub": {
            "centre_car_mm": vec((hub*car_inv).ExtractTranslation(), 1000),
            "axis_car": vec(hub_axis_car),
            "moved_from_design_mm": vec((hub*car_inv).ExtractTranslation()-design_hub, 1000),
            "stud0_direction_car": vec((hub*car_inv).TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()),
            "pilot_diameter_mm": round(tolerance.pilot*1000, 4),
        },
        "wheel": {
            "centre_car_mm": vec((wheel*car_inv).ExtractTranslation(), 1000),
            "axis_car": vec(wheel_axis_car),
            "orientation_car_wxyz": vec([(wheel*car_inv).ExtractRotationQuat().GetReal(),
                                         *(wheel*car_inv).ExtractRotationQuat().GetImaginary()]),
            "axial_gap_at_centre_mm": round(centre[2]*1000, 3),
            "radial_offset_mm": vec((centre[0]*1000, centre[1]*1000)),
            "tilt_deg": round(tilt, 3),
            "tilt_axis_tip_shift_car": vec(lean),
            "bolt_phase_error_deg": round(phase, 3),
            "tire_ring_offset_mm": vec(tire_rel.ExtractTranslation(), 1000),
        },
        "studs": studs,
    }


def run(stage_path, seed, depth, seconds):
    stage = Usd.Stage.Open(str(stage_path))
    stage.SetEditTarget(stage.GetSessionLayer())
    cell = Cell(stage, seed, robots=False, start="gate")
    try:
        car_world = place(stage, cell, depth)
        car_inv = car_world.GetInverse()
        design = {s.name: Gf.Vec3d(s.hub) for s in cell.stations}
        tolerances = {s.name: stud_centres(seed, s.name) for s in cell.stations}
        contacts = {s.name: set() for s in cell.stations}
        history = {s.name: [] for s in cell.stations}
        steps = int(seconds*HZ)
        for i in range(steps):
            cell.sim.simulate(1/HZ, i/HZ)
            cell.sim.fetch_results()
            if i >= steps-HZ//2:                       # contacts over the last half second
                for header in cell.sim.get_contact_report()[0]:
                    if not header.num_contact_data:
                        continue
                    paths = [str(PhysicsSchemaTools.intToSdfPath(v)) for v in (header.collider0, header.collider1)]
                    for s in cell.stations:
                        if any(p.startswith(s.wheel+"/") for p in paths):
                            contacts[s.name].update(p.rsplit("/", 1)[-1] for p in paths
                                                    if p.startswith(s.hub_path+"/"))
            else:
                cell.sim.get_contact_report()
            if i % (HZ//4) == 0 or i == steps-1:
                for s in cell.stations:
                    rel = frame(cell.api, s.wheel)*frame(cell.api, s.hub_path).GetInverse()
                    history[s.name].append((i/HZ, rel.ExtractTranslation()[2]*1000,
                                            degrees(acos(max(-1., min(1., rel.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()[2]))))))
        result = {}
        for s in cell.stations:
            m = measure(cell, s, tolerances[s.name], car_inv, design[s.name])
            h = history[s.name]
            last_second = [p for p in h if p[0] >= seconds-1.]
            m["settling"] = {
                "gap_mm_over_time": [[round(t, 2), round(g, 3), round(a, 3)] for t, g, a in h[::4]],
                "gap_change_last_1s_mm": round(last_second[-1][1]-last_second[0][1], 4),
                "tilt_change_last_1s_deg": round(last_second[-1][2]-last_second[0][2], 4),
            }
            m["hub_parts_touching"] = sorted(contacts[s.name])
            m["on_studs"] = m["wheel"]["axial_gap_at_centre_mm"] < STUD_PROJECTION*1000-WEB_THICKNESS*1000
            result[s.name] = m
            print("HANDOVER", f"seed={seed}", f"depth={depth*1000:.1f}mm", s.name,
                  f"gap={m['wheel']['axial_gap_at_centre_mm']:+.3f}mm",
                  f"tilt={m['wheel']['tilt_deg']:.3f}deg",
                  f"faces={[st['face_gap_mm'] for st in m['studs']]}",
                  f"drift1s={m['settling']['gap_change_last_1s_mm']:+.4f}mm",
                  f"touch={m['hub_parts_touching']}", flush=True)
        return result
    finally:
        cell.close()


def main():
    args = argparse.ArgumentParser()
    args.add_argument("--stage", type=Path, default=root.parent/"scene"/"wheel_cell.usda")
    args.add_argument("--seeds", type=int, nargs="+", default=[42, 7, 123, 2024])
    args.add_argument("--depths-mm", type=float, nargs="+", default=[6., .3])
    args.add_argument("--seconds", type=float, default=10.)
    args.add_argument("--out", type=Path, default=root/"output"/"handover")
    args.add_argument("--config", type=Path)
    config = args.parse_args()
    report = {
        "what": "wheel on its hub after the mount robot let go and withdrew, gravity only, settled",
        "frames": {
            "car": "CN7 body frame (car at the origin): x forward, y left, z up, metres in the "
                   "file shown as mm",
            "hub": "hub body frame: z = wheel axis outboard, hub face at z = OFFSET; "
                   "x = direction of stud 0 (stud k at 72k deg)",
        },
        "model": {
            "wheel_mass_kg": WHEEL_MASS, "rim_mass_kg": RIM_MASS, "tire_mass_kg": TIRE_MASS,
            "offset_mm": OFFSET*1000, "stud_projection_mm": STUD_PROJECTION*1000,
            "web_thickness_mm": WEB_THICKNESS*1000, "physics_hz": HZ, "seconds": config.seconds,
        },
        "runs": [],
    }
    for depth in config.depths_mm:
        for seed in config.seeds:
            report["runs"].append({"release_depth_mm": depth, "seed": seed,
                                   "stations": run(config.stage, seed, depth/1000, config.seconds)})
    config.out.mkdir(parents=True, exist_ok=True)
    path = config.out/f"handover_{datetime.now():%Y%m%d-%H%M%S}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=1))
    print("HANDOVER_FILE", path, flush=True)
    return True


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = 0 if main() else 1
    finally:
        omni.kit.app.get_app().post_quit(exit_code)
