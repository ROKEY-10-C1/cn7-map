"""Drop each asset on a floor in CPU PhysX for 2 s and check it settles where it should."""
import sys
from pathlib import Path

import omni.kit.app
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_assets import urdf_joints  # noqa: E402
HZ = 120


def scene(asset, z=0., variant=None, rotate_x=0., gravity=9.81):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    physics = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(gravity)
    floor = UsdGeom.Cube.Define(stage, "/Floor")
    floor.CreateSizeAttr(1.)
    floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -.05))
    floor.AddScaleOp().Set(Gf.Vec3f(20, 20, .1))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    prim = stage.DefinePrim("/Asset", "Xform")
    prim.GetReferences().AddReference(str(HERE/asset))
    if variant:
        prim.GetVariantSet(variant[0]).SetVariantSelection(variant[1])
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
    if rotate_x:
        xf.AddRotateXOp().Set(rotate_x)
    return stage


def run(stage, *paths, seconds=2.):
    """Step the stage and return the world positions of `paths` (read before detaching)."""
    import omni.physx
    sim = omni.physx.get_physx_simulation_interface()
    key = UsdUtils.StageCache.Get().Insert(stage)
    sim.attach_stage(key.ToLongInt())
    try:
        for i in range(int(seconds*HZ)):
            sim.simulate(1/HZ, i/HZ)
            sim.fetch_results()
        return [position(stage, path) for path in paths]
    finally:
        sim.detach_stage()
        UsdUtils.StageCache.Get().Erase(key)


def position(stage, path):
    return UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(Sdf.Path(path))).ExtractTranslation()


def expect(name, value, target, tolerance):
    ok = abs(value-target) <= tolerance
    print(f"{'PASS' if ok else 'FAIL'} {name}: {value:.4f} (expected {target:.4f} ± {tolerance})", flush=True)
    return ok


def main():
    results = []
    # Wheel lying flat (spin axis vertical) rests on its tread side: centre at half the width.
    s = scene("vehicle/CN7_Wheel_205-55R16.usd", z=.3)
    wheel, = run(s, "/Asset")
    results.append(expect("wheel rests flat, centre height m", wheel[2], .103, .01))
    # Car with the four wheels mounted stands on its tires (origin = ground).
    s = scene("vehicle/Hyundai_Avante_CN7_Body.usd", z=.02, variant=("wheels", "mounted"))
    car, = run(s, "/Asset")
    results.append(expect("car on its wheels, origin height m", car[2], 0., .02))
    # AMR stands on the floor; the lift drive holds the hanger at its zero (raised) position.
    s = scene("amr/KUKA_KMP1500_AMR.usd", z=.01)
    chassis, hanger = run(s, "/Asset/Chassis", "/Asset/Hanger")
    results.append(expect("AMR chassis height m", chassis[2], 0., .01))
    results.append(expect("AMR hanger height m", hanger[2], 0., .005))
    # Fixed-base arms hold the URDF zero pose against gravity.
    for asset, root in (("robots/Doosan_H2515.usd", "H2515"), ("robots/Doosan_M1509.usd", "M1509"),
                        ("robots/MountRobot_H2515.usd", "MountRobot"),
                        ("robots/FastenRobot_M1509.usd", "FastenRobot")):
        s = scene(asset)
        start = position(s, "/Asset/link_6")
        end, = run(s, "/Asset/link_6")
        drift = (end-start).GetLength()
        results.append(expect(f"{root} link_6 drift m", drift, 0., .005))
    # Joint drives follow targets: the flange reaches the URDF forward-kinematics position.
    # Gravity off: with the live cell's drive gains the H2515 sags ~1 cm when stretched out.
    for asset, model in (("robots/MountRobot_H2515.usd", "h2515"), ("robots/FastenRobot_M1509.usd", "m1509")):
        s = scene(asset, gravity=0.)
        _, joints = urdf_joints(HERE.parent/"wheel_cell"/"assets"/f"{model}.urdf")
        q = (20., 30., 40., 0., 30., 0.)
        world = Gf.Matrix4d(1.)
        for i, ((xyz, rot, _), angle) in enumerate(zip(joints, q), 1):
            s.GetPrimAtPath(Sdf.Path(f"/Asset/Joints/joint_{i}")).GetAttribute(
                "drive:angular:physics:targetPosition").Set(angle)
            world = Gf.Matrix4d(Gf.Rotation((0, 0, 1), angle), Gf.Vec3d(0))*Gf.Matrix4d(rot, xyz)*world
        end, = run(s, "/Asset/link_6", seconds=4.)
        results.append(expect(f"{model} flange error vs FK m", (end-world.ExtractTranslation()).GetLength(), 0., .001))
    return all(results)


if __name__ == "__main__":
    code = 1
    try:
        code = 0 if main() else 1
    finally:
        omni.kit.app.get_app().post_quit(code)
