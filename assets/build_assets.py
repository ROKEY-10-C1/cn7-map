"""Export the cell's AMR, robots, car body and wheel as standalone USD assets.

Source: the composed scene map/scene/wheel_cell.usda (order CN7_001, before the process starts).
Every output file is self-contained (meshes and materials copied in), metres / kilograms, +Z up.
Run with ./map/assets/build_assets.sh after ./map/wheel_cell/build.sh.
"""
import sys
import xml.etree.ElementTree as ET
from math import degrees
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
MAP = HERE.parent
sys.path.insert(0, str(MAP/"wheel_cell"))
from physical_data import (AMR_LIFT_MASS, AMR_MASS, BODY_MASS, CAMERA_MASS, CAMERA_SIZE,  # noqa: E402
                           WHEEL_MASS, box_inertia, wheel_inertia)
from process import CN7_R16, HUB_Z  # noqa: E402

SCENE = MAP/"scene"/"wheel_cell.usda"
ORDER = "/World/Orders/CN7_001"
LIBRARY = "/AssetLibrary"
LIFT_TRAVEL = .12           # hanger lift stroke in the scene animation
ARM_DRIVE = (550., 16.)     # same position-drive gains as the live cell (per degree)
VERSION = "1.0"
CAMERA_FILE = "RealSense_D455.usd"


# ---------------------------------------------------------------- copying from the scene
class Source:
    def __init__(self, path):
        self.stage = Usd.Stage.Open(str(path))
        self.layer = self.stage.Flatten()
        self.xforms = UsdGeom.XformCache(Usd.TimeCode.Default())

    def world(self, path):
        return self.xforms.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))

    def relative(self, path, anchor):
        """Transform of `path`'s own frame expressed in the frame of `anchor`."""
        return self.world(path)*self.world(anchor).GetInverse()


def new_stage(path, root, kind="component"):
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.)
    prim = UsdGeom.Xform.Define(stage, "/"+root).GetPrim()
    stage.SetDefaultPrim(prim)
    Usd.ModelAPI(prim).SetKind(kind)
    return stage, prim


def describe(prim, name, spec):
    prim.SetAssetInfo({"name": name, "version": VERSION, "identifier": Sdf.AssetPath(
        prim.GetStage().GetRootLayer().identifier.split("/")[-1])})
    prim.SetCustomDataByKey("cn7:spec", spec)
    prim.SetCustomDataByKey("cn7:source", f"map/scene/wheel_cell.usda {ORDER} (build_assets.py)")


def place(prim, matrix):
    """Author a matrix as translate + orient (+ scale when not unit)."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    t = Gf.Transform(matrix)
    xf.AddTranslateOp().Set(t.GetTranslation())
    q = t.GetRotation().GetQuat()
    if abs(q.GetReal()-1.) > 1e-9:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)
    s = t.GetScale()
    if max(abs(v-1.) for v in s) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(*s))


def group(stage, path, matrix=None):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    if matrix is not None:
        place(prim, matrix)
    return prim


def copy(src, stage, source_path, target_path, keep=lambda name: True):
    """Copy a prim (or only the children passing `keep`) from the flattened scene."""
    layer = stage.GetRootLayer()
    source = src.layer.GetPrimAtPath(source_path)
    if source is None:
        raise ValueError(f"Missing in scene: {source_path}")
    if keep is None:
        Sdf.CopySpec(src.layer, source_path, layer, target_path)
        return
    for child in source.nameChildren:
        if keep(child.name):
            Sdf.CopySpec(src.layer, child.path, layer, Sdf.Path(target_path).AppendChild(child.name))


def inline_instances(src, layer, root):
    """Flatten() turns instanceable prims into references to /Flattened_Prototype_N: copy them in."""
    while True:
        found = []

        def visit(path):
            spec = layer.GetObjectAtPath(path)
            if isinstance(spec, Sdf.PrimSpec):
                refs = spec.referenceList.GetAddedOrExplicitItems()
                if any(r.primPath.pathString.startswith("/Flattened_Prototype") for r in refs):
                    found.append(spec)
        layer.Traverse(Sdf.Path(f"/{root}"), visit)
        if not found:
            return
        for spec in found:
            refs = spec.referenceList.GetAddedOrExplicitItems()
            spec.referenceList.ClearEdits()
            spec.ClearInfo("instanceable")
            for ref in refs:
                proto = src.layer.GetPrimAtPath(ref.primPath)
                for child in proto.nameChildren:
                    Sdf.CopySpec(src.layer, child.path, layer, spec.path.AppendChild(child.name))
                for prop in proto.properties:
                    if prop.name not in spec.properties:
                        Sdf.CopySpec(src.layer, prop.path, layer, spec.path.AppendProperty(prop.name))


def finish(src, stage, root):
    """Freeze animation, copy the bound materials in, and check nothing points outside the file."""
    layer = stage.GetRootLayer()
    looks = Sdf.Path(f"/{root}/Looks")
    used = set()

    inline_instances(src, layer, root)
    paths = []
    layer.Traverse(Sdf.Path(f"/{root}"), paths.append)
    for path in paths:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            samples = layer.ListTimeSamplesForPath(path)
            if samples:
                spec.default = layer.QueryTimeSample(path, sorted(samples)[0])
                for time in samples:
                    layer.EraseTimeSample(path, time)
        if isinstance(spec, Sdf.RelationshipSpec) and spec.name.startswith("material:binding"):
            targets = list(spec.targetPathList.GetAddedOrExplicitItems())
            used.update(t for t in targets if t.HasPrefix("/Looks"))
            spec.targetPathList.ClearEdits()
            spec.targetPathList.explicitItems = [
                t.ReplacePrefix("/Looks", looks) if t.HasPrefix("/Looks") else t for t in targets]
    if used:
        spec = Sdf.CreatePrimInLayer(layer, looks)
        spec.specifier = Sdf.SpecifierDef
        spec.typeName = "Scope"
        for material in sorted(used):
            Sdf.CopySpec(src.layer, material, layer, material.ReplacePrefix("/Looks", looks))
    bad = []

    def check(path):
        spec = layer.GetObjectAtPath(path)
        items = []
        if isinstance(spec, Sdf.RelationshipSpec):
            items = spec.targetPathList.GetAddedOrExplicitItems()
        elif isinstance(spec, Sdf.AttributeSpec):
            items = spec.connectionPathList.GetAddedOrExplicitItems()
            if isinstance(spec.default, Sdf.AssetPath) and spec.default.path:
                bad.append(f"{path} asset {spec.default.path}")
        bad.extend(f"{path} -> {t}" for t in items if not t.HasPrefix(f"/{root}"))
    layer.Traverse(Sdf.Path(f"/{root}"), check)
    if bad:
        raise ValueError("References outside the asset:\n  "+"\n  ".join(bad[:20]))


# ---------------------------------------------------------------- physics helpers
def rigid(prim, mass, center=None, inertia=None, axes=None):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(float(mass))
    if center is not None:
        api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, center)))
    if inertia is not None:
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    if axes is not None:
        api.CreatePrincipalAxesAttr(Gf.Quatf(axes))


def tensor_mass(prim, mass, center, tensor):
    inertia, axes = np.linalg.eigh(tensor)
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    rotation = Gf.Matrix3d(*axes.T.ravel().tolist()).ExtractRotation().GetQuat()
    rigid(prim, mass, center, inertia, rotation)


def collide(prim, approximation=None, hide=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if approximation and prim.IsA(UsdGeom.Mesh):
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
    if hide:
        UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)


def joint(stage, kind, path, body0, body1, pos0=(0, 0, 0), rot0=None, pos1=(0, 0, 0)):
    j = kind.Define(stage, path)
    if body0 is not None:
        j.CreateBody0Rel().SetTargets([body0.GetPath()])
    j.CreateBody1Rel().SetTargets([body1.GetPath()])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(rot0) if rot0 is not None else Gf.Quatf(1.))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot1Attr(Gf.Quatf(1.))
    return j


def drive(j, axis, stiffness, damping, force):
    api = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
    api.CreateTypeAttr("force")
    api.CreateStiffnessAttr(stiffness)
    api.CreateDampingAttr(damping)
    api.CreateMaxForceAttr(force)
    api.CreateTargetPositionAttr(0.)


# ---------------------------------------------------------------- AMR
def build_amr(src, out):
    root = "KMP1500"
    stage, prim = new_stage(out, root)
    carrier = ORDER+"/Carrier"
    hanger = carrier+"/Hanger"
    lift_parts = ("SaddleRail", "Support", "Crossmember", "ChassisPad")
    chassis = group(stage, f"/{root}/Chassis")
    copy(src, stage, carrier+"/AMR", f"/{root}/Chassis/Visuals", keep=None)
    lift = group(stage, f"/{root}/Hanger", src.relative(hanger, carrier))
    group(stage, f"/{root}/Hanger/Visuals")
    copy(src, stage, hanger, f"/{root}/Hanger/Visuals", keep=lambda n: n.startswith(lift_parts))
    finish(src, stage, root)
    # Physics: chassis + hanger on a vertical prismatic lift; drive targets are metres.
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    size = (2., .8, .47)
    box = UsdGeom.Cube.Define(stage, f"/{root}/Chassis/Collision")
    box.CreateSizeAttr(1.)
    box.AddTranslateOp().Set(Gf.Vec3d(0, 0, size[2]/2))
    box.AddScaleOp().Set(Gf.Vec3f(*size))
    collide(box.GetPrim(), hide=True)
    rigid(chassis, AMR_MASS, (0, 0, .2), box_inertia(AMR_MASS, size))
    for part in stage.GetPrimAtPath(f"/{root}/Hanger/Visuals").GetChildren():
        if part.GetName().startswith(("SaddleRail", "ChassisPad")):
            collide(part)
    rigid(lift, AMR_LIFT_MASS)
    hanger_pos = src.relative(hanger, carrier).ExtractTranslation()
    j = joint(stage, UsdPhysics.PrismaticJoint, f"/{root}/LiftJoint", chassis, lift, pos0=hanger_pos)
    j.CreateAxisAttr("Z")
    j.CreateLowerLimitAttr(-LIFT_TRAVEL)
    j.CreateUpperLimitAttr(0.)
    drive(j, "linear", 2.e6, 1.e5, 2.e4)
    describe(prim, "KUKA KMP 1500 omniMove AMR (CN7 carrier)", {
        "model": "KUKA KMP 1500 omniMove", "sizeM": Gf.Vec3d(*size), "massKg": AMR_MASS,
        "liftMassKg": AMR_LIFT_MASS, "payloadKg": 1500., "liftStrokeM": LIFT_TRAVEL,
        "origin": "footprint centre on the floor, +X forward"})
    stage.GetRootLayer().Save()


# ---------------------------------------------------------------- car body and wheel
CAR_PARTS = ("SedanShell", "Undertray", "TorsionBeam", "FrontSubframe", "DoorHandle", "Grille",
             "Wheelhouse_", "FL", "FR", "RL", "RR")


def car_anchor(src):
    hanger = ORDER+"/Carrier/Hanger"
    ground = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, HUB_Z-CN7_R16.radius))
    return ground*src.world(hanger)


def build_wheel(src, out):
    root = "Wheel_205_55R16"
    stage, prim = new_stage(out, root)
    copy(src, stage, LIBRARY+"/Wheel_16", f"/{root}/Visuals", keep=None)
    finish(src, stage, root)
    rigid(prim, WHEEL_MASS, (0, 0, 0), wheel_inertia(CN7_R16))
    for part in Usd.PrimRange(stage.GetPrimAtPath(f"/{root}/Visuals/Tire")):
        if part.IsA(UsdGeom.Mesh):
            collide(part, "convexHull")
    describe(prim, "CN7 wheel 205/55 R16 on 6.5J x 16", {
        "tire": CN7_R16.name, "rim": "6.5J x 16", "outerDiameterM": 2*CN7_R16.radius,
        "widthM": CN7_R16.width, "massKg": WHEEL_MASS, "pcdM": .1143, "centerBoreM": .0671,
        "origin": "wheel centre, spin axis local Z"})
    stage.GetRootLayer().Save()


def build_car(src, out, wheel_file):
    root = "CN7_Body"
    stage, prim = new_stage(out, root)
    hanger = ORDER+"/Carrier/Hanger"
    anchor = car_anchor(src)
    group(stage, f"/{root}/Visuals", src.world(hanger)*anchor.GetInverse())
    copy(src, stage, hanger, f"/{root}/Visuals",
         keep=lambda n: n.startswith(CAR_PARTS) and not n.startswith("Wheel_"))
    finish(src, stage, root)
    rigid(prim, BODY_MASS)
    shell = stage.GetPrimAtPath(f"/{root}/Visuals/SedanShell/StampedBody")
    if shell:
        collide(shell, "convexHull")
    collide(stage.GetPrimAtPath(f"/{root}/Visuals/Undertray"))
    # Variant: the four wheels as fitted at the end of the line.
    variants = prim.GetVariantSets().AddVariantSet("wheels")
    for name in ("none", "mounted"):
        variants.AddVariant(name)
    variants.SetVariantSelection("mounted")
    with variants.GetVariantEditContext():
        for corner in ("FL", "FR", "RL", "RR"):
            # Only the wheel's geometry: its colliders join the body, its own rigid body does not.
            w = stage.DefinePrim(f"/{root}/Wheels/Wheel_{corner}", "Xform")
            w.GetReferences().AddReference("./"+wheel_file, "/Wheel_205_55R16/Visuals")
            place(w, src.world(f"{hanger}/Wheel_{corner}")*anchor.GetInverse())
    variants.SetVariantSelection("none")
    describe(prim, "Hyundai Avante CN7 body-in-white (no wheels)", {
        "model": "Hyundai Avante (Elantra) CN7 facelift G1.6 IVT", "lengthM": 4.710,
        "widthM": 1.825, "wheelbaseM": 2.720, "frontTrackM": 1.585, "rearTrackM": 1.596,
        "massKg": BODY_MASS, "hubHeightM": CN7_R16.radius,
        "origin": "ground, midway between the axles; +X forward, +Y left"})
    stage.GetRootLayer().Save()


# ---------------------------------------------------------------- Doosan arms
def urdf_joints(urdf):
    tree = ET.parse(urdf).getroot()
    joints = []
    for i in range(1, 7):
        j = tree.find(f"./joint[@name='joint_{i}']")
        origin = j.find("origin")
        xyz = Gf.Vec3d(*map(float, origin.get("xyz").split()))
        r, p, y = map(degrees, map(float, origin.get("rpy", "0 0 0").split()))
        rot = (Gf.Rotation((1, 0, 0), r)*Gf.Rotation((0, 1, 0), p)*Gf.Rotation((0, 0, 1), y))
        limit = j.find("limit").attrib
        joints.append((xyz, rot, {k: float(v) for k, v in limit.items()}))
    return tree, joints


def inertial(link):
    node = link.find("inertial")
    values = node.find("inertia").attrib
    tensor = np.array([[float(values["i"+min(a, b)+max(a, b)]) for b in "xyz"] for a in "xyz"])
    origin = node.find("origin")
    center = np.array(list(map(float, origin.get("xyz", "0 0 0").split())))
    r, p, y = map(degrees, map(float, origin.get("rpy", "0 0 0").split()))
    rot = Gf.Rotation((1, 0, 0), r)*Gf.Rotation((0, 1, 0), p)*Gf.Rotation((0, 0, 1), y)
    axes = np.asarray(Gf.Matrix3d(rot)).T
    return float(node.find("mass").get("value")), center, axes@tensor@axes.T


ARMS = {
    "h2515": ("Doosan H2515", 25., 1.500),
    "m1509": ("Doosan M1509", 15., .900),
}


def build_arm(src, model, out):
    title, payload, reach = ARMS[model]
    root = model.upper()
    stage, prim = new_stage(out, root)
    tree, joints = urdf_joints(MAP/"wheel_cell"/"assets"/f"{model}.urdf")
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    names = ["base_link"]+[f"link_{i}" for i in range(1, 7)]
    world = Gf.Matrix4d(1.)
    links = []
    for index, name in enumerate(names):
        if index:
            xyz, rot, _ = joints[index-1]
            world = Gf.Matrix4d(rot, xyz)*world
        link = group(stage, f"/{root}/{name}", world)
        copy(src, stage, f"{LIBRARY}/{model}_Link_{index}", f"/{root}/{name}/Visuals", keep=None)
        copy(src, stage, f"{LIBRARY}/{model}_Collision_{index}", f"/{root}/{name}/Collisions", keep=None)
        links.append(link)
    finish(src, stage, root)
    total = 0.
    for index, (name, link) in enumerate(zip(names, links)):
        mass, center, tensor = inertial(tree.find(f"./link[@name='{name}']"))
        total += mass
        tensor_mass(link, mass, center, tensor)
        collisions = stage.GetPrimAtPath(f"{link.GetPath()}/Collisions")
        UsdGeom.Imageable(collisions).CreatePurposeAttr(UsdGeom.Tokens.guide)
        for part in Usd.PrimRange(collisions):
            if part.IsA(UsdGeom.Mesh):
                collide(part, "convexHull")
    # Fixed to the world at the asset origin; delete root_joint to mount it on a moving carriage.
    joint(stage, UsdPhysics.FixedJoint, f"/{root}/Joints/root_joint", None, links[0])
    limits = []
    for i, (xyz, rot, limit) in enumerate(joints, 1):
        j = joint(stage, UsdPhysics.RevoluteJoint, f"/{root}/Joints/joint_{i}", links[i-1], links[i],
                  pos0=xyz, rot0=rot.GetQuat())
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(degrees(limit["lower"]))
        j.CreateUpperLimitAttr(degrees(limit["upper"]))
        drive(j, "angular", *ARM_DRIVE, limit["effort"])
        j.GetPrim().CreateAttribute("cn7:maxVelocityDeg", Sdf.ValueTypeNames.Float).Set(
            degrees(limit["velocity"]))
        limits.append((degrees(limit["lower"]), degrees(limit["upper"]),
                       degrees(limit["velocity"]), limit["effort"]))
    describe(prim, title, {
        "model": title, "payloadKg": payload, "reachM": reach, "armMassKg": round(total, 3),
        "dof": 6, "origin": "base_link (mounting flange centre), zero pose = URDF zero"})
    stage.GetRootLayer().Save()
    return links, limits, total


TOOLS = {
    "mount": ("LeftMount", "h2515", ("GuideAdapter", "GuideGripper", "GuideJaw", "D455Assembly"),
              "Mount robot: H2515 + wheel guide gripper + RealSense D455"),
    "fasten": ("LeftFasten", "m1509", ("Nutrunner", "DriveFins", "GearHead", "Socket", "D455Assembly"),
               "Fastening robot: M1509 + 5-spindle nutrunner + RealSense D455"),
}


def build_tooled(src, role, arm_file, out):
    actor, model, parts, title = TOOLS[role]
    root = {"mount": "MountRobot", "fasten": "FastenRobot"}[role]
    stage, prim = new_stage(out, root)
    prim.GetReferences().AddReference("./"+arm_file)
    tool_path = f"/{root}/link_6/Tool"
    group(stage, tool_path)
    copy(src, stage, f"{ORDER}/{actor}/Joint_6", tool_path,
         keep=lambda n: n.startswith(parts) and n != "D455Assembly")
    finish(src, stage, root)
    camera = group(stage, tool_path+"/D455Assembly",
                   src.relative(f"{ORDER}/{actor}/Joint_6/D455Assembly", f"{ORDER}/{actor}/Joint_6"))
    camera.GetReferences().AddReference("../tools/"+CAMERA_FILE)
    for part in stage.GetPrimAtPath(tool_path).GetChildren():
        if part.GetName().startswith(parts[:-1]) and part.IsA(UsdGeom.Gprim):
            collide(part, "convexHull")
    # Link 6 mass with the camera, as in the live cell; other tool masses are unknown.
    tree = ET.parse(MAP/"wheel_cell"/"assets"/f"{model}.urdf").getroot()
    mass, center, tensor = inertial(tree.find("./link[@name='link_6']"))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    c = np.array(box.ComputeRelativeBound(camera, stage.GetPrimAtPath(f"/{root}/link_6"))
                 .ComputeAlignedRange().GetMidpoint())
    cam_tensor = np.diag(box_inertia(CAMERA_MASS, CAMERA_SIZE))
    total = mass+CAMERA_MASS
    com = (mass*center+CAMERA_MASS*c)/total
    combined = np.zeros((3, 3))
    for m, p, t in ((mass, center, tensor), (CAMERA_MASS, c, cam_tensor)):
        d = p-com
        combined += t+m*(np.dot(d, d)*np.eye(3)-np.outer(d, d))
    tensor_mass(stage.GetPrimAtPath(f"/{root}/link_6"), total, com, combined)
    describe(prim, title, {"arm": ARMS[model][0], "tool": ", ".join(parts),
                           "link6MassWithCameraKg": total})
    stage.GetRootLayer().Save()


def build_camera(src, out):
    root = "D455"
    stage, prim = new_stage(out, root)
    copy(src, stage, LIBRARY+"/D455", f"/{root}")
    finish(src, stage, root)
    describe(prim, "Intel RealSense D455 on tool bracket", {
        "model": "Intel RealSense D455", "massKg": CAMERA_MASS, "sizeM": Gf.Vec3d(*CAMERA_SIZE),
        "origin": "tool-flange side of the bracket; camera prim ColorOptics"})
    stage.GetRootLayer().Save()


def main():
    src = Source(SCENE)
    built = []
    build_camera(src, HERE/"tools"/CAMERA_FILE); built.append("tools/"+CAMERA_FILE)
    wheel = "CN7_Wheel_205-55R16.usd"
    build_wheel(src, HERE/"vehicle"/wheel); built.append("vehicle/"+wheel)
    build_car(src, HERE/"vehicle"/"Hyundai_Avante_CN7_Body.usd", wheel)
    built.append("vehicle/Hyundai_Avante_CN7_Body.usd")
    build_amr(src, HERE/"amr"/"KUKA_KMP1500_AMR.usd"); built.append("amr/KUKA_KMP1500_AMR.usd")
    for model, role, tooled in (("h2515", "mount", "MountRobot_H2515.usd"),
                                ("m1509", "fasten", "FastenRobot_M1509.usd")):
        arm = f"Doosan_{model.upper()}.usd"
        build_arm(src, model, HERE/"robots"/arm)
        build_tooled(src, role, arm, HERE/"robots"/tooled)
        built += ["robots/"+arm, "robots/"+tooled]
    for item in built:
        print("wrote", item, f"{(HERE/item).stat().st_size/1e6:.1f} MB")


if __name__ == "__main__":
    main()
