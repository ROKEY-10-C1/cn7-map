import sys
from pxr import Usd, UsdPhysics, UsdGeom
st = Usd.Stage.Open(sys.argv[1])
print("default", st.GetDefaultPrim().GetPath(), "mpu", UsdGeom.GetStageMetersPerUnit(st), "up", UsdGeom.GetStageUpAxis(st))
for prim in st.Traverse():
    t = prim.GetTypeName(); tags = []
    if prim.HasAPI(UsdPhysics.RigidBodyAPI): tags.append("RB")
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI): tags.append("ARTROOT")
    if prim.HasAPI(UsdPhysics.MassAPI):
        tags.append(f"mass={UsdPhysics.MassAPI(prim).GetMassAttr().Get()}")
    if "Joint" in t:
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        if d and d.GetStiffnessAttr().Get() is not None:
            tags.append(f"k={d.GetStiffnessAttr().Get()} c={d.GetDampingAttr().Get()} f={d.GetMaxForceAttr().Get()}")
        j = UsdPhysics.Joint(prim)
        tags.append(f"b0={j.GetBody0Rel().GetTargets()} b1={j.GetBody1Rel().GetTargets()}")
    if t == "Mesh" or (prim.GetPath().pathString.count("/") > 3 and not tags): continue
    print(prim.GetPath(), t, " ".join(map(str, tags)))
