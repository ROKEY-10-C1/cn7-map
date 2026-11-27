"""Physical roller surfaces and containment of the wheel supply conveyors."""

from math import degrees

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

from physical_data import ROLLER_RATED_SPEED, ROLLER_TORQUE, tube_properties
from process import LINE_Y, REJECT_GATE_HALF, REJECT_X

FACTORY="/World/Factory"
START_AHEAD=.76
TREAD_HALF=.095


def collider(prim, material=None):
    UsdPhysics.CollisionAPI.Apply(prim)
    collision=PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr(.002)
    collision.CreateRestOffsetAttr(0.)
    if material:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose="physics")


def powered_roller(shape, joint_path, speed, material, cache):
    prim=shape.GetPrim()
    stage=prim.GetStage()
    matrix=cache.GetLocalToWorldTransform(prim)
    origin=matrix.ExtractTranslation()
    orientation=matrix.ExtractRotationQuat()
    body=UsdPhysics.RigidBodyAPI.Apply(prim)
    # Angular speed is about the roller axis. X-axis rollers convey along -Y.
    axis=shape.GetAxisAttr().Get()
    radius=shape.GetRadiusAttr().Get()
    rate=degrees(speed/radius)*(1 if axis=="Y" else -1)
    mass,inertia=tube_properties(radius,shape.GetHeightAttr().Get())
    a,b,c=inertia
    properties=UsdPhysics.MassAPI.Apply(prim)
    properties.CreateMassAttr(mass)
    properties.CreateCenterOfMassAttr(Gf.Vec3f(0.))
    properties.CreateDiagonalInertiaAttr(Gf.Vec3f(a,c,b) if axis=="Y" else Gf.Vec3f(c,a,b))
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateAngularDampingAttr(0.)
    # Analytic circular contact avoids a faceted cylinder climbing under the load.
    roller=UsdGeom.Capsule.Define(stage,str(prim.GetPath())+"/RollingSurface")
    roller.CreateAxisAttr(axis)
    roller.CreateRadiusAttr(radius)
    roller.CreateHeightAttr(shape.GetHeightAttr().Get()-2*radius)
    roller.CreateVisibilityAttr("invisible")
    collider(roller.GetPrim(),material)
    joint=UsdPhysics.RevoluteJoint.Define(stage,joint_path)
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(Gf.Vec3f(origin))
    joint.CreateLocalRot0Attr(Gf.Quatf(orientation.GetReal(),Gf.Vec3f(orientation.GetImaginary())))
    # Match the unloaded surface speed immediately for the running line.
    body.CreateAngularVelocityAttr(Gf.Vec3f(matrix.TransformDir(Gf.Vec3d(0.,rate,0.) if axis=="Y"
                                                                  else Gf.Vec3d(rate,0.,0.))))
    motor=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),"angular")
    motor.CreateTypeAttr("force")
    motor.CreateStiffnessAttr(0.)
    # Speed-controller gain estimate: rated torque at a tenth of rated speed error.
    motor.CreateDampingAttr(ROLLER_TORQUE/degrees(.10*ROLLER_RATED_SPEED/radius))
    motor.CreateMaxForceAttr(ROLLER_TORQUE)
    motor.CreateTargetVelocityAttr(rate)
    return motor


def block(stage,path,pos,size,material=None):
    # Keep scale on the collider child so PhysX sees an unscaled actor frame.
    parent=UsdGeom.Xform.Define(stage,path)
    parent.AddTranslateOp().Set(Gf.Vec3d(*pos))
    shape=UsdGeom.Cube.Define(stage,path+"/Shape")
    shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3d(*size))
    shape.CreateDisplayColorAttr([(.22,.24,.26)])
    collider(shape.GetPrim(),material)
    return shape.GetPath()


def setup_factory(stage, root, material, guide_material, branch_side):
    """Reuse render primitives as colliders where geometry is simple."""
    from pxr import Usd

    cache=UsdGeom.XformCache()
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(FACTORY))):
        if prim.IsA(UsdGeom.Mesh) and prim.GetName()=="SafetyMesh":
            points=UsdGeom.Mesh(prim).GetPointsAttr().Get()
            low,high=min(p[0] for p in points),max(p[0] for p in points)
            bottom,top=min(p[2] for p in points),max(p[2] for p in points)
            slab=block(stage,str(prim.GetParent().GetPath())+"/ScreenCollision",
                       ((low+high)/2,0,(top+bottom)/2),(high-low,.013,top-bottom))
            UsdGeom.Imageable(stage.GetPrimAtPath(slab)).CreateVisibilityAttr("invisible")
            continue
        if not (prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Cylinder)):
            continue
        relative=str(prim.GetPath())[len(FACTORY)+1:]
        main=relative.split("/",1)[0]
        name=prim.GetName()
        if any(token in main for token in ("AxleTransfer","Feed","TruckUnload","QA_Reject","ConveyorBend","DiverterRoll")):
            if "roller" in name.lower() or "DiverterRoll" in name:
                if "DiverterRoll" in name:
                    UsdGeom.Xformable(prim).AddRotateZOp().Set(branch_side*150.)
                    speed=.30
                else:
                    speed=.28 if "QA_Reject" in main or "ConveyorBend" in main else -.32
                joint_path=str(prim.GetPath())+"_Axle"
                powered_roller(UsdGeom.Cylinder(prim),joint_path,speed,material,cache)
            else:
                is_guide=("_rail" in main or name.startswith("Edge_") or main.endswith("_Gate"))
                collider(prim,guide_material if is_guide else None)
        elif main in ("LeftRail","RightRail","DockPlate","RejectCrate","PoweredDiverter",
                      "TruckCargo","TruckCab","TruckFrame") or (
                "Guard" in main and name.startswith(("Stile_","Header_"))):
            collider(prim)
    for side,label in ((1,"Left"),(-1,"Right")):
        for i,(x0,x1) in enumerate(((-1.04,1.04),(1.68,4.71))):
            for edge in (-1,1):
                spans=((x0,REJECT_X-REJECT_GATE_HALF),(REJECT_X+REJECT_GATE_HALF,x1)) if (
                    side==edge==i==1 and stage.GetPrimAtPath(FACTORY+"/LeftFeed_Gate")) else ((x0,x1),)
                for j,(start,end) in enumerate(spans):
                    block(stage,f"{root}/Conveyor/{label}Guide_{i}_{edge+1}_{j}",
                          ((start+end)/2,side*LINE_Y+edge*.349,.549),
                          (end-start,.035,.102),guide_material)


def quaternion(data):
    q=data["rotation"]
    return Gf.Quatd(q[3],Gf.Vec3d(q[0],q[1],q[2]))
