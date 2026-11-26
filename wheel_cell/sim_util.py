"""USD/PhysX building helpers shared by every live module (bodies, joints, drives, colliders)."""

from math import degrees

import numpy as np
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

from cell_config import CONFIG
from live_params import ROOT


def vec(v):
    return Gf.Vec3f(*v)


def qfloat(q):
    return Gf.Quatf(q.GetReal(), vec(q.GetImaginary()))


def rotate(axis, angle):
    return Gf.Rotation(Gf.Vec3d(*axis), angle)


def transform(stage, path, pos=(0, 0, 0), orientation=None):
    item = UsdGeom.Xform.Define(stage, path)
    item.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if orientation is not None:
        item.AddOrientOp().Set(qfloat(orientation))
    return item


def reference(stage, path, source, strip_transform=False):
    prim = stage.DefinePrim(path)
    prim.GetReferences().AddInternalReference(source)
    if strip_transform:
        UsdGeom.Xformable(prim).ClearXformOpOrder()
    return prim


def body(stage, path, pos, orientation, mass, kinematic=False, light=False):
    # light: drive-dominated tool bodies (gripper); solver iterations are scene-wide here.
    prim = transform(stage, path, pos, orientation).GetPrim()
    api = UsdPhysics.RigidBodyAPI.Apply(prim)
    api.CreateKinematicEnabledAttr(kinematic)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    options = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    options.CreateSolverPositionIterationCountAttr(CONFIG["physics"]["position_iterations"])
    options.CreateSolverVelocityIterationCountAttr(CONFIG["physics"]["velocity_iterations"])
    return prim


def mass_properties(prim, center, inertia, axes=None):
    props=UsdPhysics.MassAPI(prim)
    props.CreateCenterOfMassAttr(vec(center))
    props.CreateDiagonalInertiaAttr(vec(inertia))
    if axes is not None:
        props.CreatePrincipalAxesAttr(qfloat(axes))


def tensor_properties(prim, center, tensor):
    inertia, axes=np.linalg.eigh(tensor)
    if min(inertia)<=0:
        raise ValueError(f"Invalid inertia tensor at {prim.GetPath()}")
    if np.linalg.det(axes)<0:
        axes[:,0]*=-1
    rotation=np.eye(4)
    rotation[:3,:3]=axes.T
    mass_properties(prim,center,inertia,Gf.Matrix4d(*rotation.ravel().tolist()).ExtractRotationQuat())


def urdf_properties(inertial):
    values=inertial.find("inertia").attrib
    tensor=np.array([[float(values["i"+min(a,b)+max(a,b)]) for b in "xyz"] for a in "xyz"])
    origin=inertial.find("origin")
    center=np.array(list(map(float,origin.get("xyz","0 0 0").split())))
    roll,pitch,yaw=map(degrees,map(float,origin.get("rpy","0 0 0").split()))
    rotation=rotate((1,0,0),roll)*rotate((0,1,0),pitch)*rotate((0,0,1),yaw)
    axes=np.asarray(Gf.Matrix3d(rotation)).T
    return float(inertial.find("mass").get("value")),center,axes@tensor@axes.T


def combined_properties(prim, components):
    mass=sum(value[0] for value in components)
    center=sum(weight*position for weight,position,_ in components)/mass
    tensor=np.zeros((3,3))
    for weight,position,inertia in components:
        d=position-center
        tensor+=inertia+weight*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    UsdPhysics.MassAPI(prim).GetMassAttr().Set(mass)
    tensor_properties(prim,center,tensor)


def collision(shape, precision=False):
    prim = shape.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    cfg = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    cfg.CreateContactOffsetAttr(.000045 if precision else .002)
    cfg.CreateRestOffsetAttr(0.)
    UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
    return prim.GetPath()


def cube(stage, path, pos, size):
    item = UsdGeom.Cube.Define(stage, path)
    item.CreateSizeAttr(1.)
    item.AddTranslateOp().Set(Gf.Vec3d(*pos))
    item.AddScaleOp().Set(Gf.Vec3d(*size))
    return collision(item)


def pin(stage, path, xy, z0, length, radius, material):
    item = UsdGeom.Cylinder.Define(stage, path)
    item.CreateAxisAttr("Z")
    item.CreateHeightAttr(length)
    item.CreateRadiusAttr(radius)
    item.AddTranslateOp().Set(Gf.Vec3d(xy[0],xy[1],z0+length/2))
    shape=collision(item, precision=True)
    UsdShade.MaterialBindingAPI.Apply(item.GetPrim()).Bind(material,materialPurpose="physics")
    return shape


def hull(stage, path, outline, z0, z1, precise, material=None):
    n = len(outline)
    pts = [Gf.Vec3f(x,y,z) for z in (z0,z1) for x,y in outline]
    return convex_prism(stage,path,pts,n,precise,material)


def convex_prism(stage,path,points,n,precise,material):
    faces = [tuple(reversed(range(n))), tuple(range(n,2*n))]
    faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    shape = UsdGeom.Mesh.Define(stage,path)
    shape.CreatePointsAttr(points)
    shape.CreateFaceVertexCountsAttr([len(f) for f in faces])
    shape.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("convexHull")
    result = collision(shape,precision=precise)
    if material:
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose="physics")
    return result


def constrain(joint, parent, child, pos0=(0,0,0), pos1=(0,0,0), rot0=None):
    if parent:
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([child.GetPath()])
    joint.CreateLocalPos0Attr(vec(pos0))
    joint.CreateLocalPos1Attr(vec(pos1))
    joint.CreateLocalRot0Attr(qfloat(rot0) if rot0 else Gf.Quatf(1.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint.CreateCollisionEnabledAttr(False)
    return joint


def drive(joint, axis, target, stiffness, damping, force):
    api=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis)
    api.CreateTypeAttr("force")
    api.CreateStiffnessAttr(stiffness)
    api.CreateDampingAttr(damping)
    api.CreateMaxForceAttr(force)
    return api.CreateTargetPositionAttr(target)


def material(stage,name,friction,dynamic=None,combine=None):
    mat=UsdShade.Material.Define(stage,ROOT+"/Materials/"+name)
    api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction*.82 if dynamic is None else dynamic)
    api.CreateRestitutionAttr(.02)
    if combine:
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return mat


def pose(stage, path, seconds):
    time=seconds*stage.GetTimeCodesPerSecond()
    matrix=UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(time)
    return matrix.ExtractTranslation(), matrix.ExtractRotationQuat()
