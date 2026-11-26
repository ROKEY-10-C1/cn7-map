"""Live four-corner PhysX cell, method 1: flat tires on the station conveyors stop in a V-stop
pick zone for the mount robot's roller/centre-pin gripper; the car is docked on the AMR with
hubs on their bearings and suspension. Robot control is not part of this module yet."""

import os
from contextlib import nullcontext
from dataclasses import dataclass
from math import atan2, cos, degrees, sin, sqrt, tau
from pathlib import Path
from random import Random
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

from conveyor import START_AHEAD, TREAD_HALF, collider, quaternion, setup_factory
from fitment import ARCH_RADIUS, LUG_HOLE, OFFSET, PCD, STUD_DIAMETER, half_track
from physical_data import BODY_MASS, PAD_DYNAMIC_FRICTION, PAD_STATIC_FRICTION
from physical_data import RIM_MASS, annulus_inertia
from physical_data import CAMERA_MASS, CAMERA_SIZE, box_inertia, servo_damping
from physical_data import GUIDE_DYNAMIC_FRICTION, GUIDE_STATIC_FRICTION
from physical_data import AMR_ACCELERATION, AMR_LIFT_MASS, AMR_LIFT_SPEED, AMR_MASS, AMR_PAYLOAD, AMR_SIZE
from physical_data import AMR_SPEED_X, AMR_SPEED_Y, AMR_TRACTION_COEFFICIENT, AMR_WHEEL_RADIUS
from physical_data import DOCK_CLAMP_CLOSED, DOCK_CLAMP_OPEN
from process import BELT_Z, CN7_R16, HUB_Z, LINE_Y, RAIL_Y, STATIONS, schedule
from cell_config import CONFIG

ROOT = "/World/Live"
REFERENCE = "/World/Orders/CN7_001"
HZ = CONFIG["physics"]["hz"]
HOLE_ANGLE = degrees((LUG_HOLE-STUD_DIAMETER)/PCD)  # wheel turn the stud-hole clearance allows (~0.7 deg)
DOCK_READY = 20.                  # live AMR runs the 7 m gate -> dock leg and lifts by ~15 s
SOURCE_POSE = 30.5                # seconds on the mechanical preview stage
MIN_RAIL_SPACING = 1.55           # meters between carriage origins, including braking clearance
_FIRST_ORDER = next(schedule(False))
FRONT_SEAT = _FIRST_ORDER[5]-SOURCE_POSE   # task seconds at which the replay seats the front wheels
REAR_SEAT = _FIRST_ORDER[6]-SOURCE_POSE
GUIDE_LEAD = 2.2                  # replay: mount tool reaches the wheel-side standby before seating
GUIDE_EXIT = 1.35                 # replay: mount tool is back at its safe pose after seating
GUIDE_RETREAT = 2.0               # live: joint-space withdrawal from standby to the safe pose
TERMINAL_PHASES = frozenset(("pick_ready","feed_failed","interlocked","interference","dock_lost"))
# A lost dock can shift the car; no station may continue after it.
INTERLOCK_FAULTS = frozenset(("dock_lost",))


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


def body(stage, path, pos, orientation, mass, kinematic=False):
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


def _cross(o,a,b):
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])


def _convex(poly,sign):
    n=len(poly)
    return all(sign*_cross(poly[i],poly[(i+1)%n],poly[(i+2)%n])>=-1e-12 for i in range(n))


def merge_convex(triangles,max_vertices=24):
    """Hertel–Mehlhorn style merge: join polygons across shared edges while they stay convex.

    The union is exactly the input triangles, so the apertures keep their shape; PhysX just
    sees ~10x fewer convex pieces (each piece is one more broad/narrow-phase pair per contact).
    """
    key=lambda p:(round(p[0],7),round(p[1],7))
    polys=[[key(p) for p in tri] for tri in triangles]
    sign=1 if _cross(*polys[0])>0 else -1
    merged=True
    while merged:
        merged=False
        edges={}
        for index,poly in enumerate(polys):
            for i in range(len(poly)):
                edges[(poly[i],poly[(i+1)%len(poly)])]=index
        for index,poly in enumerate(polys):
            if poly is None:
                continue
            for i in range(len(poly)):
                a,b=poly[i],poly[(i+1)%len(poly)]
                other=edges.get((b,a))
                if other is None or other==index or polys[other] is None:
                    continue
                q=polys[other]
                j=q.index(b)
                # Walk this polygon from b round to a, then the neighbour from a round to b.
                k=poly.index(b)
                path=[poly[(k+m)%len(poly)] for m in range(len(poly))]          # b ... a
                qa=q.index(a)
                rest=[q[(qa+m)%len(q)] for m in range(1,len(q)-1)]              # after a, before b
                candidate=path+rest
                if len(candidate)<=max_vertices and _convex(candidate,sign):
                    polys[index]=candidate
                    polys[other]=None
                    merged=True
                    break
            if merged:
                break
        polys=[p for p in polys if p is not None]
    return polys


def collision_web(stage):
    """Use only mounting-face triangles, keeping the six apertures open on CPU."""
    web=UsdGeom.Mesh(stage.GetPrimAtPath("/AssetLibrary/Wheel_16/CenterWeb"))
    coords=web.GetPointsAttr().Get()
    cursor=0
    result=[]
    indices=web.GetFaceVertexIndicesAttr().Get()
    for n in web.GetFaceVertexCountsAttr().Get():
        face=[coords[j] for j in indices[cursor:cursor+n]]
        cursor+=n
        if n==3 and all(abs(p[2]-OFFSET)<1e-6 for p in face):
            result.append([(p[0],p[1]) for p in reversed(face)])
    if not result:
        raise ValueError("CN7 wheel mounting face is missing")
    if CONFIG["physics"]["merge_web_colliders"]:
        result=merge_convex(result)
    return result


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


@dataclass
class Station:
    """One wheel corner, method 1: the tire arrives flat on the station conveyor, the pick
    zone (V-stop, photo-eyes, centering arms, lift bars) presents it to the mount robot's
    gripper. Insertion is the robot's job (control comes later); the station measures the
    wheel against its hub so that control can use it."""
    name: str
    side: int
    hub: Gf.Vec3d                 # hub centre in the car frame (design position)
    wheel: str
    initial: float                # tire angle as it was put on the conveyor
    hub_path: str
    tire: str
    zone: object
    sensor: object
    face_path: str
    phase: str = "arriving"
    timer: float = 0.
    now: float = 0.
    error: float = 0.             # wheel bolt pattern to hub studs, deg (camera when it sees it)
    error_true: float = 0.
    gap: float = 0.               # wheel mounting face to hub face along the hub axis, m
    center_error: float = 0.
    face_contact: bool = False
    face_contact_now: bool = False
    belt_contact: bool = False
    stop_contact: bool = False
    guide_contact: bool = False
    conveyed_distance: float = 0.
    max_lateral_travel: float = 0.
    interlocked: bool = False

    @property
    def stopper(self):
        return self.zone.stopper_path

    def step(self, api, dt):
        self.timer+=dt
        self.now+=dt
        window=api.get_rigidbody_transformation(self.wheel)
        hub=api.get_rigidbody_transformation(self.hub_path)
        if not (window.get("ret_val") and hub.get("ret_val")):
            return
        p=window["position"]
        self.conveyed_distance=max(self.conveyed_distance,self.hub[0]+START_AHEAD-p[0])
        if self.phase=="arriving":
            self.max_lateral_travel=max(self.max_lateral_travel,abs(p[1]-self.side*LINE_Y))
        hub_inv=Gf.Rotation(quaternion(hub)).GetInverse()
        in_hub=hub_inv.TransformDir(Gf.Rotation(quaternion(window)).TransformDir(Gf.Vec3d(1,0,0)))
        self.error_true=(degrees(atan2(in_hub[1],in_hub[0]))+36.)%72.-36.
        rel=hub_inv.TransformDir(Gf.Vec3d(*p)-Gf.Vec3d(*hub["position"]))
        self.gap=rel[2]
        self.center_error=(rel[0]**2+rel[1]**2)**.5
        self.sensor.update(self.now,self.error_true)
        measured=self.sensor.value(self.now)
        self.error=measured if measured is not None else self.error_true
        if self.phase not in TERMINAL_PHASES:
            self.phase=self.zone.step(dt)


def make_station(stage, car, name, seed, sections, mats, run_seed=0):
    from corner import Sensor, build_corner, ring_tire
    from handling import PickZone
    axle,side=STATIONS[name]
    hub_y=side*half_track(axle)
    axle_q=rotate((1,0,0),-side*90).GetQuat()
    hub_xform=transform(stage,str(car.GetPath())+"/Contact_"+name,(axle,hub_y,HUB_Z),axle_q)
    hp=str(hub_xform.GetPath())
    # Knuckle on the suspension, hub on its bearing (random parked angle, drag, play, tolerances).
    car_pose=UsdGeom.Xformable(car).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    hub_path,_,_=build_corner(stage,car_pose,ROOT,name,axle,side,hub_y,mats[0],run_seed)
    # Sectioned arch proxy shares the CAD liner's radial datum.
    for i in range(10):
        a=-side*i*tau/20
        b=-side*(i+1)*tau/20
        outline=[(r*cos(t),r*sin(t)) for r,t in
                 ((ARCH_RADIUS,a),(ARCH_RADIUS+.025,a),(ARCH_RADIUS+.025,b),(ARCH_RADIUS,b))]
        hull(stage,hp+f"/Wheelhouse_{i}",outline,-.115,.11,False)
    wp=ROOT+"/"+name
    guide=material(stage,"GuideUHMW",GUIDE_STATIC_FRICTION,GUIDE_DYNAMIC_FRICTION,combine="min")
    zone=PickZone(stage,wp,axle,side,mats[2],guide)
    # Rim body (mounting face colliders) and the tire's tread ring sprung to it; flat on the feed.
    delivery=(axle+START_AHEAD,side*LINE_Y,BELT_Z+.125)
    wheel_q=rotate((0,0,1),seed).GetQuat()
    position=body(stage,wp+"/Wheel",delivery,wheel_q,RIM_MASS)
    mass_properties(position,(0.,0.,0.),annulus_inertia(RIM_MASS,.095,CN7_R16.inner+.012,CN7_R16.rim_width))
    PhysxSchema.PhysxRigidBodyAPI(position).CreateEnableCCDAttr(True)
    wheel_path=str(position.GetPath())
    PhysxSchema.PhysxContactReportAPI.Apply(position).CreateThresholdAttr(0.)
    reference(stage,wheel_path+"/Visual","/AssetLibrary/Wheel_16")
    for i,points in enumerate(sections):
        hull(stage,wheel_path+f"/Web_{i}",points,OFFSET,OFFSET+.014,True,mats[0])
    # Tire envelope with a crowned tread and rounded shoulders at the 205 mm sidewalls.
    radius=CN7_R16.radius
    half_width=CN7_R16.width/2
    profile=((.216,-TREAD_HALF),(.270,-half_width),(.290,-TREAD_HALF),
             (radius-.004,-.081),(radius,-.062),(radius,.062),
             (radius-.004,.081),(.290,TREAD_HALF),(.270,half_width),(.216,TREAD_HALF))

    def tread(base):
        for i in range(32):
            a=i*tau/32
            b=(i+1)*tau/32
            points=[Gf.Vec3f(r*cos(t),r*sin(t),z) for t in (a,b) for r,z in profile]
            convex_prism(stage,base+f"/Tread_{i}",points,len(profile),False,mats[1])
    tire=ring_tire(stage,wheel_path,wp+"/Tire",delivery,wheel_q,mats,tread)
    sensing=CONFIG["sensing"]
    sensor=Sensor(Random(f"{run_seed}-{name}-sensing"),sensing["wheel_angle_noise_deg"])
    return Station(name,side,Gf.Vec3d(axle,hub_y,HUB_Z),wheel_path,seed,hub_path,tire,zone,sensor,
                   hub_path+"/Face")


class Robot:
    def __init__(self, stage, source):
        self.stage=stage
        self.source=source
        self.paused_targets=False
        source_prim=stage.GetPrimAtPath(source)
        model=source_prim.GetAttribute("cell:model").Get().lower()
        tree=ET.parse(Path(__file__).parent/"assets"/(model+".urdf")).getroot()
        self.path=ROOT+"/"+source_prim.GetName()
        self.clock=0.
        self.sample=SOURCE_POSE
        self.detour=None
        self.holds=[(seat-GUIDE_LEAD,seat,axle) for seat,axle in ((FRONT_SEAT,"F"),(REAR_SEAT,"R"))
                    ] if "Mount" in source else []
        transform(stage,self.path)
        carriage,_=pose(stage,source,SOURCE_POSE)
        base=body(stage,self.path+"/RailCarriage",carriage,None,115.)
        PhysxSchema.PhysxContactReportAPI.Apply(base).CreateThresholdAttr(0.)
        UsdPhysics.ArticulationRootAPI.Apply(base)
        PhysxSchema.PhysxArticulationAPI.Apply(base).CreateEnabledSelfCollisionsAttr(True)
        rail=constrain(UsdPhysics.PrismaticJoint.Define(stage,self.path+"/Rail"),None,base)
        rail.CreateAxisAttr("X")
        rail.CreateLowerLimitAttr(-3.64)
        rail.CreateUpperLimitAttr(3.64)
        rail.CreateExcludeFromArticulationAttr(True)
        self.rail=drive(rail,"linear",carriage[0],42000.,1650.,3800.)
        reference(stage,self.path+"/RailCarriage/Base",source+"/Base")
        reference(stage,self.path+"/RailCarriage/Pedestal",source+"/Pedestal")
        reference(stage,self.path+"/RailCarriage/Carriage",source+"/Carriage")
        side=1 if "Left" in self.path else -1
        carriage_center=(0.,side*(RAIL_Y+.20),.358)
        carriage_size=(.59,.26,.074)
        cube(stage,self.path+"/RailCarriage/Shape",carriage_center,carriage_size)
        mount=UsdGeom.Xformable(stage.GetPrimAtPath(self.path+"/RailCarriage/Base")).GetLocalTransformation()
        base_kg,base_com,base_tensor=urdf_properties(tree.find("./link[@name='base_link']/inertial"))
        mounted_com=mount.Transform(Gf.Vec3d(*base_com))
        combined_properties(base,[(115.,np.array(carriage_center),np.diag(box_inertia(115.,carriage_size))),
                                  (base_kg,np.array(mounted_com),base_tensor)])
        self.add_collision(stage,model,0,self.path+"/RailCarriage/BaseCollision",mount)
        first=tree.find("./joint[@name='joint_1']/origin")
        first_xyz=Gf.Vec3d(*map(float,first.get("xyz").split()))
        shoulder,_=pose(stage,source+"/Joint_1",SOURCE_POSE)
        # The rail assembly base frame is at the floor, below the robot mounting flange.
        base_offset=shoulder-carriage-first_xyz
        self.drives=[]
        parent=base
        for i in range(1,7):
            key=source+f"/Joint_{i}"
            source_link=stage.GetPrimAtPath(key)
            p,q=pose(stage,key,SOURCE_POSE)
            link=tree.find(f"./link[@name='link_{i}']")
            mass,center,tensor=urdf_properties(link.find("inertial"))
            actor=body(stage,self.path+f"/Link_{i}",p,q,mass)
            PhysxSchema.PhysxContactReportAPI.Apply(actor).CreateThresholdAttr(0.)
            for child in source_link.GetChildren():
                # Mount robots carry the method 1 gripper (gripper.py), not the replay's guide tool.
                if not child.GetName().startswith("CarriedNut") and not (
                        "Mount" in self.path and child.GetName().startswith("Guide")):
                    reference(stage,str(actor.GetPath())+"/"+child.GetName(),str(child.GetPath()))
            self.add_collision(stage,model,i,str(actor.GetPath())+"/Collision")
            if i==6:
                camera=stage.GetPrimAtPath(str(actor.GetPath())+"/D455Assembly")
                bounds=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_],False,True)
                camera_box=bounds.ComputeRelativeBound(camera,actor).ComputeAlignedRange()
                camera_center=camera_box.GetMidpoint()
                cube(stage,str(actor.GetPath())+"/CameraHousing",camera_center,camera_box.GetSize())
                combined_properties(actor,[(mass,center,tensor),(CAMERA_MASS,np.array(camera_center),
                                         np.diag(box_inertia(CAMERA_MASS,CAMERA_SIZE)))])
                for part in actor.GetChildren():
                    if part.GetName().startswith(("GuideAdapter","GuideGripper","GuideJaw","Nutrunner","GearHead","Socket")):
                        collider(part)
            else:
                tensor_properties(actor,center,tensor)
            definition=tree.find(f"./joint[@name='joint_{i}']")
            xyz=Gf.Vec3d(*map(float,definition.find("origin").get("xyz").split()))
            if i==1:
                xyz+=base_offset
            roll,pitch,yaw=map(degrees,map(float,definition.find("origin").get("rpy").split()))
            rpy=rotate((1,0,0),roll)*rotate((0,1,0),pitch)*rotate((0,0,1),yaw)
            hinge=constrain(UsdPhysics.RevoluteJoint.Define(stage,self.path+f"/Joint_{i}"),
                            parent,actor,xyz,rot0=rpy.GetQuat())
            hinge.CreateAxisAttr("Z")
            limits=definition.find("limit")
            hinge.CreateLowerLimitAttr(degrees(float(limits.get("lower"))))
            hinge.CreateUpperLimitAttr(degrees(float(limits.get("upper"))))
            joint_data=PhysxSchema.PhysxJointAPI.Apply(hinge.GetPrim())
            joint_data.CreateMaxJointVelocityAttr(degrees(float(limits.get("velocity"))))
            joint_data.CreateJointFrictionAttr(float(definition.find("dynamics").get("friction","0")))
            source_angle=source_link.GetAttribute("cell:jointTargetDeg")
            target=source_angle.Get(SOURCE_POSE*stage.GetTimeCodesPerSecond())
            if target is None:
                raise ValueError("Rebuild the CN7 stage with build.sh to generate articulation targets.")
            PhysxSchema.JointStateAPI.Apply(hinge.GetPrim(),"angular").CreatePositionAttr(target)
            attr=drive(hinge,"angular",target,550.,16.,float(limits.get("effort")))
            self.drives.append((attr,source_angle))
            parent=actor

    @staticmethod
    def add_collision(stage,model,index,path,placement=None):
        source=f"/AssetLibrary/{model}_Collision_{index}"
        if not stage.GetPrimAtPath(source):
            raise ValueError("Rebuild the stage with the Doosan URDF collision meshes.")
        root=reference(stage,path,source)
        if placement is not None:
            UsdGeom.Xformable(root).MakeMatrixXform().Set(placement)
        for part in root.GetChildren():
            UsdPhysics.MeshCollisionAPI.Apply(part).CreateApproximationAttr("convexHull")
            collision(UsdGeom.Mesh(part))

    def hold(self,api):
        self.paused_targets=True
        frame=api.get_rigidbody_transformation(self.path+"/RailCarriage")
        if frame.get("ret_val"):
            self.rail.Set(frame["position"][0])
        for target,_ in self.drives:
            joint=UsdPhysics.RevoluteJoint(target.GetPrim())
            paths=[rel.GetTargets()[0] for rel in (joint.GetBody0Rel(),joint.GetBody1Rel())]
            parent,child=[api.get_rigidbody_transformation(str(path)) for path in paths]
            if not (parent.get("ret_val") and child.get("ret_val")):
                continue
            parent_rotation=Gf.Rotation(quaternion(parent))
            child_rotation=Gf.Rotation(quaternion(child))
            rest=joint.GetLocalRot0Attr().Get()
            datum=Gf.Rotation(Gf.Quatd(rest.GetReal(),Gf.Vec3d(rest.GetImaginary())))
            relative=child_rotation*parent_rotation.GetInverse()*datum.GetInverse()
            axis=relative.TransformDir(Gf.Vec3d(1,0,0))
            measured=degrees(atan2(axis[1],axis[0]))
            target.Set(measured+360*round((target.Get()-measured)/360))

    def step(self, seconds):
        if self.paused_targets:
            return
        sample=SOURCE_POSE+seconds
        if "Fasten" in self.source:
            # Clearance posture while the wheels are still held by the insertion fixtures.
            sample=min(sample,54.)
        self.sample=sample
        self.rail.Set(pose(self.stage,self.source,sample)[0][0])
        time=sample*self.stage.GetTimeCodesPerSecond()
        for attr, source in self.drives:
            attr.Set(source.Get(time))

    def blend(self, a, b, fraction):
        self.rail.Set(pose(self.stage,self.source,a)[0][0]*(1-fraction)+pose(self.stage,self.source,b)[0][0]*fraction)
        rate=self.stage.GetTimeCodesPerSecond()
        for attr, source in self.drives:
            attr.Set(source.Get(a*rate)*(1-fraction)+source.Get(b*rate)*fraction)

    def advance(self, dt, running, finished):
        """Follow the replay keyframes on a process clock.

        In the replay the mount robot carries the wheel onto the hub; live, the station
        shuttle presses it. The mount robot therefore waits at the wheel-side standby
        pose until finished(axle) reports that axle's insertion over, then withdraws in
        joint space to the replay's post-seating safe pose instead of following the
        wheel inward ahead of the press.
        """
        if self.paused_targets:
            return
        if not running:
            self.step(self.clock)
            return
        if self.detour is not None:
            start,end,elapsed=self.detour
            elapsed+=dt
            if elapsed<GUIDE_RETREAT:
                self.detour=(start,end,elapsed)
                fraction=elapsed/GUIDE_RETREAT
                self.blend(start,end,fraction*fraction*(3-2*fraction))
                return
            self.detour=None
            self.clock=end-SOURCE_POSE
        if self.holds and self.clock+dt>=self.holds[0][0]:
            hold,seat,axle=self.holds[0]
            self.clock=hold
            if finished(axle):
                self.holds.pop(0)
                self.detour=(SOURCE_POSE+hold,SOURCE_POSE+seat+GUIDE_EXIT,0.)
                self.sample=None
                return
            self.step(self.clock)
            return
        self.clock+=dt
        self.step(self.clock)


class Carrier:
    """Force-limited equivalent traction for a low-speed omniMove platform."""

    def __init__(self,stage,deck,lift):
        self.path=str(deck.GetPath())
        self.lift_path=str(lift.GetPath())
        self.start=Gf.Vec3d(UsdGeom.Xformable(deck).GetOrderedXformOps()[0].Get())
        self.waypoints=(Gf.Vec3d(self.start[0],0.,0.),Gf.Vec3d(0.))
        self.segment=0
        self.previous=self.start
        self.total_mass=AMR_MASS+AMR_LIFT_MASS+BODY_MASS
        self.docked=False
        self.ready=False
        self.lift_target=0.
        self.clamp_target=0.
        self.clamps=[]
        self.clamp_contacts=set()
        self.grounded=False
        self.velocity=Gf.Vec3d(0.)
        traction=PhysxSchema.PhysxForceAPI.Apply(deck)
        PhysxSchema.PhysxContactReportAPI.Apply(deck).CreateThresholdAttr(0.)
        traction.CreateWorldFrameEnabledAttr(True)
        traction.CreateModeAttr("force")
        self.force=traction.CreateForceAttr(Gf.Vec3f(0.))
        self.yaw_torque=traction.CreateTorqueAttr(Gf.Vec3f(0.))
        self.previous_yaw=0.
        support=material(stage,"AMRSupport",0.,0.,combine="min")
        for i,x in enumerate((-.72,.72)):
            for j,y in enumerate((-.31,.31)):
                item=UsdGeom.Sphere.Define(stage,self.path+f"/Support_{i}_{j}")
                item.CreateRadiusAttr(AMR_WHEEL_RADIUS)
                item.AddTranslateOp().Set(Gf.Vec3d(x,y,AMR_WHEEL_RADIUS))
                collider(item.GetPrim(),support)
                item.CreateVisibilityAttr("invisible")
        cube(stage,self.path+"/Collision",(0.,0.,.28),(AMR_SIZE[0],AMR_SIZE[1],.33))
        joint=constrain(UsdPhysics.PrismaticJoint.Define(stage,ROOT+"/VehicleLift"),deck,lift,(0.,0.,-.12))
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(-.02)
        joint.CreateUpperLimitAttr(.14)
        self.stiffness=1600000.
        self.lift_load=(AMR_LIFT_MASS+BODY_MASS)*9.81
        self.lift_drive=drive(joint,"linear",self.lift_load/self.stiffness,self.stiffness,
                              servo_damping(self.stiffness,AMR_LIFT_MASS+BODY_MASS),
                              (AMR_PAYLOAD+AMR_LIFT_MASS)*9.81)

    def add_clamp(self,stage,path,source):
        base=reference(stage,path+"/Base",source+"/Base")
        collider(base)
        start,orientation=pose(stage,source+"/Slide",0.)
        slide=body(stage,path+"/Slide",start,orientation,3.)
        PhysxSchema.PhysxContactReportAPI.Apply(slide).CreateThresholdAttr(0.)
        mass_properties(slide,(0.,0.,.22),box_inertia(3.,(.13,.09,.105)))
        visual=reference(stage,path+"/Slide/Visual",source+"/Slide",True)
        for part in visual.GetChildren():
            if part.GetName() in ("Jaw","Pad"):
                collider(part)
        joint=constrain(UsdPhysics.PrismaticJoint.Define(stage,path+"/Travel"),None,slide,start)
        joint.CreateAxisAttr("Y")
        span=DOCK_CLAMP_OPEN-DOCK_CLAMP_CLOSED
        side=1 if start[1]>0 else -1
        joint.CreateLowerLimitAttr(-span if side>0 else 0.)
        joint.CreateUpperLimitAttr(0. if side>0 else span)
        target=drive(joint,"linear",0.,12000.,servo_damping(12000.,3.),400.)
        self.clamps.append((str(slide.GetPath()),side,target))

    def step(self,api,dt):
        state=api.get_rigidbody_transformation(self.path)
        if not state.get("ret_val"):
            return
        point=Gf.Vec3d(*state["position"])
        measured=(point-self.previous)/dt
        self.previous=point
        velocity=Gf.Vec3d(measured[0],measured[1],0.)
        remaining=self.waypoints[self.segment]-point
        distance=Gf.Vec3d(remaining[0],remaining[1],0.).GetLength()
        if distance<.006 and velocity.GetLength()<.025:
            if self.segment==0:
                self.segment=1
                remaining=self.waypoints[1]-point
            elif not self.docked:
                self.docked=True
        direction=Gf.Vec3d(remaining[0],remaining[1],0.)
        if self.docked:
            v_goal=direction*2.
        else:
            speed_limit=AMR_SPEED_Y if self.segment==0 else AMR_SPEED_X
            speed=min(speed_limit,sqrt(2*AMR_ACCELERATION*direction.GetLength()))
            v_goal=direction.GetNormalized()*speed
        delta=v_goal-self.velocity
        if delta.GetLength()>AMR_ACCELERATION*dt:
            delta=delta.GetNormalized()*(AMR_ACCELERATION*dt)
        self.velocity+=delta
        force=(self.velocity-velocity)*(self.total_mass*6.)
        limit=AMR_TRACTION_COEFFICIENT*self.total_mass*9.81
        if force.GetLength()>limit:
            force=force.GetNormalized()*limit
        self.force.Set(vec(force) if self.grounded else Gf.Vec3f(0.))
        front=Gf.Rotation(quaternion(state)).TransformDir(Gf.Vec3d(1,0,0))
        yaw=atan2(front[1],front[0])
        target_torque=-3600*yaw-900*(yaw-self.previous_yaw)/dt
        bounded=max(-limit*.31,min(limit*.31,target_torque)) if self.grounded else 0.
        self.yaw_torque.Set(Gf.Vec3f(0.,0.,bounded))
        self.previous_yaw=yaw
        if self.docked:
            close=DOCK_CLAMP_OPEN-DOCK_CLAMP_CLOSED
            self.clamp_target=min(close,self.clamp_target+.12*dt)
            for _,side,target in self.clamps:
                target.Set(-side*self.clamp_target)
            clamped=len(self.clamp_contacts)==len(self.clamps)
            if clamped:
                self.lift_target=min(.12,self.lift_target+AMR_LIFT_SPEED*dt)
                self.lift_drive.Set(self.lift_target+self.lift_load/self.stiffness)
            lift=api.get_rigidbody_transformation(self.lift_path)
            self.ready=(clamped and lift.get("ret_val") and self.lift_target>=.12 and
                        abs(lift["position"][2]-point[2])<.006)

    def author_inbound_drives(self, stage):
        """Supply native USD Play with force-limited planar servos for inbound travel."""
        deck=stage.GetPrimAtPath(self.path)
        # The drive frame is a world reference. There are no locked degrees of
        # freedom: all contacts, gravity, and deflection under load are solved.
        joint=constrain(UsdPhysics.Joint.Define(stage,ROOT+"/InboundTraction"),None,deck,self.start)
        stiffness=self.total_mass*12.
        damping=servo_damping(stiffness,self.total_mass,1.)
        max_force=self.total_mass*AMR_ACCELERATION
        drive(joint,"rotZ",0.,80.,30.,max_force*.31)
        rate=stage.GetTimeCodesPerSecond()
        begins=0.
        for axis,distance,speed in (("transY",-self.start[1],AMR_SPEED_Y),
                                    ("transX",-self.start[0],AMR_SPEED_X)):
            target=drive(joint,axis,0.,stiffness,damping,max_force)
            target.Set(0.,0.)
            target.Set(0.,rate*begins)
            direction=1. if distance>=0. else -1.
            length=abs(distance)
            speed=min(speed*.9,sqrt(length*AMR_ACCELERATION))
            if not speed:
                continue
            ramp=speed/AMR_ACCELERATION
            cruise_end=length/speed
            duration=cruise_end+ramp
            for frame in range(1,int(duration*rate)+1):
                t=frame/rate
                if t<=ramp:
                    travel=.5*AMR_ACCELERATION*t*t
                elif t<=cruise_end:
                    travel=speed*(t-.5*ramp)
                else:
                    travel=length-.5*AMR_ACCELERATION*(duration-t)**2
                target.Set(direction*travel,rate*begins+frame)
            target.Set(distance,rate*(begins+duration))
            begins+=duration+2.  # Lateral braking and settling before the longitudinal leg.


class Cell:
    def __init__(self, stage, seed, robots=True, branch_side=1, *, manual=True):
        import omni.physx

        self.stage=stage
        self.api=omni.physx.get_physx_interface()
        self.sim=omni.physx.get_physx_simulation_interface()
        self.seconds=0.
        self.timer=None                    # perf.StepTimer, set by the caller to profile each step
        self._collider_paths={}
        self._robot_by_collider={}
        self.dock_ready_time=None
        self.pending=[]
        self.interference=[]
        self.min_rail_spacing=float("inf")
        transform(stage,ROOT)
        UsdGeom.Scope.Define(stage,"/World/Orders").CreateVisibilityAttr().Set("invisible")
        scene=UsdPhysics.Scene.Define(stage,ROOT+"/Physics")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
        scene.CreateGravityMagnitudeAttr(9.81)
        options=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        options.CreateEnableGPUDynamicsAttr(False)
        options.CreateBroadphaseTypeAttr("MBP")
        options.CreateTimeStepsPerSecondAttr(HZ)
        options.CreateEnableCCDAttr(True)
        cube(stage,ROOT+"/Floor",(0,0,-.055),(20,15,.11))
        # Keep the first car on the AMR at its low inbound datum until the dock closes.
        source_amr=REFERENCE+"/Carrier"
        source_lift=source_amr+"/Hanger"
        deck_start,_=pose(stage,source_amr,0.)
        car_start,_=pose(stage,source_lift,0.)
        deck=body(stage,ROOT+"/DockedAMR",deck_start,None,AMR_MASS)
        mass_properties(deck,(0.,0.,.23),box_inertia(AMR_MASS,AMR_SIZE))
        lift=body(stage,ROOT+"/DockLift",car_start,None,AMR_LIFT_MASS)
        mass_properties(lift,(0.,0.,.58),box_inertia(AMR_LIFT_MASS,(3.52,.88,.66)))
        self.carrier=Carrier(stage,deck,lift)
        reference(stage,ROOT+"/DockedAMR/Visual",REFERENCE+"/Carrier/AMR")
        for axle in range(2):
            for side in (0,2):
                clamp=f"/DockClamp_{axle}_{side}"
                self.carrier.add_clamp(stage,ROOT+clamp,REFERENCE+clamp)
        car=body(stage,ROOT+"/CN7",car_start,None,BODY_MASS)
        # The replay hierarchy contains wheel/nut animation, not just the car shell.
        # Copy only static car parts so timeline scrubbing cannot reveal extra wheels.
        visual=transform(stage,ROOT+"/CN7/Visual")
        for child in stage.GetPrimAtPath(source_lift).GetChildren():
            if child.GetName().startswith(("Support_","Crossmember_","ChassisPad_","SaddleRail_")):
                reference(stage,ROOT+"/DockLift/"+child.GetName(),str(child.GetPath()))
            elif not child.GetName().startswith("Wheel_"):
                target=str(visual.GetPath())+"/"+child.GetName()
                reference(stage,target,str(child.GetPath()))
                if child.GetName() in STATIONS:
                    for item in child.GetChildren():
                        if item.GetName().startswith("Nut_"):
                            stage.OverridePrim(target+"/"+item.GetName()).SetActive(False)
        cube(stage,ROOT+"/CN7/Undertray",(0,0,1.10),(3.7,1.16,.23))
        cube(stage,ROOT+"/CN7/Body",(0,0,1.55),(4.56,1.57,.36))
        sections=collision_web(stage)
        mats=(material(stage,"Steel",.27),material(stage,"Rubber",.82),material(stage,"RollerGrip",.62),
              material(stage,"GripPad",PAD_STATIC_FRICTION,PAD_DYNAMIC_FRICTION,combine="min"),
              # Cradle support rollers are free idlers; model their rolling resistance only.
              material(stage,"IdlerRoller",.02,.02,combine="min"))
        for ax in (-1,1):
            for side in (-1,1):
                pad=cube(stage,f"{ROOT}/DockLift/LoadPad_{ax+1}_{side+1}",(ax*1.65,side*.40,.927),
                         (.16,.10,.116))
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(pad)).Bind(mats[1],materialPurpose="physics")
        for side in (-1,1):
            cube(stage,f"{ROOT}/DockLift/SideLocator_{side+1}",(0,side*.615,1.01),(.20,.06,.06))
            cube(stage,f"{ROOT}/DockLift/EndLocator_{side+1}",(side*1.885,0,1.01),(.06,.20,.06))
        guide=material(stage,"GuideUHMW",GUIDE_STATIC_FRICTION,GUIDE_DYNAMIC_FRICTION,combine="min")
        setup_factory(stage,ROOT,mats[2],guide,branch_side)
        rng=Random(seed)
        self.stations=[make_station(stage,car,name,rng.uniform(-180.,180.),sections,mats,seed) for name in STATIONS]
        self.robots=[Robot(stage,REFERENCE+"/"+prefix+role)
                for prefix in ("Left","Right") for role in ("Mount","Fasten")] if robots else []
        self.robots_by_path={robot.path:robot for robot in self.robots}
        self.rail_pairs=list(zip(self.robots[::2],self.robots[1::2]))
        self.pair_by_robot={robot.path:pair for pair in self.rail_pairs for robot in pair}
        self.by_contact_face={station.face_path:station for station in self.stations}
        self.by_wheel={station.wheel:station for station in self.stations}
        self.by_wheel.update({station.tire:station for station in self.stations})
        # Method 1 gripper on each mount robot's flange (Link_6).
        from gripper import WheelGripper
        self.grippers={}
        grip=material(stage,"GripperRoller",.8)
        for robot in self.robots:
            if "Mount" in robot.path:
                flange=UsdGeom.Xformable(stage.GetPrimAtPath(robot.path+"/Link_6")).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                self.grippers[robot.path]=WheelGripper(stage,robot.path,flange,grip)
        # Pick-zone moving parts: anything but their own tire touching them is reported.
        self.zone_bodies=[path for station in self.stations for path in station.zone.contact_bodies]
        self.zone_contacts=set()
        self.stage_id=None
        if manual:
            self.stage_id=UsdUtils.StageCache.Get().Insert(stage)
            self.sim.attach_stage(self.stage_id.ToLongInt())

    def export_setup(self, path):
        compiled=Usd.Stage.Open(self.stage.Flatten(False))
        compiled.RemovePrim("/World/Orders")
        self.carrier.author_inbound_drives(compiled)
        texture=UsdShade.Shader.Get(compiled,"/Looks/Concrete/Texture")
        if texture:
            texture.GetInput("file").Set(Sdf.AssetPath("./concrete.png"))
        return compiled.GetRootLayer().Export(str(path))

    def robot_clearance(self):
        for mount,fasten in self.rail_pairs:
            paths=[r.path+"/RailCarriage" for r in (mount,fasten)]
            poses=[self.api.get_rigidbody_transformation(p) for p in paths]
            if mount.rail.Get()-fasten.rail.Get()<MIN_RAIL_SPACING:
                self.stop_pair(mount,fasten,"rail_command")
            if not all(p.get("ret_val") for p in poses):
                continue
            spacing=poses[0]["position"][0]-poses[1]["position"][0]
            self.min_rail_spacing=min(self.min_rail_spacing,spacing)
            if spacing<MIN_RAIL_SPACING:
                self.stop_pair(mount,fasten,"rail_clearance")

    def stop_pair(self,mount,fasten,reason):
        if mount.paused_targets and fasten.paused_targets:
            return
        self.interference.append((self.seconds,mount.path,fasten.path,reason))
        self.pending.append((self.seconds,mount.path.rsplit("/",1)[1],reason))
        for robot in (mount,fasten):
            robot.hold(self.api)
        for station in self.stations:
            if any((station.side>0)==("Left" in r.path) for r in (mount,fasten)) and station.phase not in TERMINAL_PHASES:
                station.press.Set(station.gap-station.initial_gap)
                station.phase="interference"

    def section(self,name):
        return self.timer.section(name) if self.timer else nullcontext()

    def _collider(self,encoded):
        # intToSdfPath + str() for every contact header cost ~2 ms per step; paths never change.
        path=self._collider_paths.get(encoded)
        if path is None:
            path=self._collider_paths[encoded]=str(PhysicsSchemaTools.intToSdfPath(encoded))
        return path

    def _robot_of(self,path):
        robot=self._robot_by_collider.get(path,False)
        if robot is False:
            robot=self._robot_by_collider[path]=self.robots_by_path.get("/".join(path.split("/")[:4]))
        return robot

    def prepare_step(self,dt):
        with self.section("prepare"):
            self._prepare_step(dt)

    def _prepare_step(self,dt):
        self.carrier.step(self.api,dt)
        if self.carrier.ready and self.dock_ready_time is None:
            self.dock_ready_time=self.seconds
            self.pending.append((self.seconds,"AMR","docked"))
        if round(self.seconds*HZ)%4==0:
            # Robot motion is not part of the environment: the arms hold their start pose
            # until robot control is added.
            for robot in self.robots:
                robot.advance(4*dt,False,lambda axle:True)
        self.robot_clearance()
        fault=next((station for station in self.stations if station.phase in INTERLOCK_FAULTS),None)
        for station in self.stations:
            if fault and station is not fault and not station.interlocked:
                station.interlocked=True
                self.pending.append((self.seconds,station.name,f"interlock_by_{fault.name}_{fault.phase}"))
            old=station.phase
            station.step(self.api,dt)
            station.face_contact_now=False
            if station.phase!=old:
                self.pending.append((self.seconds,station.name,station.phase))

    def finish_step(self,dt):
        with self.section("finish"):
            self._finish_step(dt)
        if self.timer:
            self.timer.count_step()

    def _finish_step(self,dt):
        headers,contacts=self.sim.get_contact_report()[:2]
        self.carrier.grounded=False
        self.carrier.clamp_contacts.clear()
        for header in headers:
            if not header.num_contact_data:
                continue
            collider0,collider1=self._collider(header.collider0),self._collider(header.collider1)
            pair=[self._robot_of(path) for path in (collider0,collider1)]
            if all(pair) and pair[0]!=pair[1]:
                self.stop_pair(*pair,"robot_contact")
            elif bool(pair[0])!=bool(pair[1]):
                robot=pair[0] or pair[1]
                first=header.contact_data_offset
                if any(contacts[i].separation<-.001 for i in range(first,first+header.num_contact_data)):
                    if os.environ.get("CELL_DEBUG"):
                        print("CONTACTDBG",round(self.seconds,3),collider0,collider1,
                              min(contacts[i].separation for i in range(first,first+header.num_contact_data)),
                              [round(v,3) for v in contacts[first].position],"normal",[round(v,3) for v in contacts[first].normal],
                              "sample",robot.sample,"clock",round(robot.clock,2),"detour",robot.detour,flush=True)
                    self.stop_pair(*self.pair_by_robot[robot.path],"equipment_contact")
            for face,other in ((collider0,collider1),(collider1,collider0)):
                if face.startswith(self.carrier.path+"/Support_") and other in (ROOT+"/Floor","/World/Factory/DockPlate"):
                    self.carrier.grounded=True
                if face.startswith(ROOT+"/DockClamp_") and face.endswith("/Pad") and other==self.carrier.path+"/Collision":
                    self.carrier.clamp_contacts.add(face)
                station=self.by_contact_face.get(face)
                if station and other.startswith(station.wheel+"/Web_"):
                    station.face_contact=True
                    station.face_contact_now=True
                zone=next((z for z in self.zone_bodies if face.startswith(z+"/")),None)
                if zone and not other.startswith(zone.rsplit("/",1)[0]+"/Wheel") \
                        and not other.startswith(zone.rsplit("/",1)[0]+"/Tire"):
                    self.zone_contacts.add((zone[len(ROOT)+1:],other[len(ROOT)+1:] if other.startswith(ROOT) else other))
                parent=other.rsplit("/",1)[0]
                unit=self.by_wheel.get(parent)
                if unit:
                    if "/Conveyor/" in face and "Guide_" in face:
                        unit.guide_contact=True
                    if face.startswith(unit.stopper+"/"):
                        unit.stop_contact=True
                    elif "roller" in face.lower() or "/TransferBed/" in face:
                        unit.belt_contact=True
        self.seconds+=dt

    def step(self):
        dt=1/HZ
        self.prepare_step(dt)
        with self.section("simulate"):
            self.sim.simulate(dt,self.seconds)
            self.sim.fetch_results()
        self.finish_step(dt)

    @property
    def finished(self):
        return all(item.phase in TERMINAL_PHASES for item in self.stations)

    def close(self):
        if self.stage_id is not None:
            self.sim.detach_stage()
            UsdUtils.StageCache.Get().Erase(self.stage_id)
            self.stage_id=None
