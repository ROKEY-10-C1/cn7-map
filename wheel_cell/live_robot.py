"""Doosan mount and fasten robots: URDF links and joints, replay following on the rail."""

from math import atan2, degrees
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from conveyor import collider, quaternion
from physical_data import CAMERA_MASS, CAMERA_SIZE, box_inertia
from process import RAIL_Y
from sim_util import body, collision, combined_properties, constrain, cube, drive, pose, reference, rotate, tensor_properties, transform, urdf_properties
from live_params import FRONT_SEAT, GUIDE_EXIT, GUIDE_LEAD, GUIDE_RETREAT, REAR_SEAT, ROOT, SOURCE_POSE


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
