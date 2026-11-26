"""AMR carrier: force-limited omniMove traction, lift and dock clamps."""

from math import atan2, sqrt

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from conveyor import collider, quaternion
from physical_data import BODY_MASS
from physical_data import box_inertia, servo_damping
from physical_data import AMR_ACCELERATION, AMR_LIFT_MASS, AMR_LIFT_SPEED, AMR_MASS, AMR_PAYLOAD, AMR_SIZE
from physical_data import AMR_SPEED_X, AMR_SPEED_Y, AMR_TRACTION_COEFFICIENT, AMR_WHEEL_RADIUS
from physical_data import DOCK_CLAMP_CLOSED, DOCK_CLAMP_OPEN
from sim_util import body, constrain, cube, drive, mass_properties, material, pose, reference, vec
from live_params import ROOT


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
