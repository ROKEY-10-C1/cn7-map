"""Wheel station, method 1: pick zone, wheel (rim + ring tire) and its measurement against the hub."""

from dataclasses import dataclass
from math import atan2, cos, degrees, sin, tau
from random import Random

from pxr import Gf, PhysxSchema, Usd, UsdGeom

from conveyor import START_AHEAD, TREAD_HALF, quaternion
from fitment import ARCH_RADIUS, OFFSET, half_track
from physical_data import RIM_MASS, annulus_inertia
from physical_data import GUIDE_DYNAMIC_FRICTION, GUIDE_STATIC_FRICTION
from process import PICK_X, BELT_Z, CN7_R16, HUB_Z, LINE_Y, STATIONS
from cell_config import CONFIG
from sim_util import body, convex_prism, hull, mass_properties, material, reference, rotate, transform
from live_params import ROOT, TERMINAL_PHASES


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
            # The pick-zone timeout starts at its inlet, after the 1F–B1 transfer.
            if self.phase=="arriving" and p[0]>self.zone.axle+START_AHEAD:
                return
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
    zone=PickZone(stage,wp,PICK_X if axle>0 else axle,side,mats[2],guide)
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
