"""Check the live cell on CPU: supply, pick zone, AMR docking, hub/wheel contact, pose write-back."""

import argparse
import json
import os
from math import dist
from pathlib import Path
import sys

import omni.kit.app

# --config must be in the environment before the live modules read their parameters.
for _i, _arg in enumerate(sys.argv[:-1]):
    if _arg == "--config":
        os.environ["CELL_CONFIG"] = str(Path(sys.argv[_i+1]).resolve())
from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

sys.dont_write_bytecode = True
root = Path(__file__).resolve().parent
sys.path.insert(0, str(root))

from physical_data import AMR_LIFT_SPEED, AMR_SPEED_X, AMR_SPEED_Y, WHEEL_MASS
from physics import Cell, DOCK_READY, HZ, MIN_RAIL_SPACING, ROOT, STATIONS
from physics import constrain, drive, reference
from process import AMR_GATE_X, BELT_Z, LINE_Y


def count_colliders(stage):
    counts={}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            path=str(prim.GetPath())[len(ROOT)+1:]
            key=path.split("/")[0]
            key="wheel" if "/Wheel/" in path else "robot" if key.startswith(("Left","Right")) else key
            counts[key]=counts.get(key,0)+1
    counts["total"]=sum(counts.values())
    return counts


def check_actors(stage, robots, exported=False):
    orders=stage.GetPrimAtPath("/World/Orders")
    if exported:
        if orders:
            print("Unexpected replay actors in the physical USD",flush=True)
            return False
    elif UsdGeom.Imageable(orders).ComputeVisibility()!=UsdGeom.Tokens.invisible:
        print("Replay actors are visible in the live stage",flush=True)
        return False
    live=list(Usd.PrimRange(stage.GetPrimAtPath(ROOT)))
    wheels={str(prim.GetPath()) for prim in live if prim.GetName()=="Wheel" and prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    if wheels!={f"{ROOT}/{name}/Wheel" for name in STATIONS}:
        print("Unexpected wheel bodies",sorted(wheels),flush=True)
        return False
    for path in wheels:
        props=UsdPhysics.MassAPI(stage.GetPrimAtPath(path))
        tire=stage.GetPrimAtPath(path.rsplit("/",1)[0]+"/Tire")   # ring tire model carries the tread
        mass=props.GetMassAttr().Get()+(UsdPhysics.MassAPI(tire).GetMassAttr().Get() if tire else 0.)
        if abs(mass-WHEEL_MASS)>.001 or min(props.GetDiagonalInertiaAttr().Get())<=0.:
            return False
    if any(prim.GetName()=="TipGrip" for prim in live):
        return False
    car=stage.GetPrimAtPath(ROOT+"/CN7/Visual")
    duplicates=[str(prim.GetPath()) for prim in Usd.PrimRange(car) if prim.GetName().startswith(("Wheel_","Nut_"))]
    if duplicates:
        print("Unexpected animated car parts",duplicates,flush=True)
        return False
    carriages=[str(prim.GetPath()) for prim in live if prim.GetName()=="RailCarriage" and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(carriages)!=(4 if robots else 0):
        print("Unexpected robot carriages",carriages,flush=True)
        return False
    return True


def check_supply(stage,seed,side):
    cell=Cell(stage,seed,robots=False,branch_side=side,start="gate")
    test_path=ROOT+"/IncomingWheel"
    try:
        # A second tire from the truck infeed: rim and tread ring, held together for this probe.
        wheel=reference(stage,test_path,cell.stations[0].wheel)
        tire=reference(stage,test_path+"Tire",cell.stations[0].tire)
        for prim in (wheel,tire):
            UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(7.57,0.,BELT_Z+.125))
        constrain(UsdPhysics.FixedJoint.Define(stage,test_path+"Mount"),wheel,tire)
        for i in range(18*HZ):
            cell.step()
            p=cell.api.get_rigidbody_transformation(test_path)["position"]
            if i%HZ==0:
                print("INFEED",i/HZ,tuple(round(v,4) for v in p),flush=True)
            if p[0]<4.60 and abs(p[1]-side*LINE_Y)<.10 and .55<p[2]<.66:
                return True
        return False
    finally:
        cell.close()


def check_insertion(stage,seed,skew):
    """Bench: stationary car, FL wheel upright on a force-limited axial slide in front of its hub,
    pressed home at 76 mm/s / 145 N (the reach of a robot with a compliance unit). Nominal must
    seat on the face; skew (4 mm radial offset) must be stopped by the studs."""
    cell=Cell(stage,seed,robots=False,start="gate")
    try:
        unit=cell.stations[0]
        car=stage.GetPrimAtPath(ROOT+"/CN7")
        cache=UsdGeom.XformCache()
        before=cache.GetLocalToWorldTransform(car)
        UsdGeom.Xformable(car).GetOrderedXformOps()[0].Set(Gf.Vec3d(0.))
        UsdPhysics.RigidBodyAPI(car).GetKinematicEnabledAttr().Set(True)
        cache.Clear()
        moved=before.GetInverse()*cache.GetLocalToWorldTransform(car)
        # Knuckles and hubs are bodies of their own on joints to the car: move them with it.
        for station in cell.stations:
            for path in (station.hub_path.rsplit("/",1)[0]+"/Knuckle",station.hub_path):
                prim=stage.GetPrimAtPath(path)
                world=cache.GetLocalToWorldTransform(prim)*moved
                prim.GetAttribute("xformOp:translate").Set(world.ExtractTranslation())
                if prim.GetAttribute("xformOp:orient"):
                    prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(world.ExtractRotationQuat()))
        hub=stage.GetPrimAtPath(unit.hub_path)
        hub_q=hub.GetAttribute("xformOp:orient").Get()
        start=Gf.Vec3d(unit.hub[0]+(.004 if skew else 0.),unit.hub[1]+unit.side*.040,unit.hub[2])
        for path in (unit.wheel,unit.tire):
            prim=stage.GetPrimAtPath(path)
            prim.GetAttribute("xformOp:translate").Set(start)
            prim.GetAttribute("xformOp:orient").Set(hub_q)
        slide=constrain(UsdPhysics.PrismaticJoint.Define(stage,ROOT+"/FL/BenchSlide"),None,
                        stage.GetPrimAtPath(unit.wheel),start,rot0=Gf.Quatd(hub_q))
        slide.CreateAxisAttr("Z")
        press=drive(slide,"linear",0.,9000.,2300.,145.)
        touches=set()
        for i in range(9*HZ):
            press.Set(max(-.049,-.076*i/HZ))
            cell.sim.simulate(1/HZ,i/HZ)
            cell.sim.fetch_results()
            for header in cell.sim.get_contact_report()[0]:
                if not header.num_contact_data:
                    continue
                paths=[str(PhysicsSchemaTools.intToSdfPath(value)) for value in (header.collider0,header.collider1)]
                if any(path.startswith(unit.wheel+"/Web_") for path in paths):
                    touches.update(path.rsplit("/",1)[-1] for path in paths if path.startswith(unit.hub_path+"/"))
            unit.step(cell.api,1/HZ)
            if os.environ.get("BENCH_DEBUG") and i%24==0:
                print("BENCHDBG",round(i/HZ,2),round(unit.gap*1000,2),round(unit.error_true,3),round(unit.center_error*1000,3),flush=True)
        print("INSERTION_BENCH",round(unit.gap*1000,4),round(unit.error_true,3),sorted(touches),flush=True)
        if skew:
            return unit.gap>.005 and "Face" not in touches and any(name.startswith("Stud_") for name in touches)
        return abs(unit.gap)<.0007 and "Face" in touches
    finally:
        cell.close()


def check_buffer(stage,seed,slot,entry=None):
    """Inbound vehicle buffer only: the car stands on the four stands of `slot` (slots nearer the
    lane are empty, as after earlier pick-ups); the empty AMR + hanger crabs in under it at the
    entry height, lifts it off the stands to the carry height and crabs back to the lane over the
    empty stands. Nothing may touch the car or the stands except the hanger pads lifting it."""
    from vehicle import AISLE_X
    cell=Cell(stage,seed,robots=False,slot=slot)
    try:
        run=cell.buffer_run
        if entry is not None:
            run.entry=entry
        carried=[]
        for i in range(70*HZ):
            cell.step()
            z=cell.api.get_rigidbody_transformation(cell.car_path)["position"][2]
            if run.phase=="exit":
                carried.append(z)
            if os.environ.get("BUFFER_DEBUG") and i%HZ==0:
                d=cell.api.get_rigidbody_transformation(cell.carrier.path)["position"]
                print("BUFDBG",round(cell.seconds,1),run.phase,"deck",[round(v,3) for v in d],"car_z",round(z,4),
                      "joint",round(run.joint,3),flush=True)
            if run.phase=="turn":
                break
        deck_end=cell.api.get_rigidbody_transformation(cell.carrier.path)["position"]
        report={"slot":slot,"entry":run.entry,"phase":run.phase,"sim_s":round(cell.seconds,2),
                "car_z_parked":round(cell.parked_z,4),"lift_off":run.lift_off,
                "car_z_carried_min":round(min(carried),4) if carried else None,
                "deck_end":[round(v,3) for v in deck_end],
                "touches":sorted([list(k)+[n] for k,n in run.touches.items()])[:20]}
        print("BUFFER_CHECK",json.dumps(report),flush=True)
        return (run.phase=="turn" and not run.touches and carried and min(carried)>cell.parked_z+.03
                and abs(deck_end[0]-AISLE_X)<.02)
    finally:
        cell.close()


def check(path, seed, robots, export, interference_probe, until=None, start="buffer"):
    """Full cell: AMR docks, each tire is fed, stopped in the V, centred, lifted and left
    pick-ready within +-3 mm along / +-2 mm across the flow; nothing touches the robots or the
    pick-zone mechanisms; runtime poses are written back to USD."""
    stage = Usd.Stage.Open(str(path))
    stage.SetEditTarget(stage.GetSessionLayer())
    if interference_probe:
        start="gate"                  # the probe is timed from the gate start
    cell = Cell(stage,seed,robots,start=start)
    from perf import StepTimer
    import time as _time
    cell.timer = StepTimer()
    wall_start = _time.perf_counter()
    try:
        if not check_actors(stage,robots):
            return False
        if export:
            if not cell.export_setup(export):
                raise OSError(f"Could not save {export}")
            if not check_actors(Usd.Stage.Open(str(export)),robots,exported=True):
                return False
        car_path=ROOT+"/CN7"
        car=stage.GetPrimAtPath(car_path)
        if start=="gate" and dist(UsdGeom.Xformable(car).GetLocalTransformation().ExtractTranslation(),(AMR_GATE_X,0.,-.12))>.001:
            return False
        previous_deck=None
        previous_lift=None
        maximum_speed=[0.,0.,0.]
        settled=None
        # From the buffer the AMR first fetches the car and brings it to the gate (~50 s more).
        for i in range(int(DOCK_READY+10+(60 if start=="buffer" else 0))*HZ):
            if interference_probe and i==int(DOCK_READY+2)*HZ:
                for robot in cell.rail_pairs[0]:
                    robot.rail.Set(0.)
                    robot.advance=lambda *_:None
            cell.step()
            if os.environ.get("CELL_DEBUG") and cell.pending:
                for event in cell.pending:
                    print("EVENTDBG",*event,flush=True)
                cell.pending.clear()
            deck=cell.api.get_rigidbody_transformation(cell.carrier.path)["position"]
            lift=cell.api.get_rigidbody_transformation(cell.carrier.lift_path)["position"][2]-deck[2]
            if i%HZ==0:
                if previous_deck is not None and previous_lift is not None:
                    for axis,delta in enumerate((deck[0]-previous_deck[0],deck[1]-previous_deck[1],lift-previous_lift)):
                        maximum_speed[axis]=max(maximum_speed[axis],abs(delta))
                previous_deck=tuple(deck)
                previous_lift=lift
            if interference_probe and i>int(DOCK_READY+10)*HZ:
                break
            if until is not None and cell.seconds>=until:
                break
            # Stop 2 s after the car is docked and every tire is pick-ready: anything that
            # drifts in the zone or on the hub shows up in that hold.
            if settled is None and cell.carrier.ready and cell.finished:
                settled=cell.seconds
            if settled is not None and cell.seconds>=settled+2. and not interference_probe:
                break
        results={}
        for station in cell.stations:
            p=cell.api.get_rigidbody_transformation(station.wheel)["position"]
            results[station.name]={"result":station.phase,
                                   "along_mm":round((p[0]-station.hub[0])*1000,2),
                                   "across_mm":round((p[1]-station.side*LINE_Y)*1000,2),
                                   "s1":station.zone.s1.blocked,"s2":station.zone.s2.blocked,
                                   "belt_contact":station.belt_contact,"stopper_contact":station.stop_contact,
                                   "guide_contact":station.guide_contact,
                                   "conveyed_mm":round(station.conveyed_distance*1000,1),
                                   "max_lateral_mm":round(station.max_lateral_travel*1000,1),
                                   "hub_angle_error_deg":round(station.error,3)}
        print("PHYSICS_RESULT",json.dumps(results),flush=True)
        wall = _time.perf_counter()-wall_start
        print("PROFILE",json.dumps({"sim_s":round(cell.seconds,2),"wall_s":round(wall,1),
                                    "realtime_factor":round(cell.seconds/wall,3),"hz":HZ,
                                    "step_ms":cell.timer.summary_ms(),"steps":cell.timer.steps,
                                    "colliders":count_colliders(stage)}),flush=True)
        cell.api.update_transformations(False,True,False,False)
        print("RAIL_CHECK",cell.min_rail_spacing,cell.interference,flush=True)
        print("ZONE_CONTACTS",sorted(cell.zone_contacts),flush=True)
        measured_car=tuple(cell.api.get_rigidbody_transformation(car_path)["position"])
        print("CARRIER_CHECK",cell.dock_ready_time,measured_car,maximum_speed,flush=True)
        run=cell.buffer_run
        if run is not None:
            print("BUFFER_RUN",json.dumps({"phase":run.phase,"lift_off":run.lift_off,
                                           "touches":sorted([list(k)+[n] for k,n in run.touches.items()])[:20]}),flush=True)
            if run.touches or run.phase!="done":
                return False
        if until is not None and cell.seconds>=until:
            # Partial run (one area of the cycle): pass if nothing has failed so far.
            faults=[(s.name,s.phase) for s in cell.stations if s.phase in ("feed_failed","dock_lost")]
            broken=[s.name for s in cell.stations
                    if any(v!=v for v in cell.api.get_rigidbody_transformation(s.wheel)["position"])]
            print("PARTIAL_CHECK",round(cell.seconds,2),faults,flush=True)
            return not faults and not broken and not cell.zone_contacts
        if (not cell.carrier.ready or dist(measured_car,(0.,0.,0.))>.040
                or any(found>rated+.020 for found,rated in zip(maximum_speed,(AMR_SPEED_X,AMR_SPEED_Y,AMR_LIFT_SPEED)))):
            return False
        if interference_probe:
            return (any(event[-1]=="rail_command" for event in cell.interference)
                    and all(robot.paused_targets for robot in cell.rail_pairs[0])
                    and cell.min_rail_spacing>=MIN_RAIL_SPACING)
        for station in cell.stations:
            pose_usd=UsdGeom.Xformable(stage.GetPrimAtPath(station.wheel)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            runtime=cell.api.get_rigidbody_transformation(station.wheel)
            if dist(pose_usd.ExtractTranslation(),runtime["position"])>.0001:
                return False
            r=results[station.name]
            if not (station.belt_contact and station.stop_contact and station.conveyed_distance>.65):
                return False
            if r["result"]!="pick_ready" or abs(r["along_mm"])>3. or abs(r["across_mm"])>2.:
                return False
        return not cell.interference and not cell.zone_contacts
    finally:
        cell.close()


def main():
    args=argparse.ArgumentParser()
    args.add_argument("--stage",type=Path,default=root.parent/"scene"/"wheel_cell.usda")
    args.add_argument("--seed",type=int,default=42)
    args.add_argument("--without-robots",action="store_true")
    args.add_argument("--interference-probe",action="store_true",help="Command same-rail arms toward each other")
    args.add_argument("--supply-probe",choices=("left","right"),help="Run one wheel from truck infeed into a branch")
    args.add_argument("--insertion-probe",choices=("nominal","skew"),help="Press the FL wheel onto its hub on a bench slide")
    args.add_argument("--export-setup",type=Path,help="Save the physical scene before stepping")
    args.add_argument("--config",type=Path,help="JSON overrides for cell_config.DEFAULTS")
    args.add_argument("--until",type=float,help="Stop at this simulated time (s) and check only for faults so far")
    args.add_argument("--start",choices=("buffer","gate"),default="buffer",
                      help="buffer: the AMR fetches the car from the inbound buffer first (whole order); gate: start loaded at the gate")
    args.add_argument("--buffer-probe",type=int,choices=(0,1,2),help="Only fetch the car from this inbound buffer slot")
    args.add_argument("--entry",type=float,help="Hanger entry height for --buffer-probe (default process.AMR_LIFT_ENTRY)")
    config=args.parse_args()
    if config.interference_probe and config.without_robots:
        args.error("--interference-probe requires robots")
    if config.buffer_probe is not None:
        stage=Usd.Stage.Open(str(config.stage))
        stage.SetEditTarget(stage.GetSessionLayer())
        return check_buffer(stage,config.seed,config.buffer_probe,config.entry)
    if config.supply_probe or config.insertion_probe:
        stage=Usd.Stage.Open(str(config.stage))
        stage.SetEditTarget(stage.GetSessionLayer())
        if config.insertion_probe:
            return check_insertion(stage,config.seed,config.insertion_probe=="skew")
        return check_supply(stage,config.seed,1 if config.supply_probe=="left" else -1)
    return check(config.stage,config.seed,not config.without_robots,config.export_setup,
                 config.interference_probe,config.until,config.start)


if __name__=="__main__":
    exit_code=1
    try:
        exit_code=0 if main() else 1
    finally:
        omni.kit.app.get_app().post_quit(exit_code)
