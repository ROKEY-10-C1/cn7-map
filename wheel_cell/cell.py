"""The live cell: builds everything and steps stations, robots, AMR and contact reports."""

import os
from contextlib import nullcontext
from random import Random

from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

from conveyor import setup_factory
from physical_data import BODY_MASS, PAD_DYNAMIC_FRICTION, PAD_STATIC_FRICTION
from physical_data import box_inertia
from physical_data import GUIDE_DYNAMIC_FRICTION, GUIDE_STATIC_FRICTION
from physical_data import AMR_LIFT_MASS, AMR_MASS, AMR_SIZE
from process import AMR_BAY_Y, BUFFER_SLOT_X, STATIONS
from layout import LAYOUT
from sim_util import body, cube, mass_properties, material, pose, reference, transform
from live_params import HZ, INTERLOCK_FAULTS, MIN_RAIL_SPACING, REFERENCE, ROOT, TERMINAL_PHASES
from carrier import Carrier
from live_robot import Robot
from station import make_station
from wheel import collision_web


class Cell:
    def __init__(self, stage, seed, robots=True, branch_side=1, *, manual=True, start="buffer", slot=0):
        """start="buffer": the AMR fetches the car from inbound buffer slot `slot` and brings it to
        the gate, then docks (the whole order). start="gate": the AMR starts at the gate with the
        car already loaded (the cell part only)."""
        import omni.physx

        self.stage=stage
        self.api=omni.physx.get_physx_interface()
        self.sim=omni.physx.get_physx_simulation_interface()
        self.seconds=0.
        self.timer=None                    # perf.StepTimer, set by the caller to profile each step
        self._collider_paths={}
        self._robot_by_collider={}
        self.dock_ready_time=None
        datums=stage.GetPrimAtPath("/World/LayoutDatums")
        self.amr_stops=tuple(datums.GetAttribute("cell:amrStopsX").Get()) if datums else LAYOUT.stops
        if self.amr_stops != LAYOUT.stops:
            raise ValueError("Rebuild the stage or run with the CELL_LAYOUT_CONFIG used to build it")
        self.amr_stop=0
        self.amr_index_pending=False
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
        cube(stage,ROOT+"/Floor",(0,0,-.055),(20,2*(AMR_BAY_Y+3.),.11))
        # Keep the first car on the AMR at its low inbound datum until the dock closes.
        source_amr=REFERENCE+"/Carrier"
        source_lift=source_amr+"/Hanger"
        source_car=stage.GetPrimAtPath(REFERENCE+"/Vehicle") or stage.GetPrimAtPath(source_lift)
        gate_time=stage.GetPrimAtPath(REFERENCE).GetAttribute("cell:gateTimeS").Get() or 0.
        deck_start,_=pose(stage,source_amr,gate_time)
        car_start,_=pose(stage,source_lift,gate_time)
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
                # Index stops B and C have their own floor clamps (build.dock_clamps); closed while the robots work.
                for prefix,stop in (("RearDockClamp",1),("FinalDockClamp",2)):
                    clamp=f"/{prefix}_{axle}_{side}"
                    if stage.GetPrimAtPath(REFERENCE+clamp):
                        self.carrier.add_clamp(stage,ROOT+clamp,REFERENCE+clamp,self.amr_stops[stop])
        car=body(stage,ROOT+"/CN7",car_start,None,BODY_MASS)
        PhysxSchema.PhysxContactReportAPI.Apply(car).CreateThresholdAttr(0.)
        # The replay hierarchy contains wheel/nut animation, not just the car shell.
        # Copy only static car parts so timeline scrubbing cannot reveal extra wheels.
        visual=transform(stage,ROOT+"/CN7/Visual")
        for child in stage.GetPrimAtPath(source_lift).GetChildren():
            if child.GetName().startswith(("Support_","Crossmember_","ChassisPad_","SaddleRail_")):
                reference(stage,ROOT+"/DockLift/"+child.GetName(),str(child.GetPath()))
        for child in source_car.GetChildren():
            if not child.GetName().startswith(("Wheel_","Support_","Crossmember_","ChassisPad_","SaddleRail_")):
                target=str(visual.GetPath())+"/"+child.GetName()
                reference(stage,target,str(child.GetPath()))
                if child.GetName() in STATIONS:
                    for item in child.GetChildren():
                        if item.GetName().startswith("Nut_"):
                            stage.OverridePrim(target+"/"+item.GetName()).SetActive(False)
        cube(stage,ROOT+"/CN7/Undertray",(0,0,1.10),(3.7,1.16,.23))
        cube(stage,ROOT+"/CN7/Body",(0,0,1.55),(4.56,1.57,.36))
        # Sills and bumper corners cut from the body mesh (the buffer stands and the hanger meet them).
        from vehicle import body_underside, buffer_stands
        body_underside(stage,ROOT+"/CN7")
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
            self.carrier.add_locator(stage,f"{ROOT}/HangerLocator/Side_{side+1}",(0,side*.615,1.01),(.20,.06,.06))
            self.carrier.add_locator(stage,f"{ROOT}/HangerLocator/End_{side+1}",(side*1.885,0,1.01),(.06,.20,.06))
        guide=material(stage,"GuideUHMW",GUIDE_STATIC_FRICTION,GUIDE_DYNAMIC_FRICTION,combine="min")
        # Feed belts end in a fixed V stop (scene/overrides/factory.usda): it replaces the pick zone. Does nothing
        # on a stage without that stop. See wheel_mount_sim/cell_layout.py.
        import sys
        from pathlib import Path
        sys.path.append(str(Path(__file__).resolve().parent.parent/"wheel_mount_sim"))
        import cell_layout
        cell_layout.pre_build(stage)
        setup_factory(stage,ROOT,mats[2],guide,branch_side)
        self.buffer_stands=buffer_stands(stage)
        rng=Random(seed)
        self.stations=[make_station(stage,car,name,rng.uniform(-180.,180.),sections,mats,seed) for name in STATIONS]
        # Mount robots and mount physics (hub pins, wheel pad, ring tire) from map/wheel_mount_sim;
        # the layout stays as built. See wheel_mount_sim/cell_mount.py.
        from cell_mount import install as install_mount
        from process import MOUNT_BASE_X, ROBOT_BASE_Y
        self.mount_arms=(install_mount(stage,self.stations,MOUNT_BASE_X,ROBOT_BASE_Y,arms=robots)
                         if os.environ.get("CELL_MOUNT_SIM","1")!="0" else {})
        cell_layout.post_build(stage,self.stations)
        # Earlier saved stages have rail actors under Orders. Fixed RS080N arms
        # are permanent Factory equipment and already visible in the live stage.
        sources = [REFERENCE + f"/{prefix}Mount" for prefix in ("Left", "Right")]
        self.robots = [Robot(stage, path) for path in sources if stage.GetPrimAtPath(path)] if robots else []
        self.robots_by_path={robot.path:robot for robot in self.robots}
        self.rail_pairs=[(robot,) for robot in self.robots]
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
        self.car_path=ROOT+"/CN7"
        self.buffer_run=None
        if start=="buffer":
            from vehicle import BufferRun, place_at_buffer
            stands,self.parked_z=place_at_buffer(stage,self,slot)
            self.buffer_run=BufferRun(self.carrier,BUFFER_SLOT_X[slot],stands)
            self.pending.append((0.,"AMR",f"buffer_slot_{slot+1}"))
        elif start=="gate":
            waiting=stage.GetPrimAtPath(f"/World/Factory/LeftVehicleBuffer/Vehicle_{slot}")
            if waiting:
                waiting.SetActive(False)
        else:
            raise ValueError(f"start must be 'buffer' or 'gate', not {start!r}")
        for k in range(len(BUFFER_SLOT_X)):
            waiting=stage.GetPrimAtPath(f"/World/Factory/LeftVehicleBuffer/Vehicle_{k}")
            if waiting and waiting.IsActive():
                UsdGeom.Imageable(waiting).MakeVisible()
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
        for pair in self.rail_pairs:
            if len(pair)<2:
                continue
            mount,fasten=pair
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

    def stop_pair(self,*args):
        """Hold the robots sharing a rail (now one per rail) and fail that side's stations."""
        *robots,reason=args
        if all(robot.paused_targets for robot in robots):
            return
        self.interference.append((self.seconds,*(robot.path for robot in robots),reason))
        self.pending.append((self.seconds,robots[0].path.rsplit("/",1)[1],reason))
        for robot in robots:
            robot.hold(self.api)
        for station in self.stations:
            if any((station.side>0)==("Left" in r.path) for r in robots) and station.phase not in TERMINAL_PHASES:
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

    @property
    def buffer_active(self):
        return self.buffer_run is not None and self.buffer_run.phase!="done"

    def advance_amr(self, *, completed, robots_clear, load_secured):
        """Accept measured completion/safety from the process controller before moving."""
        needed=({"front_mounted"}, {"front_fastened","rear_mounted"}, {"rear_fastened"})
        if (self.amr_stop>=len(needed) or self.amr_index_pending or self.buffer_active or
            not robots_clear or not load_secured or self.interference or
            any(unit.phase in INTERLOCK_FAULTS for unit in self.stations) or
            not needed[self.amr_stop].issubset(completed)):
            return False
        if not self.carrier.request_index(self.amr_stops[self.amr_stop+1]):
            return False
        self.amr_index_pending=True
        return True

    def _prepare_step(self,dt):
        if self.buffer_active:
            old=self.buffer_run.phase
            if self.buffer_run.step(self.api,dt)!=old:
                self.pending.append((self.seconds,"AMR",f"buffer_{self.buffer_run.phase}"))
            if not self.buffer_active:
                # At the gate with the car at -0.12: the dock approach starts from here.
                state=self.api.get_rigidbody_transformation(self.carrier.path)
                self.carrier.previous=Gf.Vec3d(*state["position"])
                self.carrier.velocity=Gf.Vec3d(0.)
                self.carrier.previous_yaw=0.
        else:
            self.carrier.step(self.api,dt)
            if self.amr_index_pending and self.carrier.index_arrived:
                self.amr_stop+=1
                self.amr_index_pending=False
                self.pending.append((self.seconds,"AMR",("park_a","index_b","index_c","exit")[self.amr_stop]))
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
        buffer_pairs=[]
        self.carrier.grounded=False
        self.carrier.clamp_contacts.clear()
        for header in headers:
            if not header.num_contact_data:
                continue
            collider0,collider1=self._collider(header.collider0),self._collider(header.collider1)
            if self.buffer_active:
                first=header.contact_data_offset
                deep=min(contacts[k].separation for k in range(first,first+header.num_contact_data))<=-.0002
                buffer_pairs.append((collider0,collider1,deep))
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
        if self.buffer_active:
            car=self.api.get_rigidbody_transformation(self.car_path)
            self.buffer_run.observe(self.car_path,self.seconds,car["position"][2],self.parked_z,buffer_pairs)
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
