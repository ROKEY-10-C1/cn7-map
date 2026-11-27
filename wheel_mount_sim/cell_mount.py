"""Put the wheel_mount_sim mount robots and mount physics into the live wheel cell (wheel_cell/cell.py).

The cell layout stays as built (robot bases, pick zone, hubs, conveyors). Only mount-related physics is
replaced by the models validated in this folder (docs/GOALS.md, params.py):

- mount robots: RS080N articulation + SCHUNK PHL-W 63-080-S tread clamp (synchronised jaws, PU shoes 10 deg
  under the horizontal), on the cell's fixed bases, starting in the cell's PARK pose. The keyframed visual
  arms of the same place are switched off in the live stage only.
- hub: 144-gon pilot prism (the native cylinder made false contacts), contact offsets from the clearances,
  preloaded bearing (no play, 0.75 N m drag), suspension wheel rate from the ride frequency, steel/alloy friction.
- wheel: mounting pad as Delaunay triangles with the hub-side bore chamfer; ring-tire stiffness from the
  measured-average literature values.

Call `install(stage, stations)` after the stations exist; it returns the arm paths for control.
"""
import sys
from math import pi
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import params as P  # noqa: E402
import scene as S  # noqa: E402
from geom import circle, extrude, limit, look, material, per_deg, prism  # noqa: E402
from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

PARK = np.deg2rad([0, -30, -65, 0, -130, 0])   # wheel_cell/cell_motion.PARK (same joint convention)
ROOT = "/World/Live"


def _materials(stage, root):
    M = dict(pu=material(stage, root+"/Mat/PU", *P.MU_SHOE_TREAD),
             steel=material(stage, root+"/Mat/Steel", 1.5, 1.5),
             alloy=material(stage, root+"/Mat/Alloy", *P.MU_ALU_STEEL))
    V = dict(blue=look(stage, root+"/Look/Schunk", (.10, .30, .62), .45, .2),
             alu=look(stage, root+"/Look/Alu", (.75, .76, .78), .35, .8),
             dark=look(stage, root+"/Look/Dark", (.12, .12, .13), .6, .3),
             pu=look(stage, root+"/Look/PU", (.90, .55, .10), .7),
             steel=look(stage, root+"/Look/Steel", (.55, .56, .58), .35, .9))
    return M, V


def _bind_physics(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def _offset(prim, value):
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(value)
    api.CreateRestOffsetAttr(0.)


def _hub(stage, station, M, V):
    hub = station.hub_path
    pilot = stage.GetPrimAtPath(hub+"/Pilot")
    radius = UsdGeom.Cylinder(pilot).GetRadiusAttr().Get() if pilot and pilot.IsA(UsdGeom.Cylinder) else P.PILOT/2
    for name in ("Pilot", "PilotTip"):
        if stage.GetPrimAtPath(hub+"/"+name):
            stage.GetPrimAtPath(hub+"/"+name).SetActive(False)
    z0, L = P.OFFSET, P.PILOT_LEN
    po = P.PILOT_CONTACT_OFFSET
    extrude(stage, hub+"/MountSimPilot", circle(0, 0, radius, 144), z0, z0+L-.0015, phys=M["steel"], vis=V["steel"],
            offset=po)
    prism(stage, hub+"/MountSimPilotTip", [(x, y, z0+L-.0015) for x, y in circle(0, 0, radius, 144)],
          [(x, y, z0+L) for x, y in circle(0, 0, radius-.001, 144)], phys=M["steel"], vis=V["steel"], offset=po)
    for child in stage.GetPrimAtPath(hub).GetChildren():
        n = child.GetName()
        if n == "Face":
            _offset(child, P.FACE_CONTACT_OFFSET)
        if n == "Face" or n.startswith("Stud"):
            _bind_physics(child, M["steel"])
    name = station.name
    brg = stage.GetPrimAtPath(f"{ROOT}/{name}/Bearing")
    if brg:
        UsdPhysics.DriveAPI(brg, "rotZ").GetDampingAttr().Set(P.BEARING_DRAG/.5)   # 0.75 N m at 0.5 deg/s
        j = UsdPhysics.Joint(brg)
        for dof in ("transX", "transY"):
            limit(j, dof, 1., -1.)                                                  # preloaded: no radial play
    susp = stage.GetPrimAtPath(f"{ROOT}/{name}/Suspension")
    if susp:
        d = UsdPhysics.DriveAPI(susp, "transZ")
        d.GetStiffnessAttr().Set(P.WHEEL_RATE)
        d.GetTargetPositionAttr().Set(-P.DROOP_PRELOAD/P.WHEEL_RATE)


def _wheel(stage, station, M, V):
    """Keep the cell's CAD pad triangles; add the hub-side bore chamfer, our alloy friction and contact offset."""
    wheel = stage.GetPrimAtPath(station.wheel)
    ch, rb = P.BORE_CHAMFER, P.BORE/2
    for child in list(wheel.GetChildren()):
        name = child.GetName()
        if not name.startswith("Web_") or not child.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(child).GetPointsAttr().Get()
        n = len(pts)//2
        tri = [(float(p[0]), float(p[1])) for p in pts[:n]]
        child.SetActive(False)
        on_bore = [abs(np.hypot(x, y)-rb) < 2e-5 for x, y in tri]
        z0 = P.OFFSET
        if any(on_bore):
            low = [((x*(rb+ch)/np.hypot(x, y), y*(rb+ch)/np.hypot(x, y)) if f else (x, y)) for (x, y), f in zip(tri, on_bore)]
            p = prism(stage, station.wheel+f"/{name}MSCh", [(x, y, P.OFFSET) for x, y in low],
                      [(x, y, P.OFFSET+ch) for x, y in tri], phys=M["alloy"], offset=P.PAD_CONTACT_OFFSET)
            UsdGeom.Imageable(p).CreatePurposeAttr("guide")
            z0 = P.OFFSET+ch
        p = extrude(stage, station.wheel+f"/{name}MS", tri, z0, P.OFFSET+P.WEB_T, phys=M["alloy"],
                    offset=P.PAD_CONTACT_OFFSET)
        UsdGeom.Imageable(p).CreatePurposeAttr("guide")
    # ring tire on its sidewall springs (cell joint, our rates)
    mount = stage.GetPrimAtPath(station.tire+"Mount")
    if mount:
        z = P.TIRE_ZETA
        r_i, r_o = P.RIM_R, P.TIRE_R
        I_ax = (r_i**2+r_o**2)/2*P.TIRE_MASS
        for dof, k in (("transX", P.TIRE_K_RADIAL), ("transY", P.TIRE_K_RADIAL), ("transZ", P.TIRE_K_LAT)):
            d = UsdPhysics.DriveAPI(mount, dof)
            d.GetStiffnessAttr().Set(k)
            d.GetDampingAttr().Set(2*z*np.sqrt(k*P.TIRE_MASS))
        for dof, k, inertia in (("rotZ", P.TIRE_K_TORSION, I_ax), ("rotX", P.TIRE_K_TILT, I_ax/2),
                                ("rotY", P.TIRE_K_TILT, I_ax/2)):
            d = UsdPhysics.DriveAPI(mount, dof)
            d.GetStiffnessAttr().Set(per_deg(k))
            d.GetDampingAttr().Set(per_deg(2*z*np.sqrt(k*inertia)))


def install(stage, stations, base_x, base_y, arms=True):
    """Mount physics of wheel_mount_sim into the live cell. Returns {side label: paths} of the mount arms."""
    root = ROOT+"/MountSim"
    UsdGeom.Scope.Define(stage, root)
    M, V = _materials(stage, root)
    for station in stations:
        _hub(stage, station, M, V)
        _wheel(stage, station, M, V)
    out = {}
    if not arms:
        return out
    if not S.ROBOT_USD.exists():
        raise FileNotFoundError(f"{S.ROBOT_USD} missing: run wheel_mount_sim/tools/fetch_assets.sh")
    for side, label in ((1, "Left"), (-1, "Right")):
        # The cell turns its arm 180 deg on the left (URDF world2base 0); this USD carries world2base -90 deg.
        yaw = (180. if side > 0 else 0.)+90.
        prefix = f"{root}/{label}"
        rp = prefix+"/Arm"
        base = np.array([base_x, side*base_y, 0.])
        S.robot(stage, M, V, True, rp=rp, base=base, yaw=yaw, prefix=prefix, q0=PARK)
        visual = stage.GetPrimAtPath(f"/World/Factory/{label}MountRobot")
        if visual:
            visual.SetActive(False)                       # live stage only: the physics arm stands in its place
        out[label] = dict(root=rp, prefix=prefix, side=side, base=base, yaw=yaw,
                          tool=prefix+"/Tool", jaw_slide=prefix+"/JawRSlide")
    return out
