"""Build a self-contained animated USD and its simulated production history.

Run with Isaac Sim's Python:  isaac_python map/wheel_cell/build.py
"""
import os
import sys


def _ensure_usd():
    """Isaac Sim's python.sh does not expose pxr; re-exec with omni.usd.libs on the paths."""
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    sim = Path(os.environ.get("ISAAC_SIM_ROOT", Path.home()/"isaacsim"))
    libs = sorted(sim.glob("extscache/omni.usd.libs-*/pxr"))
    if not libs or os.environ.get("_CELL_USD_BOOTSTRAPPED"):
        sys.exit("Cannot import pxr: set ISAAC_SIM_ROOT to the Isaac Sim installation.")
    usd = libs[0].parent
    env = dict(os.environ, _CELL_USD_BOOTSTRAPPED="1")
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(usd), env.get("PYTHONPATH")]))
    env["LD_LIBRARY_PATH"] = os.pathsep.join(filter(None, [str(usd/"bin"), env.get("LD_LIBRARY_PATH")]))
    os.execve(sys.executable, [sys.executable, "-B", *sys.argv], env)


from pathlib import Path

_ensure_usd()
sys.path.insert(0, str(Path(__file__).resolve().parent))

import argparse
import shutil
import sqlite3
from math import atan2, ceil, cos, degrees, hypot, sin, sqrt, tau
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
from scipy.spatial.transform import Rotation

from fitment import (
    BODY_LENGTH, BODY_WIDTH, BOLT_ANGLES, BOLT_CENTERS, BORE, DEMO_TORQUE, LUG_HOLE,
    NUT_LENGTH, NUT_SEAT, NUT_TORQUE, OFFSET, PCD, PILOT, PITCH, STUD_DIAMETER,
    STUD_PROJECTION, WHEELBASE, WHEEL_MASS, fit, half_track,
)
from physical_data import AMR_SIZE, AMR_WHEEL_RADIUS, DOCK_CLAMP_CLOSED, DOCK_CLAMP_OPEN, ROLLER_RADIUS
from kmp1500_urdf import SCANNER_X, SCANNER_Y, SCANNER_Z, S300_SIZE
from process import (
    AMR_BAY_Y, AMR_DOCKED, AMR_GATE_LIFT, AMR_GATE_X, AMR_LIFTED, AXLE, BELT_Z, BUFFER_CAR_Z,
    BUFFER_SLOT_X, BUFFER_STAND_X, BUFFER_STAND_Y, CELL_Y, CN7_R16, DOCK_X, FASTEN_BASE_X, FEED_START_X, MOUNT_BASE_X, MOUNT_HUB_X, MOUNT_CLEAR, ROBOT_BASE_Y, FPS,
    HUB_Z, INDEX_TRAVEL, LINE_Y, NUT_CYCLE, NUT_DRIVE, NUT_ENGAGE, NUT_FINAL,
    REJECT_CRATE_Y, REJECT_GATE_HALF, REJECT_X, STATIONS, inbound_pickup, outbound_dropoff, schedule,
)
from layout import LAYOUT
from process import AMR_TO_RETURN, CHARGE_BAY_SIZE, CHARGE_BAY_X, CHARGE_BAY_YS, EXIT_X, FASTEN_HUB_X, FINAL_X, INDEX_X, RETURN_X, belt_height
from wheel_geometry import barrel, center_web, hub as make_hub, nut as make_nut, wheelhouse
from robot import Arm
from cell_motion import (GRIP_AT, GRIP_CENTRE, HEAD_LENGTH, MAGAZINE, NUT_ADVANCE, NUT_PICK_AT, fasten_keys,
                         mount_keys, samples, solve_plan)
from visuals import (ALLOY, BRUSHED, CONCRETE, DECAL_FILES, DECAL_SOURCE, GLASS, LIB, PANEL, RUBBER, ZINC, VisualAssets, add_instance,
                     copy_mesh, decal_for, decal_prototype, finish, mesh)

BLUE = (.10, .34, .55)
STEEL = (.38, .45, .49)
DARK = (.045, .06, .073)
AMBER = (.85, .60, .039)
WHITE = (.72, .77, .79)
RED = (.73, .10, .08)
TEAL = (.08, .58, .43)
# Plant layout outside the cell: fence at CELL_Y (process.py), walkway, buffer guard, buffers, outer guard, wall.
BUFFER_GUARD_Y = AMR_BAY_Y-2.65
OUTER_GUARD_Y = AMR_BAY_Y+2.65
HALL_Y = OUTER_GUARD_Y+.40
FLOOR_Y = HALL_Y+.40
# The outbound (-y) side is open (no hall wall); the floor ends as far out as on the inbound side.
FLOOR_S = FLOOR_Y
# Floor x range: 1 m extra on the aisle (west) side so the aisle paint keeps 1.05 m to the floor edge;
# the east return aisle (RETURN_X) keeps the same 3.5 m to the east edge.
# West edge: 1 m past the AMR charging bays at the end of the west aisle (process.CHARGE_BAY_X).
FLOOR_X0, FLOOR_X1 = min(-11., CHARGE_BAY_X-CHARGE_BAY_SIZE[1]/2-1.0), max(10., EXIT_X+2.65, LAYOUT.supply_x+.7, RETURN_X+3.5)
FLOOR_XMID, FLOOR_W = (FLOOR_X0+FLOOR_X1)/2, FLOOR_X1-FLOOR_X0
FLOOR_MID, FLOOR_LEN = (FLOOR_Y-FLOOR_S)/2, FLOOR_Y+FLOOR_S
# AMR travel zones painted on the floor: the car-carrying AMR's swept area (CN7 body 4.71 x 1.825 m
# plus 0.1 m) and the in-place turn circles at the aisle ends.
CAR_HALF = (4.71/2, 1.825/2)
ZONE_MARGIN = .10
ZONE_FILL = (.78, .70, .44)
# Fixed fastening robot: Kawasaki RS080N from the fastening team (map_fasten/, main 1fc0e9c).
RS080N_ASSETS = Path(__file__).resolve().parent.parent/"map_fasten"/"assets"
KAWASAKI_GRAY = (.84, .85, .86)    # displayColor of map_fasten/assets/RS080N.usdc
# Static park posture (no fastening motion in the cell yet): wrist over the base front, sockets
# facing down. In the robot frame (+y = toward the car) every joint origin and the socket tips stay
# within y -0.30..+0.50 m and below z 2.32 m in the parked pose.
RS080N_PARK = np.deg2rad([0, -30, -65, 0, -130, 0])
# Flange frame of the fastening tool (see Arm.solve): x = world x, y = world z * side, so
# stud k at hub angle a sits at (PCD/2 cos a, -PCD/2 sin a) on either side of the car.
SOCKET_LAYOUT = tuple((PCD/2*cos(a), -PCD/2*sin(a)) for a in BOLT_ANGLES)


def frame(seconds):
    return seconds * FPS


def vector(values):
    return Gf.Vec3d(*map(float, values))


def color(shape, rgb):
    finish(shape, rgb)


def xform(stage, path, pos=(0, 0, 0)):
    item = UsdGeom.Xform.Define(stage, path)
    op = item.AddTranslateOp()
    op.Set(vector(pos))
    return item, op


def move(op, seconds, pos):
    op.Set(vector(pos), frame(seconds))


def box(stage, path, pos, dimensions, rgb):
    item = UsdGeom.Cube.Define(stage, path)
    item.CreateSizeAttr(1)
    item.AddTranslateOp().Set(vector(pos))
    item.AddScaleOp().Set(vector(dimensions))
    item.CreateExtentAttr([(-.5, -.5, -.5), (.5, .5, .5)])
    color(item, rgb)
    return item


def cylinder(stage, path, pos, radius, height, rgb, axis="Z"):
    item = UsdGeom.Cylinder.Define(stage, path)
    item.CreateRadiusAttr(radius)
    item.CreateHeightAttr(height)
    item.CreateAxisAttr(axis)
    extent = tuple(height/2 if direction == axis else radius for direction in "XYZ")
    item.CreateExtentAttr([tuple(-value for value in extent), extent])
    item.AddTranslateOp().Set(vector(pos))
    color(item, rgb)
    return item


def ring(stage, path, pos, outer, inner, width, rgb, segments=32):
    mesh = UsdGeom.Mesh.Define(stage, path)
    points = [(r * cos(tau * i / segments), r * sin(tau * i / segments), z)
              for z in (-width / 2, width / 2) for r in (outer, inner)
              for i in range(segments)]
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        n = segments
        faces.extend(((i, j, j+2*n, i+2*n), (j+n, i+n, i+3*n, j+3*n),
                      (j, i, i+n, j+n), (i+2*n, j+2*n, j+3*n, i+3*n)))
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr([(-outer, -outer, -width/2), (outer, outer, width/2)])
    mesh.AddTranslateOp().Set(vector(pos))
    color(mesh, rgb)


def visibility(item, start, end=None):
    attr = UsdGeom.Imageable(item.GetPrim()).CreateVisibilityAttr("invisible")
    attr.Set("invisible", 0)
    attr.Set("inherited", frame(start))
    if end is not None:
        attr.Set("invisible", frame(end))


def belt(stage, name, a, b, width=.8, outlet_x=None):
    x, y = (a[0]+b[0])/2, (a[1]+b[1])/2
    longitudinal = abs(a[0]-b[0]) > abs(a[1]-b[1])
    length = abs(a[0]-b[0]) + abs(a[1]-b[1])
    dx, dy = (length, width) if longitudinal else (width, length)
    body = f"/World/Factory/{name}"
    box(stage, body+"/Bed", (x, y, .410), (dx, dy, .060), RUBBER)
    spacing = max(1, ceil(length/1.35))
    for i in range(spacing):
        along = (i+.5)*length/spacing-length/2
        for sign in (-1, 1):
            px = x+along if longitudinal else x+sign*.26
            py = y+sign*.26 if longitudinal else y+along
            box(stage, body+f"/Leg_{i}_{sign+1}", (px, py, .21), (.06, .06, .41), BRUSHED)
            box(stage, body+f"/Foot_{i}_{sign+1}", (px, py, .027), (.13, .115, .023), ZINC)
    gate=None
    for i in range(2):
        offset = (i - .5) * (width-.026)
        pos = (x, y + offset, .451) if longitudinal else (x + offset, y, .451)
        dims = (dx, .045, .084) if longitudinal else (.045, dy, .084)
        path=f"/World/Factory/{name}_rail{i}"
        if i==1 and outlet_x is not None:
            low,high=outlet_x-REJECT_GATE_HALF,outlet_x+REJECT_GATE_HALF
            for j,(x0,x1) in enumerate(((min(a[0],b[0]),low),(high,max(a[0],b[0])))):
                box(stage,path+f"_{j}",((x0+x1)/2,pos[1],pos[2]),(x1-x0,.045,.084),BRUSHED)
            panel=box(stage,f"/World/Factory/{name}_Gate",(outlet_x,y+width/2+.014,.549),
                      (2*REJECT_GATE_HALF,.035,.102),BRUSHED)
            gate=panel.GetOrderedXformOps()[0]
        else:
            box(stage,path,pos,dims,BRUSHED)
    rollers = ceil(length / .105)
    for i in range(rollers):
        offset = (i + .5) * length / rollers - length / 2
        z=BELT_Z-ROLLER_RADIUS
        pos = (x + offset, y, z) if longitudinal else (x, y + offset, z)
        cylinder(stage, f"/World/Factory/{name}_roller{i}", pos, ROLLER_RADIUS, width-.09, ZINC,
                 "Y" if longitudinal else "X")
    return gate


def fence(stage, path, a, b, height=1.46):
    dx,dy=b[0]-a[0],b[1]-a[1]
    for i in range(ceil(hypot(dx,dy)/1.40)):
        count=ceil(hypot(dx,dy)/1.40)
        t=(i+.5)/count
        length=hypot(dx,dy)/count
        node,_=xform(stage, path+f"/Panel_{i}", (a[0]+dx*t,a[1]+dy*t,0.))
        node.AddRotateZOp().Set(degrees(atan2(dy,dx)))
        panel=str(node.GetPath())
        for j in (-1,1):
            box(stage,panel+f"/Stile_{j+1}",(j*length/2,0,height/2),(.045,.06,height),AMBER)
            box(stage,panel+f"/Header_{j+1}",(0,0,.10 if j<0 else height-.018),(length,.04,.036),AMBER)
        # One batched mesh for the thin wire faces.
        points=[]
        for j in range(1,ceil(length/.084)):
            x=-length/2+j*.084
            points.extend(((x,-.005,.10),(x+.008,-.005,.10),(x+.008,-.005,height-.018),(x,-.005,height-.018)))
        for j in range(ceil((height-.14)/.081)):
            z=.10+j*.081
            points.extend(((-length/2,-.005,z),(length/2,-.005,z),(length/2,-.005,z+.008),(-length/2,-.005,z+.008)))
        mesh(stage,panel+"/SafetyMesh",points,[4]*(len(points)//4),list(range(len(points))),BRUSHED,True)


def charging_bays(stage, root):
    """AMR charging bays west of the outbound buffer, beside the end of the west AMR aisle: a zone floor,
    three hatched bays stacked along y (the AMR stands along x, heading -x) and a contact charger at each
    bay's west end."""
    w, l = CHARGE_BAY_SIZE                                  # across (y), along the AMR (x)
    cx = CHARGE_BAY_X
    ys = CHARGE_BAY_YS
    zy0, zy1 = min(ys)-w/2-.45, max(ys)+w/2+.45
    zx0, zx1 = cx-l/2-.55, cx+l/2+.35
    box(stage, root+"/ZoneFloor", ((zx0+zx1)/2, (zy0+zy1)/2, .002), (zx1-zx0, zy1-zy0, .002), (.30, .52, .40))
    for j, (x, sx) in enumerate(((zx0, .06), (zx1, .06))):
        box(stage, root+f"/ZoneEdgeX_{j}", (x, (zy0+zy1)/2, .003), (sx, zy1-zy0, .003), WHITE)
    for j, y in enumerate((zy0, zy1)):
        box(stage, root+f"/ZoneEdgeY_{j}", ((zx0+zx1)/2, y, .003), (zx1-zx0, .06, .003), WHITE)
    for i, y in enumerate(ys):
        bay = root+f"/ChargeBay_{i}"
        for j, (dx, dy, sx, sy) in enumerate(((0, -w/2, l, .05), (0, w/2, l, .05), (-l/2, 0, .05, w+.05), (l/2, 0, .05, w+.05))):
            box(stage, bay+f"/Edge_{j}", (cx+dx, y+dy, .004), (sx, sy, .004), AMBER)
        n = 7                                               # diagonal hatch, 45 deg, across the whole bay
        for k in range(n):
            item = box(stage, bay+f"/Hatch_{k}", (cx+(k-(n-1)/2)*(l-1.32)/(n-1), y, .004), (.035, w, .003), AMBER)
            ops = item.GetOrderedXformOps()                 # translate, rotate, then scale the unit cube
            turn = item.AddRotateZOp()
            turn.Set(45.)
            item.SetXformOpOrder([ops[0], turn, ops[1]])
        # contact charger at the bay's west end; floor contact plate under the AMR rear
        ex = cx-l/2-.16
        box(stage, bay+"/Charger", (ex, y, .25), (.26, .55, .50), PANEL)
        box(stage, bay+"/ChargerFace", (ex+.135, y, .26), (.012, .40, .22), DARK)
        box(stage, bay+"/ChargerLED", (ex+.136, y+.20, .44), (.012, .05, .03), TEAL)
        box(stage, bay+"/ContactPlate", (cx-l/2+.30, y, .006), (.30, .42, .012), BRUSHED)
        for s_ in (-1, 1):
            box(stage, bay+f"/ContactRail_{s_+1}", (cx-l/2+.30, y+s_*.12, .014), (.26, .04, .008), ZINC)


def parking(stage, root, side):
    bay_y=side*AMR_BAY_Y
    marker=(.06,.53,.74)
    for i,x in enumerate(BUFFER_SLOT_X):
        box(stage,root+f"/Line_{i}_a",(x-.99,bay_y,.004),(.032,4.98,.004),marker)
        box(stage,root+f"/Line_{i}_b",(x+.99,bay_y,.004),(.032,4.98,.004),marker)
        if side>0:
            # Live PhysX and the ROS bridge drive these waiting cars with replay disabled.
            pos,_=xform(stage,root+f"/Vehicle_{i}",(x,bay_y,BUFFER_CAR_Z))
            pos.AddRotateZOp().Set(-90.)
            pos.CreateVisibilityAttr("invisible")
            car=str(pos.GetPath())
            add_instance(stage,car+"/Body",LIB+"/Body")
            add_instance(stage,car+"/Decal",decal_prototype(stage,decal_for(i+1)))
            for name,(axle,sign) in STATIONS.items():
                y=sign*half_track(axle)
                cylinder(stage,car+f"/Hub_{name}",(axle,y,HUB_Z),.132,.065,ALLOY,"Y")
        # Stands under the body ends (not under the hubs), leaving the lane for the hanger free.
        # Tops touch the lowest body point over each 0.2 m stand (AssetLibrary Body, measured 2026-11-26:
        # z 1.0615 front, 1.0305 rear; rounding to mm left the front 0.5 mm short and the rear 0.5 mm
        # into the body); the floor is at -BUFFER_CAR_Z in this frame. A car carried at
        # AMR_LIFT_CARRY clears these tops by 4.7 cm when it passes an empty slot.
        for end,top in ((1,1.0615),(-1,1.0305)):
            for sign in (-1,1):
                box(stage,root+f"/Stand_{i}_{'F' if end>0 else 'R'}{'L' if sign>0 else 'R'}",
                    (x-side*sign*BUFFER_STAND_Y,bay_y-side*end*BUFFER_STAND_X,(top+BUFFER_CAR_Z)/2),
                    (.20,.20,top+BUFFER_CAR_Z),STEEL)


def amr_zones(stage, path):
    """Floor paint for where the AMR runs with a car: a tinted fill (z 1-2.6 mm, under the lane and
    slot lines) with amber edge lines and turn circles at the aisle ends."""
    half_l, half_w = CAR_HALF[0]+ZONE_MARGIN, CAR_HALF[1]+ZONE_MARGIN
    turn = hypot(*CAR_HALF)+ZONE_MARGIN          # car corner radius when it turns in place
    cut = sqrt(turn**2-half_l**2)                # where a straight lane edge meets the circle
    fill = lambda name, x0, x1, y0, y1, rgb=ZONE_FILL: box(
        stage, path+f"/{name}", ((x0+x1)/2, (y0+y1)/2, .0018), (x1-x0, y1-y0, .0016), rgb)
    def line(name, a, b, rgb=AMBER, width=.06):
        (x0, y0), (x1, y1) = a, b
        size = (abs(x1-x0)+width, width, .003) if y0 == y1 else (width, abs(y1-y0)+width, .003)
        box(stage, path+f"/{name}", ((x0+x1)/2, (y0+y1)/2, .004), size, rgb)
    gx, rx = AMR_GATE_X, RETURN_X
    # West aisle (inbound only): inbound lane head -> gate, car lengthwise along x.
    fill("Aisle", gx-half_l, gx+half_l, -half_w, AMR_BAY_Y)
    for k, x in enumerate((gx-half_l, gx+half_l)):
        line(f"AisleEdge_{k}", (x, -half_w), (x, AMR_BAY_Y-cut))
    line("AisleEnd", (gx-half_l, -half_w), (gx+half_l, -half_w))
    # Forward exit -> east return aisle -> outbound lane head (car lengthwise along x).
    fill("ExitLane", EXIT_X, rx-half_l, -half_w, half_w)
    for k, e in enumerate((-1, 1)):
        line(f"ExitLaneEdge_{k}", (EXIT_X, e*half_w), (rx-half_l, e*half_w))
    fill("ReturnAisle", rx-half_l, rx+half_l, -AMR_BAY_Y, half_w)
    for k, x in enumerate((rx-half_l, rx+half_l)):
        line(f"ReturnAisleEdge_{k}", (x, -AMR_BAY_Y+cut), (x, half_w if k else -half_w))
    line("ReturnAisleEnd", (rx-half_l, half_w), (rx+half_l, half_w))
    # Buffer lanes: the AMR crabs along the slot row with the car across it, from the west aisle
    # into the inbound lane and from the east aisle into the outbound lane.
    for side, label, head, tail in ((1, "In", gx, BUFFER_SLOT_X[-1]+half_w),
                                    (-1, "Out", rx, BUFFER_SLOT_X[0]-half_w)):
        y = side*AMR_BAY_Y
        inward = 1 if tail > head else -1
        fill(f"{label}Lane", min(head, tail), max(head, tail), y-half_l, y+half_l)
        for k, e in enumerate((-1, 1)):
            line(f"{label}LaneEdge_{k}", (head+inward*cut, y+e*half_l), (tail, y+e*half_l))
        line(f"{label}LaneEnd", (tail, y-half_l), (tail, y+half_l))
        cylinder(stage, path+f"/{label}Turn", (head, y, .0018), turn, .0016, ZONE_FILL)
        ring(stage, path+f"/{label}TurnEdge", (head, y, .004), turn+.03, turn-.03, .003, AMBER, 96)

def concrete_floor(stage, path):
    box(stage,path+"/Slab",(FLOOR_XMID,FLOOR_MID,-.108),(FLOOR_W,FLOOR_LEN,.2),CONCRETE)
    floor=mesh(stage,path+"/Concrete", [(FLOOR_X0,-FLOOR_S,0),(FLOOR_X1,-FLOOR_S,0),(FLOOR_X1,FLOOR_Y,0),(FLOOR_X0,FLOOR_Y,0)],
               [4],[0,1,2,3],CONCRETE)
    uv=UsdGeom.PrimvarsAPI(floor).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,"vertex")
    uv.Set([(0.,0.),(FLOOR_W/4,0.),(FLOOR_W/4,FLOOR_LEN/4),(0.,FLOOR_LEN/4)])
    shader=UsdShade.Shader.Get(stage,"/Looks/Concrete/Surface")
    coords=UsdShade.Shader.Define(stage,"/Looks/Concrete/UV")
    coords.CreateIdAttr("UsdPrimvarReader_float2")
    coords.CreateInput("varname",Sdf.ValueTypeNames.Token).Set("st")
    coords.CreateOutput("result",Sdf.ValueTypeNames.Float2)
    image=UsdShade.Shader.Define(stage,"/Looks/Concrete/Texture")
    image.CreateIdAttr("UsdUVTexture")
    image.CreateInput("file",Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./concrete.png"))
    image.CreateInput("wrapS",Sdf.ValueTypeNames.Token).Set("repeat")
    image.CreateInput("wrapT",Sdf.ValueTypeNames.Token).Set("repeat")
    image.CreateInput("sourceColorSpace",Sdf.ValueTypeNames.Token).Set("sRGB")
    image.CreateInput("st",Sdf.ValueTypeNames.Float2).ConnectToSource(coords.ConnectableAPI(),"result")
    image.CreateOutput("rgb",Sdf.ValueTypeNames.Float3)
    shader.GetInput("diffuseColor").ConnectToSource(image.ConnectableAPI(),"rgb")


def write_concrete(path):
    from PIL import Image
    from scipy.ndimage import gaussian_filter

    rng=np.random.default_rng(54161)
    base=rng.standard_normal((768,768))
    mottling=gaussian_filter(base,16,mode="wrap")
    grit=gaussian_filter(base,1.2,mode="wrap")
    shade=184+5*mottling/mottling.std()+2.3*grit/grit.std()+1.5*base  # light gray epoxy-sealed slab
    rgb=np.stack((shade+1,shade,shade-1),axis=-1)
    Image.fromarray(np.clip(rgb,0,255).astype(np.uint8)).save(path,optimize=True)


def tiered_feed(stage, label, side):
    """One outboard lane from the 1F loading deck to the B1 pick conveyor."""
    y = side*LINE_Y
    prefix = f"/World/Factory/{label}TieredFeed"
    for name, start, end in (("Ramp", LAYOUT.ramp_bottom_x, LAYOUT.ramp_top_x),
                              ("Upper", LAYOUT.ramp_top_x, LAYOUT.supply_x+.4)):
        z0, z1 = belt_height(start), belt_height(end)
        length = hypot(end-start, z1-z0)
        part = f"{prefix}/{name}"
        frame, _ = xform(stage, part, ((start+end)/2,y,(z0+z1)/2))
        frame.AddRotateYOp().Set(-degrees(atan2(z1-z0,end-start)))
        box(stage, part+"/Bed", (0,0,-.090), (length,.67,.060), RUBBER)
        for sign in (-1, 1):
            box(stage, part+f"/Edge_{sign+1}", (0,sign*.347,.04), (length,.040,.12), BRUSHED)
        count = ceil(length/.105)
        for i in range(count):
            x = ((i+.5)/count-.5)*length
            cylinder(stage, part+f"/Roller_{i}", (x,0,-ROLLER_RADIUS), ROLLER_RADIUS,.58,ZINC,"Y")
        for i in range(max(1,ceil((end-start)/1.25))):
            x = start+(i+.5)*(end-start)/max(1,ceil((end-start)/1.25))
            height = belt_height(x)-.12
            for sign in (-1,1):
                box(stage,prefix+f"/{name}Support_{i}_{sign+1}",(x,y+sign*.25,height/2),
                    (.08,.08,height),BRUSHED)
    box(stage,prefix+"/FirstFloorLoadingDeck",(LAYOUT.supply_x,y+side*.8,LAYOUT.floor_rise-.10),
        (1.1,.95,.20),CONCRETE)
    for sign in (-1,1):
        box(stage,prefix+f"/DeckSupport_{sign+1}",(LAYOUT.supply_x+sign*.44,y+side*1.10,
            LAYOUT.floor_rise/2),(.12,.12,LAYOUT.floor_rise),BRUSHED)


def factory(stage, arm):
    path="/World/Factory"
    concrete_floor(stage,path)
    for i,k in enumerate(range(int(FLOOR_X0//2.5)+1,ceil(FLOOR_X1/2.5)),1):   # every 2.5 m inside the edges
        box(stage,path+f"/FloorJoint_{i}",(k*2.5,FLOOR_MID,.001),(.007,FLOOR_LEN-.05,.002),(.17,.19,.20))
    joints,south=int(FLOOR_Y//2.5),int(FLOOR_S//2.5)
    for j in range(-south,joints+1):
        box(stage,path+f"/TransverseJoint_{j+south}",(FLOOR_XMID,j*2.5,.001),(FLOOR_W-.04,.007,.002),(.17,.19,.20))
    box(stage,path+"/DockPlate",(DOCK_X+WHEELBASE,0,-.012),(4.95+2*WHEELBASE,2.22,.024),(.15,.18,.20))
    for i in range(int((-2.69-(AMR_GATE_X+.47))/.48)+1):     # gate -> dock arrow
        box(stage,path+f"/AMR_Center_{i}",(AMR_GATE_X+.47+i*.48,0,.003),(.265,.045,.005),(.04,.54,.78))
    arrow_points=[(-2.62,0.,.004),(-3.06,.20,.004),(-3.06,.071,.004),
                  (-3.52,.071,.004),(-3.52,-.071,.004),(-3.06,-.071,.004),(-3.06,-.20,.004)]
    mesh(stage,path+"/DockDirection",arrow_points,[3,4],[0,1,6,2,3,4,5],(.04,.54,.78),True)
    amr_zones(stage,path+"/AMRZone")
    datums, _ = xform(stage, "/World/LayoutDatums")
    datums.GetPrim().CreateAttribute("cell:amrStopsX", Sdf.ValueTypeNames.DoubleArray).Set(list(LAYOUT.stops))
    datums.GetPrim().CreateAttribute("cell:floorRise", Sdf.ValueTypeNames.Double).Set(LAYOUT.floor_rise)
    for name, x in zip(("ParkA","IndexB","IndexC","Exit"), LAYOUT.stops):
        xform(stage, f"/World/LayoutDatums/{name}", (x,0,0))
        if name!="Exit":
            box(stage,f"{path}/AMR_{name}",(x,0,.004),(.06,2.10,.004),TEAL)
    for i in range(1,5):
        x = FINAL_X+(EXIT_X-FINAL_X)*i/5
        box(stage,f"{path}/AMR_ExitLine_{i}",(x,0,.004),(.30,.045,.004),TEAL)
    for i in range(int((RETURN_X-.24-EXIT_X)/.48)+1):      # forward exit -> east return aisle
        box(stage,f"{path}/AMR_Return_{i}",(EXIT_X+.24+i*.48,0,.005),(.265,.045,.005),(.04,.54,.78))
    for side, name in ((1,"Left"),(-1,"Right")):
        for role, x in (("Mount",MOUNT_HUB_X),("Fasten",FASTEN_HUB_X)):
            xform(stage, f"/World/LayoutDatums/{name}{role}", (x,side*LAYOUT.work_hub_y,LAYOUT.work_hub_z))

    cell_floor=(.36,.43,.52)  # inside-cell tint from the layout legend; the dock plate stays dark
    for name,(x0,x1,y0,y1) in (("Left",(-4.52,6.65,1.11,CELL_Y)),("Right",(-4.52,6.65,-CELL_Y,-1.11)),
                               ("Entry",(-4.52,DOCK_X-2.475,-1.11,1.11)),("Exit",(DOCK_X+2.475+WHEELBASE,5.65,-1.11,1.11))):
        box(stage,path+f"/CellFloor_{name}",((x0+x1)/2,(y0+y1)/2,.0015),(x1-x0,y1-y0,.002),cell_floor)
    for side,label in ((1,"Left"),(-1,"Right")):
        # Inbound (+y) runs on the west aisle, outbound (-y) on the east return aisle.
        ax,inward=(AMR_GATE_X,1) if side>0 else (RETURN_X,-1)
        for i in range(int((AMR_BAY_Y-.24)/.48)+1):
            box(stage,path+f"/AMR_{label}_{i}",(ax,side*(.24+i*.48),.005),
                (.044,.26,.005),(.04,.54,.78))
        # AMR loop between the buffer lane and its aisle, dashed as in the layout image.
        for i in range(4):
            box(stage,path+f"/AMR_{label}Loop_{i}",(ax+inward*(.30+i*.48),side*AMR_BAY_Y,.005),
                (.26,.044,.005),(.04,.54,.78))
        tip=[(ax+inward*2.38,side*AMR_BAY_Y,.006),(ax+inward*2.02,side*AMR_BAY_Y+.20,.006),
             (ax+inward*2.02,side*AMR_BAY_Y-.20,.006)]
        mesh(stage,path+f"/AMR_{label}LoopArrow",tip,[3],[0,1,2],(.04,.54,.78),True)
        parking(stage,path+f"/{label}VehicleBuffer",side)
        if side<0:
            charging_bays(stage,path+"/AMRCharging")
        mount_flange = fixed_robot(stage, f"{path}/{label}MountRobot", arm, MOUNT_BASE_X, side, "mount")
        mount_gripper(stage, mount_flange+"/PHL_W_63_080_S")
        fasten_flange = fixed_robot(stage, f"{path}/{label}FastenRobot", arm, FASTEN_BASE_X, side, "tighten")
        fasten_head(stage, fasten_flange+"/FastenHead")
        magazine(stage, f"{path}/{label}NutMagazine", side)
        # Enclose the outboard conveyors. Maintenance doors open onto the walkway.
        fy = side*CELL_Y
        fence(stage,path+f"/{label}InnerGuard",(-2.70,fy),(6.65,fy),1.80)
        fence(stage,path+f"/{label}ServiceGuard",(-4.52,fy),(-3.80,fy),1.80)
        door = path+f"/{label}MaintenanceGuard"
        fence(stage,door,(-3.79,fy),(-2.71,fy),1.80)
        box(stage,door+"/Interlock",(-2.74,fy+side*.05,1.42),(.10,.07,.13),AMBER)
        box(stage,door+"/Handle",(-2.83,fy+side*.07,1.05),(.045,.06,.20),DARK)
        for x, tag, opening in ((-4.52, "AMR", 1.50), (6.65, "Infeed", 1.16)):
            spans = ((opening, CELL_Y),) if tag=="AMR" else (
                (opening, LINE_Y-.48), (LINE_Y+.48, CELL_Y))
            for i,(lower,upper) in enumerate(spans):
                fence(stage,f"{path}/{label}{tag}Guard_{i}",(x,side*lower),(x,side*upper),1.80)
            box(stage,path+f"/{label}{tag}Guard_0/LightCurtainPost",(x,side*opening,.90),
                (.065,.065,1.80),AMBER)
        for x,tag in ((-3.94,"Service"),(6.44,"Supply")):
            station = path+f"/{label}{tag}GuardControl"
            box(stage,station+"/Post",(x,fy+side*.09,.62),(.07,.08,1.24),BRUSHED)
            box(stage,station+"/Plate",(x,fy+side*.14,1.15),(.12,.06,.16),AMBER)
            cylinder(stage,station+"/EmergencyStop",(x,fy+side*.19,1.18),.034,.04,RED,"Y")
        # Walkway between the cell fence and the vehicle buffer, as in the layout image.
        fence(stage,path+f"/{label}BufferGuard",(-4.52,side*BUFFER_GUARD_Y),(4.95,side*BUFFER_GUARD_Y))
        for k,y in enumerate((CELL_Y+.18,BUFFER_GUARD_Y-.18)):
            box(stage,path+f"/CellFloor_{label}Walkway_{k}",(.215,side*y,.004),(9.47,.06,.003),AMBER)
        fence(stage,path+f"/{label}OuterGuard",(-5.55,side*OUTER_GUARD_Y),(6.10,side*OUTER_GUARD_Y))
        # Outbound side: the end guard stops at the walkway so the outbound lane opens to the east aisle.
        fence(stage,path+f"/{label}EndGuard",(5.65,side*(CELL_Y+.02)),
              (5.65,side*(OUTER_GUARD_Y if side>0 else BUFFER_GUARD_Y)))
        if side>0:
            reject_gate=belt(stage,label+"Feed",(FEED_START_X,side*LINE_Y),(LAYOUT.ramp_bottom_x,side*LINE_Y),.67,outlet_x=REJECT_X)
        else:
            belt(stage,label+"Feed",(FEED_START_X,side*LINE_Y),(LAYOUT.ramp_bottom_x,side*LINE_Y),.67)
        tiered_feed(stage,label,side)
        for j in (-1,1):
            beacon(stage,path+f"/{label}Signal_{j+1}",(j*4.65,side*CELL_Y))
    belt(stage,"QA_Reject",(REJECT_X,LINE_Y+.38),(REJECT_X,REJECT_CRATE_Y-.22),.67)
    box(stage,path+"/RejectCrate",(REJECT_X,REJECT_CRATE_Y,.255),(.72,.42,.48),RED)
    industrial_hall(stage,path)
    for name,eye,target,focal in (
            ("Overview",(-17.6,0.,16.6),(.65,0.,.44),20.8),
            ("Assembly",(-6.0,-4.35,3.34),(.18,0.,1.0),26),
            ("WheelDetail",(DOCK_X+2.37,2.76,1.95),(DOCK_X+1.40,1.13,.82),30),
            ("HubDetail",(DOCK_X+1.94,2.13,1.53),(DOCK_X+AXLE,.81,.97),48),
            ("TopView",(0.,0.,22.),(0.,0.,0.),35)):
        cam=UsdGeom.Camera.Define(stage,f"/World/{name}")
        up=Gf.Vec3d(0,1,0) if name=="TopView" else Gf.Vec3d(0,0,1)
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(vector(eye),vector(target),up).GetInverse())
        if name=="TopView":
            cam.CreateProjectionAttr("orthographic")
            cam.CreateHorizontalApertureAttr(16/9*(2*FLOOR_Y+.6)*10)
            cam.CreateVerticalApertureAttr((2*FLOOR_Y+.6)*10)
        cam.CreateFocalLengthAttr(focal)
    light=UsdLux.DomeLight.Define(stage,"/World/Fill")
    light.CreateIntensityAttr(260)
    sun=UsdLux.DistantLight.Define(stage,"/World/Ambient")
    sun.CreateIntensityAttr(420)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(35,-23,0))
    for side,tag in ((1,"Left"),(-1,"Right")):
        for axle,kind in ((AXLE,"Front"),(-AXLE,"Rear")):
            spot=UsdLux.SphereLight.Define(stage,f"/World/Lighting/{tag}{kind}Inspection")
            spot.AddTranslateOp().Set(Gf.Vec3d(DOCK_X+axle+.11,side*1.13,.76))
            spot.CreateRadiusAttr(.035)
            spot.CreateIntensityAttr(14)
            spot.CreateColorAttr((.12,.43,1.))
    return reject_gate


def beacon(stage, root, xy):
    x,y=xy
    cylinder(stage,root+"/Post",(x,y,.69),.035,1.35,BRUSHED)
    for i,rgb in enumerate((TEAL,AMBER,RED)):
        cylinder(stage,root+f"/Signal_{i}",(x,y,1.38+i*.062),.045,.055,rgb)


def industrial_hall(stage, root):
    wall = (.51, .55, .58)
    bay = FLOOR_W/4
    for i in range(4):
        center = FLOOR_X0+(i+.5)*bay
        box(stage, root+f"/WallBase_{i}", (center, HALL_Y, .59), (bay-.02, .16, 1.18), CONCRETE)
        box(stage, root+f"/Cladding_{i}", (center, HALL_Y+.03, 2.20), (bay-.02, .12, 2.1), wall)
        box(stage, root+f"/Clerestory_{i}", (center, HALL_Y+.02, 3.83), (bay-.25, .11, 1.12), GLASS)
        for j in range(5):
            box(stage, root+f"/Mullion_{i}_{j}", (center+(j-2)*bay/5, HALL_Y-.05, 3.83), (.047, .09, 1.11), BRUSHED)
        box(stage, root+f"/UpperCladding_{i}", (center, HALL_Y+.03, 4.77), (bay-.02, .12, .74), wall)
    # End wall from the open front edge to the side wall, in equal bays.
    bays = 4
    y0, y1 = -FLOOR_S+.5, HALL_Y
    width = (y1-y0)/bays
    for i in range(bays):
        y = y0+(i+.5)*width
        box(stage, root+f"/EndWall_{i}", (FLOOR_X1-.06, y, 2.45), (.12, width-.02, 4.9), wall)
        for j in range(2):
            box(stage, root+f"/EndWindow_{i}_{j}", (FLOOR_X1-.14, y+(j-.5)*width*.39, 3.73),
                (.035, width*.37, .86), GLASS)
    for i in range(bays-1):
        box(stage, root+f"/EndUpright_{i}", (FLOOR_X1-.18, y0+(i+1)*width, 2.45), (.16, .17, 4.9), BRUSHED)
    for i, x in enumerate((FLOOR_X0+.2, *(FLOOR_X0+k*bay for k in (1, 2, 3)), FLOOR_X1-.2)):
        box(stage, root+f"/Column_{i}", (x, HALL_Y-.26, 2.61), (.28, .24, 5.2), BRUSHED)
        box(stage, root+f"/ColumnFoot_{i}", (x, HALL_Y-.26, .10), (.43, .37, .20), ZINC)
        box(stage, root+f"/RoofBeam_{i}", (x, HALL_Y-.52, 5.13), (.16, 1.24, .24), BRUSHED)
        for j, y in enumerate((-AMR_BAY_Y, -1.65, 1.65, AMR_BAY_Y)):
            fixture = f"/World/Lighting/Strip_{i}_{j}"
            rect = UsdLux.RectLight.Define(stage, fixture)
            rect.AddTranslateOp().Set(Gf.Vec3d(x, y, 4.789))
            rect.CreateWidthAttr(1.92)
            rect.CreateHeightAttr(.19)
            rect.CreateIntensityAttr(650 if i in (1, 2, 3) else 380)
    box(stage,root+"/RoofEdge",(FLOOR_XMID,HALL_Y-.39,5.16),(FLOOR_W-.25,.16,.23),BRUSHED)
    for i, x in enumerate((7.20, 7.92, 8.64, 9.36)):
        cabinet = root+f"/Panel_{i}"
        box(stage, cabinet+"/Housing", (x, -4.88, .96), (.66, .52, 1.86), PANEL)
        box(stage, cabinet+"/Door", (x, -4.606, .97), (.611, .026, 1.69), (.49, .53, .53))
        box(stage, cabinet+"/Latch", (x+.22, -4.576, 1.05), (.031, .035, .15), RUBBER)
        for j in range(5):
            box(stage, cabinet+f"/Vent_{j}", (x, -4.589, .52+j*.057), (.42, .016, .015), RUBBER)


def amr_housing(stage, root):
    outline=((1.53,.43),(1.46,.55),(.99,.59),(-.99,.59),(-1.46,.55),(-1.53,.43),
             (-1.53,-.43),(-1.46,-.55),(-.99,-.59),(.99,-.59),(1.46,-.55),(1.53,-.43))
    for name,levels,rgb in (
            ("ImpactSkirt",((.065,.94),(.10,1.),(.155,1.),(.180,.98)),RUBBER),
            ("Pan",((.146,.965),(.187,1.),(.252,1.),(.284,.96)),AMBER),
            ("UpperShell",((.275,.96),(.311,.93),(.344,.86),(.36,.80)),PANEL)):
        points=[(x*scale*AMR_SIZE[0]/3.06,y*scale*AMR_SIZE[1]/1.18,z*AMR_SIZE[2]/.36)
                for z,scale in levels for x,y in outline]
        n=len(outline)
        faces=[tuple(reversed(range(n))),tuple(range(3*n,4*n))]
        faces.extend((ring*n+i,ring*n+(i+1)%n,(ring+1)*n+(i+1)%n,(ring+1)*n+i)
                     for ring in range(3) for i in range(n))
        mesh(stage,root+"/"+name,points,[len(face) for face in faces],
             [i for face in faces for i in face],rgb,True)
    box(stage,root+"/PowerHatch",(.24,0,.461),(.45,.32,.017),BRUSHED)
    for i,x in enumerate((-.72,.72)):
        for side in (-1,1):
            cylinder(stage,root+f"/Traction_{i}_{side+1}",(x,side*.31,AMR_WHEEL_RADIUS),
                     AMR_WHEEL_RADIUS,.132,RUBBER,"Y")
            status=box(stage,root+f"/StatusBar_{i}_{side+1}",(x*1.32,side*.224,.338),
                       (.024,.123,.039),(.03,.43,.76))
            finish(status,(.03,.43,.76),("AMRStatus",0.,.24))
    for name, x, y, yaw in (("FrontLeft", SCANNER_X, SCANNER_Y, 45.),
                            ("RearRight", -SCANNER_X, -SCANNER_Y, -135.)):
        scanner, _ = xform(stage, root+f"/SafetyScanner_{name}", (x, y, SCANNER_Z))
        scanner.AddRotateZOp().Set(yaw)
        box(stage, root+f"/SafetyScanner_{name}/Housing", (0, 0, 0), S300_SIZE, GLASS)
    for i in range(6):
        box(stage,root+f"/Vent_{i}",(-.1-i*.061,0,.466),(.029,.29,.009),RUBBER)


def carrier(stage, assets, root):
    model, position = xform(stage, root)
    yaw = model.AddRotateZOp()
    yaw.Set(0.)
    amr = root+"/AMR"
    if assets.has_amr:
        add_instance(stage,amr,LIB+"/AMR")
    else:
        amr_housing(stage,amr)
    _, lift = xform(stage, root+"/Hanger")
    h = root+"/Hanger"
    for side in (-1,1):
        box(stage,h+f"/SaddleRail_{side+1}",(0,side*.29,.44),(3.60,.13,.14),BRUSHED)
    for i, x in enumerate((-1.65, 1.65)):
        box(stage, h+f"/Support_{i}", (x, 0, .51), (.25, .41, .62), BRUSHED)
        box(stage, h+f"/Crossmember_{i}", (x, 0, .83), (.31, .88, .072), AMBER)
        for side in (-1, 1):
            box(stage, h+f"/ChassisPad_{i}_{side+1}", (x, side*.40, .927),
                (.16, .10, .116), RUBBER)
    return model, position, yaw, lift


def vehicle_body(stage, h, number=1):
    add_instance(stage, h+"/SedanShell", LIB+"/Body")
    add_instance(stage, h+"/DoorDecal", decal_prototype(stage, decal_for(number)))
    box(stage, h+"/Undertray", (0, 0, 1.035), (3.43, 1.02, .115), DARK)
    beam_x=-AXLE+.14
    for i,x in enumerate((beam_x-.039,beam_x+.039)):
        box(stage,h+f"/TorsionBeamFlange_{i}",(x,0.,HUB_Z-.022),(.018,1.32,.074),BRUSHED)
    box(stage,h+"/TorsionBeamWeb",(beam_x,0.,HUB_Z-.056),(.093,1.32,.021),BRUSHED)
    box(stage, h+"/FrontSubframe", (1.36, 0, HUB_Z+.06), (.43, 1.23, .10), BRUSHED)
    for side, suffix in ((-1, "R"), (1, "L")):
        for n, x in enumerate((-.43, .43)):
            box(stage, h+f"/DoorHandle_{suffix}{n}", (x, side*.858, 1.574),
                (.128, .026, .023), ALLOY)
    box(stage, h+"/Grille", (2.337, 0, 1.37), (.047, 1.15, .13), RUBBER)
    for i in range(5):
        box(stage, h+f"/GrilleSlat_{i}", (2.364, 0, 1.331+i*.022), (.012, 1.11, .006), BRUSHED)
    for name,(x,side) in STATIONS.items():
        hub = h+"/"+name
        y = half_track(x)
        link,_ = xform(stage,hub,(x,side*y,HUB_Z))
        link.AddRotateXOp().Set(-side*90)
        prototype=LIB+f"/Corner_{name}"
        if not stage.GetPrimAtPath(prototype):
            UsdGeom.Xform.Define(stage,prototype)
            make_hub(stage,prototype,x>0,side,cylinder,box)
        link.GetPrim().GetReferences().AddInternalReference(prototype)
        # The per-vehicle nuts remain separately animated children of this frame.
        wheelhouse(stage,h+f"/Wheelhouse_{name}",x,side,HUB_Z,y)


def make_wheel(stage, assets, path, wheel, damaged=False):
    item, position = xform(stage, path)
    tilt = item.AddRotateXOp()
    spin = item.AddRotateZOp()
    tilt.Set(0)
    spin.Set(0)
    spec = {"tireWidthM":wheel.width, "tireDiameterM":wheel.radius*2,
            "rimBeadDiameterM":wheel.inner*2, "rimBeadWidthM":wheel.rim_width,
            "centerBoreM":BORE, "boltCircleM":PCD, "offsetM":OFFSET}
    if wheel==CN7_R16:
        spec["massKg"]=WHEEL_MASS
    for key,value in spec.items():
        item.GetPrim().CreateAttribute("cell:"+key,Sdf.ValueTypeNames.Double).Set(value)
    prototype=LIB+f"/Wheel_{wheel.inches}"+("_Rejected" if damaged else "")
    if not stage.GetPrimAtPath(prototype):
        UsdGeom.Xform.Define(stage,prototype)
        wheel_visual(stage,assets,prototype,wheel,damaged)
    item.GetPrim().GetReferences().AddInternalReference(prototype)
    item.GetPrim().SetInstanceable(True)
    return item, position, tilt, spin


def wheel_visual(stage, assets, path, wheel, damaged):
    assets.tire_instance(path+"/Tire",wheel,damaged)
    barrel(stage,path+"/Barrel",wheel)
    center_web(stage,path+"/CenterWeb")
    for i,a in enumerate(BOLT_ANGLES):
        a += tau/10
        spoke, _ = xform(stage, path+f"/Spoke_{i}")
        spoke.AddRotateZOp().Set(degrees(a))
        for j, offset in enumerate((-.010, .010)):
            edge = wheel.inner-.009
            points = [(r, offset+shift+y, z) for r, shift, z in
                      ((.076, 0, .049), (edge, offset, wheel.rim_width/2-.007)) for y in (-.007, .007)]
            points += [(r, y, z+.011) for r, y, z in points]
            mesh(stage, path+f"/Spoke_{i}/Blade_{j}", points, [4]*6,
                 [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6,
                  0, 2, 6, 4, 1, 5, 7, 3], ALLOY, double=True)
    cylinder(stage, path+"/ValveStem", (-.88*wheel.inner, .049, .083), .005, .017, RUBBER)


def rs080n_library(stage):
    """RS080N link meshes (link frames, zero pose) as visual-only prototypes, no physics APIs."""
    source=Usd.Stage.Open(str(RS080N_ASSETS/"RS080N.usdc"))
    for i,name in enumerate(["base_link"]+[f"link_{k}" for k in range(1,7)]):
        housing=source.GetPrimAtPath(f"/RS080N/{name}/Housing")
        if not housing:
            raise FileNotFoundError(f"RS080N.usdc has no /RS080N/{name}/Housing")
        UsdGeom.Xform.Define(stage,f"{LIB}/rs080n_Link_{i}")
        copy_mesh(stage,f"{LIB}/rs080n_Link_{i}/Housing",housing,Gf.Matrix4d(1),KAWASAKI_GRAY)


ROBOT_LINKS = {}   # robot prim path -> [(translate op, orient op)] of Link_1..6


def fixed_robot(stage, path, arm, base_x, side, role):
    """Place the six RS080N links at their parked joint angles, facing the car."""
    node,_=xform(stage,path,(base_x,side*ROBOT_BASE_Y,0.))
    node.AddRotateZOp().Set(180. if side>0 else 0.)
    prim=node.GetPrim()
    prim.CreateAttribute("cell:role",Sdf.ValueTypeNames.String).Set(role)
    prim.CreateAttribute("cell:model",Sdf.ValueTypeNames.String).Set("RS080N")
    prim.CreateAttribute("cell:fixedBase",Sdf.ValueTypeNames.Bool).Set(True)
    add_instance(stage,path+"/Base/Housing",f"{LIB}/rs080n_Link_0")
    ops=[]
    for i,(p,rot,_) in enumerate(arm.fk((0.,0.,0.),RS080N_PARK),1):
        link,position=xform(stage,path+f"/Link_{i}",p)
        qx,qy,qz,qw=Rotation.from_matrix(rot).as_quat()
        orient=link.AddOrientOp()
        orient.Set(Gf.Quatf(qw,qx,qy,qz))
        ops.append((position,orient))
        add_instance(stage,path+f"/Link_{i}/Housing",f"{LIB}/rs080n_Link_{i}")
    ROBOT_LINKS[path]=ops
    return path+"/Link_6"


def animate_arm(stage, arm, path, plan):
    """Key the six links (robot frame) along a joint plan; joint targets go on cell:jointTargetDeg."""
    for seconds,q in samples(plan):
        for i,((position,orient),(p,rot,_)) in enumerate(zip(ROBOT_LINKS[path],arm.fk((0.,0.,0.),q)),1):
            move(position,seconds,p)
            qx,qy,qz,qw=Rotation.from_matrix(rot).as_quat()
            orient.Set(Gf.Quatf(qw,qx,qy,qz),frame(seconds))
            stage.GetPrimAtPath(path+f"/Link_{i}").CreateAttribute(
                "cell:jointTargetDeg",Sdf.ValueTypeNames.Double).Set(degrees(q[i-1]),frame(seconds))


def mount_gripper(stage, path):
    """Parked visual of the PHL-W 63-080-S, with the two shoes at the tread diameter."""
    box(stage, path+"/Adapter", (0,0,.010), (.22,.16,.020), ALLOY)
    box(stage, path+"/Housing", (0,0,.08435), (.541,.164,.1287), BLUE)
    for side, name in ((-1, "Left"), (1, "Right")):
        jaw = path+"/"+name
        box(stage, jaw+"/Slide", (side*.267,0,.1612), (.10,.12,.025), BRUSHED)
        box(stage, jaw+"/Finger", (side*.324,0,.276), (.025,.10,.22), ALLOY)
        box(stage, jaw+"/Shoe", (side*.31,0,.32), (.012,.090,.155), RUBBER)


def fasten_head(stage, head):
    """Compact five-spindle head of map_fasten (build_head, e32662d) in the flange frame, +Z to the
    sockets: adapter r 55 mm, main plate r 81 mm, housings r 20 mm, socket tip at HEAD_LENGTH 235 mm.
    The D405, floating stroke and reaction pins are left out as in map_fasten."""
    rotor=HEAD_LENGTH-.048
    cylinder(stage,head+"/FlangeAdapter",(0,0,.015),.055,.030,ALLOY)
    cylinder(stage,head+"/MainPlate",(0,0,.070),.081,.080,ALLOY)
    for i,(sx,sy) in enumerate(SOCKET_LAYOUT):
        cylinder(stage,head+f"/Housing_{i}",(sx,sy,(.110+rotor-.007)/2),.020,rotor-.007-.110,DARK)
        cylinder(stage,head+f"/Spindle_{i}",(sx,sy,rotor),.013,.018,BRUSHED)
        cylinder(stage,head+f"/Socket_{i}",(sx,sy,rotor+.028),.016,.040,AMBER)


def magazine(stage, path, side):
    px,py,pz=MAGAZINE[0],side*MAGAZINE[1],MAGAZINE[2]
    box(stage,path+"/Stand",(px,py,(pz-.12)/2),(.10,.10,pz-.12),BRUSHED)
    # Nut presenter: five outlets on the stud pattern facing up, filled for one wheel per pick.
    # The head comes down with cell_motion.FACE_DOWN, so socket (sx, sy) lands on (px+sx, py-sy).
    box(stage,path+"/Cassette",(px,py,pz-.065),(.17,.17,.11),PANEL)
    for i,(sx,sy) in enumerate(SOCKET_LAYOUT):
        cylinder(stage,path+f"/Outlet_{i}",(px+sx,py-sy,pz-.018),.020,.041,BRUSHED,"Z")


def dock_clamps(stage, root, dock, release, *, base_x=0., prefix="DockClamp"):
    """Animate lateral locating contacts on the cell floor, clear before travel."""
    for i,x in enumerate((-.40,.40)):
        for side in (-1,1):
            path=root+f"/{prefix}_{i}_{side+1}"
            clamp_x=base_x+x
            box(stage,path+"/Base",(clamp_x,side*.70,.08),(.35,.19,.148),BRUSHED)
            _,slide=xform(stage,path+"/Slide")
            box(stage,path+"/Slide/Jaw",(0,0,.22),(.13,.057,.105),AMBER)
            box(stage,path+"/Slide/Pad",(0,-side*.032,.22),(.09,.016,.074),ALLOY)
            cylinder(stage,path+"/Slide/Pin",(0,side*.055,.22),.020,.086,ZINC,"Y")
            for t,y in ((0.,DOCK_CLAMP_OPEN),(dock,DOCK_CLAMP_OPEN),(dock+2.,DOCK_CLAMP_CLOSED),
                        (release-2.,DOCK_CLAMP_CLOSED),(release,DOCK_CLAMP_OPEN)):
                move(slide,t,(clamp_x,side*y,0))


def order(stage, assets, arm, data, record):
    number,wheel,start,deliveries,reject,front,rear,rear_tight,front_tight,second_index,final_dock,departure,end,lane_head=data
    vehicle=f"CN7-{number:03}"
    root=f"/World/Orders/CN7_{number:03}"
    cycle,_=xform(stage,root)
    cycle.GetPrim().CreateAttribute("cell:gateTimeS",Sdf.ValueTypeNames.Double).Set(start)
    slot_x=BUFFER_SLOT_X[number-1]
    staged=start-inbound_pickup(slot_x)[-1][0]
    pickup=inbound_pickup(slot_x,staged)
    dropoff=outbound_dropoff(slot_x,lane_head)
    lift_off=pickup[2][0]
    parked=dropoff[3][0]
    cleared=dropoff[-1][0]
    amr_model,amr,amr_yaw,lift=carrier(stage,assets,root+"/Carrier")
    visibility(amr_model,staged,cleared)
    car_path=root+"/Vehicle"
    car,car_pos=xform(stage,car_path,(slot_x,AMR_BAY_Y,BUFFER_CAR_Z))
    car_yaw=car.AddRotateZOp()
    car_yaw.Set(-90.)
    move(car_pos,0.,(slot_x,AMR_BAY_Y,BUFFER_CAR_Z))
    car_yaw.Set(-90.,0.)
    vehicle_body(stage,car_path,number)
    index_start=front+MOUNT_CLEAR
    redock=index_start+INDEX_TRAVEL
    dock_clamps(stage,root,start+AMR_DOCKED,index_start,base_x=DOCK_X)
    dock_clamps(stage,root,redock,second_index,base_x=INDEX_X,prefix="RearDockClamp")
    dock_clamps(stage,root,final_dock,departure,base_x=FINAL_X,prefix="FinalDockClamp")
    cell_route=((start+AMR_DOCKED,DOCK_X,0.,0.,AMR_GATE_LIFT),
                (start+AMR_LIFTED,DOCK_X,0.,0.,0.),
                (index_start,DOCK_X,0.,0.,0.),(redock,INDEX_X,0.,0.,0.),
                (second_index,INDEX_X,0.,0.,0.),(final_dock,FINAL_X,0.,0.,0.),
                (departure,FINAL_X,0.,0.,0.),(end,EXIT_X,0.,0.,0.),
                (end+AMR_TO_RETURN,RETURN_X,0.,0.,0.))
    for t,x,y,yaw,height in (*pickup,*cell_route,*dropoff):
        move(amr,t,(x,y,0.))
        amr_yaw.Set(yaw,frame(t))
        move(lift,t,(0.,0.,height))
        # The vehicle stays on its stands until contact, then follows the lift until set-down.
        if lift_off<=t<=parked:
            move(car_pos,t,(x,y,height))
            car_yaw.Set(yaw,frame(t))
    record(start,vehicle,"","order",wheel.name)
    record(start+AMR_LIFTED,vehicle,"","docked","AMR dock and lift complete")
    record(index_start,vehicle,"","index_start","Front wheels hung, mount arms parked; moving to stop B")
    record(redock+2.,vehicle,"","rear_docked","Stop B: front hubs at the fastening arms, rear hubs at the mount arms")
    record(second_index,vehicle,"","second_index","Front fastening and rear mounting complete; arms parked")
    record(final_dock,vehicle,"","final_docked","Stop C: rear hubs at the fastening arms")
    record(departure,vehicle,"","depart","AMR leaving the dock")
    record(end,vehicle,"","exit","AMR reached forward exit; cell free")
    record(parked,vehicle,"","outbound",f"Vehicle placed on outbound stands in slot {number}")
    record(staged,vehicle,"","staged",f"AMR approaches inbound slot {number}")
    for name,(x,side) in STATIONS.items():
        wheel_y=side*half_track(x)
        seated,tight=(front,front_tight) if x>0 else (rear,rear_tight)
        label="Left" if side>0 else "Right"
        mount=f"/World/Factory/{label}MountRobot"
        fasten=f"/World/Factory/{label}FastenRobot"
        item,pos,_,_=make_wheel(stage,assets,root+f"/SupplyWheel_{name}",wheel)
        path=deliveries[name]
        # Flat on the belt until the mount arm's jaws close on the tread.
        visibility(item,path[0][0],seated+GRIP_AT)
        for t,(wx,wy) in path:
            move(pos,t,(wx,wy,belt_height(wx)+wheel.width/2))
        # Carried in the PHL-W jaws: wheel axis on the flange Z, outboard face toward the flange.
        held,held_pos,held_tilt,_=make_wheel(stage,assets,mount+f"/Link_6/HeldWheel_{number:03}_{name}",wheel)
        held_pos.Set(vector((0,0,GRIP_CENTRE)))
        held_tilt.Set(180)
        visibility(held,seated+GRIP_AT,seated)
        animate_arm(stage,arm,mount,solve_plan(arm,MOUNT_BASE_X,side,mount_keys(side,seated,wheel)))
        animate_arm(stage,arm,fasten,solve_plan(arm,FASTEN_BASE_X,side,fasten_keys(side,tight)))
        for k,(sx,sy) in enumerate(SOCKET_LAYOUT):
            carried_root=fasten+f"/Link_6/CarriedNut_{number:03}_{name}_{k}"
            carried,_=xform(stage,carried_root,(sx,sy,HEAD_LENGTH+NUT_LENGTH))
            carried.AddRotateXOp().Set(180.)
            make_nut(stage,carried_root)
            visibility(carried,tight+NUT_PICK_AT,tight+NUT_ENGAGE)
        installed,mounted,mounted_tilt,_=make_wheel(stage,assets,car_path+f"/Wheel_{name}",wheel)
        mounted.Set(vector((x,wheel_y,HUB_Z)))
        for name_key,value in fit(wheel).items():
            installed.GetPrim().CreateAttribute("cell:clearance_"+name_key,Sdf.ValueTypeNames.Double).Set(value)
        mounted_tilt.Set(-side*90)
        visibility(installed,seated)
        record(path[-1][0],vehicle,name,"inspect","Simulated wheel inspection")
        record(seated-1,vehicle,name,"aligned",
               f"Nominal static envelope: pilot clearance {fit(wheel)['pilot_m']*1000:.2f} mm radial")
        record(seated,vehicle,name,"seated","Hung on the hub studs by the mount RS080N, jaws open")
        # Fastening steps of map_fasten (FastenState) on the keyframe clock.
        record(tight+1.94,vehicle,name,"tool_aligned","Head centre, axis and 5-nut pattern aligned 80 mm off the nuts")
        record(tight+NUT_ENGAGE,vehicle,name,"engaged","Five nuts at the stud tips (geometric entry)")
        record(tight+NUT_ENGAGE+NUT_DRIVE+NUT_FINAL+.1,vehicle,name,"released","Socket retention released, nuts stay seated")
        t=tight+NUT_ENGAGE
        for index,(bx,by) in enumerate(BOLT_CENTERS):
            nut_root=car_path+f"/{name}/Nut_{index}"
            nut,nut_pos=xform(stage,nut_root)
            nut_spin=nut.AddRotateZOp()
            visibility(nut,t)
            make_nut(stage,nut_root)
            for seconds,distance,spin_angle in ((t,NUT_ADVANCE,0.),(t+NUT_DRIVE,0.,1800.),
                                                (t+NUT_DRIVE+NUT_FINAL,0.,1830.)):
                move(nut_pos,seconds,(bx,by,NUT_SEAT+distance))
                nut_spin.Set(spin_angle,frame(seconds))
            record(t+NUT_DRIVE+NUT_FINAL,vehicle,name,"tightened",f"Nut {index+1}; virtual {DEMO_TORQUE:.0f} N m (accept {NUT_TORQUE[0]}–{NUT_TORQUE[1]})")
        record(tight+NUT_CYCLE+2,vehicle,name,"complete","Five nuts in one multi-spindle cycle, head withdrawn")
    # Order-specific tooling is visible only while this order occupies the cell.
    for child in cycle.GetPrim().GetChildren():
        if child.GetName()!="Vehicle" and not UsdGeom.Imageable(child).GetVisibilityAttr().HasAuthoredValue():
            visibility(UsdGeom.Xformable(child),start,end)
    return cleared


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"output",
                        help="Folder for build intermediates and the physics setup export")
    parser.add_argument("--scene", type=Path, default=Path(__file__).resolve().parent.parent/"scene",
                        help="Layered scene folder; overrides/ inside it is never overwritten")
    parser.add_argument("--inject-defect", action="store_true", help="Reject and replace first FL wheel")
    parser.add_argument("--fasten-urdf", "--urdf", type=Path, default=RS080N_ASSETS/"rs080n.urdf")
    parser.add_argument("--mixed-wheel-sizes", action="store_true", help="Include R15 and R17 demo orders")
    parser.add_argument("--asset-root", type=Path,
                        default=Path.home()/"cobot3_ws/isaacpjt")
    parser.add_argument("--tool-usd", "--robot-usd", dest="tool_usd", type=Path,
                        help="Assembled local USD used as the D455 tool reference")
    parser.add_argument("--amr-usd", type=Path, help="Replacement AMR render asset")
    args = parser.parse_args()
    fasten_arm = Arm(args.fasten_urdf)
    if fasten_arm.model != "khi_rs080n":
        parser.error("The robot URDF must match the bundled RS080N link meshes.")
    stage = Usd.Stage.CreateInMemory()
    world, _ = xform(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetFramesPerSecond(FPS)
    stage.SetStartTimeCode(0)
    UsdGeom.Scope.Define(stage,"/World/Orders")
    assets = VisualAssets(stage, args.asset_root, args.tool_usd, args.amr_usd)
    for key,value in {"bodyLengthM":BODY_LENGTH,"bodyWidthM":BODY_WIDTH,"wheelbaseM":WHEELBASE,
                      "hubPilotDiameterM":PILOT,"studDiameterM":STUD_DIAMETER,"studPitchM":PITCH,
                      "studProjectionM":STUD_PROJECTION,"lugThroatM":LUG_HOLE,
                      "minNutTorqueNm":NUT_TORQUE[0],"maxNutTorqueNm":NUT_TORQUE[1]}.items():
        world.GetPrim().CreateAttribute("cell:"+key,Sdf.ValueTypeNames.Double).Set(value)
    rs080n_library(stage)
    reject_gate=factory(stage,fasten_arm)
    events = []

    def record(seconds, vehicle, station, code, detail):
        events.append((round(seconds, 3), vehicle, station, code, detail))

    first_gate=inbound_pickup(BUFFER_SLOT_X[0])[-1][0]
    for data in schedule(args.inject_defect, args.mixed_wheel_sizes, start=first_gate):
        margins=fit(data[1])
        if min(margins.values())<=0:
            raise ValueError(f"Nominal wheel interference for {data[1].name}: {margins}")
        end_time=order(stage, assets, fasten_arm, data, record)
        reject=data[4]
        if reject:
            closed=reject_gate.Get()
            opened=Gf.Vec3d(closed[0],closed[1],.405)
            turn=reject[-2][0]
            clear=reject[-1][0]
            for time,position in ((0.,closed),(turn-1.,closed),(turn-.2,opened),
                                  (clear,opened),(clear+.8,closed)):
                move(reject_gate,time,position)
    stage.SetEndTimeCode(frame(end_time))
    args.output.mkdir(parents=True, exist_ok=True)
    write_concrete(args.output/"concrete.png")
    for name in DECAL_FILES:
        shutil.copy2(DECAL_SOURCE.parent/name, args.output/name)
    usd = args.output/"wheel_cell.usdc"
    if not stage.GetRootLayer().Export(str(usd)):
        raise OSError(f"Could not export {usd}")
    # The scene is delivered as a root layer with one sublayer per area; the
    # monolithic file is only the intermediate the split is made from.
    from split_scene import split
    usd_root, parts = split(usd, args.scene)
    usd.unlink()
    db = args.scene/"production.sqlite3"
    with sqlite3.connect(db) as connection:
        connection.execute("CREATE TABLE IF NOT EXISTS events ("
                           "sim_s REAL, vehicle TEXT, station TEXT, code TEXT, detail TEXT)")
        connection.execute("DELETE FROM events")
        connection.executemany("INSERT INTO events VALUES (?, ?, ?, ?, ?)", sorted(events))
    print(f"{usd_root} ({len(parts)} layers, {end_time:.1f}s), {db} ({len(events)} events)")


if __name__ == "__main__":
    main()
