"""Extend the feed belts of the team scene to the fixed V end stop (values in cell_layout.py).

Writes into scene/overrides/factory.usda, the factory hand-edit layer that a scene rebuild keeps:
- continues each station belt to -x down to the end stop (bed, side rails, rollers, legs as the team belt),
- adds the fixed V end stop (UHMW plates on a steel bracket) at the belt end.
Only these prims are authored; other edits in the layer stay. Re-running gives the same result.
Run: ./wheel_mount_sim/tools/pxrpy.sh wheel_mount_sim/tools/extend_belt_end.py
"""
import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import cell_layout as L  # noqa: E402

SCENE = L.SCENE_DIR/"wheel_cell.usda"
OUT = L.SCENE_DIR/"overrides"/"factory.usda"


def cube(stage, path, centre, scale, look, yaw=None, colour=None):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.)
    c.AddTranslateOp().Set(Gf.Vec3d(*centre))
    if yaw is not None:
        c.AddRotateZOp().Set(float(yaw))
    c.AddScaleOp().Set(Gf.Vec3f(*scale))
    if colour:
        c.CreateDisplayColorAttr([colour])
    if look:
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(look)))
    return c


def stretch(stage, path, x0):
    """Extend a box along x so it starts at x0 (keeps its +x end)."""
    prim = stage.GetPrimAtPath(path)
    t = prim.GetAttribute("xformOp:translate").Get()
    s = prim.GetAttribute("xformOp:scale").Get()
    x1 = t[0]+s[0]/2
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d((x0+x1)/2, t[1], t[2]))
    prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(x1-x0, s[1], s[2]))


def main():
    layer = Sdf.Layer.FindOrOpen(str(OUT))
    root = Sdf.Layer.FindOrOpen(str(SCENE))
    assert layer and "./overrides/factory.usda" in list(root.subLayerPaths)
    stage = Usd.Stage.Open(str(SCENE))
    stage.SetEditTarget(Usd.EditTarget(layer))
    for side, label in ((1, "Left"), (-1, "Right")):
        y0 = side*L.LINE_Y
        feed = f"/World/Factory/{label}Feed"
        stretch(stage, feed+"/Bed", L.FRAME_END_X)
        for prim in stage.GetPrimAtPath("/World/Factory").GetChildren():
            n = prim.GetName()
            if n.startswith(label+"Feed_rail"):
                t = prim.GetAttribute("xformOp:translate").Get()
                s = prim.GetAttribute("xformOp:scale").Get()
                if t[0]-s[0]/2 < .0:                     # the pieces that ended at the old belt end
                    stretch(stage, prim.GetPath(), L.FRAME_END_X)
        src = stage.GetPrimAtPath(f"/World/Factory/{label}Feed_roller0")
        for k, x in enumerate(L.new_roller_x()):
            r = UsdGeom.Cylinder.Define(stage, f"/World/Factory/{label}Feed_rollerX{k}")
            for name in ("radius", "height", "axis", "extent", "primvars:displayColor"):
                r.GetPrim().CreateAttribute(name, src.GetAttribute(name).GetTypeName()).Set(src.GetAttribute(name).Get())
            r.AddTranslateOp().Set(Gf.Vec3d(x, y0, src.GetAttribute("xformOp:translate").Get()[2]))
            UsdShade.MaterialBindingAPI.Apply(r.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath("/Looks/Zinc")))
        leg = stage.GetPrimAtPath(feed+"/Leg_0_0")
        foot = stage.GetPrimAtPath(feed+"/Foot_0_0")
        for i, x in enumerate(((.545+L.FRAME_END_X)/2, L.FRAME_END_X+.04)):
            for j, dy in enumerate((-.26, .26)):
                for src_, name in ((leg, "Leg"), (foot, "Foot")):
                    t = src_.GetAttribute("xformOp:translate").Get()
                    cube(stage, f"{feed}/{name}_X{i}_{j}", (x, y0+dy, t[2]), src_.GetAttribute("xformOp:scale").Get(),
                         "/Looks/BrushedSteel" if name == "Leg" else "/Looks/Zinc")
        # fixed V end stop: steel bracket across the frame end, two UHMW plates on the V legs
        end = f"/World/Factory/{label}AccumEnd"
        UsdGeom.Xform.Define(stage, end)
        cube(stage, end+"/Bracket", (L.FRAME_END_X+.015, y0, L.BELT_Z+.08), (.03, .64, .18), "/Looks/BrushedSteel")
        for k, (c, yaw, s) in enumerate(L.stop_plates(side)):
            cube(stage, f"{end}/Plate_{k}", c, s, None, yaw, (.92, .92, .88))
    layer.Save()
    print("wrote", OUT, "pick x", round(L.PICK_X, 3), "apex", round(L.APEX_X, 3), "frame end", round(L.FRAME_END_X, 3),
          "rollers", len(L.new_roller_x()), "guides from", round(L.GUIDE_START_X, 3))


if __name__ == "__main__":
    main()
