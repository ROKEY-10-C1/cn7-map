"""Shared world layout for stage generation and force-limited AMR travel (metres)."""

from dataclasses import dataclass
import json
from math import isfinite
import os
from pathlib import Path

from fitment import BODY_LENGTH, WHEELBASE


@dataclass(frozen=True)
class Layout:
    # Provisional design values, configurable with CELL_LAYOUT_CONFIG.
    dock_x: float = -.45
    exit_x: float = 7.20
    # East return aisle: past the forward exit the loaded AMR keeps going +x to this x, then crabs -y
    # to the outbound buffer lane (mirror of the west aisle at process.AMR_GATE_X). The car body
    # (x +-2.355) clears the 1F loading deck (supply_x+0.55 = 10.20) by 0.545 m.
    return_x: float = 13.10
    floor_rise: float = 2.40
    ramp_bottom_x: float = 4.71
    ramp_top_x: float = 9.05
    supply_x: float = 9.65
    minimum_clearance: float = .08
    work_hub_y: float = .79525
    work_hub_z: float = .94

    @property
    def stops(self):
        return (self.dock_x, self.dock_x + WHEELBASE,
                self.dock_x + 2 * WHEELBASE, self.exit_x)


def load_layout():
    source = os.environ.get("CELL_LAYOUT_CONFIG")
    layout = Layout(**json.loads(Path(source).read_text())) if source else Layout()
    # 4.62: the ramp must start downstream of the reject gate (process.REJECT_X 4.25 + gate half 0.365);
    # the value is repeated here because process imports this module.
    if (not all(isfinite(value) for value in vars(layout).values()) or
        layout.exit_x <= layout.stops[2] or layout.floor_rise <= 0 or
        layout.return_x <= layout.exit_x or
        layout.return_x-BODY_LENGTH/2 <= layout.supply_x+.55+layout.minimum_clearance or
        not 4.62 < layout.ramp_bottom_x < layout.ramp_top_x < layout.supply_x or
        layout.minimum_clearance < 0 or layout.work_hub_y <= 0 or layout.work_hub_z <= 0):
        raise ValueError("Invalid layout spacing, rise or clearance")
    return layout


LAYOUT = load_layout()


def check_clearance(stage):
    """Conservative swept boxes for the loaded vehicle and factory geometry."""
    from pxr import Usd, UsdGeom
    from fitment import BODY_LENGTH, BODY_WIDTH, REAR_TRACK
    from physical_data import AMR_SIZE
    from process import AMR_BAY_Y, AMR_GATE_X, BUFFER_SLOT_X, WHEELS

    half_x = max(BODY_LENGTH, AMR_SIZE[0])/2
    # Include the widest catalog tire and 20 mm of outboard placement tolerance.
    half_y = max(BODY_WIDTH/2, AMR_SIZE[1]/2,
                 REAR_TRACK/2 + max(wheel.width for wheel in WHEELS)/2 + .02)
    # Upper bound for the carried vehicle and hanger above the B1 floor.
    height = 2.25
    turn = (half_x**2+half_y**2)**.5   # in-place turn at the outbound lane head
    lane_end = BUFFER_SLOT_X[0]
    # (start, end, half sizes): yaw 0 along x, the lane leg with the car across (yaw +90).
    paths = (((AMR_GATE_X,AMR_BAY_Y), (AMR_GATE_X,0.), (half_x,half_y)),
             ((AMR_GATE_X,0.), (LAYOUT.return_x,0.), (half_x,half_y)),
             ((LAYOUT.return_x,0.), (LAYOUT.return_x,-AMR_BAY_Y), (half_x,half_y)),
             ((LAYOUT.return_x,-AMR_BAY_Y), (LAYOUT.return_x,-AMR_BAY_Y), (turn,turn)),
             ((LAYOUT.return_x,-AMR_BAY_Y), (lane_end,-AMR_BAY_Y), (half_y,half_x)))
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    closest={}
    conflicts=[]
    for part in Usd.PrimRange(stage.GetPrimAtPath("/World/Factory")):
        path=str(part.GetPath())
        name=path.split("/")[3] if path.count("/")>=3 else ""
        if not any(token in name for token in
                   ("Feed","Guard","Reject","Column","Wall","EndUpright","Robot","Magazine","Signal","Panel")):
            continue
        if not part.IsA(UsdGeom.Gprim):
            continue
        bounds=cache.ComputeWorldBound(part).ComputeAlignedRange()
        if bounds.IsEmpty() or bounds.GetMax()[2]<=0 or bounds.GetMin()[2]>=height:
            continue
        lo, hi=bounds.GetMin(), bounds.GetMax()
        group="conveyor" if any(token in name for token in ("Feed","Reject")) else "factory"
        for a,b,(hx,hy) in paths:
            x0,x1=min(a[0],b[0])-hx,max(a[0],b[0])+hx
            y0,y1=min(a[1],b[1])-hy,max(a[1],b[1])+hy
            clearance=max(x0-hi[0],lo[0]-x1,y0-hi[1],lo[1]-y1)
            if group not in closest or clearance < closest[group][0]:
                closest[group]=(clearance,path)
            if clearance < LAYOUT.minimum_clearance:
                conflicts.append((round(clearance,4),path))
    return (2*half_x,2*half_y,height), closest, sorted(set(conflicts))
