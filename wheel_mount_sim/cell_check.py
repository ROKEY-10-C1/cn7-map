"""Check the wheel_mount_sim mount arms inside the live wheel cell: they stand in the cell's PARK pose,
hold it, and touch nothing. Run with cell_check.sh (Kit app of wheel_cell/check_physics).

Prints MOUNTCHECK lines (drift of the flange from the PARK forward kinematics, jaw positions, contacts of
the arm/tool/jaws with anything else) and exits 0 when all pass.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import omni.kit.app
from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom

HERE = Path(__file__).resolve().parent
CELL = HERE.parent/"wheel_cell"
sys.path.insert(0, str(CELL))
sys.dont_write_bytecode = True

from physics import Cell, HZ, ROOT  # noqa: E402

sys.path.insert(0, str(HERE))
import cell_mount  # noqa: E402
import scene as S  # noqa: E402


def world(stage, path):
    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(m.ExtractRotationMatrix()).T, np.array(m.ExtractTranslation())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", type=Path, default=HERE.parent/"scene"/"wheel_cell.usda")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--seconds", type=float, default=8.)
    a = ap.parse_args()
    stage = Usd.Stage.Open(str(a.stage))
    stage.SetEditTarget(stage.GetSessionLayer())
    cell = Cell(stage, a.seed, True, start="gate")
    arms = cell.mount_arms
    ours = [info["prefix"] for info in arms.values()]
    for prefix in ours:                     # report contacts of every body of the arm and the tool
        for prim in Usd.PrimRange(stage.GetPrimAtPath(prefix)):
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or prim.GetName().startswith(("link", "Tool", "Jaw")):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    touches = {}

    def on_contact(headers, data):
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            mine0 = any(a0.startswith(p) for p in ours)
            mine1 = any(a1.startswith(p) for p in ours)
            if mine0 == mine1:
                continue                     # both ours (filtered/jointed) or neither
            imp = sum(abs(data[i].impulse[0])+abs(data[i].impulse[1])+abs(data[i].impulse[2])
                      for i in range(h.contact_data_offset, h.contact_data_offset+h.num_contact_data))
            key = (a0, a1) if mine0 else (a1, a0)
            touches[key] = max(touches.get(key, 0.), imp*HZ)
    sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    expected = {}
    for label, info in arms.items():
        R6, p6 = S.link_poses(cell_mount.PARK)["link6"]
        Rb = S.rot((0, 0, 1), info["yaw"])
        expected[label] = info["base"]+Rb@p6
    ok = True
    try:
        steps = int(a.seconds*HZ)
        for i in range(steps):
            cell.step()
        report = {}
        for label, info in arms.items():
            _, p = world(stage, info["root"]+"/link6")
            drift = float(np.linalg.norm(p-expected[label]))*1000
            report[label] = dict(flange_drift_mm=round(drift, 3), flange=[round(float(v), 4) for v in p])
            ok &= drift < 2.
        report["touches_N"] = {f"{k[0]} | {k[1]}": round(v, 1) for k, v in touches.items()}
        ok &= not touches
        print("MOUNTCHECK", json.dumps(report), flush=True)
        print("MOUNTCHECK", "PASS" if ok else "FAIL", flush=True)
    finally:
        sub = None
        cell.close()
    return ok


if __name__ == "__main__":
    code = 1
    try:
        code = 0 if main() else 1
    finally:
        omni.kit.app.get_app().post_quit(code)
