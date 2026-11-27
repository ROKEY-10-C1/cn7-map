"""Accumulation end of the feed belts (scene/, cell_layout.py): the tires convey in flat, the head tire seats in the V end
stop and the next one queues behind it; the mount arms stay parked and touch nothing.

Run: CELL_CHECK_SCRIPT=cell_feed_check.py ./cell_check.sh [--seconds 30]
Prints FEEDCHECK lines (per station: phase, time to pick_ready, head-tire offset from the pick point, tilt,
stop and tire-to-tire contact forces) and exits 0 when all pass.
"""
import argparse
import json
import sys
from math import acos, degrees
from pathlib import Path

import numpy as np
import omni.kit.app
from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, Usd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/"wheel_cell"))
sys.dont_write_bytecode = True
from physics import Cell, HZ  # noqa: E402

sys.path.insert(0, str(HERE))
import cell_layout as L  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", type=Path, default=L.SCENE_DIR/"wheel_cell.usda")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--seconds", type=float, default=30.)
    a = ap.parse_args()
    stage = Usd.Stage.Open(str(a.stage))
    stage.SetEditTarget(stage.GetSessionLayer())
    cell = Cell(stage, a.seed, True, start="gate")
    assert L.active(stage), "no belt end stop in this scene (run tools/extend_belt_end.py)"
    arms = [info["prefix"] for info in cell.mount_arms.values()]
    for prefix in arms:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(prefix)):
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or prim.GetName().startswith(("link", "Tool", "Jaw")):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    peak = {}
    cur = {}
    steady = {}
    arm_touch = {}

    def on_contact(headers, data):
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            f = sum(np.linalg.norm(data[i].impulse) for i in range(h.contact_data_offset,
                                                                    h.contact_data_offset+h.num_contact_data))*HZ
            mine = [any(x.startswith(p) for p in arms) for x in (a0, a1)]
            if mine[0] != mine[1]:
                key = f"{a0} | {a1}"
                arm_touch[key] = max(arm_touch.get(key, 0.), f)
                continue
            for x, y in ((c0, a1), (c1, a0)):
                if "AccumEnd" in x:
                    k = ("stop", y.rsplit("/", 2)[-2])
                    peak[k] = max(peak.get(k, 0.), f)
                    cur[k] = cur.get(k, 0.)+f
            if "/Tire" in a0 and "/Tire" in a1 and a0.split("/")[3] != a1.split("/")[3]:
                k = ("tire_tire", "-".join(sorted((a0.split("/")[3], a1.split("/")[3]))))
                peak[k] = max(peak.get(k, 0.), f)
                cur[k] = cur.get(k, 0.)+f
    sub = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    ready_t = {}
    ok = True
    try:
        for i in range(int(a.seconds*HZ)):
            cur.clear()
            cell.step()
            if i >= int((a.seconds-1.)*HZ):            # last second: steady line pressure
                for k, v in cur.items():
                    steady[k] = steady.get(k, 0.)+v/HZ
            for s in cell.stations:
                if s.phase == "pick_ready" and s.name not in ready_t:
                    ready_t[s.name] = round((i+1)/HZ, 2)
        report = {}
        for s in cell.stations:
            pose = cell.api.get_rigidbody_transformation(s.wheel)
            p = np.array(pose["position"])
            qx, qy, qz, qw = pose["rotation"]
            z_axis = np.array([2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), 1-2*(qx*qx+qy*qy)])
            tilt = degrees(acos(min(1., abs(z_axis[2]))))
            report[s.name] = dict(phase=s.phase, ready_s=ready_t.get(s.name), x=round(float(p[0]), 4),
                                  along_mm=round(float(p[0]-L.PICK_X)*1000, 2),
                                  across_mm=round(float(p[1]-s.side*L.LINE_Y)*1000, 2), tilt_deg=round(tilt, 2))
        heads = [r for r in report.values() if r["phase"] == "pick_ready"]
        queued = [r for r in report.values() if r["phase"] == "queued"]
        ok &= len(heads) == 2 and len(queued) == 2
        ok &= all(abs(r["along_mm"]) < 3. and abs(r["across_mm"]) < 2. and r["tilt_deg"] < 1. for r in heads)
        # the queued tire rests against the head tire: one diameter behind it
        ok &= all(abs(r["along_mm"]-2000*L.TIRE_R) < 15. for r in queued)
        ok &= not arm_touch
        report["peak_N"] = {f"{k[0]}:{k[1]}": round(v, 1) for k, v in sorted(peak.items())}
        report["steady_last_1s_N"] = {f"{k[0]}:{k[1]}": round(v, 1) for k, v in sorted(steady.items())}
        report["arm_touch_N"] = {k: round(v, 1) for k, v in arm_touch.items()}
        print("FEEDCHECK", json.dumps(report), flush=True)
        print("FEEDCHECK", "PASS" if ok else "FAIL", flush=True)
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
