"""Regression set: run several seeds and probes in parallel and compare with a saved baseline.

    python3 map/wheel_cell/regress.py                 # default set, 4 in parallel
    python3 map/wheel_cell/regress.py --quick         # module probes only (about a minute)
    python3 map/wheel_cell/regress.py --save-baseline # store this run as the reference
    python3 map/wheel_cell/regress.py --only seed42 jam42 -- --config my.json

Each scenario is one check_physics.sh run. Results land in output/regress/<time>.json and a
table is printed. Any station whose result differs from the baseline is flagged.
"""
import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CHECK = ROOT/"check_physics.sh"
BASELINE = ROOT/"regression_baseline.json"
OUT = ROOT/"output"/"regress"

# name: (arguments, what passing means)
SCENARIOS = {
    "seed42": (["--seed", "42", "--require-seated"], "full cycle, all four seated"),
    "seed7": (["--seed", "7", "--require-seated"], "full cycle, all four seated"),
    "seed123": (["--seed", "123", "--require-seated"], "full cycle, all four seated"),
    "seed2024": (["--seed", "2024", "--require-seated"], "full cycle, all four seated"),
    "jam42": (["--seed", "42", "--jam"], "FL must jam without face contact"),
    "insert_nominal": (["--insertion-probe", "nominal"], "bench insertion seats"),
    "insert_skew": (["--insertion-probe", "skew"], "bench insertion with skew stalls on studs"),
    "supply_left": (["--supply-probe", "left"], "infeed reaches the left branch"),
    "supply_right": (["--supply-probe", "right"], "infeed reaches the right branch"),
}
QUICK = ("insert_nominal", "insert_skew", "supply_left")


def run(name, args, extra):
    started = time.time()
    proc = subprocess.run([str(CHECK), *args, *extra], capture_output=True, text=True)
    result = {"name": name, "args": args + extra, "exit": proc.returncode,
              "wall_s": round(time.time() - started, 1), "stations": None, "profile": None, "lines": []}
    for line in proc.stdout.splitlines():
        if line.startswith("PHYSICS_RESULT "):
            result["stations"] = json.loads(line.split(" ", 1)[1])
        elif line.startswith("PROFILE "):
            result["profile"] = json.loads(line.split(" ", 1)[1])
        elif line.startswith(("INSERTION_BENCH", "UNPOWERED_TIP", "INSERTION_PROBE", "MODULE_RESULT",
                              "Traceback", "RAIL_CHECK", "CARRIER_CHECK")):
            result["lines"].append(line[:400])
    if proc.returncode and not result["lines"]:
        result["lines"] = [l[:300] for l in (proc.stdout + proc.stderr).splitlines()
                           if "Error" in l or "Traceback" in l][-5:]
    return result


def summary(result):
    stations = result["stations"] or {}
    phases = " ".join(f"{k}:{v['result']}" for k, v in stations.items())
    profile = result["profile"] or {}
    rtf = profile.get("realtime_factor", "")
    return f"{'PASS' if result['exit'] == 0 else 'FAIL':4}  {result['name']:15} {result['wall_s']:6.0f}s  rtf {rtf!s:6}  {phases}"


def compare(results, baseline):
    changes = []
    for result in results:
        old = baseline.get(result["name"])
        if not old:
            continue
        if old["exit"] != result["exit"]:
            changes.append(f"{result['name']}: exit {old['exit']} -> {result['exit']}")
        for station, data in (result["stations"] or {}).items():
            before = (old.get("stations") or {}).get(station, {}).get("result")
            if before and before != data["result"]:
                changes.append(f"{result['name']} {station}: {before} -> {data['result']}")
    return changes


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--only", nargs="+", choices=SCENARIOS)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--save-baseline", action="store_true")
    parser.add_argument("extra", nargs="*", help="arguments passed to every check (after --)")
    config = parser.parse_args()
    names = config.only or (QUICK if config.quick else tuple(SCENARIOS))
    with ThreadPoolExecutor(config.jobs) as pool:
        results = list(pool.map(lambda n: run(n, SCENARIOS[n][0], config.extra), names))
    for result in results:
        print(summary(result))
        for line in result["lines"]:
            print("      ", line)
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT/f"{datetime.now():%Y%m%d-%H%M%S}.json"
    path.write_text(json.dumps({r["name"]: r for r in results}, indent=1))
    print("saved", path)
    if BASELINE.exists():
        changes = compare(results, json.loads(BASELINE.read_text()))
        print("baseline:", "no change" if not changes else "")
        for change in changes:
            print("   CHANGED", change)
    if config.save_baseline:
        existing = json.loads(BASELINE.read_text()) if BASELINE.exists() else {}
        existing.update({r["name"]: r for r in results})
        BASELINE.write_text(json.dumps(existing, indent=1))
        print("baseline saved", BASELINE)
    return 0 if all(r["exit"] == 0 for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
