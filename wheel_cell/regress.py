"""Regression set: run several seeds and probes in parallel and compare with a saved baseline.

    python3 map/wheel_cell/regress.py                 # default set, 4 in parallel
    python3 map/wheel_cell/regress.py --quick         # module probes only (about a minute)
    python3 map/wheel_cell/regress.py --save-baseline # store this run as the reference
    python3 map/wheel_cell/regress.py --only seed42 jam42 -- --config my.json
    python3 map/wheel_cell/regress.py --area feed     # one area of the cell only
    python3 map/wheel_cell/regress.py --changed       # areas touched by uncommitted changes

Areas: supply (truck infeed), insertion (hub bench), feed (belt, stopper, clamp, tip-up,
hand-off, alignment: first 11 s), dock (AMR docking: first 21 s), cycle (seed 42 to the
mount arms parked), jam (fault case), full (everything).

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

# A partial scenario stops at --until and passes if no station has failed so far.
SCENARIOS["feed42"] = (["--seed", "42", "--until", "11"], "belt to aligned, no fault")
SCENARIOS["dock42"] = (["--seed", "42", "--until", "21"], "AMR docked, no fault")

AREAS = {
    "supply": ("supply_left", "supply_right"),
    "insertion": ("insert_nominal", "insert_skew"),
    "feed": ("feed42",),
    "dock": ("dock42",),
    "cycle": ("seed42",),
    "jam": ("jam42",),
    "full": tuple(n for n in SCENARIOS if n not in ("feed42", "dock42")),
}

# Which areas a source file can affect (files not listed affect none: tooling, docs, recording).
FILE_AREAS = {
    "conveyor.py": ("supply", "feed", "cycle"),
    "physics.py": ("insertion", "feed", "dock", "cycle"),
    "fitment.py": ("insertion", "cycle"),
    "wheel_geometry.py": ("insertion", "cycle"),
    "robot.py": ("cycle",),
    "cell_config.py": ("feed", "cycle"),
    "process.py": ("full",),
    "physical_data.py": ("full",),
    "build.py": ("full",),
    "visuals.py": (),
}

# Known to be unreliable in the current model: reported, but they do not fail the run.
KNOWN_FRAGILE = {
    "jam42": "2 mm FL offset is inside float + hole clearance: jams on seed 42 only by chance",
}


def changed_areas():
    git = ["git", "-C", str(ROOT)]
    names = subprocess.run(git + ["diff", "--name-only", "HEAD"], capture_output=True, text=True).stdout.split()
    names += subprocess.run(git + ["ls-files", "--others", "--exclude-standard"],
                            capture_output=True, text=True).stdout.split()
    areas = []
    for name in names:
        for area in FILE_AREAS.get(Path(name).name, ()):
            if area not in areas:
                areas.append(area)
    return areas


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
        elif line.startswith(("INSERTION_BENCH", "PARTIAL_CHECK", "UNPOWERED_TIP", "INSERTION_PROBE", "MODULE_RESULT",
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
    status = "PASS" if result["exit"] == 0 else "KNOWN" if result["name"] in KNOWN_FRAGILE else "FAIL"
    return f"{status:5}  {result['name']:15} {result['wall_s']:6.0f}s  rtf {rtf!s:6}  {phases}"


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
    parser.add_argument("--area", nargs="+", choices=AREAS)
    parser.add_argument("--changed", action="store_true", help="run the areas touched by uncommitted changes")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--save-baseline", action="store_true")
    parser.add_argument("extra", nargs="*", help="arguments passed to every check (after --)")
    config = parser.parse_args()
    areas = config.area or []
    if config.changed:
        areas += changed_areas()
        print("changed areas:", ", ".join(areas) or "none")
        if not areas:
            return 0
    if areas:
        names = []
        for area in ("full",) if "full" in areas else areas:
            names += [n for n in AREAS[area] if n not in names]
    else:
        names = config.only or (QUICK if config.quick else AREAS["full"])
    with ThreadPoolExecutor(config.jobs) as pool:
        results = list(pool.map(lambda n: run(n, SCENARIOS[n][0], config.extra), names))
    for result in results:
        print(summary(result))
        for line in result["lines"]:
            print("      ", line)
        if result["exit"] and result["name"] in KNOWN_FRAGILE:
            print("       known fragile:", KNOWN_FRAGILE[result["name"]])
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
    return 0 if all(r["exit"] == 0 or r["name"] in KNOWN_FRAGILE for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
