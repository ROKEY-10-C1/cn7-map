#!/usr/bin/env bash
# Replay each earlier failure (one cause put back) and the current design, with video, for the records.
set -uo pipefail
cd "$(dirname "$0")/.."
run() { name=$1; shift; echo "== $name $*"; ~/isaacsim/python.sh run.py --out output/scenarios/$name "$@" > output/logs/scenario_$name.log 2>&1
        grep -E "EVENT|RESULT" output/logs/scenario_$name.log | tail -4; }
mkdir -p output/scenarios output/logs
run shoe45          --scenario shoe45 --stop-after lifted
run nosync          --scenario nosync --stop-after aligned_0
run shoe20          --scenario shoe20 --stop-after aligned_0
run joint_transfer  --scenario joint_transfer --stop-after at_hub
run novision        --scenario novision --stop-after inserted
run position_insert --scenario position_insert
run nominal         --scenario nominal
