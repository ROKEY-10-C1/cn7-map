#!/usr/bin/env bash
# Run a script with Isaac Sim's USD libraries but without starting Kit (fast, for authoring/inspection).
set -euo pipefail
sim_dir="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
shopt -s nullglob
usd=( "$sim_dir"/extscache/omni.usd.libs-*/pxr )
usd_dir="$(dirname "${usd[0]}")"
export PYTHONPATH="$usd_dir${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$usd_dir/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$sim_dir/python.sh" -B "$@"
