#!/usr/bin/env bash
# Regenerate map/assets/*/*.usd from the built scene (run ./map/wheel_cell/build.sh first).
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_dir="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
shopt -s nullglob
usd=( "$sim_dir"/extscache/omni.usd.libs-*/pxr )
if [[ ${#usd[@]} -eq 0 ]]; then
    echo "Set ISAAC_SIM_ROOT to the Isaac Sim installation." >&2
    exit 1
fi
usd_dir="$(dirname "${usd[0]}")"
export PYTHONPATH="$usd_dir${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$usd_dir/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$sim_dir/python.sh" -B "$here/build_assets.py" "$@"
