#!/usr/bin/env bash
# CPU PhysX smoke test of map/assets (uses the wheel cell's headless kit app).
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_dir="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
sandbox_dir="$(mktemp -d)"
trap 'rm -rf -- "$sandbox_dir"' EXIT
"$sim_dir/kit/kit" "$here/../wheel_cell/check_physics.kit" --portable --portable-root "$sandbox_dir" --no-window \
    --ext-folder "$sim_dir/exts" --ext-folder "$sim_dir/extscache" \
    --ext-folder "$sim_dir/kit/exts" --ext-folder "$sim_dir/kit/extscore" \
    --exec "\"$here/check_assets.py\""
