#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cell_dir="$here/../wheel_cell"
sim_dir="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
sandbox_dir="$(mktemp -d)"
trap 'rm -rf -- "$sandbox_dir"' EXIT
printf -v script_args '"%s"' "$here/${CELL_CHECK_SCRIPT:-cell_check.py}"
for arg; do
    # The Kit --exec parser uses shell-like argument splitting.
    printf -v quoted ' %q' "$arg"
    script_args+="$quoted"
done
"$sim_dir/kit/kit" "$cell_dir/check_physics.kit" --portable --portable-root "$sandbox_dir" --no-window \
    --ext-folder "$sim_dir/exts" --ext-folder "$sim_dir/extscache" \
    --ext-folder "$sim_dir/kit/exts" --ext-folder "$sim_dir/kit/extscore" \
    --exec "$script_args"
