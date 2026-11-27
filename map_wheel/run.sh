#!/usr/bin/env bash
# map_wheel 장면 빌드(없으면) 후 Isaac Sim으로 열어 재생.  ./map_wheel/run.sh [--camera Hub] [--rebuild]
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim="${ISAAC_SIM_ROOT:-$HOME/isaacsim}/python.sh"
[[ -x "$sim" ]] || { echo "Isaac Sim 없음: $sim (ISAAC_SIM_ROOT 설정)" >&2; exit 2; }
args=()
rebuild=0
for a in "$@"; do [[ "$a" == --rebuild ]] && rebuild=1 || args+=("$a"); done
if [[ $rebuild == 1 || ! -f "$here/scene/mount_bench.usda" ]]; then
  "$sim" -B "$here/build.py"
fi
exec "$sim" -B "$here/play.py" "${args[@]}"
