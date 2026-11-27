#!/usr/bin/env bash
# Download the Kawasaki RS080N robot USD of Isaac Sim 5.1 (NVIDIA asset store) next to this script.
# Not committed: the NVIDIA asset terms apply; the file is fetched on each machine.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dst="$here/assets/rs080n"
base="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Kawasaki/RS080N"
mkdir -p "$dst/configuration"
curl -fsSL -o "$dst/rs080n_onrobot_rg2.usd" "$base/rs080n_onrobot_rg2.usd"
curl -fsSL -o "$dst/configuration/rs080n_onrobot_rg2_robot_schema.usd" "$base/configuration/rs080n_onrobot_rg2_robot_schema.usd"
echo "RS080N USD in $dst"
