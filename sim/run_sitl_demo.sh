#!/usr/bin/env bash
# Full chain: 4 x SITL + 4 x adsas_node (ext-pos) + bridge.
# Node 4 is lost at FAIL_AT seconds; its vehicle lands and the rest of the
# swarm takes over its cells.
#
# Start SITL first in another terminal:  ./sim/launch_sitl_swarm.sh
set -euo pipefail
cd "$(dirname "$0")/.."
FAIL_AT=${FAIL_AT:-60}
[ -x build/adsas_node ] || make build/adsas_node
mkdir -p logs

pids=()
for id in 1 2 3 4; do
  extra=()
  if [ "$id" = 4 ]; then extra=(--fail-at "$FAIL_AT"); fi
  ./build/adsas_node --id "$id" --nodes 4 --grid 6x6 --cell 25 \
      --ext-port $((48000 + id)) --csv "logs/sitl_node$id.csv" "${extra[@]}" \
      > "logs/sitl_node$id.log" 2>&1 &
  pids+=($!)
done
trap 'kill "${pids[@]}" 2>/dev/null' EXIT

python3 sim/sitl_bridge.py --instances 4
