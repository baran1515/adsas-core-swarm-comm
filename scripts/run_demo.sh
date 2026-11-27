#!/usr/bin/env bash
# 4-node swarm demo without any simulator:
#   - nodes 1..4 start on localhost and form a TCP full mesh
#   - node 4 is "lost" at t=FAIL_AT seconds (process exits)
#   - coordinator (node 1) announces NODE_DOWN, all nodes re-allocate
#   - optionally node 2 is jammed for a while (JAM_AT / JAM_FOR)
#
# usage: ./scripts/run_demo.sh            (node loss scenario)
#        SCENARIO=jam ./scripts/run_demo.sh  (jamming scenario)
set -euo pipefail
cd "$(dirname "$0")/.."

BIN=./build/adsas_node
DURATION=${DURATION:-45}
FAIL_AT=${FAIL_AT:-12}
SCENARIO=${SCENARIO:-loss}
mkdir -p logs
[ -x "$BIN" ] || make build/adsas_node

pids=()
for id in 1 2 3 4; do
  extra=()
  if [ "$SCENARIO" = "loss" ] && [ "$id" = 4 ]; then extra=(--fail-at "$FAIL_AT"); fi
  if [ "$SCENARIO" = "jam" ]  && [ "$id" = 2 ]; then extra=(--jam-at 10 --jam-for 15); fi
  "$BIN" --id "$id" --nodes 4 --csv "logs/node$id.csv" "${extra[@]}" > "logs/node$id.log" 2>&1 &
  pids+=($!)
done

echo "swarm running for ${DURATION}s (scenario: $SCENARIO) - logs in ./logs"
sleep "$DURATION"
kill "${pids[@]}" 2>/dev/null || true
wait 2>/dev/null || true

echo "---------------- event summary ----------------"
grep -hE "DOWN|SUSPECT|CONSISTENT|MISMATCH|ISOLATED|RTL|complete|JAMMED|LOSS" logs/node*.log | sort -t']' -k1,1 || true
