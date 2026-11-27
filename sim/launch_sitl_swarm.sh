#!/usr/bin/env bash
# Start N ArduCopter SITL instances side by side (default 4).
#
#   ARDUPILOT_HOME=~/ardupilot ./sim/launch_sitl_swarm.sh          # SITL only
#   GAZEBO=1 ARDUPILOT_HOME=~/ardupilot ./sim/launch_sitl_swarm.sh # Gazebo FDM
#
# Instance i -> sysid i+1, MAVLink tcp:127.0.0.1:5760+10*i, JSON FDM port 9002+10*i
set -euo pipefail

N=${N:-4}
ARDUPILOT_HOME=${ARDUPILOT_HOME:-$HOME/ardupilot}
WORK=${WORK:-/tmp/adsas_sitl}
LAT=-35.363261
LON=149.165230
ALT=584
HDG=0
SIM_VEHICLE="$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py"

if [ ! -f "$SIM_VEHICLE" ]; then
  echo "sim_vehicle.py not found, set ARDUPILOT_HOME"
  exit 1
fi
mkdir -p "$WORK"

for ((i = 0; i < N; i++)); do
  # spawn vehicles 3 m apart on an east-west line (1 deg lon ~ 90.7 km at this lat)
  lon_i=$(python3 -c "print('%.7f' % ($LON + $i * 3.0 / 90730.0))")
  extra=()
  if [ "${GAZEBO:-0}" = 1 ]; then extra=(-f gazebo-iris --model JSON); fi
  echo "starting SITL instance $i (sysid $((i + 1)))"
  "$SIM_VEHICLE" -v ArduCopter -I "$i" --sysid "$((i + 1))" --no-mavproxy --no-rebuild \
      --use-dir="$WORK/inst$i" --custom-location="$LAT,$lon_i,$ALT,$HDG" "${extra[@]}" \
      > "$WORK/inst$i.log" 2>&1 &
done

echo "SITL logs: $WORK/inst*.log  (Ctrl+C stops all)"
trap 'kill $(jobs -p) 2>/dev/null' INT TERM
wait
