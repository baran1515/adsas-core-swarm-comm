#!/usr/bin/env bash
# Create 4 copies of the ardupilot_gazebo "iris_with_ardupilot" model, each
# with its own FDM port (9002, 9012, 9022, 9032) so that every SITL instance
# (-I 0..3) drives its own airframe in one Gazebo world.
#
#   ARDUPILOT_GAZEBO=~/ardupilot_gazebo ./sim/gazebo/make_swarm_models.sh
set -euo pipefail
SRC=${ARDUPILOT_GAZEBO:-$HOME/ardupilot_gazebo}/models/iris_with_ardupilot
HERE=$(cd "$(dirname "$0")" && pwd)
DST=$HERE/models
if [ ! -d "$SRC" ]; then
  echo "model not found: $SRC"
  exit 1
fi
mkdir -p "$DST"
for i in 0 1 2 3; do
  name="iris_adsas_$i"
  port=$((9002 + 10 * i))
  rm -rf "${DST:?}/$name"
  cp -r "$SRC" "$DST/$name"
  sed -i "s#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>$port</fdm_port_in>#" "$DST/$name/model.sdf"
  sed -i "s#<name>iris_with_ardupilot</name>#<name>$name</name>#" "$DST/$name/model.config"
  echo "created $name (fdm_port_in $port)"
done
echo
echo "export GZ_SIM_RESOURCE_PATH=$DST:\$GZ_SIM_RESOURCE_PATH"
echo "gz sim -v4 -r $HERE/swarm_world.sdf"
