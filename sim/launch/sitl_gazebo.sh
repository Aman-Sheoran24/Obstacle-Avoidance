#!/usr/bin/env bash
# Three-terminal SITL launcher for the obstacle-avoidance stack.
#
# Prereqs (one-time, see README.md):
#   * Ubuntu 22.04 + Gazebo Harmonic + libgz-sim8-dev
#   * ArduPilot source tree built once via Tools/environment_install/install-prereqs-ubuntu.sh
#   * ardupilot_gazebo plugin built, GZ_SIM_SYSTEM_PLUGIN_PATH + GZ_SIM_RESOURCE_PATH set
#   * obstacle-avoidance installed:  pip install -e .[dev]
#
# Usage:   bash sim/launch/sitl_gazebo.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORLD="$REPO_ROOT/sim/worlds/iris_avoidance.sdf"
PARAM_FILE="$REPO_ROOT/sim/params/copter_avoid.parm"

# Make our `models/` discoverable to Gazebo.
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/sim/models:${GZ_SIM_RESOURCE_PATH:-}"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found. Run the three commands below in three terminals manually:"
  echo
  echo "  1) gz sim -v4 -r $WORLD"
  echo "  2) sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \\"
  echo "         --add-param-file=$PARAM_FILE"
  echo "  3) python -m obstacle_avoidance.run --sim --connection udpin:127.0.0.1:14551"
  exit 0
fi

gnome-terminal --tab --title="gz sim"   -- bash -c "gz sim -v4 -r $WORLD; exec bash"
sleep 2
gnome-terminal --tab --title="ardupilot" -- bash -c \
  "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \
     --add-param-file=$PARAM_FILE; exec bash"
sleep 5
gnome-terminal --tab --title="avoidance" -- bash -c \
  "cd $REPO_ROOT && python -m obstacle_avoidance.run --sim --connection udpin:127.0.0.1:14551; exec bash"
