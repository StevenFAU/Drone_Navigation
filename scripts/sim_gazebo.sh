#!/usr/bin/env bash
# Terminal 1: start Gazebo with the ArduPilot Iris drone.
#   scripts/sim_gazebo.sh                 # with GUI
#   scripts/sim_gazebo.sh --headless      # no window (lighter on RAM)
#   WORLD=my_world.sdf scripts/sim_gazebo.sh
set -e
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
WORLD=${WORLD:-iris_runway.sdf}
if [ "$1" = "--headless" ]; then
  exec gz sim -v3 -s -r "$WORLD"
else
  exec gz sim -v3 -r "$WORLD"
fi
