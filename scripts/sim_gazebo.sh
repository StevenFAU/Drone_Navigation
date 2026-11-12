#!/usr/bin/env bash
# Terminal 1: start Gazebo with the ArduPilot Iris drone.
#   scripts/sim_gazebo.sh                 # with GUI
#   scripts/sim_gazebo.sh --headless      # no window (lighter on RAM)
#   WORLD=iris_runway.sdf scripts/sim_gazebo.sh   # another world (default: sim/worlds/aruco_world.sdf)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
WORLD=${WORLD:-aruco_world.sdf}
if [ "$1" = "--headless" ]; then
  # --headless-rendering: the camera still renders without a display (EGL)
  exec gz sim -v3 -s -r --headless-rendering "$WORLD"
else
  exec gz sim -v3 -r "$WORLD"
fi
