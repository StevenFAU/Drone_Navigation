#!/bin/bash
# Copy this repo to the drone's Pi (~/Drone_Navigation). The containers mount the
# drone_mission package and robot/config, so node and config changes need no rebuild:
# just `docker compose run --rm mission` again. Changes to robot/Dockerfile need ./install.sh.
# The camera calibration (robot/config/camera.yaml) is made on the Pi and never overwritten;
# fetch it with: scp $DRONE:Drone_Navigation/robot/config/camera.yaml robot/config/
#   scripts/drone_sync.sh                 # DRONE=group7@10.43.0.42 by default
set -e
DRONE=${DRONE:-group7@10.43.0.42}
cd "$(dirname "$0")/.."
rsync -az --exclude .git --exclude 'ros2_ws/build' --exclude 'ros2_ws/install' --exclude 'ros2_ws/log' \
      --exclude results --exclude logs --exclude __pycache__ \
      --exclude robot/config/camera.yaml ./ "$DRONE:Drone_Navigation/"
echo "synced to $DRONE:Drone_Navigation"
