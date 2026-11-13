#!/bin/bash
# Run ON THE PI (from ~/Drone_Navigation/robot): switch from the T27 team's ROS 1 containers to
# this project's ROS 2 stack. Nothing of theirs is deleted.
#   ./install.sh            build the image (first time ~30+ min) and start MAVROS + camera
#   ./install.sh --revert   stop ours, start the T27 ROS 1 containers again
set -e
cd "$(dirname "$0")"
T27=(roscore mavros camera_mono vinsfusion)
if [ "$1" = "--revert" ]; then
    docker compose down
    docker start "${T27[@]}"
    echo "T27 ROS 1 stack running again (it also starts at boot)."
    exit 0
fi
docker compose build mavros
# Only one program can own the flight-controller serial port and the camera. A stopped
# 'unless-stopped' container stays stopped across reboots, so ours are the ones that come up.
docker stop "${T27[@]}" 2>/dev/null || true
docker compose up -d mavros camera
echo "ROS 2 stack up (domain 27). Check:  docker compose logs -f"
