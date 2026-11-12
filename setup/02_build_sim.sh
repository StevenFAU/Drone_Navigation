#!/usr/bin/env bash
# Builds ArduCopter SITL, the ArduPilot Gazebo plugin, and this project's ROS 2 workspace.
# No sudo needed. Run after 01_install_system.sh.
set -eo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS=${JOBS:-4}   # keep this modest: the laptop has ~7 GB RAM

echo "== Building ArduCopter SITL (~/ardupilot) =="
source "$HOME/venv-ardupilot/bin/activate"
cd "$HOME/ardupilot"
./waf configure --board sitl
./waf copter -j"$JOBS"
deactivate

echo "== Building ArduPilot Gazebo plugin (~/ardupilot_gazebo) =="
source /opt/ros/lyrical/setup.bash   # Gazebo Jetty libraries come from the ROS vendor packages
export GZ_VERSION=jetty
cd "$HOME/ardupilot_gazebo"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$JOBS"

echo "== Building ROS 2 workspace ($PROJ/ros2_ws) =="
cd "$PROJ/ros2_ws"
colcon build --symlink-install

echo
echo "Build finished. See README.md for how to run the simulation."
