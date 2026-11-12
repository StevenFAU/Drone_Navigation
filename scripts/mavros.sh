#!/usr/bin/env bash
# Terminal 3: start MAVROS (ROS 2) connected to SITL.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
exec ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762
