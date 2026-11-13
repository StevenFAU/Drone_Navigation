#!/bin/bash
# Source ROS 2 + the workspace. Drone ROS traffic lives on its own DDS domain (27) so it never
# mixes with a sim running on the laptop (domain 0); the laptop joins with scripts/drone_env.sh.
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27}
exec "$@"
