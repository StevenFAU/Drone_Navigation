#!/bin/bash
# Source ROS 2 + the workspace. Drone ROS traffic lives on its own DDS domain (27) so it never
# mixes with a sim running on the laptop (domain 0); the laptop joins with scripts/drone_env.sh.
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27}
if [ -z "$LOGS_OWNER" ]; then
    exec "$@"
fi
# Containers run as root (DDS shared memory between them needs one user), so hand anything
# written to /logs back to the Pi user when the command ends, Ctrl-C included.
trap 'chown -R "$LOGS_OWNER" /logs 2>/dev/null' EXIT
"$@" &
child=$!
trap 'kill -INT $child 2>/dev/null' INT TERM
wait $child; rc=$?
wait $child 2>/dev/null
exit $rc
