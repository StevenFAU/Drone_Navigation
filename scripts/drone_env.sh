# Source this on the laptop to talk to the real drone's ROS 2 stack instead of the sim:
#   source scripts/drone_env.sh
#   ros2 topic echo /mavros/state --once
#   ros2 run rqt_image_view rqt_image_view /camera/image_raw
# The drone uses DDS domain 27 so it can never mix with a sim (domain 0) on this laptop.
# Needs the laptop and the Pi on one network: the Ethernet cable (nmcli con up drone-link) or
# the drone's Wi-Fi hotspot.
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
export ROS_DOMAIN_ID=27
export DRONE=${DRONE:-group7@10.43.0.42}
echo "ROS_DOMAIN_ID=27 (real drone). Pi: $DRONE"
