# Source this in every terminal you use for the sim:   source setup/env.sh
_PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
[ -f "$_PROJ/ros2_ws/install/setup.bash" ] && source "$_PROJ/ros2_ws/install/setup.bash"

export GZ_VERSION=jetty
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$_PROJ/sim/models:$_PROJ/sim/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"

# Gazebo's GUI is more reliable under XWayland than native Wayland.
[ "${XDG_SESSION_TYPE:-}" = "wayland" ] && export QT_QPA_PLATFORM=xcb

export DRONE_MISSION_ROOT="$_PROJ"
unset _PROJ
