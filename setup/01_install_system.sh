#!/usr/bin/env bash
# One-time system install (asks for your sudo password). For Ubuntu 26.04.
# Downloads ArduPilot + its Gazebo plugin into your home folder, then installs
# ROS 2 Lyrical, MAVROS, ros_gz (brings Gazebo Jetty), and ArduPilot SITL prerequisites.
# Safe to re-run: steps that are already done are skipped.
set -euo pipefail
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Pinned versions (the ones verified on the first machine).
ARDUPILOT_TAG=Copter-4.6.3                                   # same as the real drone
ARDUPILOT_GAZEBO_COMMIT=f116b03552d05c8d0b0bed5ed67033a1183121d1

. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "resolute" ]; then
  echo "WARNING: these scripts were written for Ubuntu 26.04 (resolute); this is ${PRETTY_NAME}." >&2
  echo "ROS 2 Lyrical may not exist for this release. Press Ctrl-C to stop, or wait 10 s to continue." >&2
  sleep 10
fi

echo "== Downloading ArduPilot ($ARDUPILOT_TAG) and ardupilot_gazebo =="
if [ ! -d "$HOME/ardupilot" ]; then
  git clone --branch "$ARDUPILOT_TAG" --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
fi
if [ ! -d "$HOME/ardupilot_gazebo" ]; then
  git clone https://github.com/ArduPilot/ardupilot_gazebo "$HOME/ardupilot_gazebo"
  git -C "$HOME/ardupilot_gazebo" checkout --quiet "$ARDUPILOT_GAZEBO_COMMIT"
fi
# ArduPilot 4.6.3's own install script predates Ubuntu 26.04, so use a newer copy of it
# (from ArduPilot master, kept in setup/vendor). It must live inside the ArduPilot tree.
PREREQS="$HOME/ardupilot/Tools/environment_install/install-prereqs-ubuntu-latest.sh"
cp "$SETUP_DIR/vendor/ardupilot-install-prereqs-ubuntu.sh" "$PREREQS"
chmod +x "$PREREQS"

echo "== Adding ROS 2 apt source =="
sudo apt update
sudo apt install -y curl git software-properties-common
sudo add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt update

echo "== Installing ROS 2 Lyrical, MAVROS, Gazebo (via ros_gz) =="
sudo apt install -y \
  ros-lyrical-desktop \
  ros-lyrical-ros-gz \
  ros-lyrical-mavros \
  ros-lyrical-mavros-extras \
  ros-lyrical-cv-bridge \
  ros-dev-tools \
  python3-opencv \
  geographiclib-tools

echo "== Installing ArduPilot Gazebo plugin build deps =="
sudo apt install -y \
  cmake build-essential rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-gl

echo "== Installing MAVProxy console GUI library from Ubuntu =="
# ArduPilot's script would otherwise compile wxPython from source (30+ min, and it
# stalled on the first machine). Ubuntu's package works because the venv sees system packages.
sudo apt install -y python3-wxgtk4.0 fonts-freefont-ttf xfonts-base

echo "== Installing GeographicLib datasets (MAVROS needs these) =="
GEO_SCRIPT=/opt/ros/lyrical/lib/mavros/install_geographiclib_datasets.sh
if [ -x "$GEO_SCRIPT" ]; then
  sudo "$GEO_SCRIPT"
else
  sudo geographiclib-get-geoids egm96-5
  sudo geographiclib-get-gravity egm96
  sudo geographiclib-get-magnetic emm2015
fi

echo "== Initialising rosdep =="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update || true

echo "== Installing ArduPilot SITL prerequisites =="
# DO_AP_STM_ENV=0         skip the STM32 firmware toolchain (only for building real-board firmware)
# DO_PYTHON_VENV_ENV=0    don't auto-activate ArduPilot's venv in every shell (confuses ROS builds)
# SKIP_AP_GRAPHIC_ENV=1   skip the wxPython source build (installed from apt above)
# SKIP_AP_GIT_CHECK=1     submodules were already fetched by the shallow clone above
# Note: the script also removes modemmanager and brltty, and adds ArduPilot tools to PATH in ~/.profile.
DO_AP_STM_ENV=0 DO_PYTHON_VENV_ENV=0 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_GIT_CHECK=1 \
  "$PREREQS" -y

echo
echo "System install finished. Next: setup/02_build_sim.sh (no sudo)."
