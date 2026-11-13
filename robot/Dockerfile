# ROS 2 Jazzy stack for the T27's Raspberry Pi 5 (Pi OS is Debian, which has no ROS 2
# packages, so everything runs in this Ubuntu 24.04 container).
#   - MAVROS 2 to the flight controller
#   - camera_ros on Raspberry Pi's libcamera fork (upstream libcamera can't drive the Pi 5's
#     camera pipeline), so the Pi Camera 3 publishes sensor_msgs/Image
#   - this repo's drone_mission package
# Built on the Pi by robot/install.sh (context = repo root). First build takes ~20-30 min.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-mavros ros-jazzy-mavros-extras \
        ros-jazzy-camera-info-manager ros-jazzy-cv-bridge ros-jazzy-image-transport \
        python3-opencv python3-numpy python3-colcon-common-extensions \
        git meson ninja-build pkg-config cmake python3-jinja2 python3-ply python3-yaml \
        libyaml-dev libgnutls28-dev openssl libssl-dev libudev-dev libdw-dev libunwind-dev \
        libboost-dev nlohmann-json3-dev libevent-dev wget \
    && rm -rf /var/lib/apt/lists/*
# MAVROS needs the GeographicLib geoid/magnetic datasets or it refuses to start
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Raspberry Pi's libcamera fork (Pi 5 = "pisp" pipeline; libpisp comes in as a meson subproject)
RUN git clone --depth 1 https://github.com/raspberrypi/libcamera.git /opt/src/libcamera \
    && cd /opt/src/libcamera \
    && meson setup build --buildtype=release --prefix=/usr \
         -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
         -Dv4l2=false -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
         -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled \
    && ninja -C build install && ldconfig && rm -rf /opt/src/libcamera/build

# camera_ros (ROS 2 node for libcamera) + drone_mission
WORKDIR /ws
RUN git clone --depth 1 https://github.com/christianrauch/camera_ros.git src/camera_ros
COPY ros2_ws/src/drone_mission src/drone_mission
RUN . /opt/ros/jazzy/setup.sh \
    && apt-get update \
    && (rosdep update --rosdistro jazzy || true) \
    && rosdep install --from-paths src --ignore-src -y --skip-keys=libcamera \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf log

# ArduPilot over serial: the T27 team found MAVROS' timesync/system-time messages upset the
# stream rates, so turn them off (same fix as their ROS 1 apm_config.yaml).
RUN f=/opt/ros/jazzy/share/mavros/launch/apm_config.yaml \
    && sed -i -E 's/^(\s*timesync_rate:).*/\1 0.0/; s/^(\s*system_time_rate:).*/\1 0.0/' $f \
    && (grep -nE "timesync_rate|system_time_rate" $f || echo "no timesync params in $f")

COPY robot/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
