#!/usr/bin/env bash
# Bridge the drone's forward camera from Gazebo to ROS 2:
#   /camera/image        sensor_msgs/Image       (1280x720, 15 Hz)
#   /camera/camera_info  sensor_msgs/CameraInfo  (intrinsics for solvePnP)
# start_sim.sh runs this for you. View the image with:  ros2 run rqt_image_view rqt_image_view /camera/image
set -e
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
exec ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
