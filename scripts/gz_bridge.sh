#!/usr/bin/env bash
# Bridge Gazebo topics to ROS 2 (start_sim.sh runs this for you):
#   /camera/image         sensor_msgs/Image       forward camera, 1280x720, 15 Hz
#   /camera/camera_info   sensor_msgs/CameraInfo  intrinsics for solvePnP
#   /ground_truth/drone   geometry_msgs/PoseStamped  true pose of iris_fwd_cam, 50 Hz
#   /ground_truth/marker  geometry_msgs/PoseStamped  true pose of aruco_marker, 1 Hz
# Ground truth comes from PosePublisher plugins in aruco_world.sdf (world frame, ENU, origin
# where the drone spawned); the mission nodes log it next to the drone's own estimate.
# View the camera with:
#   ros2 run rqt_image_view rqt_image_view /camera/image
set -e
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
exec ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /model/iris_fwd_cam/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \
  /model/aruco_marker/pose_static@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \
  --ros-args \
  -r /model/iris_fwd_cam/pose:=/ground_truth/drone \
  -r /model/aruco_marker/pose_static:=/ground_truth/marker
