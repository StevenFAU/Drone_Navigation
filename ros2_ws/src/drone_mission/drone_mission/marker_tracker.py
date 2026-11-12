"""ArUco detection on the forward camera, turned into a marker pose in MAVROS's local ENU frame.

Chain for each frame:  pixels --solvePnP--> marker in camera optical frame (x right, y down,
z forward) --fixed mount--> body FLU (x forward, y left, z up) --drone attitude--> local ENU.

Because the marker doesn't move, its ENU position and facing direction are low-pass filtered
across frames. That smooths detection noise, removes the effect of the drone tilting while it
moves, and lets the controller keep going through a few dropped frames.
"""
import math
import time

import cv2
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

# optical (x right, y down, z fwd) -> body FLU (x fwd, y left, z up)
R_BODY_CAM = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


class MarkerTracker:
    def __init__(self, node, marker_id=0, marker_size=0.20, dictionary='DICT_4X4_50',
                 cam_offset=(0.15, 0.0, -0.02), smoothing=0.3):
        """node: a FlightNode (for .pose and .rotation()). cam_offset: camera position in body
        FLU, metres (sim: 15 cm ahead of the IMU, 2 cm below). smoothing: weight of each new
        detection in the filtered estimate (0..1)."""
        self.node = node
        self.marker_id = marker_id
        self.cam_offset = np.array(cam_offset, float)
        self.alpha = smoothing
        s = marker_size / 2.0
        # Corner order returned by the detector: top-left, top-right, bottom-right, bottom-left.
        self.obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)),
            cv2.aruco.DetectorParameters())

        self.K = None
        self.D = None
        self.position = None      # filtered marker centre, ENU (np.array 3)
        self.normal = None        # filtered horizontal unit vector the marker faces, ENU (2)
        self.last_seen = 0.0      # time.time() of the last detection
        self.raw_range = None     # camera-to-marker distance from the last detection (m)
        self.detections = 0

        group = MutuallyExclusiveCallbackGroup()   # image work doesn't block MAVROS callbacks
        node.create_subscription(CameraInfo, '/camera/camera_info', self._on_info,
                                 qos_profile_sensor_data, callback_group=group)
        node.create_subscription(Image, '/camera/image', self._on_image,
                                 qos_profile_sensor_data, callback_group=group)

    # --- state ---
    def seen_within(self, seconds):
        return self.position is not None and time.time() - self.last_seen < seconds

    def reset(self):
        self.position = self.normal = None
        self.last_seen = 0.0

    def camera_position(self):
        """Camera centre in ENU from the drone's current pose."""
        p = self.node.pose.position
        return np.array([p.x, p.y, p.z]) + np.array(self.node.rotation()) @ self.cam_offset

    # --- callbacks ---
    def _on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, float).reshape(3, 3)
            self.D = np.array(msg.d, float) if len(msg.d) else np.zeros(5)

    def _on_image(self, msg):
        if self.K is None or self.node.pose is None:
            return
        gray = to_gray(msg)
        if gray is None:
            return
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return
        ids = ids.ravel()
        idx = [i for i, m in enumerate(ids) if self.marker_id < 0 or m == self.marker_id]
        if not idx:
            return
        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, corners[idx[0]][0], self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return

        R_wb = np.array(self.node.rotation())
        t_cam = tvec.ravel()
        n_cam = cv2.Rodrigues(rvec)[0][:, 2]          # marker +z: out of its face, toward us
        pos = self.camera_position() + R_wb @ (R_BODY_CAM @ t_cam)
        n = R_wb @ (R_BODY_CAM @ n_cam)
        n_h = np.array([n[0], n[1]])
        if np.linalg.norm(n_h) < 1e-3:
            return
        n_h /= np.linalg.norm(n_h)

        if self.position is None or np.linalg.norm(pos - self.position) > 1.0:
            self.position, self.normal = pos, n_h     # first sighting, or a jump: restart filter
        else:
            a = self.alpha
            self.position = (1 - a) * self.position + a * pos
            n_f = (1 - a) * self.normal + a * n_h
            self.normal = n_f / np.linalg.norm(n_f)
        self.raw_range = float(np.linalg.norm(t_cam))
        self.last_seen = time.time()
        self.detections += 1

    # --- geometry the controller needs ---
    def errors(self, target_range):
        """Where the camera is relative to the spot `target_range` m in front of the marker.

        Returns dict (metres / radians), all from the filtered marker estimate:
          range    distance from the marker plane, measured along its normal
          lateral  sideways offset from the marker's normal line (+ = marker's left)
          height   camera height minus marker centre height
          bearing  angle from the drone's heading to the marker (+ = marker is to the left)
        """
        cam = self.camera_position()
        d = cam - self.position
        n = self.normal
        rng = d[0] * n[0] + d[1] * n[1]
        lat = d[0] * -n[1] + d[1] * n[0]
        bearing = math.remainder(math.atan2(-d[1], -d[0]) - self.node.yaw, 2 * math.pi)
        return {'range': rng, 'range_err': rng - target_range, 'lateral': lat,
                'height': d[2], 'bearing': bearing}


def to_gray(msg):
    """sensor_msgs/Image -> grayscale numpy array (no cv_bridge needed)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    return None
