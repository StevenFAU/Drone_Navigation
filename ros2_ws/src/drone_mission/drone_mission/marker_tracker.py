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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

# optical (x right, y down, z fwd) -> body FLU (x fwd, y left, z up)
R_BODY_CAM = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


class MarkerTracker:
    def __init__(self, node, marker_id=0, marker_size=0.20, dictionary='DICT_4X4_50',
                 cam_offset=(0.15, 0.0, -0.02), smoothing=0.3,
                 image_topic='/camera/image', info_topic='/camera/camera_info', reliable=True):
        """node: a FlightNode (for .pose and .rotation()). cam_offset: camera position in body
        FLU, metres (sim: 15 cm ahead of the IMU, 2 cm below). smoothing: weight of each new
        detection in the filtered estimate (0..1). The camera_info's distortion_model may be
        plumb_bob/rational_polynomial (cv2.solvePnP handles it) or equidistant (fisheye, as
        tools/calibrate_camera.py writes for the wide lens): corners are undistorted first."""
        self.node = node
        self.marker_id = marker_id
        self.cam_offset = np.array(cam_offset, float)
        self.alpha = smoothing
        s = marker_size / 2.0
        # Corner order returned by the detector: top-left, top-right, bottom-right, bottom-left.
        self.obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
        self.detector = make_detector(dictionary)

        self.K = None
        self.D = None
        self.fisheye = False
        self.position = None      # filtered marker centre, ENU (np.array 3)
        self.normal = None        # filtered horizontal unit vector the marker faces, ENU (2)
        self.last_seen = 0.0      # time.time() of the last detection
        self.raw_range = None     # camera-to-marker distance from the last detection (m)
        self.detections = 0

        group = MutuallyExclusiveCallbackGroup()   # image work doesn't block MAVROS callbacks
        # Reliable delivery: 2.7 MB frames over best-effort DDS lose fragments, which dropped the
        # sim camera from 15 Hz to ~10 Hz with gaps over 1 s. Keep only the newest frame. Use
        # reliable=False if a camera driver publishes best-effort only.
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
               if reliable else qos_profile_sensor_data)
        node.create_subscription(CameraInfo, info_topic, self._on_info, qos, callback_group=group)
        node.create_subscription(Image, image_topic, self._on_image, qos, callback_group=group)

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
            self.fisheye = msg.distortion_model in ('equidistant', 'fisheye')

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
        img_pts = corners[idx[0]][0].astype(np.float64)
        D = self.D
        if self.fisheye:   # undistort with the fisheye model, then solve as an ideal pinhole
            img_pts = cv2.fisheye.undistortPoints(img_pts.reshape(-1, 1, 2), self.K,
                                                  self.D[:4], P=self.K).reshape(-1, 2)
            D = np.zeros(5)
        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_pts, self.K, D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
            return   # degenerate solve (seen at grazing angles far away); a NaN would poison the filter

        R_wb = np.array(self.node.rotation())
        t_cam = tvec.ravel()
        n_cam = cv2.Rodrigues(rvec)[0][:, 2]          # marker +z: out of its face, toward us
        pos = self.camera_position() + R_wb @ (R_BODY_CAM @ t_cam)
        n = R_wb @ (R_BODY_CAM @ n_cam)
        n_h = np.array([n[0], n[1]])
        if not np.all(np.isfinite(pos)) or not np.linalg.norm(n_h) > 1e-3:
            return
        n_h /= np.linalg.norm(n_h)

        if self.position is None or np.linalg.norm(pos - self.position) > 1.0:
            self.position, self.normal = pos, n_h     # first sighting, or a jump: restart filter
        else:
            a = self.alpha
            self.position = (1 - a) * self.position + a * pos
            # A planar square has two pose solutions that can swap far away; a facing direction
            # more than 90 deg from the current estimate is that flip, so skip it.
            if float(np.dot(n_h, self.normal)) > 0.0:
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


def make_detector(dictionary='DICT_4X4_50'):
    """ArUco detector with a detectMarkers(gray) method on any OpenCV 4.x. OpenCV >= 4.7 has
    cv2.aruco.ArucoDetector; the 4.6 in Ubuntu 24.04 (the drone's ROS 2 Jazzy image) only has the
    old function API."""
    aruco = cv2.aruco
    if hasattr(aruco, 'ArucoDetector'):
        return aruco.ArucoDetector(aruco.getPredefinedDictionary(getattr(aruco, dictionary)),
                                   aruco.DetectorParameters())

    class _Legacy:
        def __init__(self):
            self.dictionary = aruco.Dictionary_get(getattr(aruco, dictionary))
            self.params = aruco.DetectorParameters_create()

        def detectMarkers(self, gray):
            return aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
    return _Legacy()


def to_gray(msg):
    """sensor_msgs/Image -> grayscale numpy array (no cv_bridge needed)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
    if enc in ('rgba8', 'bgra8'):    # the Pi camera's default XRGB8888 arrives as one of these
        img = buf.reshape(msg.height, msg.step // 4, 4)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY if enc == 'rgba8' else cv2.COLOR_BGRA2GRAY)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    return None
