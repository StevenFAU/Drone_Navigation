#!/usr/bin/env python3
"""Check the camera pipeline: grab one frame from /camera/image, detect ArUco, print the pose.

    source setup/env.sh && python3 sim/tools/check_marker.py              # saves logs/camera_check.png
    python3 sim/tools/check_marker.py --size 0.20 --out /tmp/frame.png

Prints the marker position in the camera optical frame (x right, y down, z forward), from
cv2.solvePnP with the intrinsics in /camera/camera_info. Run it while the sim is up.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

ap = argparse.ArgumentParser()
ap.add_argument('--size', type=float, default=0.20, help='marker side in metres (black border included)')
ap.add_argument('--dict', default='DICT_4X4_50')
ap.add_argument('--out', default=os.path.join(os.environ.get('DRONE_MISSION_ROOT', '.'), 'logs', 'camera_check.png'))
args = ap.parse_args()
rclpy.init()
n = rclpy.create_node('check_marker')
got = {}
n.create_subscription(Image, '/camera/image', lambda m: got.setdefault('img', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/camera/camera_info', lambda m: got.setdefault('info', m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < 2 and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)
if len(got) < 2:
    print('no', ' / '.join(sorted({'image', 'camera_info'} - {'image' if 'img' in got else '', 'camera_info' if 'info' in got else ''})), 'received - is the sim running (scripts/start_sim.sh)?')
    sys.exit(1)
m, info = got['img'], got['info']
img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
cv2.imwrite(args.out, bgr)
K = np.array(info.k).reshape(3, 3); D = np.array(info.d) if len(info.d) else np.zeros(5)
print(f'image {m.width}x{m.height} {m.encoding}; fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}')
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict)))
corners, ids, rej = det.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
print('ids:', None if ids is None else ids.ravel().tolist(), f'(rejected candidates: {len(rej)})')
if ids is not None:
    s = args.size / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, corners[0][0], K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    t = tvec.ravel()
    print(f'marker in camera frame: x={t[0]:.3f} (right) y={t[1]:.3f} (down) z={t[2]:.3f} (forward) m, range {np.linalg.norm(t):.3f} m')
    cv2.aruco.drawDetectedMarkers(bgr, corners, ids); cv2.imwrite(args.out, bgr)
print('frame saved to', args.out)
n.destroy_node()
rclpy.try_shutdown()
