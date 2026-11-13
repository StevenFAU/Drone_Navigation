"""Save camera frames as JPGs, e.g. calibration photos, from the same stream the mission uses
(same resolution and focus). On the Pi, from ~/Drone_Navigation/robot:

    docker compose run --rm mission python3 -m drone_mission.grab_frames --out /logs/calib --count 25 --every 2
    docker compose run --rm mission python3 -m drone_mission.grab_frames --out /logs/marker_1m --count 1

Photos land in ~/Drone_Navigation/logs/... on the Pi; copy them to the laptop with
    scp -r group7@10.43.0.42:Drone_Navigation/logs/calib .
"""
import argparse
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def to_bgr(msg):
    import numpy as np
    buf = np.frombuffer(msg.data, np.uint8)
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img.copy()
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(msg.height, msg.step // 4, 4)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width].copy()
    raise ValueError(f'unsupported encoding {msg.encoding}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--out', default='/logs/frames')
    ap.add_argument('--count', type=int, default=25)
    ap.add_argument('--every', type=float, default=2.0, help='seconds between photos')
    ap.add_argument('--start-delay', type=float, default=3.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Node('grab_frames')
    latest = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(Image, args.topic, lambda m: latest.update(msg=m), qos)

    print(f'waiting for {args.topic}...', flush=True)
    while 'msg' not in latest:
        rclpy.spin_once(node, timeout_sec=0.2)
    m = latest['msg']
    print(f'{m.width}x{m.height} {m.encoding}. First photo in {args.start_delay:.0f} s', flush=True)
    next_t = time.time() + args.start_delay
    saved = 0
    while saved < args.count:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= next_t:
            path = os.path.join(args.out, f'frame_{saved:03d}.jpg')
            cv2.imwrite(path, to_bgr(latest['msg']), [cv2.IMWRITE_JPEG_QUALITY, 95])
            saved += 1
            print(f'  {saved}/{args.count} {path}' + ('  (move the board)' if saved < args.count else ''),
                  flush=True)
            next_t = time.time() + args.every
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
