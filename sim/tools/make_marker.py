#!/usr/bin/env python3
"""Generate the ArUco texture used by sim/models/aruco_marker (and a printable copy).

    python3 sim/tools/make_marker.py            # DICT_4X4_50, id 0
    python3 sim/tools/make_marker.py --id 3

The PNG is the marker only (black border included, no white margin): the model maps it
onto a square exactly `marker_size` wide and puts a white board behind it. Print the same
PNG at the same size (default 0.20 m) for the real drone.
"""
import argparse
import os

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'models', 'aruco_marker', 'materials', 'textures', 'marker.png')

p = argparse.ArgumentParser()
p.add_argument('--dict', default='DICT_4X4_50')
p.add_argument('--id', type=int, default=0)
p.add_argument('--px', type=int, default=600, help='image side in pixels')
args = p.parse_args()

dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
img = cv2.aruco.generateImageMarker(dictionary, args.id, args.px)
cv2.imwrite(OUT, img)
print(f'wrote {os.path.normpath(OUT)} ({args.dict} id {args.id}, {args.px}px)')
