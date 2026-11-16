#!/usr/bin/env python3
"""Calibrate the real camera (Pi Camera 3 Wide) from chessboard photos, and check marker range.

1. Print robot/print/chessboard_9x6_25mm_letter.pdf at 100% ("actual size", not "fit to page"),
   check the 100 mm scale bar and one square (25 mm) with a ruler, and tape it flat to something
   rigid (cardboard bends: use a clipboard, foam board or a cabinet door).
2. On the Pi, take ~25 photos from the camera stream the mission uses (1280x720, focus locked).
   Move the board around the whole image, especially the corners and edges where the wide lens
   bends most, at 0.4-1.5 m, and tilt it up to ~45 deg:
       cd ~/Drone_Navigation/robot
       docker compose run --rm mission python3 -m drone_mission.grab_frames --out /logs/calib --count 25 --every 2
3. Copy them to the laptop and calibrate:
       scp -r group7@10.43.0.42:Drone_Navigation/logs/calib calib_photos
       python3 tools/calibrate_camera.py calib_photos/ --square 0.025 --show
   then copy camera.yaml to robot/config/camera.yaml, scripts/drone_sync.sh, and on the Pi
   `docker compose restart camera`.
   It fits both the standard pinhole model (OpenCV "rational", 8 coefficients) and the fisheye
   model, prints the reprojection error (RMS, pixels) of each, and writes the better one as a
   ROS camera_info YAML (camera.yaml). The marker tracker handles both. Under ~0.5 px is good.
4. Check range against a tape measure: print robot/print/aruco_4x4_id0_200mm_letter.pdf at
   100%, stand it 1.00 m (then 2 and 3 m) from the camera lens, grab one frame each
   (`... grab_frames --out /logs/marker_1m --count 1`), copy it over, then
       python3 tools/calibrate_camera.py --test camera.yaml marker_1m.jpg --marker-size 0.20
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def find_corners(paths, board, show_dir=None):
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    img_pts, used, size = [], [], None
    for p in paths:
        gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        if size is None:
            size = gray.shape[::-1]
        elif gray.shape[::-1] != size:
            print(f'  skip {os.path.basename(p)}: {gray.shape[::-1]} differs from {size}')
            continue
        ok, c = cv2.findChessboardCornersSB(gray, board, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if not ok:
            ok, c = cv2.findChessboardCorners(gray, board, flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
            if ok:
                c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), crit)
        print(f"  {os.path.basename(p)}: {'ok' if ok else 'no board'}")
        if ok:
            img_pts.append(c.reshape(-1, 1, 2).astype(np.float64))
            used.append(p)
            if show_dir:
                vis = cv2.drawChessboardCorners(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), board, c, ok)
                cv2.imwrite(os.path.join(show_dir, 'corners_' + os.path.basename(p)), vis)
    return img_pts, used, size


def calibrate(img_pts, board, square, size):
    objp = np.zeros((board[0] * board[1], 1, 3), np.float64)
    objp[:, 0, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square
    obj_pts = [objp] * len(img_pts)
    results = {}
    # Pinhole, rational model (handles moderate wide-angle distortion)
    rms, K, D, _, _ = cv2.calibrateCamera(
        [o.astype(np.float32) for o in obj_pts], [i.astype(np.float32) for i in img_pts], size,
        None, None, flags=cv2.CALIB_RATIONAL_MODEL)
    results['rational_polynomial'] = (rms, K, D.ravel()[:8])   # ROS uses k1 k2 p1 p2 k3 k4 k5 k6
    # Fisheye (equidistant) model
    try:
        Kf, Df = np.zeros((3, 3)), np.zeros((4, 1))
        rms_f, Kf, Df, _, _ = cv2.fisheye.calibrate(
            obj_pts, img_pts, size, Kf, Df,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7))
        results['equidistant'] = (rms_f, Kf, Df.ravel())
    except cv2.error as e:
        print(f'fisheye fit failed ({e.err}); usually a bad photo, try removing blurry ones')
    return results


def write_yaml(path, model, K, D, size, name='pi_camera_3_wide'):
    w, h = size
    k = ', '.join(f'{v:.6f}' for v in K.ravel())
    d = ', '.join(f'{v:.8f}' for v in D)
    P = np.hstack([K, np.zeros((3, 1))])
    p = ', '.join(f'{v:.6f}' for v in P.ravel())
    with open(path, 'w') as f:
        f.write(f"""image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{k}]
distortion_model: {model}
distortion_coefficients:
  rows: 1
  cols: {len(D)}
  data: [{d}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{p}]
""")


def read_yaml(path):
    # Parse the simple ROS camera_info layout without a YAML dependency
    txt = open(path).read()
    def block(key):
        i = txt.index(key)
        j = txt.index('data: [', i) + 7
        return np.array([float(v) for v in txt[j:txt.index(']', j)].split(',')])
    model = txt.split('distortion_model:')[1].split()[0]
    return model, block('camera_matrix').reshape(3, 3), block('distortion_coefficients')


def test_marker(calib, image, size_m, dictionary):
    model, K, D = read_yaml(calib)
    gray = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    aruco = cv2.aruco
    if hasattr(aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        det = aruco.ArucoDetector(aruco.getPredefinedDictionary(getattr(aruco, dictionary)))
        corners, ids, _ = det.detectMarkers(gray)
    else:                                         # OpenCV 4.6 (the drone's Jazzy container)
        corners, ids, _ = aruco.detectMarkers(gray, aruco.Dictionary_get(getattr(aruco, dictionary)),
                                              parameters=aruco.DetectorParameters_create())
    if ids is None:
        sys.exit('no marker found in the image')
    s = size_m / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    for c, i in zip(corners, ids.ravel()):
        pts = c[0].astype(np.float64)
        dist = D
        if model in ('equidistant', 'fisheye'):
            pts = cv2.fisheye.undistortPoints(pts.reshape(-1, 1, 2), K, D[:4], P=K).reshape(-1, 2)
            dist = np.zeros(5)
        ok, rvec, tvec = cv2.solvePnP(obj, pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        t = tvec.ravel()
        print(f'marker id {i}: distance {np.linalg.norm(t):.3f} m  (forward {t[2]:.3f}, right {t[0]:+.3f}, '
              f'down {t[1]:+.3f} m)  using {model} calibration')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('photos', nargs='?', help='folder (or glob) of chessboard photos')
    ap.add_argument('--board', default='9x6', help='inner corners, columns x rows')
    ap.add_argument('--square', type=float, default=0.025, help='square size in metres')
    ap.add_argument('--out', default='camera.yaml')
    ap.add_argument('--model', choices=['auto', 'fisheye', 'pinhole'], default='auto')
    ap.add_argument('--show', action='store_true', help='save corner overlays + an undistorted preview')
    ap.add_argument('--test', nargs=2, metavar=('CALIB_YAML', 'IMAGE'),
                    help='measure the marker distance in IMAGE with an existing calibration')
    ap.add_argument('--marker-size', type=float, default=0.20)
    ap.add_argument('--dict', default='DICT_4X4_50')
    args = ap.parse_args()

    if args.test:
        test_marker(args.test[0], args.test[1], args.marker_size, args.dict)
        return
    if not args.photos:
        ap.error('give a folder of chessboard photos, or --test')
    paths = sorted(glob.glob(os.path.join(args.photos, '*')) if os.path.isdir(args.photos)
                   else glob.glob(args.photos))
    paths = [p for p in paths if p.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    board = tuple(int(v) for v in args.board.lower().split('x'))
    show_dir = os.path.splitext(args.out)[0] + '_check' if args.show else None
    if show_dir:
        os.makedirs(show_dir, exist_ok=True)
    print(f'{len(paths)} photos, board {board[0]}x{board[1]} inner corners, {args.square * 1000:.1f} mm squares')
    img_pts, used, size = find_corners(paths, board, show_dir)
    if len(img_pts) < 8:
        sys.exit(f'only {len(img_pts)} usable photos; take at least ~15 with the whole board visible')
    results = calibrate(img_pts, board, args.square, size)
    print(f'\n{len(img_pts)} photos used, image {size[0]}x{size[1]}')
    for m, (rms, K, D) in results.items():
        print(f'  {m:20s} RMS {rms:.3f} px   fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}')
    if args.model == 'fisheye':
        model = 'equidistant'
    elif args.model == 'pinhole':
        model = 'rational_polynomial'
    else:
        model = min(results, key=lambda m: results[m][0])
    rms, K, D = results[model]
    write_yaml(args.out, model, K, D, size)
    hfov = 2 * np.degrees(np.arctan(size[0] / (2 * K[0, 0])))
    print(f'\nwrote {args.out} ({model}, RMS {rms:.3f} px). Pinhole-equivalent horizontal FOV at the centre: '
          f'{hfov:.0f} deg')
    if rms > 1.0:
        print('RMS over 1 px: check for blurry photos or a board that is not flat, and recalibrate.')
    if show_dir:
        img = cv2.imread(used[0])
        if model == 'equidistant':
            newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D[:4], size, np.eye(3), balance=0.5)
            und = cv2.fisheye.undistortImage(img, K, D[:4], Knew=newK)
        else:
            und = cv2.undistort(img, K, D)
        cv2.imwrite(os.path.join(show_dir, 'undistorted_preview.jpg'), und)
        print(f'corner overlays and undistorted_preview.jpg in {show_dir}/ (straight lines should look straight)')


if __name__ == '__main__':
    main()
