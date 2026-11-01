#!/usr/bin/env python3
"""Compute camera intrinsics from saved chessboard images.

Writes the result in the same OpenCV YAML format as
robot_vision/config/camera_parameter.yaml, and saves an undistorted
example image so you can check the result.

Usage:
    python3 calibrate.py --images ~/calib_images --cols 10 --rows 7 --square 0.0207 \
        --out ~/calib_images/camera_parameter.yaml
"""
import argparse
import glob
import os

import cv2
import numpy as np


def find_corners(gray, pattern):
    found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_ACCURACY)
    if found:
        return corners, 'SB'
    # SB gives up on low-contrast boards (e.g. backlit), the classic detector does not
    found, corners = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None, None
    corners = cv2.cornerSubPix(
        gray, corners, (4, 4), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001))
    return corners, 'classic'


def write_yaml(path, K, D, w, h):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write('image_width', w)
    fs.write('image_height', h)
    fs.write('camera_name', 'camera')
    fs.write('camera_matrix', K)
    fs.write('distortion_model', 'plumb_bob')
    fs.write('distortion_coefficients', D.reshape(1, -1))
    fs.write('rectification_matrix', np.eye(3))
    P = np.zeros((3, 4))
    P[:, :3] = K
    fs.write('projection_matrix', P)
    fs.release()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', nargs='+', default=['~/calib_images'])
    p.add_argument('--cols', type=int, default=10)
    p.add_argument('--rows', type=int, default=7)
    p.add_argument('--square', type=float, default=0.0207, help='square size in meters')
    p.add_argument('--out', default='~/calib_images/camera_parameter.yaml')
    args = p.parse_args()

    files = []
    for folder in args.images:
        files += sorted(glob.glob(os.path.join(os.path.expanduser(folder), 'calib_*.png')))
    pattern = (args.cols, args.rows)

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    obj_points, img_points, used = [], [], []
    methods = {'SB': 0, 'classic': 0}
    w = h = None
    for f in files:
        gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        h, w = gray.shape
        corners, method = find_corners(gray, pattern)
        if corners is None:
            print(f'  skip (board not found): {os.path.basename(f)}')
            continue
        methods[method] += 1
        obj_points.append(objp)
        img_points.append(corners)
        used.append(f)
    print(f'Using {len(used)}/{len(files)} images, size {w}x{h}, detectors {methods}')
    if len(used) < 10:
        raise SystemExit('Need at least 10 good images.')

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

    # Per-image reprojection error, to spot bad images
    errors = []
    for objp_i, imgp, rv, tv in zip(obj_points, img_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(objp_i, rv, tv, K, D)
        diff = proj.reshape(-1, 2) - imgp.reshape(-1, 2)
        errors.append(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))
    for f, e in sorted(zip(used, errors), key=lambda x: -x[1])[:5]:
        print(f'  worst: {os.path.basename(f)}  {e:.3f} px')

    print(f'\nRMS reprojection error: {rms:.3f} px  (good: < 0.5, OK: < 1.0)')
    print('Camera matrix K:\n', np.array2string(K, precision=3, suppress_small=True))
    print('Distortion [k1 k2 p1 p2 k3]:\n', np.array2string(D.ravel(), precision=5))
    fov_x = 2 * np.degrees(np.arctan(w / (2 * K[0, 0])))
    fov_y = 2 * np.degrees(np.arctan(h / (2 * K[1, 1])))
    print(f'Field of view: {fov_x:.1f} deg (H) x {fov_y:.1f} deg (V)')

    out = os.path.expanduser(args.out)
    write_yaml(out, K, D, w, h)
    print(f'\nWrote {out}')

    # Side-by-side original vs undistorted for a visual check
    img = cv2.imread(used[0])
    und = cv2.undistort(img, K, D)
    both = np.hstack([img, und])
    check = os.path.join(os.path.dirname(out), 'undistort_check.jpg')
    cv2.imwrite(check, cv2.resize(both, (both.shape[1] // 2, both.shape[0] // 2)))
    print(f'Wrote {check} (left: original, right: undistorted)')


if __name__ == '__main__':
    main()
