#!/usr/bin/env python3
"""Check the calibration against tape-measured distances.

Estimates the chessboard pose from the camera intrinsics and shows the distance
on /calib/distance/compressed. When the board holds still and faces the camera,
the distance is logged once to the terminal and the CSV file. Comparing these to
measured distances gives the scale error (focal length) and a fixed offset
(lens position relative to where the tape starts).

Usage (on the rover, with the camera node running):
    python3 distance_check.py --square 0.0207
"""
import argparse
import collections
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

MAX_TILT = 8.0        # deg; beyond this the reading is flagged and not logged
STABLE_FRAMES = 15    # frames that must agree before logging
STABLE_RANGE = 0.004  # m; max spread of Z across those frames
MOVED = 0.02          # m; board must move this far before the next log


def load_intrinsics(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    K, D = fs.getNode('camera_matrix').mat(), fs.getNode('distortion_coefficients').mat()
    fs.release()
    if K is None or D is None:
        raise SystemExit(f'Could not read intrinsics from {path}')
    return K, D


def board_pose(gray, pattern, objp, K, D):
    """Return (Z in m, tilt in deg, corners), or None if the board is not visible."""
    found, corners = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    corners = cv2.cornerSubPix(
        gray, corners, (4, 4), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    center = R @ objp.mean(axis=0) + tvec.ravel()
    tilt = float(np.degrees(np.arccos(abs(R[2, 2]))))
    return float(center[2]), tilt, corners


class DistanceCheck(Node):
    def __init__(self, args):
        super().__init__('distance_check')
        self.pattern = (args.cols, args.rows)
        self.K, self.D = load_intrinsics(os.path.expanduser(args.params))
        self.objp = np.zeros((args.cols * args.rows, 3), np.float64)
        self.objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
        self.csv = os.path.expanduser(args.csv)
        self.recent = collections.deque(maxlen=STABLE_FRAMES)
        self.last_logged = None

        self.create_subscription(CompressedImage, args.topic, self.on_image, qos_profile_sensor_data)
        self.preview_pub = self.create_publisher(CompressedImage, '/calib/distance/compressed', 1)
        self.get_logger().info(f'fx={self.K[0, 0]:.2f} fy={self.K[1, 1]:.2f} square={args.square} m')

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        pose = board_pose(gray, self.pattern, self.objp, self.K, self.D)

        status = 'board not found'
        if pose is None:
            self.recent.clear()
        else:
            z, tilt, corners = pose
            cv2.drawChessboardCorners(img, self.pattern, corners, True)
            status = f'Z = {z * 100:.1f} cm  tilt {tilt:.1f} deg'
            if tilt > MAX_TILT:
                status += ' - face the camera'
                self.recent.clear()
            else:
                self.recent.append(z)
                status += self.update_log(tilt)

        cv2.rectangle(img, (0, 0), (img.shape[1], 44), (0, 0, 0), -1)
        cv2.putText(img, status, (10, 31), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        small = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
        ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.preview_pub.publish(out)

    def update_log(self, tilt):
        if len(self.recent) < STABLE_FRAMES or np.ptp(self.recent) > STABLE_RANGE:
            return ''
        z = float(np.median(self.recent))
        if self.last_logged is not None and abs(z - self.last_logged) < MOVED:
            return '  LOGGED'
        self.last_logged = z
        with open(self.csv, 'a') as f:
            f.write(f'{time.time():.1f},{z:.4f},{tilt:.1f}\n')
        self.get_logger().info(f'LOGGED Z = {z * 100:.1f} cm (tilt {tilt:.1f} deg)')
        return '  LOGGED'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cols', type=int, default=10)
    p.add_argument('--rows', type=int, default=7)
    p.add_argument('--square', type=float, default=0.0207, help='square size in meters')
    p.add_argument('--params', default='~/ros2_ws/src/robot_vision/config/camera_parameter.yaml')
    p.add_argument('--csv', default='~/distance_check.csv')
    p.add_argument('--topic', default='/camera/image_raw/compressed')
    args = p.parse_args()

    rclpy.init()
    node = DistanceCheck(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
