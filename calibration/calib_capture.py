#!/usr/bin/env python3
"""Capture chessboard images for camera calibration.

Subscribes to the camera topic, detects the chessboard, and saves a frame
whenever the board is in a new enough position/size/tilt. Publishes an
annotated preview on /calib/preview/compressed so you can watch in Foxglove.

Usage (on the rover, with the camera node running):
    python3 calib_capture.py --cols 10 --rows 7 --out ~/calib_images
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CalibCapture(Node):
    def __init__(self, args):
        super().__init__('calib_capture')
        self.pattern = (args.cols, args.rows)
        self.out = os.path.expanduser(args.out)
        self.max_images = args.max
        self.min_interval = args.interval
        self.min_square = args.min_square
        os.makedirs(self.out, exist_ok=True)
        # Continue numbering so a later session never overwrites earlier images
        existing = glob.glob(os.path.join(self.out, 'calib_*.png'))
        self.next_idx = max((int(os.path.basename(f)[6:9]) for f in existing), default=-1) + 1

        self.saved = []  # (cx, cy, size, skew) of each saved view, normalized
        self.last_save = 0.0
        self.tilted = 0
        self.coverage = None

        self.create_subscription(CompressedImage, args.topic, self.on_image, qos_profile_sensor_data)
        self.preview_pub = self.create_publisher(CompressedImage, '/calib/preview/compressed', 1)
        self.get_logger().info(f'Saving to {self.out}, pattern {self.pattern}, max {self.max_images}')

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        h, w = img.shape[:2]
        if self.coverage is None:
            self.coverage = np.zeros((h, w), dtype=np.uint8)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Full-size detection: squares can be only ~15 px on a small board
        found, corners = cv2.findChessboardCorners(
            gray, self.pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)

        preview = img.copy()
        status = 'board not found'
        if found:
            # Small window so the search stays inside one square at long range
            corners = cv2.cornerSubPix(
                gray, corners, (4, 4), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            cv2.drawChessboardCorners(preview, self.pattern, corners, found)
            view = self.describe(corners, w, h)
            square_px, persp = self.measure(corners)
            if square_px < self.min_square:
                status = f'too far ({square_px:.0f}px) - move closer'
            elif not self.is_new(view) or time.time() - self.last_save < self.min_interval:
                status = 'board found - move to a new pose'
            elif not cv2.findChessboardCornersSB(gray, self.pattern, cv2.CALIB_CB_ACCURACY)[0]:
                # Slow, so only run on frames that would otherwise be saved
                status = 'low contrast/blurry - add light, hold still'
            else:
                self.tilted += persp > 1.12
                self.save(img, corners, view)
                status = f'SAVED #{len(self.saved)}'

        self.draw_overlay(preview, status)
        self.publish_preview(preview, msg.header)

        if len(self.saved) >= self.max_images:
            self.get_logger().info(f'Collected {len(self.saved)} images, done.')
            raise SystemExit

    def describe(self, corners, w, h):
        pts = corners.reshape(-1, 2)
        cx, cy = pts.mean(axis=0)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        size = np.sqrt((x1 - x0) * (y1 - y0) / (w * h))
        # Skew: ratio of the two side lengths of the board outline (tilt indicator)
        c, r = self.pattern
        tl, tr, bl = pts[0], pts[c - 1], pts[(r - 1) * c]
        skew = np.linalg.norm(tr - tl) / max(np.linalg.norm(bl - tl), 1e-6)
        return np.array([cx / w, cy / h, size, skew])

    def measure(self, corners):
        # Median square size in px, and how far the outline is from a rectangle
        # (ratio of opposite sides; 1.0 = board faces the camera, no tilt)
        c, r = self.pattern
        grid = corners.reshape(r, c, 2)
        square_px = float(np.median(np.linalg.norm(np.diff(grid, axis=1), axis=2)))
        top = np.linalg.norm(grid[0, -1] - grid[0, 0])
        bottom = np.linalg.norm(grid[-1, -1] - grid[-1, 0])
        left = np.linalg.norm(grid[-1, 0] - grid[0, 0])
        right = np.linalg.norm(grid[-1, -1] - grid[0, -1])
        persp = max(max(top, bottom) / min(top, bottom), max(left, right) / min(left, right))
        return square_px, float(persp)

    def is_new(self, view):
        if not self.saved:
            return True
        d = np.abs(np.array(self.saved) - view)
        # New if it differs enough from every saved view in position, size, or tilt
        diff = (d[:, 0] > 0.08) | (d[:, 1] > 0.08) | (d[:, 2] > 0.06) | (d[:, 3] > 0.15)
        return bool(diff.all())

    def save(self, img, corners, view):
        name = f'calib_{self.next_idx:03d}.png'
        self.next_idx += 1
        cv2.imwrite(os.path.join(self.out, name), img)
        self.saved.append(view)
        self.last_save = time.time()
        hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.int32))
        cv2.fillConvexPoly(self.coverage, hull, 255)
        self.get_logger().info(
            f'Saved {name} (#{len(self.saved)} this session): center=({view[0]:.2f},{view[1]:.2f}) size={view[2]:.2f} skew={view[3]:.2f}')

    def draw_overlay(self, preview, status):
        # Tint the areas of the image already covered by saved boards
        green = np.zeros_like(preview)
        green[:, :, 1] = self.coverage
        cv2.addWeighted(preview, 1.0, green, 0.3, 0, dst=preview)
        covered = 100.0 * np.count_nonzero(self.coverage) / self.coverage.size
        text = (f'{status} | saved {len(self.saved)}/{self.max_images} '
                f'| tilted {self.tilted} | coverage {covered:.0f}%')
        cv2.rectangle(preview, (0, 0), (preview.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(preview, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    def publish_preview(self, preview, header):
        small = cv2.resize(preview, (preview.shape[1] // 2, preview.shape[0] // 2))
        ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            out = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.preview_pub.publish(out)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cols', type=int, default=10, help='inner corners along the long side')
    p.add_argument('--rows', type=int, default=7, help='inner corners along the short side')
    p.add_argument('--out', default='~/calib_images')
    p.add_argument('--max', type=int, default=40)
    p.add_argument('--interval', type=float, default=1.0, help='min seconds between saves')
    p.add_argument('--min-square', type=float, default=22.0, help='reject boards with smaller squares (px)')
    p.add_argument('--topic', default='/camera/image_raw/compressed')
    args = p.parse_args()

    rclpy.init()
    node = CalibCapture(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    node.get_logger().info(f'Total saved: {len(node.saved)} images in {node.out}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
