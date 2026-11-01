from dataclasses import dataclass

import cv2
import numpy as np

# Turn a pixel box into range and bearing, given the landmark's real height.
#
# Lens distortion is removed first: the box top, bottom and center are mapped to
# normalized image coordinates, where a point at (X, Y, Z) in the camera frame
# lands on (X/Z, Y/Z). A vertical landmark of height H then spans H/Z, so
#
#   Z = H / (y_bottom - y_top)    forward distance
#   X = Z * x_center              lateral offset, positive to the right
#
# Boxes cut by the image border are rejected: their height is not the landmark's
# height, so the range would come out too long. The box height comes from the
# nearest face, so the range is to the front of the landmark, not its center.

EDGE_PX = 3.0  # a box this close to the border counts as cut


@dataclass
class Observation:
    forward: float  # m along the optical axis
    left: float     # m, positive to the left of the optical axis
    range: float    # m in the horizontal plane
    bearing: float  # rad, positive to the left (counterclockwise), like the robot's heading


def is_cut(box, width, height, edge=EDGE_PX):
    x1, y1, x2, y2 = box
    return x1 < edge or y1 < edge or x2 > width - 1 - edge or y2 > height - 1 - edge


def estimate(box, landmark_height, K, D, width, height):
    """Return an Observation for a box (x1, y1, x2, y2), or None if it is unusable."""
    if is_cut(box, width, height):
        return None
    x1, y1, x2, y2 = box
    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    points = np.array([[[u, y1]], [[u, y2]], [[u, v]]], dtype=np.float64)
    (_, top), (_, bottom), (x_center, _) = cv2.undistortPoints(points, K, D).reshape(-1, 2)
    if bottom <= top:
        return None
    forward = landmark_height / float(bottom - top)
    right = forward * float(x_center)
    return Observation(forward, -right, float(np.hypot(forward, right)), float(np.arctan2(-right, forward)))
