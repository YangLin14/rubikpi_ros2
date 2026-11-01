import math

import cv2
import numpy as np
import pytest

from landmark_detector.geometry import estimate, is_cut

# Final calibration of the rover camera
K = np.array([[648.25, 0.0, 639.82], [0.0, 648.44, 306.78], [0.0, 0.0, 1.0]])
D = np.array([-0.29729, 0.08772, -0.00128, 0.00054, -0.01165])
W, H = 1280, 720
CAMERA_HEIGHT = 0.12  # m above the floor


def box_of(forward, left, height, width=0.1):
    """Project a flat object standing on the floor, facing the camera, to its pixel box."""
    pts = []
    for dx in (-width / 2, width / 2):
        for top in (0.0, height):
            pts.append([-left + dx, CAMERA_HEIGHT - top, forward])
    px, _ = cv2.projectPoints(np.array(pts), np.zeros(3), np.zeros(3), K, D)
    px = px.reshape(-1, 2)
    return px[:, 0].min(), px[:, 1].min(), px[:, 0].max(), px[:, 1].max()


@pytest.mark.parametrize('forward', [0.8, 1.5, 3.0])
@pytest.mark.parametrize('angle', [-25.0, 0.0, 25.0])
def test_range_and_bearing(forward, angle):
    left = forward * math.tan(math.radians(angle))
    obs = estimate(box_of(forward, left, 0.3), 0.3, K, D, W, H)
    assert obs.range == pytest.approx(math.hypot(forward, left), rel=0.03)
    assert math.degrees(obs.bearing) == pytest.approx(angle, abs=1.0)


def test_left_is_positive():
    assert estimate(box_of(1.5, 0.5, 0.3), 0.3, K, D, W, H).left > 0


def test_cut_box_is_rejected():
    assert is_cut((0.0, 100.0, 50.0, 200.0), W, H)
    assert estimate((0.0, 100.0, 50.0, 200.0), 0.3, K, D, W, H) is None
    assert not is_cut((100.0, 100.0, 200.0, 200.0), W, H)
