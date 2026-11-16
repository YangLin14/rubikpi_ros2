import math
import random

import numpy as np

from robot_control.ekf import wrap_angle
from robot_control.slam import SlamEkf

# 8 ft arena centered on the origin, two landmarks per side. The robot starts at
# START (unknown to the filter, which uses its own start as the map origin).
H = 1.22
LANDMARKS = {'a': (H, -0.6), 'b': (H, 0.6), 'c': (0.6, H), 'd': (-0.6, H),
             'e': (-H, 0.6), 'f': (-H, -0.6), 'g': (-0.6, -H), 'h': (0.6, -H)}
START = (-0.457, -0.457, 0.0)
FOV = math.radians(45.0)
SQUARE = [(0.914, 0.0), (0.914, 0.914), (0.0, 0.914), (0.0, 0.0)]


def to_world(p):
    x0, y0, t0 = START
    c, s = math.cos(t0), math.sin(t0)
    return x0 + c * p[0] - s * p[1], y0 + s * p[0] + c * p[1]


def run(seed, loops=2, speed_scale=0.93, false_hits=0):
    rng = random.Random(seed)
    ekf = SlamEkf()
    x, y, th = START
    dt = 0.05

    def look(n=4):
        for _ in range(n):
            for name, (lx, ly) in LANDMARKS.items():
                r = math.hypot(lx - x, ly - y)
                b = wrap_angle(math.atan2(ly - y, lx - x) - th)
                if abs(b) < FOV and 0.3 < r < 2.3:
                    ekf.observe(name, r * (1 + rng.gauss(0, 0.04)), b + rng.gauss(0, math.radians(2)),
                                0.05 * r, math.radians(2))
            for _ in range(false_hits):  # one-off false detection somewhere random
                ekf.observe('ghost', rng.uniform(0.5, 2.0), rng.uniform(-0.7, 0.7), 0.1, 0.05)

    look()
    for _ in range(loops):
        for gx, gy in SQUARE:
            gwx, gwy = to_world((gx, gy))
            for _ in range(400):  # turn using the estimate
                err = wrap_angle(math.atan2(gy - ekf.state[1], gx - ekf.state[0]) - ekf.state[2])
                if abs(err) < math.radians(3):
                    break
                w = math.copysign(1.5, err)
                th = wrap_angle(th + w * (1 + rng.gauss(0, 0.05)) * dt)
                ekf.predict(0.0, w, dt)
            look()
            for _ in range(2000):  # drive; true speed slower than assumed
                if (gx - ekf.state[0]) * math.cos(ekf.state[2]) + (gy - ekf.state[1]) * math.sin(ekf.state[2]) <= 0:
                    break
                x += 0.31 * speed_scale * dt * math.cos(th)
                y += 0.31 * speed_scale * dt * math.sin(th)
                ekf.predict(0.31, 0.0, dt)
            look()
    errors = []
    for name, i in ekf.index.items():
        if name in LANDMARKS:
            errors.append(math.dist(to_world(ekf.state[i:i + 2]), LANDMARKS[name]))
    return ekf, errors, (x, y)


def test_finds_all_landmarks_accurately():
    for seed in range(5):
        ekf, errors, _ = run(seed)
        assert set(ekf.index) == set(LANDMARKS)
        assert np.mean(errors) < 0.12, errors


def test_covariance_stays_symmetric_and_positive():
    ekf, _, _ = run(1)
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-9)
    assert np.linalg.eigvalsh(ekf.P).min() > -1e-9


def test_single_false_detections_do_not_create_landmarks():
    ekf, _, _ = run(2, false_hits=1)
    assert 'ghost' not in ekf.index


if __name__ == '__main__':
    for seed in range(5):
        ekf, e, _ = run(seed)
        print(seed, sorted(ekf.index), f'mean err {np.mean(e):.3f} max {max(e):.3f}')
