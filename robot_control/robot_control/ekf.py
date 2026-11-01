import math

import numpy as np

# EKF localization on a known landmark map. The state is the pose (x, y, theta).
# Prediction uses the commanded speed and the gyro; each landmark sighting is a
# range and bearing from the robot center. Keeping the landmarks fixed means their
# positions are not estimated, only the pose.
#
# Motion noise grows with distance and turned angle rather than time, because a
# stopped robot stays put and speed errors add up along the path.


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseEkf:
    def __init__(self, x=0.0, y=0.0, theta=0.0, sigma_xy=0.03, sigma_theta=0.05,
                 k_distance=0.10, k_turn=0.08, sigma_drift=0.005):
        self.state = np.array([x, y, theta], dtype=float)
        self.P = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_theta ** 2])
        self.k_distance = k_distance    # m of error per sqrt(m) driven
        self.k_turn = k_turn            # rad of error per sqrt(rad) turned
        self.sigma_drift = sigma_drift  # rad per sqrt(s), leftover gyro bias

    def predict(self, v, w, dt):
        theta = self.state[2]
        mid = theta + 0.5 * w * dt
        c, s = math.cos(mid), math.sin(mid)
        d = v * dt
        self.state += (d * c, d * s, w * dt)
        self.state[2] = wrap_angle(self.state[2])

        F = np.array([[1.0, 0.0, -d * s], [0.0, 1.0, d * c], [0.0, 0.0, 1.0]])
        G = np.array([[c, 0.0], [s, 0.0], [0.0, 1.0]])
        Q = np.diag([self.k_distance ** 2 * abs(d),
                     self.k_turn ** 2 * abs(w * dt) + self.sigma_drift ** 2 * dt])
        self.P = F @ self.P @ F.T + G @ Q @ G.T

    def update(self, landmark, rng, bearing, sigma_range, sigma_bearing, gate=13.8):
        """Fuse one sighting. Returns False if it fails the outlier gate.

        gate is the chi-square limit on 2 degrees of freedom: 13.8 rejects a true
        sighting about 1 time in 1000.
        """
        dx, dy = landmark[0] - self.state[0], landmark[1] - self.state[1]
        q = dx * dx + dy * dy
        r = math.sqrt(q)
        if r < 1e-6:
            return False
        H = np.array([[-dx / r, -dy / r, 0.0], [dy / q, -dx / q, -1.0]])
        y = np.array([rng - r, wrap_angle(bearing - (math.atan2(dy, dx) - self.state[2]))])
        R = np.diag([sigma_range ** 2, sigma_bearing ** 2])
        S = H @ self.P @ H.T + R
        if float(y @ np.linalg.solve(S, y)) > gate:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.state[2] = wrap_angle(self.state[2])
        A = np.eye(3) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        return True
