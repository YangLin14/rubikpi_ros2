import math

import numpy as np

from robot_control.ekf import wrap_angle

# EKF-SLAM with range-bearing landmarks and known data association.
#
# State  x = [xr, yr, theta, l1x, l1y, l2x, l2y, ...]   (3 + 2N, N grows at run time)
# Control u = (v, w): commanded forward speed and gyro yaw rate
# Measurement z = (range, bearing) of one landmark, from the robot center
#
# Every landmark class appears once in the field, so the detector's class name is
# the landmark ID. The filter starts with no landmarks: a class seen for the first
# time is held as a candidate, and enters the state only when MIN_CANDIDATE of its
# last CANDIDATE_WINDOW sightings agree within CANDIDATE_GATE, so scattered false
# detections do not create a landmark. A landmark out of view is simply not updated: its estimate and
# covariance stay as they are, while the robot's uncertainty grows. When it is seen
# again, the usual update both corrects it and pulls the robot back (loop closure),
# through the robot-landmark cross covariance.

MIN_CANDIDATE = 3        # sightings before a new landmark enters the state
CANDIDATE_WINDOW = 5     # only the most recent candidate sightings count
CANDIDATE_GATE = 0.35    # m; candidate sightings must agree this well with their mean


class SlamEkf:
    def __init__(self, x=0.0, y=0.0, theta=0.0, sigma_xy=0.0, sigma_theta=0.0,
                 k_distance=0.10, k_turn=0.08, sigma_drift=0.005):
        # The start pose defines the map frame, so it is known exactly: P0 = 0
        self.state = np.array([x, y, theta], dtype=float)
        self.P = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_theta ** 2])
        self.k_distance = k_distance    # m of error per sqrt(m) driven
        self.k_turn = k_turn            # rad of error per sqrt(rad) turned
        self.sigma_drift = sigma_drift  # rad per sqrt(s), leftover gyro bias
        self.index = {}                 # landmark name -> index of its x in the state
        self.candidates = {}            # name -> list of (x, y) world sightings

    # ---- prediction -------------------------------------------------------
    def predict(self, v, w, dt):
        theta = self.state[2]
        mid = theta + 0.5 * w * dt
        c, s = math.cos(mid), math.sin(mid)
        d = v * dt
        self.state[:3] += (d * c, d * s, w * dt)
        self.state[2] = wrap_angle(self.state[2])

        # Landmarks do not move: F is identity except the robot block
        Fr = np.array([[1.0, 0.0, -d * s], [0.0, 1.0, d * c], [0.0, 0.0, 1.0]])
        G = np.array([[c, 0.0], [s, 0.0], [0.0, 1.0]])
        Q = np.diag([self.k_distance ** 2 * abs(d),
                     self.k_turn ** 2 * abs(w * dt) + self.sigma_drift ** 2 * dt])
        P = self.P
        P[:3, :3] = Fr @ P[:3, :3] @ Fr.T + G @ Q @ G.T
        P[:3, 3:] = Fr @ P[:3, 3:]
        P[3:, :3] = P[:3, 3:].T

    # ---- measurement ------------------------------------------------------
    def expected(self, name):
        i = self.index[name]
        dx, dy = self.state[i] - self.state[0], self.state[i + 1] - self.state[1]
        return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - self.state[2])

    def landmark(self, name):
        i = self.index[name]
        return self.state[i:i + 2].copy(), self.P[i:i + 2, i:i + 2].copy()

    def observe(self, name, rng, bearing, sigma_range, sigma_bearing, gate=13.8):
        """Fuse one sighting. Returns 'new', 'candidate', 'used' or 'rejected'."""
        if name in self.index:
            return 'used' if self.update(name, rng, bearing, sigma_range, sigma_bearing, gate) else 'rejected'
        x, y, theta = self.state[:3]
        p = (x + rng * math.cos(theta + bearing), y + rng * math.sin(theta + bearing))
        seen = self.candidates.setdefault(name, [])
        seen.append(p)
        del seen[:-CANDIDATE_WINDOW]
        center = np.median(seen, axis=0)
        agree = [q for q in seen if math.hypot(q[0] - center[0], q[1] - center[1]) < CANDIDATE_GATE]
        # The newest sighting must be one of them, since it initializes the landmark
        if len(agree) < MIN_CANDIDATE or p not in agree:
            return 'candidate'
        del self.candidates[name]
        self.add(name, rng, bearing, sigma_range, sigma_bearing)
        return 'new'

    def add(self, name, rng, bearing, sigma_range, sigma_bearing):
        """Augment the state with a new landmark, initialized from this sighting."""
        x, y, theta = self.state[:3]
        a = theta + bearing
        c, s = math.cos(a), math.sin(a)
        lm = np.array([x + rng * c, y + rng * s])
        # Jacobians of the landmark position wrt the robot pose and the measurement
        Gr = np.array([[1.0, 0.0, -rng * s], [0.0, 1.0, rng * c]])
        Gz = np.array([[c, -rng * s], [s, rng * c]])
        R = np.diag([sigma_range ** 2, sigma_bearing ** 2])
        n = len(self.state)
        P = np.zeros((n + 2, n + 2))
        P[:n, :n] = self.P
        P[n:, :n] = Gr @ self.P[:3, :n]
        P[:n, n:] = P[n:, :n].T
        P[n:, n:] = Gr @ self.P[:3, :3] @ Gr.T + Gz @ R @ Gz.T
        self.P = P
        self.state = np.concatenate([self.state, lm])
        self.index[name] = n

    def update(self, name, rng, bearing, sigma_range, sigma_bearing, gate=13.8):
        """Standard EKF update; gate is the chi-square limit on 2 degrees of freedom."""
        i = self.index[name]
        dx, dy = self.state[i] - self.state[0], self.state[i + 1] - self.state[1]
        q = dx * dx + dy * dy
        r = math.sqrt(q)
        if r < 1e-6:
            return False
        n = len(self.state)
        H = np.zeros((2, n))
        H[:, :3] = [[-dx / r, -dy / r, 0.0], [dy / q, -dx / q, -1.0]]
        H[:, i:i + 2] = [[dx / r, dy / r], [-dy / q, dx / q]]
        y = np.array([rng - r, wrap_angle(bearing - (math.atan2(dy, dx) - self.state[2]))])
        R = np.diag([sigma_range ** 2, sigma_bearing ** 2])
        S = H @ self.P @ H.T + R
        if float(y @ np.linalg.solve(S, y)) > gate:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.state[2] = wrap_angle(self.state[2])
        A = np.eye(n) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        return True
