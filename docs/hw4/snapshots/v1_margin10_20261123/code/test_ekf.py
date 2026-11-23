import math
import random

from robot_control.ekf import GATE, PoseEkf, rank_matches, wrap_angle

LANDMARKS = {'tp': (1.3, 0.0), 'dog': (1.3, 1.0), 'bottle': (1.3, 2.0), 'box': (-0.2, -0.4)}
FOV = math.radians(40.0)
SPEED = 0.377


def sight(rng_gen, x, y, theta):
    for name, (lx, ly) in LANDMARKS.items():
        r = math.hypot(lx - x, ly - y)
        b = wrap_angle(math.atan2(ly - y, lx - x) - theta)
        if abs(b) < FOV and 0.3 < r < 2.4:
            yield (lx, ly), r * (1 + rng_gen.gauss(0, 0.05)), b + rng_gen.gauss(0, math.radians(2))


def simulate(seed, speed_scale, use_landmarks=True):
    """Drive the HW2 route with a robot slower than commanded. Returns (errors at waypoints, rejected)."""
    rng_gen = random.Random(seed)
    ekf = PoseEkf()
    x, y, theta = 0.0, 0.0, 0.0
    dt = 0.05
    rejected = 0
    errors = []
    for gx, gy in [(1.0, 0.0), (1.0, 2.0), (0.0, 0.0)]:
        for _ in range(400):  # turn toward the goal
            err = wrap_angle(math.atan2(gy - ekf.state[1], gx - ekf.state[0]) - ekf.state[2])
            if abs(err) < math.radians(3):
                break
            w = math.copysign(1.5, err)
            theta = wrap_angle(theta + w * (1 + rng_gen.gauss(0, 0.02)) * dt)
            ekf.predict(0.0, w, dt)
        for _ in range(4):  # stopped, look
            if use_landmarks:
                for lm, r, b in sight(rng_gen, x, y, theta):
                    rejected += not ekf.update(lm, r, b, 0.05 * r, math.radians(2))
        ux, uy = gx - ekf.state[0], gy - ekf.state[1]
        for _ in range(2000):
            if (gx - ekf.state[0]) * ux + (gy - ekf.state[1]) * uy <= 0:
                break
            hold = math.atan2(gy - ekf.state[1], gx - ekf.state[0])
            w = max(-0.5, min(0.5, 2.0 * wrap_angle(hold - ekf.state[2])))
            true_v = SPEED * speed_scale
            x += true_v * dt * math.cos(theta)
            y += true_v * dt * math.sin(theta)
            theta = wrap_angle(theta + w * dt)
            ekf.predict(SPEED, w, dt)
            if use_landmarks and rng_gen.random() < 0.25:
                for lm, r, b in sight(rng_gen, x, y, theta):
                    rejected += not ekf.update(lm, r, b, max(0.05 * r, 0.08), math.radians(2))
        for _ in range(8):
            if use_landmarks:
                for lm, r, b in sight(rng_gen, x, y, theta):
                    rejected += not ekf.update(lm, r, b, 0.05 * r, math.radians(2))
        errors.append(math.hypot(x - gx, y - gy))
    return errors, rejected


def test_stopped_robot_does_not_move():
    ekf = PoseEkf()
    ekf.predict(0.0, 0.0, 1.0)
    assert all(abs(v) < 1e-9 for v in ekf.state)


def test_outlier_is_rejected():
    ekf = PoseEkf()
    assert not ekf.update((1.3, 0.0), 0.5, 0.0, 0.065, math.radians(2))


def test_sighting_pulls_pose_toward_truth():
    ekf = PoseEkf(sigma_xy=0.3)
    ekf.state[0] = 0.2  # the truth is 0.0
    assert ekf.update((1.3, 0.0), 1.3, 0.0, 0.03, math.radians(2))
    assert abs(ekf.state[0]) < 0.05


def test_angle_wraps_past_pi():
    ekf = PoseEkf(theta=3.1)
    ekf.predict(0.0, 1.0, 0.2)
    assert -math.pi <= ekf.state[2] <= math.pi


def test_landmarks_beat_a_wrong_speed():
    trials = [(simulate(s, 0.88), simulate(s, 0.88, use_landmarks=False)) for s in range(30)]
    with_lm = sum(sum(t[0][0]) for t in trials) / (3 * len(trials))
    without = sum(sum(t[1][0]) for t in trials) / (3 * len(trials))
    assert with_lm < without / 2


def sighting_of(ekf_truth, landmark):
    x, y, theta = ekf_truth
    r = math.hypot(landmark[0] - x, landmark[1] - y)
    return r, wrap_angle(math.atan2(landmark[1] - y, landmark[0] - x) - theta)


def test_duplicate_tag_id_matches_the_right_instance():
    # HW4: tag_0 at the arena edge and a second tag_0 on the center box
    edge, box = (0.48, -0.08), (1.77, 1.52)
    ekf = PoseEkf(0.75, 0.75, math.radians(45))
    for seen in (edge, box):
        r, b = sighting_of((0.75, 0.75, math.radians(45)), seen)
        ranked = rank_matches(ekf, [(name, lm, r, b, 0.03 * r, math.radians(1.5))
                                    for name, lm in (('edge', edge), ('box', box))])
        assert ranked[0][1][1] == seen
        assert ranked[0][0] <= GATE < ranked[1][0]


def test_two_instances_that_both_fit_are_reported():
    # Two tag_0 at the same range and bearing from a very uncertain pose: both pass the gate
    ekf = PoseEkf(0.0, 0.0, 0.0, sigma_xy=1.0, sigma_theta=1.0)
    a, b = (1.0, 0.1), (1.0, -0.1)
    r, bearing = sighting_of((0.0, 0.0, 0.0), a)
    ranked = rank_matches(ekf, [(n, lm, r, bearing, 0.03, math.radians(1.5)) for n, lm in (('a', a), ('b', b))])
    assert ranked[1][0] <= GATE
