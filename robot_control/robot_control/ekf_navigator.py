import argparse
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args
from vision_msgs.msg import Detection2DArray

from robot_control.ekf import PoseEkf, wrap_angle
from robot_control.waypoint_navigator import (
    DRIVE_CMD, HEADING_KP, MAX_CORRECTION, MAX_TURN_TIME, SETTLE_TIME, TURN_CMD, TURN_LEAD,
    WaypointNavigator, load_waypoints)

# Closed-loop waypoint driving: the HW1 turn -> drive -> turn strategy, steered by
# an EKF pose from gyro, commanded speed and landmark sightings.
#
# A frame is about 0.3 s old when it arrives, so sightings are skipped during and
# just after a move: the bearing would be several degrees behind in a turn, and
# the robot would look further from the landmark than it is after a stop. While
# driving straight, the distance covered in that time is subtracted, and range
# noise is raised. The robot stops at each waypoint to get clean sightings, then
# drives on if the goal is still ahead of it. It never spins to fix a small error.
# With no landmark in view the EKF keeps going on gyro and speed alone, and its
# covariance grows until the next sighting.
#
#   ros2 launch landmark_detector camera_detector_launch.py     # terminal 1
#   ros2 run robot_control motor_control                        # terminal 2
#   ros2 run robot_control ekf_navigator waypoints/hw2.txt      # terminal 3
#   ros2 run robot_control ekf_navigator waypoints/hw2.txt --look-only   # no driving
#
# Each run writes ~/hw2_runs/<time>_pose.csv and _obs.csv.

DRIVE_SPEED = 0.31        # m/s at DRIVE_CMD, from the sighted range rate over 3 runs
DRIVE_TRIM = 0.04         # subtracted from right, added to left: the robot pulls left when driving
CAMERA_X = 0.125          # m, camera ahead of the turning center: half of the 25 cm body
CAMERA_YAW = math.radians(7.0)  # camera points this far left of the car, mean over 6 runs
RANGE_OFFSET = 0.03       # m; camera ranges read this much long from 0.3 to 2 m, tape-measured
SIGMA_RANGE = 0.05        # fraction of the range
SIGMA_RANGE_MOVING = 0.08  # m, lower limit while driving: the frame is 5 to 8 cm old
SIGMA_BEARING = math.radians(2.0)
TURNING_RATE = math.radians(15.0)  # rad/s; above this the robot counts as turning
MOTION_BLANK = 0.5        # seconds after a turn or stop before sightings count again
IMAGE_LATENCY = 0.3       # seconds; fitted to the overshoot on the first test drive
# Frames seconds apart repeat the same box error, so fusing all of them would make
# the EKF far too confident. Each landmark counts at most this often.
SIGHTING_PERIOD = 0.5     # seconds
STILL_RATE = math.radians(5.0)  # rad/s; with the wheels off, a slower gyro rate is bias
# The IMU sometimes repeats one gyro value for seconds while driving. Integrating
# it makes the EKF see a turn that is not happening, and the controller then steers
# the real robot off course. Repeated samples are replaced by the rate expected
# from the wheel commands, and heading noise goes up so sightings can correct it.
FROZEN_SAMPLES = 5        # identical gyro samples in a row
STEER_GAIN = 0.43         # rad/s per unit of (right - left) while driving, from logs
PIVOT_GAIN = 2.33         # rad/s per unit of (right - left) turning in place, from logs
FROZEN_NOISE = 0.2        # rad per sqrt(s)

# Stop the run rather than drive on a pose that cannot be trusted
MAX_REJECTIONS = 3        # sightings in a row that fail the outlier gate
MAX_SIGMA = 0.25          # m, position uncertainty
MAX_TURN_ERROR = math.radians(45.0)  # heading error left after a turn
MAX_RUN_TIME = 120.0      # seconds

POSITION_TOLERANCE = 0.05  # m
TURN_FIRST_ABOVE = math.radians(15.0)  # smaller heading errors are steered out while driving
AHEAD_COS = 0.9           # a landmark this aligned with the leg, past the goal, can stop the robot
RETRY_WITHIN = math.radians(60.0)  # after stopping short, drive on if the goal is this close to ahead
HOLD_HEADING_WITHIN = 0.4  # m; closer than this, bearing to the goal is too noisy to steer by
MAX_ATTEMPTS = 3          # drives per waypoint
LOOK_TIME = 1.5           # seconds stopped at a waypoint to collect sightings
DRIVE_TIME_MARGIN = 2.5   # safety limit on one drive, as a multiple of the expected time
LOG_PERIOD = 0.1          # seconds between pose log rows


class Abort(Exception):
    pass


def load_map():
    path = os.path.join(get_package_share_directory('landmark_detector'), 'config', 'landmarks.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)['landmarks']
    return {k: v for k, v in data.items() if 'position' in v}


def to_range_bearing(forward, left, radius):
    """Convert a camera sighting of a landmark's front face to its center, seen from the robot center."""
    r = math.hypot(forward, left)
    scale = (r - RANGE_OFFSET + radius) / r
    c, s = math.cos(CAMERA_YAW), math.sin(CAMERA_YAW)
    x = (forward * c - left * s) * scale + CAMERA_X
    y = (forward * s + left * c) * scale
    return math.hypot(x, y), math.atan2(y, x)


class EkfNavigator(WaypointNavigator):
    def __init__(self, stem):
        super().__init__()
        self.landmarks = load_map()
        self.ekf = PoseEkf()
        self.speed = 0.0
        self.turning = False
        self.last_turning = 0.0
        self.last_driving = 0.0
        self.last_sighting = {}
        self.localizing = False
        self.stop_landmark = None
        self.stop_range = 0.0
        self.arrived = False
        self.rejections = 0  # in a row
        self.limits_on = False
        self.rate = 0.0
        self.command = (0.0, 0.0)
        self.frozen = 0
        self.last_gyro = None
        self.frozen_time = 0.0
        self.used = self.rejected = 0
        self.started = time.time()
        self.pose_log = open(stem + '_pose.csv', 'w')
        self.pose_log.write('t,x,y,theta,sx,sy,stheta,used,rejected,rate,left,right\n')
        self.obs_log = open(stem + '_obs.csv', 'w')
        self.obs_log.write('t,name,range,bearing,expected_range,expected_bearing,used\n')
        self.create_subscription(Detection2DArray, '/landmarks/observations', self.on_observations, 10)
        self.create_timer(LOG_PERIOD, self.log_pose)
        self.get_logger().info(f'Map: { {k: v["position"] for k, v in self.landmarks.items()} }')

    def start_at(self, pose):
        self.measure_gyro_bias()
        self.ekf = PoseEkf(*pose)
        self.heading = pose[2]
        self.localizing = True

    def integrate(self, rate, dt):
        self.frozen = self.frozen + 1 if rate == self.last_gyro else 0
        self.last_gyro = rate
        if self.frozen >= FROZEN_SAMPLES:
            L, R = self.command
            rate = (PIVOT_GAIN if L * R < 0.0 else STEER_GAIN) * (R - L)
            self.ekf.P[2, 2] += FROZEN_NOISE ** 2 * dt
            self.frozen_time += dt
        self.rate = rate
        if abs(rate) > TURNING_RATE:
            self.last_turning = time.time()
        if not self.turning and abs(self.speed) < 0.01 and abs(rate) < STILL_RATE:
            rate = 0.0
        self.ekf.predict(self.speed, rate, dt)
        self.heading = float(self.ekf.state[2])

    def send(self, L, R):
        # Equal and opposite wheels turn in place, so the mean is the forward speed
        self.speed = DRIVE_SPEED * (L + R) / (2.0 * DRIVE_CMD)
        self.turning = L != R
        self.command = (L, R)
        if abs(self.speed) > 0.01:
            self.last_driving = time.time()
        super().send(L, R)

    def on_observations(self, msg):
        now = time.time()
        moving = abs(self.speed) > 0.01
        if not self.localizing or now - self.last_turning < MOTION_BLANK:
            return
        if not moving and now - self.last_driving < MOTION_BLANK:
            return
        for det in msg.detections:
            name = det.results[0].hypothesis.class_id
            lm = self.landmarks.get(name)
            if lm is None or now - self.last_sighting.get(name, 0.0) < SIGHTING_PERIOD:
                continue
            self.last_sighting[name] = now
            p = det.results[0].pose.pose.position
            rng, bearing = to_range_bearing(p.x - self.speed * IMAGE_LATENCY, p.y, lm['radius'])
            if moving and name == self.stop_landmark and rng <= self.stop_range:
                self.arrived = True
            sigma = SIGMA_RANGE * rng
            if moving:
                sigma = max(sigma, SIGMA_RANGE_MOVING)
            x, y, theta = self.ekf.state
            dx, dy = lm['position'][0] - x, lm['position'][1] - y
            expected = (math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - theta))
            ok = self.ekf.update(lm['position'], rng, bearing, sigma, SIGMA_BEARING)
            self.used += ok
            self.rejected += not ok
            self.check_lost(name, ok)
            self.obs_log.write(f'{now - self.started:.2f},{name},{rng:.3f},{bearing:.4f},'
                               f'{expected[0]:.3f},{expected[1]:.4f},{int(ok)}\n')
        self.heading = float(self.ekf.state[2])

    def check_lost(self, name, ok):
        self.rejections = 0 if ok else self.rejections + 1
        if self.rejections >= MAX_REJECTIONS:
            raise Abort(f'{self.rejections} sightings in a row rejected, last {name}')

    def turn_once(self, target):
        """One continuous turn, no correction pulses, so the robot never rocks left and right."""
        s = 1.0 if wrap_angle(target - self.ekf.state[2]) > 0 else -1.0
        end = time.time() + MAX_TURN_TIME
        while rclpy.ok() and time.time() < end and \
                s * wrap_angle(target - self.ekf.state[2]) > TURN_LEAD:
            self.send(-s * TURN_CMD, s * TURN_CMD)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.drive_for(SETTLE_TIME)
        self.look()
        err = wrap_angle(target - self.ekf.state[2])
        print(f'  turned to {math.degrees(self.ekf.state[2]):+7.1f} deg (error {math.degrees(err):+.1f})')
        if abs(err) > MAX_TURN_ERROR:
            raise Abort(f'turn failed, {math.degrees(err):+.1f} deg off target')

    def log_pose(self):
        x, y, theta = self.ekf.state
        P = self.ekf.P
        elapsed = time.time() - self.started
        self.pose_log.write(f'{elapsed:.2f},{x:.4f},{y:.4f},{theta:.4f},'
                            f'{math.sqrt(P[0, 0]):.4f},{math.sqrt(P[1, 1]):.4f},'
                            f'{math.sqrt(P[2, 2]):.4f},{self.used},{self.rejected},'
                            f'{self.rate:.4f},{self.command[0]:.3f},{self.command[1]:.3f}\n')
        if self.limits_on:
            if elapsed > MAX_RUN_TIME:
                raise Abort(f'run time over {MAX_RUN_TIME:.0f} s')
            if math.sqrt(P[0, 0] + P[1, 1]) > MAX_SIGMA:
                raise Abort(f'position uncertainty over {MAX_SIGMA} m')

    def distance_to(self, gx, gy):
        return math.hypot(gx - self.ekf.state[0], gy - self.ekf.state[1])

    def set_stop_landmark(self, gx, gy, ux, uy, leg):
        """Find a landmark straight past the goal. Its seen range stops the robot, whatever the EKF says."""
        self.stop_landmark = None
        self.arrived = False
        for name, lm in self.landmarks.items():
            dx, dy = lm['position'][0] - gx, lm['position'][1] - gy
            d = math.hypot(dx, dy)
            if d > 0.0 and (dx * ux + dy * uy) / (d * leg) > AHEAD_COS:
                self.stop_landmark, self.stop_range = name, d

    def drive_to(self, gx, gy):
        """Drive to (gx, gy), steering on the EKF pose. Stops if it passes the goal."""
        sx, sy = self.ekf.state[:2]
        ux, uy = gx - sx, gy - sy
        leg = math.hypot(ux, uy)
        self.set_stop_landmark(gx, gy, ux, uy, leg)
        hold = math.atan2(uy, ux)
        end = time.time() + DRIVE_TIME_MARGIN * leg / DRIVE_SPEED
        while rclpy.ok() and time.time() < end:
            x, y = self.ekf.state[:2]
            left = ((gx - x) * ux + (gy - y) * uy) / leg  # distance still to go along the leg
            if left <= 0.0 or self.arrived:
                break
            if self.distance_to(gx, gy) > HOLD_HEADING_WITHIN:
                hold = math.atan2(gy - y, gx - x)
            err = wrap_angle(hold - self.ekf.state[2])
            corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, HEADING_KP * err))
            self.send(DRIVE_CMD + DRIVE_TRIM - corr, DRIVE_CMD - DRIVE_TRIM + corr)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop_landmark = None
        self.drive_for(SETTLE_TIME)

    def look(self):
        self.drive_for(LOOK_TIME)

    def report(self, label):
        x, y, theta = self.ekf.state
        P = self.ekf.P
        print(f'  {label}: ({x:+.3f}, {y:+.3f}, {math.degrees(theta):+.1f} deg), '
              f'sigma ({math.sqrt(P[0, 0]):.3f}, {math.sqrt(P[1, 1]):.3f} m, '
              f'{math.degrees(math.sqrt(P[2, 2])):.1f} deg), '
              f'sightings used {self.used}, rejected {self.rejected}, '
              f'gyro frozen {self.frozen_time:.1f} s')

    def look_only(self, waypoints):
        """Stay still and print the estimate, to check the map and the signs."""
        self.start_at(waypoints[0])
        while rclpy.ok():
            self.look()
            self.report('still')

    def run(self, waypoints):
        self.start_at(waypoints[0])
        self.limits_on = True
        self.look()
        self.report('start')
        for i, (gx, gy, gtheta) in enumerate(waypoints[1:], start=1):
            print(f'[{i}/{len(waypoints) - 1}] -> ({gx:.2f}, {gy:.2f}, {math.degrees(gtheta):+.1f} deg)')
            for attempt in range(MAX_ATTEMPTS):
                if self.distance_to(gx, gy) < POSITION_TOLERANCE:
                    break
                x, y, theta = self.ekf.state
                direction = math.atan2(gy - y, gx - x)
                off = abs(wrap_angle(direction - theta))
                if attempt > 0 and off > RETRY_WITHIN:
                    break  # the goal is off to the side; reaching it would take another spin
                if attempt == 0 and off > TURN_FIRST_ABOVE:
                    self.turn_once(direction)
                self.drive_to(gx, gy)
                self.look()
            if abs(wrap_angle(gtheta - self.ekf.state[2])) > TURN_FIRST_ABOVE:
                self.turn_once(gtheta)
            self.report(f'waypoint {i}')
        print('Done.')


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Closed-loop waypoint driving with landmarks.')
    parser.add_argument('waypoints', help='file with one "x,y,theta" per line')
    parser.add_argument('--look-only', action='store_true', help='estimate the pose without driving')
    opts = parser.parse_args(remove_ros_args(args=None)[1:])
    waypoints = load_waypoints(opts.waypoints)

    folder = os.path.expanduser('~/hw2_runs')
    os.makedirs(folder, exist_ok=True)
    stem = os.path.join(folder, time.strftime('%m%d_%H%M%S'))
    node = EkfNavigator(stem)
    try:
        if opts.look_only:
            node.look_only(waypoints)
        else:
            node.run(waypoints)
    except KeyboardInterrupt:
        pass
    except Abort as e:
        print(f'ABORT: {e}')
    finally:
        node.drive_for(0.2)  # make sure the robot stops
        node.pose_log.close()
        node.obs_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'Logs: {stem}_pose.csv, {stem}_obs.csv')
    return 0


if __name__ == '__main__':
    main()
