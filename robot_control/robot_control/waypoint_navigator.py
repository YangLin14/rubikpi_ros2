import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

# Drive through a list of (x, y, theta) waypoints with a turn -> drive -> turn
# strategy. Heading comes from integrating the IMU gyro z rate (the fused IMU yaw
# drifts badly while turning); distance is open-loop, from calibrated speed.
# IMU messages may arrive in bursts, so a single arrival-time gap can't be used as
# dt; each sample is integrated with a running average of the sample period.
#
#   ros2 run robot_control waypoint_navigator                  # default waypoints
#   ros2 run robot_control waypoint_navigator waypoints.txt    # one "x,y,theta" per line
#
# motor_control must be running in another terminal.

DEFAULT_WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 1.57),
    (2.0, 1.0, 0.0),
    (2.0, 2.0, -1.57),
    (1.0, 1.0, -0.78),
    (0.0, 0.0, 0.0),
]

# ---- Calibration (from calibrate.py runs) ----
DRIVE_CMD = 0.3          # wheel command when driving straight
DRIVE_SPEED = 0.377      # m/s at DRIVE_CMD (1 m in 2.65 s)
HEADING_KP = 0.3         # wheel command per rad of heading error while driving
MAX_CORRECTION = 0.1     # cap on the heading correction while driving

TURN_CMD = 0.5           # in-place turn command; 0.45 often fails to start from rest
TURN_LEAD = math.radians(15)       # stop the continuous turn this early (latency)
TURN_TOLERANCE = math.radians(5)   # good enough heading error
PULSE_TIME = 0.12         # seconds per small correction pulse
MAX_TURN_STEPS = 8       # continuous turn + correction pulses before giving up
MAX_TURN_TIME = 5.0      # seconds; safety limit for one continuous turn

GYRO_SCALE = 1.0         # multiply gyro rate by this if turns are consistently off
BIAS_TIME = 2.0          # seconds standing still to measure gyro bias
SETTLE_TIME = 0.3        # seconds to stop between motions
WAYPOINT_PAUSE = 1.0     # seconds to stop at each waypoint


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_waypoints(path):
    waypoints = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                x, y, theta = (float(v) for v in line.split(','))
                waypoints.append((x, y, theta))
    return waypoints


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 50)
        self.heading = 0.0      # rad, integrated from gyro z
        self.gyro_bias = 0.0    # deg/s
        self.sample_dt = None   # seconds per IMU sample (running average)
        self.last_stamp = None
        self.bias_samples = None  # list while measuring bias, else None

    def imu_callback(self, msg):
        gz = msg.angular_velocity.z  # deg/s, counter-clockwise positive
        t = time.time()
        if self.last_stamp is not None and self.sample_dt is not None:
            self.sample_dt = 0.98 * self.sample_dt + 0.02 * (t - self.last_stamp)
        self.last_stamp = t
        if self.bias_samples is not None:
            self.bias_samples.append(gz)
        elif self.sample_dt is not None:
            rate = math.radians(gz - self.gyro_bias) * GYRO_SCALE
            self.heading = wrap_angle(self.heading + rate * self.sample_dt)

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def drive_for(self, duration, L=0.0, R=0.0):
        """Keep publishing (L, R) for `duration` seconds."""
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            self.send(L, R)
            rclpy.spin_once(self, timeout_sec=0.05)

    def measure_gyro_bias(self):
        print(f'Measuring gyro bias, keep the robot still for {BIAS_TIME:.0f}s ...')
        self.bias_samples = []
        self.drive_for(BIAS_TIME)
        if not self.bias_samples:
            raise RuntimeError('No IMU data on /imu_data. Is motor_control running?')
        self.gyro_bias = sum(self.bias_samples) / len(self.bias_samples)
        self.sample_dt = BIAS_TIME / len(self.bias_samples)
        print(f'Gyro bias: {self.gyro_bias:+.2f} deg/s ({len(self.bias_samples)} samples, '
              f'{1.0 / self.sample_dt:.1f} Hz)')
        self.bias_samples = None

    def turn_to(self, target):
        """Turn in place until heading is within TURN_TOLERANCE of target."""
        for _ in range(MAX_TURN_STEPS):
            err = wrap_angle(target - self.heading)
            if abs(err) < TURN_TOLERANCE:
                break
            s = 1.0 if err > 0 else -1.0  # +1: turn left (counter-clockwise)
            if abs(err) > TURN_LEAD:
                # continuous turn until we are within TURN_LEAD of the target
                end = time.time() + MAX_TURN_TIME
                while rclpy.ok() and time.time() < end and \
                        s * wrap_angle(target - self.heading) > TURN_LEAD:
                    self.send(-s * TURN_CMD, s * TURN_CMD)
                    rclpy.spin_once(self, timeout_sec=0.02)
            else:
                # small error: nudge with a short pulse
                self.drive_for(PULSE_TIME, -s * TURN_CMD, s * TURN_CMD)
            self.drive_for(SETTLE_TIME)
        err = wrap_angle(target - self.heading)
        print(f'  turned to {math.degrees(self.heading):+7.1f} deg '
              f'(target {math.degrees(target):+7.1f}, error {math.degrees(err):+.1f})')

    def drive_straight(self, distance, target_heading):
        """Drive forward `distance` m (open-loop time), holding target_heading."""
        duration = distance / DRIVE_SPEED
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            err = wrap_angle(target_heading - self.heading)
            corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, HEADING_KP * err))
            self.send(DRIVE_CMD - corr, DRIVE_CMD + corr)  # err > 0: speed up right wheel
            rclpy.spin_once(self, timeout_sec=0.02)
        self.drive_for(SETTLE_TIME)
        print(f'  drove {distance:.2f} m in {duration:.2f} s')

    def run(self, waypoints):
        self.measure_gyro_bias()
        x, y, theta = waypoints[0]
        self.heading = theta  # robot starts aligned with the first waypoint
        print(f'Start at ({x:.2f}, {y:.2f}, {math.degrees(theta):+.1f} deg)')

        for i, (gx, gy, gtheta) in enumerate(waypoints[1:], start=1):
            print(f'[{i}/{len(waypoints) - 1}] -> ({gx:.2f}, {gy:.2f}, '
                  f'{math.degrees(gtheta):+.1f} deg)')
            dx, dy = gx - x, gy - y
            distance = math.hypot(dx, dy)
            if distance > 0.01:
                direction = math.atan2(dy, dx)
                self.turn_to(direction)
                self.drive_straight(distance, direction)
            self.turn_to(gtheta)
            x, y = gx, gy
            self.drive_for(WAYPOINT_PAUSE)
        print('Done.')


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Drive through (x, y, theta) waypoints.')
    parser.add_argument('waypoints', nargs='?', help='file with one "x,y,theta" per line')
    opts = parser.parse_args(remove_ros_args(args=None)[1:])
    waypoints = load_waypoints(opts.waypoints) if opts.waypoints else DEFAULT_WAYPOINTS

    node = WaypointNavigator()
    try:
        node.run(waypoints)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive_for(0.2)  # make sure the robot stops
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
