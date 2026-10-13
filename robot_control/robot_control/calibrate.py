import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

# Calibration helper: drive with a fixed (L, R) for a fixed duration, then report
# how far the IMU yaw turned. Measure the travelled distance with a tape measure.
#
#   ros2 run robot_control calibrate 0.3 0.3 3      # straight: measure distance
#   ros2 run robot_control calibrate -0.3 0.3 3     # spin: read the yaw change
#
# motor_control must be running in another terminal.

MAX_DURATION = 10.0  # seconds; safety limit
SETTLE_TIME = 0.5    # seconds to wait after stopping before reading final yaw


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.yaw = None
        self.total_yaw = 0.0  # unwrapped yaw change, so turns > 180 deg still add up
        self.gyro_yaw = 0.0   # integrated gyro z (assumes deg/s), for comparison
        self.last_stamp = None
        self.log = []         # raw samples: (t, yaw_deg, gz)

    def imu_callback(self, msg):
        yaw = yaw_from_quaternion(msg.orientation)
        if self.yaw is not None:
            self.total_yaw += wrap_angle(yaw - self.yaw)
        self.yaw = yaw
        t = time.time()
        if self.last_stamp is not None:
            self.gyro_yaw += math.radians(msg.angular_velocity.z) * (t - self.last_stamp)
        self.last_stamp = t
        self.log.append((t, math.degrees(yaw), msg.angular_velocity.z))

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def spin_for(self, duration, L=0.0, R=0.0):
        """Keep publishing (L, R) at 20Hz for `duration` seconds."""
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            self.send(L, R)
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Drive (L, R) for a fixed time.')
    parser.add_argument('L', type=float, help='left wheel command (-0.5 ~ 0.5)')
    parser.add_argument('R', type=float, help='right wheel command (-0.5 ~ 0.5)')
    parser.add_argument('duration', type=float, help=f'seconds (max {MAX_DURATION})')
    opts = parser.parse_args(remove_ros_args(args=None)[1:])
    duration = min(opts.duration, MAX_DURATION)

    node = CalibrationNode()
    try:
        # wait for IMU so we have a starting yaw
        start = time.time()
        while rclpy.ok() and node.yaw is None and time.time() - start < 3.0:
            node.spin_for(0.1)
        if node.yaw is None:
            print('No IMU data on /imu_data. Is motor_control running?')
            return 1

        print(f'Driving L={opts.L:.2f}, R={opts.R:.2f} for {duration:.1f}s ...')
        node.total_yaw = 0.0
        node.gyro_yaw = 0.0
        node.log = []
        node.spin_for(duration, opts.L, opts.R)
        node.spin_for(SETTLE_TIME)  # stop, let the robot settle

        turned = node.total_yaw
        print('========== Result ==========')
        print(f'Command      : L={opts.L:.2f}, R={opts.R:.2f}, {duration:.1f}s')
        print(f'Yaw change   : {turned:+.3f} rad  ({math.degrees(turned):+.1f} deg)')
        print(f'Turn rate    : {turned / duration:+.3f} rad/s '
              f'({math.degrees(turned) / duration:+.1f} deg/s)')
        print(f'Gyro z integ.: {node.gyro_yaw:+.3f} rad  '
              f'({math.degrees(node.gyro_yaw):+.1f} deg)')
        log_path = time.strftime('/tmp/calib_%H%M%S.csv')
        with open(log_path, 'w') as f:
            f.write('t,yaw_deg,gz\n')
            t0 = node.log[0][0] if node.log else 0.0
            for t, y, gz in node.log:
                f.write(f'{t - t0:.3f},{y:.2f},{gz:.3f}\n')
        print(f'Raw IMU log  : {log_path}')
        print('Now measure the distance travelled; speed = distance / '
              f'{duration:.1f} s')
    except KeyboardInterrupt:
        pass
    finally:
        node.spin_for(0.2)  # make sure the robot stops
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
