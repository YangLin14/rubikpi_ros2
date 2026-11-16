import argparse
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args

from robot_control import ekf_navigator as nav
from robot_control.ekf import wrap_angle
from robot_control.ekf_navigator import Abort, EkfNavigator
from robot_control.slam import SlamEkf
from robot_control.waypoint_navigator import load_waypoints

# HW3: EKF-SLAM while driving a square or an octagon, several loops.
#
# Same driving as HW2 (turn, drive, stop and look at each corner), but the filter
# is SlamEkf: the landmark map is not given, it is built during the run. The
# start pose is the origin of the map frame. Landmarks are the four HW2 objects
# (YOLO, range from box height) and AprilTags tag_0..tag_3 (range from solvePnP).
#
#   ros2 launch landmark_detector camera_detector_launch.py           # terminal 1
#   ros2 run robot_control motor_control                              # terminal 2
#   ros2 run robot_control slam_navigator waypoints/square.txt --loops 2
#   ros2 run robot_control slam_navigator waypoints/octagon.txt --loops 2
#   ros2 run robot_control slam_navigator waypoints/square.txt --look-only
#
# Each run writes ~/hw3_runs/<time>_pose.csv, _obs.csv and _map.csv. The map file
# has one row per landmark at the end of each loop, with its 2x2 covariance.

OBJECT_SIGMA_RANGE = 0.05   # fraction of range, from box height
TAG_SIGMA_RANGE = 0.03      # fraction of range, from solvePnP
TAG_SIGMA_MIN = 0.02        # m
OBJECT_SIGMA_BEARING = math.radians(2.0)
TAG_SIGMA_BEARING = math.radians(1.5)
LOOP_TIME = 100.0           # s allowed per loop
MAX_SIGMA = 0.35            # m, robot position uncertainty before abort
MAX_REJECTIONS = 5
# YOLO sometimes boxes an AprilTag's black square as cereal_box. An object box whose
# center falls inside a recent tag box (grown by TAG_MASK_GROW on each side) is dropped.
TAG_MASK_TIME = 0.6         # s
TAG_MASK_GROW = 0.5         # fraction of the tag box size


def load_objects():
    path = os.path.join(get_package_share_directory('landmark_detector'), 'config', 'landmarks.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['landmarks']


def to_range_bearing(forward, left, radius, offset):
    """Camera sighting of a landmark's front face -> range and bearing of its center from the robot center."""
    r = math.hypot(forward, left)
    scale = (r - offset + radius) / r
    c, s = math.cos(nav.CAMERA_YAW), math.sin(nav.CAMERA_YAW)
    x = (forward * c - left * s) * scale + nav.CAMERA_X
    y = (forward * s + left * c) * scale
    return math.hypot(x, y), math.atan2(y, x)


class SlamNavigator(EkfNavigator):
    def __init__(self, stem):
        super().__init__(stem)
        self.objects = load_objects()
        self.landmarks = {}  # nothing known in advance; no stop-landmark logic
        self.loop = 0
        self.added = 0
        self.tag_boxes = {}  # tag name -> (time, x1, y1, x2, y2) in pixels
        self.masked = 0
        self.map_log = open(stem + '_map.csv', 'w')
        self.map_log.write('loop,name,x,y,sxx,sxy,syy\n')
        nav.MAX_SIGMA = MAX_SIGMA
        nav.MAX_REJECTIONS = MAX_REJECTIONS

    def start_at(self, pose):
        self.measure_gyro_bias()
        self.ekf = SlamEkf(*pose)
        self.heading = pose[2]
        self.localizing = True

    def set_stop_landmark(self, gx, gy, ux, uy, leg):
        self.stop_landmark = None
        self.arrived = False

    def sighting(self, name, forward, left):
        """Return (range, bearing, sigma_range, sigma_bearing) or None for classes not used."""
        if name.startswith('tag_'):
            rng, bearing = to_range_bearing(forward, left, 0.0, 0.0)
            return rng, bearing, max(TAG_SIGMA_MIN, TAG_SIGMA_RANGE * rng), TAG_SIGMA_BEARING
        lm = self.objects.get(name)
        if lm is None or 'radius' not in lm:
            return None
        rng, bearing = to_range_bearing(forward, left, lm['radius'], nav.RANGE_OFFSET)
        return rng, bearing, OBJECT_SIGMA_RANGE * rng, OBJECT_SIGMA_BEARING

    def tag_box(self, det):
        b = det.bbox
        return (b.center.position.x - b.size_x / 2.0, b.center.position.y - b.size_y / 2.0,
                b.center.position.x + b.size_x / 2.0, b.center.position.y + b.size_y / 2.0)

    def on_a_tag(self, det, now):
        """True if this object detection sits on a recently seen AprilTag."""
        u, v = det.bbox.center.position.x, det.bbox.center.position.y
        for t, x1, y1, x2, y2 in self.tag_boxes.values():
            gx, gy = TAG_MASK_GROW * (x2 - x1), TAG_MASK_GROW * (y2 - y1)
            if now - t < TAG_MASK_TIME and x1 - gx <= u <= x2 + gx and y1 - gy <= v <= y2 + gy:
                return True
        return False

    def on_observations(self, msg):
        now = time.time()
        for det in msg.detections:  # remember tag boxes first, even while turning
            name = det.results[0].hypothesis.class_id
            if name.startswith('tag_'):
                self.tag_boxes[name] = (now, *self.tag_box(det))
        moving = abs(self.speed) > 0.01
        if not self.localizing or now - self.last_turning < nav.MOTION_BLANK:
            return
        if not moving and now - self.last_driving < nav.MOTION_BLANK:
            return
        for det in msg.detections:
            name = det.results[0].hypothesis.class_id
            if now - self.last_sighting.get(name, 0.0) < nav.SIGHTING_PERIOD:
                continue
            if not name.startswith('tag_') and self.on_a_tag(det, now):
                self.masked += 1
                continue
            p = det.results[0].pose.pose.position
            s = self.sighting(name, p.x - self.speed * nav.IMAGE_LATENCY, p.y)
            if s is None:
                continue
            self.last_sighting[name] = now
            rng, bearing, sr, sb = s
            if moving:
                sr = max(sr, nav.SIGMA_RANGE_MOVING)
            known = name in self.ekf.index
            expected = self.ekf.expected(name) if known else (float('nan'), float('nan'))
            result = self.ekf.observe(name, rng, bearing, sr, sb)
            if result == 'new':
                self.added += 1
                x, y = self.ekf.landmark(name)[0]
                print(f'  new landmark {name} at ({x:+.2f}, {y:+.2f})')
            if known:
                ok = result == 'used'
                self.used += ok
                self.rejected += not ok
                self.check_lost(name, ok)
            self.obs_log.write(f'{now - self.started:.2f},{name},{rng:.3f},{bearing:.4f},'
                               f'{expected[0]:.3f},{expected[1]:.4f},{result}\n')
        self.heading = float(self.ekf.state[2])

    def log_map(self):
        for name in sorted(self.ekf.index):
            (x, y), P = self.ekf.landmark(name)
            self.map_log.write(f'{self.loop},{name},{x:.4f},{y:.4f},{P[0, 0]:.6f},{P[0, 1]:.6f},{P[1, 1]:.6f}\n')
        self.map_log.flush()

    def print_map(self):
        print(f'    ({self.masked} object detections dropped for sitting on a tag)')
        for name in sorted(self.ekf.index):
            (x, y), P = self.ekf.landmark(name)
            print(f'    {name:13s} ({x:+.3f}, {y:+.3f})  sigma ({math.sqrt(P[0, 0]):.3f}, {math.sqrt(P[1, 1]):.3f})')

    def run_loops(self, waypoints, loops):
        self.start_at(waypoints[0])
        self.limits_on = True
        nav.MAX_RUN_TIME = LOOP_TIME * loops + 20.0
        self.look()
        for self.loop in range(1, loops + 1):
            print(f'=== loop {self.loop}/{loops} ===')
            for i, (gx, gy, gtheta) in enumerate(waypoints[1:], start=1):
                print(f'[{i}/{len(waypoints) - 1}] -> ({gx:.2f}, {gy:.2f})')
                self.go_to(gx, gy)
                self.report(f'corner {i}')
            if self.loop == loops:  # face the start heading for the end pose
                if abs(wrap_angle(waypoints[0][2] - self.ekf.state[2])) > nav.TURN_FIRST_ABOVE:
                    self.turn_once(waypoints[0][2])
            self.log_map()
            print(f'  map after loop {self.loop}: {len(self.ekf.index)} landmarks')
            self.print_map()
        print('Done.')

    def go_to(self, gx, gy):
        for attempt in range(nav.MAX_ATTEMPTS):
            if self.distance_to(gx, gy) < nav.POSITION_TOLERANCE:
                return
            x, y, theta = self.ekf.state[:3]
            direction = math.atan2(gy - y, gx - x)
            off = abs(wrap_angle(direction - theta))
            if attempt > 0 and off > nav.RETRY_WITHIN:
                return
            if attempt == 0 and off > nav.TURN_FIRST_ABOVE:
                self.turn_once(direction)
            self.drive_to(gx, gy)
            self.look()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='EKF-SLAM on a closed path.')
    parser.add_argument('waypoints', help='file with one "x,y,theta" per line, first line is the start')
    parser.add_argument('--loops', type=int, default=1)
    parser.add_argument('--look-only', action='store_true', help='build the map without driving')
    opts = parser.parse_args(remove_ros_args(args=None)[1:])
    waypoints = load_waypoints(opts.waypoints)

    folder = os.path.expanduser('~/hw3_runs')
    os.makedirs(folder, exist_ok=True)
    stem = os.path.join(folder, time.strftime('%m%d_%H%M%S'))
    node = SlamNavigator(stem)
    try:
        if opts.look_only:
            node.start_at(waypoints[0])
            while rclpy.ok():
                node.look()
                node.report('still')
                node.print_map()
        else:
            node.run_loops(waypoints, opts.loops)
    except KeyboardInterrupt:
        pass
    except Abort as e:
        print(f'ABORT: {e}')
    finally:
        node.drive_for(0.2)  # make sure the robot stops
        node.log_map()       # final map, also after an abort (loop number = loop it stopped in)
        for f in (node.pose_log, node.obs_log, node.map_log):
            f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'Logs: {stem}_pose.csv, _obs.csv, _map.csv')
    return 0


if __name__ == '__main__':
    main()
