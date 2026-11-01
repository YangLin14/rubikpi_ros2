import collections
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from landmark_detector.geometry import estimate

# Estimate range and bearing of each detected landmark.
#
#   ros2 run landmark_detector range_node
#
# Subscribes:  /camera/camera_info     intrinsics and distortion
#              /landmarks/detections   pixel boxes from detector_node
# Publishes:   /landmarks/observations vision_msgs/Detection2DArray; the pixel box
#              plus results[0].pose.pose.position, x = forward and y = left in meters
#
# All values are relative to the camera. Boxes cut by the image border and classes
# without a known height are dropped. The terminal shows the median of the last
# few frames per landmark, which is what to compare against a tape measure.


class RangeNode(Node):
    def __init__(self):
        super().__init__('landmark_range')
        default = os.path.join(get_package_share_directory('landmark_detector'), 'config', 'landmarks.yaml')
        self.declare_parameter('landmarks', default)
        self.declare_parameter('window', 15)  # frames in the median shown in the terminal
        self.declare_parameter('log_period', 1.0)

        with open(self.get_parameter('landmarks').value) as f:
            self.heights = {k: v.get('range_height', v['height']) for k, v in yaml.safe_load(f)['landmarks'].items()}
        self.K = self.D = self.size = None
        self.recent = collections.defaultdict(
            lambda: collections.deque(maxlen=self.get_parameter('window').value))

        self.obs_pub = self.create_publisher(Detection2DArray, '/landmarks/observations', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/landmarks/detections', self.on_detections, 10)
        self.create_timer(self.get_parameter('log_period').value, self.report)

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        self.size = (msg.width, msg.height)

    def on_detections(self, msg):
        if self.K is None:
            return
        out = Detection2DArray()
        out.header = msg.header
        for det in msg.detections:
            hyp = det.results[0]
            name = hyp.hypothesis.class_id
            if name not in self.heights:
                continue
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            box = (cx - det.bbox.size_x / 2.0, cy - det.bbox.size_y / 2.0,
                   cx + det.bbox.size_x / 2.0, cy + det.bbox.size_y / 2.0)
            obs = estimate(box, self.heights[name], self.K, self.D, *self.size)
            if obs is None:
                continue
            hyp.pose.pose.position.x = obs.forward
            hyp.pose.pose.position.y = obs.left
            out.detections.append(det)
            self.recent[name].append((obs.range, obs.bearing, det.bbox.size_y))
        self.obs_pub.publish(out)

    def report(self):
        for name in sorted(self.recent):
            rows = self.recent[name]
            if rows:
                r, b, h = np.median(np.array(rows), axis=0)
                self.get_logger().info(
                    f'{name}: range {r:.3f} m, bearing {np.degrees(b):+.1f} deg, '
                    f'height {h:.0f} px (n={len(rows)})')
                rows.clear()


def main():
    rclpy.init()
    node = RangeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
