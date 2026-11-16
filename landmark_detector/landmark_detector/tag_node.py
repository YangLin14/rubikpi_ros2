import collections
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from landmark_detector.tags import detect, make_detector

# AprilTag landmarks, published in the same format as range_node so the SLAM node
# treats tags and objects alike.
#
#   ros2 run landmark_detector tag_node --ros-args -p size:=0.16
#
# Subscribes:  /camera/image_raw/compressed, /camera/camera_info
# Publishes:   /landmarks/observations   class_id 'tag_<id>', results[0].pose.pose.position
#              x = forward, y = left in meters, from the camera to the tag center
#
# The terminal shows the median range and bearing per tag once a second, to check
# against a tape measure.


class TagNode(Node):
    def __init__(self):
        super().__init__('tag_node')
        self.declare_parameter('size', 0.16)        # m, edge of the black square (printed: 16 cm)
        self.declare_parameter('ids', [-1])  # tags to keep; [-1] = any id
        self.declare_parameter('max_fps', 5.0)
        self.size = self.get_parameter('size').value
        self.ids = {i for i in self.get_parameter('ids').value if i >= 0}  # empty = all
        self.period = 1.0 / self.get_parameter('max_fps').value
        self.detect_fn = make_detector()
        self.K = self.D = None
        self.last = 0.0
        self.recent = collections.defaultdict(list)
        self.pub = self.create_publisher(Detection2DArray, '/landmarks/observations', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.report)
        self.get_logger().info(f'AprilTag 36h11, size {self.size} m, ids {sorted(self.ids) or "any"}')

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)

    def on_image(self, msg):
        now = time.time()
        if self.K is None or now - self.last < self.period:
            return
        self.last = now
        gray = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        out = Detection2DArray()
        out.header = msg.header
        for tag_id, c, forward, left in detect(self.detect_fn, gray, self.size, self.K, self.D):
            if self.ids and tag_id not in self.ids:
                continue
            det = Detection2D()
            det.header = msg.header
            x1, y1 = c.min(axis=0)
            x2, y2 = c.max(axis=0)
            det.bbox.center.position.x = float(x1 + x2) / 2.0
            det.bbox.center.position.y = float(y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = f'tag_{tag_id}'
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = forward
            hyp.pose.pose.position.y = left
            det.results.append(hyp)
            out.detections.append(det)
            self.recent[tag_id].append((np.hypot(forward, left), np.degrees(np.arctan2(left, forward)),
                                        float(y2 - y1)))
        self.pub.publish(out)

    def report(self):
        for tag_id in sorted(self.recent):
            rows = self.recent[tag_id]
            if rows:
                r, b, h = np.median(np.array(rows), axis=0)
                self.get_logger().info(f'tag_{tag_id}: range {r:.3f} m, bearing {b:+.1f} deg, '
                                       f'{h:.0f} px (n={len(rows)})')
                rows.clear()


def main():
    rclpy.init()
    node = TagNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
