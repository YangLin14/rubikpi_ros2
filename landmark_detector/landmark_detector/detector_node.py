import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from landmark_detector.yolo import YoloDetector, draw_detections

# Run YOLO on the camera stream and publish detections.
#
#   ros2 run landmark_detector detector_node
#   ros2 run landmark_detector detector_node --ros-args -p model:=yolo11n_416.onnx \
#       -p classes:="['bottle','cup','tv']"
#
# Subscribes:  /camera/image_raw/compressed
# Publishes:   /landmarks/detections          vision_msgs/Detection2DArray (pixel boxes)
#              /landmarks/preview/compressed  annotated half-size JPEG for Foxglove
#
# The camera node must be running. Only the newest frame is processed, so if
# inference is slower than the camera, frames are dropped rather than queued.
# max_fps caps the detection rate: the board throttles hard when it runs hot.


class DetectorNode(Node):
    def __init__(self):
        super().__init__('landmark_detector')
        self.declare_parameter('model', 'landmarks_round2_416.onnx')
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('iou', 0.5)
        self.declare_parameter('threads', 4)
        self.declare_parameter('classes', [''])  # class names to keep; empty = all
        self.declare_parameter('publish_preview', True)
        self.declare_parameter('max_fps', 5.0)  # cap detection rate to limit CPU load and heat; 0 = every frame

        model = self.get_parameter('model').value
        if not os.path.isabs(model):
            model = os.path.join(get_package_share_directory('landmark_detector'), 'models', model)
        # Custom models keep their class names in a .txt next to the .onnx
        names_file = os.path.splitext(model)[0] + '.txt'
        names = None
        if os.path.exists(names_file):
            with open(names_file) as f:
                names = f.read().split()
        self.detector = YoloDetector(
            model,
            names=names,
            conf=self.get_parameter('conf').value,
            iou=self.get_parameter('iou').value,
            threads=self.get_parameter('threads').value)

        wanted = [c for c in self.get_parameter('classes').value if c]
        unknown = [c for c in wanted if c not in self.detector.names]
        if unknown:
            self.get_logger().warn(f'Unknown class names ignored: {unknown}')
        self.class_filter = {self.detector.names.index(c) for c in wanted if c in self.detector.names} or None
        self.publish_preview = self.get_parameter('publish_preview').value
        max_fps = self.get_parameter('max_fps').value
        self.min_period = 1.0 / max_fps if max_fps > 0 else 0.0
        self.last_run = 0.0

        self.det_pub = self.create_publisher(Detection2DArray, '/landmarks/detections', 10)
        self.preview_pub = self.create_publisher(CompressedImage, '/landmarks/preview/compressed', 1)
        self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)

        self.stats = {'frames': 0, 'infer': 0.0, 'total': 0.0, 'since': time.time()}
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'Loaded {os.path.basename(model)} (input {self.detector.size}), '
            f'classes: {wanted or "all"}')

    def on_image(self, msg):
        t0 = time.perf_counter()
        # 0.8: frames arrive with jitter, so a strict cutoff would skip one too many
        if t0 - self.last_run < 0.8 * self.min_period:
            return
        self.last_run = t0
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        dets = self.detector.detect(img, self.class_filter)

        out = Detection2DArray()
        out.header = msg.header
        for d in dets:
            x1, y1, x2, y2 = d.box
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.name
            hyp.hypothesis.score = d.score
            det.results.append(hyp)
            out.detections.append(det)
        self.det_pub.publish(out)

        if self.publish_preview:
            self.publish_preview_image(img, dets, msg.header)

        self.stats['frames'] += 1
        self.stats['infer'] += self.detector.last_infer_ms
        self.stats['total'] += (time.perf_counter() - t0) * 1000.0

    def publish_preview_image(self, img, dets, header):
        draw_detections(img, dets)
        small = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
        ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.preview_pub.publish(msg)

    def report(self):
        s = self.stats
        n = s['frames']
        if n:
            fps = n / (time.time() - s['since'])
            self.get_logger().info(
                f'{fps:.1f} fps | inference {s["infer"] / n:.1f} ms | per frame {s["total"] / n:.1f} ms')
        else:
            self.get_logger().warn('No images received - is the camera node running?')
        self.stats = {'frames': 0, 'infer': 0.0, 'total': 0.0, 'since': time.time()}


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
