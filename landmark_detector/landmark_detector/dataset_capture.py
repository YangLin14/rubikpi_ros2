import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

# Save camera frames for the landmark dataset. Blurry frames and near-duplicates
# are skipped, and the JPEG bytes are written as received (no re-encoding), so the
# dataset matches what the detector sees at runtime.
#
#   ros2 run landmark_detector dataset_capture --session living_room_1
#
# Watch /dataset/preview/compressed in Foxglove for the saved count.

MIN_SHARPNESS = 600.0  # variance of Laplacian on the upper part of the half-size image
MIN_CHANGE = 3.0       # mean abs pixel difference on a thumbnail


class DatasetCapture(Node):
    def __init__(self, args):
        super().__init__('dataset_capture')
        self.out = os.path.join(os.path.expanduser(args.out), args.session)
        os.makedirs(self.out, exist_ok=True)
        self.interval = args.interval
        self.max_images = args.max
        self.count = len(os.listdir(self.out))
        self.last_save = 0.0
        self.last_thumb = None
        self.create_subscription(CompressedImage, args.topic, self.on_image, qos_profile_sensor_data)
        self.preview_pub = self.create_publisher(CompressedImage, '/dataset/preview/compressed', 1)
        self.get_logger().info(f'Saving to {self.out} ({self.count} already there)')

    def on_image(self, msg):
        data = bytes(msg.data)
        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        small = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        # Upper part only: the carpet below the horizon stays textured even when blurred
        sharp = cv2.Laplacian(gray[:int(gray.shape[0] * 0.53)], cv2.CV_64F).var()
        thumb = cv2.resize(gray, (64, 36), interpolation=cv2.INTER_AREA).astype(np.float32)
        change = 999.0 if self.last_thumb is None else float(np.abs(thumb - self.last_thumb).mean())

        status = 'waiting'
        if sharp < MIN_SHARPNESS:
            status = 'blurry - hold still'
        elif change < MIN_CHANGE:
            status = 'same view - move rover or objects'
        elif time.time() - self.last_save >= self.interval:
            name = f'{self.count:05d}.jpg'
            with open(os.path.join(self.out, name), 'wb') as f:
                f.write(data)
            self.count += 1
            self.last_save = time.time()
            self.last_thumb = thumb
            status = f'SAVED {name}'

        text = f'{status} | total {self.count}/{self.max_images} | sharp {sharp:.0f}'
        cv2.rectangle(small, (0, 0), (small.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(small, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok and rclpy.ok():
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.preview_pub.publish(out)

        if self.count >= self.max_images:
            self.get_logger().info(f'Reached {self.count} images, done.')
            raise SystemExit


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--session', required=True, help='folder name, e.g. living_room_1')
    p.add_argument('--out', default='~/dataset/raw')
    p.add_argument('--interval', type=float, default=1.0, help='min seconds between saves')
    p.add_argument('--max', type=int, default=500)
    p.add_argument('--topic', default='/camera/image_raw/compressed')
    args = p.parse_args()

    rclpy.init()
    node = DatasetCapture(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    node.get_logger().info(f'Total: {node.count} images in {node.out}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
