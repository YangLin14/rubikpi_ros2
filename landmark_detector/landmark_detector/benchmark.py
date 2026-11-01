import argparse
import os
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from landmark_detector.yolo import YoloDetector

# Time YOLO inference on random images for each model and thread count, without
# needing the camera. Helps pick the model size / thread count for the rover.
#
#   ros2 run landmark_detector benchmark
#   ros2 run landmark_detector benchmark --threads 2 4 8 --runs 30


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--models', nargs='+', default=['yolo11n_320.onnx', 'yolo11n_416.onnx'])
    p.add_argument('--threads', nargs='+', type=int, default=[2, 4, 6])
    p.add_argument('--runs', type=int, default=20)
    args = p.parse_args()

    models_dir = os.path.join(get_package_share_directory('landmark_detector'), 'models')
    img = np.random.default_rng(0).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
    for model in args.models:
        for threads in args.threads:
            det = YoloDetector(os.path.join(models_dir, model), threads=threads)
            for _ in range(3):  # warm-up
                det.detect(img)
            infer, total = [], []
            for _ in range(args.runs):
                t0 = time.perf_counter()
                det.detect(img)
                total.append((time.perf_counter() - t0) * 1000.0)
                infer.append(det.last_infer_ms)
            print(f'{model:20s} threads={threads}  inference {np.median(infer):6.1f} ms  '
                  f'total {np.median(total):6.1f} ms  (~{1000.0 / np.median(total):.1f} fps)')


if __name__ == '__main__':
    main()
