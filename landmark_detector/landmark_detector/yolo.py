import os
import time
from dataclasses import dataclass

import cv2
import numpy as np
import onnxruntime as ort

# Thin ONNX Runtime wrapper around an Ultralytics YOLO detection model exported
# with `yolo export format=onnx`. The model output is (1, 4 + num_classes, N):
# box center/size in input pixels followed by per-class scores (no NMS inside).
#
# On the Rubik Pi (QCS6490) cores 0-3 are slow A55s and 4-7 are A78s. Left alone,
# the scheduler often puts the inference threads on the A55s (~3.5x slower), so
# the process is pinned to the big cores before the session creates its threads.

BIG_CORES = {4, 5, 6, 7}

COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
]


@dataclass
class Detection:
    class_id: int
    name: str
    score: float
    box: tuple  # (x1, y1, x2, y2) in original image pixels


class YoloDetector:
    def __init__(self, model_path, names=None, conf=0.4, iou=0.5, threads=4, cpus=BIG_CORES):
        if cpus and hasattr(os, 'sched_setaffinity'):
            available = os.sched_getaffinity(0)
            if set(cpus) <= available:
                os.sched_setaffinity(0, cpus)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            model_path, opts, providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.size = inp.shape[2]  # square input, e.g. 320
        self.names = names or COCO_NAMES
        self.conf = conf
        self.iou = iou
        self.last_infer_ms = 0.0

    def preprocess(self, img):
        # Letterbox: resize keeping aspect ratio, pad to a square with gray
        h, w = img.shape[:2]
        scale = self.size / max(h, w)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        pad_x, pad_y = (self.size - nw) // 2, (self.size - nh) // 2
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return np.ascontiguousarray(blob), scale, pad_x, pad_y

    def detect(self, img, class_filter=None):
        blob, scale, pad_x, pad_y = self.preprocess(img)
        t0 = time.perf_counter()
        out = self.session.run(None, {self.input_name: blob})[0][0]  # (4 + C, N)
        self.last_infer_ms = (time.perf_counter() - t0) * 1000.0

        scores_all = out[4:]
        class_ids = scores_all.argmax(axis=0)
        scores = scores_all[class_ids, np.arange(out.shape[1])]
        keep = scores > self.conf
        if class_filter is not None:
            keep &= np.isin(class_ids, list(class_filter))
        if not keep.any():
            return []
        cx, cy, bw, bh = out[:4, keep]
        scores, class_ids = scores[keep], class_ids[keep]

        # Undo letterbox to get boxes in original image pixels
        x1 = (cx - bw / 2 - pad_x) / scale
        y1 = (cy - bh / 2 - pad_y) / scale
        boxes_xywh = np.stack([x1, y1, bw / scale, bh / scale], axis=1)

        # Per-class NMS: offset boxes by class so different classes never suppress each other
        offset = class_ids[:, None] * 10000.0
        shifted = boxes_xywh.copy()
        shifted[:, :2] += offset
        idx = cv2.dnn.NMSBoxes(shifted.tolist(), scores.tolist(), self.conf, self.iou)

        h, w = img.shape[:2]
        dets = []
        for i in np.array(idx).flatten():
            x, y, bw_i, bh_i = boxes_xywh[i]
            box = (float(max(0.0, x)), float(max(0.0, y)),
                   float(min(w - 1.0, x + bw_i)), float(min(h - 1.0, y + bh_i)))
            cid = int(class_ids[i])
            dets.append(Detection(cid, self.names[cid], float(scores[i]), box))
        return dets


def draw_detections(img, dets):
    for d in dets:
        x1, y1, x2, y2 = map(int, d.box)
        color = tuple(int(c) for c in np.random.default_rng(d.class_id).integers(64, 256, 3))
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f'{d.name} {d.score:.2f}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (x1, max(0, y1 - th - 8)), (x1 + tw + 4, y1), color, -1)
        cv2.putText(img, label, (x1 + 2, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return img
