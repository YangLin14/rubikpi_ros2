#!/usr/bin/env python3
"""Draft YOLO labels with YOLO-World, for review in a labeling tool.

Run on the Mac. Writes one YOLO .txt per image and annotated previews:

    python autolabel.py --images data/raw --out data/labels_draft

Draft labels are a head start, not ground truth: every image must be reviewed.
In testing, only the mug was reliable. The dog box misses its cup, and the bottle,
cereal box and fan were mostly missed, so a model trained on hand labels is the
better pre-labeler once a first batch exists.
"""
import argparse
import glob
import os

import cv2
from ultralytics import YOLOWorld

# Text prompts for each class, in the order of classes.txt
PROMPTS = {
    'water_bottle': 'water bottle',
    'toilet_paper': 'roll of toilet paper',
    'mug': 'coffee mug',
    'cereal_box': 'cereal box',
    'black_fan': 'small black fan',
    'dog_plushie': 'dog plush toy',
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--model', default='yolov8s-worldv2.pt')
    p.add_argument('--conf', type=float, default=0.05)
    p.add_argument('--imgsz', type=int, default=960)
    args = p.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'classes.txt')) as f:
        classes = f.read().split()
    model = YOLOWorld(args.model)
    model.set_classes([PROMPTS[c] for c in classes])

    os.makedirs(args.out, exist_ok=True)
    preview_dir = os.path.join(args.out, 'preview')
    os.makedirs(preview_dir, exist_ok=True)
    files = sorted(glob.glob(os.path.join(args.images, '**', '*.jpg'), recursive=True))
    counts = dict.fromkeys(classes, 0)
    for path in files:
        r = model.predict(path, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        name = os.path.splitext(os.path.basename(path))[0]
        with open(os.path.join(args.out, name + '.txt'), 'w') as f:
            for b in r.boxes:
                cid = int(b.cls)
                counts[classes[cid]] += 1
                f.write(f'{cid} ' + ' '.join(f'{v:.5f}' for v in b.xywhn[0].tolist()) + '\n')
        cv2.imwrite(os.path.join(preview_dir, name + '.jpg'), r.plot())
    print(f'{len(files)} images, detections per class: {counts}')


if __name__ == '__main__':
    main()
