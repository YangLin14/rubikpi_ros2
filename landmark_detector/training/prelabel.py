#!/usr/bin/env python3
"""Pre-label images with a trained model and import them into Label Studio.

Images must sit inside LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT. Boxes show up as
predictions, so a person reviews them instead of drawing every box.

    python prelabel.py --weights best.pt --images ../../dataset/round2 \
        --root ../../dataset --project 1 --token $LS_TOKEN
"""
import argparse
import glob
import json
import os
import urllib.request

from ultralytics import YOLO


def call(base, token, method, path, data=None):
    req = urllib.request.Request(
        base + path, method=method, data=None if data is None else json.dumps(data).encode(),
        headers={'Authorization': f'Token {token}', 'Content-Type': 'application/json'})
    with urllib.request.urlopen(req) as r:
        return json.loads(r.read() or '{}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', required=True)
    p.add_argument('--images', required=True)
    p.add_argument('--root', required=True, help='Label Studio local files document root')
    p.add_argument('--project', type=int, default=1)
    p.add_argument('--token', required=True)
    p.add_argument('--url', default='http://127.0.0.1:8080')
    p.add_argument('--conf', type=float, default=0.25)
    p.add_argument('--imgsz', type=int, default=416)
    args = p.parse_args()

    images, root = os.path.abspath(args.images), os.path.abspath(args.root)
    # Label Studio only serves local files under a storage connected to the project
    call(args.url, args.token, 'POST', '/api/storages/localfiles', {
        'project': args.project, 'title': os.path.basename(images), 'path': images,
        'regex_filter': r'.*\.jpg', 'use_blob_urls': True})

    model = YOLO(args.weights)
    tasks, boxes = [], 0
    for path in sorted(glob.glob(os.path.join(images, '*.jpg'))):
        r = model.predict(path, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        h, w = r.orig_shape
        result = []
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            result.append({
                'from_name': 'label', 'to_name': 'image', 'type': 'rectanglelabels',
                'original_width': w, 'original_height': h, 'score': float(b.conf),
                'value': {'x': x1 / w * 100, 'y': y1 / h * 100,
                          'width': (x2 - x1) / w * 100, 'height': (y2 - y1) / h * 100,
                          'rotation': 0, 'rectanglelabels': [r.names[int(b.cls)]]}})
        boxes += len(result)
        rel = os.path.relpath(path, root)
        tasks.append({'data': {'image': f'/data/local-files/?d={rel}'},
                      'predictions': [{'model_version': os.path.basename(args.weights),
                                       'result': result}]})
    out = call(args.url, args.token, 'POST', f'/api/projects/{args.project}/import', tasks)
    print(f'{len(tasks)} images, {boxes} predicted boxes, imported: {out.get("task_count")}')


if __name__ == '__main__':
    main()
