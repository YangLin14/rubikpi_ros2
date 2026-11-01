#!/usr/bin/env python3
"""Convert a Label Studio JSON export into YOLO label files.

    python export_labels.py export.json ../../dataset/labels
"""
import argparse
import json
import os

MIN_BOX_PX = 15  # smaller boxes are accidental clicks in the labeling tool


def main():
    p = argparse.ArgumentParser()
    p.add_argument('export')
    p.add_argument('out')
    args = p.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'classes.txt')) as f:
        classes = f.read().split()
    with open(args.export) as f:
        tasks = json.load(f)

    os.makedirs(args.out, exist_ok=True)
    counts = dict.fromkeys(classes, 0)
    written = skipped = dropped = 0
    for task in tasks:
        anns = [a for a in task['annotations'] if not a.get('was_cancelled')]
        if not anns:
            skipped += 1
            continue
        name = os.path.splitext(os.path.basename(task['data']['image']))[0]
        lines = []
        for r in anns[0]['result']:
            if r.get('type') != 'rectanglelabels':
                continue
            v = r['value']
            if min(v['width'] / 100 * r['original_width'],
                   v['height'] / 100 * r['original_height']) < MIN_BOX_PX:
                dropped += 1
                continue
            label = v['rectanglelabels'][0]
            cid = classes.index(label)
            w, h = v['width'] / 100, v['height'] / 100
            cx, cy = v['x'] / 100 + w / 2, v['y'] / 100 + h / 2
            lines.append(f'{cid} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}')
            counts[label] += 1
        with open(os.path.join(args.out, name + '.txt'), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        written += 1
    print(f'{written} label files, {skipped} tasks without annotations, '
          f'{dropped} tiny boxes dropped, boxes: {counts}')


if __name__ == '__main__':
    main()
