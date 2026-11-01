#!/usr/bin/env python3
"""Build a YOLO dataset folder from images and labels, with a held-out val split.

Photos are shot in sequence, so neighbors are nearly identical and a random split
would leak training images into val. Whole blocks of consecutive images go to val,
and with --test-prefix a whole batch can be kept out of training entirely.

    python build_dataset.py --images ../../dataset/round1 --labels ../../dataset/labels \
        --out ../../dataset/yolo_round1
"""
import argparse
import glob
import os
import shutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', nargs='+', required=True)
    p.add_argument('--labels', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--block', type=int, default=6, help='consecutive images per val block')
    p.add_argument('--every', type=int, default=5, help='one val block in this many blocks')
    p.add_argument('--offset', type=int, default=0, help='shift which blocks go to val')
    p.add_argument('--test-prefix', default=None, help='images with this prefix form the test split')
    args = p.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'classes.txt')) as f:
        classes = f.read().split()

    images = []
    for folder in args.images:
        images += sorted(glob.glob(os.path.join(folder, '*.jpg')))
    out = os.path.abspath(args.out)
    if os.path.exists(out):
        shutil.rmtree(out)
    counts = {'train': 0, 'val': 0, 'test': 0}
    usable = [f for f in images
              if os.path.exists(os.path.join(args.labels, os.path.basename(f)[:-4] + '.txt'))]
    for i, path in enumerate(usable):
        name = os.path.basename(path)[:-4]
        if args.test_prefix and name.startswith(args.test_prefix):
            split = 'test'
        else:
            val_block = (args.every // 2 + args.offset) % args.every
            split = 'val' if (i // args.block) % args.every == val_block else 'train'
        for sub, src, ext in (('images', path, '.jpg'),
                              ('labels', os.path.join(args.labels, name + '.txt'), '.txt')):
            folder = os.path.join(out, sub, split)
            os.makedirs(folder, exist_ok=True)
            shutil.copy2(src, os.path.join(folder, name + ext))
        counts[split] += 1

    with open(os.path.join(out, 'data.yaml'), 'w') as f:
        f.write(f'path: {out}\ntrain: images/train\nval: images/val\n')
        if counts['test']:
            f.write('test: images/test\n')
        f.write(f'names: {classes}\n')
    print(counts, 'without labels:', len(images) - len(usable))


if __name__ == '__main__':
    main()
