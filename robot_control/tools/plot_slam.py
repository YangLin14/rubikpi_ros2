#!/usr/bin/env python3
"""Plot a slam_navigator run in the world frame and score the map against ground truth.

    python plot_slam.py ~/hw3_runs/1113_150000 square ../../docs/hw3/ground_truth.yaml out.png

The SLAM map frame is the robot's start pose, so each estimate is moved into the
world frame with the tape-measured start pose: p_world = R(theta0) p_map + t0.
Covariance ellipses are 2 sigma (86% for 2D).
"""
import argparse
import csv
import math

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Ellipse, Rectangle

PATH = '#2a6fb0'
EST = '#d9480f'
INK = '#333333'


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('stem', help='run path without _pose.csv')
    p.add_argument('path', help='start pose name in ground_truth.yaml (square / octagon)')
    p.add_argument('truth')
    p.add_argument('out')
    p.add_argument('--title', default=None)
    p.add_argument('--start', default=None, help='x,y,deg: overrides the start pose in the yaml')
    args = p.parse_args()

    with open(args.truth) as f:
        gt = yaml.safe_load(f)
    x0, y0, t0 = (float(v) for v in args.start.split(',')) if args.start else gt['starts'][args.path]
    t0 = math.radians(t0)
    R = np.array([[math.cos(t0), -math.sin(t0)], [math.sin(t0), math.cos(t0)]])
    world = lambda v: R @ np.asarray(v, float) + (x0, y0)  # noqa: E731

    poses = read(args.stem + '_pose.csv')
    rows = read(args.stem + '_map.csv')
    loops = sorted({int(r['loop']) for r in rows})

    print(f'{"loop":>4} ' + ' '.join(f'{n[:12]:>12}' for n in gt['landmarks']) + '      mean')
    final = {}
    for loop in loops:
        est = {r['name']: r for r in rows if int(r['loop']) == loop}
        errs = []
        cells = []
        for name, truth in gt['landmarks'].items():
            if name in est:
                e = float(np.linalg.norm(world((float(est[name]['x']), float(est[name]['y']))) - truth))
                errs.append(e)
                cells.append(f'{e * 100:10.1f}cm')
            else:
                cells.append(f'{"missing":>12}')
        extra = sorted(set(est) - set(gt['landmarks']))
        print(f'{loop:4d} ' + ' '.join(cells) + f'  {np.mean(errs) * 100:6.1f} cm'
              + (f'  extra: {extra}' if extra else ''))
        final = est

    fig, ax = plt.subplots(figsize=(6, 8), dpi=160)
    W, L = gt.get('arena', [2.0, 3.0])
    ax.add_patch(Rectangle((0, 0), W, L, fill=False, ec='#999999', lw=1, ls='--'))
    traj = np.array([world((float(r['x']), float(r['y']))) for r in poses if int(r['used']) >= 0])
    ax.plot(traj[:, 0], traj[:, 1], color=PATH, lw=1.2, alpha=0.8, label='Estimated trajectory')
    ax.scatter(*traj[0], s=80, marker='^', color=PATH, zorder=5, label='Start (measured)')
    for i, (name, truth) in enumerate(gt['landmarks'].items()):
        ax.scatter(*truth, s=70, marker='s', color=INK, zorder=4, label='Ground truth' if i == 0 else None)
        ax.annotate(name, truth, xytext=(6, 6), textcoords='offset points', fontsize=8, color=INK)
    for i, (name, r) in enumerate(sorted(final.items())):
        c = world((float(r['x']), float(r['y'])))
        cov = R @ np.array([[float(r['sxx']), float(r['sxy'])], [float(r['sxy']), float(r['syy'])]]) @ R.T
        vals, vecs = np.linalg.eigh(cov)
        ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
        ax.add_patch(Ellipse(c, 2 * 2 * math.sqrt(vals[1]), 2 * 2 * math.sqrt(vals[0]), angle=ang,
                             fc=EST, alpha=0.2, ec=EST, lw=0.8))
        ax.scatter(*c, s=30, marker='x', color=EST, zorder=6, label='SLAM estimate (2σ)' if i == 0 else None)
        if name in gt['landmarks']:
            ax.plot(*zip(c, gt['landmarks'][name]), color=EST, lw=0.6, alpha=0.6)
    ax.set_aspect('equal')
    ax.set_xlim(-0.3, W + 0.3)
    ax.set_ylim(-0.3, L + 0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(args.title or f'EKF-SLAM, {args.path}, {len(loops)} loop(s)', fontsize=11, loc='left')
    ax.grid(color='#dddddd', lw=0.6)
    ax.set_axisbelow(True)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=4, frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out)
    print('saved', args.out)


if __name__ == '__main__':
    main()
