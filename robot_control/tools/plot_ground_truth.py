#!/usr/bin/env python3
"""Top-down sketch of the measured HW3 map with the planned square and octagon.

    python plot_ground_truth.py ../../docs/hw3/ground_truth.yaml ../waypoints ../../docs/hw3/ground_truth_map.png
"""
import argparse
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Rectangle


def load(path):
    with open(path) as f:
        return [tuple(float(v) for v in line.split(',')[:2]) for line in f
                if line.strip() and not line.startswith('#')]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('truth')
    p.add_argument('waypoints')
    p.add_argument('out')
    args = p.parse_args()
    with open(args.truth) as f:
        gt = yaml.safe_load(f)
    W, L = gt['arena']
    fig, ax = plt.subplots(figsize=(6, 8), dpi=160)
    ax.add_patch(Rectangle((0, 0), W, L, fill=False, ec='#999999', lw=1, ls='--'))
    for name, color in (('square', '#2a6fb0'), ('octagon', '#2f9e44')):
        x0, y0, t0 = gt['starts'][name]
        t0 = math.radians(t0)
        R = np.array([[math.cos(t0), -math.sin(t0)], [math.sin(t0), math.cos(t0)]])
        pts = np.array([R @ q + (x0, y0) for q in load(os.path.join(args.waypoints, name + '.txt'))])
        ax.plot(pts[:, 0], pts[:, 1], '-o', color=color, ms=3, lw=1.2, label=f'{name} path')
        ax.annotate('', xy=(x0 + 0.15 * math.cos(t0), y0 + 0.15 * math.sin(t0)), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
    for name, (x, y) in gt['landmarks'].items():
        tag = name.startswith('tag_')
        ax.scatter(x, y, s=80, marker='s' if tag else 'o', color='#333333' if tag else '#d9480f', zorder=4)
        ax.annotate(f'{name}\n({x:.2f}, {y:.2f})', (x, y), xytext=(0, 9), textcoords='offset points',
                    fontsize=7, ha='center')
    ax.set_aspect('equal')
    ax.set_xlim(-0.45, W + 0.45)
    ax.set_ylim(-0.35, L + 0.4)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Ground-truth map (tape-measured), 2 x 3 m area', fontsize=11, loc='left')
    ax.grid(color='#dddddd', lw=0.6)
    ax.set_axisbelow(True)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=2, frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out)


if __name__ == '__main__':
    main()
