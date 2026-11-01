#!/usr/bin/env python3
"""Plot the estimated trajectory of an ekf_navigator run and print its path length.

    python plot_run.py ../../hw2_runs/1101_023758_pose.csv ../../docs/hw2_trajectory.png
"""
import argparse
import csv
import math

import matplotlib.pyplot as plt

WAYPOINTS = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]
LANDMARKS = {'toilet paper': (1.35, 0.0), 'dog': (1.2, 1.0), 'water bottle': (1.2, 2.0), 'cereal box': (-0.15, -0.30)}
DOT_PERIOD = 0.5  # seconds
PATH = '#2a6fb0'
INK = '#333333'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('pose_csv')
    p.add_argument('out')
    args = p.parse_args()

    with open(args.pose_csv) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]

    dist = turn = 0.0
    for a, b in zip(rows, rows[1:]):
        if a['left'] + a['right'] > 0.1:
            dist += math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        elif a['left'] * a['right'] < 0.0:
            turn += abs(math.atan2(math.sin(b['theta'] - a['theta']), math.cos(b['theta'] - a['theta'])))
    moving = [r for r in rows if r['used'] > 0]
    print(f'Driven {dist:.2f} m, turned {math.degrees(turn):.0f} deg, '
          f'{moving[-1]["t"] - moving[0]["t"]:.1f} s')

    dots, next_t = [], moving[0]['t']
    for r in moving:
        if r['t'] >= next_t:
            dots.append(r)
            next_t += DOT_PERIOD

    fig, ax = plt.subplots(figsize=(6, 7.5), dpi=160)
    ax.plot([r['x'] for r in dots], [r['y'] for r in dots], color=PATH, lw=1, alpha=0.5, zorder=2)
    ax.scatter([r['x'] for r in dots], [r['y'] for r in dots], s=22, color=PATH, zorder=3,
               edgecolor='white', linewidth=0.5, label=f'Estimated pose, every {DOT_PERIOD} s')
    ax.scatter(*zip(*WAYPOINTS), s=130, marker='o', facecolor='none', edgecolor=INK, linewidth=1.5,
               zorder=4, label='Waypoint')
    ax.scatter(*zip(*LANDMARKS.values()), s=70, marker='s', color=INK, zorder=4, label='Landmark')
    for name, (x, y) in LANDMARKS.items():
        ax.annotate(name, (x, y), xytext=(9, -3), textcoords='offset points', fontsize=9, color=INK)
    ax.set_aspect('equal')
    ax.set_xlim(-0.5, 2.0)
    ax.set_ylim(-0.6, 2.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Estimated trajectory, full route', fontsize=11, loc='left')
    ax.grid(color='#dddddd', lw=0.6)
    ax.set_axisbelow(True)
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(loc='lower right', frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out)


if __name__ == '__main__':
    main()
