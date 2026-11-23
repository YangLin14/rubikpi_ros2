#!/usr/bin/env python3
"""Plan, drive and analyze HW4 runs. The offline commands use only Python's stdlib."""

import argparse
import csv
import hashlib
import heapq
import itertools
import json
import math
import re
import subprocess
import sys
from dataclasses import dataclass
from html import escape
from pathlib import Path

MODES = ("distance", "safety")
OBJECTS = {"water_bottle", "toilet_paper", "cereal_box", "dog_plushie"}
TAG = re.compile(r"tag_\d+")
# Two landmarks of one class (two printed tag_0) closer than this may be confused.
SAME_CLASS_WARNING_M = 1.0
SAMPLE_M = 0.01  # spacing of clearance samples along a leg when judging safety
# Near the start and goal, a safety route must gain clearance at least this fast per meter
# driven. Leaving a corner diagonally gains 1/sqrt(2) per meter, so 0.5 is always possible.
ESCAPE_RATE = 0.5
EPS = 1e-9


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"Expected a finite number, got {value!r}")
    return float(value)


def vector(value, length):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"Expected {length} numeric entries, got {value!r}")
    return tuple(finite(item) for item in value)


@dataclass
class Arena:
    kind: str
    size: tuple
    grid: float
    radius: float
    margin: float
    start: tuple
    goal: tuple
    obstacles: dict
    landmarks: dict
    # The shortest route may keep a smaller margin than the safety route (default: the same)
    distance_margin: float = None

    @classmethod
    def read(cls, path):
        raw = json.loads(path.read_text())
        kind = raw["kind"]
        size = vector(raw["size_m"], 2)
        grid = finite(raw["grid_m"])
        radius = finite(raw["robot_radius_m"])
        margin = finite(raw["tracking_margin_m"])
        distance_margin = finite(raw.get("distance_margin_m", margin))
        start = vector(raw["start"], 3)
        goal = vector(raw["goal"], 2)
        obstacles = {
            item["name"]: vector(item["bounds_m"], 4) for item in raw["obstacles"]
        }
        landmarks = {}
        for name, item in raw["landmarks"].items():
            # class is the detector label; it defaults to the name, so a second tag_0
            # is written as e.g. "box_tag_0": {"class": "tag_0", ...}
            label = item.get("class", name)
            if label not in OBJECTS and not (isinstance(label, str) and TAG.fullmatch(label)):
                raise ValueError(f"Landmark {name}: class must be an HW2 object or tag_<id>, got {label!r}")
            landmarks[name] = {"class": label, "position": list(vector(item["position"], 2))}
            if label in OBJECTS:
                landmark_radius = finite(item["radius"])
                if landmark_radius < 0:
                    raise ValueError(f"Negative landmark radius for {name}")
                landmarks[name]["radius"] = landmark_radius

        if kind not in {"example", "measured"}:
            raise ValueError("kind must be example or measured")
        if min(size) <= 0 or grid <= 0 or radius <= 0 or margin <= 0:
            raise ValueError("Arena size, grid spacing, radius and margin must be positive")
        if not 0 <= distance_margin <= margin:
            raise ValueError("distance_margin_m must be between 0 and tracking_margin_m")
        if math.prod(math.ceil(side / grid) + 3 for side in size) > 40000:
            raise ValueError("Grid exceeds 40,000 vertices. Increase grid_m.")
        if not obstacles or len(obstacles) != len(raw["obstacles"]):
            raise ValueError("Give every obstacle a unique name")
        for name, (x0, y0, x1, y1) in obstacles.items():
            if not (0 <= x0 < x1 <= size[0] and 0 <= y0 < y1 <= size[1]):
                raise ValueError(f"Obstacle {name} must be a rectangle inside the arena")
        if start[:2] == goal:
            raise ValueError("Start and goal must differ")
        if not landmarks:
            raise ValueError("Give at least one landmark")
        arena = cls(kind, size, grid, radius, margin, start, goal, obstacles, landmarks, distance_margin)
        for label, point in (("start", start[:2]), ("goal", goal)):
            if arena.clearance(point, point) < margin:
                raise ValueError(f"The {label} has less than the specified tracking margin")
        return arena

    def close_duplicates(self):
        """Pairs of same-class landmarks near enough that the EKF may confuse them."""
        items = list(self.landmarks.items())
        return [
            (a, b, math.dist(la["position"], lb["position"]))
            for (a, la), (b, lb) in itertools.combinations(items, 2)
            if la["class"] == lb["class"]
            and math.dist(la["position"], lb["position"]) < SAME_CLASS_WARNING_M
        ]

    def clearance(self, a, b):
        """Minimum free space outside the robot's swept circular footprint."""
        width, height = self.size
        wall = min(min(x, y, width - x, height - y) for x, y in (a, b))
        clearance = min(wall, *(segment_box_distance(a, b, box) for box in self.obstacles.values()))
        return clearance - self.radius


def point_clearance(arena, point):
    """Arena.clearance for a single point, written out because the safety search calls it often."""
    x, y = point
    width, height = arena.size
    distance = min(x, y, width - x, height - y)
    for x0, y0, x1, y1 in arena.obstacles.values():
        distance = min(distance, math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)))
    return distance - arena.radius


def safety_width(arena):
    """Return width(a, b): the clearance that limits a safety route on leg a-b.

    The start and goal are given by the task, so their clearance is not a planning
    choice. A point at distance d from one of them may be as close as that endpoint's
    clearance + ESCAPE_RATE * d. Points within this allowance do not limit the route
    (width inf); elsewhere the width is the point's clearance. Without the allowance, a
    start near a wall would set the bottleneck and the safety route would collapse onto
    the shortest one.
    """
    ends = [(point, point_clearance(arena, point)) for point in (arena.start[:2], arena.goal)]

    def width(a, b):
        steps = max(1, math.ceil(math.dist(a, b) / SAMPLE_M))
        result = math.inf
        for k in range(steps + 1):
            t = k / steps
            point = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            clearance = point_clearance(arena, point)
            if clearance + EPS < min(c + ESCAPE_RATE * math.dist(point, end) for end, c in ends):
                result = min(result, clearance)
        return result

    return width


def point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if not length_sq:
        return math.dist(point, a)
    t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def segment_box_distance(a, b, box):
    x0, y0, x1, y1 = box
    enter, leave = 0.0, 1.0
    for start, end, low, high in zip(a, b, (x0, y0), (x1, y1)):
        delta = end - start
        if delta == 0:
            if not low <= start <= high:
                break
        else:
            first, last = sorted(((low - start) / delta, (high - start) / delta))
            enter, leave = max(enter, first), min(leave, last)
            if enter > leave:
                break
    else:
        return 0.0

    endpoint_distance = min(
        math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)) for x, y in (a, b)
    )
    return min(
        endpoint_distance,
        *(point_segment_distance(corner, a, b)
          for corner in itertools.product((x0, x1), (y0, y1))),
    )


def axis_positions(length, spacing, endpoints):
    # Keep exact measured endpoints and discard nearly coincident regular lines.
    regular = (spacing * index for index in range(math.ceil(length / spacing)))
    return sorted(set(endpoints) | {length} | {
        value for value in regular if all(abs(value - point) > EPS for point in endpoints)
    })


def build_graph(arena, margin=None):
    margin = arena.margin if margin is None else margin
    axes = [
        axis_positions(arena.size[axis], arena.grid, (arena.start[axis], arena.goal[axis]))
        for axis in range(2)
    ]
    points = {}
    for i, x in enumerate(axes[0]):
        for j, y in enumerate(axes[1]):
            if arena.clearance((x, y), (x, y)) + EPS >= margin:
                points[i, j] = (x, y)
    neighbors = {node: [] for node in points}
    for (i, j), a in points.items():
        for di, dj in ((0, 1), (1, -1), (1, 0), (1, 1)):
            other = (i + di, j + dj)
            if other not in points:
                continue
            b = points[other]
            clearance = arena.clearance(a, b)
            if clearance + EPS >= margin:
                distance = math.dist(a, b)
                neighbors[i, j].append((other, distance, clearance))
                neighbors[other].append(((i, j), distance, clearance))
    start = tuple(axis.index(value) for axis, value in zip(axes, arena.start))
    goal = tuple(axis.index(value) for axis, value in zip(axes, arena.goal))
    return points, neighbors, start, goal


def widest_threshold(neighbors, start, goal):
    best = {start: math.inf}
    queue = [(-math.inf, start)]
    while queue:
        negative_width, node = heapq.heappop(queue)
        width = -negative_width
        if width < best[node]:
            continue
        if node == goal:
            return width
        for other, _, clearance in neighbors[node]:
            candidate = min(width, clearance)
            if candidate > best.get(other, -math.inf):
                best[other] = candidate
                heapq.heappush(queue, (-candidate, other))
    raise ValueError("No route connects start to goal with the specified footprint and margin")


def shortest_path(points, neighbors, start, goal, threshold):
    best = {start: 0.0}
    parents = {}
    queue = [(math.dist(points[start], points[goal]), 0.0, start)]
    while queue:
        _, distance, node = heapq.heappop(queue)
        if distance > best[node]:
            continue
        if node == goal:
            result = [points[node]]
            while node != start:
                node = parents[node]
                result.append(points[node])
            return list(reversed(result))
        for other, step, clearance in neighbors[node]:
            cost = distance + step
            if clearance + EPS < threshold or cost >= best.get(other, math.inf) - EPS:
                continue
            parents[other] = node
            best[other] = cost
            heapq.heappush(queue, (cost + math.dist(points[other], points[goal]), cost, other))
    raise ValueError("No route connects start to goal at the requested clearance")


def simplify(arena, path, threshold, width=None, margin=None):
    """Find a short sequence of drivable straight legs along the sampled route.

    Each leg keeps the tracking margin and has width(a, b) >= threshold
    (default width: the leg's clearance)."""
    width = width or arena.clearance
    margin = arena.margin if margin is None else margin
    best = [(0, 0.0)]
    parents = [None]
    for end, b in enumerate(path[1:], start=1):
        choices = []
        for start, a in enumerate(path[:end]):
            if (arena.clearance(a, b) + EPS >= margin
                    and width(a, b) + EPS >= threshold):
                legs, distance = best[start]
                choices.append((legs + 1, distance + math.dist(a, b), start))
        legs, distance, parent = min(choices)
        best.append((legs, distance))
        parents.append(parent)
    result = []
    current = len(path) - 1
    while current is not None:
        result.append(path[current])
        current = parents[current]
    return list(reversed(result))


def metrics(arena, path):
    legs = list(zip(path, path[1:])) or [(path[0], path[0])]
    return {
        "length_m": sum(math.dist(a, b) for a, b in legs),
        "min_clearance_m": min(arena.clearance(a, b) for a, b in legs),
        # footprint to the nearest obstacle only, ignoring the arena boundary
        "obstacle_clearance_m": min(
            segment_box_distance(a, b, box) for a, b in legs for box in arena.obstacles.values()
        ) - arena.radius,
    }


def plan_routes(arena):
    """Return (grid points, safety width, grid routes, simplified routes) for both modes."""
    points, neighbors, start, goal = build_graph(arena, arena.distance_margin)
    width = safety_width(arena)
    widths = {}
    safe_neighbors = {}
    for node, edges in neighbors.items():
        safe_neighbors[node] = []
        for other, distance, clearance in edges:
            if clearance + EPS < arena.margin:
                continue  # the safety route keeps the full tracking margin
            key = frozenset((node, other))
            if key not in widths:
                widths[key] = width(points[node], points[other])
            safe_neighbors[node].append((other, distance, widths[key]))
    widest = widest_threshold(safe_neighbors, start, goal)
    grid_routes = {
        "distance": shortest_path(points, neighbors, start, goal, arena.distance_margin),
        "safety": shortest_path(points, safe_neighbors, start, goal, widest),
    }
    routes = {
        "distance": simplify(arena, grid_routes["distance"], arena.distance_margin,
                             margin=arena.distance_margin),
        "safety": simplify(arena, grid_routes["safety"], widest, width),
    }
    return points, widest, grid_routes, routes


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def write_waypoints(path, arena, route):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(arena.start)
        for index, point in enumerate(route[1:], start=1):
            a, b = (route[index - 1], point) if index == len(route) - 1 else (point, route[index + 1])
            writer.writerow((*point, math.atan2(b[1] - a[1], b[0] - a[0])))


def plot(arena, output, planned, estimated=None, vertices=None):
    colors = {"distance": "#17628c", "safety": "#c75325"}
    size = 630
    pad = 90
    scale = size / max(arena.size)
    total = size + 2 * pad
    estimate_label = " · Dashed: EKF estimate" if estimated else ""
    title = "Measured arena" if arena.kind == "measured" else "Example arena • sample inputs"
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{total}" height="{total}" viewBox="0 0 {total} {total}">',
        f'<rect width="{total}" height="{total}" fill="white"/>',
        f'<g font-family="sans-serif" fill="#182431">',
        f'<text x="{pad}" y="31" font-size="21" font-weight="bold">{title}</text>',
        f'<text x="{pad}" y="55" font-size="14"><tspan fill="{colors["distance"]}">Blue: shortest distance</tspan>'
        f' · <tspan fill="{colors["safety"]}">Orange: maximum clearance</tspan>{estimate_label}</text>',
    ]

    def xy(point):
        return (pad + point[0] * scale, pad + (arena.size[1] - point[1]) * scale)

    def rect(x0, y0, x1, y1, fill, **attrs):
        x, y = xy((x0, y1))
        attributes = " ".join(f'{key.replace("_", "-")}="{value}"' for key, value in attrs.items())
        return (f'<rect x="{x:.2f}" y="{y:.2f}" width="{(x1 - x0) * scale:.2f}" '
                f'height="{(y1 - y0) * scale:.2f}" fill="{fill}" {attributes}/>')

    width, height = arena.size
    offset = arena.radius + arena.margin
    svg.append(rect(0, 0, width, height, "#e2e9ee", stroke="#324553"))
    svg.append(rect(offset, offset, width - offset, height - offset, "#f6f9fb"))
    # A rounded rectangular dilation shows the exact footprint + margin around each box.
    for x0, y0, x1, y1 in arena.obstacles.values():
        svg.append(rect(x0 - offset, y0 - offset, x1 + offset, y1 + offset, "#d9e1e7",
                        rx=f"{offset * scale:.2f}"))
        if arena.distance_margin < arena.margin:
            inner = arena.radius + arena.distance_margin
            svg.append(rect(x0 - inner, y0 - inner, x1 + inner, y1 + inner, "none",
                            stroke=colors["distance"], stroke_dasharray="4 3",
                            rx=f"{inner * scale:.2f}"))
        svg.append(rect(x0, y0, x1, y1, "#758590"))
    for point in vertices or ():
        x, y = xy(point)
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="0.75" fill="#a1afb8"/>')
    for label, paths, dashed in (("planned", planned, False), ("estimated", estimated or {}, True)):
        for mode, path in paths.items():
            coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(xy, path))
            dash = 'stroke-dasharray="8 5"' if dashed else ''
            svg.append(
                f'<polyline aria-label="{mode} {label}" points="{coords}" fill="none" '
                f'stroke="{colors[mode]}" stroke-width="3" stroke-linejoin="round" {dash}/>'
            )
    for name, lm in arena.landmarks.items():
        x, y = xy(lm["position"])
        shift = -7 if x > total / 2 else 7
        anchor = "end" if shift < 0 else "start"
        svg.append(f'<circle cx="{x}" cy="{y}" r="3" fill="#364a59"/>')
        svg.append(f'<text x="{x + shift}" y="{y - 5}" text-anchor="{anchor}" font-size="11">'
                   f'{escape(name)}</text>')
    for label, point in (("Start", arena.start), ("Goal", arena.goal)):
        x, y = xy(point)
        svg.append(f'<circle cx="{x}" cy="{y}" r="5" fill="#152936" stroke="white"/>')
        svg.append(f'<text x="{x + 9}" y="{y - 9}" font-size="13">{label}</text>')
    x, y = xy((0, 0))
    svg += [
        f'<text x="{x}" y="{y + 19}" font-size="12">(0, 0)</text>',
        f'<text x="{x + width * scale - 38}" y="{y + 19}" font-size="12">x (m)</text>',
        f'<text x="{x - 40}" y="{pad + 10}" font-size="12">y (m)</text>',
        f'<text x="{pad}" y="{min(total - 12, y + 50)}" font-size="13">'
        'Gray band: footprint + tracking margin · Dark gray: obstacle'
        + (' · Blue dashes: footprint + shortest-route margin' if arena.distance_margin < arena.margin else '')
        + '</text>',
        '</g></svg>',
    ]
    output.write_text("\n".join(svg) + "\n")


def plan(args):
    arena = Arena.read(args.arena)
    points, widest, grid_routes, routes = plan_routes(arena)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "arena.json").write_bytes(args.arena.read_bytes())
    map_path = args.out / "ekf_landmarks.json"
    write_json(map_path, {"landmarks": arena.landmarks})
    for a, b, gap in arena.close_duplicates():
        print(f"Warning: {a} and {b} share a class and are only {gap:.2f} m apart. "
              "Sightings of them may be skipped as ambiguous.", file=sys.stderr)
    summary = {"kind": arena.kind, "safety_width_m": widest if math.isfinite(widest) else None,
               "routes": {}}
    for mode, route in routes.items():
        file = args.out / f"{mode}.txt"
        write_waypoints(file, arena, route)
        summary["routes"][mode] = {
            **metrics(arena, route), "grid": metrics(arena, grid_routes[mode]),
            "waypoints": len(route), "sha256": digest(file)
        }
    summary["map_sha256"] = digest(map_path)
    summary["arena_sha256"] = digest(args.out / "arena.json")
    plot(arena, args.out / "planned.svg", routes, vertices=points.values())
    write_json(args.out / "plan.json", summary)
    print(json.dumps(summary["routes"], indent=2))


def read_plan(folder, measured=False):
    metadata = json.loads((folder / "plan.json").read_text())
    expected = {
        "arena.json": metadata["arena_sha256"],
        "ekf_landmarks.json": metadata["map_sha256"],
        **{f"{mode}.txt": metadata["routes"][mode]["sha256"] for mode in MODES},
    }
    for name, checksum in expected.items():
        if digest(folder / name) != checksum:
            raise ValueError(f"{name} differs from the generated plan. Re-run plan.")
    arena = Arena.read(folder / "arena.json")
    if arena.kind != metadata["kind"]:
        raise ValueError("Arena and plan disagree about measurement status")
    if measured and arena.kind != "measured":
        raise ValueError("The sample plan is offline only. Plan with measured arena values first.")
    return arena, metadata


def read_waypoints(folder, mode):
    with (folder / f"{mode}.txt").open(newline="") as stream:
        return [tuple(map(float, row[:2])) for row in csv.reader(stream)]


def run(args):
    arena, metadata = read_plan(args.plan, measured=True)
    args.out.mkdir(parents=True)
    mode = args.mode
    command = [
        "ros2", "run", "robot_control", "ekf_navigator",
        str(args.plan / f"{mode}.txt"), "--landmark-map", str(args.plan / "ekf_landmarks.json"),
        "--log-prefix", str(args.out / "run"), "--strict-waypoints",
        # A miss within the tracking margin keeps the footprint clear of the obstacles.
        "--waypoint-tolerance", str(arena.margin),
    ]
    if args.look_only:
        command.append("--look-only")
        print("Stationary check. Stop with Ctrl-C.", flush=True)
    else:
        print(f"Starting {mode} drive. Keep the stop control accessible.", flush=True)
    record = {
        "mode": mode, "look_only": args.look_only, "plan": metadata,
        "status": "started",
    }
    record_path = args.out / "run.json"
    write_json(record_path, record)
    exit_code = 1
    try:
        exit_code = subprocess.run(command, check=False).returncode
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        record["status"] = "completed" if exit_code == 0 else "interrupted" if exit_code == 130 else "failed"
        record["exit_code"] = exit_code
        write_json(record_path, record)
    return 0 if args.look_only and exit_code == 130 else exit_code


def read_pose(path):
    with path.open(newline="") as stream:
        rows = [
            {key: finite(float(row[key])) for key in ("t", "x", "y", "used", "rejected", "left", "right")}
            for row in csv.DictReader(stream)
        ]
    if len(rows) < 2 or any(a["t"] > b["t"] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"{path} must have at least two pose rows in chronological order")
    return rows


def analyze(args):
    arena, plan_data = read_plan(args.plan, measured=True)
    planned = {mode: read_waypoints(args.plan, mode) for mode in MODES}
    estimated = {}
    result = {}
    for mode in MODES:
        folder = getattr(args, mode)
        record = json.loads((folder / "run.json").read_text())
        # A run stays valid while its own waypoints and the landmark map are unchanged, so
        # replanning the other route (e.g. a new distance margin) does not void it.
        used = record["plan"]
        if (record["mode"] != mode or record["look_only"] or used["kind"] != plan_data["kind"]
                or used["map_sha256"] != plan_data["map_sha256"]
                or used["routes"][mode]["sha256"] != plan_data["routes"][mode]["sha256"]):
            raise ValueError(f"{folder} is not a {mode} drive of the current {mode} waypoints and map")
        if record["status"] != "completed" or record["exit_code"] != 0:
            raise ValueError(f"{folder} did not complete. Retain its logs and repeat the drive.")
        rows = read_pose(folder / "run_pose.csv")
        # Older logs begin with placeholder (0, 0) rows written before the EKF was set to the
        # start. The start itself cannot be (0, 0): it must keep the margin from the walls.
        first = next((i for i, row in enumerate(rows) if (row["x"], row["y"]) != (0.0, 0.0)), None)
        if first is None:
            raise ValueError(f"{folder} has no pose estimates")
        rows = rows[first:]
        if math.dist((rows[0]["x"], rows[0]["y"]), arena.start[:2]) > 0.1:
            raise ValueError(f"{folder} starts {math.dist((rows[0]['x'], rows[0]['y']), arena.start[:2]):.2f} m "
                             "from the planned start pose")
        moving =[row for row in rows if abs(row["left"]) + abs(row["right"]) > EPS]
        if not moving:
            raise ValueError(f"{folder} has no nonzero wheel commands")
        path = [(row["x"], row["y"]) for row in rows]
        estimated[mode] = path
        max_error = max(
            min(point_segment_distance(point, a, b)
                for a, b in zip(planned[mode], planned[mode][1:]))
            for point in path
        )
        result[mode] = {
            **metrics(arena, path),
            "command_span_s": moving[-1]["t"] - moving[0]["t"],
            "goal_error_m": math.dist(path[-1], arena.goal),
            "max_cross_track_error_m": max_error,
            "accepted": int(rows[-1]["used"]),
            "rejected": int(rows[-1]["rejected"]),
        }
    args.out.mkdir(parents=True, exist_ok=True)
    plot(arena, args.out / "executed.svg", planned, estimated)
    write_json(args.out / "results.json", {"plan": plan_data, "estimated": result})
    print(json.dumps(result, indent=2))


def report(args):
    arena, plan_data = read_plan(args.plan)
    rows = []
    for mode in MODES:
        data = plan_data["routes"][mode]
        rows.append(f'| {mode.title()} | {data["length_m"]:.3f} | {data["min_clearance_m"]:.3f} | '
                    f'{data["obstacle_clearance_m"]:.3f} | {data["waypoints"] - 1} |')
    preliminary = "The table and figure below use example inputs. Field measurements and rover runs are pending."
    links = "Distance and safety video links pending."
    run_results = ("Rover data are pending. Record the final physical goal offset and whether the rover touched a "
                   "boundary or obstacle, then compare those observations with the EKF results.")
    if arena.kind == "measured":
        preliminary = "The planned paths below use field measurements."
    if args.results:
        results = json.loads((args.results / "results.json").read_text())
        if arena.kind != "measured" or results["plan"] != plan_data:
            raise ValueError("Results do not correspond to this measured plan")
        run_rows = []
        for mode in MODES:
            data = results["estimated"][mode]
            run_rows.append(
                f'| {mode.title()} | {data["length_m"]:.3f} | {data["min_clearance_m"]:.3f} | '
                f'{data["obstacle_clearance_m"]:.3f} | '
                f'{data["command_span_s"]:.2f} | {data["goal_error_m"]:.3f} | '
                f'{data["max_cross_track_error_m"]:.3f} | {data["accepted"]} / {data["rejected"]} |'
            )
        run_results = "\n".join([
            "![Planned paths and estimated trajectories](executed.svg)", "",
            "| Run | EKF distance (m) | EKF min. clearance (m) | EKF obstacle clearance (m) | Command span (s) |"
            " EKF goal error (m) | Max. cross-track error (m) | Sightings used / rejected |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |", *run_rows, "",
            "Physical stop error and contact observations: **[enter the tape measurements and video observations]**.",
            "", "Interpretation: **[compare the two real runs and explain any deviations from the planned paths]**.",
        ])
    if args.distance_video and args.safety_video:
        links = f"[Distance run]({args.distance_video}) · [Safety run]({args.safety_video})"
    elif args.distance_video or args.safety_video:
        raise ValueError("Provide links for both runs")

    args.out.mkdir(parents=True, exist_ok=True)
    setup = [
        f"The arena in this figure is {arena.size[0]:.3f} × {arena.size[1]:.3f} m. Its origin is the lower-left "
        f"corner of the plot, with +x right and +y up. The rover position is its turning center. The start "
        f"pose is ({arena.start[0]:.3f}, {arena.start[1]:.3f}, {arena.start[2]:.3f} rad). The goal is "
        f"({arena.goal[0]:.3f}, {arena.goal[1]:.3f}) m. The grid spacing is {arena.grid:.3f} m, the "
        f"footprint radius is {arena.radius:.3f} m, and the tracking margin is {arena.margin:.3f} m"
        + (f" for the safety route and {arena.distance_margin:.3f} m for the shortest route."
           if arena.distance_margin < arena.margin else "."),
        "", "| Obstacle | x min (m) | y min (m) | x max (m) | y max (m) |", "| --- | ---: | ---: | ---: | ---: |",
        *(f'| {name} | ' + ' | '.join(f'{value:.3f}' for value in bounds) + ' |'
          for name, bounds in arena.obstacles.items()),
        "", "| Landmark | Class | x (m) | y (m) |", "| --- | --- | ---: | ---: |",
        *(f'| {name} | {item["class"]} | {item["position"][0]:.3f} | {item["position"][1]:.3f} |'
          for name, item in arena.landmarks.items()),
    ]
    template = (Path(__file__).parent / "report.template.md").read_text()
    for token, value in (
        ("STATUS", preliminary), ("VIDEOS", links), ("SETUP", "\n".join(setup)),
        ("PLAN_TABLE", "\n".join(rows)), ("RUN_RESULTS", run_results),
    ):
        template = template.replace(f"<!-- {token} -->", value)
    (args.out / "HW4_Report.md").write_text(template)
    (args.out / "planned.svg").write_bytes((args.plan / "planned.svg").read_bytes())
    if args.results:
        (args.out / "executed.svg").write_bytes((args.results / "executed.svg").read_bytes())
    print(args.out / "HW4_Report.md")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    cmds = {name: sub.add_parser(name) for name in ("plan", "run", "analyze", "report")}
    cmds["plan"].add_argument("--arena", type=Path, required=True)
    cmds["run"].add_argument("--mode", choices=MODES, required=True)
    driving = cmds["run"].add_mutually_exclusive_group(required=True)
    driving.add_argument("--look-only", action="store_true")
    driving.add_argument("--drive", action="store_true")
    for mode in MODES:
        cmds["analyze"].add_argument(f"--{mode}", type=Path, required=True)
        cmds["report"].add_argument(f"--{mode}-video")
    cmds["report"].add_argument("--results", type=Path)
    for name, call in (("plan", plan), ("run", run), ("analyze", analyze), ("report", report)):
        cmds[name].add_argument("--out", type=Path, required=True)
        if name != "plan":
            cmds[name].add_argument("--plan", type=Path, required=True)
        cmds[name].set_defaults(call=call)
    args = parser.parse_args()
    try:
        return args.call(args) or 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"HW4: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
