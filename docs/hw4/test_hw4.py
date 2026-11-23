"""Regression checks for segment clearance and selecting the maximin route."""

import json
import tempfile
import unittest
from pathlib import Path

from hw4 import Arena, metrics, plan_routes, segment_box_distance, shortest_path, widest_threshold


class PlannerTests(unittest.TestCase):
    def test_segment_cannot_cut_across_box_with_free_endpoints(self):
        box = (1, 1, 2, 2)
        self.assertEqual(segment_box_distance((0, 0), (3, 3), box), 0)
        self.assertEqual(segment_box_distance((0, 2), (2, 0), box), 0)
        self.assertAlmostEqual(segment_box_distance((0, 2.25), (3, 2.25), box), 0.25)

    def test_maximin_tie_break_does_not_discard_shorter_prefix(self):
        points = {"s": (0, 0), "a": (0, 1), "b": (1, 0), "g": (2, 0)}
        neighbors = {
            "s": [("a", 1, 3), ("b", 1, 2)],
            "a": [("b", 2, 3)],
            "b": [("g", 1, 1)],
            "g": [],
        }
        widest = widest_threshold(neighbors, "s", "g")
        self.assertEqual(widest, 1)
        self.assertEqual(shortest_path(points, neighbors, "s", "g", widest), [(0, 0), (1, 0), (2, 0)])

    def check_routes(self, arena):
        _, widest, _, routes = plan_routes(arena)
        short, safe = routes["distance"], routes["safety"]
        distance, safety = metrics(arena, short), metrics(arena, safe)
        self.assertEqual((short[0], safe[0]), (arena.start[:2], arena.start[:2]))
        self.assertEqual((short[-1], safe[-1]), (arena.goal, arena.goal))
        self.assertGreaterEqual(distance["min_clearance_m"] + 1e-9, arena.margin)
        self.assertGreaterEqual(safety["min_clearance_m"] + 1e-9, arena.margin)
        self.assertGreater(safety["obstacle_clearance_m"], distance["obstacle_clearance_m"] + 0.1)
        self.assertGreater(safety["length_m"], distance["length_m"])
        return widest, distance, safety

    def test_example_routes_maintain_clearance_after_shortcutting(self):
        arena = Arena.read(Path(__file__).with_name("arena.example.json"))
        widest, _, safety = self.check_routes(arena)
        self.assertAlmostEqual(safety["min_clearance_m"], widest, places=2)

    def test_start_near_wall_does_not_collapse_safety_route(self):
        # 2 x 3 m arena, start 35 cm from the right edge (clearance 19 cm), box in the middle
        raw = json.loads(Path(__file__).with_name("arena.example.json").read_text())
        raw.update(size_m=[2.0, 3.0], start=[1.65, 0.40, 1.570796], goal=[0.40, 2.50],
                   obstacles=[{"name": "box", "bounds_m": [0.88, 1.42, 1.13, 1.575]}])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "arena.json"
            path.write_text(json.dumps(raw))
            arena = Arena.read(path)
        widest, distance, safety = self.check_routes(arena)
        # The corridor beside the box allows (0.88 / 2) - 0.16 = 0.28 m; the start must not cap it
        self.assertGreater(widest, 0.25)
        self.assertGreater(safety["obstacle_clearance_m"], 0.25)

    def test_smaller_distance_margin_only_changes_the_shortest_route(self):
        raw = json.loads(Path(__file__).with_name("arena.example.json").read_text())
        routes = {}
        for margin in (0.10, 0.03):
            raw["distance_margin_m"] = margin
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "arena.json"
                path.write_text(json.dumps(raw))
                arena = Arena.read(path)
            routes[margin] = (arena, plan_routes(arena)[3])
        (wide, wide_routes), (tight, tight_routes) = routes[0.10], routes[0.03]
        self.assertEqual(wide_routes["safety"], tight_routes["safety"])
        tight_distance = metrics(tight, tight_routes["distance"])
        self.assertLess(tight_distance["obstacle_clearance_m"],
                        metrics(wide, wide_routes["distance"])["obstacle_clearance_m"])
        self.assertGreaterEqual(tight_distance["min_clearance_m"] + 1e-9, 0.03)
        self.assertGreaterEqual(metrics(tight, tight_routes["safety"])["min_clearance_m"] + 1e-9, 0.10)

    def test_duplicate_tag_ids_keep_separate_instances(self):
        arena = Arena.read(Path(__file__).with_name("arena.example.json"))
        self.assertEqual(arena.landmarks["box_tag_0"]["class"], "tag_0")
        self.assertEqual(arena.landmarks["tag_0"]["class"], "tag_0")
        self.assertNotIn("radius", arena.landmarks["box_tag_0"])
        self.assertEqual(arena.landmarks["cereal_box"]["class"], "cereal_box")
        self.assertEqual(arena.close_duplicates(), [])

    def test_unknown_landmark_class_is_rejected(self):
        raw = json.loads(Path(__file__).with_name("arena.example.json").read_text())
        raw["landmarks"]["mystery"] = {"position": [0.0, -0.1]}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "arena.json"
            path.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):
                Arena.read(path)


if __name__ == "__main__":
    unittest.main()
