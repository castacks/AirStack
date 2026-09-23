"""Read-only environment evidence helpers."""
import math
from pathlib import Path
import unittest

from rrm.airstack_environment import finite_xyz_bounds


class AirStackEnvironmentTests(unittest.TestCase):
    def test_bounds_ignore_nonfinite_and_malformed_points(self):
        bounds = finite_xyz_bounds([
            (1, 2, 3), (-2, 5, -1), (math.nan, 0, 0), (1,), (4, -3, 2),
        ])
        self.assertEqual(bounds, {
            "min_x": -2.0, "max_x": 4.0,
            "min_y": -3.0, "max_y": 5.0,
            "min_z": -1.0, "max_z": 3.0,
        })

    def test_no_finite_points_produce_no_claimed_bounds(self):
        self.assertIsNone(finite_xyz_bounds([(math.inf, 0, 0), (math.nan, 1, 2)]))

    def test_discovery_uses_server_graph_and_has_no_command_surface(self):
        source = (Path(__file__).parents[1] / "scripts" / "airstack_task_discovery.py").read_text()
        self.assertIn("get_action_server_names_and_types_by_node", source)
        self.assertNotIn('suffix = "/_action/status"', source)
        for prohibited in ("ActionClient", "create_publisher(", "create_client("):
            self.assertNotIn(prohibited, source)


if __name__ == "__main__":
    unittest.main()
