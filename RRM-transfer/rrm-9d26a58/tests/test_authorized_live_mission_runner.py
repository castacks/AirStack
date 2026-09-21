"""CLI safety-gate tests for the explicit live mission runner."""
from pathlib import Path
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import rrm_authorized_live_mission as runner


class AuthorizedLiveMissionRunnerTests(unittest.TestCase):
    def test_checked_in_takeoff_context_and_authorization_are_compatible(self):
        root = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        context = runner.load_context(root / "takeoff_context.json")
        authorization = runner._authorization(root / "takeoff_authorization.example.json")
        self.assertEqual(context.task.task_id, authorization.task_id)
        self.assertEqual(context.task.revision, authorization.task_revision)
        self.assertEqual(authorization.allowed_verbs, frozenset({"TAKEOFF"}))
        self.assertEqual(authorization.allowed_targets, frozenset())

    def test_manifest_route_is_not_truncated_to_one_waypoint(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "scene.json"
            manifest.write_text(json.dumps({"markers": {"target": {
                "map_route": [
                    {"x": 1.0, "y": 0.0, "z": 1.5},
                    {"x": 2.0, "y": 1.0, "z": 1.5},
                ],
            }}}))
            bridge = runner._bridge(manifest)
        self.assertEqual(len(bridge.targets["target"].waypoints), 2)

    def test_execution_requires_both_explicit_flags_before_any_runtime_setup(self):
        base = ["rrm_authorized_live_mission.py", "--context", "context.json", "--scene-manifest", "scene.json",
                "--entity-catalog", "catalog.json", "--authorization", "authorization.json",
                "--worker-url", "http://worker", "--run-dir", "new-run"]
        for flags in (("--execute",), ("--simulator-only",)):
            with self.subTest(flags=flags), patch.object(sys, "argv", [*base, *flags]):
                with self.assertRaisesRegex(SystemExit, "requires both"):
                    runner.main()

    def test_default_parser_is_shadow_only(self):
        parsed = runner.parser().parse_args([
            "--context", "context.json", "--scene-manifest", "scene.json", "--entity-catalog", "catalog.json",
            "--authorization", "authorization.json", "--worker-url", "http://worker", "--run-dir", "new-run",
        ])
        self.assertFalse(parsed.execute)
        self.assertFalse(parsed.simulator_only)
        self.assertIsNone(parsed.feasibility_provider)

    def test_execution_requires_embodiment_feasibility_provider(self):
        base = ["rrm_authorized_live_mission.py", "--context", "context.json",
                "--scene-manifest", "scene.json", "--entity-catalog", "catalog.json",
                "--authorization", "authorization.json", "--worker-url", "http://worker",
                "--run-dir", "new-run", "--execute", "--simulator-only"]
        with patch.object(sys, "argv", base), self.assertRaisesRegex(
                SystemExit, "feasibility-provider"):
            runner.main()


if __name__ == "__main__":
    unittest.main()
