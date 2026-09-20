"""AirStack read-only dynamic feasibility provider tests."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal, MapWaypoint
from rrm.dynamic_feasibility import FeasibilityVerdict
from rrm_cosmos_reason2 import load_context
import airstack_drone_feasibility_provider as provider


ROOT = Path(__file__).parents[1]


class AirStackFeasibilityProviderTests(unittest.TestCase):
    def setUp(self):
        context = load_context(ROOT / "examples" / "office_visual_eval" / "navigation_context.json")
        image_sha = hashlib.sha256(b"live-frame").hexdigest()
        self.query = {
            "schema_version": "rrm-feasibility-query/v1",
            "semantic_action": {"id": "nav-blue", "verb": "NAVIGATE_TO",
                                "targets": ["blue_marker"], "params": {}},
            "proposal": DroneTaskProposal(
                task_id=context.task.task_id, action_id="nav-blue", kind=DroneTaskKind.NAVIGATE,
                frame_id="map", waypoints=(MapWaypoint(x=3.2, y=0.0, z=1.5),),
                goal_tolerance_m=0.3,
            ).model_dump(mode="json"),
            "task": context.task.model_dump(mode="json"),
            "snapshot": context.snapshot.model_dump(mode="json"),
            "capabilities": {
                "embodiment_id": context.capabilities.embodiment_id,
                "revision": context.capabilities.revision,
                "operations": sorted(context.capabilities.operations),
                "resources": sorted(context.capabilities.resources),
                "available_resources": sorted(context.capabilities.available_resources),
                "limits_ref": context.capabilities.limits_ref,
            },
            "now_monotonic_s": context.now_monotonic_s,
            "observation": {
                "sha256": image_sha,
                "vehicle": {"x": 0.0, "y": 0.0, "z": 1.5},
            },
            "scene_state": {"verified_entities": ["blue_marker"],
                            "provenance": "cosmos-live/v1"},
            "stop_generation": 4,
            "execution_dispatch": False,
        }
        self.report = {
            "schema_version": "airstack-feasibility-observation/v1",
            "execution_dispatch": False,
            "odometry_frame_id": "map", "odometry_child_frame_id": "base_link",
            "missing_channels": [], "stale_channels": [],
            "connected": True, "armed": True, "airborne": True, "has_control": True,
            "planner_stuck": False, "linear_speed_m_s": 0.02,
            "actions": {
                "/robot_1/tasks/navigate": ["task_msgs/action/NavigateTask"],
                "/robot_1/tasks/land": ["task_msgs/action/LandTask"],
            },
            "corridor": {
                "collision_free": True, "coverage_sufficient": True,
                "start": {"x": 0.0, "y": 0.0, "z": 1.5},
                "target": {"x": 3.2, "y": 0.0, "z": 1.5},
            },
        }

    def test_current_grounded_clear_corridor_is_feasible(self):
        with patch.object(provider, "_observe", return_value=self.report):
            result = provider.evaluate(self.query)
        self.assertEqual(result.verdict, FeasibilityVerdict.FEASIBLE)
        self.assertEqual(result.stop_generation, 4)
        self.assertEqual(result.profile_revision, self.query["capabilities"]["limits_ref"])
        self.assertEqual(json.loads(result.evidence_payload_json), self.report)
        self.assertEqual({check.name for check in result.checks}, {
            "grounding", "body_limits", "physics", "controller", "resources", "stop_channel",
        })

    def test_collision_and_grounded_vehicle_block(self):
        report = {**self.report, "airborne": False,
                  "corridor": {**self.report["corridor"], "collision_free": False}}
        with patch.object(provider, "_observe", return_value=report):
            result = provider.evaluate(self.query)
        self.assertEqual(result.verdict, FeasibilityVerdict.INFEASIBLE)
        failed = {check.name for check in result.checks if check.passed is False}
        self.assertEqual(failed, {"physics", "controller"})

    def test_missing_corridor_coverage_is_uncertain(self):
        report = {**self.report, "corridor": {}}
        with patch.object(provider, "_observe", return_value=report):
            result = provider.evaluate(self.query)
        self.assertEqual(result.verdict, FeasibilityVerdict.UNCERTAIN)

    def test_stale_channels_and_detached_corridor_are_uncertain(self):
        for report in (
            {**self.report, "stale_channels": ["cloud"]},
            {**self.report, "corridor": {
                **self.report["corridor"], "target": {"x": 9.0, "y": 0.0, "z": 1.5},
            }},
        ):
            with self.subTest(report=report):
                with patch.object(provider, "_observe", return_value=report):
                    result = provider.evaluate(self.query)
                self.assertEqual(result.verdict, FeasibilityVerdict.UNCERTAIN)

    def test_unknown_capability_profile_blocks_hardcoded_limits(self):
        self.query["capabilities"]["limits_ref"] = "unreviewed-limits/v2"
        with patch.object(provider, "_observe", return_value=self.report):
            result = provider.evaluate(self.query)
        self.assertEqual(result.verdict, FeasibilityVerdict.INFEASIBLE)
        self.assertFalse(next(check for check in result.checks if check.name == "body_limits").passed)

    def test_observer_and_provider_have_no_command_surface(self):
        for name in ("airstack_feasibility_observer.py", "airstack_drone_feasibility_provider.py"):
            source = (ROOT / "scripts" / name).read_text(encoding="utf-8").lower()
            for forbidden in ("create_publisher", "create_client", "actionclient", "send_goal", "mavros/cmd"):
                self.assertNotIn(forbidden, source)

    def test_provider_is_standalone_and_fails_closed_on_bad_query(self):
        environment = dict(os.environ)
        environment.pop("PYTHONPATH", None)
        completed = subprocess.run(
            [str(ROOT / "scripts" / "airstack_drone_feasibility_provider.py")],
            input="{}", text=True, capture_output=True, env=environment, timeout=5,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn('"error": "ValueError"', completed.stderr)


if __name__ == "__main__":
    unittest.main()
