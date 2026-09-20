"""Mission-level automatic replan tests; no worker, ROS, or execution runtime."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rrm.continuous_replan import ContinuousReplanMission, MissionAuthorization
from rrm.cosmos_reason2 import parse_cosmos_candidate
from rrm.live_replan import LiveCycleResponse, LiveReplanCycle
from rrm_cosmos_reason2 import load_context


ROOT = Path(__file__).parents[1]


class Snapshotter:
    def __init__(self):
        self.stamp = 100

    def capture(self):
        image = f"frame-{self.stamp}".encode()
        metadata = {"capture_mode": "read_only", "captured_at": datetime.now(timezone.utc).isoformat(),
                    "source_stamp_ns": self.stamp, "source_stamp_advanced": True,
                    "frame_id": "camera_left", "sha256": hashlib.sha256(image).hexdigest(),
                    "vehicle": {"connected": True, "odometry_frame_id": "map",
                                "odometry_child_frame_id": "base_link", "odometry_stamp_ns": self.stamp,
                                "x": 0.0, "y": 0.0, "z": 0.1, "linear_speed_m_s": 0.0}}
        scene = {"source_stamp_ns": self.stamp, "verified_entities": ["blue_marker", "orange_marker"],
                 "provenance": "automatic-test-verifier/v1"}
        self.stamp += 1
        return metadata, image, scene


class Provider:
    def propose(self, request):
        raw = json.dumps({"status": "READY", "grounded_entities": ["blue_marker", "orange_marker"],
                          "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
                          "ambiguity_refs": [], "explanation": "bounded test proposal",
                          "actions": [{"id": f"blue-{request.step_index}", "verb": "NAVIGATE_TO",
                                       "targets": ["blue_marker"], "dependencies": []}], "recovery_budget": 0})
        return LiveCycleResponse(cycle_id=request.cycle_id, step_index=request.step_index,
                                 observation_sha256=request.observation["sha256"],
                                 candidate=parse_cosmos_candidate(raw, request.context))


class ContinuousReplanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        fixture = ROOT / "examples" / "office_visual_eval"
        context_path = Path(self.temp.name) / "context.json"
        shutil.copy(fixture / "navigation_context.json", context_path)
        self.context = load_context(context_path)
        self.cycle = LiveReplanCycle(Path(self.temp.name) / "cycle", task_id=self.context.task.task_id,
                                     task_revision=self.context.task.revision, expected_camera_frame="camera_left")
        self.mission = ContinuousReplanMission(
            self.cycle, self.context, Provider(), Snapshotter(),
            MissionAuthorization(task_id=self.context.task.task_id, task_revision=self.context.task.revision,
                                 allowed_verbs=frozenset({"NAVIGATE_TO"}),
                                 allowed_targets=frozenset({"blue_marker"}), max_actions=2))

    def test_one_mission_approval_then_verified_outcome_automatically_replans(self):
        initial = self.mission.propose_initial_action()
        self.assertEqual(initial["state"], "REVIEW_REQUIRED")
        authorized = self.mission.approve_mission()
        self.assertTrue(authorized["execution_authorized"])
        next_step = self.mission.record_verified_outcome_and_replan(
            authorized["action"]["id"], verified=True, detail="adapter independently verified arrival")
        self.assertTrue(next_step["execution_authorized"])
        self.assertEqual(next_step["action"]["id"], "blue-1")
        self.assertEqual(self.cycle.phase.value, "OUTCOME_REQUIRED")

    def test_failed_verification_halts_instead_of_replanning(self):
        self.mission.propose_initial_action()
        action = self.mission.approve_mission()["action"]
        result = self.mission.record_verified_outcome_and_replan(
            action["id"], verified=False, detail="adapter could not confirm completion")
        self.assertEqual(result["state"], "HALTED")
        self.assertFalse(result["execution_authorized"])

    def test_source_has_no_robot_or_scheduler_control_surface(self):
        source = (ROOT / "rrm" / "continuous_replan.py").read_text()
        for forbidden in ("rclpy", "mavros", "px4", "ActionClient", "subprocess", "sbatch"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
