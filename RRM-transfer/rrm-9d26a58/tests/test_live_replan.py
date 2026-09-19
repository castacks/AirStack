"""Shadow-only live replan orchestration tests; no model, ROS, or dispatch."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rrm.cosmos_reason2 import parse_cosmos_candidate, render_cosmos_prompt
from rrm.live_replan import CycleState, LiveCycleResponse, LiveReplanCycle
from rrm_cosmos_reason2 import load_context


ROOT = Path(__file__).parents[1]


def metadata(image: bytes, stamp: int = 100) -> dict:
    return {
        "capture_mode": "read_only",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "source_stamp_ns": stamp,
        "source_stamp_advanced": True,
        "frame_id": "camera_left",
        "sha256": hashlib.sha256(image).hexdigest(),
        "vehicle": {
            "connected": True,
            "odometry_frame_id": "map",
            "odometry_child_frame_id": "base_link",
            "odometry_stamp_ns": stamp,
            "x": 0.0, "y": 0.0, "z": 0.02, "linear_speed_m_s": 0.0,
        },
    }


def scene_state(stamp: int = 100) -> dict:
    return {"source_stamp_ns": stamp, "verified_entities": ["blue_marker", "orange_marker"],
            "provenance": "test-live-entity-verifier/v1"}


class RecordingProvider:
    def __init__(self, *, wrong_observation=False):
        self.requests = []
        self.wrong_observation = wrong_observation

    def propose(self, request):
        self.requests.append(request)
        raw = json.dumps({
            "status": "READY", "grounded_entities": ["blue_marker", "orange_marker"],
            "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
            "ambiguity_refs": [], "explanation": "two-step candidate for review only",
            "actions": [
                {"id": "blue", "verb": "NAVIGATE_TO", "targets": ["blue_marker"],
                 "dependencies": []},
                {"id": "orange", "verb": "NAVIGATE_TO", "targets": ["orange_marker"],
                 "dependencies": ["blue"]},
            ],
            "recovery_budget": 0,
        })
        candidate = parse_cosmos_candidate(raw, request.context)
        return LiveCycleResponse(
            cycle_id=request.cycle_id, step_index=request.step_index,
            observation_sha256="0" * 64 if self.wrong_observation else request.observation["sha256"],
            candidate=candidate,
        )


class LiveReplanCycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.bundle = Path(self.temp.name) / "bundle"
        self.bundle.mkdir()
        fixture = ROOT / "examples" / "office_visual_eval"
        for name in ("navigation_context.json", "scene_manifest.json"):
            shutil.copy(fixture / name, self.bundle / name)
        self.context = load_context(self.bundle / "navigation_context.json")
        self.cycle = LiveReplanCycle(
            Path(self.temp.name) / "cycle", task_id=self.context.task.task_id,
            task_revision=self.context.task.revision, expected_camera_frame="camera_left",
        )

    def test_multi_action_candidate_requires_per_step_review_outcome_and_recapture(self):
        provider = RecordingProvider()
        image = b"first-live-frame"
        self.cycle.record_observation(metadata(image), image, scene_state())
        next_action = self.cycle.request_next_action(self.context, provider)
        self.assertEqual(next_action["state"], CycleState.REVIEW_REQUIRED.value)
        self.assertEqual(next_action["next_action"]["action"]["id"], "blue")
        self.assertEqual(next_action["remaining_action_ids"], ["orange"])
        self.assertFalse(next_action["execution_dispatch"])
        with self.assertRaisesRegex(ValueError, "outcome"):
            self.cycle.record_observation(metadata(b"premature", 101), b"premature", scene_state(101))
        self.cycle.mark_reviewed("blue")
        self.assertEqual(self.cycle.phase, CycleState.OUTCOME_REQUIRED)
        self.cycle.record_outcome("blue", verified=True, detail="shadow observer confirmed step completion")
        self.assertEqual(self.cycle.phase, CycleState.AWAITING_OBSERVATION)
        second = b"second-live-frame"
        self.cycle.record_observation(metadata(second, 102), second, scene_state(102))
        self.cycle.request_next_action(self.context, provider)
        self.assertEqual(provider.requests[1].prior_outcome["action_id"], "blue")
        self.assertEqual(provider.requests[1].prior_plan_action_ids, ("blue", "orange"))
        self.assertTrue((Path(self.temp.name) / "cycle" / "steps" / "0000" / "review.json").is_file())
        self.assertTrue((Path(self.temp.name) / "cycle" / "steps" / "0000" / "outcome.json").is_file())

    def test_provider_response_cannot_cross_observations(self):
        image = b"live-frame"
        self.cycle.record_observation(metadata(image), image, scene_state())
        with self.assertRaisesRegex(ValueError, "not bound"):
            self.cycle.request_next_action(self.context, RecordingProvider(wrong_observation=True))
        self.assertEqual(self.cycle.phase, CycleState.AWAITING_PROVIDER)

    def test_cycle_reopens_only_with_the_same_task_and_camera_binding(self):
        image = b"live-frame"
        self.cycle.record_observation(metadata(image), image, scene_state())
        root = Path(self.temp.name) / "cycle"
        reopened = LiveReplanCycle(
            root, task_id=self.context.task.task_id, task_revision=self.context.task.revision,
            expected_camera_frame="camera_left",
        )
        self.assertEqual(reopened.phase, CycleState.AWAITING_PROVIDER)
        with self.assertRaisesRegex(ValueError, "different immutable task"):
            LiveReplanCycle(root, task_id="other-task", task_revision=self.context.task.revision,
                            expected_camera_frame="camera_left")
        with self.assertRaisesRegex(ValueError, "different immutable task"):
            LiveReplanCycle(root, task_id=self.context.task.task_id,
                            task_revision=self.context.task.revision,
                            expected_camera_frame="other_camera")

    def test_unverified_outcome_halts_without_next_capture(self):
        image = b"live-frame"
        self.cycle.record_observation(metadata(image), image, scene_state())
        self.cycle.request_next_action(self.context, RecordingProvider())
        self.cycle.mark_reviewed("blue")
        self.cycle.record_outcome("blue", verified=False, detail="target not confirmed")
        self.assertEqual(self.cycle.phase, CycleState.HALTED)
        with self.assertRaisesRegex(ValueError, "verified outcome"):
            self.cycle.record_observation(metadata(b"later", 101), b"later", scene_state(101))

    def test_wrong_frame_or_old_scene_state_is_rejected_before_provider(self):
        image = b"live-frame"
        bad_frame = metadata(image)
        bad_frame["frame_id"] = "other_camera"
        with self.assertRaisesRegex(ValueError, "frame"):
            self.cycle.record_observation(bad_frame, image, scene_state())
        with self.assertRaisesRegex(ValueError, "older"):
            self.cycle.record_observation(metadata(image, 100), image, scene_state(99))
        self.assertEqual(self.cycle.phase, CycleState.AWAITING_OBSERVATION)

    def test_source_has_no_execution_or_ros_control_surface(self):
        source = (ROOT / "rrm" / "live_replan.py").read_text(encoding="utf-8")
        for forbidden in ("rclpy", "mavros", "px4", "ActionClient", "subprocess"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
