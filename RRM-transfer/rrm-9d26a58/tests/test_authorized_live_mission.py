"""Authorized live-mission composition tests; no worker, ROS, or dispatch runtime."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from rrm.airstack_drone import DroneOutcomeVerdict, DroneOutcomeVerification, MapWaypoint
from rrm.authorized_live_mission import AuthorizedLiveMission, VerifiedLiveStep
from rrm.continuous_replan import MissionAuthorization
from rrm.cosmos_reason2 import parse_cosmos_candidate
from rrm.drone_decision import AirStackDroneDecisionBridge, DroneNavigationTarget
from rrm.live_replan import LiveCycleResponse, LiveReplanCycle
from rrm_cosmos_reason2 import load_context


ROOT = Path(__file__).parents[1]


class Source:
    def __init__(self, context):
        self.context, self.stamp = context, 100

    def capture(self):
        image = f"verified-frame-{self.stamp}".encode()
        checksum = hashlib.sha256(image).hexdigest()
        metadata = {
            "capture_mode": "read_only", "captured_at": datetime.now(timezone.utc).isoformat(),
            "source_stamp_ns": self.stamp, "source_stamp_advanced": True,
            "frame_id": "camera_left", "sha256": checksum,
            "vehicle": {"connected": True, "odometry_frame_id": "map",
                        "odometry_child_frame_id": "base_link", "odometry_stamp_ns": self.stamp,
                        "x": 0.0, "y": 0.0, "z": 0.1, "linear_speed_m_s": 0.0},
        }
        scene = {"source_stamp_ns": self.stamp, "observation_sha256": checksum,
                 "verified_entities": ["blue_marker"], "provenance": "test-independent-verifier/v1"}
        self.stamp += 1
        return VerifiedLiveStep(metadata, image, scene, self.context)


class Provider:
    def __init__(self, *, target="blue_marker"):
        self.target = target

    def propose(self, request):
        raw = json.dumps({
            "status": "READY", "grounded_entities": [self.target],
            "grounded_goal": {"name": "near", "subject": "$self", "obj": self.target},
            "ambiguity_refs": [], "explanation": "test proposal",
            "actions": [{"id": f"action-{request.step_index}", "verb": "NAVIGATE_TO",
                         "targets": [self.target], "dependencies": []}], "recovery_budget": 0,
        })
        return LiveCycleResponse(request.cycle_id, request.step_index,
                                 request.observation["sha256"], parse_cosmos_candidate(raw, request.context))


class Dispatcher:
    def __init__(self, verdict=DroneOutcomeVerdict.VERIFIED):
        self.verdict, self.proposals = verdict, []

    def dispatch(self, proposal, evidence_dir):
        self.proposals.append(proposal)
        return DroneOutcomeVerification(
            task_id=proposal.task_id, action_id=proposal.action_id, kind=proposal.kind,
            action_success=True, action_message="independent test outcome",
            dispatch_monotonic_s=1.0, completion_monotonic_s=2.0, verdict=self.verdict,
            reasons=(), pre_odometry=None, post_odometry=None, post_vehicle_state=None,
        )


class AuthorizedLiveMissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        fixture = ROOT / "examples" / "office_visual_eval"
        context_path = Path(self.temp.name) / "context.json"
        shutil.copy(fixture / "navigation_context.json", context_path)
        self.context = load_context(context_path)
        self.cycle = LiveReplanCycle(Path(self.temp.name) / "cycle", task_id=self.context.task.task_id,
                                     task_revision=self.context.task.revision, expected_camera_frame="camera_left")
        bridge = AirStackDroneDecisionBridge(
            embodiment_id="aerial-eval", robot_name="robot_1",
            targets=(DroneNavigationTarget(entity_id="blue_marker",
                                           waypoints=(MapWaypoint(x=3.2, y=0.0, z=1.5),),
                                           goal_tolerance_m=0.3),),
        )
        self.compiler = lambda plan, context: bridge.compile_plan(
            plan, context.task, context.snapshot, context.capabilities,
            now_monotonic_s=context.now_monotonic_s)

    def mission(self, *, provider=None, dispatcher=None, max_actions=2):
        return AuthorizedLiveMission(
            self.cycle,
            MissionAuthorization(task_id=self.context.task.task_id, task_revision=self.context.task.revision,
                                 allowed_verbs=frozenset({"NAVIGATE_TO"}),
                                 allowed_targets=frozenset({"blue_marker"}), max_actions=max_actions),
            Source(self.context), provider or Provider(), self.compiler, dispatcher or Dispatcher())

    def test_authorized_verified_actions_replan_until_the_budget_is_exhausted(self):
        dispatcher = Dispatcher()
        mission = self.mission(dispatcher=dispatcher)
        self.assertEqual(mission.propose_initial_action()["state"], "REVIEW_REQUIRED")
        self.assertTrue(mission.approve_mission()["mission_authorized"])
        first = mission.dispatch_and_replan()
        self.assertEqual(first["state"], "OUTCOME_REQUIRED")
        self.assertEqual(first["next_action"]["id"], "action-1")
        final = mission.dispatch_and_replan()
        self.assertEqual(final["state"], "HALTED")
        self.assertEqual(len(dispatcher.proposals), 2)
        self.assertTrue((self.cycle.root / "steps" / "0000" / "compiled-proposal.json").is_file())
        self.assertTrue((self.cycle.root / "steps" / "0001" / "dispatch-outcome.json").is_file())

    def test_unverified_outcome_halts_without_a_second_dispatch_or_replan(self):
        dispatcher = Dispatcher(DroneOutcomeVerdict.MISMATCH)
        mission = self.mission(dispatcher=dispatcher)
        mission.propose_initial_action()
        mission.approve_mission()
        result = mission.dispatch_and_replan()
        self.assertEqual(result["state"], "HALTED")
        self.assertEqual(result["reason"], "action_outcome_not_verified")
        self.assertEqual(len(dispatcher.proposals), 1)
        self.assertFalse((self.cycle.root / "steps" / "0001").exists())

    def test_scope_or_current_entity_verification_failure_never_dispatches(self):
        dispatcher = Dispatcher()
        mission = self.mission(provider=Provider(target="orange_marker"), dispatcher=dispatcher)
        mission.propose_initial_action()
        with self.assertRaisesRegex(ValueError, "mission authorization|entity-verifier"):
            mission.approve_mission()
        self.assertEqual(dispatcher.proposals, [])

    def test_source_has_no_runtime_or_vehicle_control_surface(self):
        source = (ROOT / "rrm" / "authorized_live_mission.py").read_text(encoding="utf-8")
        for forbidden in ("import rclpy", "ActionClient", "mavros", "px4",
                          "import subprocess", "subprocess."):
            self.assertNotIn(forbidden.lower(), source.lower())


if __name__ == "__main__":
    unittest.main()
