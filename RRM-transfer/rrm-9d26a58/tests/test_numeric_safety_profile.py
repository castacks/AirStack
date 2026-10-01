"""Typed numeric limits and their replayed evidence gate physical mock application."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from pydantic import ValidationError

from rrm.benchmark import (
    MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE, SyntheticApprovalProvider,
    run_suite, mock_admission, mock_permission,
)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.contracts import CapabilityDeclaration
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import AxisLimit, NumericLimitProfile, Task, Trajectory, WorldObject
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class NumericSafetyProfileTests(unittest.TestCase):
    def test_cartesian_limits_speed_and_segment_clearance(self) -> None:
        verifier = NumericSafetyVerifier(MOCK_NUMERIC_PROFILE)
        state = MockWorld().observe()
        traj = Trajectory(action_id="a", kind="cartesian_position", frame="mock_map",
                          position_unit="m", velocity_unit="m/s",
                          axes=("x", "y", "z"), waypoints=[(1.0, 0.0, 0.0)],
                          max_velocity=0.4)
        self.assertEqual(verifier.verify(traj, state).verdict, "PASS")
        state.objects.append(WorldObject(id="person", cls="person", pose=(0.5, 0.0, 0.0)))
        verdict = verifier.verify(traj, state)
        self.assertEqual(verdict.verdict, "FAIL")
        self.assertIn("human_clearance_path", [v.check for v in verdict.violations])

        state.objects.pop()
        outside = traj.model_copy(update={"waypoints": [(4.0, 0.0, 0.0)]})
        self.assertIn("axis_limits", [v.check for v in verifier.verify(outside, state).violations])
        fast = traj.model_copy(update={"max_velocity": 2.0})
        self.assertIn("velocity_limit", [v.check for v in verifier.verify(fast, state).violations])
        state.robot.base_pose = (float("nan"), 0.0, 0.0)
        self.assertIn("state_pose", [v.check for v in verifier.verify(traj, state).violations])

    def test_joint_coordinates_are_never_interpreted_as_cartesian(self) -> None:
        state = MockWorld().observe()
        joint_profile = NumericLimitProfile(
            ref="joint-v1", embodiment_id="mock_arm", kind="joint_position",
            frame="joint_state", position_unit="rad", velocity_unit="rad/s",
            axes=(AxisLimit(axis="j1", minimum=-1.0, maximum=1.0),),
            max_velocity=1.0,
        )
        joint = Trajectory(action_id="a", kind="joint_position", frame="joint_state",
                           position_unit="rad", velocity_unit="rad/s",
                           axes=("j1",), waypoints=[(0.2,)], max_velocity=0.4)
        self.assertEqual(NumericSafetyVerifier(joint_profile).verify(joint, state).verdict,
                         "PASS")
        self.assertEqual(NumericSafetyVerifier(MOCK_NUMERIC_PROFILE).verify(joint, state).verdict,
                         "FAIL")
        state.objects.append(WorldObject(id="person", cls="person", pose=(5.0, 0.0, 0.0)))
        self.assertIn("human_clearance_path", [v.check for v in
                      NumericSafetyVerifier(joint_profile).verify(joint, state).violations])

    def test_malformed_numeric_payloads_fail_validation(self) -> None:
        with self.assertRaises(TypeError):
            NumericSafetyVerifier(None)
        base = dict(action_id="a", kind="cartesian_position", frame="mock_map",
                    position_unit="m", velocity_unit="m/s",
                    axes=("x", "y", "z"), waypoints=[(0.0, 0.0, 0.0)], max_velocity=0.4)
        for change in ({"waypoints": [(0.0, 0.0)]},
                       {"waypoints": [(float("nan"), 0.0, 0.0)]},
                       {"max_velocity": float("inf")},
                       {"axes": ("x", "x", "z")},
                       {"position_unit": "rad"}):
            with self.subTest(change=change), self.assertRaises(ValidationError):
                Trajectory(**{**base, **change})
        with self.assertRaises(ValidationError):
            AxisLimit(axis="x", minimum=1.0, maximum=1.0)

    def test_capability_limit_reference_mismatch_halts_before_safety(self) -> None:
        task = Task(id="limits-mismatch", mission="pick cup", goal=_p("holding", "$self", "obj_cup"),
                    expect_abort=True)
        capability = CapabilityDeclaration(
            embodiment_id=MOCK_CAPABILITIES.embodiment_id,
            revision="wrong-limits-v1", operations=MOCK_CAPABILITIES.operations,
            resources=MOCK_CAPABILITIES.resources,
            available_resources=MOCK_CAPABILITIES.available_resources,
            limits_ref="different-profile-v1",
        )
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                result = run(task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                             MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                             capabilities=capability, permission=mock_permission(task.id),
                             approval=SyntheticApprovalProvider(), admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        gate = next(event for event in events if event["kind"] == "numeric_profile_gate")
        self.assertEqual(gate["reasons"], ["limits_ref_mismatch"])
        self.assertTrue(result.aborted)
        self.assertFalse(any(event["kind"] in {"safety1", "safety2", "apply"}
                             for event in events))
        self.assertTrue(replay.valid, replay.findings)

    def test_unsafe_policy_chunk_is_rejected_before_apply_and_replays(self) -> None:
        class UnsafePolicy(MockPolicy):
            def step(self, action, ws):
                traj = super().step(action, ws)
                return traj.model_copy(update={"waypoints": [(4.0, 0.0, 0.0)]})

        task = Task(id="numeric-reject", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.UNSAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                result = run(task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                             UnsafePolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                             capabilities=MOCK_CAPABILITIES, permission=mock_permission(task.id),
                             approval=SyntheticApprovalProvider(), admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        self.assertTrue(result.aborted)
        self.assertTrue(any(event["kind"] == "safety2" and event["verdict"] == "FAIL"
                            for event in events))
        self.assertFalse(any(event["kind"] == "apply" for event in events))
        self.assertTrue(replay.valid, replay.findings)

    def test_replay_recomputes_numeric_verdict_and_rejects_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            original = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            self.assertTrue(replay_trace(output / "T1.jsonl", expected_task_id="T1").valid)
            cases = (
                ("profile", "numeric_profile_declaration", "profile_digest", "0" * 64,
                 "numeric_profile_digest_mismatch"),
                ("trajectory", "safety2", "trajectory_digest", "0" * 64,
                 "numeric_trajectory_digest_mismatch"),
                ("verdict", "safety2", "verdict", "FAIL", "numeric_verdict_mismatch"),
                ("apply", "apply", "profile_digest", "0" * 64,
                 "apply_numeric_binding_mismatch"),
            )
            for name, kind, field, value, finding in cases:
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    next(event for event in events if event["kind"] == kind)[field] = value
                    path = output / f"tampered-{name}.jsonl"
                    path.write_text("".join(json.dumps(event) + "\n" for event in events))
                    replay = replay_trace(path, expected_task_id="T1")
                    self.assertFalse(replay.valid)
                    self.assertTrue(any(item.startswith(finding) for item in replay.findings),
                                    replay.findings)


if __name__ == "__main__":
    unittest.main()
