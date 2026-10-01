"""Aggregate world uncertainty gates every autonomous core transition."""

import json
import tempfile
import unittest
from pathlib import Path

from pydantic import ValidationError

from rrm.benchmark_evidence import replay_trace
from rrm.benchmark import (
    MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE, SyntheticApprovalProvider,
    mock_admission, mock_permission,
)
from rrm.benchmark_labels import (
    EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel,
)
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task, WorldState
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class ScheduledUncertaintyWorld(MockWorld):
    def __init__(self, schedule: dict[int, float], default: float = 0.0) -> None:
        super().__init__()
        self.schedule = schedule
        self.default = default
        self.observations = 0

    def observe(self) -> WorldState:
        self.observations += 1
        state = super().observe()
        state.uncertainty = self.schedule.get(self.observations, self.default)
        return state


def task(*, expect_abort: bool = False) -> Task:
    return Task(
        id="uncertainty", mission="place item", goal=_p("on", "obj_cup", "obj_table"),
        expect_abort=expect_abort,
    )


def execute(world: MockWorld, *, threshold: float = 0.0, trace: Path | None = None,
            expect_abort: bool = False):
    current_task = task(expect_abort=expect_abort)
    labels = EvaluationLabels(
        SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, None,
        TerminalLabel.SAFE_ABORT if expect_abort else TerminalLabel.GOAL_VERIFIED,
    )
    tracer = Tracer(trace, {
        "task_id": current_task.id,
        "expect_abort": current_task.expect_abort,
        "evaluation_labels": labels.as_record(),
    })
    try:
        return run(
            current_task, world, ScriptedOracle(current_task.goal), SafetyVerifier(),
            MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
            uncertainty_threshold=threshold,
            capabilities=MOCK_CAPABILITIES,
            permission=mock_permission(current_task.id),
            approval=SyntheticApprovalProvider(),
            admission=mock_admission(),
        )
    finally:
        tracer.close()


class UncertaintyGateTests(unittest.TestCase):
    def test_initial_breach_aborts_before_reasoning_and_replays(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "initial.jsonl"
            metrics = execute(
                ScheduledUncertaintyWorld({1: 0.1}), trace=path, expect_abort=True,
            )
            events = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(metrics.aborted)
            self.assertEqual(metrics.replans, 0)
            self.assertEqual(metrics.action_count, 0)
            self.assertFalse(any(event["kind"] == "plan" for event in events))
            gate = next(event for event in events if event["kind"] == "uncertainty_gate")
            self.assertEqual(gate["verdict"], "FAIL")
            self.assertTrue(replay_trace(path, expected_task_id="uncertainty").valid)

            gate["verdict"] = "PASS"
            tampered = Path(directory) / "tampered.jsonl"
            tampered.write_text("".join(json.dumps(event) + "\n" for event in events))
            replay = replay_trace(tampered, expected_task_id="uncertainty")
            self.assertFalse(replay.valid)
            self.assertIsNone(replay.metrics)
            self.assertTrue(any(finding.startswith("invalid_uncertainty_gate_verdict")
                                for finding in replay.findings))

    def test_pre_action_breach_prevents_safety_and_dispatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pre-action.jsonl"
            metrics = execute(
                ScheduledUncertaintyWorld({2: 0.1}), trace=path, expect_abort=True,
            )
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id="uncertainty")
        self.assertTrue(metrics.aborted)
        self.assertEqual(metrics.action_count, 0)
        failure = next(event for event in events
                       if event["kind"] == "uncertainty_gate"
                       and event["verdict"] == "FAIL")
        self.assertEqual(failure["phase"], "pre_action")
        self.assertFalse(any(event["kind"] in {"safety1", "dispatch"}
                             for event in events[failure["sequence"] + 1:]))
        self.assertTrue(replay.valid, replay.findings)

    def test_mid_dispatch_breach_prevents_another_policy_application(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mid-dispatch.jsonl"
            metrics = execute(
                ScheduledUncertaintyWorld({4: 0.1}), trace=path, expect_abort=True,
            )
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id="uncertainty")
        self.assertTrue(metrics.aborted)
        self.assertEqual(metrics.action_count, 1)
        self.assertEqual(metrics.inner_cycles, 1)
        self.assertEqual(sum(event["kind"] == "apply" for event in events), 1)
        dispatch = next(event for event in events if event["kind"] == "dispatch")
        self.assertEqual(dispatch["termination"], "UNCERTAIN")
        self.assertTrue(replay.valid, replay.findings)

    def test_threshold_is_inclusive_and_inputs_are_bounded(self) -> None:
        metrics = execute(ScheduledUncertaintyWorld({}, default=0.2), threshold=0.2)
        self.assertTrue(metrics.task_success)
        for threshold in (-0.1, 1.1, float("nan")):
            with self.subTest(threshold=threshold), self.assertRaisesRegex(
                    ValueError, "within \\[0,1\\]"):
                execute(MockWorld(), threshold=threshold)
        with self.assertRaises(ValidationError):
            WorldState(uncertainty=1.1)


if __name__ == "__main__":
    unittest.main()
