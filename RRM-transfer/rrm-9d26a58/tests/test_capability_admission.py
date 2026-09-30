"""Core dispatch requires an immutable semantic capability declaration."""

import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import MOCK_CAPABILITIES, mock_permission
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.contracts import CapabilityDeclaration
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task, Verb
from rrm.trace import Tracer
from rrm.verbs import VERB_TABLE, _p
from rrm.world import MockWorld


class CapabilityAdmissionTests(unittest.TestCase):
    def test_every_verb_has_an_authored_resource_set(self) -> None:
        self.assertEqual(set(VERB_TABLE), set(Verb))
        expected = {
            Verb.LOCATE: {"perception"},
            Verb.TAKEOFF: {"mobility"},
            Verb.LAND: {"mobility"},
            Verb.EXPLORE: {"mobility", "perception"},
            Verb.NAVIGATE_TO: {"mobility"},
            Verb.GRASP: {"manipulation"},
            Verb.RELEASE: {"manipulation"},
            Verb.PLACE: {"manipulation"},
            Verb.OPEN: {"manipulation"},
            Verb.CLOSE: {"manipulation"},
            Verb.INSPECT: {"perception"},
            Verb.WAIT: set(),
            Verb.ABORT: set(),
        }
        self.assertEqual(
            {verb: set(spec.required_resources) for verb, spec in VERB_TABLE.items()},
            expected,
        )

    def test_unsupported_operation_aborts_before_safety_or_dispatch(self) -> None:
        task = Task(id="capability", mission="place item",
                    goal=_p("on", "obj_cup", "obj_table"), expect_abort=True)
        denied = CapabilityDeclaration(
            "mock_arm", "limited-v1", frozenset({"WAIT"}), frozenset(),
            frozenset(), "limits-v1",
        )
        labels = EvaluationLabels(
            SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, None,
            TerminalLabel.SAFE_ABORT,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(
                    task, MockWorld(), ScriptedOracle(task.goal), SafetyVerifier(),
                    MockPolicy(), NumericSafetyVerifier(), tracer, capabilities=denied,
                    permission=mock_permission(task.id),
                )
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)

        gate = next(event for event in events if event["kind"] == "capability_gate")
        self.assertEqual(gate["verdict"], "DENY")
        self.assertEqual(
            gate["reasons"], ["unsupported_operation", "undeclared_resource"])
        self.assertTrue(metrics.aborted)
        self.assertEqual(metrics.action_count, 0)
        self.assertFalse(any(event["kind"] == "safety1" for event in events))
        self.assertTrue(replay.valid, replay.findings)

    def test_unavailable_resource_denies_before_dispatch(self) -> None:
        current = Task(id="resource", mission="place item",
                       goal=_p("on", "obj_cup", "obj_table"), expect_abort=True)
        capability = CapabilityDeclaration(
            "mock_arm", "unavailable-v1", frozenset({"GRASP", "PLACE"}),
            frozenset({"manipulation"}), frozenset(), "limits-v1",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                      FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
            tracer = Tracer(path, {"task_id": current.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(current, MockWorld(), ScriptedOracle(current.goal),
                              SafetyVerifier(), MockPolicy(), NumericSafetyVerifier(),
                              tracer, capabilities=capability,
                              permission=mock_permission(current.id))
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=current.id)
        gate = next(event for event in events if event["kind"] == "capability_gate")
        self.assertEqual(gate["required_resources"], ["manipulation"])
        self.assertEqual(gate["reasons"], ["unavailable_resource"])
        self.assertEqual(metrics.action_count, 0)
        self.assertTrue(replay.valid, replay.findings)

    def test_undeclared_resource_denies_before_dispatch(self) -> None:
        current = Task(id="resource", mission="place item",
                       goal=_p("on", "obj_cup", "obj_table"), expect_abort=True)
        capability = CapabilityDeclaration(
            "mock_arm", "undeclared-v1", frozenset({"GRASP", "PLACE"}),
            frozenset(), frozenset(), "limits-v1",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                      FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
            tracer = Tracer(path, {"task_id": current.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(current, MockWorld(), ScriptedOracle(current.goal),
                              SafetyVerifier(), MockPolicy(), NumericSafetyVerifier(),
                              tracer, capabilities=capability,
                              permission=mock_permission(current.id))
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=current.id)
        gate = next(event for event in events if event["kind"] == "capability_gate")
        self.assertEqual(gate["required_resources"], ["manipulation"])
        self.assertEqual(gate["reasons"], ["undeclared_resource"])
        self.assertEqual(metrics.action_count, 0)
        self.assertTrue(replay.valid, replay.findings)

    def test_supported_operations_and_tamper_detection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            from rrm.benchmark import run_suite
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            path = output / "T1.jsonl"
            events = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(all(event["verdict"] == "ALLOW" for event in events
                                if event["kind"] == "capability_gate"))

            declaration = next(event for event in events
                               if event["kind"] == "capability_declaration")
            declaration["capability"]["operations"] = ["WAIT"]
            tampered = output / "tampered-capability.jsonl"
            tampered.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(tampered, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn("capability_declaration_digest_mismatch", result.findings)
            self.assertIsNone(result.metrics)

            events = [json.loads(line) for line in path.read_text().splitlines()]
            gate = next(event for event in events if event["kind"] == "capability_gate")
            gate["required_resources"] = []
            tampered = output / "tampered-resource.jsonl"
            tampered.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(tampered, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn(
                f"capability_resource_mismatch:sequence={gate['sequence']}",
                result.findings,
            )
            self.assertIsNone(result.metrics)

            events = [json.loads(line) for line in path.read_text().splitlines()]
            removed_sequence = next(
                event["sequence"] for event in events
                if event["kind"] == "capability_gate"
            )
            events = [event for event in events
                      if event["sequence"] != removed_sequence]
            for sequence, event in enumerate(events):
                event["sequence"] = sequence
            tampered = output / "missing-capability-gate.jsonl"
            tampered.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(tampered, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn(
                f"missing_capability_gate:sequence={removed_sequence - 1}",
                result.findings,
            )
            self.assertIsNone(result.metrics)

        self.assertIn("GRASP", MOCK_CAPABILITIES.operations)


if __name__ == "__main__":
    unittest.main()
