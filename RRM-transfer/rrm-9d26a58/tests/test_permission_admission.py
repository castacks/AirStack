"""Core permission scope is explicit, immutable, and replayable."""

from dataclasses import replace
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import MOCK_CAPABILITIES, mock_permission, run_suite
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.contracts import PermissionDeclaration
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class PermissionAdmissionTests(unittest.TestCase):
    def test_permission_scope_is_immutable_and_reports_every_mismatch(self) -> None:
        operations = {"GRASP"}
        resources = {"manipulation"}
        permission = PermissionDeclaration(
            "policy", "permission-v1", "task-1", "mock_arm",
            frozenset(operations), frozenset(resources),
        )
        operations.add("PLACE")
        resources.clear()
        self.assertEqual(permission.operations, frozenset({"GRASP"}))
        self.assertEqual(permission.resources, frozenset({"manipulation"}))
        self.assertEqual(permission.rejection_reasons(
            task_id="task-2", embodiment_id="other", operation="PLACE",
            resources=frozenset({"mobility"}),
        ), (
            "task_not_permitted", "embodiment_not_permitted",
            "operation_not_permitted", "resource_not_permitted",
        ))
        with self.assertRaisesRegex(ValueError, "invalid permitted operations"):
            PermissionDeclaration(
                "policy", "permission-v1", "task-1", "mock_arm",
                "GRASP", frozenset({"manipulation"}),  # type: ignore[arg-type]
            )

    def test_denied_permission_aborts_before_safety_or_dispatch(self) -> None:
        task = Task(id="permission", mission="place item",
                    goal=_p("on", "obj_cup", "obj_table"), expect_abort=True)
        denied = replace(mock_permission(task.id), operations=frozenset({"WAIT"}),
                         resources=frozenset())
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
                    MockPolicy(), NumericSafetyVerifier(), tracer,
                    capabilities=MOCK_CAPABILITIES, permission=denied,
                )
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)

        gate = next(event for event in events if event["kind"] == "permission_gate")
        self.assertEqual(gate["verdict"], "DENY")
        self.assertEqual(
            gate["reasons"], ["operation_not_permitted", "resource_not_permitted"])
        self.assertTrue(metrics.aborted)
        self.assertEqual(metrics.action_count, 0)
        self.assertFalse(any(event["kind"] == "safety1" for event in events))
        self.assertTrue(replay.valid, replay.findings)

    def test_exact_permission_allows_and_tampering_fails_replay(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            path = output / "T1.jsonl"
            events = [json.loads(line) for line in path.read_text().splitlines()]
            gates = [event for event in events if event["kind"] == "permission_gate"]
            self.assertTrue(gates)
            self.assertTrue(all(event["verdict"] == "ALLOW" for event in gates))

            declaration = next(event for event in events
                               if event["kind"] == "permission_declaration")
            declaration["permission"]["task_id"] = "other-task"
            encoded = json.dumps(
                declaration["permission"], sort_keys=True, separators=(",", ":"),
            )
            declaration["permission_digest"] = hashlib.sha256(encoded.encode()).hexdigest()
            tampered = output / "tampered-permission.jsonl"
            tampered.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(tampered, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertTrue(any(finding.startswith("permission_reference_mismatch")
                                or finding.startswith("invalid_permission_gate_verdict")
                                for finding in result.findings))
            self.assertIsNone(result.metrics)

            events = [json.loads(line) for line in path.read_text().splitlines()]
            removed_sequence = next(event["sequence"] for event in events
                                    if event["kind"] == "permission_gate")
            events = [event for event in events
                      if event["sequence"] != removed_sequence]
            for sequence, event in enumerate(events):
                event["sequence"] = sequence
            tampered = output / "missing-permission-gate.jsonl"
            tampered.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(tampered, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn(
                f"capability_allow_without_permission:sequence={removed_sequence - 1}",
                result.findings,
            )
            self.assertIsNone(result.metrics)


if __name__ == "__main__":
    unittest.main()
