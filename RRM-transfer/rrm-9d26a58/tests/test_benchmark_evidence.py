"""Core benchmark evidence must be complete and independently replayable."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import TASKS, run_suite
from rrm.benchmark_evidence import (
    MANIFEST_SCHEMA,
    METRICS_SCHEMA,
    REPLAY_SCHEMA,
    replay_trace,
)
from rrm.benchmark_labels import (
    EvaluationLabels,
    FailureKind,
    SafetyLabel,
    TerminalLabel,
)
from rrm.schema import RunMetrics
from rrm.trace import TRACE_EVENT_SCHEMA, Tracer


class EvaluationLabelTests(unittest.TestCase):
    def test_generic_label_record_round_trips_without_scene_fields(self) -> None:
        labels = EvaluationLabels(
            SafetyLabel.SAFE,
            SafetyLabel.UNSAFE,
            FailureKind.TRANSIENT_EFFECT,
            True,
            TerminalLabel.GOAL_VERIFIED,
        )
        record = labels.as_record()
        self.assertEqual(EvaluationLabels.from_record(record), labels)
        self.assertEqual(set(record), {
            "schema_version", "symbolic_safety", "numeric_safety",
            "failure_kind", "failure_recoverable", "expected_terminal",
        })

    def test_contradictory_and_scene_extended_labels_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "SafetyLabel vocabulary"):
            EvaluationLabels(  # type: ignore[arg-type]
                "SAFE", SafetyLabel.SAFE, FailureKind.NONE, None,
                TerminalLabel.GOAL_VERIFIED,
            )
        with self.assertRaisesRegex(ValueError, "without an injected failure"):
            EvaluationLabels(
                SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, True,
                TerminalLabel.GOAL_VERIFIED,
            )
        with self.assertRaisesRegex(ValueError, "persistent effect failure"):
            EvaluationLabels(
                SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.PERSISTENT_EFFECT, False,
                TerminalLabel.GOAL_VERIFIED,
            )
        record = EvaluationLabels(
            SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, None,
            TerminalLabel.GOAL_VERIFIED,
        ).as_record()
        with self.assertRaisesRegex(ValueError, "missing or unknown"):
            EvaluationLabels.from_record({**record, "scene": "office"})


class TraceContractTests(unittest.TestCase):
    def test_recovery_is_undefined_without_a_divergence(self) -> None:
        self.assertIsNone(RunMetrics().recovery_rate)
        self.assertEqual(RunMetrics(divergences=2, recoveries=1).recovery_rate, 0.5)

    def test_events_are_versioned_ordered_and_paths_are_immutable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": "T0"})
            tracer.event("sample", value=1)
            tracer.close()

            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([record["sequence"] for record in records], [0, 1])
            self.assertEqual([record["kind"] for record in records], ["run_start", "sample"])
            self.assertTrue(all(record["schema_version"] == TRACE_EVENT_SCHEMA
                                for record in records))
            self.assertLessEqual(records[0]["monotonic_time_s"],
                                 records[1]["monotonic_time_s"])
            with self.assertRaises(FileExistsError):
                Tracer(path, {"task_id": "replacement"})


class BenchmarkEvidenceTests(unittest.TestCase):
    def _run_suite(self, directory: str) -> Path:
        output = Path(directory) / "evidence"
        self.assertEqual(run_suite(output), 0)
        return output

    def test_suite_writes_replayable_checksum_bound_bundle(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = self._run_suite(directory)
            manifest = json.loads((output / "manifest.json").read_text())
            metrics = json.loads((output / "metrics.json").read_text())
            replay = json.loads((output / "replay-report.json").read_text())

            self.assertEqual(manifest["schema_version"], MANIFEST_SCHEMA)
            self.assertEqual(metrics["schema_version"], METRICS_SCHEMA)
            self.assertEqual(replay["schema_version"], REPLAY_SCHEMA)
            self.assertEqual(manifest["task_ids"], [task.id for task in TASKS])
            self.assertRegex(manifest["source"]["rrm_runtime_source_sha256"], r"^[0-9a-f]{64}$")
            self.assertGreater(manifest["source"]["rrm_runtime_source_file_count"], 1)
            self.assertTrue(replay["valid"])
            self.assertEqual(replay["valid_trace_count"], len(TASKS))
            self.assertEqual(metrics["task_success_rate"], {
                "numerator": len(TASKS), "denominator": len(TASKS), "value": 1.0,
            })
            self.assertFalse(metrics["performance_claim_authorized"])
            self.assertEqual(len(metrics["episodes"]), len(TASKS))
            self.assertGreater(metrics["planning_latency_ms"]["count"], len(TASKS))
            self.assertEqual(metrics["safety_verifier"]["confusion_matrix"], {
                "true_positive": 1,
                "false_positive": 0,
                "true_negative": 55,
                "false_negative": 0,
            })
            self.assertEqual(metrics["safety_verifier"]["recall"], {
                "numerator": 1, "denominator": 1, "value": 1.0,
            })
            self.assertEqual(metrics["safety_verifier"]["false_negative_rate"], {
                "numerator": 0, "denominator": 1, "value": 0.0,
            })
            self.assertEqual(metrics["failure_detection_rate"], {
                "numerator": 2, "denominator": 2, "value": 1.0,
            })
            self.assertEqual(metrics["recovery_success_rate"], {
                "numerator": 1, "denominator": 1, "value": 1.0,
            })

            for name, record in manifest["artifacts"].items():
                data = (output / name).read_bytes()
                self.assertEqual(record["bytes"], len(data))
                self.assertEqual(record["sha256"], hashlib.sha256(data).hexdigest())
            self.assertEqual(
                set(manifest["artifacts"]),
                {f"{task.id}.jsonl" for task in TASKS}
                | {"metrics.json", "replay-report.json"},
            )

    def test_truncation_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = self._run_suite(directory)
            source = output / "T1.jsonl"
            broken = output / "truncated.jsonl"
            lines = source.read_text().splitlines()
            broken.write_text("\n".join(lines[:-1]) + "\n")

            result = replay_trace(broken, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn("missing_episode_end", result.findings)
            self.assertIsNone(result.metrics)

    def test_terminal_counter_drift_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = self._run_suite(directory)
            events = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            events[-1]["action_count"] += 1
            broken = output / "counter-drift.jsonl"
            broken.write_text("".join(json.dumps(event) + "\n" for event in events))

            result = replay_trace(broken, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn("terminal_counter_mismatch:action_count", result.findings)
            self.assertIsNone(result.metrics)

    def test_reordered_sequence_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = self._run_suite(directory)
            events = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            events[1]["sequence"] = 99
            broken = output / "reordered.jsonl"
            broken.write_text("".join(json.dumps(event) + "\n" for event in events))

            result = replay_trace(broken, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertIn("non_contiguous_sequence:expected=1", result.findings)
            self.assertIsNone(result.metrics)

    def test_missing_or_scene_extended_labels_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = self._run_suite(directory)
            events = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]
            del events[0]["evaluation_labels"]
            missing = output / "missing-labels.jsonl"
            missing.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(missing, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertTrue(any(item.startswith("invalid_evaluation_labels:")
                                for item in result.findings))

            events[0]["evaluation_labels"] = {
                **EvaluationLabels(
                    SafetyLabel.SAFE, SafetyLabel.SAFE, FailureKind.NONE, None,
                    TerminalLabel.GOAL_VERIFIED,
                ).as_record(),
                "scene": "warehouse-shelves",
            }
            extended = output / "scene-extended-labels.jsonl"
            extended.write_text("".join(json.dumps(event) + "\n" for event in events))
            result = replay_trace(extended, expected_task_id="T1")
            self.assertFalse(result.valid)
            self.assertTrue(any(item.startswith("invalid_evaluation_labels:")
                                for item in result.findings))

    def test_convergence_payload_tampering_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = self._run_suite(directory)
            events = [json.loads(line) for line in (output / "T6.jsonl").read_text().splitlines()]
            convergence = next(event for event in events
                               if event["kind"] == "replan_convergence")
            convergence["replacement_action"]["params"] = {"invented": True}
            broken = output / "convergence-tampered.jsonl"
            broken.write_text("".join(json.dumps(event) + "\n" for event in events))

            result = replay_trace(broken, expected_task_id="T6")
            self.assertFalse(result.valid)
            self.assertIn("invalid_replan_convergence_unchanged_payload", result.findings)
            self.assertIsNone(result.metrics)

            events = [json.loads(line) for line in (output / "T6.jsonl").read_text().splitlines()]
            convergence = next(event for event in events
                               if event["kind"] == "replan_convergence")
            convergence["rejected_action_id"] = "unrelated-action"
            unbound = output / "convergence-unbound.jsonl"
            unbound.write_text("".join(json.dumps(event) + "\n" for event in events))

            result = replay_trace(unbound, expected_task_id="T6")
            self.assertFalse(result.valid)
            self.assertIn("invalid_replan_convergence_action_binding", result.findings)
            self.assertIsNone(result.metrics)


if __name__ == "__main__":
    unittest.main()
