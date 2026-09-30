"""Every traced action is identified by its plan version and planner-local ID."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import run_suite
from rrm.benchmark_evidence import replay_trace
from rrm.loop import _validate_graph_identity
from rrm.schema import AbstractAction, TaskGraph, Verb


def read_events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def write_events(path: Path, events: list[dict]) -> None:
    path.write_text("".join(json.dumps(event) + "\n" for event in events))


class ActionCorrelationTests(unittest.TestCase):
    def test_ambiguous_planner_identity_is_rejected_before_execution(self) -> None:
        duplicate = TaskGraph(
            mission_id="m0",
            mission_text="wait",
            nodes=[
                AbstractAction(id="same", verb=Verb.WAIT),
                AbstractAction(id="same", verb=Verb.WAIT),
            ],
        )
        with self.assertRaisesRegex(ValueError, "unique within a plan version"):
            _validate_graph_identity(duplicate, expected_version=0)

        wrong_version = duplicate.model_copy(update={
            "version": 2,
            "nodes": [AbstractAction(id="only", verb=Verb.WAIT)],
        })
        with self.assertRaisesRegex(ValueError, "does not match expected"):
            _validate_graph_identity(wrong_version, expected_version=1)

    def test_reused_local_ids_are_unambiguous_across_plan_versions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            events = read_events(output / "T8.jsonl")

            catalogs = {
                (event["version"], action["action_id"])
                for event in events if event["kind"] in {"plan", "replan"}
                for action in event["actions"]
            }
            self.assertIn((0, "a0"), catalogs)
            self.assertIn((1, "a0"), catalogs)

            lifecycle = [
                event for event in events
                if event["kind"] in {"safety1", "safety2", "apply", "dispatch", "divergence"}
            ]
            self.assertTrue(lifecycle)
            self.assertTrue(all(
                (event["plan_version"], event["action_id"]) in catalogs
                for event in lifecycle
            ))
            replan = next(event for event in events if event["kind"] == "replan")
            self.assertEqual(replan["trigger"]["plan_version"], 0)
            self.assertEqual(replan["trigger"]["action_id"], "a0")
            self.assertTrue(replay_trace(output / "T8.jsonl", expected_task_id="T8").valid)

    def test_tampered_action_correlations_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)

            cases = []
            t8 = read_events(output / "T8.jsonl")

            bad_lifecycle = copy.deepcopy(t8)
            next(event for event in bad_lifecycle if event["kind"] == "safety1")[
                "plan_version"
            ] = 99
            cases.append(("lifecycle", bad_lifecycle, "unknown_action_reference"))

            bad_catalog = copy.deepcopy(t8)
            next(event for event in bad_catalog if event["kind"] == "plan")["actions"][0][
                "params"
            ] = {"tampered": True}
            cases.append(("catalog", bad_catalog, "invalid_plan_action_digest"))

            bad_trigger = copy.deepcopy(t8)
            next(event for event in bad_trigger if event["kind"] == "replan")["trigger"][
                "plan_version"
            ] = 99
            cases.append(("trigger", bad_trigger, "unknown_replan_trigger"))

            t6 = read_events(output / "T6.jsonl")
            bad_convergence = copy.deepcopy(t6)
            next(event for event in bad_convergence
                 if event["kind"] == "replan_convergence")[
                "replacement_plan_version"
            ] = 99
            cases.append((
                "convergence", bad_convergence,
                "invalid_replan_convergence_replacement_reference",
            ))

            for name, events, expected_finding in cases:
                with self.subTest(name=name):
                    path = output / f"tampered-{name}.jsonl"
                    write_events(path, events)
                    result = replay_trace(
                        path, expected_task_id="T6" if name == "convergence" else "T8",
                    )
                    self.assertFalse(result.valid)
                    self.assertTrue(any(
                        finding.startswith(expected_finding) for finding in result.findings
                    ), result.findings)
                    self.assertIsNone(result.metrics)


if __name__ == "__main__":
    unittest.main()
