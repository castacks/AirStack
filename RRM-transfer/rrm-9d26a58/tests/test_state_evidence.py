"""Decision evidence is bound to complete immutable world snapshots."""

import json
import tempfile
import unittest
from pathlib import Path

from rrm.benchmark import run_suite
from rrm.benchmark_evidence import replay_trace


class StateEvidenceTests(unittest.TestCase):
    def test_decisions_resolve_to_prior_world_state_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            events = [json.loads(line) for line in (output / "T8.jsonl").read_text().splitlines()]
            states = {}
            for event in events:
                if event["kind"] == "world_state":
                    states.setdefault(event["state_digest"], event["sequence"])
            bound = [event for event in events if event["kind"] in {
                "plan", "replan", "uncertainty_gate", "safety1", "safety2",
                "apply", "dispatch", "divergence", "episode_end",
            }]
            self.assertTrue(states)
            self.assertTrue(all(states[event["state_digest"]] < event["sequence"]
                                for event in bound))
            self.assertTrue(replay_trace(output / "T8.jsonl", expected_task_id="T8").valid)

    def test_state_payload_and_reference_tampering_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence"
            self.assertEqual(run_suite(output), 0)
            original = [json.loads(line) for line in (output / "T1.jsonl").read_text().splitlines()]

            for name, mutate, finding in (
                ("payload", lambda events: next(e for e in events if e["kind"] == "world_state")
                 ["state"].update({"uncertainty": 0.5}),
                 "world_state_digest_mismatch"),
                ("reference", lambda events: next(e for e in events if e["kind"] == "plan")
                 .update({"state_digest": "0" * 64}), "unknown_state_reference"),
            ):
                events = json.loads(json.dumps(original))
                mutate(events)
                path = output / f"tampered-{name}.jsonl"
                path.write_text("".join(json.dumps(event) + "\n" for event in events))
                result = replay_trace(path, expected_task_id="T1")
                self.assertFalse(result.valid)
                self.assertTrue(any(item.startswith(finding) for item in result.findings))
                self.assertIsNone(result.metrics)


if __name__ == "__main__":
    unittest.main()
