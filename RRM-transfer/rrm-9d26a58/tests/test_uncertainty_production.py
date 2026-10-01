"""Versioned aggregate uncertainty is derived, gated, and replay-checked."""

import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from pydantic import ValidationError

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission,
                           run_suite)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task, WorldObject, WorldState
from rrm.trace import Tracer
from rrm.uncertainty import aggregate_uncertainty, validate_uncertainty
from rrm.verbs import _p
from rrm.world import MockWorld


class WeakCupWorld(MockWorld):
    def observe(self):
        state = super().observe()
        state.get("obj_cup").confidence = 0.7
        state.uncertainty = aggregate_uncertainty(state)
        return state


class MissingCupTimestampWorld(MockWorld):
    def observe(self):
        state = super().observe()
        state.get("obj_cup").observed_t = None
        state.uncertainty = aggregate_uncertainty(state)
        return state


class UnboundAggregateWorld(MockWorld):
    def observe(self):
        state = super().observe()
        state.uncertainty = 0.0
        state.uncertainty_provenance = "unbound"
        return state


class StaleCupAfterChunkWorld(MockWorld):
    def observe(self):
        state = super().observe()
        if state.t >= 1:
            state.get("obj_cup").observed_t = state.t - 1
            state.uncertainty = aggregate_uncertainty(state)
        return state


class UncertaintyProductionTests(unittest.TestCase):
    def test_aggregate_confidence_freshness_and_bounds(self):
        fresh = MockWorld().observe()
        self.assertEqual(fresh.uncertainty, 0.0)
        validate_uncertainty(fresh)
        cup = fresh.get("obj_cup")
        cup.confidence = 0.65
        self.assertAlmostEqual(aggregate_uncertainty(fresh), 0.35)
        cup.observed_t = None
        self.assertEqual(aggregate_uncertainty(fresh), 1.0)
        cup.observed_t = fresh.t + 1
        self.assertEqual(aggregate_uncertainty(fresh), 1.0)
        fresh.t = 2
        self.assertEqual(aggregate_uncertainty(fresh), 1.0)
        empty = WorldState(t=0, uncertainty_provenance="object_evidence_v1",
                           uncertainty=1.0)
        validate_uncertainty(empty)
        with self.assertRaises(ValidationError):
            WorldObject(id="bad", cls="item", confidence=1.1)

    def _episode(self, world_type):
        task = Task(id="uncertainty-production", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=True)
        labels = EvaluationLabels(SafetyLabel.SAFE, SafetyLabel.SAFE,
                                  FailureKind.NONE, None, TerminalLabel.SAFE_ABORT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": True,
                                   "evaluation_labels": labels.as_record()})
            try:
                metrics = run(task, world_type(), ScriptedOracle(task.goal),
                              SafetyVerifier(), MockPolicy(),
                              NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES,
                              permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(),
                              admission=mock_admission())
            finally:
                tracer.close()
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
        return metrics, events, replay

    def test_evidence_controls_planning_gate(self):
        for world_type, expected in ((WeakCupWorld, 0.3),
                                     (MissingCupTimestampWorld, 1.0)):
            with self.subTest(world=world_type.__name__):
                metrics, events, replay = self._episode(world_type)
                gate = next(e for e in events if e["kind"] == "uncertainty_gate")
                self.assertAlmostEqual(gate["uncertainty"], expected)
                self.assertEqual(gate["verdict"], "FAIL")
                self.assertFalse(any(e["kind"] in {"plan", "apply"} for e in events))
                self.assertTrue(metrics.aborted)
                self.assertTrue(replay.valid, replay.findings)

    def test_unbound_aggregate_is_not_a_valid_observation(self):
        metrics, events, replay = self._episode(UnboundAggregateWorld)
        self.assertTrue(metrics.aborted)
        self.assertFalse(metrics.task_success)
        self.assertEqual(next(e for e in events if e["kind"] == "observation_failure")["phase"],
                         "planning")
        self.assertTrue(replay.valid, replay.findings)

    def test_stale_object_evidence_blocks_next_chunk(self):
        metrics, events, replay = self._episode(StaleCupAfterChunkWorld)
        gates = [e for e in events if e["kind"] == "uncertainty_gate"]
        self.assertEqual(gates[-1]["phase"], "dispatch")
        self.assertEqual(gates[-1]["uncertainty"], 1.0)
        self.assertEqual(gates[-1]["verdict"], "FAIL")
        self.assertEqual(len([e for e in events if e["kind"] == "apply"]), 1)
        self.assertEqual(next(e for e in events if e["kind"] == "dispatch")["termination"],
                         "INTERRUPTED")
        self.assertTrue(metrics.aborted)
        self.assertTrue(replay.valid, replay.findings)

    def test_replay_rejects_rehashed_scalar_and_provenance_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "suite"
            self.assertEqual(run_suite(source), 0)
            original = [json.loads(line) for line in (source / "T1.jsonl").read_text().splitlines()]
            for name in ("scalar", "tiny_scalar", "provenance", "confidence", "timestamp"):
                with self.subTest(name=name):
                    events = copy.deepcopy(original)
                    state_event = next(e for e in events if e["kind"] == "world_state")
                    old_digest = state_event["state_digest"]
                    state = state_event["state"]
                    if name == "scalar":
                        state["uncertainty"] = 0.25
                    elif name == "tiny_scalar":
                        state["uncertainty"] = 1e-13
                    elif name == "provenance":
                        state["uncertainty_provenance"] = "unbound"
                    elif name == "confidence":
                        state["objects"][1]["confidence"] = 0.6
                    else:
                        state["objects"][1]["observed_t"] = None
                    new_digest = hashlib.sha256(json.dumps(
                        state, sort_keys=True, separators=(",", ":")
                    ).encode()).hexdigest()
                    for event in events:
                        for field in ("state_digest", "before_state_digest"):
                            if event.get(field) == old_digest:
                                event[field] = new_digest
                    path = Path(directory) / f"{name}.jsonl"
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                    result = replay_trace(path, expected_task_id="T1")
                    self.assertFalse(result.valid)
                    self.assertTrue(any(f.startswith("uncertainty_evidence_mismatch")
                                        for f in result.findings), result.findings)


if __name__ == "__main__":
    unittest.main()
