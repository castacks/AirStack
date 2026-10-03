"""Bounded, isolated proposal callbacks and independently replayed failures."""

import copy
import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from threading import Event

from rrm.benchmark import (MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE,
                           SyntheticApprovalProvider, mock_admission, mock_permission)
from rrm.benchmark_evidence import replay_trace
from rrm.benchmark_labels import EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel
from rrm.core_admission import CoreAdmission
from rrm.core_deadlines import CoreCallLimits
from rrm.loop import run
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import Task
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


class ProbeOracle(ScriptedOracle):
    def __init__(self, goal, phase, mode, task, admission):
        super().__init__(goal)
        self.phase, self.mode, self.task, self.admission = phase, mode, task, admission
        self.release, self.finished = Event(), Event()
        self.copied_graph = None

    def probe(self, phase, mission, state, graph=None, div=None):
        planning_state = state.model_copy(deep=True)
        if phase == self.phase:
            if self.mode == "stall":
                if not self.release.wait(5):
                    raise RuntimeError("fixture worker not released")
            elif self.mode == "exception":
                raise RuntimeError("reasoner failed")
            elif self.mode == "base_exception":
                raise SystemExit("reasoner terminated")
            elif self.mode == "invalid":
                return {"not": "a graph"}
            elif self.mode == "task":
                self.task.goal = _p("on", "obj_cup", "obj_table")
            elif self.mode == "stop":
                self.admission.guard.stop()
            elif self.mode == "input":
                state.get("obj_cup").pose = (99, 99, 99)
                if graph is not None:
                    self.copied_graph = graph
                    graph.nodes.reverse()
                    div.unmet.clear()
        candidate = (super().plan(mission, planning_state) if graph is None else
                     super().replan(mission, planning_state, graph, div))
        if phase == self.phase:
            if self.mode == "version":
                candidate = candidate.model_copy(update={"version": 99})
            elif self.mode == "identity":
                candidate.mission_id = ""
            elif self.mode == "shape":
                candidate.nodes[0].targets = [17]
        self.output = candidate
        return candidate

    def plan(self, mission, state):
        try:
            return self.probe("plan", mission, state)
        finally:
            if self.phase == "plan":
                self.finished.set()

    def replan(self, mission, state, graph, div):
        try:
            return self.probe("replan", mission, state, graph, div)
        finally:
            if self.phase == "replan":
                self.finished.set()


class CoreReasonerLifecycleTests(unittest.TestCase):
    export_count = 0

    def episode(self, phase="plan", mode="exception"):
        successful = mode == "input" or mode.startswith("output_")
        task = Task(id="reasoner-lifecycle", mission="pick cup",
                    goal=_p("holding", "$self", "obj_cup"), expect_abort=not successful)
        original = mock_admission()
        admission = CoreAdmission(original.guard, original.provider, original.evidence_kind,
                                  call_limits=CoreCallLimits(reasoner_s=0.08))
        oracle = ProbeOracle(task.goal, phase, mode, task, admission)
        world = MockWorld(fail_grasp_once=phase == "replan")
        labels = EvaluationLabels(
            SafetyLabel.SAFE, SafetyLabel.SAFE,
            FailureKind.TRANSIENT_EFFECT if phase == "replan" else FailureKind.NONE,
            True if phase == "replan" else None,
            TerminalLabel.GOAL_VERIFIED if successful else TerminalLabel.SAFE_ABORT)
        class MutatingPolicy(MockPolicy):
            def step(self, action, state):
                trajectory = super().step(action, state)
                if mode == "output_targets":
                    oracle.output.nodes[0].targets = ["obj_table"]
                elif mode == "output_version":
                    oracle.output.version = 99
                return trajectory
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            tracer = Tracer(path, {"task_id": task.id, "expect_abort": task.expect_abort,
                                  "evaluation_labels": labels.as_record()})
            started = time.monotonic()
            try:
                metrics = run(task, world, oracle, SafetyVerifier(), MutatingPolicy(),
                              NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                              capabilities=MOCK_CAPABILITIES, permission=mock_permission(task.id),
                              approval=SyntheticApprovalProvider(), admission=admission)
                elapsed = time.monotonic() - started
                before_release = path.read_bytes()
            finally:
                oracle.release.set()
                self.assertTrue(oracle.finished.wait(1))
                tracer.close()
            self.assertEqual(before_release, path.read_bytes(), "late proposal wrote execution evidence")
            events = [json.loads(line) for line in path.read_text().splitlines()]
            replay = replay_trace(path, expected_task_id=task.id)
            export = os.environ.get("RRM_REASONER_EVIDENCE_DIR")
            if export:
                output = Path(export)
                output.mkdir(parents=True, exist_ok=True)
                type(self).export_count += 1
                stem = f"{self.export_count:02d}-{phase}-{mode}"
                with (output / f"{stem}.jsonl").open("xb") as artifact:
                    artifact.write(path.read_bytes())
                with (output / f"{stem}.json").open("x") as artifact:
                    json.dump({"phase": phase, "mode": mode, "elapsed_s": elapsed,
                               "metrics": metrics.model_dump(), "replay_valid": replay.valid,
                               "findings": list(replay.findings), "trace_sha256": replay.trace_sha256}, artifact)
        return metrics, events, replay, elapsed, oracle

    def test_failures_and_late_results_are_terminal_and_unsuccessful(self):
        for phase in ("plan", "replan"):
            for mode in ("stall", "exception", "base_exception", "invalid", "version",
                         "identity", "shape", "task", "stop"):
                with self.subTest(phase=phase, mode=mode):
                    metrics, events, replay, elapsed, _ = self.episode(phase, mode)
                    self.assertLess(elapsed, 1.5)
                    self.assertTrue(metrics.aborted)
                    self.assertFalse(metrics.task_success)
                    self.assertEqual(metrics.replans, int(phase == "replan"))
                    self.assertEqual(metrics.action_count, int(phase == "replan"))
                    failure = next(event for event in events if event["kind"] == "reasoner_failure")
                    self.assertEqual(events[failure["sequence"] + 1:][-1]["kind"], "episode_end")
                    self.assertEqual(events[-1]["terminal_observation"], "UNAVAILABLE")
                    self.assertEqual(events[-1]["stop_status"], "NOT_REQUESTED")
                    self.assertFalse(any(e["kind"] == "stop_request" for e in events))
                    if mode == "stall":
                        self.assertEqual(failure["deadline"]["call"], "reasoner_" + phase)
                    self.assertTrue(replay.valid, replay.findings)

    def test_reasoner_input_mutation_does_not_corrupt_observed_world_or_old_plan(self):
        for phase in ("plan", "replan"):
            with self.subTest(phase=phase):
                # Expect-abort flag is immaterial here: inspect verified goal directly.
                metrics, events, replay, _, oracle = self.episode(phase, "input")
                self.assertTrue(metrics.task_success)
                self.assertTrue(replay.valid, replay.findings)
                self.assertTrue(events[-1]["goal_met"])
                self.assertFalse(any(e["kind"] == "reasoner_failure" for e in events))
                observed = [e["state"] for e in events if e["kind"] == "world_state"]
                self.assertFalse(any(obj["pose"] == [99, 99, 99]
                                     for state in observed for obj in state["objects"]))
                if phase == "replan":
                    self.assertIsNotNone(oracle.copied_graph)

    def test_replay_rejects_forged_proposal_lifecycle(self):
        for phase in ("plan", "replan"):
            _, original, replay, _, _ = self.episode(phase, "stall")
            self.assertTrue(replay.valid, replay.findings)
            for mode in ("request", "state", "task", "phase", "deadline", "duration",
                         "missing_request", "missing_failure", "success", "duplicate", "trigger",
                         "generation", "trigger_payload", "previous_plan", "old_limits"):
                if mode in {"trigger", "trigger_payload", "previous_plan"} and phase == "plan":
                    continue
                with self.subTest(phase=phase, tamper=mode), tempfile.TemporaryDirectory() as directory:
                    events = copy.deepcopy(original)
                    request = [e for e in events if e["kind"] == "reasoner_request"][-1]
                    failure = next(e for e in events if e["kind"] == "reasoner_failure")
                    if mode == "request":
                        failure["request_id"] = "detached"
                    elif mode == "state":
                        failure["state_digest"] = "0" * 64
                    elif mode == "task":
                        request["task_digest"] = "0" * 64
                    elif mode == "phase":
                        failure["phase"] = "other"
                    elif mode == "deadline":
                        failure["deadline"]["timeout_s"] = 5
                    elif mode == "duration":
                        failure["latency_ms"] = 0
                    elif mode == "missing_request":
                        events.remove(request)
                    elif mode == "missing_failure":
                        events.remove(failure)
                    elif mode == "success":
                        events[-1]["task_success"] = True
                    elif mode == "duplicate":
                        events.insert(failure["sequence"], copy.deepcopy(failure))
                    elif mode == "generation":
                        request["stop_generation"] = 999
                    elif mode == "trigger_payload":
                        request["trigger"]["unmet"] = []
                        request["trigger"]["magnitude"] = 0
                    elif mode == "previous_plan":
                        request["previous_plan_digest"] = "0" * 64
                    elif mode == "old_limits":
                        limits = next(e for e in events if e["kind"] == "call_limits_declaration")["limits"]
                        limits["version"] = "core-call-limits/v1"
                        del limits["reasoner_s"]
                    else:
                        request["trigger"]["plan_version"] = 99
                    for sequence, event in enumerate(events):
                        event["sequence"] = sequence
                    path = Path(directory) / "tampered.jsonl"
                    path.write_text("".join(json.dumps(e) + "\n" for e in events))
                    self.assertFalse(replay_trace(path, expected_task_id="reasoner-lifecycle").valid)

    def test_reasoner_retained_output_cannot_mutate_accepted_plan(self):
        for phase in ("plan", "replan"):
            for mode in ("output_targets", "output_version"):
                with self.subTest(phase=phase, mode=mode):
                    metrics, events, replay, _, _ = self.episode(phase, mode)
                    self.assertTrue(metrics.task_success)
                    self.assertTrue(events[-1]["goal_met"])
                    self.assertFalse(any(e["kind"] == "reasoner_failure" for e in events))
                    self.assertTrue(replay.valid, replay.findings)


if __name__ == "__main__":
    unittest.main()
