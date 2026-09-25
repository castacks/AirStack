"""Transport-free C06/C08/C09 negative tests; no simulator or robot is touched."""

from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from rrm.contracts import DispatchContext, SafetyDecision
from rrm.hand_execution_boundary import (
    BoundaryError, DurableJournal, HandCommand, HandExecutionBoundary, SafeStateEvidence,
)


def encoded(value):
    return json.dumps(value, sort_keys=True).encode()


class FakeAdapter:
    def __init__(self):
        self.calls = []
        self.stops = []
        self.safe = True
        self.ack = True

    def submit(self, dispatch_id, generation, command):
        self.calls.append((dispatch_id, generation, command))

    def request_stop(self, generation):
        self.stops.append(generation)
        return self.ack

    def safe_state(self, generation):
        return SafeStateEvidence("adapter-safe-1", "episode-1", generation, 10.0, self.safe,
                                 "POSITION_HOLD", False, 0.01, 0.001, 5)


class FailingJournal(DurableJournal):
    def append(self, event, payload):
        if event == "C09_DISPATCH_INTENT":
            raise OSError("disk full")
        return super().append(event, payload)


class StopFailJournal(DurableJournal):
    def append(self, event, payload):
        if event == "C08_STOP_RECEIVED":
            raise OSError("disk full")
        return super().append(event, payload)


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "trace.jsonl"
        self.probe = {"scene_recipe_sha256": "scene-1",
                      "joint_limits": [{"name": "iiwa7_joint_1", "lower_rad": -1.0,
                                        "upper_rad": 1.0, "max_velocity_rad_s": 1.0}],
                      "safe_state": {"joint_velocity_threshold_rad_s": 0.10,
                                     "object_velocity_threshold_m_s": 0.01,
                                     "consecutive_window_required": 5}}
        self.probe_bytes = encoded(self.probe)
        self.qualification = {"schema_version": "rrm-hand-qualification/v1",
            "probe_sha256": hashlib.sha256(self.probe_bytes).hexdigest(),
            "scene_recipe_sha256": "scene-1", "gates": {k: {"status": "PASS"} for k in
                ("controller_limits", "post_command_reset", "contact_observer",
                 "safe_state", "independent_stop")},
            "ready_for_single_bounded_contact_trial": True, "execution_dispatch": False}
        self.qualification_bytes = encoded(self.qualification)
        self.adapter = FakeAdapter()
        self.boundary = self.make_boundary()
        self.command = HandCommand("iiwa7_joint_1", 0.0, 0.01, 10.0, "episode-1",
            "scene-1", self.qualification["probe_sha256"],
            hashlib.sha256(self.qualification_bytes).hexdigest())

    def make_boundary(self, journal=None):
        return HandExecutionBoundary(adapter=self.adapter,
            journal=journal or DurableJournal(self.path),
            qualification_bytes=self.qualification_bytes, probe_bytes=self.probe_bytes)

    def context(self, boundary=None, dispatch_id="dispatch-1"):
        boundary = boundary or self.boundary
        return DispatchContext("run-1", "task-1", "plan-1", "calibration-1",
            dispatch_id, self.command.digest, "state-1", "capability-1",
            "permission-1", "approval-1", "constraints-1", boundary.epoch,
            boundary.generation)

    def decision(self, context=None):
        return SafetyDecision("decision-1", context or self.context(), "ALLOW", 9.0, 11.0)

    def test_initial_inhibit_and_exact_one_use_with_durable_intent(self):
        with self.assertRaisesRegex(BoundaryError, "admission_closed"):
            self.boundary.dispatch(self.decision(), self.context(), self.command, now=10.0)
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        context = self.context()
        self.boundary.dispatch(self.decision(context), context, self.command, now=10.0)
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual(records[-1]["event"], "C09_DISPATCH_INTENT")
        self.assertEqual(len(self.adapter.calls), 1)
        with self.assertRaises(BoundaryError):
            self.boundary.dispatch(self.decision(context), context, self.command, now=10.0)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_bad_command_or_decision_never_reaches_adapter(self):
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        context = self.context()
        cases = [replace(self.command, operation="GRASP"),
                 replace(self.command, target_position_rad=0.5),
                 replace(self.command, observed_at_monotonic=9.0),
                 replace(self.command, episode_id=""),
                 replace(self.command, episode_id="old-episode"),
                 replace(self.command, probe_sha256="wrong")]
        for command in cases:
            with self.subTest(command=command):
                with self.assertRaises(BoundaryError):
                    self.boundary.dispatch(self.decision(context), context, command, now=10.0)
        for decision in [replace(self.decision(context), verdict="DENY"),
                         replace(self.decision(context), expires_at=10.0)]:
            with self.assertRaises(BoundaryError):
                self.boundary.dispatch(decision, context, self.command, now=10.0)
        self.assertEqual(self.adapter.calls, [])

    def test_journal_failure_inhibits_before_adapter(self):
        self.boundary = self.make_boundary(FailingJournal(self.path))
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        context = self.context()
        with self.assertRaisesRegex(BoundaryError, "journal_unavailable"):
            self.boundary.dispatch(self.decision(context), context, self.command, now=10.0)
        self.assertEqual(self.adapter.calls, [])
        self.assertTrue(self.boundary.inhibited)

    def test_stop_invalidation_safe_evidence_and_restart(self):
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        context = self.context()
        self.boundary.dispatch(self.decision(context), context, self.command, now=10.0)
        self.adapter.ack = False
        self.assertFalse(self.boundary.stop(intervention_id="stop-1", reason="test"))
        self.assertTrue(self.boundary.inhibited)
        self.assertFalse(self.boundary.safe_confirmed)
        self.assertFalse(self.boundary.reset(authorized=True, generation=2, now=10.0))
        self.adapter.safe = False
        self.assertFalse(self.boundary.confirm_stopped(generation=2, now=10.0))
        self.adapter.safe = True
        self.assertTrue(self.boundary.confirm_stopped(generation=2, now=10.0))
        self.assertFalse(self.boundary.reset(authorized=False, generation=2, now=10.0))
        self.assertFalse(self.boundary.reset(authorized=True, generation=2, now=11.0))
        self.assertTrue(self.boundary.reset(authorized=True, generation=2, now=10.0))
        with self.assertRaises(BoundaryError):
            self.boundary.dispatch(self.decision(context), context, self.command, now=10.0)
        restarted = self.make_boundary()
        self.assertTrue(restarted.inhibited)
        self.assertNotEqual(restarted.epoch, self.boundary.epoch)
        self.assertFalse(restarted.reset(authorized=True, generation=0, now=10.0))

    def test_tampered_journal_refused(self):
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        self.path.write_text(self.path.read_text().replace("C08_RESET", "C08_FAKE"))
        with self.assertRaisesRegex(BoundaryError, "journal_chain_invalid"):
            self.make_boundary()

    def test_concurrent_duplicate_has_one_submit(self):
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        context = self.context()
        def attempt(_):
            try:
                self.boundary.dispatch(self.decision(context), context, self.command, now=10.0)
                return True
            except BoundaryError:
                return False
        with ThreadPoolExecutor(max_workers=8) as pool:
            self.assertEqual(sum(pool.map(attempt, range(8))), 1)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_stop_delivered_even_if_journal_fails_and_reset_inhibited(self):
        self.boundary = self.make_boundary(StopFailJournal(self.path))
        self.assertTrue(self.boundary.reset(authorized=True, generation=0, now=10.0))
        self.assertTrue(self.boundary.stop(intervention_id="stop-1", reason="test"))
        self.assertEqual(self.adapter.stops, [2])
        self.assertFalse(self.boundary.reset(authorized=True, generation=2, now=10.0))


if __name__ == "__main__":
    unittest.main()
