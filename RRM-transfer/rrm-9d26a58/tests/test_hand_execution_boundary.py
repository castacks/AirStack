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
    BoundaryError, DurableJournal, HandAuthorization, HandAuthorityVerifier, HandCommand,
    HandExecutionBoundary, SafeStateEvidence, dispatch_authorization_scope,
    issue_hand_authorization, reconciliation_authorization_scope,
    reset_authorization_scope,
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
        self.signing_key = b"test-hand-authority-key-32-bytes!"
        self.verifier = HandAuthorityVerifier(issuer_keys={"test-issuer": self.signing_key},
            allowed_roles=frozenset({"hand-calibration-operator"}))
        self.auth_seq = 0
        self.boundary = self.make_boundary()
        self.command = HandCommand("iiwa7_joint_1", 0.0, 0.01, 10.0, "episode-1",
            "scene-1", self.qualification["probe_sha256"],
            hashlib.sha256(self.qualification_bytes).hexdigest())

    def make_boundary(self, journal=None):
        return HandExecutionBoundary(adapter=self.adapter,
            journal=journal or DurableJournal(self.path),
            authority_verifier=self.verifier,
            qualification_bytes=self.qualification_bytes, probe_bytes=self.probe_bytes)

    def grant(self, boundary, purpose, scope_digest, *, now=10.0,
              authorization_id=None, **changes):
        self.auth_seq += 1
        values = {"signing_key": self.signing_key,
            "authorization_id": authorization_id or f"authorization-{self.auth_seq}",
            "issuer_id": "test-issuer", "subject_id": "operator-1",
            "role": "hand-calibration-operator", "purpose": purpose,
            "authority_epoch": boundary.epoch, "stop_generation": boundary.generation,
            "scope_digest": scope_digest, "issued_at_monotonic": now - 0.1,
            "expires_at_monotonic": now + 1.0}
        values.update(changes)
        return issue_hand_authorization(**values)

    def reset_boundary(self, boundary=None, *, generation=None, now=10.0,
                       authorization=None):
        boundary = boundary or self.boundary
        generation = boundary.generation if generation is None else generation
        evidence = self.adapter.safe_state(generation)
        scope = reset_authorization_scope(generation=generation, evidence=evidence,
            qualification_sha256=boundary.qualification_sha256,
            probe_sha256=boundary.qualification["probe_sha256"])
        authorization = authorization or self.grant(boundary, "RESET", scope, now=now)
        return boundary.reset(authorization=authorization, generation=generation, now=now)

    def dispatch_boundary(self, decision, context, command, *, boundary=None,
                          now=10.0, authorization=None):
        boundary = boundary or self.boundary
        scope = dispatch_authorization_scope(decision=decision, current=context,
            command=command, qualification_sha256=boundary.qualification_sha256,
            probe_sha256=boundary.qualification["probe_sha256"])
        authorization = authorization or self.grant(boundary, "DISPATCH", scope, now=now)
        return boundary.dispatch(decision, context, command,
                                 authorization=authorization, now=now)

    def reconcile_boundary(self, boundary, *, generation=None, now=10.0,
                           authorization=None):
        generation = boundary.generation if generation is None else generation
        evidence = self.adapter.safe_state(generation)
        scope = reconciliation_authorization_scope(generation=generation,
            dispatch_ids=tuple(sorted(boundary._restart_dispatch_ids)), evidence=evidence,
            qualification_sha256=boundary.qualification_sha256,
            probe_sha256=boundary.qualification["probe_sha256"])
        authorization = authorization or self.grant(boundary, "RECONCILE", scope, now=now)
        return boundary.reconcile_restart(authorization=authorization,
                                          generation=generation, now=now)

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
            self.dispatch_boundary(self.decision(), self.context(), self.command)
        self.assertTrue(self.reset_boundary())
        context = self.context()
        self.dispatch_boundary(self.decision(context), context, self.command)
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual(records[-1]["event"], "C09_DISPATCH_INTENT")
        self.assertEqual(len(self.adapter.calls), 1)
        with self.assertRaises(BoundaryError):
            self.dispatch_boundary(self.decision(context), context, self.command)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_bad_command_or_decision_never_reaches_adapter(self):
        self.assertTrue(self.reset_boundary())
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
                    self.dispatch_boundary(self.decision(context), context, command)
        for decision in [replace(self.decision(context), verdict="DENY"),
                         replace(self.decision(context), expires_at=10.0)]:
            with self.assertRaises(BoundaryError):
                self.dispatch_boundary(decision, context, self.command)
        self.assertEqual(self.adapter.calls, [])

    def test_journal_failure_inhibits_before_adapter(self):
        self.boundary = self.make_boundary(FailingJournal(self.path))
        self.assertTrue(self.reset_boundary())
        context = self.context()
        with self.assertRaisesRegex(BoundaryError, "journal_unavailable"):
            self.dispatch_boundary(self.decision(context), context, self.command)
        self.assertEqual(self.adapter.calls, [])
        self.assertTrue(self.boundary.inhibited)

    def test_stop_invalidation_safe_evidence_and_restart(self):
        self.assertTrue(self.reset_boundary())
        context = self.context()
        self.dispatch_boundary(self.decision(context), context, self.command)
        self.adapter.ack = False
        self.assertFalse(self.boundary.stop(intervention_id="stop-1", reason="test"))
        self.assertTrue(self.boundary.inhibited)
        self.assertFalse(self.boundary.safe_confirmed)
        self.assertFalse(self.reset_boundary(generation=2))
        self.adapter.safe = False
        self.assertFalse(self.boundary.confirm_stopped(generation=2, now=10.0))
        self.adapter.safe = True
        self.assertTrue(self.boundary.confirm_stopped(generation=2, now=10.0))
        valid_scope = reset_authorization_scope(generation=2,
            evidence=self.adapter.safe_state(2),
            qualification_sha256=self.boundary.qualification_sha256,
            probe_sha256=self.boundary.qualification["probe_sha256"])
        bad_auth = replace(self.grant(self.boundary, "RESET", valid_scope), signature="0" * 64)
        self.assertFalse(self.reset_boundary(generation=2, authorization=bad_auth))
        expired = self.grant(self.boundary, "RESET", valid_scope, now=10.0,
                             expires_at_monotonic=10.5)
        self.assertFalse(self.reset_boundary(generation=2, now=11.0,
                                             authorization=expired))
        self.assertTrue(self.reset_boundary(generation=2))
        with self.assertRaises(BoundaryError):
            self.dispatch_boundary(self.decision(context), context, self.command)
        restarted = self.make_boundary()
        self.assertTrue(restarted.inhibited)
        self.assertNotEqual(restarted.epoch, self.boundary.epoch)
        self.assertFalse(self.reset_boundary(restarted, generation=0))

    def test_restart_restores_generation_and_requires_safe_reconciliation_then_reset(self):
        self.assertTrue(self.reset_boundary())
        context = self.context()
        self.dispatch_boundary(self.decision(context), context, self.command)
        call_count = len(self.adapter.calls)

        restarted = self.make_boundary()
        self.assertEqual(restarted.generation, 1)
        self.assertTrue(restarted.inhibited)
        evidence = self.adapter.safe_state(1)
        scope = reconciliation_authorization_scope(generation=1,
            dispatch_ids=("dispatch-1",), evidence=evidence,
            qualification_sha256=restarted.qualification_sha256,
            probe_sha256=restarted.qualification["probe_sha256"])
        bad_auth = replace(self.grant(restarted, "RECONCILE", scope), signature="0" * 64)
        self.assertFalse(self.reconcile_boundary(restarted, authorization=bad_auth))
        self.assertFalse(self.reconcile_boundary(restarted, generation=0))
        self.adapter.safe = False
        self.assertFalse(self.reconcile_boundary(restarted))
        self.adapter.safe = True
        self.assertTrue(self.reconcile_boundary(restarted))
        self.assertTrue(restarted.inhibited)
        self.assertEqual(len(self.adapter.calls), call_count)
        self.assertTrue(self.reset_boundary(restarted, generation=1))
        self.assertEqual(restarted.generation, 2)
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        reconciled = [item for item in records
                      if item["event"] == "C09_RESTART_RECONCILED"]
        self.assertEqual(reconciled[-1]["payload"]["dispatch_ids"], ["dispatch-1"])

        replayed = self.make_boundary()
        self.assertEqual(replayed.generation, 2)
        self.assertFalse(self.reconcile_boundary(replayed, generation=2))
        self.assertTrue(self.reset_boundary(replayed, generation=2))
        replay_command = replace(self.command, observed_at_monotonic=10.0)
        reused = DispatchContext("run-2", "task-2", "plan-2", "calibration-2",
            "dispatch-1", replay_command.digest, "state-2", "capability-2",
            "permission-2", "approval-2", "constraints-2", replayed.epoch,
            replayed.generation)
        with self.assertRaisesRegex(BoundaryError, "consumed_decision_or_dispatch"):
            self.dispatch_boundary(SafetyDecision("decision-2", reused, "ALLOW", 9.0, 11.0),
                                   reused, replay_command, boundary=replayed)

    def test_tampered_journal_refused(self):
        self.assertTrue(self.reset_boundary())
        self.path.write_text(self.path.read_text().replace("C08_RESET", "C08_FAKE"))
        with self.assertRaisesRegex(BoundaryError, "journal_chain_invalid"):
            self.make_boundary()

    def test_journal_rejects_reconciliation_of_unknown_dispatch(self):
        self.journal = DurableJournal(self.path)
        with self.assertRaisesRegex(BoundaryError,
                                    "journal_reconciliation_invalid"):
            self.journal.append("C09_RESTART_RECONCILED", {
                "generation": 0, "dispatch_ids": ["unknown"],
                "evidence_ref": "safe-1", "episode_id": "episode-1"})

    def test_concurrent_duplicate_has_one_submit(self):
        self.assertTrue(self.reset_boundary())
        context = self.context()
        decision = self.decision(context)
        scope = dispatch_authorization_scope(decision=decision, current=context,
            command=self.command, qualification_sha256=self.boundary.qualification_sha256,
            probe_sha256=self.boundary.qualification["probe_sha256"])
        authorization = self.grant(self.boundary, "DISPATCH", scope,
                                   authorization_id="concurrent-authorization")
        def attempt(_):
            try:
                self.dispatch_boundary(decision, context, self.command,
                                       authorization=authorization)
                return True
            except BoundaryError:
                return False
        with ThreadPoolExecutor(max_workers=8) as pool:
            self.assertEqual(sum(pool.map(attempt, range(8))), 1)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_authorization_is_signed_exactly_scoped_and_durably_consumed(self):
        evidence = self.adapter.safe_state(0)
        scope = reset_authorization_scope(generation=0, evidence=evidence,
            qualification_sha256=self.boundary.qualification_sha256,
            probe_sha256=self.boundary.qualification["probe_sha256"])
        valid = self.grant(self.boundary, "RESET", scope,
                           authorization_id="reset-authorization")
        cases = [replace(valid, signature="0" * 64),
                 self.grant(self.boundary, "DISPATCH", scope),
                 self.grant(self.boundary, "RESET", "f" * 64),
                 self.grant(self.boundary, "RESET", scope,
                            role="untrusted-role"),
                 self.grant(self.boundary, "RESET", scope,
                            issuer_id="unknown-issuer"),
                 self.grant(self.boundary, "RESET", scope,
                            authority_epoch="old-epoch"),
                 self.grant(self.boundary, "RESET", scope,
                            stop_generation=1),
                 self.grant(self.boundary, "RESET", scope,
                            issued_at_monotonic=0.0, expires_at_monotonic=40.0)]
        for authorization in cases:
            with self.subTest(authorization=authorization.authorization_id):
                self.assertFalse(self.reset_boundary(authorization=authorization))
        self.assertFalse(self.boundary._journal_fault)
        self.assertTrue(self.reset_boundary(authorization=valid))
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual([item["event"] for item in records],
                         ["C06_AUTHORIZATION_CONSUMED", "C08_RESET"])
        replayed_journal = DurableJournal(self.path)
        self.assertIn("reset-authorization", replayed_journal.used_authorization_ids)

    def test_stop_delivered_even_if_journal_fails_and_reset_inhibited(self):
        self.boundary = self.make_boundary(StopFailJournal(self.path))
        self.assertTrue(self.reset_boundary())
        self.assertTrue(self.boundary.stop(intervention_id="stop-1", reason="test"))
        self.assertEqual(self.adapter.stops, [2])
        self.assertFalse(self.reset_boundary(generation=2))


if __name__ == "__main__":
    unittest.main()
