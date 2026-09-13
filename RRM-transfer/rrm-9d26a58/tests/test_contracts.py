"""Requirements 004/007/008/009/010: contract checks, not integrated SIL tests."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields, replace
import unittest

from rrm.contracts import (
    AdmissionGuard, CapabilityDeclaration, DispatchContext, SafetyDecision,
    Truth, evidence_truth,
)


def context(generation=1, authority_epoch="test-epoch"):
    return DispatchContext(
        run_id="run-1", task_revision="task-1/v1", plan_revision="plan-1/v1",
        action_id="grasp-1", dispatch_id="dispatch-1", action_digest="payload-sha256",
        state_revision="state-1", capability_revision="hand/v1",
        permission_revision="permission-1", approval_revision="approval-not-required/v1",
        constraints_revision="constraints-1", authority_epoch=authority_epoch, stop_generation=generation,
    )


def permit(ctx=None):
    return SafetyDecision("decision-1", ctx or context(), "ALLOW", 10.0, 20.0)


class EvidenceTests(unittest.TestCase):
    def test_missing_stale_and_unknown_never_satisfy_negation(self):
        for value in (None, Truth.UNKNOWN, "FALSE"):
            self.assertIs(evidence_truth(value, fresh=True, negated=True), Truth.UNKNOWN)
        for value in Truth:
            for negated in (False, True):
                self.assertIs(evidence_truth(value, fresh=False, negated=negated), Truth.UNKNOWN)

    def test_explicit_negative_evidence_can_satisfy_negation(self):
        self.assertIs(evidence_truth(Truth.FALSE, fresh=True, negated=True), Truth.TRUE)
        self.assertIs(evidence_truth(Truth.TRUE, fresh=True, negated=True), Truth.FALSE)


class CapabilityTests(unittest.TestCase):
    def test_same_logic_for_hand_and_bimanual_profiles(self):
        for resources in (frozenset({"finger-group"}), frozenset({"left", "right"})):
            declaration = CapabilityDeclaration("opaque-id", "v1", frozenset({"grasp"}),
                                                resources, resources, "limits/v1")
            self.assertEqual(declaration.rejection_reasons("grasp", resources), ())
            self.assertIn("unsupported_operation", declaration.rejection_reasons("navigate", resources))

    def test_unavailable_unknown_resources_and_missing_limits(self):
        cap = CapabilityDeclaration("hand", "v1", {"grasp"}, {"hand"}, set(), "")
        self.assertEqual(cap.rejection_reasons("grasp", {"hand"}),
                         ("unavailable_resource", "unknown_limits"))
        self.assertIn("undeclared_resource", cap.rejection_reasons("grasp", {"base"}))
        with self.assertRaises(ValueError):
            replace(cap, available_resources={"undeclared"})

    def test_input_set_mutation_cannot_change_declaration(self):
        operations = {"grasp"}
        cap = CapabilityDeclaration("hand", "v1", operations, set(), set(), "limits")
        operations.add("navigate")
        self.assertIn("unsupported_operation", cap.rejection_reasons("navigate", set()))


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.guard = AdmissionGuard()
        self.assertTrue(self.guard.reset(generation=0, authorized=True,
                                         safe_confirmed=True, evidence_ref="fresh-safe-1"))

    def context(self, generation=1):
        return context(generation, self.guard.epoch)

    def permit(self):
        return permit(self.context())

    def test_exact_current_allow_consumed_once(self):
        self.assertIsNone(self.guard.consume(self.permit(), self.context(), now=11))
        self.assertEqual(self.guard.consume(self.permit(), self.context(), now=11), "decision_consumed")
        other = replace(self.permit(), decision_id="decision-2")
        self.assertEqual(self.guard.consume(other, self.context(), now=11), "dispatch_consumed")

    def test_every_changed_context_field_invalidates_allow(self):
        for field in fields(self.context()):
            if field.name in {"stop_generation", "authority_epoch"}:
                continue
            with self.subTest(field=field.name):
                changed = replace(self.context(), **{field.name: "changed"})
                self.assertEqual(self.guard.consume(self.permit(), changed, now=11), "stale_context")

    def test_all_nonallow_decisions_block(self):
        for verdict in ("DENY", "NEEDS_APPROVAL", "UNKNOWN"):
            self.assertEqual(self.guard.consume(replace(self.permit(), verdict=verdict), self.context(), now=11),
                             "not_allowed")

    def test_expired_future_and_invalid_clock_block(self):
        for now in (9, 20, 21, float("nan"), float("inf")):
            self.assertEqual(self.guard.consume(self.permit(), self.context(), now=now), "outside_validity_window")
        for changes in ({"expires_at": float("nan")}, {"issued_at": 21}, {"verdict": "PASS"}):
            with self.assertRaises(ValueError):
                replace(self.permit(), **changes)

    def test_stop_before_admission_and_no_implicit_resume(self):
        generation = self.guard.stop()
        self.assertEqual(self.guard.consume(self.permit(), self.context(), now=11), "stopped")
        for kwargs in ({"authorized": False}, {"safe_confirmed": False},
                       {"evidence_ref": ""}, {"generation": generation - 1}):
            options = dict(generation=generation, authorized=True, safe_confirmed=True, evidence_ref="safe")
            options.update(kwargs)
            self.assertFalse(self.guard.reset(**options))
        self.assertEqual(self.guard.consume(self.permit(), self.context(), now=11), "stopped")
        self.assertTrue(self.guard.reset(generation=generation, authorized=True,
                                         safe_confirmed=True, evidence_ref="safe"))
        self.assertEqual(self.guard.consume(self.permit(), self.context(), now=11), "stale_stop_generation")
        fresh = self.context(self.guard.generation)
        self.assertIsNone(self.guard.consume(permit(fresh), fresh, now=11))

    def test_restart_is_inhibited_and_reset_invalidates_prior_epoch(self):
        restarted = AdmissionGuard()
        self.assertEqual(restarted.consume(self.permit(), self.context(), now=11), "stopped")
        self.assertTrue(restarted.reset(generation=0, authorized=True,
                                        safe_confirmed=True, evidence_ref="fresh-safe"))
        self.assertEqual(restarted.consume(self.permit(), self.context(), now=11),
                         "stale_authority_epoch")
        # Adapter reconciliation and persistent deduplication are still required.
        before_reset = self.context(0)
        self.assertEqual(self.guard.consume(permit(before_reset), before_reset, now=11),
                         "stale_stop_generation")

    def test_concurrent_duplicate_admission_has_one_winner(self):
        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(lambda _: self.guard.consume(self.permit(), self.context(), now=11), range(32)))
        self.assertEqual(results.count(None), 1)
        self.assertEqual(results.count("decision_consumed"), 31)

    def test_stop_after_reservation_blocks_next_dispatch(self):
        self.assertIsNone(self.guard.consume(self.permit(), self.context(), now=11))
        self.guard.stop()
        next_ctx = replace(self.context(), dispatch_id="dispatch-2")
        self.assertEqual(self.guard.consume(replace(permit(next_ctx), decision_id="decision-2"),
                                            next_ctx, now=11), "stopped")

    def test_empty_context_refs_rejected(self):
        with self.assertRaises(ValueError):
            replace(self.context(), permission_revision="")


if __name__ == "__main__":
    unittest.main()
