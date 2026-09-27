import hashlib
import json
import unittest

from rrm.embodiment_learning import (
    EmbodimentEvidenceLedger, EvidenceScope, EffectVerdict, assess_route,
    compile_command_mission_evidence,
    evidence_from_drone_outcome,
)
from rrm.goal_contracts import EmbodimentRoute, RouteStatus


def scope(scene="office", body="aerial-eval"):
    return EvidenceScope(
        embodiment_id=body, operation="TAKEOFF", scene_revision=scene,
        capability_revision="cap-v2", adapter_revision="adapter-v1",
        controller_revision="controller-sha-123",
    )


def outcome(verdict, action_success, reasons=()):
    return {"kind": "TAKEOFF", "verdict": verdict,
            "action_success": action_success, "reasons": list(reasons)}


class EmbodimentLearningTests(unittest.TestCase):
    def test_only_independently_resolved_effects_change_estimate(self):
        ledger = EmbodimentEvidenceLedger()
        attempts = (
            ("good", outcome("VERIFIED", True)),
            ("hovered", outcome("MISMATCH", True, ("takeoff_altitude_mismatch",))),
            ("aborted", outcome("UNCONFIRMED", False, ("task_result_unsuccessful",))),
        )
        for attempt_id, result in attempts:
            ledger.append(evidence_from_drone_outcome(
                attempt_id=attempt_id, scope=scope(), outcome=result,
                verification_ref=f"sha256:{attempt_id}", verifier_revision="drone-v1",
            ))
        estimate = ledger.estimate(scope())
        self.assertEqual((estimate.verified_count, estimate.unmet_count,
                          estimate.unknown_count), (1, 1, 1))
        self.assertEqual(estimate.posterior_mean, 0.5)
        self.assertEqual(ledger.estimate(scope("warehouse")).posterior_mean, None)
        self.assertEqual(ledger.estimate(scope("office")).evidence_revision,
                         estimate.evidence_revision)

    def test_server_success_alone_cannot_become_verified_effect(self):
        record = evidence_from_drone_outcome(
            attempt_id="one", scope=scope(),
            outcome=outcome("UNCONFIRMED", True, ("post_odometry_missing",)),
            verification_ref="sha256:one", verifier_revision="drone-v1",
        )
        self.assertIs(record.effect_verdict, EffectVerdict.UNKNOWN)
        with self.assertRaises(ValueError):
            evidence_from_drone_outcome(
                attempt_id="two", scope=scope(),
                outcome=outcome("VERIFIED", False),
                verification_ref="sha256:two", verifier_revision="drone-v1",
            )

    def test_duplicate_attempt_is_idempotent_but_conflict_rejected(self):
        ledger = EmbodimentEvidenceLedger()
        record = evidence_from_drone_outcome(
            attempt_id="one", scope=scope(), outcome=outcome("VERIFIED", True),
            verification_ref="sha256:one", verifier_revision="drone-v1",
        )
        ledger.append(record)
        ledger.append(record)
        self.assertEqual(ledger.estimate(scope()).verified_count, 1)
        with self.assertRaises(ValueError):
            ledger.append(record.model_copy(update={"verification_ref": "sha256:changed"}))

    def test_route_assessment_is_advisory_and_exactly_scoped(self):
        route = EmbodimentRoute(
            goal_id="goal", goal_revision="v1", status=RouteStatus.CANDIDATES,
            candidate_embodiment_ids=("aerial-eval", "wheeled-eval"),
        )
        ledger = EmbodimentEvidenceLedger()
        original = route.model_dump()
        assessments = assess_route(route, (scope(), scope(body="wheeled-eval")), ledger)
        self.assertEqual(len(assessments), 2)
        self.assertIsNone(assessments[0].posterior_mean)
        self.assertEqual(route.model_dump(), original)
        with self.assertRaises(ValueError):
            assess_route(route, (scope(),), ledger)

    def test_mission_import_ignores_skipped_actions_and_binds_plan_bytes(self):
        plan = {"schema_version": "rrm-airstack-command-plan/v1",
                "active_scene": "office", "actions": [
            {"action_id": "takeoff-0", "task_id": "task", "kind": "TAKEOFF"},
            {"action_id": "land-1", "task_id": "task", "kind": "LAND"},
        ]}
        raw = json.dumps(plan, sort_keys=True).encode()
        mission = {
            "schema_version": "rrm-airstack-command-outcome/v1",
            "plan_sha256": hashlib.sha256(raw).hexdigest(),
            "results": [
                {"action_id": "takeoff-0", "outcome": {
                    "task_id": "task", "action_id": "takeoff-0", **outcome("VERIFIED", True),
                }},
                {"action_id": "land-1", "dispatch_skipped": True,
                 "outcome": {"verdict": "VERIFIED"}},
            ],
        }
        params = dict(
            plan_bytes=raw, mission=mission, embodiment_id="aerial-eval",
            active_scene="office",
            scene_revision="office-v1", capability_revision="cap-v2",
            adapter_revision="adapter-v1", controller_revision="controller-sha-123",
        )
        records = compile_command_mission_evidence(**params)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0].scope.operation, "TAKEOFF")
        with self.assertRaises(ValueError):
            compile_command_mission_evidence(**{**params, "plan_bytes": raw + b" "})
        with self.assertRaises(ValueError):
            compile_command_mission_evidence(**{**params, "active_scene": "warehouse-shelves"})


if __name__ == "__main__":
    unittest.main()
