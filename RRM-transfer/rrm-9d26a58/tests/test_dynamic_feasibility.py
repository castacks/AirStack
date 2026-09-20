"""Body-agnostic feasibility and single-use admission tests."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal, MapWaypoint
from rrm.dynamic_feasibility import (
    DynamicFeasibilityResult, EvidenceAuthority, FeasibilityCheck, FeasibilityVerdict,
    SingleUseAdmission, proposal_sha256,
)
from rrm_cosmos_reason2 import load_context


ROOT = Path(__file__).parents[1]


class DynamicFeasibilityTests(unittest.TestCase):
    def setUp(self):
        self.context = load_context(
            ROOT / "examples" / "office_visual_eval" / "navigation_context.json"
        )
        self.proposal = DroneTaskProposal(
            task_id=self.context.task.task_id, action_id="navigate-blue",
            kind=DroneTaskKind.NAVIGATE, frame_id="map",
            waypoints=(MapWaypoint(x=3.2, y=0.0, z=1.5),), goal_tolerance_m=0.3,
        )
        self.image_sha = hashlib.sha256(b"frame").hexdigest()
        self.observation = {"sha256": self.image_sha}
        self.scene = {"provenance": "isaac-office-physics/v1"}

    def result(self, *, verdict=FeasibilityVerdict.FEASIBLE, proposal=None,
               observation_sha=None, stop_generation=2):
        values = {
            "grounding": True, "body_limits": True, "physics": True,
            "controller": True, "resources": True, "stop_channel": True,
        }
        if verdict is FeasibilityVerdict.INFEASIBLE:
            values["physics"] = False
        elif verdict is FeasibilityVerdict.UNCERTAIN:
            values["physics"] = None
        selected = proposal or self.proposal
        return DynamicFeasibilityResult(
            task_id=selected.task_id, action_id=selected.action_id,
            embodiment_id=self.context.capabilities.embodiment_id,
            proposal_sha256=proposal_sha256(selected),
            observation_sha256=observation_sha or self.image_sha,
            state_revision=self.context.snapshot.revision,
            capability_revision=self.context.capabilities.revision,
            scene_revision=self.scene["provenance"],
            profile_revision=self.context.capabilities.limits_ref,
            stop_generation=stop_generation,
            checked_monotonic_s=10.0, expires_monotonic_s=11.0,
            verdict=verdict,
            checks=tuple(FeasibilityCheck(
                name=name, passed=passed, authority=EvidenceAuthority.AUTHORITATIVE,
                evidence_ref=f"evidence:{name}",
                source_revision="test/v1", detail=f"{name} result",
            ) for name, passed in values.items()),
        )

    def test_exact_current_feasible_result_is_consumed_once(self):
        admission = SingleUseAdmission()
        result = self.result()
        record = admission.consume(
            result, self.proposal, self.context, self.observation, self.scene,
            stop_generation=2, now_monotonic_s=10.5,
        )
        self.assertEqual(record["decision"], "ALLOW")
        self.assertTrue(record["single_use_consumed"])
        with self.assertRaisesRegex(ValueError, "already consumed"):
            admission.consume(
                result, self.proposal, self.context, self.observation, self.scene,
                stop_generation=2, now_monotonic_s=10.5,
            )

    def test_every_material_dependency_is_bound(self):
        cases = {
            "proposal": {"proposal": self.proposal.model_copy(update={"action_id": "other"})},
            "observation": {"observation": {"sha256": "f" * 64}},
            "scene": {"scene_state": {"provenance": "changed-scene/v2"}},
            "stop": {"stop_generation": 3},
        }
        for name, changes in cases.items():
            values = {
                "result": self.result(), "proposal": self.proposal,
                "context": self.context, "observation": self.observation,
                "scene_state": self.scene, "stop_generation": 2,
                "now_monotonic_s": 10.5,
            }
            values.update(changes)
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "stale or detached"):
                SingleUseAdmission().consume(**values)

    def test_infeasible_uncertain_and_expired_results_block(self):
        for result, now, reason in (
            (self.result(verdict=FeasibilityVerdict.INFEASIBLE), 10.5, "did not explicitly pass"),
            (self.result(verdict=FeasibilityVerdict.UNCERTAIN), 10.5, "did not explicitly pass"),
            (self.result(), 11.0, "expired"),
        ):
            with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
                SingleUseAdmission().consume(
                    result, self.proposal, self.context, self.observation, self.scene,
                    stop_generation=2, now_monotonic_s=now,
                )

    def test_feasible_schema_requires_all_authoritative_checks(self):
        with self.assertRaisesRegex(ValueError, "missing a required"):
            DynamicFeasibilityResult(
                **self.result().model_dump(exclude={"checks"}),
                checks=(FeasibilityCheck(name="physics", passed=True,
                                         authority=EvidenceAuthority.AUTHORITATIVE,
                                         evidence_ref="physics:test", source_revision="v1",
                                         detail="only physics"),),
            )

    def test_learned_prediction_is_advisory_and_cannot_replace_physics(self):
        base = self.result().model_dump(exclude={"checks"})
        learned = FeasibilityCheck(
            name="physics", passed=True, authority=EvidenceAuthority.ADVISORY,
            evidence_ref="model:learned-feasibility", source_revision="model/v1",
            detail="learned predictor estimates success",
        )
        other = tuple(FeasibilityCheck(
            name=name, passed=True, authority=EvidenceAuthority.AUTHORITATIVE,
            evidence_ref=f"evidence:{name}", source_revision="test/v1",
            detail=f"{name} passed",
        ) for name in (
            "grounding", "body_limits", "controller", "resources", "stop_channel",
        ))
        with self.assertRaisesRegex(ValueError, "missing a required"):
            DynamicFeasibilityResult(**base, checks=(*other, learned))

    def test_contract_has_no_runtime_control_surface(self):
        source = (ROOT / "rrm" / "dynamic_feasibility.py").read_text(encoding="utf-8").lower()
        for forbidden in ("import rclpy", "actionclient", "mavros", "px4", "import subprocess"):
            self.assertNotIn(forbidden, source)

    def test_inline_evidence_is_checksum_bound(self):
        base = self.result().model_dump()
        payload = json.dumps({"physics": "fresh"}, sort_keys=True, separators=(",", ":"))
        base.update(evidence_payload_json=payload, evidence_sha256=hashlib.sha256(payload.encode()).hexdigest())
        DynamicFeasibilityResult(**base)
        base["evidence_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            DynamicFeasibilityResult(**base)


if __name__ == "__main__":
    unittest.main()
