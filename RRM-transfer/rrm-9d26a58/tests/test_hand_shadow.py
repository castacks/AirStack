"""Proposal-only hand contracts and synthetic visual scoring."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from rrm.contracts import CapabilityDeclaration, Truth
from rrm.goal_contracts import GoalRequest, bind_selected_route_to_c01, route_goal
from rrm.hand_shadow import HandSceneManifest, HandShadowBridge, HandShadowStatus
from rrm.state_contracts import FactEvidence, FactKey, FactProvenance, StateSnapshot
from rrm.task_contracts import TaskRequest
from rrm.visual_world_builder import score_visual_snapshot
from rrm_hand_shadow import build_records, write_records


FIXTURE = Path(__file__).parents[1] / "examples" / "hand_shadow" / "fixture.json"


def fixture() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


class HandShadowTests(unittest.TestCase):
    def test_neutral_goal_routes_into_existing_dry_run_c01_adapter(self):
        value = fixture()
        scene = HandSceneManifest.model_validate(value["scene"])
        capability = CapabilityDeclaration(**value["capability"])
        goal = GoalRequest(
            goal_id=value["task"]["task_id"],
            revision=value["task"]["revision"],
            objective=value["task"]["objective"],
            context_refs=tuple(value["task"]["context_refs"]),
            required_operations=frozenset({"GRASP", "PLACE"}),
            required_resources=frozenset({"arm", "hand"}),
            qualitative_constraints=("use the context-selected block",),
        )
        route = route_goal(goal, (capability,))
        binding = bind_selected_route_to_c01(
            goal, route, capability,
            constraints_revision=value["task"]["constraints_revision"],
            issuer_id=value["task"]["issuer_id"],
            permission_revision=value["task"]["permission_revision"],
        )
        records = build_records(value)
        snapshot = StateSnapshot.model_validate(records["c02-teacher"])
        decision = HandShadowBridge(scene).decide(
            binding.task, snapshot, capability,
            now_monotonic_s=value["now_monotonic_s"],
        )
        self.assertIs(decision.status, HandShadowStatus.PROPOSED)
        self.assertFalse(decision.execution_dispatch)
        self.assertTrue(all(
            "shadow-semantic-only" in node.feasibility_ref
            for node in decision.plan.actions
        ))

    def test_context_binding_builds_two_action_proposal_and_synthetic_score(self):
        records = build_records(fixture())
        decision = records["shadow-decision"]
        self.assertEqual(decision["status"], "PROPOSED")
        self.assertEqual(decision["selected_entity_id"], "red_block")
        self.assertFalse(decision["execution_dispatch"])
        self.assertEqual([node["action"]["verb"] for node in records["c05-plan"]["actions"]],
                         ["GRASP", "PLACE"])
        self.assertEqual(records["c05-plan"]["actions"][1]["dependencies"],
                         ["grasp-red_block"])
        self.assertTrue(all("shadow-semantic-only" in node["feasibility_ref"]
                            for node in records["c05-plan"]["actions"]))
        self.assertFalse(records["c03-capability"]["numeric_feasibility_verified"])
        score = records["visual-score"]
        self.assertEqual((score["exact_matches"], score["missed_teacher_facts"]), (5, 4))
        self.assertEqual(score["recall"], 5 / 9)
        self.assertFalse(score["perception_result"])

    def test_stale_or_contradictory_required_fact_holds(self):
        value = fixture()
        value["now_monotonic_s"] = 12.0
        self.assertEqual(build_records(value)["shadow-decision"]["status"], "HOLD")
        value = fixture()
        records = build_records(value)
        scene = HandSceneManifest.model_validate(value["scene"])
        task = TaskRequest.model_validate(value["task"])
        capability = CapabilityDeclaration(**value["capability"])
        snapshot = StateSnapshot.model_validate(records["c02-teacher"])
        opposite = FactEvidence(
            key=FactKey(subject="red_block", predicate="graspable"),
            truth=Truth.FALSE, provenance=FactProvenance.SENSOR,
            source_ref="contradictory-sensor", observed_monotonic_s=10.0,
            received_monotonic_s=10.1, max_age_s=1.0,
        )
        conflict = snapshot.model_copy(update={"evidence": snapshot.evidence + (opposite,)})
        result = HandShadowBridge(scene).decide(
            task, conflict, capability, now_monotonic_s=10.2,
        )
        self.assertEqual(result.status, HandShadowStatus.HOLD)
        self.assertIn("required_fact_unknown", result.reasons[0])

    def test_wrong_context_episode_and_capability_do_not_plan(self):
        value = fixture()
        value["task"]["context_refs"] = ["selection/unknown"]
        self.assertEqual(build_records(value)["shadow-decision"]["status"],
                         "NEEDS_CLARIFICATION")
        value = fixture()
        value["capability"]["available_resources"] = ["arm"]
        self.assertEqual(build_records(value)["shadow-decision"]["status"],
                         "UNSUPPORTED")
        value = fixture()
        records = build_records(value)
        scene = HandSceneManifest.model_validate(value["scene"])
        task = TaskRequest.model_validate(value["task"])
        capability = CapabilityDeclaration(**value["capability"])
        snapshot = StateSnapshot.model_validate(records["c02-teacher"])
        prior = snapshot.model_copy(update={"episode_id": "prior-episode"})
        self.assertEqual(
            HandShadowBridge(scene).decide(task, prior, capability,
                                           now_monotonic_s=10.2).status,
            HandShadowStatus.HOLD,
        )

    def test_visual_score_refuses_cross_episode_comparison(self):
        records = build_records(fixture())
        teacher = StateSnapshot.model_validate(records["c02-teacher"])
        visual = StateSnapshot.model_validate(records["c02-visual-probe"]["snapshot"])
        wrong_episode = visual.model_copy(update={"episode_id": "previous-episode"})
        with self.assertRaisesRegex(ValueError, "task/episode"):
            score_visual_snapshot(
                wrong_episode, teacher, now_monotonic_s=10.2,
                entity_catalog={"red_block": "block", "blue_block": "block", "tray_1": "tray"},
            )

    def test_exported_manifest_hashes_and_links_are_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "run"
            manifest = write_records(FIXTURE, target)
            self.assertFalse(manifest["execution_dispatch"])
            self.assertFalse(manifest["simulator_action_sent"])
            self.assertEqual(manifest["evidence_scope"], "synthetic_contract_probe")
            for name, digest in manifest["records_sha256"].items():
                self.assertEqual(hashlib.sha256((target / f"{name}.json").read_bytes()).hexdigest(),
                                 digest)
            self.assertTrue(all(ref in manifest["records_sha256"]
                                for refs in manifest["caused_by"].values() for ref in refs))
            with self.assertRaises(FileExistsError):
                write_records(FIXTURE, target)

    def test_shadow_modules_have_no_control_import(self):
        root = Path(__file__).parents[1]
        for relative in ("rrm/hand_shadow.py", "scripts/rrm_hand_shadow.py"):
            source = (root / relative).read_text(encoding="utf-8")
            for prohibited in ("rclpy", "ActionClient", "create_publisher(", "send_goal",
                               "docker", "mavros", "PX4"):
                self.assertNotIn(prohibited, source)


if __name__ == "__main__":
    unittest.main()
