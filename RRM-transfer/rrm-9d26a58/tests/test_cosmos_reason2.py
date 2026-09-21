"""Contract tests for learned Cosmos candidate validation; no model/GPU required."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from rrm.cosmos_reason2 import (
    CosmosCandidateStatus,
    CosmosReasoningInput,
    parse_cosmos_candidate,
    render_cosmos_prompt,
)
from rrm.contracts import CapabilityDeclaration
from rrm.state_contracts import StateSnapshot
from rrm.task_contracts import TaskRequest


_FIXTURE = Path(__file__).parents[1] / "examples" / "cosmos_reason2" / "navigation_ground_truth.json"


def _context(path: Path = _FIXTURE) -> CosmosReasoningInput:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return CosmosReasoningInput(
        task=TaskRequest.model_validate(payload["task"]),
        snapshot=StateSnapshot.model_validate(payload["snapshot"]),
        capabilities=CapabilityDeclaration(
            embodiment_id=payload["capabilities"]["embodiment_id"],
            revision=payload["capabilities"]["revision"],
            operations=frozenset(payload["capabilities"]["operations"]),
            resources=frozenset(payload["capabilities"]["resources"]),
            available_resources=frozenset(payload["capabilities"]["available_resources"]),
            limits_ref=payload["capabilities"]["limits_ref"],
        ),
        now_monotonic_s=payload["now_monotonic_s"],
    )


class CosmosReason2ContractTest(unittest.TestCase):
    def test_targetless_takeoff_candidate_is_valid_only_with_takeoff_capability(self) -> None:
        context = _context()
        context = CosmosReasoningInput(
            task=context.task, snapshot=context.snapshot,
            capabilities=CapabilityDeclaration(
                embodiment_id=context.capabilities.embodiment_id,
                revision=context.capabilities.revision,
                operations=frozenset({"TAKEOFF"}),
                resources=context.capabilities.resources,
                available_resources=context.capabilities.available_resources,
                limits_ref=context.capabilities.limits_ref,
            ),
            now_monotonic_s=context.now_monotonic_s,
        )
        raw = json.dumps({
            "status": "READY",
            "grounded_goal": {"name": "airborne", "subject": "$self", "obj": None},
            "grounded_entities": [], "ambiguity_refs": [],
            "explanation": "Take off using the adapter-owned flight profile.",
            "actions": [{"id": "takeoff-1", "verb": "TAKEOFF",
                         "targets": [], "dependencies": []}],
            "recovery_budget": 0,
        })
        candidate = parse_cosmos_candidate(raw, context)
        self.assertEqual(candidate.status, CosmosCandidateStatus.ACCEPTED)
        self.assertEqual(candidate.plan.actions[0].action.targets, [])

    def test_office_prompt_exposes_ids_and_authored_goal_semantics(self) -> None:
        path = Path(__file__).parents[1] / "examples/office_visual_eval/navigation_context.json"
        prompt = render_cosmos_prompt(_context(path))
        payload = json.loads(prompt.split("C01/C02/C03 input follows:\n", 1)[1])
        self.assertEqual(payload["state"]["entity_ids"], ["blue_marker", "orange_marker"])
        self.assertIn('"name":"near"', prompt)
        self.assertIn('"subject":"$self"', prompt)
        self.assertIn('Allowed action verbs: ["NAVIGATE_TO"]', prompt)

    def test_pasted_psc_office_response_remains_rejected(self) -> None:
        # Transcription supplied by the user, not the retrieved PSC result bundle.
        root = Path(__file__).parents[1]
        raw = (root / "tests/fixtures/office_46280177_response.json").read_text()
        context = _context(root / "examples/office_visual_eval/navigation_context.json")
        candidate = parse_cosmos_candidate("```json\n" + raw + "\n```", context)
        self.assertEqual(candidate.status, CosmosCandidateStatus.REJECTED)
        self.assertEqual(candidate.reasons, (
            "ungrounded_entity:blue navigation marker",
            "ungrounded_entity:orange navigation marker",
        ))
        self.assertIsNone(candidate.plan)

    def test_prompt_uses_evidence_and_declares_non_control_boundary(self) -> None:
        prompt = render_cosmos_prompt(_context())
        self.assertIn("loading_bay_marker", prompt)
        self.assertIn("You do not control a robot", prompt)
        self.assertIn("NAVIGATE_TO", prompt)
        self.assertIn("grounded_entities is mandatory", prompt)

    def test_ready_candidate_becomes_c04_and_c05(self) -> None:
        raw = json.dumps({
            "status": "READY",
            "grounded_goal": {"name": "near", "subject": "$self", "obj": "loading_bay_marker"},
            "grounded_entities": ["loading_bay_marker"],
            "ambiguity_refs": [],
            "explanation": "The labelled target is localized.",
            "actions": [{"id": "navigate-marker", "verb": "NAVIGATE_TO",
                         "targets": ["loading_bay_marker"], "dependencies": []}],
            "recovery_budget": 1,
        })
        candidate = parse_cosmos_candidate(raw, _context())
        self.assertEqual(candidate.status, CosmosCandidateStatus.ACCEPTED)
        self.assertEqual(candidate.intent.grounded_entities, ("loading_bay_marker",))
        self.assertEqual(candidate.plan.actions[0].action.verb.value, "NAVIGATE_TO")

    def test_invented_entity_is_rejected(self) -> None:
        raw = json.dumps({
            "status": "READY",
            "grounded_goal": {"name": "near", "subject": "$self", "obj": "invented_marker"},
            "grounded_entities": ["invented_marker"],
            "ambiguity_refs": [], "explanation": "",
            "actions": [{"id": "navigate-invented", "verb": "NAVIGATE_TO",
                         "targets": ["invented_marker"], "dependencies": []}],
            "recovery_budget": 0,
        })
        candidate = parse_cosmos_candidate(raw, _context())
        self.assertEqual(candidate.status, CosmosCandidateStatus.REJECTED)
        self.assertIn("ungrounded_entity:invented_marker", candidate.reasons)

    def test_malformed_or_actionless_ready_candidate_is_rejected(self) -> None:
        malformed = parse_cosmos_candidate("no JSON here", _context())
        self.assertEqual(malformed.status, CosmosCandidateStatus.REJECTED)
        actionless = parse_cosmos_candidate(json.dumps({
            "status": "READY",
            "grounded_goal": {"name": "near", "subject": "$self", "obj": "loading_bay_marker"},
            "grounded_entities": ["loading_bay_marker"], "ambiguity_refs": [],
            "explanation": "", "actions": [], "recovery_budget": 0,
        }), _context())
        self.assertIn("ready_candidate_missing_actions", actionless.reasons)

    def test_clarification_never_creates_plan(self) -> None:
        raw = json.dumps({
            "status": "NEEDS_CLARIFICATION", "grounded_goal": None,
            "grounded_entities": [], "ambiguity_refs": ["target_identity"],
            "explanation": "Two targets match.", "actions": [], "recovery_budget": 0,
        })
        candidate = parse_cosmos_candidate(raw, _context())
        self.assertEqual(candidate.status, CosmosCandidateStatus.NEEDS_CLARIFICATION)
        self.assertIsNone(candidate.plan)

    def test_unsupported_verb_is_rejected(self) -> None:
        raw = json.dumps({
            "status": "READY",
            "grounded_goal": {"name": "near", "subject": "$self", "obj": "loading_bay_marker"},
            "grounded_entities": ["loading_bay_marker"], "ambiguity_refs": [],
            "explanation": "", "actions": [{"id": "grasp-marker", "verb": "GRASP",
                                                  "targets": ["loading_bay_marker"], "dependencies": []}],
            "recovery_budget": 0,
        })
        candidate = parse_cosmos_candidate(raw, _context())
        self.assertEqual(candidate.status, CosmosCandidateStatus.REJECTED)
        self.assertIn("invalid_ready_candidate:unsupported_verb:GRASP", candidate.reasons)

    def test_runtime_cli_has_no_ros_or_dispatch_import(self) -> None:
        script = (Path(__file__).parents[1] / "scripts" / "rrm_cosmos_reason2.py").read_text(
            encoding="utf-8",
        )
        for forbidden in ("rclpy", "send_goal", "ActionClient", "DroneTaskProposal"):
            self.assertNotIn(forbidden, script)


if __name__ == "__main__":
    unittest.main()
