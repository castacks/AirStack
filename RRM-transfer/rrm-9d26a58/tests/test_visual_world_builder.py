"""C02 visual-world-builder tests; no model, simulator or control runtime needed."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from rrm.contracts import Truth
from rrm.ground_truth import GroundTruthWorldBuilder
from rrm.state_contracts import FactProvenance, StateSnapshot
from rrm.visual_world_builder import (
    MediaArtifact, VisualCandidateStatus, VisualGroundingInput, parse_visual_candidate,
    render_visual_prompt, score_visual_snapshot,
)


def _context() -> VisualGroundingInput:
    return VisualGroundingInput(
        task_id="visual-nav-001", episode_id="office-seed-1", state_revision="office-seed-1/vlm-1",
        entity_catalog={"loading_bay_marker": "marker", "red_crate": "crate"},
        media=MediaArtifact(source_ref="office-seed-1/camera-front.png", sha256="a" * 64,
                            observed_monotonic_s=10.0),
        received_monotonic_s=10.2, max_age_s=1.0, model_ref="cosmos-reason2-8b",
    )


def _ready(claims: list[dict[str, object]]) -> str:
    return json.dumps({"status": "READY", "claims": claims})


class VisualWorldBuilderTests(unittest.TestCase):
    def test_office_fixture_catalog_and_teacher_are_consistent(self) -> None:
        fixture_dir = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        catalog = json.loads((fixture_dir / "entity_catalog.json").read_text(encoding="utf-8"))
        teacher = StateSnapshot.model_validate(
            json.loads((fixture_dir / "teacher_snapshot.json").read_text(encoding="utf-8")),
        )
        self.assertEqual(set(catalog), {"blue_marker", "orange_marker"})
        self.assertEqual({item.key.subject for item in teacher.evidence}, set(catalog))
        self.assertTrue(all(item.provenance is FactProvenance.SIMULATOR
                            for item in teacher.evidence))

    def test_catalog_bound_candidate_becomes_inferred_c02(self) -> None:
        candidate = parse_visual_candidate(_ready([
            {"subject": "loading_bay_marker", "predicate": "exists", "obj": None, "truth": "TRUE"},
            {"subject": "loading_bay_marker", "predicate": "kind", "obj": "marker", "truth": "TRUE"},
            {"subject": "loading_bay_marker", "predicate": "localized", "obj": None, "truth": "TRUE"},
        ]), _context())
        self.assertEqual(candidate.status, VisualCandidateStatus.ACCEPTED)
        self.assertTrue(all(item.provenance is FactProvenance.INFERRED
                            for item in candidate.snapshot.evidence))
        self.assertIn("sha256:" + "a" * 64, candidate.snapshot.evidence[0].source_ref)

    def test_unknown_entity_and_unsupported_predicate_are_rejected(self) -> None:
        unknown = parse_visual_candidate(_ready([
            {"subject": "invented_marker", "predicate": "exists", "obj": None, "truth": "TRUE"},
        ]), _context())
        self.assertEqual(unknown.status, VisualCandidateStatus.REJECTED)
        self.assertIn("unknown_catalog_entity:invented_marker", unknown.reasons[0])
        unsupported = parse_visual_candidate(_ready([
            {"subject": "red_crate", "predicate": "near", "obj": None, "truth": "TRUE"},
        ]), _context())
        self.assertEqual(unsupported.status, VisualCandidateStatus.REJECTED)

    def test_media_manifest_is_required_and_prompt_has_no_control_surface(self) -> None:
        with self.assertRaises(ValueError):
            MediaArtifact(source_ref="", sha256="a" * 64, observed_monotonic_s=1.0)
        prompt = render_visual_prompt(_context())
        self.assertIn("loading_bay_marker", prompt)
        self.assertIn("Do not invent", prompt)
        self.assertNotIn("send_goal", prompt)

    def test_readonly_capture_utility_has_no_task_dispatch_surface(self) -> None:
        script = (Path(__file__).parents[1] / "scripts" / "airstack_capture_image.py").read_text(
            encoding="utf-8",
        )
        self.assertIn("capture_mode", script)
        for forbidden in ("ActionClient", "send_goal", "takeoff", "navigate", "land"):
            self.assertNotIn(forbidden, script)

    def test_teacher_score_counts_absence_as_missed_not_false(self) -> None:
        candidate = parse_visual_candidate(_ready([
            {"subject": "loading_bay_marker", "predicate": "exists", "obj": None, "truth": "TRUE"},
        ]), _context()).snapshot
        teacher_builder = GroundTruthWorldBuilder(task_id="visual-nav-001", episode_id="office-seed-1")
        teacher_builder.ingest_entity(entity_id="loading_bay_marker", entity_kind="marker", exists=True,
                                      localized=True, observed_monotonic_s=10.0, received_monotonic_s=10.2)
        teacher = teacher_builder.snapshot()
        score = score_visual_snapshot(candidate, teacher, now_monotonic_s=10.3,
                                      entity_catalog=_context().entity_catalog)
        self.assertEqual(score.exact_matches, 1)
        self.assertEqual(score.missed_teacher_facts, 2)
        self.assertEqual(score.mismatches, 0)
        self.assertEqual(score.recall, 1 / 3)

    def test_teacher_score_counts_wrong_truth_and_extra_fact(self) -> None:
        candidate = parse_visual_candidate(_ready([
            {"subject": "loading_bay_marker", "predicate": "exists", "obj": None, "truth": "FALSE"},
            {"subject": "red_crate", "predicate": "exists", "obj": None, "truth": "TRUE"},
        ]), _context()).snapshot
        teacher_builder = GroundTruthWorldBuilder(task_id="visual-nav-001", episode_id="office-seed-1")
        teacher_builder.ingest_entity(entity_id="loading_bay_marker", entity_kind="marker", exists=True,
                                      localized=None, observed_monotonic_s=10.0, received_monotonic_s=10.2)
        score = score_visual_snapshot(candidate, teacher_builder.snapshot(), now_monotonic_s=10.3,
                                      entity_catalog=_context().entity_catalog)
        self.assertEqual(score.mismatches, 1)
        self.assertEqual(score.extra_candidate_facts, 1)


if __name__ == "__main__":
    unittest.main()
