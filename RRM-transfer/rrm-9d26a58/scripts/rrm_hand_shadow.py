#!/usr/bin/env python3
"""Run one synthetic hand-contract fixture and export proposal-only evidence."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

from rrm.contracts import CapabilityDeclaration, Truth
from rrm.ground_truth import GroundTruthWorldBuilder
from rrm.hand_shadow import HandSceneManifest, HandShadowBridge
from rrm.state_contracts import FactKey
from rrm.task_contracts import TaskRequest
from rrm.visual_world_builder import (
    MediaArtifact, VisualCandidateStatus, VisualGroundingInput,
    parse_visual_candidate, score_visual_snapshot,
)


def _capability_record(value: CapabilityDeclaration) -> dict:
    return {
        "schema_version": "rrm-hand-shadow-capability/v1",
        "embodiment_id": value.embodiment_id,
        "revision": value.revision,
        "operations": sorted(value.operations),
        "resources": sorted(value.resources),
        "available_resources": sorted(value.available_resources),
        "limits_ref": value.limits_ref,
        "numeric_feasibility_verified": False,
        "execution_dispatch": False,
    }


def build_records(fixture: dict) -> dict[str, dict]:
    """Validate and evaluate one fixture entirely without a control adapter."""
    if fixture.get("schema_version") != "rrm-hand-shadow-fixture/v1":
        raise ValueError("unsupported hand fixture schema")
    scene = HandSceneManifest.model_validate(fixture["scene"])
    task = TaskRequest.model_validate(fixture["task"])
    capabilities = CapabilityDeclaration(**fixture["capability"])
    observed = float(fixture["observed_monotonic_s"])
    received = float(fixture["received_monotonic_s"])
    now = float(fixture["now_monotonic_s"])
    max_age = float(fixture["max_age_s"])
    teacher = GroundTruthWorldBuilder(
        task_id=task.task_id, episode_id=scene.episode_id,
        default_source_ref="synthetic-hand-teacher/v1",
        default_max_age_s=max_age,
    )
    for item in fixture["teacher_facts"]:
        teacher.ingest_fact(
            FactKey.model_validate(item["key"]), Truth(item["truth"]),
            observed_monotonic_s=observed, received_monotonic_s=received,
        )
    snapshot = teacher.snapshot()
    decision = HandShadowBridge(scene).decide(
        task, snapshot, capabilities, now_monotonic_s=now,
    )
    records = {
        "scene": scene.model_dump(mode="json"),
        "c01-task": task.model_dump(mode="json"),
        "c02-teacher": snapshot.model_dump(mode="json"),
        "c03-capability": _capability_record(capabilities),
        "shadow-decision": decision.model_dump(mode="json"),
    }
    if decision.intent is not None:
        records["c04-intent"] = decision.intent.model_dump(mode="json")
    if decision.plan is not None:
        records["c05-plan"] = decision.plan.model_dump(mode="json")
    probe = fixture.get("visual_protocol_probe")
    if probe is not None:
        if probe.get("synthetic") is not True:
            raise ValueError("fixture visual probe must be marked synthetic")
        context = VisualGroundingInput(
            task_id=task.task_id, episode_id=scene.episode_id,
            state_revision=f"{scene.episode_id}/visual-probe-1",
            entity_catalog=scene.catalog(),
            media=MediaArtifact(
                source_ref=probe["media_source_ref"], sha256=probe["media_sha256"],
                observed_monotonic_s=observed,
            ),
            received_monotonic_s=received, max_age_s=max_age,
            model_ref=probe["model_ref"],
        )
        candidate = parse_visual_candidate(probe["raw_response"], context)
        records["c02-visual-probe"] = {
            **candidate.model_dump(mode="json"),
            "synthetic_protocol_probe": True,
            "media_artifact_present": False,
            "execution_dispatch": False,
        }
        if candidate.status is VisualCandidateStatus.ACCEPTED:
            score = score_visual_snapshot(
                candidate.snapshot, snapshot, now_monotonic_s=now,
                entity_catalog=scene.catalog(),
            )
            records["visual-score"] = {
                **score.model_dump(mode="json"),
                "precision": score.precision, "recall": score.recall,
                "synthetic_protocol_probe": True,
                "perception_result": False,
            }
    return records


def write_records(fixture_path: Path, output_dir: Path) -> dict:
    raw = fixture_path.read_bytes()
    fixture = json.loads(raw)
    records = build_records(fixture)
    output_dir.mkdir(parents=True, exist_ok=False)
    digests = {}
    for name, record in records.items():
        path = output_dir / f"{name}.json"
        encoded = (json.dumps(record, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
        path.write_bytes(encoded)
        digests[name] = hashlib.sha256(encoded).hexdigest()
    causes = {
        "c02-teacher": ["scene", "c01-task"],
        "c03-capability": ["scene"],
        "c04-intent": ["c01-task", "c02-teacher", "c03-capability"],
        "c05-plan": ["c04-intent"],
        "c02-visual-probe": ["scene", "c01-task"],
        "visual-score": ["c02-teacher", "c02-visual-probe"],
    }
    manifest = {
        "schema_version": "rrm-hand-shadow-run/v1",
        "run_id": fixture["run_id"],
        "created_at": datetime.now(timezone.utc).isoformat(),
        "fixture_sha256": hashlib.sha256(raw).hexdigest(),
        "records_sha256": digests,
        "caused_by": {name: refs for name, refs in causes.items() if name in records},
        "status": records["shadow-decision"]["status"],
        "execution_dispatch": False,
        "simulator_action_sent": False,
        "evidence_scope": "synthetic_contract_probe",
    }
    (output_dir / "run-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result = write_records(args.fixture, args.output_dir)
    print(json.dumps({
        "run_id": result["run_id"], "status": result["status"],
        "record_count": len(result["records_sha256"]),
        "execution_dispatch": result["execution_dispatch"],
        "output_dir": str(args.output_dir),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
