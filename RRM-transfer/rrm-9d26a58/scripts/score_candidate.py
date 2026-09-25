#!/usr/bin/env python3
"""Score an image-only C02 candidate against a matching simulator teacher."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

from rrm.state_contracts import StateSnapshot
from rrm.visual_world_builder import score_visual_snapshot


ENTITY_CATALOG = {
    "red_block": "block",
    "blue_block": "block",
    "tray_1": "tray",
    "table": "table",
}


def load_image_only_candidate(candidate_data: dict, teacher_data: dict) -> StateSnapshot:
    """Validate that an inferred candidate is bound to this exact frozen frame."""
    if teacher_data.get("schema_version") != "rrm-hand-tabletop-c02-export/v1":
        raise ValueError("teacher is not a tabletop C02 export")
    if teacher_data.get("execution_dispatch") is not False:
        raise ValueError("teacher export is not observation-only")
    if candidate_data.get("execution_dispatch") is not False:
        raise ValueError("candidate is not image-only")
    image_sha256 = teacher_data["sample"].get("camera_sha256")
    if candidate_data.get("image_sha256") != image_sha256:
        raise ValueError("candidate is not bound to the teacher camera image")
    candidate = StateSnapshot.model_validate(candidate_data["snapshot"])
    for fact in candidate.evidence:
        if fact.provenance.value != "INFERRED" or image_sha256 not in fact.source_ref:
            raise ValueError("candidate evidence is not image-only inferred evidence")
    return candidate


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher", required=True, type=Path,
                        help="C02 export produced by export_c02_facts.py")
    parser.add_argument("--candidate", required=True, type=Path,
                        help="a separately authored image-only C02 snapshot or wrapper")
    parser.add_argument("--output", type=Path,
                        help="optional immutable score record")
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing score: {args.output}")
    teacher_data = json.loads(args.teacher.read_text(encoding="utf-8"))
    candidate_data = json.loads(args.candidate.read_text(encoding="utf-8"))
    teacher = StateSnapshot.model_validate(teacher_data["snapshot"])
    candidate = load_image_only_candidate(candidate_data, teacher_data)
    now_monotonic_s = teacher_data["sample"]["observed_monotonic_s"]
    score = score_visual_snapshot(
        candidate, teacher, now_monotonic_s=now_monotonic_s,
        entity_catalog=ENTITY_CATALOG,
    )
    record = {
        "schema_version": "rrm-hand-image-score/v1",
        "teacher_c02_sha256": hashlib.sha256(args.teacher.read_bytes()).hexdigest(),
        "candidate_sha256": hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
        "image_only_candidate": True,
        "execution_dispatch": False,
        "score": score.model_dump(mode="json"),
    }
    serialized = json.dumps(record, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
