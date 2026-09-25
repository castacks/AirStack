#!/usr/bin/env python3
"""Export a no-action tabletop-probe sample as simulator C02 evidence.

The probe is the sole teacher here. This utility only copies the explicitly
labelled entities in one camera/state sample into the existing C02 schema; it
does not infer graspability, reachability, contacts, hand state, or safety.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

from rrm.ground_truth import GroundTruthWorldBuilder


ENTITY_KINDS = {
    "red_block": "block",
    "blue_block": "block",
    "tray_1": "tray",
    "table": "table",
}


def load_probe(path: Path) -> dict[str, Any]:
    """Load and validate exactly the read-only output accepted by this exporter."""
    probe = json.loads(path.read_text(encoding="utf-8"))
    if probe.get("schema_version") != "rrm-hand-tabletop-probe/v1":
        raise ValueError("unsupported tabletop probe schema")
    for field in ("controller_command_sent", "ros_connected", "execution_dispatch"):
        if probe.get(field) is not False:
            raise ValueError(f"probe is not a no-action observation: {field}")
    if probe.get("reset_hashes_match") is not True:
        raise ValueError("probe reset repeatability did not pass")
    if set(probe.get("scene_entities", {})) != set(ENTITY_KINDS):
        raise ValueError("probe entity set does not match the frozen tabletop catalog")
    if not isinstance(probe.get("reset_samples"), list) or not probe["reset_samples"]:
        raise ValueError("probe has no reset samples")
    return probe


def build_export(probe: dict[str, Any], *, probe_sha256: str, reset_index: int,
                 task_id: str) -> dict[str, Any]:
    """Build one teacher snapshot tied to an identical camera/state sample."""
    samples = probe["reset_samples"]
    if reset_index < 0 or reset_index >= len(samples):
        raise ValueError("reset index is outside the probe samples")
    sample = samples[reset_index]
    capture = sample.get("camera_capture")
    if not isinstance(capture, dict):
        raise ValueError("selected sample has no usable camera capture")
    observed_s = sample.get("observed_monotonic_s")
    episode_id = sample.get("episode_id")
    if (capture.get("episode_id") != episode_id
            or capture.get("observed_monotonic_s") != observed_s
            or not isinstance(capture.get("sha256"), str)):
        raise ValueError("camera capture is not paired with the selected state sample")

    source_ref = f"probe:{episode_id}:{capture['sha256']}"
    builder = GroundTruthWorldBuilder(
        task_id=task_id,
        episode_id=episode_id,
        default_source_ref=source_ref,
        default_max_age_s=10.0,
    )
    for entity_id in sorted(ENTITY_KINDS):
        builder.ingest_entity(
            entity_id=entity_id,
            entity_kind=ENTITY_KINDS[entity_id],
            exists=True,
            localized=True,
            observed_monotonic_s=observed_s,
            received_monotonic_s=observed_s,
        )
    snapshot = builder.snapshot()
    return {
        "schema_version": "rrm-hand-tabletop-c02-export/v1",
        "probe_sha256": probe_sha256,
        "probe_scene_recipe_sha256": probe["scene_recipe_sha256"],
        "sample": {
            "reset_index": reset_index,
            "episode_id": episode_id,
            "camera_path": capture["path"],
            "camera_sha256": capture["sha256"],
            "observed_monotonic_s": observed_s,
        },
        "execution_dispatch": False,
        "snapshot": snapshot.model_dump(mode="json"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-json", required=True, type=Path)
    parser.add_argument("--output", type=Path,
                        help="defaults to c02_facts.json beside --probe-json")
    parser.add_argument("--reset-index", type=int, default=0)
    parser.add_argument("--task-id", default="hand-task-001")
    args = parser.parse_args()
    output = args.output or args.probe_json.with_name("c02_facts.json")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing evidence: {output}")
    probe_bytes = args.probe_json.read_bytes()
    export = build_export(
        load_probe(args.probe_json),
        probe_sha256=hashlib.sha256(probe_bytes).hexdigest(),
        reset_index=args.reset_index,
        task_id=args.task_id,
    )
    output.write_text(json.dumps(export, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "episode_id": export["sample"]["episode_id"],
        "camera_sha256": export["sample"]["camera_sha256"],
        "fact_count": len(export["snapshot"]["evidence"]),
        "execution_dispatch": False,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
