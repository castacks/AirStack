#!/usr/bin/env python3
"""Export verified AirStack mission effects for advisory RRM-EM analysis only."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rrm.embodiment_learning import EmbodimentEvidenceLedger, compile_command_mission_evidence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--mission-outcome", required=True, type=Path)
    parser.add_argument("--embodiment-id", required=True)
    parser.add_argument("--active-scene", required=True,
                        help="catalog scene shortname recorded in the command plan")
    parser.add_argument("--scene-revision", required=True)
    parser.add_argument("--capability-revision", required=True)
    parser.add_argument("--adapter-revision", required=True)
    parser.add_argument("--controller-revision", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    plan_bytes = args.plan.read_bytes()
    mission_bytes = args.mission_outcome.read_bytes()
    records = compile_command_mission_evidence(
        plan_bytes=plan_bytes, mission=json.loads(mission_bytes),
        embodiment_id=args.embodiment_id, active_scene=args.active_scene,
        scene_revision=args.scene_revision,
        capability_revision=args.capability_revision,
        adapter_revision=args.adapter_revision,
        controller_revision=args.controller_revision,
    )
    ledger = EmbodimentEvidenceLedger()
    for record in records:
        ledger.append(record)
    scopes = sorted({record.scope for record in records}, key=lambda scope: scope.operation)
    export = {
        "schema_version": "rrm-em-mission-evidence/v1",
        "execution_dispatch": False,
        "plan_sha256": hashlib.sha256(plan_bytes).hexdigest(),
        "mission_outcome_sha256": hashlib.sha256(mission_bytes).hexdigest(),
        "records": [record.model_dump(mode="json") for record in records],
        "estimates": [ledger.estimate(scope).model_dump(mode="json") for scope in scopes],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(export, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Wrote {len(records)} RRM-EM evidence records to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
