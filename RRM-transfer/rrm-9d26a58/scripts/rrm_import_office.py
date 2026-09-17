#!/usr/bin/env python3
"""Verify a PSC artifact and compile its actual C05 into an unexecuted proposal.

Offline replay time is intentionally used only for this import. It is NOT a live
freshness check or execution approval. A live run must revalidate scene binding,
vehicle readiness and evidence before using the public task action adapter.
"""
import argparse
import hashlib
import json
from pathlib import Path

from rrm.cosmos_reason2 import CosmosCandidateStatus, parse_cosmos_candidate, render_cosmos_prompt
from rrm.drone_decision import AirStackDroneDecisionBridge, DroneNavigationTarget, DroneDecisionStatus
from rrm.airstack_drone import MapWaypoint
from rrm_cosmos_reason2 import load_context


def import_bundle(bundle: Path):
    record = json.loads((bundle / "result.json").read_text())
    for key, name in (("input_sha256", "input.json"), ("media_sha256", "input.png")):
        if record.get(key) != hashlib.sha256((bundle / name).read_bytes()).hexdigest():
            raise ValueError(f"artifact hash mismatch: {name}")
    context = load_context(bundle / "input.json")
    scene = json.loads((bundle / "scene_manifest.json").read_text())
    trusted_scene = json.loads((Path(__file__).resolve().parents[1] / "examples" /
                               "office_visual_eval" / "scene_manifest.json").read_text())
    if scene != trusted_scene:
        raise ValueError("scene binding differs from local reviewed fixture")
    if scene["scene_id"] != context.snapshot.episode_id:
        raise ValueError("scene/episode mismatch")
    if record["prompt"] != render_cosmos_prompt(context):
        raise ValueError("prompt/context mismatch")
    candidate = parse_cosmos_candidate(record["raw_response"], context)
    if candidate.status is not CosmosCandidateStatus.ACCEPTED:
        raise ValueError(f"model candidate not accepted: {candidate.reasons}")
    if candidate.model_dump(mode="json") != record["candidate"]:
        raise ValueError("stored candidate differs from reparsed raw response")
    targets = tuple(DroneNavigationTarget(
        entity_id=entity, waypoints=(MapWaypoint(**item["map_waypoint"]),), goal_tolerance_m=0.3,
    ) for entity, item in scene["markers"].items() if "map_waypoint" in item)
    adapter = AirStackDroneDecisionBridge(
        embodiment_id=context.capabilities.embodiment_id, robot_name="robot_1", targets=targets,
    )
    decision = adapter.compile_plan(candidate.plan, context.task, context.snapshot,
                                    context.capabilities, now_monotonic_s=context.now_monotonic_s)
    if decision.status is not DroneDecisionStatus.READY:
        raise ValueError(f"adapter refused plan: {decision.reasons}")
    return decision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    decision = import_bundle(args.bundle)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "decision.json").write_text(decision.model_dump_json(indent=2) + "\n")
    (args.output_dir / "proposal.json").write_text(decision.proposal.model_dump_json(indent=2) + "\n")
    print("Actual learned plan imported. Proposal requires live scene/readiness validation; no dispatch.")


if __name__ == "__main__":
    main()
