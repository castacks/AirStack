#!/usr/bin/env python3
"""Render a dry-run RRM semantic navigation decision as an AirStack proposal.

This is deliberately a file-to-file evaluation boundary.  It imports no ROS client
and cannot dispatch the emitted proposal; the separately gated dispatcher is the
only component that can create an AirStack action client.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from rrm.contracts import CapabilityDeclaration
from rrm.drone_decision import AirStackDroneDecisionBridge, DroneNavigationTarget
from rrm.state_contracts import StateSnapshot
from rrm.task_contracts import TaskRequest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-json", type=Path, required=True)
    parser.add_argument("--snapshot-json", type=Path, required=True)
    parser.add_argument("--capability-json", type=Path, required=True)
    parser.add_argument("--targets-json", type=Path, required=True,
                        help="JSON array of adapter-local DroneNavigationTarget records")
    parser.add_argument("--robot-name", default="robot_1")
    parser.add_argument("--now-monotonic-s", type=float, required=True)
    return parser


def _json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    args = _parser().parse_args()
    task = TaskRequest.model_validate(_json(args.task_json))
    snapshot = StateSnapshot.model_validate(_json(args.snapshot_json))
    capability_data = _json(args.capability_json)
    capabilities = CapabilityDeclaration(
        embodiment_id=capability_data["embodiment_id"],
        revision=capability_data["revision"],
        operations=frozenset(capability_data["operations"]),
        resources=frozenset(capability_data["resources"]),
        available_resources=frozenset(capability_data["available_resources"]),
        limits_ref=capability_data["limits_ref"],
    )
    targets = tuple(DroneNavigationTarget.model_validate(value) for value in _json(args.targets_json))
    bridge = AirStackDroneDecisionBridge(
        embodiment_id=capabilities.embodiment_id,
        robot_name=args.robot_name,
        targets=targets,
    )
    decision = bridge.decide(task, snapshot, capabilities, now_monotonic_s=args.now_monotonic_s)
    print(decision.model_dump_json(indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
