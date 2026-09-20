#!/usr/bin/env python3
"""Authoritative read-only C03 provider for the AirStack drone simulator."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

# Standalone provider execution must not depend on a Remote-SSH shell inheriting
# repository paths. The test/bootstrap script installs Pydantic into .rrm-deps.
SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal
from rrm.dynamic_feasibility import (
    DynamicFeasibilityResult, EvidenceAuthority, FeasibilityCheck,
    FeasibilityVerdict, proposal_sha256,
)


OBSERVER = Path(__file__).with_name("airstack_feasibility_observer.py")
CONTAINER = "airstack-robot-desktop-1"
PROFILE_REVISION = "airstack-drone-lidar-corridor/v1"
EXPECTED_EMBODIMENT = "aerial-eval"
EXPECTED_CAPABILITY_REVISION = "office-airframe-v1"
EXPECTED_LIMITS_REF = "office-bounded-nav-v1"


def _check(name: str, passed: bool | None, report_sha: str, detail: str,
           source: str = PROFILE_REVISION) -> FeasibilityCheck:
    return FeasibilityCheck(
        name=name, passed=passed, authority=EvidenceAuthority.AUTHORITATIVE,
        evidence_ref=f"inline-sha256:{report_sha}", source_revision=source, detail=detail,
    )


def _observe(proposal: DroneTaskProposal) -> dict:
    if proposal.kind is not DroneTaskKind.NAVIGATE or len(proposal.waypoints) != 1:
        raise ValueError("AirStack feasibility provider supports one NAVIGATE waypoint.")
    target = proposal.waypoints[0]
    remote = "/tmp/rrm-airstack-feasibility-observer.py"
    subprocess.run(["docker", "cp", str(OBSERVER), f"{CONTAINER}:{remote}"],
                   check=True, capture_output=True, timeout=10)
    command = (
        "source /root/AirStack/robot/ros_ws/install/local_setup.bash; "
        "exec python3 \"$1\" --robot \"$2\" --target-x \"$3\" --target-y \"$4\" "
        "--target-z \"$5\" --clearance-m 0.4 --timeout-s 8"
    )
    completed = subprocess.run(
        ["docker", "exec", "-e", "ROS_DOMAIN_ID=1", CONTAINER, "bash", "-lc", command,
         "rrm-feasibility", remote, proposal.robot_name, str(target.x), str(target.y), str(target.z)],
        check=True, capture_output=True, text=True, timeout=15,
    )
    lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
    if not lines:
        raise RuntimeError("AirStack feasibility observer returned no report.")
    report = json.loads(lines[-1])
    if report.get("execution_dispatch") is not False:
        raise RuntimeError("Feasibility observer crossed the no-dispatch boundary.")
    return report


def evaluate(query: dict) -> DynamicFeasibilityResult:
    if query.get("schema_version") != "rrm-feasibility-query/v1":
        raise ValueError("Unsupported feasibility query schema.")
    if query.get("execution_dispatch") is not False:
        raise ValueError("Feasibility query must be explicitly non-dispatching.")
    proposal = DroneTaskProposal.model_validate(query["proposal"])
    action = query["semantic_action"]
    snapshot, capabilities = query["snapshot"], query["capabilities"]
    observation, scene_state = query["observation"], query["scene_state"]
    target_ids = action.get("targets") if isinstance(action, dict) else None
    verified = set(scene_state.get("verified_entities", []))
    localized = {
        item.get("key", {}).get("subject") for item in snapshot.get("evidence", [])
        if item.get("key", {}).get("predicate") == "localized" and item.get("truth") == "TRUE"
    }
    grounding = bool(
        isinstance(action, dict) and action.get("id") == proposal.action_id
        and action.get("verb") == "NAVIGATE_TO" and isinstance(target_ids, list)
        and len(target_ids) == 1 and target_ids[0] in verified and target_ids[0] in localized
    )
    target = proposal.waypoints[0] if proposal.waypoints else None
    vehicle = observation.get("vehicle", {})
    start = (vehicle.get("x"), vehicle.get("y"), vehicle.get("z"))
    numerics = (*start, *((target.x, target.y, target.z) if target else (None, None, None)))
    numeric_ok = all(
        isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
        for value in numerics
    )
    distance = (math.dist(start, (target.x, target.y, target.z)) if numeric_ok and target else math.inf)
    profile_ok = bool(
        capabilities.get("embodiment_id") == EXPECTED_EMBODIMENT
        and capabilities.get("revision") == EXPECTED_CAPABILITY_REVISION
        and capabilities.get("limits_ref") == EXPECTED_LIMITS_REF
        and "NAVIGATE_TO" in capabilities.get("operations", [])
        and "airframe" in capabilities.get("resources", [])
        and "airframe" in capabilities.get("available_resources", [])
    )
    body_limits = bool(
        profile_ok and target and numeric_ok and proposal.frame_id == "map"
        and proposal.robot_name == "robot_1" and 0.5 <= target.z <= 3.0
        and distance <= 10.0 and proposal.goal_tolerance_m is not None
        and 0.1 <= proposal.goal_tolerance_m <= 0.5
    )

    report = _observe(proposal)
    if report.get("schema_version") != "airstack-feasibility-observation/v1":
        raise ValueError("Unsupported AirStack feasibility observation schema.")
    report_bytes = json.dumps(report, sort_keys=True, separators=(",", ":")).encode()
    report_sha = hashlib.sha256(report_bytes).hexdigest()
    channels_ok = not report.get("missing_channels") and not report.get("stale_channels")
    corridor = report.get("corridor") or {}
    corridor_start = corridor.get("start") or {}
    corridor_target = corridor.get("target") or {}
    state_matches = bool(
        numeric_ok
        and report.get("odometry_frame_id") == "map"
        and report.get("odometry_child_frame_id") == "base_link"
        and all(isinstance(corridor_start.get(axis), (int, float)) for axis in ("x", "y", "z"))
        and math.dist(start, tuple(corridor_start[axis] for axis in ("x", "y", "z"))) <= 0.25
    )
    target_matches = bool(
        target
        and all(isinstance(corridor_target.get(axis), (int, float)) for axis in ("x", "y", "z"))
        and math.dist(
            (target.x, target.y, target.z),
            tuple(corridor_target[axis] for axis in ("x", "y", "z")),
        ) <= 1e-6
    )
    coverage = corridor.get("coverage_sufficient")
    collision_free = corridor.get("collision_free")
    physics = (
        bool(collision_free)
        if (channels_ok and state_matches and target_matches
            and coverage is True and isinstance(collision_free, bool))
        else None
    )
    navigate_type = report.get("actions", {}).get(f"/{proposal.robot_name}/tasks/navigate", [])
    land_type = report.get("actions", {}).get(f"/{proposal.robot_name}/tasks/land", [])
    controller = (bool(
        report.get("connected") and report.get("armed") and report.get("airborne")
        and report.get("has_control") and "task_msgs/action/NavigateTask" in navigate_type
    ) if channels_ok else None)
    resources = (bool(
        not report.get("planner_stuck")
        and isinstance(report.get("linear_speed_m_s"), (int, float))
        and not isinstance(report.get("linear_speed_m_s"), bool)
        and math.isfinite(report["linear_speed_m_s"])
        and report["linear_speed_m_s"] <= 0.15
    ) if channels_ok else None)
    stop_channel = bool(
        "task_msgs/action/NavigateTask" in navigate_type
        and "task_msgs/action/LandTask" in land_type
    )
    checks = (
        _check("grounding", grounding, report_sha, "semantic target is live-verified and localized"),
        _check("body_limits", body_limits, report_sha,
               f"profile, map waypoint, and distance {distance:.3f} m are within limits"),
        _check("physics", physics, report_sha,
               "fresh matching Ouster corridor has sufficient range and expanded clearance"),
        _check("controller", controller, report_sha,
               "vehicle is connected, armed, airborne, in control, and Navigate server is live"),
        _check("resources", resources, report_sha,
               "planner is not stuck and vehicle is stationary enough for a new action"),
        _check("stop_channel", stop_channel, report_sha,
               "Navigate cancellation and Land task endpoints are present"),
    )
    authoritative = [check.passed for check in checks]
    verdict = (FeasibilityVerdict.INFEASIBLE if any(value is False for value in authoritative)
               else FeasibilityVerdict.UNCERTAIN if any(value is None for value in authoritative)
               else FeasibilityVerdict.FEASIBLE)
    checked = time.monotonic()
    return DynamicFeasibilityResult(
        task_id=proposal.task_id, action_id=proposal.action_id,
        embodiment_id=capabilities["embodiment_id"],
        proposal_sha256=proposal_sha256(proposal),
        observation_sha256=observation["sha256"],
        state_revision=snapshot["revision"], capability_revision=capabilities["revision"],
        scene_revision=str(scene_state["provenance"]),
        profile_revision=str(capabilities["limits_ref"]),
        stop_generation=query["stop_generation"], checked_monotonic_s=checked,
        expires_monotonic_s=checked + 2.0, verdict=verdict, checks=checks,
        evidence_payload_json=report_bytes.decode(), evidence_sha256=report_sha,
    )


def main() -> int:
    raw = sys.stdin.buffer.read(2 * 1024 * 1024 + 1)
    if not raw or len(raw) > 2 * 1024 * 1024:
        raise SystemExit("invalid feasibility query size")
    try:
        result = evaluate(json.loads(raw))
    except Exception as error:
        # A provider failure must not fabricate a typed FEASIBLE result. The caller
        # records the exception and halts before admission.
        print(json.dumps({"error": type(error).__name__}), file=sys.stderr)
        return 2
    print(result.model_dump_json())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
