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
PROFILE_REVISION = "airstack-drone-route-corridor/v2"
EXPECTED_EMBODIMENT = "aerial-eval"
EXPECTED_CAPABILITY_REVISION = "office-airframe-v1"
EXPECTED_LIMITS_REF = "office-bounded-nav-v1"
FLIGHT_CAPABILITY_REVISION = "office-airframe-v2"
FLIGHT_LIMITS_REF = "office-bounded-flight-v2"
MAX_ROUTE_WAYPOINTS = 16


def _check(name: str, passed: bool | None, report_sha: str, detail: str,
           source: str = PROFILE_REVISION) -> FeasibilityCheck:
    return FeasibilityCheck(
        name=name, passed=passed, authority=EvidenceAuthority.AUTHORITATIVE,
        evidence_ref=f"inline-sha256:{report_sha}", source_revision=source, detail=detail,
    )


def _observe(proposal: DroneTaskProposal) -> dict:
    if proposal.kind is DroneTaskKind.NAVIGATE:
        if not 1 <= len(proposal.waypoints) <= MAX_ROUTE_WAYPOINTS:
            raise ValueError("AirStack route must contain 1-16 NAVIGATE waypoints.")
        target_args = [
            "--route-json",
            json.dumps([[point.x, point.y, point.z] for point in proposal.waypoints],
                       separators=(",", ":")),
        ]
    elif proposal.kind is DroneTaskKind.TAKEOFF:
        target_args = ["--takeoff-altitude-m", str(proposal.target_altitude_m)]
    else:
        raise ValueError("AirStack feasibility provider supports TAKEOFF and NAVIGATE.")
    remote = "/tmp/rrm-airstack-feasibility-observer.py"
    subprocess.run(["docker", "cp", str(OBSERVER), f"{CONTAINER}:{remote}"],
                   check=True, capture_output=True, timeout=10)
    command = (
        "source /root/AirStack/robot/ros_ws/install/local_setup.bash; "
        "exec python3 \"$1\" --robot \"$2\" \"${@:3}\" --clearance-m 0.4 --timeout-s 8"
    )
    completed = subprocess.run(
        ["docker", "exec", "-e", "ROS_DOMAIN_ID=1", CONTAINER, "bash", "-lc", command,
         "rrm-feasibility", remote, proposal.robot_name, *target_args],
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
    if proposal.kind is DroneTaskKind.NAVIGATE:
        grounding = bool(
            isinstance(action, dict) and action.get("id") == proposal.action_id
            and action.get("verb") == "NAVIGATE_TO" and isinstance(target_ids, list)
            and len(target_ids) == 1 and target_ids[0] in verified and target_ids[0] in localized
        )
    else:
        grounding = bool(
            proposal.kind is DroneTaskKind.TAKEOFF and isinstance(action, dict)
            and action.get("id") == proposal.action_id and action.get("verb") == "TAKEOFF"
            and target_ids == []
        )
    route = tuple(proposal.waypoints)
    vehicle = observation.get("vehicle", {})
    start = (vehicle.get("x"), vehicle.get("y"), vehicle.get("z"))
    command_values = ([coordinate for point in route for coordinate in (point.x, point.y, point.z)]
                      if proposal.kind is DroneTaskKind.NAVIGATE
                      else [proposal.target_altitude_m, proposal.velocity_m_s])
    numerics = (*start, *command_values)
    numeric_ok = all(
        isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
        for value in numerics
    )
    route_points = [start, *((point.x, point.y, point.z) for point in route)]
    distance = (sum(math.dist(first, second) for first, second in zip(route_points, route_points[1:]))
                if numeric_ok and route else 0.0)
    base_profile_ok = bool(
        capabilities.get("embodiment_id") == EXPECTED_EMBODIMENT
        and "airframe" in capabilities.get("resources", [])
        and "airframe" in capabilities.get("available_resources", [])
    )
    capability_revision = capabilities.get("revision")
    limits_ref = capabilities.get("limits_ref")
    if proposal.kind is DroneTaskKind.NAVIGATE:
        profile_ok = bool(
            base_profile_ok and "NAVIGATE_TO" in capabilities.get("operations", [])
            and ((capability_revision == FLIGHT_CAPABILITY_REVISION
                  and limits_ref == FLIGHT_LIMITS_REF)
                 or (capability_revision == EXPECTED_CAPABILITY_REVISION
                     and limits_ref == EXPECTED_LIMITS_REF and len(route) == 1))
        )
        body_limits = bool(
            profile_ok and route and numeric_ok and proposal.frame_id == "map"
            and proposal.robot_name == "robot_1" and len(route) <= MAX_ROUTE_WAYPOINTS
            and all(0.5 <= point.z <= 3.0 for point in route)
            and distance <= 25.0 and proposal.goal_tolerance_m is not None
            and 0.1 <= proposal.goal_tolerance_m <= 0.5
        )
    else:
        profile_ok = bool(
            base_profile_ok and capability_revision == FLIGHT_CAPABILITY_REVISION
            and limits_ref == FLIGHT_LIMITS_REF
            and "TAKEOFF" in capabilities.get("operations", [])
        )
        body_limits = bool(
            profile_ok and numeric_ok and proposal.robot_name == "robot_1"
            and -0.1 <= start[2] <= 0.3
            and 0.5 <= proposal.target_altitude_m <= 3.0
            and 0.1 <= proposal.velocity_m_s <= 1.0
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
    reported_waypoints = corridor.get("waypoints")
    if reported_waypoints is None and proposal.kind is DroneTaskKind.NAVIGATE and len(route) == 1:
        reported_waypoints = [corridor_target]
    expected_waypoints = (
        [(point.x, point.y, point.z) for point in route]
        if proposal.kind is DroneTaskKind.NAVIGATE
        else [(start[0], start[1], proposal.target_altitude_m)]
    )
    target_matches = bool(
        numeric_ok and isinstance(reported_waypoints, list)
        and len(reported_waypoints) == len(expected_waypoints)
        and all(
            isinstance(actual, dict)
            and all(isinstance(actual.get(axis), (int, float))
                    and not isinstance(actual.get(axis), bool) for axis in ("x", "y", "z"))
            and math.dist(expected, tuple(actual[axis] for axis in ("x", "y", "z"))) <= 1e-6
            for expected, actual in zip(expected_waypoints, reported_waypoints)
        )
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
    takeoff_type = report.get("actions", {}).get(f"/{proposal.robot_name}/tasks/takeoff", [])
    land_type = report.get("actions", {}).get(f"/{proposal.robot_name}/tasks/land", [])
    if proposal.kind is DroneTaskKind.NAVIGATE:
        controller = (bool(
            report.get("connected") and report.get("armed") and report.get("airborne")
            and report.get("has_control") and "task_msgs/action/NavigateTask" in navigate_type
        ) if channels_ok else None)
    else:
        controller = (bool(
            report.get("connected") and not report.get("armed") and not report.get("airborne")
            and "task_msgs/action/TakeoffTask" in takeoff_type
        ) if channels_ok else None)
    stationary = bool(
        isinstance(report.get("linear_speed_m_s"), (int, float))
        and not isinstance(report.get("linear_speed_m_s"), bool)
        and math.isfinite(report["linear_speed_m_s"])
        and report["linear_speed_m_s"] <= 0.15
    )
    resources = ((stationary and not report.get("planner_stuck"))
                 if channels_ok and proposal.kind is DroneTaskKind.NAVIGATE
                 else stationary if channels_ok else None)
    active_type = (navigate_type if proposal.kind is DroneTaskKind.NAVIGATE else takeoff_type)
    expected_type = ("task_msgs/action/NavigateTask" if proposal.kind is DroneTaskKind.NAVIGATE
                     else "task_msgs/action/TakeoffTask")
    stop_channel = bool(expected_type in active_type and "task_msgs/action/LandTask" in land_type)
    checks = (
        _check("grounding", grounding, report_sha, "semantic target is live-verified and localized"),
        _check("body_limits", body_limits, report_sha,
               f"profile and {proposal.kind.value.lower()} command are within limits; route={distance:.3f} m"),
        _check("physics", physics, report_sha,
               "fresh matching Ouster corridor has sufficient range and expanded clearance"),
        _check("controller", controller, report_sha,
               "vehicle state and the selected public task server match the command kind"),
        _check("resources", resources, report_sha,
               "required planner state and vehicle motion are ready for a new action"),
        _check("stop_channel", stop_channel, report_sha,
               "selected task cancellation and Land task endpoints are present"),
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
