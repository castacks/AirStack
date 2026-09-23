#!/usr/bin/env python3
"""Execute one checksum-recorded sequence through public AirStack task actions."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

import airstack_drone_dispatch as dispatcher
from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal


def _observe_state(robot_name: str, timeout_s: float = 4.0) -> dict:
    """Obtain a fresh read-only state snapshot in a separate ROS process."""
    completed = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("airstack_task_discovery.py")),
         "--robot", robot_name, "--timeout-s", str(timeout_s), "--state-only"],
        capture_output=True, text=True, timeout=timeout_s + 2.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "inter-action state discovery failed: "
            + (completed.stderr.strip() or f"exit {completed.returncode}")
        )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("inter-action state discovery returned no report")
    report = json.loads(lines[-1])
    if report.get("schema_version") != "airstack-task-discovery/v1":
        raise RuntimeError("inter-action state discovery schema is incompatible")
    return report


def _replan_decision(proposal: DroneTaskProposal, observation: dict) -> dict:
    """Reconcile one already-authorized action with fresh canonical state."""
    base = {
        "schema_version": "rrm-command-replan/v1",
        "action_id": proposal.action_id,
        "kind": proposal.kind.value,
        "observation": observation,
        "execution_dispatch": False,
    }
    missing = set(observation.get("missing_state") or ())
    stale = set(observation.get("stale_state") or ())
    if missing.intersection({"airborne", "vehicle", "odometry"}) or stale.intersection(
            {"airborne", "vehicle", "odometry"}):
        return {**base, "decision": "HALT", "reason": "canonical_state_unavailable"}
    if observation.get("connected") is not True:
        return {**base, "decision": "HALT", "reason": "vehicle_not_connected"}
    if observation.get("flight_state_consistent") is False:
        return {**base, "decision": "HALT", "reason": "contradictory_flight_state"}
    airborne = observation.get("airborne") is True
    armed = observation.get("armed") is True
    if proposal.kind is DroneTaskKind.TAKEOFF and airborne and armed:
        return {**base, "decision": "SKIP_SATISFIED", "reason": "already_airborne"}
    if proposal.kind in {DroneTaskKind.NAVIGATE, DroneTaskKind.EXPLORE}:
        if not airborne or not armed:
            return {**base, "decision": "HALT", "reason": "airborne_action_without_flight_state"}
        position = observation.get("position")
        if (not isinstance(position, dict)
                or not isinstance(position.get("z"), (int, float))
                or position["z"] <= 0.3):
            return {**base, "decision": "HALT", "reason": "contradictory_airborne_altitude"}
        if proposal.kind is DroneTaskKind.NAVIGATE and observation.get("position"):
            point = proposal.waypoints[-1]
            position = observation["position"]
            try:
                distance = ((float(position["x"]) - point.x) ** 2
                            + (float(position["y"]) - point.y) ** 2
                            + (float(position["z"]) - point.z) ** 2) ** 0.5
            except (KeyError, TypeError, ValueError):
                return {**base, "decision": "HALT", "reason": "invalid_canonical_position"}
            base["distance_to_goal_m"] = distance
            if distance <= proposal.goal_tolerance_m:
                return {**base, "decision": "SKIP_SATISFIED",
                        "reason": "navigation_endpoint_already_satisfied"}
    if proposal.kind is DroneTaskKind.LAND and not airborne and not armed:
        return {**base, "decision": "SKIP_SATISFIED", "reason": "already_grounded_disarmed"}
    return {**base, "decision": "CONTINUE", "reason": "fresh_state_compatible"}


def _dispatch(proposal: DroneTaskProposal, outcome: Path) -> int:
    timeout = ((proposal.time_limit_s or 0) + 60.0
               if proposal.kind is DroneTaskKind.EXPLORE else 120.0)
    try:
        return dispatcher._execute(
            proposal, 5.0, verify_observation=True, outcome_json=outcome,
            observation_timeout_s=10.0, max_observation_age_s=2.0,
            takeoff_acceptance_distance_m=0.3,
            takeoff_max_horizontal_displacement_m=0.3,
            landing_max_altitude_m=0.3, action_timeout_s=timeout,
        )
    except KeyboardInterrupt:
        outcome.write_text(json.dumps({
            "verdict": "UNCONFIRMED", "reason": "operator_stop_before_goal_tracking",
            "physical_stop_verified": False,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 130
    except Exception as error:
        outcome.write_text(json.dumps({
            "verdict": "UNCONFIRMED", "reason": "dispatch_error",
            "error_type": type(error).__name__,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 4


def _needs_takeoff_recovery(proposal: DroneTaskProposal, record: dict) -> bool:
    """Recover only from a completed takeoff with fresh evidence of flight."""
    vehicle = record.get("post_vehicle_state")
    odometry = record.get("post_odometry")
    return bool(
        proposal.kind is DroneTaskKind.TAKEOFF
        and isinstance(record.get("action_success"), bool)
        and record.get("verdict") != "VERIFIED"
        and isinstance(vehicle, dict)
        and vehicle.get("connected") is True
        and vehicle.get("armed") is True
        and isinstance(odometry, dict)
        and isinstance(odometry.get("z"), (int, float))
        and odometry["z"] > 0.3
    )


def _needs_replan_recovery(decision: dict) -> bool:
    """Land only after a halt with fresh, explicit evidence of meaningful flight."""
    observation = decision.get("observation")
    position = observation.get("position") if isinstance(observation, dict) else None
    unavailable = (
        set(observation.get("missing_state") or ())
        | set(observation.get("stale_state") or ())
        if isinstance(observation, dict) else set()
    )
    return bool(
        decision.get("decision") == "HALT"
        and isinstance(observation, dict)
        and not unavailable.intersection({"airborne", "vehicle", "odometry"})
        and observation.get("flight_state_consistent") is not False
        and observation.get("connected") is True
        and observation.get("armed") is True
        and observation.get("airborne") is True
        and isinstance(position, dict)
        and isinstance(position.get("z"), (int, float))
        and position["z"] > 0.3
    )


def _recovery_action(value: dict, proposals: tuple[DroneTaskProposal, ...]) -> DroneTaskProposal | None:
    specification = value.get("recovery")
    has_takeoff = any(item.kind is DroneTaskKind.TAKEOFF for item in proposals)
    if not has_takeoff:
        if specification is not None:
            raise SystemExit("recovery action is only valid for a plan containing takeoff")
        return None
    if (not isinstance(specification, dict)
            or specification.get("trigger") not in {
                "verified_takeoff_mismatch_while_airborne",
                "verified_mission_halt_while_airborne",
            }):
        raise SystemExit("takeoff plan requires a predeclared recovery action")
    action = DroneTaskProposal.model_validate(specification.get("action"))
    takeoff = next(item for item in proposals if item.kind is DroneTaskKind.TAKEOFF)
    if (action.kind is not DroneTaskKind.LAND
            or action.task_id != takeoff.task_id
            or action.robot_name != takeoff.robot_name
            or action.action_id in {item.action_id for item in proposals}):
        raise SystemExit("invalid takeoff recovery action")
    return action


def _run_recovery(action: DroneTaskProposal, evidence_dir: Path,
                  trigger_action_id: str) -> dict:
    print(json.dumps({
        "event": "recovery_started", "trigger_action_id": trigger_action_id,
        "recovery_action": action.preview(),
    }, sort_keys=True), flush=True)
    recovery_outcome = evidence_dir / f"recovery-{action.action_id}-outcome.json"
    recovery_code = _dispatch(action, recovery_outcome)
    recovery_record = (json.loads(recovery_outcome.read_text())
                       if recovery_outcome.is_file() else
                       {"verdict": "UNCONFIRMED", "reason": "missing_outcome"})
    result = {
        "trigger_action_id": trigger_action_id, "action_id": action.action_id,
        "return_code": recovery_code, "outcome": recovery_record,
    }
    print(json.dumps({"event": "recovery_finished", **result}, sort_keys=True), flush=True)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-json", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    raw = args.plan_json.read_bytes()
    value = json.loads(raw)
    if value.get("schema_version") != "rrm-airstack-command-plan/v1":
        raise SystemExit("unsupported command plan")
    proposals = tuple(DroneTaskProposal.model_validate(item) for item in value.get("actions", []))
    if not proposals:
        raise SystemExit("command plan has no actions")
    recovery_action = _recovery_action(value, proposals)
    preview = {
        "plan_sha256": hashlib.sha256(raw).hexdigest(),
        "actions": [proposal.preview() for proposal in proposals],
        "recovery": recovery_action.preview() if recovery_action is not None else None,
        "execution_requested": args.execute,
    }
    print(json.dumps(preview, sort_keys=True), flush=True)
    if not args.execute:
        return 0
    args.evidence_dir.mkdir(parents=True, exist_ok=False)
    results = []
    replans = []
    recovery_result = None
    replan_policy = value.get("replan_policy") or {}
    for index, proposal in enumerate(proposals):
        if index and replan_policy.get("mode") == "observe_between_actions":
            try:
                observation = _observe_state(proposal.robot_name)
                decision = _replan_decision(proposal, observation)
            except Exception as error:
                decision = {
                    "schema_version": "rrm-command-replan/v1",
                    "action_id": proposal.action_id,
                    "kind": proposal.kind.value,
                    "decision": "HALT",
                    "reason": "observation_failed",
                    "error_type": type(error).__name__,
                    "execution_dispatch": False,
                }
            decision["observed_at"] = datetime.now(timezone.utc).isoformat()
            replan_path = args.evidence_dir / f"replan-{index:04d}-{proposal.action_id}.json"
            replan_path.write_text(
                json.dumps(decision, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            replans.append(decision)
            print(json.dumps({"event": "replan_decision", **decision}, sort_keys=True),
                  flush=True)
            if decision["decision"] == "HALT":
                if recovery_action is not None and _needs_replan_recovery(decision):
                    recovery_result = _run_recovery(
                        recovery_action, args.evidence_dir, proposal.action_id
                    )
                break
            if decision["decision"] == "SKIP_SATISFIED":
                results.append({
                    "action_id": proposal.action_id,
                    "return_code": 0,
                    "dispatch_skipped": True,
                    "outcome": {
                        "verdict": "VERIFIED",
                        "reason": decision["reason"],
                        "independently_observed": True,
                    },
                })
                continue
        outcome = args.evidence_dir / f"{index:04d}-{proposal.action_id}-outcome.json"
        code = _dispatch(proposal, outcome)
        record = json.loads(outcome.read_text()) if outcome.is_file() else {
            "verdict": "UNCONFIRMED", "reason": "missing_outcome",
        }
        results.append({"action_id": proposal.action_id, "return_code": code,
                        "outcome": record})
        if code != 0:
            if recovery_action is not None and _needs_takeoff_recovery(proposal, record):
                recovery_result = _run_recovery(
                    recovery_action, args.evidence_dir, proposal.action_id
                )
            break
    complete = len(results) == len(proposals) and all(item["return_code"] == 0 for item in results)
    recovered = bool(
        recovery_result
        and recovery_result["return_code"] == 0
        and recovery_result["outcome"].get("verdict") == "VERIFIED"
    )
    status = ("VERIFIED" if complete else "RECOVERED_HALT" if recovered
              else "RECOVERY_FAILED" if recovery_result is not None else "HALTED")
    mission = {
        "schema_version": "rrm-airstack-command-outcome/v1",
        "completed_at": datetime.now(timezone.utc).isoformat(),
        "plan_sha256": preview["plan_sha256"],
        "status": status,
        "results": results,
        "replans": replans,
        "recovery": recovery_result,
        "execution_dispatch": True,
    }
    (args.evidence_dir / "mission-outcome.json").write_text(
        json.dumps(mission, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(mission, sort_keys=True), flush=True)
    return 0 if complete else 4


if __name__ == "__main__":
    raise SystemExit(main())
