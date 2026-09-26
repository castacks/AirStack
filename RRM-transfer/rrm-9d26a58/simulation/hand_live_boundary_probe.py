#!/usr/bin/env python3
"""Live Isaac execution-boundary probe for Gate 5 evidence.

This script applies motion and must only be run under the hand-command approval
described in the hand embodiment decision. Importing it is CPU-only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import traceback

from simulation.hand_live_action_probe import (
    _has_ordered_events,
    _ledger_events,
    sample_until_safe,
)
from simulation.hand_live_binding_probe import compare_profile, verify_artifacts


SCHEMA_VERSION = "rrm-hand-live-boundary-probe/v2"


def validate_boundary_evidence(*, profile_mismatches: list[str],
        qualified_scene_equivalent: bool, reset_succeeded: bool,
        target_status: str, target_motion_observed: bool,
        stop_accepted: bool, hold_status: str,
        stopped_confirmed: bool, required_safe_count: int, safe_count: int,
        boundary_events: list[str],
        adapter_events: list[str]) -> list[str]:
    """Return fail-closed errors for boundary-to-adapter live evidence."""
    errors: list[str] = []
    if profile_mismatches:
        errors.append("profile_mismatch")
    if qualified_scene_equivalent is not True:
        errors.append("qualified_scene_not_recreated")
    if reset_succeeded is not True:
        errors.append("authorized_reset_failed")
    if target_status != "TARGET_APPLIED":
        errors.append("target_not_applied")
    if target_motion_observed is not True:
        errors.append("target_motion_not_observed")
    if stop_accepted is not True:
        errors.append("boundary_stop_not_accepted")
    if hold_status != "HOLD_APPLIED":
        errors.append("hold_not_applied")
    if safe_count < required_safe_count or stopped_confirmed is not True:
        errors.append("stopped_state_not_confirmed")
    if not _has_ordered_events(boundary_events, ("C06_AUTHORIZATION_CONSUMED",
            "C08_RESET", "C06_AUTHORIZATION_CONSUMED", "C09_DISPATCH_INTENT",
            "C08_STOP_RECEIVED", "C08_CANCEL_ACCEPTED", "C08_MOTION_STOPPED",
            "C08_SAFE_CONFIRMED", "C09_DISPATCH_RECONCILED")):
        errors.append("boundary_ledger_sequence_missing")
    if not _has_ordered_events(adapter_events, ("C09_ADAPTER_ENQUEUE",
            "C09_ADAPTER_APPLY_INTENT", "C08_ADAPTER_STOP_REQUEST",
            "C08_ADAPTER_HOLD_APPLIED")):
        errors.append("adapter_ledger_sequence_missing")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--physics-dt-s", type=float, default=1.0 / 120.0)
    parser.add_argument("--safe-state-max-steps", type=int, default=240)
    parser.add_argument("--motion-observation-steps", type=int, default=3)
    args = parser.parse_args()
    if not math.isfinite(args.physics_dt_s) or not 0 < args.physics_dt_s <= 0.1:
        raise ValueError("invalid_physics_dt")
    if not 5 <= args.safe_state_max_steps <= 2000:
        raise ValueError("invalid_safe_state_step_budget")
    if not 1 <= args.motion_observation_steps <= 10:
        raise ValueError("invalid_motion_observation_step_budget")

    probe_bytes = args.probe.read_bytes()
    qualification_bytes = args.qualification.read_bytes()
    probe, qualification = verify_artifacts(probe_bytes, qualification_bytes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    adapter_ledger = args.output.parent / "live-boundary-adapter-ledger.jsonl"
    boundary_ledger = args.output.parent / "live-boundary-decision-ledger.jsonl"
    for path in (args.output, adapter_ledger, boundary_ledger):
        if path.exists():
            raise FileExistsError(path)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    adapter = None
    fixture_attestation = None
    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils.types import ArticulationAction
        from simulation.hand_isaac_adapter import IsaacHandAdapter
        from simulation.hand_live_qualified_fixture import build_qualified_fixture
        from rrm.contracts import DispatchContext, SafetyDecision
        from rrm.hand_execution_boundary import (
            DurableJournal,
            HandAuthorityVerifier,
            HandCommand,
            HandExecutionBoundary,
            dispatch_authorization_scope,
            issue_hand_authorization,
            reset_authorization_scope,
        )

        world = World(stage_units_in_meters=1.0, physics_dt=args.physics_dt_s)
        fixture = build_qualified_fixture(app=app, world=world, probe=probe,
            requested_physics_dt_s=args.physics_dt_s)
        fixture_attestation = fixture.attestation
        if not fixture_attestation["qualified_scene_equivalent"]:
            raise RuntimeError(f"qualified scene mismatch: {fixture_attestation}")
        hand = fixture.hand
        names = tuple(hand.dof_names)
        live = fixture.live_profile
        mismatches = compare_profile(probe, live)
        if mismatches:
            raise RuntimeError(f"live profile mismatch: {mismatches}")
        safe_state = fixture.safe_state

        adapter = IsaacHandAdapter(articulation=hand,
            action_factory=ArticulationAction, ledger_path=adapter_ledger,
            episode_id="live-boundary-probe",
            scene_recipe_sha256=probe["scene_recipe_sha256"], joint_names=names,
            lower_rad=tuple(item["lower_rad"] for item in live),
            upper_rad=tuple(item["upper_rad"] for item in live),
            max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
            object_speeds=fixture.object_speeds,
            controller_mode=lambda: "POSITION_HOLD",
            external_motion_active=lambda: False, motion_enabled=True,
            joint_safe_threshold_rad_s=safe_state["joint_velocity_threshold_rad_s"],
            object_safe_threshold_m_s=safe_state["object_velocity_threshold_m_s"],
            safe_window=safe_state["consecutive_window_required"],
            max_tick_gap_s=0.5)
        reset_evidence, reset_safe_steps = sample_until_safe(world=world,
            adapter=adapter, generation=0, max_steps=args.safe_state_max_steps)
        if reset_evidence is None:
            raise RuntimeError("initial safe-state window not established")

        secret = b"rrm-live-boundary-probe-key-32-bytes"
        verifier = HandAuthorityVerifier(issuer_keys={"probe-issuer": secret},
            allowed_roles=frozenset({"CALIBRATION"}))
        boundary = HandExecutionBoundary(adapter=adapter,
            journal=DurableJournal(boundary_ledger), authority_verifier=verifier,
            qualification_bytes=qualification_bytes, probe_bytes=probe_bytes,
            max_delta_rad=0.02)
        now = time.monotonic()
        reset_scope = reset_authorization_scope(generation=0, evidence=reset_evidence,
            qualification_sha256=boundary.qualification_sha256,
            probe_sha256=qualification["probe_sha256"])
        reset_grant = issue_hand_authorization(signing_key=secret,
            authorization_id="reset-authorization-1", issuer_id="probe-issuer",
            subject_id="probe-operator", role="CALIBRATION", purpose="RESET",
            authority_epoch=boundary.epoch, stop_generation=0,
            scope_digest=reset_scope, issued_at_monotonic=now - 0.01,
            expires_at_monotonic=now + 5.0)
        reset_succeeded = boundary.reset(authorization=reset_grant,
            generation=0, now=now)
        if not reset_succeeded:
            raise RuntimeError("authorized boundary reset failed")

        index = names.index("iiwa7_joint_1")
        observed = float(hand.get_joint_positions()[index])
        command = HandCommand("iiwa7_joint_1", observed, observed + 0.01,
            time.monotonic(), "live-boundary-probe",
            probe["scene_recipe_sha256"], qualification["probe_sha256"],
            hashlib.sha256(qualification_bytes).hexdigest())
        context = DispatchContext("live-boundary-run", "task-v1", "plan-v1",
            "calibration-action-1", "dispatch-boundary-1", command.digest,
            "state-v1", "capability-v1", "permission-v1", "approval-v1",
            "constraints-v1", boundary.epoch, boundary.generation)
        decision_now = time.monotonic()
        decision = SafetyDecision("decision-boundary-1", context, "ALLOW",
            decision_now - 0.01, decision_now + 5.0)
        dispatch_scope = dispatch_authorization_scope(decision=decision,
            current=context, command=command,
            qualification_sha256=boundary.qualification_sha256,
            probe_sha256=qualification["probe_sha256"])
        dispatch_grant = issue_hand_authorization(signing_key=secret,
            authorization_id="dispatch-authorization-1", issuer_id="probe-issuer",
            subject_id="probe-operator", role="CALIBRATION", purpose="DISPATCH",
            authority_epoch=boundary.epoch, stop_generation=boundary.generation,
            scope_digest=dispatch_scope, issued_at_monotonic=decision_now - 0.01,
            expires_at_monotonic=decision_now + 5.0)
        boundary.dispatch(decision, context, command,
            authorization=dispatch_grant, now=decision_now)

        statuses: list[str] = []
        world.add_physics_callback("rrm_boundary_callback",
            lambda _dt: statuses.append(adapter.tick()))
        world.step(render=False)
        target_status = statuses[-1]
        target_positions = [float(hand.get_joint_positions()[index])]
        target_velocities = [float(hand.get_joint_velocities()[index])]
        for _ in range(args.motion_observation_steps - 1):
            world.step(render=False)
            target_positions.append(float(hand.get_joint_positions()[index]))
            target_velocities.append(float(hand.get_joint_velocities()[index]))
        target_motion = {"initial_position_rad": observed,
            "target_position_rad": command.target_position_rad,
            "observed_positions_rad": target_positions,
            "observed_velocities_rad_s": target_velocities,
            "max_displacement_rad": max(abs(value - observed)
                                        for value in target_positions),
            "max_target_progress_rad": abs(command.target_position_rad - observed) -
                min(abs(command.target_position_rad - value)
                    for value in target_positions),
            "max_velocity_rad_s": max(abs(value) for value in target_velocities)}
        target_motion_observed = target_motion["max_target_progress_rad"] > 1e-5
        stop_accepted = boundary.stop(intervention_id="live-boundary-stop-1",
            reason="bounded probe action completed")
        world.step(render=False)
        hold_status = statuses[-1]
        stopped_evidence, stopped_safe_steps = sample_until_safe(world=world,
            adapter=adapter, generation=boundary.generation,
            max_steps=args.safe_state_max_steps)
        stopped_confirmed = boundary.confirm_stopped(generation=boundary.generation,
            now=time.monotonic())
        world.remove_physics_callback("rrm_boundary_callback")

        boundary_events = _ledger_events(boundary_ledger)
        adapter_events = _ledger_events(adapter_ledger)
        qualified_scene_equivalent = fixture_attestation["qualified_scene_equivalent"]
        errors = validate_boundary_evidence(profile_mismatches=mismatches,
            qualified_scene_equivalent=qualified_scene_equivalent,
            reset_succeeded=reset_succeeded, target_status=target_status,
            target_motion_observed=target_motion_observed,
            stop_accepted=stop_accepted, hold_status=hold_status,
            stopped_confirmed=stopped_confirmed,
            required_safe_count=safe_state["consecutive_window_required"],
            safe_count=0 if stopped_evidence is None else
                stopped_evidence.consecutive_safe_count,
            boundary_events=boundary_events, adapter_events=adapter_events)
        report = {"schema_version": SCHEMA_VERSION,
            "probe_sha256": hashlib.sha256(probe_bytes).hexdigest(),
            "qualification_sha256": hashlib.sha256(qualification_bytes).hexdigest(),
            "profile_mismatches": mismatches,
            "qualified_scene_equivalent": qualified_scene_equivalent,
            "fixture_attestation": fixture_attestation,
            "authorized_reset_succeeded": reset_succeeded,
            "reset_safe_state_steps": reset_safe_steps,
            "target_tick_status": target_status,
            "target_motion_observed": target_motion_observed,
            "target_motion": target_motion, "stop_accepted": stop_accepted,
            "hold_tick_status": hold_status,
            "stopped_safe_state_steps": stopped_safe_steps,
            "stopped_state_confirmed": stopped_confirmed,
            "safe_state": None if stopped_evidence is None else
                stopped_evidence.__dict__, "boundary_ledger_events": boundary_events,
            "adapter_ledger_events": adapter_events,
            "validation_errors": errors,
            "gate_5_live_probe_passed": not errors}
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return 0 if not errors else 1
    except Exception as exc:
        emergency_hold = None
        if adapter is not None:
            try:
                accepted = adapter.request_stop(1_000_000)
                status = adapter.tick()
                emergency_hold = {"stop_accepted": accepted, "tick_status": status}
            except Exception as cleanup_exc:
                emergency_hold = {"cleanup_exception": type(cleanup_exc).__name__}
        if not args.output.exists():
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump({"schema_version": SCHEMA_VERSION,
                    "probe_exception": type(exc).__name__,
                    "probe_exception_message": traceback.format_exc(),
                    "fixture_attestation": fixture_attestation,
                    "emergency_hold": emergency_hold,
                    "gate_5_live_probe_passed": False}, stream,
                    indent=2, sort_keys=True)
                stream.write("\n")
        return 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
