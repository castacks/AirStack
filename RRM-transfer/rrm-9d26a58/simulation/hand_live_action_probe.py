#!/usr/bin/env python3
"""Live Isaac action/stop probe for Gate 4 evidence.

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
from typing import Any

from simulation.hand_live_binding_probe import compare_profile, verify_artifacts


SCHEMA_VERSION = "rrm-hand-live-action-probe/v2"


def _ledger_events(path: Path) -> list[str]:
    return [json.loads(line)["event"] for line in path.read_text().splitlines()]


def _has_ordered_events(events: list[str], required: tuple[str, ...]) -> bool:
    position = 0
    for event in events:
        if position < len(required) and event == required[position]:
            position += 1
    return position == len(required)


def sample_until_safe(*, world: object, adapter: object, generation: int,
                      max_steps: int) -> tuple[object | None, int]:
    """Step and sample until measured safe evidence exists or the budget expires."""
    for step in range(1, max_steps + 1):
        world.step(render=False)
        evidence = adapter.sample(generation)
        if evidence is not None:
            return evidence, step
    return None, max_steps


def validate_action_evidence(*, profile_mismatches: list[str],
        qualified_scene_equivalent: bool, target_status: str,
        target_motion_observed: bool,
        stop_accepted: bool, stop_status: str, stop_liveness: dict[str, Any],
        required_safe_count: int, stop_initial_safe_count: int,
        stop_safe_count: int, stop_events: list[str], watchdog_reason: str,
        watchdog_status: str, watchdog_safe_count: int,
        watchdog_initial_safe_count: int,
        watchdog_target_motion_observed: bool,
        watchdog_events: list[str]) -> list[str]:
    """Return fail-closed errors for the evidence emitted by the live run."""
    errors: list[str] = []
    if profile_mismatches:
        errors.append("profile_mismatch")
    if qualified_scene_equivalent is not True:
        errors.append("qualified_scene_not_recreated")
    if target_status != "TARGET_APPLIED":
        errors.append("target_not_applied")
    if target_motion_observed is not True:
        errors.append("target_motion_not_observed")
    if stop_initial_safe_count < required_safe_count:
        errors.append("stop_initial_safe_state_unconfirmed")
    if stop_accepted is not True:
        errors.append("stop_not_accepted")
    if stop_status != "HOLD_APPLIED":
        errors.append("stop_hold_not_applied")
    if stop_liveness.get("stop_pending") is not False or \
            stop_liveness.get("stop_to_hold_s") is None or \
            stop_liveness.get("stop_to_hold_s", math.inf) > \
            stop_liveness.get("max_stop_to_hold_s", -math.inf):
        errors.append("stop_deadline_not_met")
    if stop_safe_count < required_safe_count:
        errors.append("stop_safe_state_unconfirmed")
    if not _has_ordered_events(stop_events, ("C09_ADAPTER_ENQUEUE",
            "C09_ADAPTER_APPLY_INTENT", "C08_ADAPTER_STOP_REQUEST",
            "C08_ADAPTER_HOLD_APPLIED")):
        errors.append("stop_ledger_sequence_missing")
    if watchdog_reason != "TICK_STALE":
        errors.append("watchdog_did_not_fence")
    if watchdog_target_motion_observed is not True:
        errors.append("watchdog_target_motion_not_observed")
    if watchdog_initial_safe_count < required_safe_count:
        errors.append("watchdog_initial_safe_state_unconfirmed")
    if watchdog_status != "HOLD_APPLIED":
        errors.append("watchdog_hold_not_applied")
    if watchdog_safe_count < required_safe_count:
        errors.append("watchdog_safe_state_unconfirmed")
    if not _has_ordered_events(watchdog_events, ("C09_ADAPTER_ENQUEUE",
            "C09_ADAPTER_APPLY_INTENT", "C08_ADAPTER_LIVENESS_FAULT")):
        errors.append("watchdog_ledger_sequence_missing")
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
    stop_ledger = args.output.parent / "live-stop-fence-ledger.jsonl"
    watchdog_ledger = args.output.parent / "live-watchdog-ledger.jsonl"
    for path in (args.output, stop_ledger, watchdog_ledger):
        if path.exists():
            raise FileExistsError(path)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    adapters = []
    fixture_attestation = None
    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils.types import ArticulationAction
        from simulation.hand_isaac_adapter import IsaacHandAdapter
        from simulation.hand_live_qualified_fixture import build_qualified_fixture
        from rrm.hand_execution_boundary import HandCommand

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

        def make_adapter(ledger_path: Path, episode_id: str, max_tick_gap_s: float):
            adapter = IsaacHandAdapter(articulation=hand,
                action_factory=ArticulationAction, ledger_path=ledger_path,
                episode_id=episode_id,
                scene_recipe_sha256=probe["scene_recipe_sha256"],
                joint_names=names,
                lower_rad=tuple(item["lower_rad"] for item in live),
                upper_rad=tuple(item["upper_rad"] for item in live),
                max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
                object_speeds=fixture.object_speeds,
                controller_mode=lambda: "POSITION_HOLD",
                external_motion_active=lambda: False, motion_enabled=True,
                joint_safe_threshold_rad_s=safe_state["joint_velocity_threshold_rad_s"],
                object_safe_threshold_m_s=safe_state["object_velocity_threshold_m_s"],
                safe_window=safe_state["consecutive_window_required"],
                max_tick_gap_s=max_tick_gap_s)
            adapters.append(adapter)
            return adapter

        qualification_sha256 = hashlib.sha256(qualification_bytes).hexdigest()
        probe_sha256 = qualification["probe_sha256"]

        stop_adapter = make_adapter(stop_ledger, "live-action-probe-stop", 0.5)
        stop_initial_evidence, stop_initial_safe_steps = sample_until_safe(world=world,
            adapter=stop_adapter, generation=0,
            max_steps=args.safe_state_max_steps)
        if stop_initial_evidence is None:
            raise RuntimeError("stop scenario initial safe-state window not established")
        stop_statuses: list[str] = []
        world.add_physics_callback("rrm_stop_callback",
            lambda _dt: stop_statuses.append(stop_adapter.tick()))
        index = names.index("iiwa7_joint_1")
        observed = float(hand.get_joint_positions()[index])
        command = HandCommand("iiwa7_joint_1", observed, observed + 0.01,
            time.monotonic(), "live-action-probe-stop",
            probe["scene_recipe_sha256"], probe_sha256, qualification_sha256)
        stop_adapter.submit("dispatch-stop-1", 1, command)
        world.step(render=False)
        target_status = stop_statuses[-1]
        stop_positions = [float(hand.get_joint_positions()[index])]
        stop_velocities = [float(hand.get_joint_velocities()[index])]
        for _ in range(args.motion_observation_steps - 1):
            world.step(render=False)
            stop_positions.append(float(hand.get_joint_positions()[index]))
            stop_velocities.append(float(hand.get_joint_velocities()[index]))
        stop_motion = {"initial_position_rad": observed,
            "target_position_rad": command.target_position_rad,
            "observed_positions_rad": stop_positions,
            "observed_velocities_rad_s": stop_velocities,
            "max_displacement_rad": max(abs(value - observed)
                                        for value in stop_positions),
            "max_target_progress_rad": abs(command.target_position_rad - observed) -
                min(abs(command.target_position_rad - value)
                    for value in stop_positions),
            "max_velocity_rad_s": max(abs(value) for value in stop_velocities)}
        stop_motion_observed = stop_motion["max_target_progress_rad"] > 1e-5
        stop_accepted = stop_adapter.request_stop(2)
        world.step(render=False)
        stop_status = stop_statuses[-1]
        stop_evidence, stop_safe_steps = sample_until_safe(world=world,
            adapter=stop_adapter, generation=2, max_steps=args.safe_state_max_steps)
        stop_liveness = stop_adapter.liveness().__dict__
        world.remove_physics_callback("rrm_stop_callback")

        world.reset()
        for _ in range(30):
            world.step(render=False)
        watchdog_adapter = make_adapter(watchdog_ledger,
            "live-action-probe-watchdog", 0.2)
        watchdog_initial_evidence, watchdog_initial_safe_steps = sample_until_safe(
            world=world, adapter=watchdog_adapter, generation=0,
            max_steps=args.safe_state_max_steps)
        if watchdog_initial_evidence is None:
            raise RuntimeError("watchdog scenario initial safe-state window not established")
        watchdog_statuses: list[str] = []
        world.add_physics_callback("rrm_watchdog_callback",
            lambda _dt: watchdog_statuses.append(watchdog_adapter.tick()))
        index = names.index("iiwa7_joint_2")
        observed = float(hand.get_joint_positions()[index])
        command = HandCommand("iiwa7_joint_2", observed, observed + 0.01,
            time.monotonic(), "live-action-probe-watchdog",
            probe["scene_recipe_sha256"], probe_sha256, qualification_sha256)
        watchdog_adapter.submit("dispatch-watchdog-1", 1, command)
        world.step(render=False)
        if watchdog_statuses[-1] != "TARGET_APPLIED":
            raise RuntimeError("watchdog scenario target was not applied")
        watchdog_positions = [float(hand.get_joint_positions()[index])]
        watchdog_velocities = [float(hand.get_joint_velocities()[index])]
        for _ in range(args.motion_observation_steps - 1):
            world.step(render=False)
            watchdog_positions.append(float(hand.get_joint_positions()[index]))
            watchdog_velocities.append(float(hand.get_joint_velocities()[index]))
        watchdog_motion = {"initial_position_rad": observed,
            "target_position_rad": command.target_position_rad,
            "observed_positions_rad": watchdog_positions,
            "observed_velocities_rad_s": watchdog_velocities,
            "max_displacement_rad": max(abs(value - observed)
                                        for value in watchdog_positions),
            "max_target_progress_rad": abs(command.target_position_rad - observed) -
                min(abs(command.target_position_rad - value)
                    for value in watchdog_positions),
            "max_velocity_rad_s": max(abs(value) for value in watchdog_velocities)}
        watchdog_motion_observed = watchdog_motion["max_target_progress_rad"] > 1e-5
        watchdog_adapter.start_watchdog(interval_s=0.02)
        deadline = time.monotonic() + 1.0
        watchdog_reason = "HEALTHY"
        while time.monotonic() < deadline:
            watchdog_reason = watchdog_adapter.liveness().reason
            if watchdog_reason == "TICK_STALE" and not watchdog_adapter.motion_enabled:
                break
            time.sleep(0.01)
        world.step(render=False)
        watchdog_status = watchdog_statuses[-1]
        watchdog_evidence, watchdog_safe_steps = sample_until_safe(world=world,
            adapter=watchdog_adapter, generation=1,
            max_steps=args.safe_state_max_steps)
        world.remove_physics_callback("rrm_watchdog_callback")
        watchdog_adapter.stop_watchdog()

        stop_events = _ledger_events(stop_ledger)
        watchdog_events = _ledger_events(watchdog_ledger)
        qualified_scene_equivalent = fixture_attestation["qualified_scene_equivalent"]
        errors = validate_action_evidence(profile_mismatches=mismatches,
            qualified_scene_equivalent=qualified_scene_equivalent,
            target_status=target_status, target_motion_observed=stop_motion_observed,
            stop_accepted=stop_accepted,
            stop_status=stop_status, stop_liveness=stop_liveness,
            required_safe_count=safe_state["consecutive_window_required"],
            stop_initial_safe_count=stop_initial_evidence.consecutive_safe_count,
            stop_safe_count=0 if stop_evidence is None else
                stop_evidence.consecutive_safe_count,
            stop_events=stop_events, watchdog_reason=watchdog_reason,
            watchdog_status=watchdog_status,
            watchdog_target_motion_observed=watchdog_motion_observed,
            watchdog_initial_safe_count=
                watchdog_initial_evidence.consecutive_safe_count,
            watchdog_safe_count=0 if watchdog_evidence is None else
                watchdog_evidence.consecutive_safe_count,
            watchdog_events=watchdog_events)
        report = {"schema_version": SCHEMA_VERSION,
            "probe_sha256": hashlib.sha256(probe_bytes).hexdigest(),
            "qualification_sha256": qualification_sha256,
            "profile_mismatches": mismatches,
            "qualified_scene_equivalent": qualified_scene_equivalent,
            "fixture_attestation": fixture_attestation,
            "stop": {"target_tick_status": target_status,
                "target_motion_observed": stop_motion_observed,
                "target_motion": stop_motion,
                "initial_safe_state_steps": stop_initial_safe_steps,
                "initial_safe_state": stop_initial_evidence.__dict__,
                "stop_accepted": stop_accepted, "hold_tick_status": stop_status,
                "liveness": stop_liveness,
                "safe_state_steps": stop_safe_steps,
                "safe_state": None if stop_evidence is None else stop_evidence.__dict__,
                "ledger_events": stop_events},
            "watchdog": {"reason": watchdog_reason,
                "target_tick_status": watchdog_statuses[0],
                "target_motion_observed": watchdog_motion_observed,
                "target_motion": watchdog_motion,
                "initial_safe_state_steps": watchdog_initial_safe_steps,
                "initial_safe_state": watchdog_initial_evidence.__dict__,
                "hold_tick_status": watchdog_status,
                "safe_state_steps": watchdog_safe_steps,
                "safe_state": None if watchdog_evidence is None else
                    watchdog_evidence.__dict__, "ledger_events": watchdog_events},
            "validation_errors": errors,
            "gate_4_live_probe_passed": not errors}
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return 0 if not errors else 1
    except Exception as exc:
        emergency_holds = []
        for adapter in reversed(adapters):
            try:
                accepted = adapter.request_stop(1_000_000)
                status = adapter.tick()
                emergency_holds.append({"stop_accepted": accepted,
                                        "tick_status": status})
            except Exception as cleanup_exc:
                emergency_holds.append({"cleanup_exception":
                                        type(cleanup_exc).__name__})
        if not args.output.exists():
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump({"schema_version": SCHEMA_VERSION,
                    "probe_exception": type(exc).__name__,
                    "probe_exception_message": traceback.format_exc(),
                    "fixture_attestation": fixture_attestation,
                    "emergency_holds": emergency_holds,
                    "gate_4_live_probe_passed": False}, stream,
                    indent=2, sort_keys=True)
                stream.write("\n")
        return 1
    finally:
        for adapter in adapters:
            adapter.stop_watchdog()
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
