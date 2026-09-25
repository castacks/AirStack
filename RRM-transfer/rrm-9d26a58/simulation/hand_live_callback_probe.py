#!/usr/bin/env python3
"""No-action live Isaac physics-callback heartbeat probe.

This advances only a separate zero-gravity World.  The hand gateway remains disabled,
and a hard wrapper raises on any articulation action.  This is not stop, safe-state,
controller, contact, or motion qualification.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import os
import time
from typing import Any, Callable

from simulation.hand_live_binding_probe import (
    _NoActionArticulation,
    compare_profile,
    verify_artifacts,
)


def _timing_summary(values: list[float]) -> dict[str, float | None]:
    ordered = sorted(values)
    p95 = ordered[min(len(ordered) - 1,
        math.ceil(len(ordered) * 0.95) - 1)] if ordered else None
    return {"min": min(values) if values else None,
            "median": statistics.median(values) if values else None,
            "p95": p95, "max": max(values) if values else None}


class PhysicsCallbackRecorder:
    """Collect gateway ticks from the Isaac physics-callback context."""

    def __init__(self, adapter: object, *, asset_is_active: Callable[[], bool],
                 clock: Callable[[], float] = time.monotonic):
        self._adapter = adapter
        self._asset_is_active = asset_is_active
        self._clock = clock
        self.statuses: list[str] = []
        self.physics_dts_s: list[float] = []
        self.callback_gaps_s: list[float] = []
        self.callback_durations_s: list[float] = []
        self.asset_active: list[bool] = []
        self._previous_started_at: float | None = None

    def __call__(self, physics_dt: float) -> None:
        started_at = self._clock()
        if self._previous_started_at is not None:
            self.callback_gaps_s.append(started_at - self._previous_started_at)
        self._previous_started_at = started_at
        self.physics_dts_s.append(float(physics_dt))
        self.asset_active.append(bool(self._asset_is_active()))
        self.statuses.append(self._adapter.tick(now=started_at))
        self.callback_durations_s.append(self._clock() - started_at)


def validate_callback_run(*, statuses: list[str], physics_dts_s: list[float],
                          callback_gaps_s: list[float], asset_active: list[bool],
                          evidence: dict[str, Any] | None,
                          apply_action_calls: int, expected_steps: int,
                          expected_physics_dt_s: float, max_callback_gap_s: float,
                          initial_max_velocity_rad_s: float,
                          allowed_velocity_rad_s: float) -> list[str]:
    """Return fail-closed callback errors; empty does not authorize motion."""
    errors: list[str] = []
    if len(statuses) != expected_steps or expected_steps < 1:
        errors.append("callback_count_mismatch")
    if any(status != "IDLE" for status in statuses):
        errors.append("non_idle_callback")
    if len(physics_dts_s) != expected_steps or any(
            not math.isfinite(dt) or abs(dt - expected_physics_dt_s) > 1e-9
            for dt in physics_dts_s):
        errors.append("physics_dt_mismatch")
    if len(callback_gaps_s) != max(0, expected_steps - 1) or any(
            not math.isfinite(gap) or gap < 0 or gap > max_callback_gap_s
            for gap in callback_gaps_s):
        errors.append("callback_gap_exceeded")
    if len(asset_active) != expected_steps or any(asset_active):
        errors.append("asset_active_during_callback")
    if evidence is None or evidence.get("tick_count") != expected_steps:
        errors.append("heartbeat_tick_count_mismatch")
    if evidence is None or evidence.get("healthy") is not True or \
            evidence.get("reason") != "HEALTHY":
        errors.append("heartbeat_unhealthy")
    if evidence is None or evidence.get("motion_enabled") is not False:
        errors.append("motion_not_inhibited")
    if apply_action_calls != 0:
        errors.append("action_call_observed")
    if not isinstance(initial_max_velocity_rad_s, (int, float)) or \
            isinstance(initial_max_velocity_rad_s, bool) or \
            not math.isfinite(initial_max_velocity_rad_s) or \
            initial_max_velocity_rad_s < 0:
        errors.append("invalid_joint_state_measurement")
    elif initial_max_velocity_rad_s > allowed_velocity_rad_s:
        errors.append("joint_velocity_observed")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--qualification", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--physics-steps", type=int, default=240)
    parser.add_argument("--physics-dt-s", type=float, default=1 / 120)
    parser.add_argument("--max-callback-gap-s", type=float, default=0.10)
    parser.add_argument("--max-joint-velocity-rad-s", type=float, default=1e-6)
    args = parser.parse_args()
    finite_positive = (args.physics_dt_s, args.max_callback_gap_s,
                       args.max_joint_velocity_rad_s)
    if not 1 <= args.physics_steps <= 10000 or any(
            not math.isfinite(value) or value <= 0 for value in finite_positive) or \
            args.physics_dt_s > 0.1 or args.max_callback_gap_s > 5.0:
        raise ValueError("invalid_callback_probe_bounds")
    probe_bytes = args.probe.read_bytes()
    qualification_bytes = args.qualification.read_bytes()
    probe, _ = verify_artifacts(probe_bytes, qualification_bytes)
    if args.output.exists():
        raise FileExistsError(args.output)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    guarded = None
    try:
        import numpy as np
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.core.utils.types import ArticulationAction
        from simulation.hand_controller_probe import ASSET_URL
        from simulation.hand_isaac_adapter import IsaacHandAdapter

        if probe.get("asset_url") != ASSET_URL:
            raise ValueError("asset_reference_changed")
        world = World(stage_units_in_meters=1.0, physics_dt=args.physics_dt_s)
        world.get_physics_context().set_gravity(0.0)
        add_reference_to_stage(usd_path=ASSET_URL, prim_path="/World/KukaAllegro")
        app.update()
        roots = [str(prim.GetPath()) for prim in get_current_stage().Traverse()
                 if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                 and str(prim.GetPath()).startswith("/World/KukaAllegro")]
        if len(roots) != 1:
            raise RuntimeError(f"expected one hand articulation, got {roots}")
        hand = world.scene.add(SingleArticulation(prim_path=roots[0], name="kuka_allegro"))
        world.reset()
        names = tuple(hand.dof_names)
        limits = np.asarray(hand._articulation_view.get_dof_limits(), dtype=float)
        if limits.ndim == 3:
            limits = limits[0]
        properties = hand.dof_properties
        live = [{"index": index, "name": name,
                 "lower_rad": float(limits[index, 0]),
                 "upper_rad": float(limits[index, 1]),
                 "max_velocity_rad_s": float(properties[index]["maxVelocity"])}
                for index, name in enumerate(names)]
        mismatches = compare_profile(probe, live)
        guarded = _NoActionArticulation(hand)
        adapter = None
        recorder = None
        initial_positions = np.asarray(hand.get_joint_positions(), dtype=float)
        initial_velocities = np.asarray(hand.get_joint_velocities(), dtype=float)
        initial_max_velocity = float(np.max(np.abs(initial_velocities)))
        step_durations_s: list[float] = []
        precheck_errors: list[str] = []
        if mismatches:
            precheck_errors.append("profile_mismatch")
        if not np.all(np.isfinite(initial_positions)) or \
                not np.all(np.isfinite(initial_velocities)):
            precheck_errors.append("invalid_initial_joint_state")
        elif initial_max_velocity > args.max_joint_velocity_rad_s:
            precheck_errors.append("initial_joint_velocity_observed")

        if not precheck_errors:
            adapter = IsaacHandAdapter(articulation=guarded,
                action_factory=ArticulationAction,
                ledger_path=args.output.parent / "disabled-callback-gateway-ledger.jsonl",
                episode_id="physics-callback-smoke-only",
                scene_recipe_sha256=probe["scene_recipe_sha256"],
                joint_names=names,
                lower_rad=tuple(item["lower_rad"] for item in live),
                upper_rad=tuple(item["upper_rad"] for item in live),
                max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
                object_speeds=lambda: (), controller_mode=lambda: "UNKNOWN",
                external_motion_active=lambda: True, motion_enabled=False,
                max_tick_gap_s=args.max_callback_gap_s)
            asset_prim = get_current_stage().GetPrimAtPath("/World/KukaAllegro")
            # World must not retain a managed articulation whose prim is inactive.
            # The disabled adapter keeps only an inert reference and IDLE never reads it.
            world.scene.remove_object("kuka_allegro", registry_only=True)
            asset_prim.SetActive(False)
            if asset_prim.IsActive():
                precheck_errors.append("asset_deactivation_failed")
            else:
                recorder = PhysicsCallbackRecorder(adapter,
                    asset_is_active=lambda: asset_prim.IsActive())
                world.add_physics_callback("rrm_disabled_gateway_heartbeat", recorder)
                for _ in range(args.physics_steps):
                    started_at = time.monotonic()
                    world.step(render=False)
                    step_durations_s.append(time.monotonic() - started_at)
                world.remove_physics_callback("rrm_disabled_gateway_heartbeat")

        evidence = None if adapter is None else adapter.liveness().__dict__
        callback_errors = list(precheck_errors)
        if recorder is not None:
            callback_errors.extend(validate_callback_run(statuses=recorder.statuses,
                physics_dts_s=recorder.physics_dts_s,
                callback_gaps_s=recorder.callback_gaps_s,
                asset_active=recorder.asset_active, evidence=evidence,
                apply_action_calls=guarded.apply_calls,
                expected_steps=args.physics_steps,
                expected_physics_dt_s=args.physics_dt_s,
                max_callback_gap_s=args.max_callback_gap_s,
                initial_max_velocity_rad_s=initial_max_velocity,
                allowed_velocity_rad_s=args.max_joint_velocity_rad_s))

        report = {
            "schema_version": "rrm-hand-live-physics-callback/v2",
            "joint_count": len(live), "profile_mismatches": mismatches,
            "zero_gravity_fixture": True,
            "asset_deactivated_before_callbacks": recorder is not None,
            "asset_active_during_callbacks": [] if recorder is None else
                sorted(set(recorder.asset_active)),
            "physics_steps_requested": args.physics_steps,
            "physics_callbacks_completed": 0 if recorder is None else len(recorder.statuses),
            "physics_callback_statuses": [] if recorder is None else
                sorted(set(recorder.statuses)),
            "physics_dt_s": args.physics_dt_s,
            "callback_gap_s": _timing_summary([] if recorder is None else
                recorder.callback_gaps_s),
            "callback_duration_s": _timing_summary([] if recorder is None else
                recorder.callback_durations_s),
            "physics_step_duration_s": _timing_summary(step_durations_s),
            "initial_max_velocity_rad_s": initial_max_velocity,
            "allowed_velocity_rad_s": args.max_joint_velocity_rad_s,
            "gateway_liveness": evidence,
            "callback_errors": callback_errors,
            "apply_action_calls": guarded.apply_calls,
            "adapter_motion_enabled": None if adapter is None else adapter.motion_enabled,
            "execution_dispatch": False, "stop_requested": False,
            "hold_applied": False, "safe_state_qualified": False,
            "stop_qualified": False, "scheduler_under_load_qualified": False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        exit_code = 0 if not callback_errors and guarded.apply_calls == 0 else 1
    except Exception as exc:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not args.output.exists():
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump({"schema_version": "rrm-hand-live-physics-callback/v2",
                    "probe_exception": type(exc).__name__,
                    "probe_exception_message": str(exc),
                    "execution_dispatch": False,
                    "apply_action_calls": None if guarded is None else
                        guarded.apply_calls},
                    stream, indent=2, sort_keys=True)
                stream.write("\n")
        exit_code = 1
    finally:
        app.close()
    os._exit(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
