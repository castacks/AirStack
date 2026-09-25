#!/usr/bin/env python3
"""No-motion Isaac hand-profile smoke check in a separate headless process.

This is NOT the qualified tabletop/controller scene and does not test stop,
safe-state, contact, or execution. Never enables the adapter.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from rrm.hand_qualification import evaluate_hand_probe


EXPECTED_GATES = frozenset({
    "controller_limits", "post_command_reset", "contact_observer",
    "safe_state", "independent_stop",
})


def verify_artifacts(probe_bytes: bytes, qualification_bytes: bytes) -> tuple[dict, dict]:
    """Check exact raw-byte binding and recompute the strict evaluator gates."""
    probe, qualification = json.loads(probe_bytes), json.loads(qualification_bytes)
    probe_digest = hashlib.sha256(probe_bytes).hexdigest()
    recomputed = evaluate_hand_probe(probe, probe_sha256=probe_digest)
    if qualification != recomputed or \
            set(qualification.get("gates", {})) != EXPECTED_GATES or \
            not all(g.get("status") == "PASS" for g in qualification["gates"].values()) or \
            qualification.get("execution_dispatch") is not False:
        raise ValueError("strict_qualification_not_current_or_passing")
    return probe, qualification


def compare_profile(probe: dict[str, Any], live: list[dict[str, Any]],
                    *, tolerance: float = 1e-5) -> list[str]:
    """Return exact-index mismatches; empty list is only profile parity."""
    expected = probe.get("joint_limits")
    if not isinstance(expected, list) or len(expected) != 23 or len(live) != 23:
        return ["joint_count_not_23"]
    errors = []
    for index, (saved, current) in enumerate(zip(expected, live)):
        if saved.get("name") != current.get("name") or \
                saved.get("index") != index or current.get("index") != index:
            errors.append(f"joint_{index}_identity")
        for key in ("lower_rad", "upper_rad", "max_velocity_rad_s"):
            lhs, rhs = saved.get(key), current.get(key)
            if not all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in (lhs, rhs)) or abs(lhs - rhs) > tolerance:
                errors.append(f"joint_{index}_{key}")
    return errors


class _NoActionArticulation:
    """Forward reads but make even an accidental adapter action impossible."""

    def __init__(self, articulation: object):
        self._articulation = articulation
        self.apply_calls = 0

    def get_joint_positions(self):
        return self._articulation.get_joint_positions()

    def get_joint_velocities(self):
        return self._articulation.get_joint_velocities()

    def apply_action(self, action):
        self.apply_calls += 1
        raise RuntimeError("no_motion_probe_forbids_apply_action")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--qualification", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    probe_bytes, qualification_bytes = args.probe.read_bytes(), args.qualification.read_bytes()
    probe, qualification = verify_artifacts(probe_bytes, qualification_bytes)
    if args.output.exists():
        raise FileExistsError(args.output)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
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
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)
        add_reference_to_stage(usd_path=ASSET_URL, prim_path="/World/KukaAllegro")
        app.update()
        roots = [str(prim.GetPath()) for prim in get_current_stage().Traverse()
                 if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                 and str(prim.GetPath()).startswith("/World/KukaAllegro")]
        if len(roots) != 1:
            raise RuntimeError(f"expected one hand articulation, got {roots}")
        hand = world.scene.add(SingleArticulation(prim_path=roots[0], name="kuka_allegro"))
        world.reset()  # Builds articulation; no action or physics stepping follows.
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
        idle_tick = "NOT_RUN"
        if not mismatches:
            adapter = IsaacHandAdapter(articulation=guarded,
                action_factory=ArticulationAction,
                ledger_path=args.output.parent / "disabled-gateway-ledger.jsonl",
                episode_id="profile-smoke-only",
                scene_recipe_sha256=probe["scene_recipe_sha256"],
                joint_names=names,
                lower_rad=tuple(item["lower_rad"] for item in live),
                upper_rad=tuple(item["upper_rad"] for item in live),
                max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
                object_speeds=lambda: (), controller_mode=lambda: "UNKNOWN",
                external_motion_active=lambda: True, motion_enabled=False)
            idle_tick = adapter.tick()
        report = {"schema_version": "rrm-hand-live-binding-smoke/v1",
            "probe_sha256": hashlib.sha256(probe_bytes).hexdigest(),
            "qualification_sha256": hashlib.sha256(qualification_bytes).hexdigest(),
            "strict_gates_recomputed_pass": True,
            "asset_url": ASSET_URL, "articulation_root": roots[0],
            "asset_content_hash_verified": False,
            "joint_count": len(live), "profile_mismatches": mismatches,
            "idle_tick": idle_tick, "apply_action_calls": guarded.apply_calls,
            "adapter_motion_enabled": None if adapter is None else adapter.motion_enabled,
            "controller_gains_verified": False,
            "safe_state_qualified": False, "stop_qualified": False,
            "execution_dispatch": False, "scene_equivalent_to_qualification": False}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return 0 if not mismatches and idle_tick == "IDLE" and guarded.apply_calls == 0 else 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
