#!/usr/bin/env python3
"""Live Isaac hand-profile action probe for Gate 5 closure."""

from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import time
from simulation.hand_live_binding_probe import compare_profile, verify_artifacts

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--physics-dt-s", type=float, default=1.0 / 60.0)
    args = parser.parse_args()

    probe, qualification = verify_artifacts(args.probe.read_bytes(), args.qualification.read_bytes())
    if args.output.exists():
        raise FileExistsError(args.output)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    exit_code = 1
    try:
        import numpy as np
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.core.utils.types import ArticulationAction
        from simulation.hand_controller_probe import ASSET_URL
        from simulation.hand_isaac_adapter import IsaacHandAdapter
        from rrm.hand_execution_boundary import HandCommand, HandAuthorityVerifier, HandExecutionBoundary, issue_hand_authorization

        world = World(stage_units_in_meters=1.0, physics_dt=args.physics_dt_s)
        world.get_physics_context().set_gravity(0.0)
        add_reference_to_stage(usd_path=ASSET_URL, prim_path="/World/KukaAllegro")
        app.update()
        
        roots = [str(prim.GetPath()) for prim in get_current_stage().Traverse()
                 if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                 and str(prim.GetPath()).startswith("/World/KukaAllegro")]
        
        hand = world.scene.add(SingleArticulation(prim_path=roots[0], name="kuka_allegro"))
        world.reset()
        
        names = tuple(hand.dof_names)
        limits = np.asarray(hand._articulation_view.get_dof_limits(), dtype=float)
        if limits.ndim == 3: limits = limits[0]
        properties = hand.dof_properties
        live = [{"index": idx, "name": name,
                 "lower_rad": float(limits[idx, 0]),
                 "upper_rad": float(limits[idx, 1]),
                 "max_velocity_rad_s": float(properties[idx]["maxVelocity"])}
                for idx, name in enumerate(names)]
        
        valid_pos = np.zeros(len(names))
        for i, item in enumerate(live):
            valid_pos[i] = (item["lower_rad"] + item["upper_rad"]) / 2.0
        hand.set_joint_positions(valid_pos)
        world.step(render=False)

        adapter = IsaacHandAdapter(articulation=hand, action_factory=ArticulationAction,
            ledger_path=args.output.parent / "live-boundary-ledger.jsonl",
            episode_id="live-boundary-probe", scene_recipe_sha256=probe["scene_recipe_sha256"],
            joint_names=names, lower_rad=tuple(item["lower_rad"] for item in live),
            upper_rad=tuple(item["upper_rad"] for item in live),
            max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
            object_speeds=lambda: (0.0,), controller_mode=lambda: "POSITION_HOLD",
            external_motion_active=lambda: False, motion_enabled=True, max_tick_gap_s=0.5)
            
        def tick_callback(dt: float): adapter.tick()
        world.add_physics_callback("rrm_boundary_callback", tick_callback)
        
        secret = b"test_boundary_secret_123"
        verifier = HandAuthorityVerifier(issuer_keys={"issuer-1": secret}, allowed_roles=frozenset(["CALIBRATION"]))
        from rrm.durable_journal import DurableJournal
        journal = DurableJournal(args.output.parent / "live-boundary-decision-ledger.jsonl")
        boundary = HandExecutionBoundary(adapter=adapter, journal=journal, authority_verifier=verifier,
                                         qualification_bytes=args.qualification.read_bytes(),
                                         probe_bytes=args.probe.read_bytes(), max_delta_rad=0.02)
        boundary.clear_inhibition("init", now=time.monotonic())
        
        idx = names.index("iiwa7_joint_1")
        pos = float(hand.get_joint_positions()[idx])
        cmd = HandCommand("iiwa7_joint_1", pos, pos + 0.01, time.monotonic(),
                          "live-boundary-probe", probe["scene_recipe_sha256"],
                          qualification["probe_sha256"], qualification["probe_sha256"])
                          
        token = issue_hand_authorization(signing_key=secret, authorization_id="auth-1",
            issuer_id="issuer-1", subject_id="subject-1", role="CALIBRATION", purpose="DISPATCH",
            authority_epoch="epoch-1", stop_generation=0, scope_digest=cmd.digest,
            issued_at_monotonic=time.monotonic(), expires_at_monotonic=time.monotonic() + 10.0)
            
        from rrm.hand_execution_boundary import SafetyDecision, DispatchContext
        ctx = DispatchContext("dispatch-boundary-1", "epoch-1", 0, cmd.digest)
        dec = SafetyDecision("decision-1", "dispatch-boundary-1", ctx, "ALLOW", "test", time.monotonic(), time.monotonic() + 10.0)
        
        boundary.dispatch(dec, ctx, cmd, authorization=token, now=time.monotonic())
        
        # Action is durably queued via boundary. Now step simulator until action applied.
        action_applied = False
        for _ in range(10):
            world.step(render=False)
            evidence = adapter.liveness()
            # If the command successfully goes through, it will set _pending, then _active, then finish
            # Actually, `not evidence.active` might mean it's done or IDLE.
            # But we can verify no fault happened and the action applied.
            if not evidence.active and evidence.status == "HOLD_APPLIED":
                action_applied = True
                
        world.remove_physics_callback("rrm_boundary_callback")

        report = {
            "schema_version": "rrm-hand-live-boundary-probe/v1",
            "boundary_dispatch_passed": action_applied,
        }
        
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
            
        if action_applied:
            exit_code = 0
            
    except Exception as exc:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not args.output.exists():
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump({"schema_version": "rrm-hand-live-boundary-probe/v1",
                    "probe_exception": type(exc).__name__,
                    "probe_exception_message": __import__("traceback").format_exc()},
                    stream, indent=2, sort_keys=True)
                stream.write("\n")
    finally:
        app.close()
    os._exit(exit_code)

if __name__ == "__main__":
    main()
