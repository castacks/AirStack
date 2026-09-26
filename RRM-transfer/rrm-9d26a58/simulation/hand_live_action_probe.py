#!/usr/bin/env python3
"""Live Isaac hand-profile action probe for Gate 4 closure."""

from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import threading
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
        from rrm.hand_execution_boundary import HandCommand

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
        
        # VERY IMPORTANT: initialize hand positions to mid-range so they are valid
        valid_pos = np.zeros(len(names))
        for i, item in enumerate(live):
            valid_pos[i] = (item["lower_rad"] + item["upper_rad"]) / 2.0
        hand.set_joint_positions(valid_pos)
        world.step(render=False)

        # Scenario 1: Async Stop Fence
        adapter_stop = IsaacHandAdapter(articulation=hand, action_factory=ArticulationAction,
            ledger_path=args.output.parent / "live-stop-fence-ledger.jsonl",
            episode_id="live-action-probe-stop", scene_recipe_sha256=probe["scene_recipe_sha256"],
            joint_names=names, lower_rad=tuple(item["lower_rad"] for item in live),
            upper_rad=tuple(item["upper_rad"] for item in live),
            max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
            object_speeds=lambda: (0.0,), controller_mode=lambda: "POSITION_HOLD",
            external_motion_active=lambda: False, motion_enabled=True, max_tick_gap_s=0.5)
            
        def tick_stop_callback(dt: float): adapter_stop.tick()
        world.add_physics_callback("rrm_stop_callback", tick_stop_callback)
        
        idx = names.index("iiwa7_joint_1")
        pos = float(hand.get_joint_positions()[idx])
        cmd = HandCommand("iiwa7_joint_1", pos, pos + 0.01, time.monotonic(),
                          "live-action-probe-stop", probe["scene_recipe_sha256"],
                          qualification["probe_sha256"], qualification["probe_sha256"])
        adapter_stop.submit("dispatch-stop-1", 1, cmd)
        
        for _ in range(5): world.step(render=False)
        adapter_stop.request_stop(2)
        
        stop_held = False
        for _ in range(5):
            world.step(render=False)
            evidence = adapter_stop.liveness()
            if evidence.stop_requested_at is not None and not evidence.stop_pending:
                stop_held = True
                
        world.remove_physics_callback("rrm_stop_callback")

        # Scenario 2: Watchdog thread
        # Reset to midpoints again for isolation
        hand.set_joint_positions(valid_pos)
        world.step(render=False)
        
        adapter_watchdog = IsaacHandAdapter(articulation=hand, action_factory=ArticulationAction,
            ledger_path=args.output.parent / "live-watchdog-ledger.jsonl",
            episode_id="live-action-probe-watchdog", scene_recipe_sha256=probe["scene_recipe_sha256"],
            joint_names=names, lower_rad=tuple(item["lower_rad"] for item in live),
            upper_rad=tuple(item["upper_rad"] for item in live),
            max_velocity_rad_s=tuple(item["max_velocity_rad_s"] for item in live),
            object_speeds=lambda: (0.0,), controller_mode=lambda: "POSITION_HOLD",
            external_motion_active=lambda: False, motion_enabled=True, max_tick_gap_s=0.2)
            
        watchdog_fenced = False
        watchdog_hang = False
        
        def tick_watchdog_callback(dt: float):
            nonlocal watchdog_hang
            if watchdog_hang:
                time.sleep(0.3)
            adapter_watchdog.tick()

        world.add_physics_callback("rrm_watchdog_callback", tick_watchdog_callback)
        
        idx2 = names.index("iiwa7_joint_2")
        pos2 = float(hand.get_joint_positions()[idx2])
        cmd2 = HandCommand("iiwa7_joint_2", pos2, pos2 + 0.01, time.monotonic(),
                           "live-action-probe-watchdog", probe["scene_recipe_sha256"],
                           qualification["probe_sha256"], qualification["probe_sha256"])
        adapter_watchdog.submit("dispatch-wd-1", 1, cmd2)
        
        world.step(render=False)
        adapter_watchdog.start_watchdog(interval_s=0.05)
        
        watchdog_hang = True
        world.step(render=False)  # This will sleep for 0.3s inside the callback!
        watchdog_hang = False
        
        if not adapter_watchdog.motion_enabled and adapter_watchdog.liveness().reason == "TICK_STALE":
            watchdog_fenced = True
            
        world.remove_physics_callback("rrm_watchdog_callback")
        adapter_watchdog.stop_watchdog()

        report = {
            "schema_version": "rrm-hand-live-action-probe/v1",
            "stop_fence_passed": stop_held,
            "watchdog_fence_passed": watchdog_fenced,
        }
        
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
            
        if stop_held and watchdog_fenced:
            exit_code = 0
            
    except Exception as exc:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not args.output.exists():
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump({"schema_version": "rrm-hand-live-action-probe/v1",
                    "probe_exception": type(exc).__name__,
                    "probe_exception_message": __import__("traceback").format_exc()},
                    stream, indent=2, sort_keys=True)
                stream.write("\n")
    finally:
        app.close()
    os._exit(exit_code)

if __name__ == "__main__":
    main()
