#!/usr/bin/env python3
"""Controller-limits, contact-observation and safe-state qualification probe.

Run with Isaac Sim's python.sh in a separate headless process.  This extends
the tabletop probe: it loads the same scene, reads joint limits, sends one
bounded position command to a safe home pose, observes settling and contact,
and measures safe-state conditions.  It never imports ROS.

Gate 1 completion: joint limits, controller behavior, contact setup.
Gate 4 partial:    safe-state measurement (speeds below thresholds for a window).
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

ASSET_URL = (
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1/Isaac/"
    "IsaacLab/Robots/KukaAllegro/kuka.usd"
)
ASSET_SHA256 = "935957108f80625b58afb3ace9fff326edbb820a0b3629793c5f4931adb58fb2"

# A conservative pre-grasp home pose for the Kuka arm (7 joints).
# Finger joints (16) stay at their default positions.
# These are small perturbations from default to prove the controller works.
KUKA_ARM_HOME_OFFSETS_RAD = [0.0, -0.3, 0.0, 0.8, 0.0, -0.4, 0.0]

# Safe-state thresholds (from hand-embodiment-decision.md):
# "arm/hand and object speeds below measured limits for a configured consecutive
#  window, controller in declared hold/disabled mode"
SAFE_JOINT_VEL_THRESHOLD_RAD_S = 0.10
SAFE_OBJECT_VEL_THRESHOLD_M_S = 0.01
SAFE_CONSECUTIVE_WINDOW = 5  # consecutive samples at or below threshold


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        import numpy as np
        from pxr import UsdLux, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage

        np.random.seed(0)
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)
        world.get_physics_context().enable_ccd(True)
        add_reference_to_stage(usd_path=ASSET_URL, prim_path="/World/KukaAllegro")
        app.update()
        light = UsdLux.DomeLight.Define(get_current_stage(), "/World/DomeLight")
        light.CreateIntensityAttr(1000.0)
        roots = [
            str(prim.GetPath()) for prim in get_current_stage().Traverse()
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and str(prim.GetPath()).startswith("/World/KukaAllegro")
        ]
        if len(roots) != 1:
            raise RuntimeError(f"expected one Kuka-Allegro articulation, got {roots}")
        hand = world.scene.add(SingleArticulation(prim_path=roots[0], name="kuka_allegro"))
        world.scene.add(FixedCuboid(
            prim_path="/World/Table", name="table", position=np.array([0.68, 0.0, 0.45]),
            scale=np.array([0.90, 0.80, 0.10]), color=np.array([0.55, 0.55, 0.55]),
        ))
        world.scene.add(FixedCuboid(
            prim_path="/World/Tray", name="tray_1", position=np.array([0.84, -0.24, 0.505]),
            scale=np.array([0.18, 0.18, 0.01]), color=np.array([0.9, 0.8, 0.1]),
        ))
        block_specs = (
            ("red_block", [0.68, 0.16, 0.535], [0.9, 0.1, 0.1]),
            ("blue_block", [0.68, -0.02, 0.535], [0.1, 0.2, 0.9]),
        )
        blocks = {}
        for entity_id, position, color in block_specs:
            block = world.scene.add(DynamicCuboid(
                prim_path=f"/World/{entity_id}", name=entity_id,
                position=np.array(position), scale=np.array([0.06] * 3),
                color=np.array(color), mass=0.05,
            ))
            block.set_default_state(
                position=np.array(position), orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
            blocks[entity_id] = block

        world.reset()
        joint_default = np.asarray(hand.get_joint_positions(), dtype=float)
        num_joints = joint_default.size
        if num_joints != 23:
            raise RuntimeError(f"unexpected Kuka-Allegro joint count: {num_joints}")

        # ── Read limits early so we can fix out-of-range defaults ──
        joint_names = list(hand.dof_names)
        dof_limits = hand._articulation_view.get_dof_limits()
        dof_limits_np = np.asarray(dof_limits, dtype=float)
        if dof_limits_np.ndim == 3:
            dof_limits_np = dof_limits_np[0]  # single env

        # Clamp default positions to within physical joint limits.
        # The USD default for thumb_joint_0 is 0.0 but its limits are
        # [0.279, 1.571]; leaving it out of range causes persistent oscillation.
        clamped_default = joint_default.copy()
        out_of_range_fixes = []
        for i in range(num_joints):
            lower, upper = float(dof_limits_np[i, 0]), float(dof_limits_np[i, 1])
            if clamped_default[i] < lower or clamped_default[i] > upper:
                midpoint = (lower + upper) / 2.0
                out_of_range_fixes.append({
                    "joint": joint_names[i], "index": i,
                    "original_rad": float(clamped_default[i]),
                    "clamped_to_rad": midpoint,
                    "lower_rad": lower, "upper_rad": upper,
                })
                clamped_default[i] = midpoint

        hand.set_joints_default_state(
            positions=clamped_default.copy(), velocities=np.zeros_like(clamped_default),
        )

        # ── Gate 1a: Record joint limits ──
        joint_limits_record = []
        for i, name in enumerate(joint_names):
            lower = float(dof_limits_np[i, 0])
            upper = float(dof_limits_np[i, 1])
            default_pos = float(joint_default[i])
            clamped_pos = float(clamped_default[i])
            joint_limits_record.append({
                "index": i,
                "name": name,
                "lower_rad": lower,
                "upper_rad": upper,
                "range_rad": upper - lower,
                "usd_default_position_rad": default_pos,
                "clamped_default_position_rad": clamped_pos,
                "usd_default_within_limits": lower <= default_pos <= upper,
            })

        # ── Gate 1b: Test ArticulationController with a bounded position target ──
        # Warm reset
        world.reset()
        for _ in range(30):
            world.step(render=False)

        # Record pre-command state
        pre_positions = np.asarray(hand.get_joint_positions(), dtype=float).copy()
        pre_velocities = np.asarray(hand.get_joint_velocities(), dtype=float).copy()

        # Compute safe home target: apply small arm offsets, leave fingers at clamped default
        target_positions = clamped_default.copy()
        for i, offset in enumerate(KUKA_ARM_HOME_OFFSETS_RAD):
            proposed = clamped_default[i] + offset
            lower = dof_limits_np[i, 0]
            upper = dof_limits_np[i, 1]
            target_positions[i] = float(np.clip(proposed, lower, upper))

        # Apply position command via ArticulationAction
        from isaacsim.core.utils.types import ArticulationAction
        action = ArticulationAction(joint_positions=target_positions)
        hand.apply_action(action)
        controller_command_sent = True

        # Step physics and record trajectory: 240 steps = 2s at 120Hz
        trajectory_samples = []
        for step_index in range(240):
            world.step(render=False)
            joints = np.asarray(hand.get_joint_positions(), dtype=float)
            joint_vels = np.asarray(hand.get_joint_velocities(), dtype=float)
            # Read object states
            object_speeds = {}
            for entity_id, block in blocks.items():
                lin_vel = np.asarray(block.get_linear_velocity(), dtype=float)
                object_speeds[entity_id] = float(np.linalg.norm(lin_vel))
            if step_index % 24 == 0 or step_index == 239:  # sample every 0.2s + final
                trajectory_samples.append({
                    "step": step_index,
                    "time_s": round((step_index + 1) / 120.0, 4),
                    "arm_position_error_rad": float(np.max(np.abs(joints[:7] - target_positions[:7]))),
                    "max_joint_velocity_rad_s": float(np.max(np.abs(joint_vels))),
                    "max_arm_velocity_rad_s": float(np.max(np.abs(joint_vels[:7]))),
                    "max_finger_velocity_rad_s": float(np.max(np.abs(joint_vels[7:]))),
                    "object_speeds_m_s": object_speeds,
                })

        # Post-command state
        post_positions = np.asarray(hand.get_joint_positions(), dtype=float)
        post_velocities = np.asarray(hand.get_joint_velocities(), dtype=float)
        arm_position_error = float(np.max(np.abs(post_positions[:7] - target_positions[:7])))
        arm_converged = arm_position_error < 0.01  # within 0.01 rad of target

        controller_record = {
            "command_type": "position",
            "target_joint_count": int(num_joints),
            "arm_offsets_applied_rad": KUKA_ARM_HOME_OFFSETS_RAD,
            "arm_targets_clipped_to_limits": True,
            "finger_targets_unchanged": True,
            "steps_simulated": 240,
            "duration_s": 2.0,
            "arm_converged": arm_converged,
            "final_arm_position_error_rad": arm_position_error,
            "final_max_joint_velocity_rad_s": float(np.max(np.abs(post_velocities))),
            "trajectory_samples": trajectory_samples,
        }

        # ── Gate 4 partial: Safe-state measurement ──
        # Continue stepping and check if speeds stay below thresholds
        consecutive_safe = 0
        safe_state_achieved = False
        safe_state_samples = []
        for step_index in range(240):  # 2 seconds to settle
            world.step(render=False)
            joint_vels = np.asarray(hand.get_joint_velocities(), dtype=float)
            max_joint_vel = float(np.max(np.abs(joint_vels)))
            max_object_vel = 0.0
            for entity_id, block in blocks.items():
                lin_vel = np.asarray(block.get_linear_velocity(), dtype=float)
                speed = float(np.linalg.norm(lin_vel))
                max_object_vel = max(max_object_vel, speed)
            joint_safe = max_joint_vel <= SAFE_JOINT_VEL_THRESHOLD_RAD_S
            object_safe = max_object_vel <= SAFE_OBJECT_VEL_THRESHOLD_M_S
            if joint_safe and object_safe:
                consecutive_safe += 1
            else:
                consecutive_safe = 0
            if consecutive_safe >= SAFE_CONSECUTIVE_WINDOW and not safe_state_achieved:
                safe_state_achieved = True
                safe_state_samples.append({
                    "step": step_index,
                    "time_s": round(2.0 + (step_index + 1) / 120.0, 4),
                    "max_joint_velocity_rad_s": max_joint_vel,
                    "max_object_velocity_m_s": max_object_vel,
                    "consecutive_safe_count": consecutive_safe,
                    "verdict": "SAFE_CONFIRMED",
                })
        if not safe_state_achieved:
            # Record the final state even if not safe
            joint_vels = np.asarray(hand.get_joint_velocities(), dtype=float)
            max_joint_vel = float(np.max(np.abs(joint_vels)))
            max_object_vel = 0.0
            for entity_id, block in blocks.items():
                lin_vel = np.asarray(block.get_linear_velocity(), dtype=float)
                max_object_vel = max(max_object_vel, float(np.linalg.norm(lin_vel)))
            safe_state_samples.append({
                "step": 119,
                "time_s": 3.0,
                "max_joint_velocity_rad_s": max_joint_vel,
                "max_object_velocity_m_s": max_object_vel,
                "consecutive_safe_count": consecutive_safe,
                "verdict": "SAFE_UNCONFIRMED",
            })

        safe_state_record = {
            "joint_velocity_threshold_rad_s": SAFE_JOINT_VEL_THRESHOLD_RAD_S,
            "object_velocity_threshold_m_s": SAFE_OBJECT_VEL_THRESHOLD_M_S,
            "consecutive_window_required": SAFE_CONSECUTIVE_WINDOW,
            "safe_state_achieved": safe_state_achieved,
            "samples": safe_state_samples,
        }

        # ── Gate 4: Contact observation check ──
        # Measure fingertip contact forces via ArticulationView
        contact_observed = False
        max_contact_force = 0.0
        hand_contact_forces = None
        try:
            hand_contact_forces = hand._articulation_view.get_net_contact_forces()
            if hand_contact_forces is not None:
                forces_np = np.asarray(hand_contact_forces, dtype=float)
                force_mags = np.linalg.norm(forces_np, axis=-1)
                max_contact_force = float(np.max(force_mags))
                contact_observed = max_contact_force >= 0.0
        except Exception as e:
            print(f"Contact reading failed: {e}")
            
        contact_record = {
            "fingertip_contact_sensor_active": hand_contact_forces is not None,
            "max_contact_force_n": max_contact_force
        }

        # ── Gate 4: Independent stop test ──
        # Move the arm quickly and interrupt it
        stop_action = ArticulationAction(joint_positions=clamped_default.copy())
        hand.apply_action(stop_action)
        # 10 steps to gain some speed
        for _ in range(10):
            world.step(render=False)
            
        mid_vel = float(np.max(np.abs(hand.get_joint_velocities())))
        
        # INTERVENE: send current position as target to stop
        current_pos = hand.get_joint_positions()
        halt_action = ArticulationAction(
            joint_positions=current_pos, 
            joint_velocities=np.zeros_like(current_pos)
        )
        hand.apply_action(halt_action)
        
        # Verify it stops quickly
        consecutive_safe = 0
        stop_safe_achieved = False
        stop_steps = -1
        for step_index in range(240): # 2.0s max to stop
            world.step(render=False)
            max_joint_vel = float(np.max(np.abs(hand.get_joint_velocities())))
            if max_joint_vel <= SAFE_JOINT_VEL_THRESHOLD_RAD_S:
                consecutive_safe += 1
            else:
                consecutive_safe = 0
            if consecutive_safe >= SAFE_CONSECUTIVE_WINDOW:
                stop_safe_achieved = True
                stop_steps = step_index
                break
                
        stop_record = {
            "motion_started": mid_vel > SAFE_JOINT_VEL_THRESHOLD_RAD_S,
            "mid_motion_velocity_rad_s": mid_vel,
            "stop_command_sent": True,
            "safe_state_achieved": stop_safe_achieved,
            "steps_to_safe": stop_steps
        }

        # ── Gate 2 revisit: full reset determinism with controller state ──
        # After commanding, do 3 resets and verify state hashes match
        reset_samples = []
        for index in range(3):
            world.reset()
            for _ in range(30):
                world.step(render=False)
            for _ in range(8):
                world.step(render=False)
            joints = np.asarray(hand.get_joint_positions(), dtype=float)
            joint_vels = np.asarray(hand.get_joint_velocities(), dtype=float)
            state_parts = [joints, joint_vels]
            for entity_id, block in blocks.items():
                position, orientation = block.get_world_pose()
                lin_vel = block.get_linear_velocity()
                ang_vel = block.get_angular_velocity()
                state_parts.extend([
                    np.asarray(item, dtype=float)
                    for item in (position, orientation, lin_vel, ang_vel)
                ])
            reset_samples.append({
                "reset_index": index,
                "episode_id": f"controller-probe-20260924/post-command-reset-{index}",
                "observed_monotonic_s": time.monotonic(),
                "state_sha256_rounded_1e-4": hashlib.sha256(
                    np.round(np.concatenate(state_parts), 4).tobytes()
                ).hexdigest(),
                "max_joint_velocity_rad_s": float(np.max(np.abs(joint_vels))),
            })

        report = {
            "schema_version": "rrm-hand-controller-probe/v1",
            "scene_recipe_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "scene_seed": 0, "physics_dt_s": 1 / 120,
            "asset_url": ASSET_URL, "asset_sha256": ASSET_SHA256,
            "articulation_root": roots[0],
            "joint_names": joint_names,
            "joint_count": num_joints,
            "joint_limits": joint_limits_record,
            "out_of_range_fixes": out_of_range_fixes,
            "scene_entities": {
                "red_block": "/World/red_block", "blue_block": "/World/blue_block",
                "tray_1": "/World/Tray", "table": "/World/Table",
            },
            "controller": controller_record,
            "safe_state": safe_state_record,
            "contact_observation": contact_record,
            "independent_stop": stop_record,
            "post_command_reset_samples": reset_samples,
            "post_command_reset_hashes_match": len(
                {item["state_sha256_rounded_1e-4"] for item in reset_samples}
            ) == 1,
            "controller_command_sent": controller_command_sent,
            "ros_connected": False,
            "execution_dispatch": False,
        }
        (args.output_dir / "probe.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8",
        )
        print(json.dumps({
            "schema_version": report["schema_version"],
            "joint_count": num_joints,
            "arm_converged": arm_converged,
            "safe_state_achieved": safe_state_achieved,
            "post_command_reset_hashes_match": report["post_command_reset_hashes_match"],
            "output_dir": str(args.output_dir),
            "controller_command_sent": controller_command_sent,
            "execution_dispatch": False,
        }, sort_keys=True), flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
