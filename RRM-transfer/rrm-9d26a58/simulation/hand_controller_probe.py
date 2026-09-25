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
KUKA_ARM_HOME_OFFSETS_RAD = [0.0, -0.05, 0.0, 0.10, 0.0, -0.05, 0.0]

# Safe-state thresholds (from hand-embodiment-decision.md):
# "arm/hand and object speeds below measured limits for a configured consecutive
#  window, controller in declared hold/disabled mode"
SAFE_JOINT_VEL_THRESHOLD_RAD_S = 0.10
SAFE_OBJECT_VEL_THRESHOLD_M_S = 0.01
SAFE_CONSECUTIVE_WINDOW = 5  # consecutive samples at or below threshold
CONTACT_FORCE_THRESHOLD_N = 0.10
CONTACT_BASELINE_SAMPLES = 10
CONTACT_CHALLENGE_STEPS = 60

# Runtime drive gains from the official Isaac Lab KUKA_ALLEGRO_CFG.  The raw
# USD reports stiffness near 5.73e8 with almost no damping, which produced
# startup oscillations above the USD joint-velocity limits in probe L.  Keep
# the asset's limits unchanged and replace only the implicit-controller gains.
KUKA_REFERENCE_STIFFNESS = 200.0
KUKA_REFERENCE_DAMPING = [42.2, 54.4, 45.6, 37.8, 25.3, 25.5, 23.5]
# The standalone core-api probe has no gravity-compensation term or Isaac Lab
# actuator armature. Scale Kp for holding authority and Kd by sqrt(Kp scale) to
# preserve the reference damping ratio.
KUKA_ARM_STIFFNESS = 4000.0
KUKA_ARM_GAIN_FACTOR = KUKA_ARM_STIFFNESS / KUKA_REFERENCE_STIFFNESS
KUKA_ARM_DAMPING = [value * KUKA_ARM_GAIN_FACTOR**0.5 for value in KUKA_REFERENCE_DAMPING]
ALLEGRO_REFERENCE_STIFFNESS = 3.0
ALLEGRO_FINGER_STIFFNESS = 30.0
ALLEGRO_REFERENCE_DAMPING = 0.1
# Probe M showed four passive fingers grazing the USD 6.283 rad/s ceiling
# (worst 6.339 rad/s) with the reference damping; probes N-R showed that low
# finger holding authority also allows arm motion to excite persistent drift.
# Increase Kp 10x and Kd 20x for a damped hold while leaving every physical
# position, effort and velocity limit unchanged.
ALLEGRO_FINGER_DAMPING = 2.0
GAIN_REFERENCE = {
    "source": "isaac-sim/IsaacLab",
    "file": "source/isaaclab_assets/isaaclab_assets/robots/kuka_allegro.py",
    "commit": "481a676a993b4054f46556ebd2c14a155b87602f",
}
ALLEGRO_REFERENCE_POSE_RAD = {
    "index_joint_0": 0.0, "index_joint_1": 0.3,
    "index_joint_2": 0.3, "index_joint_3": 0.3,
    "middle_joint_0": 0.0, "middle_joint_1": 0.3,
    "middle_joint_2": 0.3, "middle_joint_3": 0.3,
    "ring_joint_0": 0.0, "ring_joint_1": 0.3,
    "ring_joint_2": 0.3, "ring_joint_3": 0.3,
    "thumb_joint_0": 1.5, "thumb_joint_1": 0.60147215,
    "thumb_joint_2": 0.33795027, "thumb_joint_3": 0.60845138,
}


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
        from isaacsim.core.prims import RigidPrim, SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.sensors.physics import ContactSensor

        np.random.seed(0)
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)
        world.get_physics_context().enable_ccd(True)
        add_reference_to_stage(usd_path=ASSET_URL, prim_path="/World/KukaAllegro")
        app.update()
        light = UsdLux.DomeLight.Define(get_current_stage(), "/World/DomeLight")
        light.CreateIntensityAttr(1000.0)
        stage = get_current_stage()
        roots = [
            str(prim.GetPath()) for prim in get_current_stage().Traverse()
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and str(prim.GetPath()).startswith("/World/KukaAllegro")
        ]
        if len(roots) != 1:
            raise RuntimeError(f"expected one Kuka-Allegro articulation, got {roots}")
        hand = world.scene.add(SingleArticulation(prim_path=roots[0], name="kuka_allegro"))

        # Build the articulation graph from USD joint relationships. Its leaf rigid
        # bodies are the four Allegro distal links. Track only contacts between those
        # bodies and the red calibration block so self-contact cannot masquerade as a
        # successful fingertip observation.
        rigid_body_paths = {
            str(prim.GetPath()) for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and str(prim.GetPath()).startswith("/World/KukaAllegro")
        }
        joint_parent_paths = set()
        joint_child_paths = set()
        parent_by_child = {}
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            parents = [str(path) for path in joint.GetBody0Rel().GetTargets()]
            children = [str(path) for path in joint.GetBody1Rel().GetTargets()]
            joint_parent_paths.update(parents)
            joint_child_paths.update(children)
            if len(parents) == 1 and len(children) == 1:
                parent_by_child[children[0]] = parents[0]
        terminal_tip_paths = sorted(
            path for path in joint_child_paths
            if path in rigid_body_paths and path not in joint_parent_paths
        )
        distal_paths = [parent_by_child.get(path) for path in terminal_tip_paths]
        fingertip_paths = sorted({path for path in distal_paths if path is not None})
        if len(terminal_tip_paths) != 4 or len(fingertip_paths) != 4 or None in distal_paths:
            raise RuntimeError(
                f"expected four terminal tips and distal bodies, got "
                f"tips={terminal_tip_paths}, distal={fingertip_paths}"
            )
        fingertip_contacts = world.scene.add(RigidPrim(
            prim_paths_expr=fingertip_paths,
            name="kuka_allegro_fingertip_contacts",
            reset_xform_properties=False,
        ))
        fingertip_sensors = []
        for index, fingertip_path in enumerate(fingertip_paths):
            sensor = world.scene.add(ContactSensor(
                prim_path=f"{fingertip_path}/rrm_contact_sensor",
                name=f"rrm_fingertip_contact_{index}",
                min_threshold=0.0,
                max_threshold=1_000_000.0,
                radius=-1.0,
                translation=np.zeros(3),
            ))
            sensor.add_raw_contact_data_to_frame()
            fingertip_sensors.append(sensor)
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
        dof_properties = hand.dof_properties

        # The USD's authored drive gains are not the gains used by the current
        # official Isaac Lab configuration. Apply the latter through the public
        # ArticulationController API while leaving position, effort and velocity
        # limits untouched.
        controller = hand.get_articulation_controller()
        source_kps, source_kds = controller.get_gains()
        configured_kps = np.full(num_joints, ALLEGRO_FINGER_STIFFNESS, dtype=float)
        configured_kds = np.full(num_joints, ALLEGRO_FINGER_DAMPING, dtype=float)
        configured_kps[:7] = KUKA_ARM_STIFFNESS
        configured_kds[:7] = np.asarray(KUKA_ARM_DAMPING, dtype=float)
        controller.set_gains(kps=configured_kps, kds=configured_kds)
        applied_kps, applied_kds = controller.get_gains()
        gains_applied = bool(
            np.allclose(applied_kps, configured_kps)
            and np.allclose(applied_kds, configured_kds)
        )
        if not gains_applied:
            raise RuntimeError("official Kuka-Allegro runtime gains were not applied")

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

        # Use the supported Isaac Lab Allegro reset pose instead of the USD's
        # mostly-zero hand pose. The latter leaves the thumb drifting after an
        # otherwise successful stop command. Every override remains bounded by
        # the unchanged USD position limits.
        controller_default_overrides = []
        for name, reference_position in ALLEGRO_REFERENCE_POSE_RAD.items():
            index = joint_names.index(name)
            lower, upper = dof_limits_np[index]
            bounded_position = float(np.clip(reference_position, lower, upper))
            controller_default_overrides.append({
                "joint": name,
                "usd_or_clamped_position_rad": float(clamped_default[index]),
                "controller_default_position_rad": bounded_position,
            })
            clamped_default[index] = bounded_position

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
                "max_velocity_rad_s": float(dof_properties[i]["maxVelocity"]),
                "max_effort": float(dof_properties[i]["maxEffort"]),
                "stiffness": float(dof_properties[i]["stiffness"]),
                "damping": float(dof_properties[i]["damping"]),
                "drive_mode": int(dof_properties[i]["driveMode"]),
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
        max_observed_joint_velocities = np.zeros(num_joints, dtype=float)
        for step_index in range(240):
            world.step(render=False)
            joints = np.asarray(hand.get_joint_positions(), dtype=float)
            joint_vels = np.asarray(hand.get_joint_velocities(), dtype=float)
            max_observed_joint_velocities = np.maximum(
                max_observed_joint_velocities, np.abs(joint_vels),
            )
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
            "declared_hold_mode": "POSITION_HOLD",
            "runtime_gain_override": {
                "reference": GAIN_REFERENCE,
                "arm_stiffness_reference": KUKA_REFERENCE_STIFFNESS,
                "arm_gain_factor": KUKA_ARM_GAIN_FACTOR,
                "finger_damping_reference": ALLEGRO_REFERENCE_DAMPING,
                "finger_stiffness_reference": ALLEGRO_REFERENCE_STIFFNESS,
                "finger_stiffness_safety_factor": (
                    ALLEGRO_FINGER_STIFFNESS / ALLEGRO_REFERENCE_STIFFNESS
                ),
                "finger_damping_safety_factor": (
                    ALLEGRO_FINGER_DAMPING / ALLEGRO_REFERENCE_DAMPING
                ),
                "limits_changed": False,
                "source_stiffness": np.asarray(source_kps, dtype=float).tolist(),
                "source_damping": np.asarray(source_kds, dtype=float).tolist(),
                "configured_stiffness": configured_kps.tolist(),
                "configured_damping": configured_kds.tolist(),
                "applied_as_configured": gains_applied,
            },
            "max_observed_joint_velocities_rad_s": max_observed_joint_velocities.tolist(),
            "velocity_limits_respected": all(
                observed <= float(dof_properties[index]["maxVelocity"])
                for index, observed in enumerate(max_observed_joint_velocities)
            ),
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
            "controller_mode": "POSITION_HOLD",
            "active_motion_command": False,
            "samples": safe_state_samples,
        }

        # Read a no-contact baseline now. A known-contact challenge follows the stop
        # trial, so contact injection cannot affect stop latency evidence.
        baseline_contact_peak_n = 0.0
        contact_sensors_readable = False
        for _ in range(CONTACT_BASELINE_SAMPLES):
            world.step(render=False)
            for sensor in fingertip_sensors:
                frame = sensor.get_current_frame()
                if isinstance(frame, dict) and "force" in frame:
                    contact_sensors_readable = True
                    baseline_contact_peak_n = max(
                        baseline_contact_peak_n, float(frame["force"]),
                    )

        # ── Gate 4: Independent stop test ──
        # Move the arm quickly and interrupt it
        arm_joint_indices = np.arange(7, dtype=np.int32)
        stop_action = ArticulationAction(
            joint_positions=clamped_default[:7].copy(),
            joint_indices=arm_joint_indices,
        )
        hand.apply_action(stop_action)
        # 10 steps to gain some speed
        for _ in range(10):
            world.step(render=False)
            
        mid_vel = float(np.max(np.abs(hand.get_joint_velocities())))
        
        # INTERVENE: send current position as target to stop
        current_pos = np.asarray(hand.get_joint_positions(), dtype=float).copy()
        halt_action = ArticulationAction(
            joint_positions=current_pos[:7].copy(),
            joint_velocities=np.zeros(7, dtype=float),
            joint_indices=arm_joint_indices,
        )
        hand.apply_action(halt_action)
        
        # Verify it stops quickly
        consecutive_safe = 0
        stop_safe_achieved = False
        stop_steps = -1
        stop_samples = []
        for step_index in range(240): # 2.0s max to stop
            world.step(render=False)
            stop_joint_velocities = np.asarray(hand.get_joint_velocities(), dtype=float)
            max_joint_index = int(np.argmax(np.abs(stop_joint_velocities)))
            max_joint_vel = float(np.abs(stop_joint_velocities[max_joint_index]))
            if step_index % 24 == 0:
                stop_samples.append({
                    "step": step_index,
                    "max_joint": joint_names[max_joint_index],
                    "max_joint_velocity_rad_s": max_joint_vel,
                })
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
            "commanded_joint_names": joint_names[:7],
            "safe_state_achieved": stop_safe_achieved,
            "steps_to_safe": stop_steps,
            "stop_latency_s": None if stop_steps < 0 else (stop_steps + 1) / 120.0,
            "samples": stop_samples,
            "controller_mode": "POSITION_HOLD",
            "active_motion_command": False,
        }

        # ── Gate 4: known fingertip-contact observation ──
        # Place the red calibration block at the first distal-link origin for one
        # bounded collision-resolution trial. The filtered force matrix proves which
        # external entity caused the reading; it does not infer a grasp.
        fingertip_positions, _ = fingertip_contacts.get_world_poses()
        challenge_position = np.asarray(fingertip_positions[0], dtype=float).copy()
        blocks["red_block"].set_world_pose(
            position=challenge_position,
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        blocks["red_block"].set_linear_velocity(np.zeros(3))
        blocks["red_block"].set_angular_velocity(np.zeros(3))
        challenge_contact_peak_n = 0.0
        challenge_body_path = None
        challenge_step = None
        challenge_contact_pairs = []
        for step_index in range(CONTACT_CHALLENGE_STEPS):
            world.step(render=False)
            for body_index, sensor in enumerate(fingertip_sensors):
                frame = sensor.get_current_frame()
                if not isinstance(frame, dict) or "force" not in frame:
                    continue
                peak = float(frame["force"])
                pairs = []
                for contact in frame.get("contacts", []):
                    pairs.append({
                        "body0": str(contact["body0"]),
                        "body1": str(contact["body1"]),
                    })
                if peak > challenge_contact_peak_n:
                    challenge_contact_peak_n = peak
                    challenge_body_path = fingertip_paths[body_index]
                    challenge_step = step_index
                    challenge_contact_pairs = pairs
        red_block_named = any(
            "red_block" in pair[side]
            for pair in challenge_contact_pairs for side in ("body0", "body1")
        )
        known_contact_detected = (
            contact_sensors_readable
            and challenge_contact_peak_n >= CONTACT_FORCE_THRESHOLD_N
            and challenge_contact_peak_n > baseline_contact_peak_n + CONTACT_FORCE_THRESHOLD_N
            and red_block_named
        )
        contact_record = {
            "sensor_type": "Isaac Sim ContactSensor with raw contact pairs",
            "fingertip_contact_sensor_active": contact_sensors_readable,
            "fingertip_body_paths": fingertip_paths,
            "terminal_tip_paths": terminal_tip_paths,
            "filter_entity_id": "red_block",
            "filter_prim_path": "/World/red_block",
            "baseline_samples": CONTACT_BASELINE_SAMPLES,
            "baseline_peak_force_n": baseline_contact_peak_n,
            "challenge_steps": CONTACT_CHALLENGE_STEPS,
            "challenge_position_m": challenge_position.tolist(),
            "challenge_peak_force_n": challenge_contact_peak_n,
            "challenge_peak_body_path": challenge_body_path,
            "challenge_peak_step": challenge_step,
            "challenge_contact_pairs": challenge_contact_pairs,
            "red_block_named_in_peak_contacts": red_block_named,
            "detection_threshold_n": CONTACT_FORCE_THRESHOLD_N,
            "known_contact_detected": known_contact_detected,
            "grasp_claimed": False,
            "calibration_injection_only": True,
            "contact_stability_qualified": False,
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
            "rigid_body_paths": sorted(rigid_body_paths),
            "joint_limits": joint_limits_record,
            "out_of_range_fixes": out_of_range_fixes,
            "controller_default_overrides": controller_default_overrides,
            "controller_default_reference": GAIN_REFERENCE,
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
