#!/usr/bin/env python3
"""Isolated, no-action Kuka-Allegro tabletop/reset/camera qualification probe.

Run with Isaac Sim's python.sh in a separate headless process. This creates a
stage and reads simulator state; it never imports ROS or sends a joint target.
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        import numpy as np
        from PIL import Image
        from pxr import UsdLux, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.sensors.camera import Camera

        np.random.seed(0)
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)
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
        camera = Camera(
            prim_path="/World/OverheadCamera", name="overhead_camera",
            position=np.array([0.40, 0.0, 3.5]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            resolution=(640, 480),
        )
        world.reset()
        camera.initialize()
        camera.set_world_pose(
            position=np.array([0.40, 0.0, 3.5]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]), camera_axes="usd",
        )
        joint_default = np.asarray(hand.get_joint_positions(), dtype=float)
        if joint_default.size != 23:
            raise RuntimeError(f"unexpected Kuka-Allegro joint count: {joint_default.size}")
        hand.set_joints_default_state(
            positions=joint_default.copy(), velocities=np.zeros_like(joint_default),
        )

        # Initial render/articulation initialization is not a reset-repeatability
        # sample. Warm it once before the three identically stepped measurements.
        world.reset()
        for _ in range(30):
            world.step(render=True)

        samples = []
        for index in range(3):
            world.reset()
            for _ in range(30):
                world.step(render=False)
            for _ in range(8):
                world.step(render=True)
            joints = np.asarray(hand.get_joint_positions(), dtype=float)
            joint_velocities = np.asarray(hand.get_joint_velocities(), dtype=float)
            if not np.all(np.isfinite(joints)) or not np.all(np.isfinite(joint_velocities)):
                raise RuntimeError("nonfinite hand state after reset")
            entities = {}
            state_parts = [joints, joint_velocities]
            for entity_id, block in blocks.items():
                position, orientation = block.get_world_pose()
                linear_velocity = block.get_linear_velocity()
                angular_velocity = block.get_angular_velocity()
                values = [np.asarray(item, dtype=float) for item in (
                    position, orientation, linear_velocity, angular_velocity,
                )]
                if not all(np.all(np.isfinite(item)) for item in values):
                    raise RuntimeError(f"nonfinite {entity_id} state after reset")
                state_parts.extend(values)
                entities[entity_id] = {
                    "position_m": values[0].tolist(), "orientation_wxyz": values[1].tolist(),
                    "linear_velocity_m_s": values[2].tolist(),
                    "angular_velocity_rad_s": values[3].tolist(),
                }
            samples.append({
                "reset_index": index,
                "episode_id": f"tabletop-probe-20260924/reset-{index}",
                "observed_monotonic_s": time.monotonic(),
                "state_sha256_rounded_1e-4": hashlib.sha256(
                    np.round(np.concatenate(state_parts), 4).tobytes()
                ).hexdigest(),
                "joint_velocity_max_abs_rad_s": float(np.max(np.abs(joint_velocities))),
                "entities": entities,
            })
            rgba = camera.get_rgba()
            if rgba is not None:
                frame = np.asarray(rgba)
                if frame.ndim == 3 and frame.shape[2] == 4 and frame.size:
                    rgb_range = int(np.max(frame[..., :3]) - np.min(frame[..., :3]))
                    if rgb_range > 10:
                        image_path = args.output_dir / f"overhead_{index}.png"
                        Image.fromarray(frame.astype(np.uint8)).save(image_path)
                        samples[index]["camera_capture"] = {
                            "path": image_path.name,
                            "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                            "shape": list(frame.shape), "rgb_range": rgb_range,
                            "episode_id": samples[index]["episode_id"],
                            "observed_monotonic_s": samples[index]["observed_monotonic_s"],
                        }
                    else:
                        samples[index]["camera_capture_issue"] = "frame_has_no_rgb_variation"
                else:
                    samples[index]["camera_capture_issue"] = "invalid_frame_shape"
            else:
                samples[index]["camera_capture_issue"] = "camera_returned_no_frame"

        report = {
            "schema_version": "rrm-hand-tabletop-probe/v1",
            "scene_recipe_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "scene_seed": 0, "physics_dt_s": 1 / 120,
            "asset_url": ASSET_URL, "asset_sha256": ASSET_SHA256,
            "articulation_root": roots[0], "joint_names": list(hand.dof_names),
            "scene_entities": {
                "red_block": "/World/red_block", "blue_block": "/World/blue_block",
                "tray_1": "/World/Tray", "table": "/World/Table",
            },
            "camera_prim": "/World/OverheadCamera",
            "reset_samples": samples,
            "reset_hashes_match": len({item["state_sha256_rounded_1e-4"] for item in samples}) == 1,
            "controller_command_sent": False, "ros_connected": False,
            "execution_dispatch": False,
        }
        (args.output_dir / "probe.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8",
        )
        print(json.dumps({
            "schema_version": report["schema_version"],
            "reset_hashes_match": report["reset_hashes_match"],
            "output_dir": str(args.output_dir),
            "execution_dispatch": False,
        }, sort_keys=True), flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
