#!/usr/bin/env python3
"""Isolated Isaac Sim articulation/reset probe for the selected hand asset.

Run with Isaac Sim's python.sh in a separate headless process. This script does
not create a controller command or connect to AirStack's ROS graph.
"""

from __future__ import annotations

import hashlib
import json

ASSET_URL = (
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1/Isaac/"
    "IsaacLab/Robots/KukaAllegro/kuka.usd"
)


def main() -> int:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)
        add_reference_to_stage(usd_path=ASSET_URL, prim_path="/World/KukaAllegro")
        app.update()
        roots = [
            str(prim.GetPath()) for prim in get_current_stage().Traverse()
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and str(prim.GetPath()).startswith("/World/KukaAllegro")
        ]
        if len(roots) != 1:
            print(json.dumps({
                "schema_version": "rrm-hand-scene-probe/v1",
                "status": "NO_UNIQUE_ARTICULATION",
                "articulation_roots": roots,
                "execution_dispatch": False,
            }), flush=True)
            return 2
        hand = world.scene.add(
            SingleArticulation(prim_path=roots[0], name="kuka_allegro_probe"),
        )
        world.reset()
        baseline = np.asarray(hand.get_joint_positions(), dtype=float)
        if baseline.size == 0 or not np.all(np.isfinite(baseline)):
            raise RuntimeError("articulation joint state is missing or nonfinite")
        hand.set_joints_default_state(
            positions=baseline.copy(), velocities=np.zeros_like(baseline),
        )
        repetitions = []
        for index in range(3):
            world.reset()
            for _ in range(10):
                world.step(render=False)
            positions = np.asarray(hand.get_joint_positions(), dtype=float)
            velocities = np.asarray(hand.get_joint_velocities(), dtype=float)
            if positions.shape != baseline.shape or not np.all(np.isfinite(positions)):
                raise RuntimeError("post-reset joint positions are invalid")
            if velocities.shape != baseline.shape or not np.all(np.isfinite(velocities)):
                raise RuntimeError("post-reset joint velocities are invalid")
            sample = {
                "reset_index": index,
                "joint_position_max_abs_delta_from_default": float(
                    np.max(np.abs(positions - baseline))
                ),
                "joint_velocity_max_abs": float(np.max(np.abs(velocities))),
                "state_sha256_rounded_1e-4": hashlib.sha256(
                    np.round(np.concatenate((positions, velocities)), 4).tobytes()
                ).hexdigest(),
            }
            repetitions.append(sample)
        print(json.dumps({
            "schema_version": "rrm-hand-scene-probe/v1",
            "status": "OBSERVED",
            "asset_url": ASSET_URL,
            "articulation_root": roots[0],
            "joint_count": int(baseline.size),
            "joint_names": list(hand.dof_names),
            "reset_repetitions": repetitions,
            "execution_dispatch": False,
        }, sort_keys=True), flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
