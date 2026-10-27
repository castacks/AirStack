#!/usr/bin/env python
"""DEBUG: a scene_gen frozen dataset scene with NUM_ROBOTS drones.

example_multi_px4_pegasus_launch_script.py, except for how the scene is referenced: frozen
scenes (e.g. .../Tornado/Suburban400/level_3/1/tornado_suburban400_lvl3_1.usd) are flattened
stages with no defaultPrim, so the plain reference resolves nothing. This references their
/World instead. Same env: ISAAC_SIM_SCENE (the USD), NUM_ROBOTS, ENABLE_LIDAR, and
ISAAC_SIM_SPAWN_XY, which here may also list one spawn per robot, "x,y[,z];x,y[,z];..." (e.g. the
SEI-COA demo's 2x3 grid "103,48;111,48;103,52;107,52;111,52;107,48"; z, default 0.07, for scenes
whose ground is not at 0, such as Disaster City's ~58 m). Select it with
  ISAAC_SIM_SCRIPT_NAME=../../../stacks/neurosymbolic/isaac/frozen_scene.py
"""
import os
import sys

sys.path.insert(0, "/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts")
from pegasus_app import create_simulation_app  # noqa: E402

simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import (  # noqa: E402
    PegasusApp,
    resolve_scene_from_env,
    resolve_spawn_center_from_env,
    row_spawn_configs,
    wait_for_stage,
)
from pxr import Sdf  # noqa: E402


class FrozenScene(PegasusApp):
    def pre_scene_prep(self, stage):
        refs = stage.GetPrimAtPath("/World/stage").GetReferences()
        refs.ClearReferences()
        refs.AddReference(self.env_url, Sdf.Path("/World"))
        wait_for_stage(stage)


def spawns(n):
    raw = os.environ.get("ISAAC_SIM_SPAWN_XY", "")
    if ";" not in raw:
        return row_spawn_configs(n, center_xy=resolve_spawn_center_from_env())
    xyz = [[float(v) for v in s.split(",")] + [0.07] for s in raw.split(";") if s.strip()]
    if len(xyz) < n:
        raise SystemExit(f"ISAAC_SIM_SPAWN_XY lists {len(xyz)} spawns for {n} robots")
    return [{"domain_id": i + 1, "x_m": v[0], "y_m": v[1], "z_m": v[2]} for i, v in enumerate(xyz[:n])]


def main():
    env_url, stage_scale = resolve_scene_from_env(SIMULATION_ENVIRONMENTS)
    n = int(os.environ.get("NUM_ROBOTS", "1"))
    print(f"[frozen_scene] {n} drone(s) in {env_url}")
    FrozenScene(env_url=env_url, stage_scale=stage_scale, drone_configs=spawns(n),
                enable_lidar=os.environ.get("ENABLE_LIDAR", "false").lower() == "true").run()


if __name__ == "__main__":
    main()
