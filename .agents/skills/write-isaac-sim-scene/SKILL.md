---
name: write-isaac-sim-scene
description: Create custom simulation scenarios in Isaac Sim by declaring them on top of the shared pegasus_app.PegasusApp base class. Use when creating test scenarios, multi-robot simulations, or custom environments for testing autonomy modules.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Write an Isaac Sim Scene (Standalone Launch Script)

## When to Use

Creating custom simulation environments for testing autonomy modules, multi-robot scenarios, or specific environmental conditions.

## The One Rule That Matters

**Do NOT copy-paste an existing launch script wholesale.** All shared boilerplate (SimulationApp creation, extension enabling, Pegasus world + environment loading, stage prep, drone/sensor spawning, the run loop) lives once in `simulation/isaac-sim/launch_scripts/pegasus_app.py`. A launch script is a *scenario declaration*: an environment URL, a list of drone configs, sensor toggles, and (only if needed) hook overrides. If you find yourself copying more than ~50 lines, you are re-creating the duplication this base class removed.

## Prerequisites

- Isaac Sim container image present (`airstack image-pull`)
- The scenario you want: which environment, how many drones, which sensors

## How a Scene Reaches the Simulator

`airstack up --sim isaac` starts the isaac-sim service, which (with `.env`'s default `ISAAC_SIM_USE_STANDALONE=true`) runs the Python file named by `ISAAC_SIM_SCRIPT_NAME` from `simulation/isaac-sim/launch_scripts/`. Scripts must live in that directory; set the variable to the filename only.

Env vars every script honors automatically (via the base class — do not re-implement):

| Env var | Effect |
|---|---|
| `ISAAC_SIM_HEADLESS` | run without a window |
| `ISAAC_SIM_LIVESTREAM` (+`_UDP_PORT`) | headless + WebRTC livestream |
| `PLAY_SIM_ON_START` | auto-play the timeline after setup (`airstack up --play`) |

## Steps

### 1. Create the script from the minimal template

Copy `barebones_pegasus_launch.py` (an environment, no drones) or start from this skeleton. The **import-order contract** is the only fragile part: Kit requires the `SimulationApp` to exist before any `omni.*`/`pegasus.*` import.

```python
#!/usr/bin/env python
"""One-line description of the scenario."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

simulation_app = create_simulation_app()   # FIRST — before any omni/pegasus import

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp, row_spawn_configs  # noqa: E402


def main():
    PegasusApp(
        env_url=SIMULATION_ENVIRONMENTS["Default Environment"],
        drone_configs=row_spawn_configs(int(os.environ.get("NUM_ROBOTS", "1"))),
        enable_lidar=os.environ.get("ENABLE_LIDAR", "false").lower() == "true",
    ).run()


if __name__ == "__main__":
    main()
```

### 2. Declare the scenario via constructor kwargs

The full list with defaults is in `PegasusApp.__init__`'s signature and docstring; the ones you'll set:

| Kwarg | Purpose |
|---|---|
| `env_url` | A `SIMULATION_ENVIRONMENTS[...]` entry or any `omniverse://` / file USD URL |
| `drone_configs` | Per-drone dicts (below); `row_spawn_configs(n, spacing_m, z_m)` for the standard row |
| `stage_scale` | `0.01` for cm-authored Nucleus assets, `1.0` for metric scenes |
| `enable_camera`, `camera_offset` | ZED stereo subgraph per drone (default on, offset `[0.2, 0, -0.05]`) |
| `enable_lidar`, `lidar_min_range`, ... | RTX Ouster lidar subgraph per drone |
| `dome_light` | `True` (defaults), `False`, or `{"prim_path":…, "intensity":…, "exposure":…}` |
| `world_gps_origin` | `(lat, lon, alt)` — writes per-drone PX4 GPS homes before SITL boots (see [spawning_drones.md](../../../docs/simulation/isaac_sim/spawning_drones.md)) |
| `scale_spawn_positions` | `True` when spawn meters must be converted into non-metric stage units |
| `save_scene_to` | Directory to export a self-contained USD of the prepared scene |
| `extra_extensions` | Additional Kit extensions to enable |

Per-drone config dict keys: `domain_id` (required — ROS domain and default vehicle id; MAVLink port `14540 + vehicle_id`), `x_m`/`y_m`/`z_m`, `orient` (quaternion `[x,y,z,w]`), and optional overrides `prim`, `node_name`, `lidar`, `lidar_min_range`, `camera_offset`.

### 3. Custom behavior goes in hooks, not copied blocks

Subclass `PegasusApp` and override (each receives the loaded USD stage):

- `pre_scene_prep(stage)` — right after the environment loads (e.g. `dedupe_physics_scenes`, `reference_root_prims_under_world` for imported scenes)
- `post_scene_prep(stage)` — after scale/colliders/dome light, before drones (e.g. overhead map camera)
- `post_spawn(stage)` — after all drones exist (e.g. author extra scene-level prims such as a mocap interface)

Reference subclass to study (not copy): `example_multi_drone_scene_import.py` (Nucleus scene import, explicit poses, overhead camera, GPS origins). For a `post_spawn` example (mocap authoring), see the asm_optitrack module's launch scripts.

Stage-prep helpers live in `simulation/isaac-sim/utils/scene_prep.py` (`add_colliders`, `scale_stage_prim`, `add_dome_light`, `add_orthographic_camera`, …) — documented in [spawning_drones.md](../../../docs/simulation/isaac_sim/spawning_drones.md).

Scene-level things that need to happen **before Pegasus imports** (e.g. overriding the Nucleus asset root via `carb.settings`) go at script top level right after `create_simulation_app()` — see the top of `example_multi_drone_scene_import.py`.

### 4. Run it

```bash
ISAAC_SIM_SCRIPT_NAME=my_scenario.py airstack up --sim isaac --play --wait
```

`--wait` (or `airstack ready`) blocks until the sim publishes `/clock`, the autonomy nodes are up, and PX4 is armable — so a hang here tells you which layer is broken. Watch script output from the host with `airstack logs isaac-sim` (tmux panes are mirrored to docker logs) or attach with `airstack connect isaac-sim`.

For multi-drone scenarios, `airstack up --sim isaac --robots N` keeps `NUM_ROBOTS` (robot containers) and the launch script consistent; if your custom script reads `NUM_ROBOTS`, say so in its docstring — preflight warns when `--robots > 1` is used with a custom script name.

### 5. Verify

1. `python3 -m py_compile simulation/isaac-sim/launch_scripts/my_scenario.py`
2. `airstack up --dry-run --sim isaac` with your `ISAAC_SIM_SCRIPT_NAME` — preflight validates the config
3. Full bring-up with `--wait`; then `ros2 topic hz` the sensor topics per drone (see the debug-module skill)
4. For scenarios meant to gate CI: run the relevant system-test marks (`airstack test -m liveliness --sim isaacsim ...`)

## Pitfalls

- ❌ Importing anything `omni.*`/`pegasus.*` before `create_simulation_app()` — Kit crashes or hangs
- ❌ Copying the extension-enable loop / run loop / stage-prep blocks into your script — they're in the base class
- ❌ Duplicate `domain_id`s in `drone_configs` — port and domain collisions, silent MAVROS failures
- ❌ Hardcoding a drone count while robot containers scale with `NUM_ROBOTS` — extra robots will wait forever for a PX4 that doesn't exist
- ❌ Forgetting `scale_spawn_positions=True` for cm-authored scenes — drones spawn 100× too far apart
- ❌ Re-reading `PLAY_SIM_ON_START`/`ISAAC_SIM_HEADLESS` yourself — the base class already does

## Documentation

Follow [update-documentation](../update-documentation): a scenario intended for others should be mentioned in `docs/simulation/isaac_sim/index.md` and, if it introduces new patterns, documented alongside [spawning_drones.md](../../../docs/simulation/isaac_sim/spawning_drones.md).
