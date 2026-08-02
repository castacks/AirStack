# Isaac Sim

Isaac Sim + [Pegasus](https://github.com/PegasusSimulator/PegasusSimulator)
integration: the container, the launch scripts that boot `SimulationApp` and
spawn drones, and the stage tooling they share.

| Path | Role |
|------|------|
| [`launch_scripts/`](launch_scripts/) | Entry points. `ISAAC_SIM_SCRIPT_NAME` in [`.env`](../../.env) picks which one `airstack up isaac-sim` runs. |
| [`utils/scene_prep.py`](utils/scene_prep.py) | Stage tooling every launch script shares: `scale_stage_prim`, `add_colliders`, `add_sky`, `get_stage_meters_per_unit`, `settle_rigid_props`. |
| [`docker/`](docker/) | Container definition and compose file. |
| [`extensions/`](extensions/) | Vendored Pegasus extension. |
| [`assets/`](assets/) | Robot descriptions and baked scene USDs. |

## Procedural scene generation

The city generator **is not here** — it lives at
[`scene_gen/`](../../scene_gen/) in the repo root, because it is sim-agnostic
(it only needs `pxr`) and carries its own configs, asset sets and asset
pipeline. See [`scene_gen/README.md`](../../scene_gen/README.md) for how it
works and [`scene_gen/GENERATION.md`](../../scene_gen/GENERATION.md) for the
config system.

Three launch scripts here drive it — `scene_preview_launch_script.py` (fastest
iteration, no drone), `local_scene_preview_launch_script.py` and
`generated_scene_launch_script.py` (full stack with a PX4 drone). They import
`scene_generator` / `compile_disaster` from `scene_gen/` and `scene_prep` from
`utils/`. Point any of them at a scene without editing code:

```bash
SCENE_CONFIG=suburban airstack up isaac-sim
```

Scenes whose asset set uses Objaverse assets need those cached first — they are
downloaded and converted to USD on the **host**, since this container has no
`uv`, `objaverse` or Blender:

```bash
.venv/bin/python scene_gen/prepare_assets.py suburban
airstack up isaac-sim
```

Re-running is a sub-second no-op, and skipping it is not fatal — uncached
assets render as placeholder prisms and the generator lists them at startup.
See [`scene_gen/GENERATION.md`](../../scene_gen/GENERATION.md).
