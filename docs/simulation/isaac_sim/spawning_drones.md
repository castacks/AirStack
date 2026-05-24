# Spawning Drones

The reference launch scripts under `simulation/isaac-sim/launch_scripts/` cover the progression from a single drone in an empty world up to multiple drones in a custom imported scene with per-drone GPS origins:

| Script | Purpose |
|---|---|
| `barebones_pegasus_launch.py` | Minimal Pegasus boilerplate. Single drone, default environment, no scene import. Use as a template for new launch scripts. |
| `example_one_px4_pegasus_launch_script.py` | One PX4 drone with the standard sensor stack (ZED stereo, optional Ouster lidar) in the default environment. |
| `example_multi_px4_pegasus_launch_script.py` | `NUM_ROBOTS` drones spawned in a row in the default environment. Each drone gets its own ROS domain id (`1..N`). |
| `example_multi_drone_scene_import.py` | `NUM_ROBOTS` drones in an **imported scene** (USD from a Nucleus server) with per-drone GPS homes set via `gps_utils.set_gps_origins`. Use this as the starting point for any custom scene. |

The first three are vanilla Pegasus patterns; this page focuses on the multi-drone + custom-scene case where you also need correct GPS homes.

## The DRONE_CONFIGS pattern

`example_multi_drone_scene_import.py` declares all per-drone settings in a single list:

```python
DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": -3.0, "y_m": 3.5, "z_m": 0.15, "orient": [0, 0, 0, 1]},
    {"domain_id": 2, "x_m":  3.0, "y_m": 3.0, "z_m": 0.15, "orient": [0, 0, 0, 1]},
]
```

| Field | Purpose |
|---|---|
| `domain_id` | ROS domain id and PX4 vehicle id. The robot container with `ROS_DOMAIN_ID=1` will see this drone. |
| `x_m`, `y_m`, `z_m` | World-frame spawn position in meters. Convention: `+X = East`, `+Y = North`, `+Z = Up`. |
| `orient` | Spawn orientation quaternion `[x, y, z, w]`. |

To add another drone, append an entry with a fresh `domain_id` and a non-overlapping spawn position. Make sure the corresponding robot container is launched with the same `ROS_DOMAIN_ID` (`NUM_ROBOTS=N airstack up robot-desktop`).

## Per-drone GPS home — `gps_utils`

PX4 needs a GPS home per vehicle. `simulation/isaac-sim/launch_scripts/gps_utils.py` derives one from each drone's world-frame spawn position so all drones share a consistent geographic anchor and end up at distinct GPS coordinates spaced according to their spawn offsets.

```python
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

set_gps_origins(DRONE_CONFIGS)   # call once before spawning vehicles
```

`set_gps_origins` does two things per drone:

1. Calls `compute_gps_origin(x_m, y_m, z_m, world_origin)` to convert the spawn offset into `(lat, lon, alt)`. The conversion is a flat-Earth approximation — accurate at scene scales (a few hundred meters), not at continental scale.
2. Writes `PX4_HOME_LAT_<domain_id>`, `PX4_HOME_LON_<domain_id>`, `PX4_HOME_ALT_<domain_id>` into the process environment. The Pegasus PX4 OmniGraph node reads these when building each drone's `PX4MavlinkBackendConfig`, which passes them to the PX4 SITL subprocess as `PX4_HOME_LAT/LON/ALT`.

### World anchor

The world origin maps to `DEFAULT_WORLD_ORIGIN = (38.736832, -9.137977, 90.0)` — Lisbon, matching the Pegasus default. Override it for a scene set elsewhere:

```python
set_gps_origins(DRONE_CONFIGS, world_origin=(40.4433, -79.9436, 280.0))  # Pittsburgh
```

The anchor only affects the geographic location reported via GPS; nothing in the scene moves. Pick something close to where you want the drones to "be" — Foxglove's Map panel will center on it, and any GPS-referenced inputs to your stack will be relative to it.

## Scene prep helpers — `scene_prep.py`

`simulation/isaac-sim/utils/scene_prep.py` is the small toolbox of stage preparation helpers `example_multi_drone_scene_import.py` uses inside its post-load callback (after the stage is loaded, before drones spawn). The full file has more — what's documented here is what you'll reach for in 95% of scenes.

```python
from utils.scene_prep import (
    get_stage_meters_per_unit, scale_stage_prim, add_colliders,
    add_dome_light, save_scene_as_contained_usd,
    add_orthographic_camera, add_overhead_camera_publisher,
)

mpu, scene_scale_factor = get_stage_meters_per_unit(stage)
```

### Scaling — `scale_stage_prim`

USD scenes are authored at all sorts of stage units. To apply a uniform scale to the imported stage root once, before drones spawn:

```python
STAGE_SCALE = 0.01   # cm → m
scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
```

### Colliders — `add_colliders`

Recursively applies `UsdPhysics.CollisionAPI` to every `UsdGeom.Mesh` under the given prim. Imported environment USDs are usually visual-only; without this, drones fall through buildings.

```python
stage_prim = stage.GetPrimAtPath("/World/stage")
add_colliders(stage_prim)
```

Skips prims that already have the API applied. Run it on the stage root after `scale_stage_prim` returns.

### Lighting — `add_dome_light`

In case the scene is missing any lights, this adds a dome light that can act like an overhead 'sun'.

```python
add_dome_light(
    stage,
    prim_path="/World/DomeLight",
    intensity=3500.0,
    exposure=-5.0,   # negative = darker; tune per scene
)
```

### Overhead camera — `add_orthographic_camera` + `add_overhead_camera_publisher`

Used as a pair: one drops an orthographic camera over the scene, the other wires an OmniGraph to publish its frame plus three Float32 spec topics (`coverage_m`, `center_x_m`, `center_y_m`) the GCS uses to texture a ground plane in Foxglove's 3D panel.

```python
cam_path = add_orthographic_camera(
    stage,
    prim_path="/World/MapCamera",
    altitude_m=165.0,
    coverage_m=225.0,
    scene_scale_factor=scene_scale_factor,
    center_x_m=0.0,   # set if your area of interest isn't at world origin
    center_y_m=0.0,
)
add_overhead_camera_publisher(
    parent_graph_path="/World/MapCameraGraph",
    camera_prim_path=cam_path,
    topic="/sim/overhead/image",
    spec_topic="/sim/overhead/spec",
    center_x_topic="/sim/overhead/center_x",
    center_y_topic="/sim/overhead/center_y",
    frame_id="map",
    coverage_m=225.0,
    center_x_m=0.0,
    center_y_m=0.0,
    pixels_per_meter=10.0,
    domain_id=0,
)
```

Full setup, GCS-side rendering, and tuning knobs are on the **[Overhead Camera](overhead_camera.md)** page.

### Saving a self-contained copy — `save_scene_as_contained_usd`

For scenes you'd like to keep working with offline (no Nucleus connection), or for sharing a scene with collaborators, collect the root USD plus every referenced asset (textures, MDLs, sublayers) into a local directory:

```python
save_scene_as_contained_usd(
    source_usd_url=ENV_URL,
    output_dir="/tmp/collected_scene",
)
```

The collected folder contains a standalone root USD with relative references — load it directly via `omniverse://localhost/...` or a local file path. Note that this collects the **source USD as-is**: scale, colliders, dome light, and any other stage edits applied in this post-load callback are *not* baked into the saved copy. To capture the live stage with your modifications, first export the in-memory stage to a USD on disk (e.g. via `stage.GetRootLayer().Export(...)`) and pass that exported path as `source_usd_url`.

## Common issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Drone shows up at the world origin in Foxglove despite being elsewhere in sim | `set_gps_origins` not called, or called *after* spawn | Move the call before vehicle spawning |
| All drones share one GPS coordinate | `domain_id` collision in `DRONE_CONFIGS` | Give each drone a unique `domain_id` |
| Map panel centers on the wrong city | Wrong `world_origin` | Override the second arg to `set_gps_origins` |
| Drone position drifts in the wrong compass direction | Stage axis mismatch | Swap `x_m` ↔ `y_m` in `gps_utils.compute_gps_origin` |
| Robot container can't see the drone's topics | `ROS_DOMAIN_ID` ≠ `domain_id` in DRONE_CONFIGS | Match them, or set `NUM_ROBOTS` correctly |

## See also

- [Pegasus Scene Setup](pegasus_scene_setup.md) — single-drone authoring background
- [Overhead Camera](overhead_camera.md) — topdown ground texture
- [GCS Foxglove Visualization](../../gcs/foxglove.md) — how multi-robot poses render on the GCS
