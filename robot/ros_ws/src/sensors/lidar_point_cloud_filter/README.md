# lidar_point_cloud_filter

Subscribes to a raw `sensor_msgs/PointCloud2`, removes points whose distance from the origin is less than `near_range_m` (distance in the cloud frame, same idea as the exploration planner’s lidar pre-process), and publishes a filtered cloud as **xyz float32** only.

## Topic flow (sim → robot)

In **Isaac Sim / Pegasus** example launch scripts (e.g. `example_one_px4_pegasus_launch_script.py`, `example_multi_px4_pegasus_launch_script.py` with `ENABLE_LIDAR`), the RTX LiDAR OmniGraph publishes **`point_cloud_raw`** under the vehicle namespace (e.g. `/robot_N/sensors/ouster/point_cloud_raw`). That stream crosses the sim→robot bridge and is what this node **subscribes** to.

On the **robot** stack:

| Topic | Role |
|-------|------|
| `.../sensors/ouster/point_cloud_raw` | **Input** — dense cloud from the sim (or hardware driver); may include near-field self-hits / clutter. |
| `.../sensors/ouster/point_cloud` | **Output** — same logical topic name consumers expect for “the” LiDAR cloud, but **filtered** (near-range sphere crop, xyz-only). |

Downstream nodes should use **`point_cloud`** for planning / visualization unless they explicitly need the raw stream.

## Parameters

| Parameter | Meaning |
|-----------|--------|
| `near_range_m` | Points with Euclidean range strictly less than this (meters) are dropped. `<= 0` disables filtering. |
| `input_topic` | Subscription topic; absolute path recommended (see defaults). |
| `output_topic` | Publication topic. |
| `qos_depth` | History depth (keep last). |
| `qos_reliable` | If true, **RELIABLE** (matches Isaac Replicator and typical RViz). If false, **BEST_EFFORT** for drivers that publish that way. |

Defaults are in `config/lidar_point_cloud_filter.yaml`. `$(env ROBOT_NAME)` is expanded when launch loads the file with `allow_substs="true"`. Python fallbacks use the `ROBOT_NAME` environment variable the same way.

## Launch

```bash
ros2 launch lidar_point_cloud_filter lidar_point_cloud_filter.launch.xml
```

Included from `sensors_bringup` under the robot and `sensors` namespaces. Defaults use **`sensors/ouster/point_cloud_raw` → `sensors/ouster/point_cloud`** to match Pegasus / Isaac and `vdb_params`. For RTX-only topic names, override `input_topic` and `output_topic` (for example under `sensors/lidar/...`).

## System tests (`sensors` mark)

Sensor checks (sim + robot topic rates, LiDAR validation) live in repo-root **`tests/test_sensors.py`** (`pytest -m sensors`), which runs **after** **`tests/test_liveliness.py`** in the default collection order. For **Isaac Sim** (`--sim isaacsim`), that suite:

- Proves the **filtered** topic is alive (`ros2 topic echo --once` on `.../point_cloud` — large clouds are not probed with `ros2 topic hz`).
- Runs `scripts/validate_lidar_filter_clouds.py` inside each robot container: checks the **filtered** cloud against `near_range_m`, optionally compares behavior when **`point_cloud_raw`** has near-field returns.

**Microsoft AirSim** does not guarantee `sensors/ouster` topics on that profile; those steps are skipped there.

See [`tests/README.md`](../../../../../tests/README.md) (Bring-up scope) for how `airstack_env` applies when you combine `liveliness` and `sensors` marks.
