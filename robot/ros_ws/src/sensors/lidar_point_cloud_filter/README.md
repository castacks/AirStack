# lidar_point_cloud_filter

Subscribes to a raw `sensor_msgs/PointCloud2`, removes points whose distance from the origin is less than `near_range_m` (distance in the cloud frame, same idea as the exploration planner’s lidar pre-process), and publishes a filtered cloud as **xyz float32** only.

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
