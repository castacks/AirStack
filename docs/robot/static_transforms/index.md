# Static Transforms

Static transforms pin down the fixed geometric relationships between frames — where sensors sit on the body, and how each robot's `map` frame relates to the `world` frame — so that every module can transform data into a common frame without per-module configuration. On the robot they come from two places, both set up by the launch preamble in [`autonomy_bringup/launch/robot.launch.xml`](https://github.com/castacks/AirStack/blob/develop/robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml), which runs for every stack.

## The Frame Tree

- **`world`** — fixed origin
- **`map`** — child of `world` (static identity); ENU, meters — the planning/state frame
- **`base_link`** — the body frame, positioned by the state estimate; sensor frames hang off it via the URDF

## `world` → `map`

The preamble publishes a **static identity transform** `world → map` via `tf2_ros static_transform_publisher` (node `world_to_map_broadcaster`). `world` is the fixed origin; `map` is the ENU planning/state frame in which odometry pose, the global plan, and the map live. The dynamic `map → base_link` relationship comes from the state estimate, not a static transform.

## Sensor Extrinsics (URDF)

Sensor and body-frame extrinsics are published by `robot_state_publisher` from the robot URDF: the preamble includes `robot_descriptions/launch/robot_state_publisher.launch.py` with the `urdf_file` launch argument, which defaults to the `URDF_FILE` environment variable. `URDF_FILE` is set in the top-level `.env` (default: `robot_descriptions/iris/urdf/iris_with_sensors.pegasus.robot.urdf`) and passed into the robot container by `robot/docker/robot-base-docker-compose.yaml`; fleet files can override it per robot via their vehicle definition.

## See Also

- [Interface Conventions — TF frames and units](../autonomy/interface_conventions.md#tf-frames-and-units) — the canonical frame table (`world`, `map`, `base_link`) and unit conventions
- [Frame Conventions](../../development/intermediate/frame_conventions.md) — the concept page explaining the frame tree
