# Sensor Packages

The **sensors** layer holds ROS 2 nodes that sit next to hardware or simulation bridges: light preprocessing, remapping, and calibration helpers so **perception** and downstream layers see stable topics.

## Overview

The sensors layer is responsible for:

- **Bridged sensor topics**: Normalizing names and QoS for data coming from Isaac Sim, MAVROS, or onboard drivers
- **Preprocessing**: Near-range LiDAR cleanup and similar filters that should run on the robot graph, not only in simulation
- **Supporting documentation**: Patterns for optional tools such as the gimbal extension in simulation

## Launch

Sensor modules ship their own canonical launch files and are composed by the
stack entry file under the `sensors` namespace. E.g. the trunk stacks
include:

```bash
ros2 launch lidar_point_cloud_filter lidar_point_cloud_filter.launch.xml
```

See `stacks/full_default/launch/stack.launch.xml` for the composed wiring.

## Key Topics

### Outputs

- `/{robot_name}/sensors/ouster/point_cloud` — Filtered `sensor_msgs/msg/PointCloud2` (xyz) after `lidar_point_cloud_filter`, when using the default Ouster-style names in config

### Inputs

- `/{robot_name}/sensors/ouster/point_cloud_raw` — Raw cloud from the simulator or driver (typical input to the LiDAR filter)
- Other hardware- or bridge-specific topics as wired in the stack entry file (e.g. `stacks/full_default/launch/stack.launch.xml`; the observed graph is recorded in the stack's `wiring.md`)

Topic strings are parameterized with `$(env ROBOT_NAME)` in YAML; override `input_topic` / `output_topic` in the filter config if your stack uses different names.

## Modules

- [**LiDAR point cloud filter**](#lidar-point-cloud-filter) (`lidar_point_cloud_filter`) — near-range sphere filter for `PointCloud2`
- [**Gimbal (simulation)**](gimbal.md) — gimbal extension usage in simulation (documentation only; no robot-side package)

## LiDAR point cloud filter

**Package:** `robot/ros_ws/src/sensors/lidar_point_cloud_filter`

**Role:** Subscribe to a **raw** `sensor_msgs/msg/PointCloud2`, drop points whose distance from the cloud origin is below `near_range_m` (typical self-hit / multipath noise near the sensor), and republish a **clean xyz float32** cloud for mapping, exploration, RViz, and VDB.

**Why it exists:** Isaac Sim’s RTX OmniLidar path exposes a **`min_range` / `nearRangeM`** hook in Pegasus (`spawn_rtx_lidar.py`), but applying near range **inside the simulator is unreliable** across Kit builds (attribute missing or ineffective). That behavior is documented as a **known limitation** in [Pegasus / Isaac Sim setup](../../../simulation/isaac_sim/pegasus_scene_setup.md#rtx-lidar-near-range). **AirStack’s supported approach** is to run this **robot-side** filter so the stack always sees a consistent filtered topic.

**Defaults (configurable):**

- Parameters and defaults: `robot/ros_ws/src/sensors/lidar_point_cloud_filter/config/lidar_point_cloud_filter.yaml`
- Typical topics: `/{robot_name}/sensors/ouster/point_cloud_raw` → `/{robot_name}/sensors/ouster/point_cloud` (override `input_topic` / `output_topic` if your bridge uses different names, for example under `sensors/lidar/...`)
- QoS: `qos_reliable` defaults to **true** to match common Isaac bridges and RViz; see the package README

**Further detail:** `robot/ros_ws/src/sensors/lidar_point_cloud_filter/README.md`

## Configuration

- **Module launch files:** each sensor package ships its own canonical launch file — `robot/ros_ws/src/sensors/lidar_point_cloud_filter/launch/lidar_point_cloud_filter.launch.xml`
- **Stack wiring:** the stack entry files include these under the `sensors` namespace (`stacks/<name>/launch/*.launch.xml`, e.g. `stacks/full_default/launch/stack.launch.xml`); the observed graph is recorded in `stacks/full_default/wiring.md`
- **LiDAR filter:** `robot/ros_ws/src/sensors/lidar_point_cloud_filter/config/lidar_point_cloud_filter.yaml` (`near_range_m`, topics, QoS)

## See Also

- [System Architecture](../system_architecture.md) — overall autonomy stack architecture
- [Perception](../perception/index.md) — downstream consumers of filtered sensor data
- [Integration Checklist](../integration_checklist.md) — adding new sensor-layer packages
- [Pegasus / Isaac Sim — RTX LiDAR and near range](../../../simulation/isaac_sim/pegasus_scene_setup.md#rtx-lidar-near-range) — simulation-side `min_range` limitation and why the filter runs on the robot
