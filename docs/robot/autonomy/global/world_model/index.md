[//]: # "global"
# World Model

The global world model maintains a persistent 3D representation of everywhere the robot has sensed — the memory the [global planner](../planning/index.md) plans over. AirStack's shipped default is **VDB Mapping**: an OpenVDB-based voxel map, built in the `map` frame from the filtered LiDAR cloud, vendored in-tree with the config owned by `global_bringup`.

## Packages

- [**VDB Mapping ROS 2**](../../../../../robot/ros_ws/src/global/world_models/vdb_mapping_ros2/README.md) (`vdb_mapping_ros2`, in-tree) — ROS 2 wrapper around the FZI [VDB Mapping](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping) library; upstream wrapper repo: [vdb_mapping_ros2](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros2)

## Launch

The mapper is composed by the selected stack's entry launch file (`stacks/<name>/launch/*.launch.xml`), which includes `vdb_mapping_ros2.py` with `global_bringup/config/vdb_params.yaml` — see `stacks/full_default/launch/stack.launch.xml`.

## Key Interchanges

All map topics are specified in [`global_map` (§3)](../../interface_conventions.md#3-global_map-global-world-model): `vdb_map_visualization` (today's de facto map interchange, consumed by the reference global planner), the `vdb_map_updates` / `_sections` / `_overwrites` grids for remote/split map synchronization, and the `vdb_map_pointcloud` export. The map lives in the `map` frame.

In the `lite_offload_global` split stack, VDB mapping runs **offboard** on the ground host: the filtered sensor cloud crosses the bridge (per the stack's `bridge.yaml`) and the map is built where the global planner consumes it.

## See Also

- [Global layer overview](../index.md)
- [System Architecture — Global Layer](../../system_architecture.md#global-layer)
