# Global Packages

The **global** layer gives the robot memory and direction beyond sensor range: a persistent 3D map of everywhere it has been, and a global planner that reasons over that map to produce a coarse waypoint path. Its output — `global_plan` — is the handoff to the [local layer](../local/index.md), which refines it into collision-free trajectory segments. Because global planning is not flight-critical, it is the one part of the stack that may run off-vehicle: the `lite_offload_global` split stack moves it to a ground host.

## Sub-layers

- [**World Model**](world_model/index.md) — the persistent VDB voxel map built from filtered sensor clouds
- [**Planning**](planning/index.md) — global planners (reference implementation: random walk exploration)

## Launch

The global layer is composed by the selected stack's entry launch file (`stacks/<name>/launch/*.launch.xml`): the trunk stacks include `vdb_mapping_ros2.py` and `random_walk_planner.launch.xml` directly — see `stacks/full_default/launch/stack.launch.xml`. The `global_bringup` package (`robot/ros_ws/src/global/global_bringup`) owns the cross-package VDB config (`config/vdb_params.yaml`), which the stack entry file passes to the mapper.

## Key Interchanges

- [`global_map` (§3)](../interface_conventions.md#3-global_map-global-world-model) — the VDB map topics the global planner consumes
- [`global_plan` (§4)](../interface_conventions.md#4-global_plan-global-waypoint-path) — the waypoint path handed to the local planner; the one interchange that may cross a machine boundary in split stacks

## See Also

- [System Architecture — Global Layer](../system_architecture.md#global-layer)
- [Task Executors](../tasks.md) — the exploration task server lives in the global planner
