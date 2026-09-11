# Local Packages

The **local** layer closes the robot's short-range sense-plan-act loop: a fast local world model built from live sensor data, a local planner that avoids obstacles the global map is too slow or too coarse to capture, and the controllers that turn planned trajectories into commands for the [interface](../interface/index.md). The loop is coupled through the trajectory controller's trajectory group — the local planner streams trajectories (receding-horizon overrides for MIGHTY, look-ahead-anchored segments for DROAN) to the controller, which tracks them.

## Sub-layers

- [**World Model**](world_model/index.md) — the planner's short-range obstacle representation: MIGHTY's sliding voxel map (default) or DROAN's disparity-space C-space expansion
- [**Planning**](planning/index.md) — the local planner (MIGHTY by default; DROAN as the alternative): turns the global plan into short, collision-free trajectories
- [**Controls**](controls/index.md) — trajectory controller (tracking/look-ahead point management) and PID controller (attitude/thrust commands)

## Launch

Local modules ship their own canonical launch files and are composed flat by the selected stack's entry launch file (`stacks/<name>/launch/*.launch.xml`) — the trunk stacks include `takeoff_landing_planner`, the trajectory controller, the local planner (`mighty_module.launch.xml` from the `asm_mighty` module in `full_default`; `droan_gl.launch.xml` from `asm_droan` in `full_droan`), and the PID controller directly; see `stacks/full_default/launch/stack.launch.xml` for the composed wiring. The planner modules are external repos pinned in each stack's `modules.repos`; `airstack up` syncs them when missing.

## Key Interchanges

- [`global_plan` (§4)](../interface_conventions.md#4-global_plan-global-waypoint-path) — the coarse path handed down from the global layer; the local planner's main input
- [`trajectory` group (§5)](../interface_conventions.md#5-trajectory-group-the-trajectory-controllers-contract-onboard-only) — the trajectory controller's onboard-only contract: `trajectory_segment_to_add`, `trajectory_override`, `tracking_point`, `look_ahead`
- [`control_setpoint` (§6)](../interface_conventions.md#6-control_setpoint-controller-interface-command-onboard-only) — the PID controller's command into the interface layer

## See Also

- [System Architecture — Local Layer](../system_architecture.md#local-layer)
- [Global](../global/index.md) — the upstream producer of `global_plan`
