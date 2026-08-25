# Local Packages

The **local** layer closes the robot's short-range sense-plan-act loop: a fast local world model built from live sensor data, a reactive local planner that avoids obstacles the global map is too slow or too coarse to capture, and the controllers that turn planned trajectories into commands for the [interface](../interface/index.md). The loop is coupled through the trajectory controller's **look-ahead point** — the local planner plans forward from where the controller will soon be, and streams trajectory segments back to it.

## Sub-layers

- [**World Model**](world_model/index.md) — disparity-based C-space obstacle representation for fast collision queries
- [**Planning**](planning/index.md) — the DROAN local planner: turns the global plan into short, collision-free trajectory segments
- [**Controls**](controls/index.md) — trajectory controller (tracking/look-ahead point management) and PID controller (attitude/thrust commands)

## Launch

Local modules ship their own canonical launch files and are composed flat by the selected stack's entry launch file (`stacks/<name>/launch/*.launch.xml`) — the trunk stacks include `takeoff_landing_planner`, the trajectory controller, `droan_gl`, and the PID controller directly; see `stacks/full_default/launch/stack.launch.xml` for the composed wiring.

## Key Interchanges

- [`global_plan` (§4)](../interface_conventions.md#4-global_plan-global-waypoint-path) — the coarse path handed down from the global layer; the local planner's main input
- [`trajectory` group (§5)](../interface_conventions.md#5-trajectory-group-the-trajectory-controllers-contract-onboard-only) — the trajectory controller's onboard-only contract: `trajectory_segment_to_add`, `trajectory_override`, `tracking_point`, `look_ahead`
- [`control_setpoint` (§6)](../interface_conventions.md#6-control_setpoint-controller-interface-command-onboard-only) — the PID controller's command into the interface layer

## See Also

- [System Architecture — Local Layer](../system_architecture.md#local-layer)
- [Global](../global/index.md) — the upstream producer of `global_plan`
