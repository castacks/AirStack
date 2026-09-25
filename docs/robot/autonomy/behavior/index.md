# Behavior

The behavior layer, as shipped, is a **safety executive**: it watches the health of the running stack and issues safety commands when something goes wrong. High-level mission sequencing does not live here — task goals (takeoff, land, explore, navigate) are sent by the operator from the GCS to the [task executors](../tasks.md) hosted in the global and local layers; see [System Architecture — Task Cascade](../system_architecture.md#task-cascade).

## Launch

Behavior modules ship their own canonical launch files and are composed by
the selected stack's entry launch file, e.g.
`ros2 launch drone_safety_monitor drone_safety_monitor.launch.xml` — see
`stacks/full_default/launch/stack.launch.xml` for the composed wiring.

## Modules

- **`drone_safety_monitor`** (`robot/ros_ws/src/behavior/drone_safety_monitor`) — the safety executive: watches the state
  estimate for timeouts and issues safety commands. It runs onboard so the
  robot can failsafe even if every ground link is lost.
- **`mtl_metrics_logger`** (`robot/ros_ws/src/behavior/mtl_metrics_logger`,
  [README](../../../../robot/ros_ws/src/behavior/mtl_metrics_logger/README.md)): used by the
  [`mtl_search`](../../../../stacks/mtl_search/README.md) stack. It is a mission recorder,
  not a safety executive. It scores target detections online from the flown pose and the
  measured gimbal (Moon et al. sigmoid, miss-product accumulation). It also scores the
  **residual belief mass**: `P(target missed)` over the whole normalised prior, where lower is
  better, used to compare planners. It writes
  `runs/<run_id>/<robot>/{telemetry.csv,detection.json,residual_belief.csv,report.html}`.

## Key Interchanges

- [`safety` (§9)](../interface_conventions.md#9-safety-safety-executive-onboard-only) — the safety executive's onboard-only topics (`state_estimate_timed_out`, `command`); these may never cross a split-stack bridge
- [`tasks/*` (§8)](../interface_conventions.md#8-tasks-task-action-servers) — where mission-level goals actually enter the stack
