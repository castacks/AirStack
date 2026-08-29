# Local Planning

Local planners turn the coarse global plan into short, collision-free trajectory segments, reacting to obstacles the global map is too slow or too coarse to capture. They plan from the trajectory controller's look-ahead point and feed segments to it continuously.

AirStack's baseline local planner is DROAN, in two implementations:

- [**DROAN GL**](../../../../../robot/ros_ws/src/local/planners/droan_gl/README.md) (`droan_gl`) — GPU-accelerated, true-sphere disparity expansion via OpenGL shaders; the default in the `full_default` stack
- [**DROAN Local Planner**](../../../../../robot/ros_ws/src/local/planners/droan_local_planner/README.md) (`droan_local_planner`) — the CPU implementation, selected by the `full_droan_cpu` stack

Both are task executors serving `NavigateTask` at `/{robot_name}/tasks/navigate`.

Specialized maneuvers are handled by the [Takeoff Landing Planner](../../../../../robot/ros_ws/src/local/planners/takeoff_landing_planner/README.md), and candidate trajectories come from the [Trajectory Library](../../../../../robot/ros_ws/src/local/planners/trajectory_library/README.md).
