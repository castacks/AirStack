# mtl_trajectory_follower

The local search follower of the [`mtl_search`](../../../../../../stacks/mtl_search/README.md)
stack. It does three jobs:

- flies this robot's `mtl_msgs/SearchPlan` by arc-length carrot pursuit;
- points the gimbal at the planned ground aim point;
- arbitrates the canonical `trajectory_controller/tracking_point`.

## Behaviour

| State | Tracking point out | Gimbal |
|---|---|---|
| `IDLE` (no started plan) | forwards `tracking_point_nominal` verbatim, so takeoff, land and hover are unchanged | parked at `idle_gimbal_pitch_deg` |
| `INGRESS` | straight to the track start at the plan altitude | leads to the first aim point |
| `SEARCH` | carrot at arc length `s* + L`, `L = 1.2·R_min`; `s*` is the closest-point projection inside a forward window of `1.5·L` (prevents lane capture on serpentines) | planned aim point at `s* + lead` |
| `COMPLETE` / `ABORTED` | holds the last point, then calls `set_trajectory_mode(ROBOT_POSE)` and returns to `IDLE` | parked |

**Gimbal law.** The aim vector `Δ = aim − p_camera` gives:

- two-axis mode: `yaw = atan2(Δy, Δx)`, `pitch = atan2(−Δz, d_h)`;
- single-axis (roll-only) mounts: the planner's scheduled roll, plus a pitch nudge limited to
  `|Δp| ≤ 5°`.

In both modes a per-axis slew limit applies. Commands go out at 20 Hz on
`gimbal/cmd_pitch_yaw` (`geometry_msgs/Vector3`: `x = roll`, `y = pitch`, `z = yaw`,
radians). Angles are earth-frame (ENU) Z-Y-X, and `pitch > 0` looks down.

**Aborts.** The follower aborts on any of:

- stale odometry (`odometry_timeout_s`);
- `behavior/drone_safety_monitor/state_estimate_timed_out`;
- `search/abort`;
- a plan older than `max_plan_age_s`. This one guards against a latched plan replaying after
  a restart.

**TF.** It publishes `base_link → camera_gimbal_link`, built from the measured `gimbal/state`
(which falls back to the command), plus a static `camera_gimbal_link → camera_optical_frame`.

## Topics

These are all launch args with canonical defaults; see
`launch/mtl_trajectory_follower.launch.xml`.

- In: `odometry_conversion/odometry`, `search/plan`,
  `trajectory_controller/tracking_point_nominal`, `gimbal/state`, `search/abort`, the safety
  timeout.
- Out: `trajectory_controller/tracking_point`, `gimbal/cmd_pitch_yaw`, `search/follower_status`,
  `search/carrot`, `search/aim_point`.
- Service client: `trajectory_controller/set_trajectory_mode`.

## Code

- `gimbal_math.py`: rotation conventions and the gimbal laws. Stdlib only.
- `follower_core.py`: the ROS-free state machine, also used by
  `scripts/mtl_offline_mission.py`.
- `follower_node.py`: the rclpy shell.

## Tests

`test/test_gimbal_math.py`, `test/test_follower_core.py` and `test/test_follower_node.py`
cover the node with hermetic `rclpy` stubs (`test/_ros_stubs.py`). They run on the host
without ROS.
