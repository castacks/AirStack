# DROAN Local Planner

## Overview

DROAN Local Planner is a disparity-based local obstacle avoidance planner based on the publication ["DROAN - Disparity-space representation for obstacle avoidance."](https://www.ri.cmu.edu/app/uploads/2018/01/root.pdf) It selects the best collision-free trajectory from a pre-generated library by evaluating each candidate against a CPU-based cost map built from stereo disparity observations.

This is the CPU implementation. For the GPU-accelerated alternative that uses OpenGL for disparity expansion and collision checking, see [droan_gl](../droan_gl/README.md).

## How It Works

### 1. Cost Map (Disparity Graph)

The cost map is built by three supporting packages:

1. **disparity_expansion** — takes stereo disparity and expands each obstacle point outward by the robot radius, producing a 3D obstacle cloud in C-space
2. **disparity_graph** — maintains a rolling graph of (pose, obstacle cloud) observations, discarding old entries as the drone moves
3. **disparity_graph_cost_map** — queries the disparity graph to assign a collision cost to any 3D point in space

The cost map plugin is configurable via the `cost_map` parameter (default: `disparity_graph_cost_map::DisparityGraphCostMap`).

### 2. Trajectory Library

A fixed set of candidate trajectories (snap-based splines in different directions) is loaded from a YAML config file at startup. Each trajectory is a sequence of 3D waypoints parameterized by time.

### 3. Trajectory Scoring (5 Hz)

At each planning step, every trajectory in the library is evaluated:

- Each waypoint is queried in the cost map to get an obstacle cost.
- A **safety cost** is computed from the maximum obstacle cost along the trajectory (or only the last waypoint, if `evaluate_only_last_waypoint` is set).
- A **deviation cost** measures lateral distance from the global plan at the trajectory endpoint.
- A **forward progress** term rewards trajectories whose endpoint is farther along the global plan.

The composite cost is:

```text
cost = safety_cost_weight * safety_cost
     + deviation_from_global_plan_weight * deviation
     - forward_progress_forgiveness_weight * progress
```

The trajectory with the lowest cost is published.

### 4. Goal Modes

The planner supports three goal modes:

- **`GLOBAL_PLAN`** — scores trajectories against the subscribed global plan path. The planner finds the closest point on the global plan to the drone and trims the path from that point forward. This is the mode used when a NavigateTask goal is active.
- **`CUSTOM_WAYPOINT`** — scores toward a single manually commanded waypoint (`custom_waypoint` topic).
- **`AUTO_WAYPOINT`** — scores toward waypoints interpolated from a rolling buffer of past drone positions (useful for unguided exploration without a global plan).

### 5. Rewind Monitoring

Two stuck conditions trigger a map-clearing rewind:

| Condition | Threshold parameter | Recovery |
| --------- | ------------------- | -------- |
| All trajectories in collision | `all_in_collision_duration_threshold` | Rewind for `map_clearing_max_rewind_time` or `map_clearing_rewind_distance` |
| Drone stationary too long | `stuck_duration_threshold`, `stationary_distance_threshold` | Same rewind |

After a rewind, the planner waits `map_clearing_wait_time` seconds before triggering another rewind to allow the cost map to update.

---

## Task Executor

This node is a **task executor**: it runs as a ROS 2 action server and is activated on demand via a `NavigateTask` goal. It does not plan continuously — planning only happens while a goal is active.

**Action server:** `/{robot_name}/tasks/navigate`
**Type:** `task_msgs/action/NavigateTask`

### Cascade

```text
random_walk_planner  →  NavigateTask  →  droan_local_planner
                                                 ↓
                                      trajectory_segment_to_add
                                                 ↓
                                       trajectory_controller
```

### Goal parameters

| Field | Type | Description |
| ----- | ---- | ----------- |
| `global_plan` | nav_msgs/Path | Path to follow; last pose is the goal |
| `goal_tolerance_m` | float32 | Distance from goal pose to consider task complete (m) |

### Feedback (published ~1 Hz)

| Field | Type | Description |
| ----- | ---- | ----------- |
| `status` | string | `"navigating"` |
| `distance_to_goal` | float32 | 3D Euclidean distance to goal pose (m) |
| `current_position` | geometry_msgs/Point | Current tracking point position |

### Result

| Field | Type | Description |
| ----- | ---- | ----------- |
| `success` | bool | True if goal pose reached within tolerance; false if cancelled or error |
| `message` | string | `"Goal reached"`, `"Cancelled"`, or `"Node shutting down"` |

### Trajectory controller mode

On goal acceptance the node calls the `set_trajectory_mode` service with mode `ADD_SEGMENT`, enabling the trajectory controller to extend the trajectory buffer as new segments arrive. On goal completion or cancellation it restores mode `TRACK`.

### CLI test

```bash
ros2 action send_goal /robot_1/tasks/navigate task_msgs/action/NavigateTask \
  '{global_plan: {header: {frame_id: "map"}, poses: [{pose: {position: {x: 10.0, y: 0.0, z: 3.0}}}]}, goal_tolerance_m: 1.0}' \
  --feedback
```

---

## Parameters

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `execute_rate` | `5.0` | Planning rate (Hz) |
| `trajectory_library_config` | `""` | Path to trajectory library YAML |
| `cost_map` | `"PointCloudMapRepresentation"` | Cost map plugin class name |
| `safety_cost_weight` | `1.0` | Weight for obstacle cost in scoring |
| `deviation_from_global_plan_weight` | `1.0` | Weight for lateral deviation from global plan |
| `forward_progress_forgiveness_weight` | `0.5` | Weight for rewarding forward progress along plan |
| `obstacle_check_radius` | `1.0` | Radius (m) around each waypoint to check for obstacles; must be > robot radius |
| `evaluate_only_last_waypoint` | `false` | Score only the trajectory endpoint instead of all waypoints |
| `yaw_mode` | `"SMOOTH_YAW"` | `"SMOOTH_YAW"` or `"TRAJECTORY_YAW"` — how to assign yaw to output trajectory |
| `auto_waypoint_buffer_duration` | `30.0` | Seconds of past trajectory to buffer for AUTO_WAYPOINT mode |
| `auto_waypoint_spacing_threshold` | `0.5` | Min spacing (m) between buffered auto waypoints |
| `auto_waypoint_angle_threshold` | `30.0` | Min angle change (deg) to add a new auto waypoint |
| `custom_waypoint_timeout_factor` | `0.3` | Fraction of estimated travel time before custom waypoint times out |
| `custom_waypoint_distance_threshold` | `0.5` | Distance (m) to consider custom waypoint reached |
| `stuck_duration_threshold` | `6.0` | Seconds stationary before triggering rewind |
| `all_in_collision_duration_threshold` | `2.0` | Seconds all trajectories must be in collision to trigger rewind |
| `stationary_distance_threshold` | `0.5` | Max movement (m) over history window to be considered stationary |
| `stationary_history_duration` | `10.0` | Observation window (s) for stationary detection |
| `map_clearing_max_rewind_time` | `10.0` | Max rewind duration (s) |
| `map_clearing_rewind_distance` | `1.5` | Target rewind distance (m) |
| `map_clearing_wait_time` | `30.0` | Seconds to wait between rewinding again |

---

## Subscriptions

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `global_plan` | nav_msgs/Path | Global path for trajectory scoring (also set via NavigateTask goal) |
| `look_ahead` | airstack_msgs/Odometry | Look-ahead point from trajectory controller (trajectory planning origin) |
| `tracking_point` | airstack_msgs/Odometry | Current robot tracking point (for goal distance and stuck detection) |
| `custom_waypoint` | geometry_msgs/PoseStamped | Override waypoint for CUSTOM_WAYPOINT mode |
| `reset_stuck` | std_msgs/Empty | Manually clear stuck detection history |
| `clear_map` | std_msgs/Empty | Trigger map clearing |

## Publications

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `trajectory_segment_to_add` | airstack_msgs/TrajectoryXYZVYaw | Best local trajectory segment |
| `trajectory_override` | airstack_msgs/TrajectoryXYZVYaw | Direct trajectory override (used during rewind) |
| `local_planner_global_plan_vis` | visualization_msgs/MarkerArray | Global plan visualization |
| `trajectory_library_vis` | visualization_msgs/MarkerArray | Trajectory library with collision status coloring |
| `stuck` | std_msgs/Bool | True when rewind is active |
| `rewind_info` | visualization_msgs/MarkerArray | Rewind status text markers |
| `obstacle_vis` | sensor_msgs/Range | Nearest obstacle distance (visualization) |
| `map_clearing_point` | geometry_msgs/PoseStamped | Target point for map clearing visualization |
