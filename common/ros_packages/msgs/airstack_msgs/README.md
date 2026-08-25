# airstack_msgs

`airstack_msgs` is AirStack's core interface package: the ROS 2 message and service definitions that modules exchange at the stack's interchange points — trajectories and waypoints for the trajectory controller, an extended odometry type, and the command services for the robot interface, trajectory controller, and takeoff/landing planner. The package defines the *types*; which canonical topics and services carry them is specified authoritatively by the [Interface Conventions Specification](../../../../docs/robot/autonomy/interface_conventions.md) (types and QoS there are verified against the observed wiring of the reference stack).

A number of definitions in this package have **no consumer in the AirStack trunk** — they were added for search-and-track mission planning work and are kept for downstream/module use. The tables below say so explicitly rather than inventing semantics; do not build new integrations on them without checking their real users first.

## At a glance

| Definition | Kind | Status in trunk |
|---|---|---|
| [`Odometry`](msg/Odometry.msg) | msg | **Active** — `trajectory_controller/tracking_point`, `look_ahead` (spec [§5](../../../../docs/robot/autonomy/interface_conventions.md#5-trajectory-group--the-trajectory-controllers-contract--onboard-only)) |
| [`TrajectoryXYZVYaw`](msg/TrajectoryXYZVYaw.msg) | msg | **Active** — the trajectory-controller command surface (spec §5) |
| [`WaypointXYZVYaw`](msg/WaypointXYZVYaw.msg) | msg | **Active** — element of `TrajectoryXYZVYaw` |
| [`FixedTrajectory`](msg/FixedTrajectory.msg) | msg | **Active** — goal payload of `task_msgs/action/FixedTrajectoryTask` |
| [`KeepOutZone`](msg/KeepOutZone.msg) | msg | Defined; no trunk publisher/subscriber (referenced only inside `PlanRequest`/`SearchMissionRequest`) |
| [`PlanRequest`](msg/PlanRequest.msg) | msg | Defined; no trunk publisher/subscriber |
| [`SearchMissionRequest`](msg/SearchMissionRequest.msg) | msg | Defined; no trunk publisher/subscriber |
| [`SearchPrior`](msg/SearchPrior.msg) | msg | Defined; no trunk publisher/subscriber (referenced only inside `PlanRequest`/`SearchMissionRequest`) |
| [`TaskAssignment`](msg/TaskAssignment.msg) | msg | Defined; no trunk publisher/subscriber |
| [`query/TextQueryResponse`](msg/query/TextQueryResponse.msg) | msg | Defined; no trunk consumer (intended for semantic-query modules) |
| [`RobotCommand`](srv/RobotCommand.srv) | srv | **Active** — served by `robot_interface` at `interface/robot_command` (spec §7) |
| [`TakeoffLandingCommand`](srv/TakeoffLandingCommand.srv) | srv | **Active** — served by `takeoff_landing_planner` at `takeoff_landing_planner/set_takeoff_landing_command`; the GCS-facing takeoff/land command (spec §8, related-service note) |
| [`TrajectoryMode`](srv/TrajectoryMode.srv) | srv | **Active** — served by `trajectory_controller` at `trajectory_controller/set_trajectory_mode` (spec §5) |
| [`PlanToWaypoint`](srv/PlanToWaypoint.srv) | srv | Defined but **not built** — absent from the `rosidl_generate_interfaces` list in [CMakeLists.txt](CMakeLists.txt); no trunk consumer |

## Messages

### Odometry

Extended odometry: `nav_msgs/Odometry` minus the covariances, plus feed-forward acceleration and jerk. This is the type the trajectory controller publishes on `trajectory_controller/tracking_point` and `trajectory_controller/look_ahead` (note: **airstack_msgs**, not nav_msgs — a classic type mismatch when wiring new modules; see spec §5). Consumers include the PID controller, both DROAN planners, the takeoff/landing planner, the random-walk global planner, and the drone safety monitor.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Stamp + frame of `pose` |
| `child_frame_id` | `string` | Frame of `twist` (body frame) |
| `pose` | `geometry_msgs/Pose` | Position and orientation |
| `twist` | `geometry_msgs/Twist` | Linear/angular velocity |
| `acceleration` | `geometry_msgs/Vector3` | Linear acceleration (feed-forward term for the controller) |
| `jerk` | `geometry_msgs/Vector3` | Linear jerk (feed-forward term for the controller) |

### TrajectoryXYZVYaw and WaypointXYZVYaw

The trajectory command interchange (spec §5, **onboard-only**): planners send these to the trajectory controller on `trajectory_controller/trajectory_override` (replaces the current trajectory) and `trajectory_controller/trajectory_segment_to_add` (appends). Producers in the trunk: `droan_local_planner`, `droan_gl`, `takeoff_landing_planner`, and the `fixed_trajectory_task` server; the [trajectory_library](../../../../robot/ros_ws/src/local/planners/trajectory_library/README.md) package converts between this type and its internal `Trajectory` class.

`TrajectoryXYZVYaw`:

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Stamp + frame the waypoints are expressed in |
| `waypoints` | `WaypointXYZVYaw[]` | Ordered waypoint list |

`WaypointXYZVYaw` ("XYZ, Velocity, Yaw"):

| Field | Type | Meaning |
|---|---|---|
| `position` | `geometry_msgs/Point` | Waypoint position (m, trajectory frame) |
| `velocity` | `float64` | Speed *magnitude* at this waypoint (m/s) — direction is inferred from the segment direction (see `trajectory_library`'s `Trajectory` constructor) |
| `yaw` | `float64` | Heading (rad, about +Z) |
| `acceleration` | `geometry_msgs/Vector3` | Feed-forward acceleration (often left zero) |
| `jerk` | `geometry_msgs/Vector3` | Feed-forward jerk (often left zero) |

### FixedTrajectory

A parametric trajectory specification: a shape `type` plus free-form key/value `attributes`. It is the goal payload of `task_msgs/action/FixedTrajectoryTask` (`tasks/fixed_trajectory`, served by the `fixed_trajectory_task` node in the `trajectory_controller` package). Types accepted by the trunk server: `Figure8`, `Circle`, `Racetrack`, `Line`, `Point`, `Lawnmower`; attributes are shape parameters such as `frame_id`, `velocity`, `radius`, `length`, `width`, `height` (see `robot/ros_ws/src/local/controls/trajectory_controller/src/fixed_trajectory_task.cpp` for each shape's accepted keys). Goals are sent from the GCS `action_relay` and the RViz Tasks Panel.

| Field | Type | Meaning |
|---|---|---|
| `type` | `string` | Trajectory shape name (e.g. `Figure8`) |
| `attributes` | `diagnostic_msgs/KeyValue[]` | Shape parameters as string key/value pairs |

### KeepOutZone

A vertical cylinder to avoid: center `(x, y)`, a `z` band, and a radius. **Defined; no trunk publisher/subscriber** — it appears only as a field of `PlanRequest` and `SearchMissionRequest` below.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Stamp + frame |
| `x`, `y` | `float64` | Cylinder center (m) |
| `z_min`, `z_max` | `float64` | Vertical extent (m) |
| `radius` | `float64` | Cylinder radius (m) |

### PlanRequest

A request for a search/coverage plan: start state, wind, planning budget, speed objective, search bounds, priors, and keep-out zones. **Defined; no trunk publisher/subscriber.** Field meanings below come from the comments in the `.msg` file.

| Field | Type | Meaning (from file comments) |
|---|---|---|
| `header` | `std_msgs/Header` | — |
| `start_pose` | `geometry_msgs/Pose` | Plan from this position and orientation |
| `wind_speed` | `geometry_msgs/Vector3` | Wind speed in m/s |
| `max_planning_time` | `float32` | Seconds |
| `maximum_range` | `float32` | Budget |
| `desired_speed` | `float32` | Desired flight speed in m/s |
| `search_bounds` | `geometry_msgs/Polygon` | Constraint/objective region |
| `search_priors` | `SearchPrior[]` | Prior information |
| `keep_out_zones` | `KeepOutZone[]` | Prior information |
| `clear_tree` | `bool` | Replan flag |
| `scenario` | `uint32` | "To match up with the correct plan" |

### SearchMissionRequest

A search mission: bounds, priors, and keep-out zones (a `PlanRequest` without the per-plan budget/state fields). **Defined; no trunk publisher/subscriber.**

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | — |
| `search_bounds` | `geometry_msgs/Polygon` | Mission area |
| `search_priors` | `SearchPrior[]` | Prior information |
| `keep_out_zones` | `KeepOutZone[]` | Zones to avoid |

### SearchPrior

Prior probability information over a region for search planning. **Defined; no trunk publisher/subscriber** (referenced only inside `PlanRequest`/`SearchMissionRequest`).

| Field | Type | Meaning (from file comments) |
|---|---|---|
| `header` | `std_msgs/Header` | — |
| `grid_prior_type` | `uint8` | One of `POLYGON_PRIOR=1` (must be a convex polygon), `LINE_SEG_PRIOR=2`, `POINT_PRIOR=3` |
| `points_list` | `geometry_msgs/Polygon` | Polygon, line segment, or list of points |
| `value` | `float32[]` | Initial value for the region |
| `priority` | `float32[]` | Higher = more important; empty ⇒ 1.0 |
| `sensor_model_id` | `uint8[]` | Sensor model type per region; default 0 |

### TaskAssignment

Assigns a search or track task (with an embedded `PlanRequest`) to a robot. **Defined; no trunk publisher/subscriber.**

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | — |
| `assigned_task_type` | `uint8` | `SEARCH=1` or `TRACK=2` |
| `assigned_task_number` | `uint32` | Per the file comment: "index of the target to track?" |
| `plan_request` | `PlanRequest` | The plan to execute |

### query/TextQueryResponse

Response to a text query against a robot semantic map ("robot semantic query" per the file comment): a tag plus a GPS geofence for where it applies. **Defined and built; no trunk consumer** — intended for semantic-search modules.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp and frame_id |
| `tag_name` | `string` | The tag associated with the query or response |
| `geofence` | `sensor_msgs/NavSatFix[]` | GPS fixes outlining the geofence |

## Services

### RobotCommand

Low-level vehicle commands into the interface layer. Served by `robot_interface` at `interface/robot_command` (spec §7); called by the takeoff/landing task executors to request control, arm, and command takeoff/land through MAVROS/PX4.

| Request | Response |
|---|---|
| `uint8 command` — one of `REQUEST_CONTROL=0`, `ARM=1`, `DISARM=2`, `TAKEOFF=3`, `LAND=4`, `SET_LOW_THRUST_MODE=5`, `UNSET_LOW_THRUST_MODE=6` | `bool success` |

### TakeoffLandingCommand

The GCS-facing takeoff/land command, served by `takeoff_landing_planner` at `takeoff_landing_planner/set_takeoff_landing_command` (spec §8, related-service note). The planner runs the full sequence (request control → arm → generate takeoff/landing trajectory → hand it to the trajectory controller) on behalf of the caller.

| Request | Response |
|---|---|
| `uint8 command` — one of `TAKEOFF=0`, `LAND=1`, `NONE=2` | `bool accepted` |

### TrajectoryMode

Sets the trajectory controller's tracking mode; served at `trajectory_controller/set_trajectory_mode` (spec §5). Task servers and the safety monitor call it to pause, hold pose, track, append segments, or rewind. See the [Trajectory Controller README](../../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md) for what each mode does.

| Request | Response |
|---|---|
| `int32 mode` — one of `PAUSE=0`, `ROBOT_POSE=1`, `TRACK=2`, `ADD_SEGMENT=3`, `REWIND=4` | `bool success` |

### PlanToWaypoint

Requests a plan to a goal pose. **Defined in `srv/` but not listed in `CMakeLists.txt`'s `rosidl_generate_interfaces`, so it is not generated or usable**; no trunk consumer.

| Request | Response |
|---|---|
| `geometry_msgs/Pose goal_pose`, `uint8 command` | `bool success` |

## Sibling interface packages

Two sibling packages under `common/ros_packages/msgs/` complete AirStack's shared interfaces: [`task_msgs`](../task_msgs/) defines the eight task action types (`TakeoffTask`, `LandTask`, `NavigateTask`, `FixedTrajectoryTask`, `ExplorationTask`, `SemanticSearchTask`, `CoverageTask`, `ChatTask`) served at `tasks/<task_name>` — see [Task Executors](../../../../docs/robot/autonomy/tasks.md) and spec §8; and [`behavior_tree_msgs`](../behavior_tree_msgs/) carries the behavior-tree engine's runtime plumbing — node `Status` (FAILURE/RUNNING/SUCCESS) and `Active` signals, `BehaviorTreeCommand(s)` for setting condition states, and `GraphVizXdot(Compressed)` for streaming the rendered tree to GUIs.
