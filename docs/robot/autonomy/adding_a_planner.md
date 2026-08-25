# Adding a Planner

AirStack has two planner slots, and the first decision is which one you are filling:

- **Local planner** — a perpetual node plus a `NavigateTask` server that consumes the [`global_plan` (spec §4)](interface_conventions.md#4-global_plan-global-waypoint-path), a world model input (disparity or point clouds), and the trajectory controller's `look_ahead`/`tracking_point`, and emits short collision-free segments on the [trajectory-controller surface (spec §5)](interface_conventions.md#5-trajectory-group-the-trajectory-controllers-contract-onboard-only) (`trajectory_controller/trajectory_segment_to_add`, `airstack_msgs/msg/TrajectoryXYZVYaw`). Reference: [DROAN](../../../robot/ros_ws/src/local/planners/droan_local_planner/README.md) ([overview](local/planning/index.md)). Spec §5 is **onboard-only** — a local planner can never run offboard.
- **Global planner** — a [task executor](tasks.md): an action server at `tasks/<task_name>` ([spec §8](interface_conventions.md#8-tasks-task-action-servers)) that plans only while a goal is active, publishes the coarse path on [`global_plan` (spec §4)](interface_conventions.md#4-global_plan-global-waypoint-path), and delegates flying to the local planner via `tasks/navigate`. Reference: [Random Walk](../../../robot/ros_ws/src/global/planners/random_walk/README.md) ([overview](global/planning/index.md)). `global_plan` is the one interchange that may cross a machine boundary, so a global planner may run offboard (`lite_offload_global`).

This guide assumes you know the [layered architecture](index.md) and have flown a stack in sim. Link the [Interface Conventions Specification](interface_conventions.md) from your README instead of restating its tables.

## Package or module?

Decide early where the planner lives:

- **In-tree package** — fastest for trunk work: a package under `robot/ros_ws/src/local/planners/` or `robot/ros_ws/src/global/planners/`, or a scaffolded module boundary in your fork via `airstack module create --in-tree <name>` (lands under `robot/ros_ws/src/modules/<name>`).
- **Module repo** — shareable, version-pinned, own CI and Docker dependency layer, added with `airstack module add <url> --version <pin>`. See [AirStack Modules](../../development/modules.md) (the [researcher fork → module workflow](../../development/modules.md#the-researcher-workflow-fork-module)) and the [create-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md); [asm_macvo](../../modules/macvo.md) is the worked precedent for a capability shipped this way.

The wiring steps below are identical either way — a module's launch file is included by a stack entry file exactly like a trunk package's.

## Path A: local planner

### 1. Create the package

Follow the [add-ros2-package skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-ros2-package/SKILL.md) and the [Module Integration Checklist](integration_checklist.md), under `robot/ros_ws/src/local/planners/`. Declare every topic endpoint as a launch argument defaulting to its canonical spec name — a conventional stack then includes you with zero remaps.

**Verify:** `docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <your_package>"` exits cleanly.

### 2. Conform to the interchange

Inputs: `global_plan` (§4, `nav_msgs/Path`, `map` frame), your world model topic, `odometry_conversion/odometry` ([§2](interface_conventions.md#2-odometry-primary-state-estimate)), and the controller's `look_ahead` (§5 — plan from the look-ahead point, not the current pose). Output: `trajectory_controller/trajectory_segment_to_add` (§5). Serve `NavigateTask` at `tasks/navigate` (goal/feedback/result fields in [Task Executors → NavigateTask](tasks.md#navigatetask)); the [add-task-executor skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-task-executor/SKILL.md) covers the four action callbacks. Emitting `trajectory_segment_to_add` (rather than commanding the interface directly) is what buys you arming, safety monitoring, and takeover for free — the spec's safety floor. For candidate-trajectory generation, scoring helpers, and `TrajectoryXYZVYaw` conversion, use the [trajectory_library](../../../robot/ros_ws/src/local/planners/trajectory_library/README.md) instead of rolling your own.

**Verify:** with the node running under a full stack, `docker exec airstack-robot-desktop-1 bash -c "sws && ros2 topic info /robot_1/trajectory_controller/trajectory_segment_to_add"` lists your node as a publisher and the trajectory controller as the subscriber, and `ros2 action list` shows `/robot_1/tasks/navigate`.

### 3. Wire it into a stack

Planner variants are named stacks, not launch arguments — the [single-locus rule](../../development/stacks.md#the-single-locus-rule-and-its-lint) puts every wiring deviation in the stack entry file. The worked swap example is [full_droan_cpu](../../../stacks/full_droan_cpu/README.md): byte-identical to `full_default` except the local-planner block, where `stacks/full_droan_cpu/launch/stack.launch.xml` replaces the single `droan_gl.launch.xml` include with two lines:

```xml
<include file="$(find-pkg-share droan_local_planner)/launch/droan_local_planner.launch.xml" />
<include file="$(find-pkg-share disparity_expansion)/launch/disparity_expansion.launch.xml" />
```

Do the same for yours: `airstack stack new full_default full_my_planner`, then in `stacks/full_my_planner/launch/stack.launch.xml` replace the DROAN include with your planner's include (plus any world-model include it needs). Only deviations from canonical names appear as include args — see how `full_macvo` passes exactly one (`droan_gl_disparity_topic`).

**Verify:** `airstack up --stack full_my_planner --sim isaac --robots 1 && airstack ready` succeeds and `ros2 node list` shows your planner in place of DROAN.

### 4. Verify with wiring and a flight

1. Snapshot and diff the wiring: `airstack test -m wiring --stack full_my_planner` regenerates `stacks/full_my_planner/wiring.md`; the diff against `full_default` should be exactly your planner block. `airstack stack diff full_default full_my_planner` compares the generated wiring directly.
2. Live check: `airstack doctor --live --stack full_my_planner` — doctor flags anything but the trajectory controller publishing `look_ahead`/`tracking_point`, and hard-errors if §5 names ever appear in a split stack's `bridge.yaml`.
3. Fly it: `airstack test -m waypoint_flight --sim isaacsim --num-robots 1 --stress-iterations 1 -v` drives a waypoint route through your `tasks/navigate` server and judges the odometry track. For a manual flight, take off from the GCS and send a `NavigateTask` goal (`ros2 action send_goal --feedback /robot_1/tasks/navigate task_msgs/action/NavigateTask ...` with a `global_plan` path and `goal_tolerance_m`), or use the [GCS waypoint editor](../../gcs/waypoints_and_geofences.md). (`airstack test -m autonomy` flies fixed trajectories straight through the controller — it checks the stack still flies, but never touches your planner.)

## Path B: global planner

### 1. Create the package

Same as Path A step 1, under `robot/ros_ws/src/global/planners/`. Keep planning logic in a ROS-free class ([Global Planning](global/planning/index.md) explains why); the node wraps it.

**Verify:** `bws --packages-select <your_package>` exits cleanly.

### 2. Implement it as a task executor

Follow [Adding a New Task Executor](tasks.md#adding-a-new-task-executor) and the [add-task-executor skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-task-executor/SKILL.md): pick or add a `.action` type in `task_msgs`, implement the four action callbacks, and remap the server to `tasks/<your_task>` in your module launch file (§8). Subscribe to the map ([`global_map`, spec §3](interface_conventions.md#3-global_map-global-world-model) — today the VDB visualization topic) and odometry (§2); publish `global_plan` (§4, `nav_msgs/Path` in the `map` frame, last pose = goal) and delegate flying by sending `NavigateTask` goals to `tasks/navigate` while your goal is active — exactly the [random_walk cascade](../../../robot/ros_ws/src/global/planners/random_walk/README.md).

**Verify:** `ros2 action list` shows `/robot_1/tasks/<your_task>`, and `ros2 topic info /robot_1/global_plan` lists your node as publisher and the local planner as subscriber.

### 3. Wire it into a stack

Swapping the global planner is replacing one include: in your copy of the stack entry file (`airstack stack new full_default full_my_global`), replace the `random_walk_planner.launch.xml` include with yours — the same one-line swap the [exploration planner](global/planning/index.md#example-planners) documents. The node keeps its canonical names, so no include args are needed unless you deviate.

**Verify:** `airstack up --stack full_my_global --sim isaac --robots 1 && airstack ready`; `ros2 node list` shows your planner and no `random_walk_node`.

### 4. Verify with wiring and a flight

1. `airstack test -m wiring --stack full_my_global` then `airstack stack diff full_default full_my_global` — the only delta is the global-planner swap; `airstack doctor --live --stack full_my_global` is clean.
2. Fly it: take off, then activate your task, e.g. for an exploration-type planner `ros2 action send_goal /robot_1/tasks/exploration task_msgs/action/ExplorationTask '{...}' --feedback` (a full goal example is in [Task Executors](tasks.md#explorationtask)), and watch `ros2 topic echo /robot_1/global_plan --once` update and the drone follow it. `airstack test -m takeoff_hover_land` confirms you have not disturbed the base flight chain.

## See also

- [Interface Conventions Specification](interface_conventions.md) — §2 odometry, §3 global_map, §4 global_plan, §5 trajectory group, §8 tasks
- [Module Integration Checklist](integration_checklist.md) — package structure, launch conventions, integration testing commands
- [Local Planning](local/planning/index.md) · [Global Planning](global/planning/index.md) — layer overviews and references
- [Task Executors](tasks.md) — action types and the task cascade
- Skills: [add-ros2-package](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-ros2-package/SKILL.md) · [add-task-executor](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-task-executor/SKILL.md) · [create-module](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md)
