# Adding a Controller

AirStack splits control into two roles ([Controls overview](local/controls/index.md)), and the first decision is which one you are replacing:

- **Trajectory controller** — a pure-pursuit trajectory *manager*, not itself a feedback controller. It **owns the [trajectory group (spec §5)](interface_conventions.md#5-trajectory-group-the-trajectory-controllers-contract-onboard-only)**: it consumes `trajectory_controller/trajectory_segment_to_add` and `trajectory_override` (`airstack_msgs/msg/TrajectoryXYZVYaw`), serves the `set_trajectory_mode` service, and emits `tracking_point` and `look_ahead` (`airstack_msgs/msg/Odometry` — not `nav_msgs`). Reference: [Trajectory Controller](../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md).
- **Feedback controller** — closes the loop between the tracking point and the vehicle's actual state and emits the [`control_setpoint` (spec §6)](interface_conventions.md#6-control_setpoint-controller-interface-command-onboard-only) command into the interface. Reference: `pid_controller` (`robot/ros_ws/src/local/controls/pid_controller` — no README; the cascaded position→velocity PID is described in the [Trajectory Controller README's Control Architecture section](../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md#control-architecture)).

The verified chain in every reference stack (`full_default`, `full_droan_cpu`, `full_macvo`, `lite_default`, `lite_offload_global` onboard) is:

```text
trajectory_controller/tracking_point (airstack_msgs/Odometry, §5)
        │                                    + odometry_conversion/odometry (§2)
        ▼
control/pid_controller  ──►  interface/cmd_roll_pitch_yawrate_thrust
                             (mav_msgs/RollPitchYawrateThrust, §6) ──► robot_interface → MAVROS/PX4
```

**Both roles are onboard-only.** Spec §5 and §6 names may never appear in a split stack's `bridge.yaml` — `airstack doctor` hard-errors on it. The rationale is the spec's safety floor: command authority flows through the trajectory controller (arming, safety monitoring, takeover come for free to anything publishing `trajectory_override`), and that floor collapses if control crosses a link that can drop. A controller can never run offboard.

This guide assumes you know the [layered architecture](index.md) and have flown a stack in sim. Link the [Interface Conventions Specification](interface_conventions.md) from your README instead of restating its tables.

## Package or module?

Decide early where the controller lives:

- **In-tree package** — a package under `robot/ros_ws/src/local/controls/`, or a scaffolded module boundary in your fork via `airstack module create --in-tree <name>`.
- **Module repo** — shareable, version-pinned, own CI and Docker dependency layer, added with `airstack module add <url> --version <pin>`. See [AirStack Modules](../../development/modules.md) and the [create-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md).

The wiring steps below are identical either way.

## Path A: replace the feedback controller (the common case)

Swap `pid_controller` for your own attitude/velocity controller. The §5 surface stays owned by the stock trajectory controller — you only consume its setpoint.

Conventions worth copying from the reference (all in `pid_controller.cpp` / its launch file):

- Runs as node `pid_controller` under the `control` namespace (node path `/robot_1/control/pid_controller`); gains load from a params YAML passed with `allow_substs="true"`.
- Gains are **dynamic parameters** (`airstack::dynamic_param` from `airstack_common`) — per-axis `p/i/d/ff/min/max/constant` plus a `_d_alpha` derivative filter — tunable at runtime with `ros2 param set`, no rebuild between tuning iterations.
- It exposes a `reset_integrators` subscription (`std_msgs/msg/Empty`, relative name in its namespace) so flight phases can clear integral windup; keep an equivalent if your controller integrates.
- It is control-rate agnostic: it computes on every `tracking_point` message (the trajectory controller ticks at 20 Hz) rather than running its own timer.

### 1. Create the package

Follow the [add-ros2-package skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-ros2-package/SKILL.md) and the [Module Integration Checklist](integration_checklist.md), under `robot/ros_ws/src/local/controls/`. Declare every topic endpoint as a launch argument defaulting to its canonical spec name — copy the pattern from `pid_controller/launch/pid_controller.launch.xml` (`pid_controller_odometry_topic`, `pid_controller_tracking_point_topic`, `pid_controller_command_topic`).

**Verify:** `docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <your_package>"` exits cleanly.

### 2. Conform to the interchange

Inputs: `trajectory_controller/tracking_point` (§5, `airstack_msgs/msg/Odometry` — pose, velocity, acceleration, jerk along the trajectory) and `odometry_conversion/odometry` ([§2](interface_conventions.md#2-odometry-primary-state-estimate), `nav_msgs/msg/Odometry`). Output: one §6 command dialect — `interface/cmd_roll_pitch_yawrate_thrust` (`mav_msgs/msg/RollPitchYawrateThrust`, the blessed publisher slot the PID fills today) or the alternates `interface/cmd_pose` / `interface/cmd_velocity`. Never publish `tracking_point` or `look_ahead` yourself — that is impersonating the trajectory controller, and `doctor --live` flags it.

**Verify:** with the node running under a full stack, `ros2 topic info /robot_1/interface/cmd_roll_pitch_yawrate_thrust` lists your node as the only publisher, and `ros2 topic info /robot_1/trajectory_controller/tracking_point` lists it as a subscriber.

### 3. Wire it into a custom stack

Controller variants are named stacks (see [Creating a Custom Stack Topology](../../development/creating_a_stack.md) and the [single-locus rule](../../development/stacks.md#the-single-locus-rule-and-its-lint)): `airstack stack new full_default full_my_controller`, then in `stacks/full_my_controller/launch/stack.launch.xml` replace the `pid_controller.launch.xml` include with your controller's include. Canonical arg defaults mean a conforming controller needs no include args.

**Verify:** `airstack up --stack full_my_controller --sim isaac --robots 1 && airstack ready` succeeds and `ros2 node list` shows your controller in place of `control/pid_controller`.

### 4. Verify with wiring and a flight

1. `airstack test -m wiring --stack full_my_controller` regenerates `wiring.md`; `airstack stack diff full_default full_my_controller` should show exactly the controller swap. `airstack doctor --live --stack full_my_controller` must be clean.
2. Fly it: `airstack test -m takeoff_hover_land --sim isaacsim --num-robots 1 -v` — every command the vehicle receives in all four phases flows through your feedback controller, so this is the cheapest full-chain exercise (`waypoint_flight` adds planner behavior, not controller coverage). Then `airstack test -m autonomy --trajectory-types Circle,Figure8` for tracking *quality*: it flies fixed trajectories straight through the controllers and records cross-track error and path RMSE — the numbers your gains actually move.

## Path B: replace the trajectory controller itself

This is a much bigger lift: you take over the **entire §5 surface**, and every task server, the local planner, the feedback controller, and the safety monitor are your clients. Study the [Trajectory Controller README](../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md) end-to-end before writing code. Your replacement must:

- **Consume** `trajectory_controller/trajectory_segment_to_add` (appended segments from the local planner, stitched into the live trajectory near the current tracking position) and `trajectory_override` (complete replacement trajectories from takeoff/land and fixed-trajectory task servers).
- **Serve** `trajectory_controller/set_trajectory_mode` (`airstack_msgs/srv/TrajectoryMode`) with all five modes — `ROBOT_POSE`, `TRACK`, `ADD_SEGMENT`, `PAUSE`, `REWIND` — including the transition semantics in the README. Clients include `takeoff_landing_planner`, the fixed-trajectory task, the local planner, and `drone_safety_monitor`.
- **Publish** `tracking_point` and `look_ahead` (`airstack_msgs/msg/Odometry`), keeping `look_ahead` far enough ahead for the planner's cycle, plus `trajectory_completion_percentage` (`std_msgs/msg/Float32`), which task servers use to judge goal completion.
- **Broadcast** the four TF frames (`tracking_point`, `look_ahead_point`, and their `_stabilized` variants) the README documents.
- **Honor the safety integration**: the safety monitor commands `PAUSE`/`REWIND` through your mode service on state-estimate timeout — this path is why §5 is a spec, and it must work before anything else does.

Wire it the same way as Path A step 3 (replace the `trajectory_controller.launch.xml` include; the `fixed_trajectory_task.launch.xml` include comes from the same package — replace or keep it deliberately), then verify as in Path A step 4 — but fly with `airstack test -m waypoint_flight --sim isaacsim --num-robots 1 -v`: its chain (takeoff → NavigateTask route → land) exercises the whole surface — `TRACK`/override for takeoff and landing, `ADD_SEGMENT` stitching under a continuously replanning local planner, and the mode transitions between them — where `takeoff_hover_land` never enters `ADD_SEGMENT`.

## See also

- [Interface Conventions Specification](interface_conventions.md) — §2 odometry, §5 trajectory group, §6 control_setpoint
- [Controls overview](local/controls/index.md) · [Trajectory Controller README](../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md)
- [Creating a Custom Stack Topology](../../development/creating_a_stack.md) · [AirStack Stacks](../../development/stacks.md)
- [Module Integration Checklist](integration_checklist.md)
- Skills: [add-ros2-package](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-ros2-package/SKILL.md) · [create-module](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md)
