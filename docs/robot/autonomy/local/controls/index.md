# Controls

Controls dictate the actuation of the robot: they close the loop between the planned trajectory and the robot's actual state, and publish control commands to the topics defined by the [Robot Interface](../../interface/index.md).

AirStack splits control into two nodes:

- [**Trajectory Controller**](../../../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md) (`trajectory_controller`) — a pure-pursuit trajectory manager that advances a **tracking point** and **look-ahead point** along the current trajectory (it is not itself a feedback controller)
- **PID Controller** (`pid_controller`) — a cascaded position/velocity PID that drives the drone toward the tracking point and publishes roll/pitch/yaw-rate/thrust commands to the interface

Both are perpetual nodes and run onboard only — control never crosses a machine boundary (see the [Interface Conventions Specification](../../interface_conventions.md)).

## Search follower (MTL)

The [`mtl_search`](../../../../../stacks/mtl_search/README.md) stack adds
[**`mtl_trajectory_follower`**](../../../../../robot/ros_ws/src/local/controls/mtl_trajectory_follower/README.md)
between the trajectory controller and the PID controller:

- **Arbitration.** The stack remaps the trajectory controller's tracking point to
  `trajectory_controller/tracking_point_nominal`. The follower owns the canonical
  `trajectory_controller/tracking_point`: it forwards the nominal point when no search is
  active, and publishes an arc-length carrot (`L = 1.2·R_min`) on the planned search track
  while one is. The PID controller is unchanged apart from a stack-local speed clamp.
- **Gimbal.** The follower also commands the gimbal (`gimbal/cmd_pitch_yaw`) at 20 Hz with
  per-axis slew limits.
