# Controls

Controls dictate the actuation of the robot: they close the loop between the planned trajectory and the robot's actual state, and publish control commands to the topics defined by the [Robot Interface](../../interface/index.md).

AirStack splits control into two nodes:

- [**Trajectory Controller**](../../../../../robot/ros_ws/src/local/controls/trajectory_controller/README.md) (`trajectory_controller`) — a pure-pursuit trajectory manager that advances a **tracking point** and **look-ahead point** along the current trajectory (it is not itself a feedback controller)
- **PID Controller** (`pid_controller`) — a cascaded position/velocity PID that drives the drone toward the tracking point and publishes roll/pitch/yaw-rate/thrust commands to the interface

Both are perpetual nodes and run onboard only — control never crosses a machine boundary (see the [Interface Conventions Specification](../../interface_conventions.md)).
