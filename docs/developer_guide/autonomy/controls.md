# Controls

Code located in `AirStack/ros_ws/src/robot/autonomy/controls/`.

## Launch

Launch files are under `src/robot/autonomy/controls/launch/`.

Can be launched with `ros2 launch src/robot/autonomy/controls/launch/launch_controls.yaml`

## Robot Interface class

The `RobotInterface` translates commands from the autonomy stack into the command for the underlying hardware.
Note the base class is unimplemented.
Specific implementations should extend `class RobotInterface` in `robot_interface.hpp`, for example MAVROSInterface.

The commands are variations of the two main command modes: Attitude control and Position control.
These are reflected in [MAVLink](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) and supported by both PX4 and [Ardupilot](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-commands).

the RobotInterface node subscribes to:

- `$(arg robot_name)/controls/robot_interface/attitude_thrust` of type `mav_msgs/AttitudeThrust.msg`

- `$(arg robot_name)/controls/robot_interface/rate_thrust` of type `mav_msgs/RateThrust.msg`

- `$(arg robot_name)/controls/robot_interface/roll_pitch_yawrate_thrust` of type `mav_msgs/RollPitchYawrateThrust.msg`

- `$(arg robot_name)/controls/robot_interface/torque_thrust` of type `mav_msgs/TorqueThrust.msg`

- `$(arg robot_name)/controls/robot_interface/velocity` of type `geometry_msgs/TwistStamped.msg`

- `$(arg robot_name)/controls/robot_interface/position` of type `geometry_msgs/PoseStamped.msg`

All messages are in the robot's body frame, except `velocity` and `position` which use the frame specified by the message header.

## Custom Robot Interface
