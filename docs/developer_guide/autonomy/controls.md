# Controls

Code located in `AirStack/ros_ws/src/robot/autonomy/controls/`.

## Launch

Launch files are under `src/robot/autonomy/controls/controls_bringup/launch`.

Can be launched with `ros2 launch controls_bringup launch_controls.yaml`


## RobotInterface 

Package `robot_interface` is a ROS2 node that interfaces with the robot's hardware.
The `RobotInterface` _gets robot state_ and forwards it to the autonomy stack, 
and also _translates control commands_ from the autonomy stack into the command for the underlying hardware.
Note the base class is unimplemented.
Specific implementations should extend `class RobotInterface` in `robot_interface.hpp`, for example MAVROSInterface.

### State
The `RobotInterface` broadcasts the robot's pose as a TF2 transform.
It also publishes the robot's odometry as a `nav_msgs/Odometry` message to `$(arg robot_name)/controls/robot_interface/odometry`.

### Commands
The commands are variations of the two main command modes: Attitude control and Position control.
These are reflected in [MAVLink](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) and supported by both PX4 and [Ardupilot](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-commands).

The RobotInterface node subscribes to:

- `$(arg robot_name)/controls/robot_interface/cmd_attitude_thrust` of type `mav_msgs/AttitudeThrust.msg`
- `$(arg robot_name)/controls/robot_interface/cmd_rate_thrust` of type `mav_msgs/RateThrust.msg`
- `$(arg robot_name)/controls/robot_interface/cmd_roll_pitch_yawrate_thrust` of type `mav_msgs/RollPitchYawrateThrust.msg`
- `$(arg robot_name)/controls/robot_interface/cmd_torque_thrust` of type `mav_msgs/TorqueThrust.msg`
- `$(arg robot_name)/controls/robot_interface/cmd_velocity` of type `geometry_msgs/TwistStamped.msg`
- `$(arg robot_name)/controls/robot_interface/cmd_position` of type `geometry_msgs/PoseStamped.msg`

All messages are in the robot's body frame, except `velocity` and `position` which use the frame specified by the message header.


## Custom Robot Interface
Implementations should do the following:

### Broadcast State
Implementations of `RobotInterface` should obtain the robot's pose and broadcast it as a TF2 transform.

Should look something like:
```c++
// callback function triggered by some loop
void your_callback_function(){
    // ...
    geometry_msgs::msg::TransformStamped t;
    // populate the transform, e.g.:
    t.header = // some header
    t.transform.translation.x = // some value
    t.transform.translation.y = // some value
    t.transform.translation.z = // some value
    t.transform.rotation = // some quaternion
    // Send the transformation
    this->tf_broadcaster_->sendTransform(t);
    // ...
}
```

### Override Command Handling
Should override all `virtual` functions in `robot_interface.hpp`:

- `cmd_attitude_thrust_callback`
- `cmd_rate_thrust_callback`
- `cmd_roll_pitch_yawrate_thrust_callback`
- `cmd_torque_thrust_callback`
- `cmd_velocity_callback`
- `cmd_position_callback`
- `request_control`
- `arm`
- `disarm`
- `is_armed`
- `has_control`
