# Robot Interface

The interface defines the communication between the autonomy stack running on the onboard computer and the robot's control unit.
For example, for drones it converts the control commands from the autonomy stack into MAVLink messages for the flight controller.

==TODO: This is not our diagram, must replace.==
![Interface Diagram](https://404warehouse.net/wp-content/uploads/2016/08/softwareoverview.png?w=800)

The code is located under `AirStack/ros_ws/src/robot/autonomy/0_interface/`.

## Launch

Launch files are under `src/robot/autonomy/0_interface/interface_bringup/launch`.

The main launch command is `ros2 launch interface_bringup interface.launch.xml`.

## RobotInterface

Package `robot_interface` is a ROS2 node that interfaces with the robot's hardware.
The `RobotInterface` _gets robot state_ and forwards it to the autonomy stack,
and also _translates control commands_ from the autonomy stack into the command for the underlying hardware.
Note the base class is unimplemented.
Specific implementations should extend `class RobotInterface` in `robot_interface.hpp`, for example `class MAVROSInterface`.

### State

The `RobotInterface` class broadcasts the robot's pose as a TF2 transform.
It also publishes the robot's odometry as a `nav_msgs/Odometry` message to `$(env ROBOT_NAME)/0_interface/robot_0_interface/odometry`.

### Commands

The commands are variations of the two main command modes: Attitude control and Position control.
These are reflected in [MAVLink](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) and supported by both PX4 and [Ardupilot](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-commands).

The RobotInterface node subscribes to:

- `/$(env ROBOT_NAME)/interface/cmd_attitude_thrust` of type `mav_msgs/AttitudeThrust.msg`
- `/$(env ROBOT_NAME)/interface/cmd_rate_thrust` of type `mav_msgs/RateThrust.msg`
- `/$(env ROBOT_NAME)/interface/cmd_roll_pitch_yawrate_thrust` of type `mav_msgs/RollPitchYawrateThrust.msg`
- `/$(env ROBOT_NAME)/interface/cmd_torque_thrust` of type `mav_msgs/TorqueThrust.msg`
- `/$(env ROBOT_NAME)/interface/cmd_velocity` of type `geometry_msgs/TwistStamped.msg`
- `/$(env ROBOT_NAME)/interface/cmd_position` of type `geometry_msgs/PoseStamped.msg`

All messages are in the robot's body frame, except `velocity` and `position` which use the frame specified by the message header.

## MAVROSInterface

The available implementation in AirStack is called `MAVROSInterface` implemented in `mavros_interface.cpp`. It simply forwards the control commands to the Ascent flight controller (based on Ardupilot) using MAVROS.

## Custom Robot Interface

If you're using a different robot control unit with its own custom API, then you need to create an associated RobotInterface. Implementations should do the following:

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

==TODO: our code doesn't currently do it like this, it instead uses an external odometry_conversion node.==

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
