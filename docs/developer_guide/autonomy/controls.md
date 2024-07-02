# Controls
Code located in `AirLab-Autonomy-Stack/ros_ws/src/robot/autonomy/controls/`.


## Robot Interface class
The `RobotInterface` translates commands from the autonomy stack into the command for the underlying hardware.
Specific implementations should extend `class RobotInterface` in `robot_interface.hpp`

Commands: Attitude control and Position control.

These are reflected in both PX4 and Ardupilot. 
