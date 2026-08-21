# Behavior
The behavior module is responsible for the high-level decision making of the robot. This includes deciding what actions to take based on the current state of the robot and the world around it. The behavior module is responsible for coordinating the actions of the local and global modules to achieve the robot's goals.

## Launch
Behavior modules ship their own canonical launch files and are composed by
the stack entry file (the legacy `behavior_bringup` package was removed with
the AUTONOMY_ROLE dispatch), e.g.
`ros2 launch drone_safety_monitor drone_safety_monitor.launch.xml` — see
`stacks/full_default/launch/stack.launch.xml` for the composed wiring.

