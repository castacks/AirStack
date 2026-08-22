# Behavior
The behavior module is responsible for the high-level decision making of the robot. This includes deciding what actions to take based on the current state of the robot and the world around it. The behavior module is responsible for coordinating the actions of the local and global modules to achieve the robot's goals.

## Launch
Behavior modules ship their own canonical launch files and are composed by
the stack entry file, e.g.
`ros2 launch drone_safety_monitor drone_safety_monitor.launch.xml` — see
`stacks/full_default/launch/stack.launch.xml` for the composed wiring.

## Modules

- **`drone_safety_monitor`** — the safety executive: watches the state
  estimate for timeouts and issues safety commands. It runs onboard so the
  robot can failsafe even if every ground link is lost.

Task goals (takeoff, land, explore, navigate) are sent by the operator from
the GCS to the [task executors](../tasks.md) in the global and local layers;
see [System Architecture — Task Cascade](../system_architecture.md#task-cascade).

