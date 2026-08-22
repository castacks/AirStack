# Local Packages
The local module includes packages that are specific to the local autonomy of the robot. This includes local mapping, planning, and control.

## Launch
Local modules ship their own canonical launch files and are composed flat by
the stack entry file: `takeoff_landing_planner`, the trajectory
controller, `droan_gl`, and the PID controller are included directly — see
`stacks/full_default/launch/stack.launch.xml` for the composed wiring.

