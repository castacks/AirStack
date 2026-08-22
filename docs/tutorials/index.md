# Tutorials

Step-by-step guides for common AirStack workflows. If you are new, start with **Getting Started**.

| Tutorial | Description |
|---|---|
| [Getting Started](../getting_started/index.md) | Install AirStack, pull Docker images, launch a simulated robot, and fly it for the first time. |
| [AirStack on OSMO (Mac/Windows OK)](airstack_on_osmo.md) | Develop on AirStack from a Mac, Windows, or no-GPU Linux laptop using NVIDIA OSMO + VS Code/Cursor Remote-SSH. No local Docker or local `airstack install`; use a local repo clone for the `airstack osmo:*` wrappers and workflow YAML. |
| [Multi-Robot Simulation](../robot/docker/robot_identity.md) | Spin up multiple simulated robots in Isaac Sim and verify independent ROS 2 namespaces. |
| [Autonomy Modes](../robot/autonomy_modes.md) | Understand the `full_default`, `lite_default`, and `lite_offload_global` stack topologies and the commands to run each. |
| [Deploying to Hardware](../real_world/deploying_to_hardware.md) | Flash a Jetson or VOXL device, configure the robot hostname, and run the autonomy stack on a real drone. |
