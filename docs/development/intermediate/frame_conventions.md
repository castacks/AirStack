# Frame Conventions

AirStack uses the East-North-Up (ENU) coordinate convention for everything ROS-side: the robot's `map` frame is ENU with units in meters, angles in radians, right-handed, yaw about +Z.

The launch preamble ([`autonomy_bringup/launch/robot.launch.xml`](https://github.com/castacks/AirStack/blob/develop/robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml)) publishes a static identity transform `world → map`, so `world` is a fixed origin and `map` is the planning/state frame in which `odometry.pose`, `global_plan`, and the map live. The full frame table (including `base_link` and units) is specified in the [Interface Conventions Specification](../../robot/autonomy/interface_conventions.md#tf-frames-and-units) — that spec is canonical.

Two boundaries to watch:

- **PX4 / MAVROS**: PX4 works in NED; MAVROS performs the NED↔ENU conversion. Everything above MAVROS is ENU. Mixing these up is the classic silent failure — see the warning in the spec.
- **Isaac Sim**: Isaac Sim follows the Forward-Left-Up (FLU) convention ([Isaac Sim reference](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/reference_conventions.html)). The Pegasus-based launch scripts handle the sim-side conversion when spawning drones — see [Spawning Drones](../../simulation/isaac_sim/spawning_drones.md).
