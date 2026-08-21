# Perception Packages

The perception module is responsible for state estimation and environment understanding. It processes sensor data to provide the robot's best estimate of its pose, velocity, and surrounding environment.

## Overview

Perception forms the foundation of the autonomy stack by:

- **State Estimation**: Fusing sensor data (IMU, cameras, GPS) to estimate robot position, orientation, and velocity
- **Sensor Processing**: Converting raw sensor data into usable formats for downstream modules
- **Environment Understanding**: Detecting and tracking objects, obstacles, and features in the environment

## Launch

Launch files are located under `robot/ros_ws/src/perception/perception_bringup/launch/`
(module launch files; the legacy `perception.launch.xml` layer wrapper was
removed with the AUTONOMY_ROLE dispatch). The stack entry files include them
directly:
```bash
ros2 launch perception_bringup stereo_image_proc.launch.xml
ros2 launch perception_bringup topic_keepalive.launch.xml
```

## Key Topics

### Outputs
- `/{robot_name}/odometry` - Best estimate of robot state (position, orientation, velocities)
- `/{robot_name}/pose` - Current robot pose
- `/{robot_name}/imu/data` - Processed IMU data

### Inputs
- Raw sensor data from sensors layer (cameras, IMU, GPS, depth sensors)

## Modules

State estimation and related perception packages live under `robot/ros_ws/src/perception/`. Only external motion capture is documented below today; other approaches (onboard sensor fusion, visual-inertial odometry, etc.) will be added here in future releases.

### External pose (motion capture)

- [**OptiTrack (asm_optitrack module)**](../../optitrack.md) — NatNet mocap support (rigid-body poses from a Motive PC, PX4 external-vision fusion bridges) moved out of the trunk into the [asm_optitrack module](https://github.com/castacks/asm_optitrack). Add it with `airstack module add https://github.com/castacks/asm_optitrack --version <tag>`; see [AirStack Modules](../../../development/modules.md).

## Configuration

Perception parameters are configured in `perception_bringup/config/` directory. Common parameters include:

- Sensor topics to subscribe to
- Fusion algorithm parameters
- Output frame IDs
- Publishing rates

## See Also

- [System Architecture](../system_architecture.md) - Overall autonomy stack architecture
- [Sensors](../sensors/index.md) - Sensor integration layer
- [Integration Checklist](../integration_checklist.md) - Adding new perception modules
