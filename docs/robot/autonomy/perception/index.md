# Perception Packages

The perception module is responsible for state estimation and environment understanding. It processes sensor data to provide the robot's best estimate of its pose, velocity, and surrounding environment.

## Overview

Perception forms the foundation of the autonomy stack by:

- **State Estimation**: Fusing sensor data (IMU, cameras, GPS) to estimate robot position, orientation, and velocity
- **Sensor Processing**: Converting raw sensor data into usable formats for downstream modules
- **Environment Understanding**: Detecting and tracking objects, obstacles, and features in the environment

## Launch

Launch files are located under `robot/ros_ws/src/perception/perception_bringup/launch/`.

The main launch command is:
```bash
ros2 launch perception_bringup perception.launch.xml
```

## Key Topics

### Outputs
- `/{robot_name}/odometry` - Best estimate of robot state (position, orientation, velocities)
- `/{robot_name}/pose` - Current robot pose
- `/{robot_name}/imu/data` - Processed IMU data

### Inputs
- Raw sensor data from sensors layer (cameras, IMU, GPS, depth sensors)

## Modules

- [**State Estimation**](state_estimation.md) - Overview of state estimation approaches and implementations

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
