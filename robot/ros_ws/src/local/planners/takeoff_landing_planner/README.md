# Takeoff Landing Planner

## Overview

The TakeoffLandingPlanner provides a rich interface for handling takeoff/landing commands, with support for both trajectory-based and ArduPilot command-based modes. It monitors the robot's position, generates appropriate trajectories, and tracks the completion status of takeoff and landing maneuvers.

## Highlights

- Two takeoff modes: standard and high altitude
- Configurable takeoff and landing velocities
- Configurable takeoff trajectory direction
- Trajectory-based takeoff and landing with position tracking
- Direct ArduPilot takeoff command support
- Completion monitoring with distance and time thresholds
- State publishing for integration with higher-level systems

## Topics

### Subscriptions

| Topic                              | Type                     | Description                                                                                                                                      |
| ---------------------------------- | ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `trajectory_completion_percentage` | `std_msgs/Float32`       | Current completion percentage of trajectory (from [trajectory_controller](../../c_controls/trajectory_controller/src/trajectory_controller.cpp)) |
| `tracking_point`                   | `airstack_msgs/Odometry` | Current tracking point for trajectory (from [trajectory_controller](../../c_controls/trajectory_controller/src/trajectory_controller.cpp))       |
| `odometry`                         | `nav_msgs/Odometry`      | Robot odometry data                                                                                                                              |
| `ekf_active`                       | `std_msgs/Bool`          | EKF status flag                                                                                                                                  |
| `high_takeoff`                     | `std_msgs/Bool`          | Flag to trigger high altitude takeoff                                                                                                            |

### Publications

| Topic                 | Type                              | Description                                              |
| --------------------- | --------------------------------- | -------------------------------------------------------- |
| `trajectory_override` | `airstack_msgs/TrajectoryXYZVYaw` | Generated takeoff/landing trajectory                     |
| `takeoff_state`       | `std_msgs/String`                 | Current takeoff state ("NONE", "TAKING_OFF", "COMPLETE") |
| `landing_state`       | `std_msgs/String`                 | Current landing state ("NONE", "LANDING", "COMPLETE")    |

## Services

| Service                       | Type                                  | Description                               |
| ----------------------------- | ------------------------------------- | ----------------------------------------- |
| `set_takeoff_landing_command` | `airstack_msgs/TakeoffLandingCommand` | Sets command mode (NONE, TAKEOFF, LAND)   |
| `ardupilot_takeoff`           | `std_srvs/Trigger`                    | Triggers direct ArduPilot takeoff command |

## Parameters

| Parameter                              | Type  | Default | Description                                             |
| -------------------------------------- | ----- | ------- | ------------------------------------------------------- |
| `takeoff_height`                       | float | 0.5     | Standard takeoff height in meters                       |
| `high_takeoff_height`                  | float | 1.2     | High altitude takeoff height in meters                  |
| `takeoff_velocity`                     | float | 0.3     | Velocity during takeoff in m/s                          |
| `landing_velocity`                     | float | 0.3     | Velocity during landing in m/s                          |
| `takeoff_acceptance_distance`          | float | 0.1     | Maximum distance to consider takeoff complete in meters |
| `takeoff_acceptance_time`              | float | 2.0     | Required time at acceptance distance in seconds         |
| `landing_stationary_distance`          | float | 0.02    | Maximum movement distance to consider landed in meters  |
| `landing_acceptance_time`              | float | 5.0     | Required time at stationary distance in seconds         |
| `landing_tracking_point_ahead_time`    | float | 5.0     | How far ahead the tracking point should be              |
| `takeoff_path_roll`                    | float | 0.0     | Roll angle for takeoff path in degrees                  |
| `takeoff_path_pitch`                   | float | 0.0     | Pitch angle for takeoff path in degrees                 |
| `takeoff_path_relative_to_orientation` | bool  | false   | Whether to use robot's current orientation for takeoff  |

## Usage

### Basic Usage

To launch the TakeoffLandingPlanner node:

```bash
ros2 run takeoff_landing_planner takeoff_landing_planner
```

### Initiating Takeoff

To command a takeoff:

```bash
ros2 service call /set_takeoff_landing_command airstack_msgs/srv/TakeoffLandingCommand "{command: 1}"
```

### Initiating Landing

To command a landing:

```bash
ros2 service call /set_takeoff_landing_command airstack_msgs/srv/TakeoffLandingCommand "{command: 2}"
```

### ArduPilot Takeoff

To use direct ArduPilot takeoff commands:

```bash
ros2 service call /ardupilot_takeoff std_srvs/srv/Trigger "{}"
```

## State Monitoring

The node publishes the current state of takeoff and landing operations through the `takeoff_state` and `landing_state` topics. Monitor these topics to track the progress and completion of operations:

```bash
ros2 topic echo /takeoff_state
ros2 topic echo /landing_state
```

## Dependencies

- airstack_msgs
- nav_msgs
- mavros_msgs
- std_msgs
- std_srvs
- tf2_ros

## Notes

- The node uses TF2 to track relative positions between the robot and tracking points
- Landing completion detection relies on both position stability and tracking point metrics
- The default landing target is hardcoded (-10000.0) and should be parameterized in future versions
