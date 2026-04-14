# Takeoff Landing Planner

## Overview

The TakeoffLandingPlanner provides ROS 2 action servers for takeoff and landing maneuvers. It monitors the robot's state, generates trajectory overrides to reach the target altitude (takeoff) or ground (landing), and tracks completion via position and time thresholds.

## Features

- Trajectory-based takeoff to a configurable target altitude and velocity
- Trajectory-based landing with stationary-detection completion
- Configurable takeoff trajectory direction (roll/pitch, absolute or body-relative)
- Precondition checking: rejects goals when state estimate is timed out or a task is already running
- Publishes `is_airborne` for downstream consumers (e.g. the RViz Tasks Panel)

## Topics

### Subscriptions

| Topic                              | Type                              | Description                                          |
|------------------------------------|-----------------------------------|------------------------------------------------------|
| `odometry`                         | `nav_msgs/Odometry`               | Robot pose and velocity                              |
| `tracking_point`                   | `airstack_msgs/Odometry`          | Current trajectory tracking point                   |
| `trajectory_completion_percentage` | `std_msgs/Float32`                | Trajectory completion (0–100) from trajectory_controller |
| `is_armed`                         | `std_msgs/Bool`                   | Vehicle arm status                                   |
| `has_control`                      | `std_msgs/Bool`                   | Offboard control acquired                            |
| `state_estimate_timed_out`         | `std_msgs/Bool`                   | Whether the state estimate has timed out             |
| `extended_state`                   | `mavros_msgs/ExtendedState`       | MAVROS landed state (used to publish `is_airborne`)  |

### Publications

| Topic               | Type                              | Description                                             |
|---------------------|-----------------------------------|---------------------------------------------------------|
| `trajectory_override` | `airstack_msgs/TrajectoryXYZVYaw` | Takeoff/landing trajectory sent to trajectory_controller |
| `is_airborne`       | `std_msgs/Bool`                   | True when `landed_state == LANDED_STATE_IN_AIR`         |

## Service Clients

| Service               | Type                              | Description                            |
|-----------------------|-----------------------------------|----------------------------------------|
| `set_trajectory_mode` | `airstack_msgs/TrajectoryMode`    | Switch trajectory controller mode      |
| `robot_command`       | `airstack_msgs/RobotCommand`      | Arm, request offboard control, disarm  |

## Action Servers

| Action          | Type                      | Description                  |
|-----------------|---------------------------|------------------------------|
| `~/takeoff_task` | `task_msgs/TakeoffTask`  | Execute a takeoff maneuver   |
| `~/land_task`    | `task_msgs/LandTask`     | Execute a landing maneuver   |

### TakeoffTask

**Goal**

| Field               | Type    | Description                    |
|---------------------|---------|--------------------------------|
| `target_altitude_m` | float32 | Target altitude in meters      |
| `velocity_m_s`      | float32 | Ascent velocity in m/s         |

**Feedback**

| Field                | Type    | Description                    |
|----------------------|---------|--------------------------------|
| `status`             | string  | Human-readable status message  |
| `current_altitude_m` | float32 | Current altitude in meters     |
| `target_altitude_m`  | float32 | Target altitude in meters      |

**Result**

| Field     | Type    | Description              |
|-----------|---------|--------------------------|
| `success` | bool    | Whether takeoff succeeded |
| `message` | string  | Outcome description      |

### LandTask

**Goal**

| Field          | Type    | Description              |
|----------------|---------|--------------------------|
| `velocity_m_s` | float32 | Descent velocity in m/s  |

**Feedback**

| Field                | Type    | Description                   |
|----------------------|---------|-------------------------------|
| `status`             | string  | Human-readable status message |
| `current_altitude_m` | float32 | Current altitude in meters    |

**Result**

| Field     | Type   | Description               |
|-----------|--------|---------------------------|
| `success` | bool   | Whether landing succeeded |
| `message` | string | Outcome description       |

## Parameters

| Parameter                              | Type  | Default | Description                                                   |
|----------------------------------------|-------|---------|---------------------------------------------------------------|
| `takeoff_velocity`                     | float | 1.0     | Default ascent velocity in m/s                                |
| `landing_velocity`                     | float | 0.3     | Default descent velocity in m/s                               |
| `takeoff_acceptance_distance`          | float | 0.3     | Distance threshold to consider target altitude reached (m)    |
| `takeoff_acceptance_time`              | float | 1.0     | Time that must be spent within acceptance distance (s)        |
| `landing_stationary_distance`          | float | 0.02    | Max movement to consider the drone stationary on landing (m)  |
| `landing_acceptance_time`              | float | 5.0     | Time the drone must remain stationary to confirm landing (s)  |
| `landing_tracking_point_ahead_time`    | float | 5.0     | Lookahead time for the landing tracking point                 |
| `takeoff_path_roll`                    | float | 0.0     | Roll offset for the takeoff trajectory (degrees)              |
| `takeoff_path_pitch`                   | float | 0.0     | Pitch offset for the takeoff trajectory (degrees)             |
| `takeoff_path_relative_to_orientation` | bool  | false   | Apply roll/pitch relative to the robot's current orientation  |

## Dependencies

- `rclcpp` / `rclcpp_action`
- `airstack_msgs`
- `airstack_common`
- `nav_msgs`
- `mavros_msgs`
- `std_msgs`
- `task_msgs`
- `trajectory_library`
