# drone_safety_monitor

A ROS 2 package that monitors critical drone safety conditions and publishes fault flags to the rest of the autonomy stack. Currently the package watches the state estimate stream and raises a timeout flag whenever odometry stops arriving, allowing downstream nodes (e.g. the behavior executive) to react safely.

## Overview

| Item | Value |
|---|---|
| Package name | `drone_safety_monitor` |
| Node name | `odom_timeout_checker` |
| Language | C++ 17 |
| ROS distro | ROS 2 (Humble / Iron) |
| License | MIT |

## Nodes

### `drone_safety_monitor` (executable → node name `odom_timeout_checker`)

Monitors the incoming state estimate (odometry) and publishes a Boolean timeout flag once per second.

**How it works**

1. Every time an odometry message is received on `state_estimate`, the internal `TimeChecker` records the current ROS clock time.
2. A 1 Hz timer fires every second and computes how long ago the last odometry message arrived.
3. If that elapsed time exceeds `state_estimate_timeout`, the node publishes `true` on `state_estimate_timed_out`; otherwise it publishes `false`.
4. If no odometry has ever been received (e.g. immediately after startup), the elapsed time is treated as infinite, so the timeout flag is raised immediately.

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `state_estimate` | `nav_msgs/msg/Odometry` | Incoming odometry / state estimate from the localization system |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `state_estimate_timed_out` | `std_msgs/msg/Bool` | `true` when the state estimate has not been received within `state_estimate_timeout` seconds; `false` otherwise. Published at 1 Hz. |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `state_estimate_timeout` | `double` | `1.0` | Maximum acceptable gap (in seconds) between consecutive state estimate messages before the timeout flag is raised |

## Dependencies

| Package | Purpose |
|---|---|
| `rclcpp` | ROS 2 C++ client library |
| `std_msgs` | `Bool` message type for the timeout flag |
| `nav_msgs` | `Odometry` message type for the state estimate |
| `airstack_common` | AirStack helper utilities (`airstack::get_param`) |
| `airstack_msgs` | AirStack custom message types |
| `trajectory_library` | Trajectory utilities (linked at build time) |
| `mavros_msgs` | MAVLink/PX4 bridge message types |
| `std_srvs` | Standard service types |

## Building

```bash
# From the workspace root
colcon build --packages-select drone_safety_monitor
source install/setup.bash
```

## Running

```bash
ros2 run drone_safety_monitor drone_safety_monitor
```

Override the timeout at the command line:

```bash
ros2 run drone_safety_monitor drone_safety_monitor \
  --ros-args -p state_estimate_timeout:=0.5
```

## Configuration

The `config/takeoff_landing_planner.yaml` file is included in the package install. Parameters can be loaded at launch time via `--params-file` or overridden inline with `--ros-args -p <name>:=<value>`.

## Internal Classes

### `TimeChecker`

A lightweight helper that tracks the timestamp of the most recent event and computes the elapsed time since then.

| Method | Description |
|---|---|
| `TimeChecker()` | Constructs the checker in an uninitialized state |
| `update(rclcpp::Time)` | Records the provided timestamp as the most recent update time |
| `elapsed_since_last_update(rclcpp::Time)` | Returns elapsed seconds since the last `update()` call, or `+∞` if `update()` has never been called |

## Topic Graph

```
[localization / EKF]
        │
        │  nav_msgs/Odometry
        ▼
  state_estimate
        │
        ▼
 drone_safety_monitor
        │
        │  std_msgs/Bool
        ▼
 state_estimate_timed_out
        │
        ▼
 [behavior executive / safety arbiter]
```
