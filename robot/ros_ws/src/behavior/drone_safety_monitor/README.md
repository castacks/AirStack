# Drone safety monitor

Watches source-stamped odometry and requests trajectory PAUSE on invalid/stale state. Operator commands are `pause`, `resume` and `rewind`. A positive controller reply confirms request acceptance; independent motion evidence is required to establish a physical stop.

## Health contract

Odometry stamps must use the node's ROS clock. Source age and age since accepted receipt must each stay within `state_estimate_timeout`. Stamps must be positive, no more than 0.1 ROS seconds in the future, and not move backwards within an epoch. Position, orientation, linear velocity and angular velocity must be finite; quaternion squared norm must lie in [0.98, 1.02]. Covariance and localization accuracy are not qualified by these checks.

Repeated equal stamps do not refresh the accepted-receipt deadline. A backwards ROS clock jump invalidates previous state and requires valid data in the new epoch. Before the ROS clock is initialized the monitor remains unhealthy. A separate steady-clock receipt deadline detects silence/replay even when simulation time stops.

Health publication and RPC deadlines use wall/steady time. A recovered state never automatically resumes motion. The existing `state_estimate_timed_out` flag remains true while either state is unhealthy **or a requested PAUSE lacks a positive acknowledgement**, so existing task consumers do not confuse fresh state with intervention readiness.

## Intervention contract

Startup requests PAUSE. A new state fault or operator PAUSE latches a pending intervention. Only one mode RPC is outstanding at once. Rejection, unavailable service or response timeout leaves PAUSE pending and triggers bounded retries. Expired client requests are removed; late replies cannot acknowledge a newer pending request. This does not cancel a request already executing remotely.

`resume` and `rewind` require healthy state and an acknowledged PAUSE with no pending mode request. They are sent once, never retried automatically. The controller's own mode interfaces remain separate command authorities; this monitor is not an exclusive arbiter for every controller writer. No forced disarm or autonomous LAND escalation is provided.

## Parameters

All periods must be finite and positive. Response timeout must not exceed retry period; health publication period must not exceed one second.

| Parameter | Default | Clock / purpose |
| --- | --- | --- |
| `state_estimate_timeout` | 1.0 s | ROS time: source and accepted-receipt age |
| `receipt_timeout_wall_s` | 2.0 s | Steady time: last accepted advancing source stamp |
| `health_publish_period_s` | 0.25 s | Wall timer: health evaluation/publication |
| `pause_response_timeout_wall_s` | 0.75 s | Steady time: pending RPC deadline |
| `pause_retry_period_wall_s` | 1.0 s | Steady time: minimum interval between PAUSE attempts |

The wall deadline is deliberately separate from the ROS-time threshold. Changing simulation speed does not silently redefine either clock domain.

## Endpoints

Names below are relative to the node namespace. The canonical launch file remaps odometry and the mode service through the selected stack.

| Endpoint | Type | Purpose |
| --- | --- | --- |
| `state_estimate` | `nav_msgs/Odometry` subscription | Source-stamped state |
| `command` | `std_msgs/String` subscription | Operator pause/resume/rewind |
| `set_trajectory_mode` | `airstack_msgs/TrajectoryMode` client | PAUSE or explicit operator mode |
| `state_estimate_timed_out` | `std_msgs/Bool` publisher | State fault or unacknowledged intervention |
| `pause_acknowledged` | `std_msgs/Bool` publisher | Latest requested PAUSE accepted; not physical stopped state |
| `state_health_reason` | `std_msgs/String` publisher | Health reason, including `pause_pending` |

## Verification

The package registers `isolated_watchdog_contract` with CTest. Run the package tests inside the robot ROS overlay:

```bash
colcon test --packages-select drone_safety_monitor --event-handlers console_direct+
colcon test-result --test-result-base build/drone_safety_monitor --verbose
```

The test starts the actual installed C++ binary on isolated DDS domain 197, using a controlled ROS clock and mock controller. It covers source/clock faults, resumed receipt, service rejection, withheld/late replies, availability recovery, bounded retries, continuing heartbeat and operator interlocks. Output defaults to a new temporary directory; `WATCHDOG_TEST_OUTPUT` selects a new persistent directory. It never commands the simulator or an aircraft. Physical regression is separate and source-specific.
