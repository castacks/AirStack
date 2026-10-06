# Takeoff Landing Planner

## Overview

The TakeoffLandingPlanner provides ROS 2 action servers for takeoff and landing maneuvers. It monitors the robot's state, generates trajectory overrides to reach the target altitude (takeoff) or ground (landing), and tracks completion via position and time thresholds.

## Features

- Trajectory-based takeoff to a configurable target altitude and velocity
- Trajectory-based landing with on-ground detection, followed by automatic disarm
- Configurable takeoff trajectory direction (roll/pitch, absolute or body-relative)
- Precondition checking: rejects goals when state estimate is timed out or a task is already running
- Publishes `is_airborne` for downstream consumers (e.g. the RViz Tasks Panel)

## Topics

### Observed takeoff authority

Takeoff service acceptance is not flight authority. Before selecting TRACK or
publishing ascent, the action requires both `is_armed` and `has_control` true,
received after the control request and within `control_state_max_age_s` (0.5s)
of steady time. Acquisition waits at most `control_acquisition_timeout_s` (2s)
after the bounded request call. Both timing settings must be finite and positive.
During ascent, false or stale authority fails the task through the hold/LAND
handover below. Failed/uncertain ARM, control requests and TRACK transitions also use
that containment path; cancellation during acquisition produces a canceled result.
An ARM request that was never sent fails without a handover. The acquisition
deadline excludes service/containment waits and is not a physical
stop-time guarantee. LAND acceptance still does not verify grounding.

These task checks do not configure MAVROS or validate thrust/flight performance.
`airstack ready` separately checks the effective child-node `thrust_scaling` is
double 1.0 for this PX4 profile, not merely present in source YAML, plus a verified
interface startup report. The [MAVROS interface](../../../interface/mavros_interface/README.md)
independently gates ARM/control/takeoff calls on a fresh readback and owns grounded
startup configuration. GUI launch is not itself admission to actuation. Invalid
startup configuration must be repaired before flight; readiness is not qualification.

### Takeoff-envelope abort handover

Horizontal-displacement, altitude-overshoot and upward-speed limit breaches remain
failed takeoff outcomes. They attempt ROBOT_POSE to clear the ascent trajectory,
then immediately request the existing interface LAND command, even if hold failed.
Service discovery and response waits are each bounded at2s (up to8s for two
serial services, plus scheduling); this is not a physical stop-time guarantee.
The result/log records `abort_hold`, `abort_land`, and `grounding=UNVERIFIED`.

LAND dispositions: `ACCEPTED` means the bool interface returned true,
`FAILED_OR_UNCONFIRMED` means false (which may hide an inner MAVROS timeout),
`UNCONFIRMED` means response timeout, `NOT_SENT` means unavailable service before
dispatch. Removing a timed-out future does not cancel its remote request.
Any sent LAND latches recovery into observation-only behavior: no TRACK mode or
landing trajectory override. Only NOT_SENT permits the ordinary landing path.
The latch survives recovery completion/cancel/timeout until the next takeoff.
Before ground confirmation, a latched recovery's cancel/timeout/shutdown sends no
trajectory cleanup commands. Autopilot LAND uses the autopilot's descent profile,
not the velocity requested in the trajectory-based LandTask goal.
LAND request acceptance is not mode-transition, descent or ground confirmation.

The recovery LandTask continues its bounded landed-state observation/disarm path;
the independent mission verifier still determines actual grounding/disarm. This
change does not fix stale telemetry, cancel/shutdown containment, active control
physics or all interface failures. No midair PID reset/disarm is added.

`isolated_abort_handover` CTest launches the actual action node with mocked services
and telemetry in empty ROS domain198; no MAVROS or vehicle exists there. It checks
all3 bounds, failed/delayed hold, failed/delayed LAND, unavailable service, ordinary
landing and no conflicting trajectory after a sent handover. These are logical
component tests, not flight qualification. Rebuild/relaunch is required to deploy.

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
| `control_acquisition_timeout_s`        | float | 2.0     | Post-request wait for fresh armed/control observations (steady seconds) |
| `control_state_max_age_s`              | float | 0.5     | Maximum steady receipt age of armed/control observations      |
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

### Authority diagnostics

The action node publishes `~/authority_diagnostic` (`std_msgs/String`) at 2 Hz.
JSON schema `takeoff-authority/v1` labels passive snapshots
`periodic_observation` and actual rejected terminal guard checks
`terminal_guard_failure`. A terminal snapshot is captured under the authority mutex
at the decision and included verbatim in the action result before hold/LAND
containment. Cancellation and shutdown do not produce invented guard failures.
Diagnostic publication errors are contained so they cannot block containment.

`reason_mask` uses bits 1/2 for armed/control false, 4/8 for armed/control receipt
not strictly after the required request bound, and 16/32 for armed/control age not
within `max_age_s`. Several bits may be set. Receipt ages and receipt steady times
are null when no receipt was seen. Steady times are process/host observations,
not ROS sensor stamps or transport latency. Passive grounded rejection is expected
while disarmed and does not indicate an action failure. The original conjunction,
strict request bound and inclusive freshness boundary are unchanged.

The RRM subscription-only control recorder captures this stream as
`authority_diagnostic`, separately from the interface `authority` boolean.
