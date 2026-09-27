# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `svg_ground_control`: `swarm_commander` draws a world-aligned ground grid clipped to the geofence footprint (`fence_grid_cell_m`) in `/svg/viz/markers`; the Foxglove layout's fixed built-in grid is turned off
- `svg_ground_control`: `swarm_commander` publishes a JSON status snapshot on `/svg/commander_status` (mission state, last lifecycle command outcome, live CBF gains, per-drone flight state / world position / odometry freshness / DDS `message_lost` reception counters / last robot_command result) and accepts `cbf_alpha` changes at runtime
- SVG Basestation Foxglove panel: Drop column is now measured from the commander's DDS loss counters (tagged `dds`), with the arrival-timing estimate (`floor`, tagged `est`) only as a fallback
- SVG Basestation Foxglove panel: mission chip and command log confirmed against the commander snapshot (not just the service reply), CBF alpha slider with live readout via `get/set_parameters`, an Agent State table with numeric per-drone positions and velocity-command stream rate, a `view` setting (main / power) with a two-instance layout, and goals in the commander's own frame (offsets adopted from the status snapshot)
- `svg_ground_control`: teleop fence — `teleop_fence_enabled` / `teleop_fence_min` / `teleop_fence_max`, a smaller keep_in box inside the geofence that bounds hand-flown drones whatever `fence_behavior` is (amber box in `/svg/viz/markers`, both boxes in the status snapshot)
- `svg_ground_control`: `teleop_accel_mps2` (live) — the teleop stick velocity is ramped at that acceleration and the ramp is fed forward to PX4 on the trajectory output, like PX4's own Position mode (`MPC_ACC_HOR_MAX`); a bare stick step was followed at ~4 m/s² only

### Fixed

- `svg_ground_control`: the teleop yaw stick did nothing on the trajectory output — the setpoint's rotation was left at the `Quaternion` default (w = 1), which px4_interface read as "hold ENU yaw 0" and so dropped the yaw rate. Teleop now sends an all-zero rotation with the yaw rate while the stick is deflected and holds the measured heading as an absolute yaw when it is centred
- `svg_ground_control`: the altitude walked down during pure x-y teleop and released sticks sometimes kept the drone coasting (bag `run_060352`) — the reference leash pulled the 3-D lead vector (scaling its z part with the horizontal lag) and handed the drone's *measured* velocity to the stick ramp as its restart point, so sink and overrun became the command. The leash now limits the horizontal and vertical lead separately (`position_hold.leash`) and the ramp re-attaches only to what was last published
- `svg_ground_control`: a keep_in fence floor above the ground (e.g. `teleop_fence_min` z = 0.3) stopped the drone from landing — the reference point was clamped into the box in every flight state, so PX4 held the clamped position setpoint against the descent. The reference is now clamped only while ACTIVE, like the velocity clip

### Changed

- `svg_ground_control`: `squeeze_rc_intruder.yaml` — the intruder (drone_3) is now a gamepad-flown teleop drone the commander arms, lifts to intruder waypoint A, hands to the sticks and lands, CBF-exempt so the holders alone yield, with its own teleop fence; the RC-link external variant is a two-line switch noted in the header
- `svg_ground_control`: the `keep_in` fence wall is a braking envelope (`fence_brake_accel_mps2`, new; `fence_keep_in_gain` is now the near-wall gain and `1/gain` the lag margin) with the deceleration sent to PX4 as the acceleration feedforward on the trajectory output; the old `gain × distance` cap overshot the wall by ~0.5 m at 6 m/s (bag `run_045417`). `fence_keep_in_gain`, `fence_brake_accel_mps2` and `fence_margin_m` are live parameters

- The SVG Basestation panel, its `svg_basestation.json` layout and installer moved from `gcs/foxglove_extensions/` to `robot/ros_ws/src/svg_ground_control/foxglove/`; the robot-desktop container runs that installer at start-up alongside the general one

## [1.0.0] - 2024-12-19

First official public release.

### Added

- Docker image robot-l4t for Jetson AGX
- Automatically load and play Isaac Sim scene upon launch
- Random walk planner
- DROAN trajectory-library based local planner
- Initial GCS rviz capable of visualizing multiple robots

### Fixed

- A bunch of stuff honestly

### Changed

- Upgrade Isaac Sim from 4.1.0 to 4.2.0
- Unified docker image naming to use AirStack's version.
- Condensed GCS TAK docker images to single docker image

### Removed

- Duplicate TAK images
