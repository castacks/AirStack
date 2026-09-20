# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `svg_ground_control`: `swarm_commander` draws a world-aligned ground grid clipped to the geofence footprint (`fence_grid_cell_m`) in `/svg/viz/markers`; the Foxglove layout's fixed built-in grid is turned off
- `svg_ground_control`: `swarm_commander` publishes a JSON status snapshot on `/svg/commander_status` (mission state, last lifecycle command outcome, live CBF gains, per-drone flight state / world position / odometry freshness / last robot_command result) and accepts `cbf_alpha` changes at runtime
- SVG Basestation Foxglove panel: mission chip and command log confirmed against the commander snapshot (not just the service reply), CBF alpha slider with live readout via `get/set_parameters`, an Agent State table with numeric per-drone positions and velocity-command stream rate, a `view` setting (main / power) with a two-instance layout, and goals in the commander's own frame (offsets adopted from the status snapshot)

### Changed

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
