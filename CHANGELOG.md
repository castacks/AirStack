# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `TARGET_ARCH` build arg (default `x86_64`) in `Dockerfile.robot` to arch-parametrize `LD_LIBRARY_PATH`; `docker-compose.yaml` passes `TARGET_ARCH: aarch64` to the `voxl` and `l4t` real-robot image builds
- `ros-${ROS_DISTRO}-mavros-extras` in the robot image (provides the vision_pose plugin used for external-pose deployments)
- `overrides/l4t-px4-realrobot.env` — site-agnostic deployment override for a single real PX4 robot on a Jetson (aarch64/l4t)
- `integration` test tier (`tests/integration/`, `integration` mark) with a shared `robot_autonomy_stack` fixture (robot container, no sim/GPU)

### Changed

- `robot-l4t` compose service knobs are now env-overridable (`AUTONOMY_ROLE`, `FCU_URL`, and the rosbag path via `BAG_STORAGE_PATH`); `FCU_URL` unquoted so the literal serial path reaches MAVROS
- `zed-l4t` image: ZED SDK 4.2 → 5.2 with the coupled ROS deps (`zed_msgs` 5.2.1, `point_cloud_transport(_plugins)` 4.x, add `backward_ros`)
- Unit tests are defined by `tests/colcon_unit_test_packages.yaml`: `conftest.py` collects each listed package's co-located `test/` dir under `--import-mode=importlib` and marks it `unit` (ament lint files are skipped and run under `colcon test`)

### Fixed

- Robot name resolution now honors a pre-set `ROBOT_NAME` (e.g. injected via docker compose) instead of always overriding it from the container/hostname mapping (`robot/docker/.bashrc`)
- Robot name-map catch-all fallback now maps to `unknown_robot` (valid ROS namespace token) instead of `unknown-robot` (`default_robot_name_map.yaml`)
- l4t robot image: replace dustynv's `/ros_entrypoint.sh` with a passthrough so its prebuilt source-ROS libs (older `fastcdr`) no longer shadow the apt Jazzy runtime and crash apt-built nodes like MAVROS
- `RECORD_BAGS=true` never brought the bag recorder up on a robot: `logging.launch.xml` hardcoded `record_bag=false` and `onboard_autonomy_all.launch.xml` includes it with no arguments, so the variable was forwarded into the container and read by nobody (only `gcs.launch.xml` consumed it). With no `bag_record` node running, the GCS control panel's `set_recording_status` toggle had nothing to reach despite being bridged in `domain_bridge.yaml` / `dds_router.yaml`. It now reads `RECORD_BAGS` and selects its topic set via `LOG_CONFIG`
- Falling back to `unknown_robot` / domain 0 now logs a warning naming both fixes (rename the device `robot-<n>` on the host, or supply a `ROBOT_NAME_MAP_CONFIG_FILE` matching your hostnames). The fallback itself is unchanged — it deliberately keeps an unidentified robot out of every real robot's namespace — but it used to resolve silently, so the symptoms surfaced far from the cause
- Dropped `ROBOT_NAME` / `ROS_DOMAIN_ID` from `overrides/l4t-px4-realrobot.env`: no compose service declares either, so an env file could never set them and the lines were inert
- `bag_record/bag_recording_status` was bridged GCS -> robot in `domain_bridge.yaml`, the same direction as the command it answers, so recorder status never reached the GCS and every recording indicator stayed blank

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
