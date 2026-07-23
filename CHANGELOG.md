# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Feature notebook workflow (`use-feature-notebook` skill): every agent-implemented feature gets a local, gitignored `notebook/NNN-feature-slug/` entry with a status-tracked `design_spec.md` (written before coding) and `results/` artifacts + self-contained `results_summary.md` that populate the feature's PR description
- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `TARGET_ARCH` build arg (default `x86_64`) in `Dockerfile.robot` to arch-parametrize `LD_LIBRARY_PATH`; `docker-compose.yaml` passes `TARGET_ARCH: aarch64` to the `voxl` and `l4t` real-robot image builds
- `ros-${ROS_DISTRO}-mavros-extras` in the robot image (provides the vision_pose plugin used for external-pose deployments)
- `overrides/l4t-px4-realrobot.env` — site-agnostic deployment override for a single real PX4 robot on a Jetson (aarch64/l4t)
- `integration` test tier (`tests/integration/`, `integration` mark) with a shared `robot_autonomy_stack` fixture (robot container, no sim/GPU)
- `waypoint_flight` system test (`tests/system/test_waypoint_flight.py`): takeoff → ordered waypoint route via `NavigateTask` (dispatched as a dense plan) → land, judged on the odometry track by the standalone stdlib-only `tests/waypoint_checker.py` (in-order corridor arrival within `--waypoint-tolerance`, final goal within `--goal-tolerance`, per-waypoint `--waypoint-timeout`); validated end-to-end in Isaac Sim; serves as the standard acceptance check after integrating or swapping a planner module
- Real-robot PX4 external-vision fusion in `natnet_ros2` (OptiTrack mocap → EKF2): `mavros_gp_origin` (geoid-corrected synthetic GPS origin so `local_position.z` == OptiTrack z, fixing the ~36 m boot offset), `vision_pose_converter`, and a PX4 param **checker** (`px4_param_setter`, `auto_set` off by default; `on_mismatch` warn/halt) — setup guide at `docs/robot/px4_external_vision.md`

### Changed

- Ephemeral CI GPU runners spawn via NVIDIA OSMO (not OpenStack); `system-tests.yml` / `docker-build.yml` still use `airstack-ephemeral`
- Default system-test `--sim` is `isaacsim`; pass `--sim msairsim` to opt in to Microsoft AirSim
- `-m build_packages` CI runs pull `cache_*` images instead of baking sim images
- `docker-build.yml` retags unchanged images on VERSION bumps (content fingerprint) instead of always rebuilding; floating `cache_*` tags still seed PR layer cache
- `robot-l4t` compose service knobs are now env-overridable (`AUTONOMY_ROLE`, `FCU_URL`, and the rosbag path via `BAG_STORAGE_PATH`); `FCU_URL` unquoted so the literal serial path reaches MAVROS
- `zed-l4t` image: ZED SDK 4.2 → 5.2 with the coupled ROS deps (`zed_msgs` 5.2.1, `point_cloud_transport(_plugins)` 4.x, add `backward_ros`)
- Unit tests are defined by `tests/colcon_unit_test_packages.yaml`: `conftest.py` collects each listed package's co-located `test/` dir under `--import-mode=importlib` and marks it `unit` (ament lint files are skipped and run under `colcon test`)

### Fixed

- Isaac Sim image: PX4 `ubuntu.sh` no longer fails dpkg configure on the NVIDIA base (`ca-certificates` / `software-properties-common`); use `--no-nuttx --no-sim-tools` like ms-airsim
- Robot image: pin `pytest<8.1` and disable `launch_testing` for colcon unit tests so ROS Jazzy's outdated pytest hook does not abort `colcon test`
- Robot name resolution now honors a pre-set `ROBOT_NAME` (e.g. injected via docker compose) instead of always overriding it from the container/hostname mapping (`robot/docker/.bashrc`)
- Robot name-map catch-all fallback now maps to `unknown_robot` (valid ROS namespace token) instead of `unknown-robot` (`default_robot_name_map.yaml`)
- l4t robot image: replace dustynv's `/ros_entrypoint.sh` with a passthrough so its prebuilt source-ROS libs (older `fastcdr`) no longer shadow the apt Jazzy runtime and crash apt-built nodes like MAVROS
- `RECORD_BAGS=true` never brought the bag recorder up on a robot: `logging.launch.xml` hardcoded `record_bag=false` and `onboard_autonomy_all.launch.xml` includes it with no arguments, so the variable was forwarded into the container and read by nobody (only `gcs.launch.xml` consumed it). With no `bag_record` node running, the GCS control panel's `set_recording_status` toggle had nothing to reach despite being bridged in `domain_bridge.yaml` / `dds_router.yaml`. It now reads `RECORD_BAGS` and selects its topic set via `LOG_CONFIG`
- Falling back to `unknown_robot` / domain 0 now logs a warning naming both fixes (rename the device `robot-<n>` on the host, or supply a `ROBOT_NAME_MAP_CONFIG_FILE` matching your hostnames). The fallback itself is unchanged — it deliberately keeps an unidentified robot out of every real robot's namespace — but it used to resolve silently, so the symptoms surfaced far from the cause
- Dropped `ROBOT_NAME` / `ROS_DOMAIN_ID` from `overrides/l4t-px4-realrobot.env`: no compose service declares either, so an env file could never set them and the lines were inert
- `bag_record/bag_recording_status` was bridged GCS -> robot in `domain_bridge.yaml`, the same direction as the command it answers, so recorder status never reached the GCS and every recording indicator stayed blank
- `bag_record_node` passed `--exclude` to `ros2 bag record`, which Jazzy renamed to `--exclude-regex`. It is now an ambiguous prefix of four options, so argparse rejected the command and any section using `exclude:` (including `log.yaml`'s `airstack` section, i.e. everything but the cameras) recorded nothing — surfacing only as a usage dump in the node's stdout. Multiple `exclude:` entries are now alternated into one regex instead of repeating a single-valued flag, which had silently kept only the last

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
