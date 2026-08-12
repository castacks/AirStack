# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `overrides/l4t-optitrack-realrobot.env` — deployment override for a real Jetson robot flying on OptiTrack mocap (PX4 EKF2 external vision instead of GPS): the NatNet server/body settings, plus the multi-NIC and FCU-parameter notes that path needs
- Feature notebook workflow (`use-feature-notebook` skill): every agent-implemented feature gets a local, gitignored `notebook/NNN-feature-slug/` entry with a status-tracked `design_spec.md` (written before coding) and `results/` artifacts + self-contained `results_summary.md` that populate the feature's PR description
- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `TARGET_ARCH` build arg (default `x86_64`) in `Dockerfile.robot` to arch-parametrize `LD_LIBRARY_PATH`; `docker-compose.yaml` passes `TARGET_ARCH: aarch64` to the `voxl` and `l4t` real-robot image builds
- `ros-${ROS_DISTRO}-mavros-extras` in the robot image (provides the vision_pose plugin used for external-pose deployments)
- `overrides/l4t-px4-realrobot.env` — site-agnostic deployment override for a single real PX4 robot on a Jetson (aarch64/l4t)
- `integration` test tier (`tests/integration/`, `integration` mark) with a shared `robot_autonomy_stack` fixture (robot container, no sim/GPU)
- `waypoint_flight` system test (`tests/system/test_waypoint_flight.py`): takeoff → ordered waypoint route via `NavigateTask` (dispatched as a dense plan) → land, judged on the odometry track by the standalone stdlib-only `tests/waypoint_checker.py` (in-order corridor arrival within `--waypoint-tolerance`, final goal within `--goal-tolerance`, per-waypoint `--waypoint-timeout`); validated end-to-end in Isaac Sim; serves as the standard acceptance check after integrating or swapping a planner module
- Real-robot PX4 external-vision fusion in `natnet_ros2` (OptiTrack mocap → EKF2): `mavros_gp_origin` (geoid-corrected synthetic GPS origin so `local_position.z` == OptiTrack z, fixing the ~36 m boot offset), `vision_pose_converter`, and a PX4 param **checker** (`px4_param_setter`, `auto_set` off by default; `on_mismatch` warn/halt) — setup guide at `docs/robot/px4_external_vision.md`
- NatNet server emulator (`optitrack.natnet.emulator`, protocol core) — pure-Python OptiTrack Motive server emulation so `natnet_ros2` can be driven without hardware; host integration tests (`tests/integration/natnet/`) wire it to the robot client
- Isaac wrapper for the NatNet emulator (USD scene → server) + natnet Pegasus launch scripts, and a dedicated OptiTrack sim e2e test (`optitrack` mark, `tests/system/test_optitrack_e2e.py`) that flies a **Circle trajectory on mocap EKF2 fusion** — GPS, baro and range aiding are disabled for the run, so the OptiTrack stream is the vehicle's only position source and cross-track error scores the whole chain

### Changed

- Ephemeral CI GPU runners spawn via NVIDIA OSMO (not OpenStack); `system-tests.yml` / `docker-build.yml` still use `airstack-ephemeral`
- Default system-test `--sim` is `isaacsim`; pass `--sim msairsim` to opt in to Microsoft AirSim
- `-m build_packages` CI runs pull `cache_*` images instead of baking sim images
- `docker-build.yml` retags unchanged images on VERSION bumps (content fingerprint) instead of always rebuilding; floating `cache_*` tags still seed PR layer cache
- Opening a PR now runs one enforced end-to-end flight — the Circle trajectory on OptiTrack/mocap EKF2 fusion — instead of pytest's full defaults (every mark, all four trajectory types). Every other suite is unchanged and still available on demand via a `/pytest <args>` comment, `workflow_dispatch` inputs, or local `airstack test -m <marks>`
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
- `natnet_config.yaml`'s `$(env NATNET_SERVER_IP ...)` could never resolve: no compose service declared the variable, so the NatNet client always fell back to its hardcoded default and could reach neither the in-sim emulator nor a real Motive host. It is now forwarded in `robot-base-docker-compose.yaml`, defaulting to the in-sim emulator
- The NatNet rigid body tracked by `robot_1` defaulted to a site-specific body (id 1146) that no emulator streams; since the client filters frames by numeric id, that produced a connected client that never published. It now defaults to the emulator's body (`Drone`, id 1). Per-robot bodies are configured in each robot's profile in `natnet_config.yaml`, selected by `ROBOT_NAME`
- OptiTrack external-vision tuning corrected from real-flight bags: `EKF2_EV_DELAY` 8.0 → 7.0 and `EKF2_EVP_NOISE` 0.01 → 0.05. The old 0.01 gave a 5 cm innovation gate (`EKF2_EVP_GATE` × 5σ) that rejected valid mocap updates and blocked arming; `px4_params.yaml` now records the supporting measurements and the drift-and-snap misdiagnosis so neither is repeated
- The synthetic GPS origin now places the mocap floor at the shared world datum (`desired_floor_amsl: 36.0`, i.e. 90 m ellipsoidal in AMSL) rather than at sea level, so a mocap robot's reported global altitude agrees with sim and the GCS. `local_position.z` still equals the OptiTrack height either way
- The robot image could ship without the GeographicLib `egm96-5` geoid: mavros' `install_geographiclib_datasets.sh` swallows a failed download and still exits 0, so the `RUN` layer succeeded either way, and `geographiclib-tools` was only ever a transitive dependency. MAVROS builds that geoid in its UAS core before any plugin loads and throws if it is missing, so `mavros_node` died at startup on affected images. `Dockerfile.robot` now pins the tool and asserts the file exists, failing the build instead
- An unrecognised `connection_type` in `natnet_config.yaml` silently fell back to `unicast`, so a typo produced a client that connected on the wrong transport and never received frames. `validate_connection_type` now throws and `natnet_ros2_node` fails at startup naming the offending value
- `natnet_ros2_node` on `robot_1` now compares the NatNet server's MODELDEF drone-body count (`Drone` / `Drone1`…`DroneN`, excluding `Target` and skeleton bones) against `NUM_ROBOTS` after the handshake and logs an error on mismatch, so a sim launch script and `natnet_config.yaml` that disagree about how many drones exist is caught at startup rather than as a robot that silently never receives frames. `NUM_ROBOTS` is forwarded into the robot container for it
- Isaac Sim PX4 never fused the mocap stream: `EKF2_EV_CTRL` defaults to 0 and the isaac compose set no PX4 parameters, so the emulator could stream perfectly while PX4 flew on sim GPS. The compose now passes the EKF2 external-vision set as `PX4_PARAM_*` (applied by PX4 SITL's `rcS` at boot), each defaulting to PX4's own default so non-mocap sims are unaffected; the mocap path opts in
- The NatNet emulator hardcoded the drone's streaming id to 1 while the client reads `NATNET_BODY_ID`, so a real Motive id desynced the two into a connected client that never published (`example_one_px4_pegasus_natnet_launch_script.py`)
- `test_optitrack_e2e.py::test_px4_fuses_vision` asserted only that `local_position/pose` publishes, which it does off GPS — the check passed with external vision disabled. It is now the pre-flight gate (an estimate exists) and the Circle flight is the actual proof of fusion
- Isaac Sim now searches the repo's own `simulation/isaac-sim/extensions` via a second `--ext-folder`, so `optitrack.natnet.emulator` resolves as a registered Kit extension (previously only the Kit shared exts dir was searched, where the emulator is never installed)

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
