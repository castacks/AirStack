# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

This release carries the **Modular AirStack campaign**
([RFC #379](https://github.com/castacks/AirStack/discussions/379) /
[RFC #380](https://github.com/castacks/AirStack/discussions/380)): the monolith
splits into **modules** (thin external repos pulled on demand), **stacks**
(self-contained topology folders under `stacks/`), and **fleets**
(`config/fleets/` — who exists, which vehicle, which stack, which ground hosts).

### New repositories

- [castacks/airstack-modules-index](https://github.com/castacks/airstack-modules-index) — the module/stack registry (one YAML per entry; DECLARED compat in the entry, VERIFIED compat CI-stamped under `compat/`)
- [castacks/asm_macvo](https://github.com/castacks/asm_macvo) — MAC-VO learned stereo visual odometry, extracted from trunk
- [castacks/asm_optitrack](https://github.com/castacks/asm_optitrack) — OptiTrack NatNet mocap integration (natnet_ros2 client, PX4 external-vision fusion, Motive-compatible NatNet server emulator for Isaac Sim), extracted from trunk
- [castacks/asm_dfm2_disturbances](https://github.com/castacks/asm_dfm2_disturbances) — Isaac Sim disturbance library (fan/vent force fields, strobe lights, lens flare)

### Added

- `airstack up --stack <name>[:<entry>]` stack dispatch with **5 reference stacks** under `stacks/` (`full_default`, `full_droan_cpu`, `full_macvo`, `lite_default`, `lite_offload_global`), each carrying pinned `modules.repos` and a CI-observed `wiring.md` graph baseline
- Module CLI — `airstack module add <url> --version <tag|sha>` (branches refused; local paths allowed), `list|sync|remove|create --in-tree|doctor`, workspace **overlay** of module packages, and auto-generated module compose overrides included by `airstack up`; Docker **module layers** composed via `modules.lock` (`airstack module lock --build`)
- Fleet system (RFC #380): `airstack up --fleet <name>` driven by `config/fleets/*.yaml` (identity, vehicle from `config/vehicles/`, stack selection, spawns) with `hosts:` split-stack placement onto ground hosts; `airstack fleet list|generate` per-robot compose for heterogeneous fleets; `airstack sync` reconciles `airstack.yaml` (modules, external stack repos, fleet validation)
- `airstack doctor [--live|--snapshot] [--stack NAME]` — observe-and-report checks with exactly **two hard gates** (module dependency-conflict gate; bridge gate: no control-setpoint / trajectory-group topics may cross a split-stack bridge); `--live` diffs the RUNNING ROS graph against the stack's committed `wiring.md`
- `airstack stack list|new <src> <dst>|diff <a> <b>` (diff compares generated wiring, not launch XML) — complementing `airstack ready` (below)
- `tests/meta/` contract-test tier (unit mark) pinning the CLI/docs/stack contracts, plus the `wiring` system-test mark: an observed wiring snapshot of the running graph drift-checked against the stack's committed `stacks/<name>/wiring.md`
- Split-stack bridging: `stacks/lite_offload_global/bridge.yaml` explicitly lists every boundary crossing and `tools/gen_dds_router.py` generates the DDS-router config from it deterministically (`--check` enforces bridge hard gate #2)
- New docs pages: [Modules](docs/development/modules.md), [Stacks](docs/development/stacks.md), [Fleets](docs/development/fleets.md), [Module CI](docs/development/module_ci.md), the generated [Module & Stack Catalog](docs/modules/index.md) marketplace, and the [Modular AirStack Walkthrough](docs/getting_started/modular_airstack.md)
- Intent flags on `airstack up` — `--sim isaac|airsim`, `--robots N`, `--headless`, `--play`/`--no-play`, `--no-autolaunch`, `--wait`, `--dry-run` — deriving the coordinated env-var sets (compose profiles, URDF, single/multi Isaac launch script) as exported leaf values, with a resolved-config banner and a per-run `.airstack/runs/<ts>/effective_config.env` dump; contract-tested in `tests/meta/test_launch_intent_contract.py` (unit mark)
- `airstack ready` (and `airstack up --wait`): staged flight-readiness gates mirroring the system-test budgets — containers → sim `/clock` → per-robot sentinel nodes → PX4 MAVROS-connected + `local_position/odom` streaming (the armable signal) — with per-gate diagnostics and `--json` for scripts
- Preflight validation in `airstack up` on **resolved** configuration (env > `--env-file` > `.env`): one-simulator guard no longer bypassed by `--env-file`; `NUM_ROBOTS>1` with the single-drone Isaac script is a named hard error; missing images are listed with an `image-pull` hint before compose starts an implicit build; missing `omni_pass.env` / empty Pegasus submodule / Docker < 29 surfaced on the host (`AIRSTACK_SKIP_PREFLIGHT=1` downgrades errors to warnings)
- tmux pane output is mirrored to container stdout via shared `.tmux.conf` hooks, so `docker logs` / `airstack logs` now show colcon builds, `ros2 launch` output, sim loading, and crashes

- Automatic `unit-tests.yml` PR gate on `ubuntu-latest`, plus `run_meta.json` outcome metadata so reports distinguish completed simulation campaigns from collection errors, empty selections, timeouts, and cancellations
- Feature notebook workflow (`use-feature-notebook` skill): every agent-implemented feature gets a local, gitignored `notebook/NNN-feature-slug/` entry with a status-tracked `design_spec.md` (written before coding) and `results/` artifacts + self-contained `results_summary.md` that populate the feature's PR description
- Battery and telemetry display in GCS RQT control panel (voltage and percentage per robot when MAVROS battery topic is bridged)
- `TARGET_ARCH` build arg (default `x86_64`) in `Dockerfile.robot` to arch-parametrize `LD_LIBRARY_PATH`; `docker-compose.yaml` passes `TARGET_ARCH: aarch64` to the `voxl` and `l4t` real-robot image builds
- `ros-${ROS_DISTRO}-mavros-extras` in the robot image (provides the vision_pose plugin used for external-pose deployments)
- `overrides/l4t-px4-realrobot.env` — site-agnostic deployment override for a single real PX4 robot on a Jetson (aarch64/l4t)
- `integration` test tier (`tests/integration/`, `integration` mark) with a shared `robot_autonomy_stack` fixture (robot container, no sim/GPU)
- `waypoint_flight` system test (`tests/system/test_waypoint_flight.py`): takeoff → ordered waypoint route via `NavigateTask` (dispatched as a dense plan) → land, judged on the odometry track by the standalone stdlib-only `tests/waypoint_checker.py` (in-order corridor arrival within `--waypoint-tolerance`, final goal within `--goal-tolerance`, per-waypoint `--waypoint-timeout`); validated end-to-end in Isaac Sim; serves as the standard acceptance check after integrating or swapping a planner module

### Changed

- **Stacks are the only launch path**: the autonomy topology is selected by `AIRSTACK_STACK_DIR`/`AIRSTACK_STACK_ENTRY` (exported by `airstack up --stack`), dispatched in `autonomy_bringup/launch/robot.launch.xml`. A set `AUTONOMY_ROLE` is now a **preflight hard error**; with no `--stack`, **`full_default`** is the default, machine-proven graph-identical to the old `AUTONOMY_ROLE=full`
- `robot-desktop` image slimmed **17.1 GB → 6.06 GB (−65%)** by moving MACVO's torch/TensorRT/weights into the `asm_macvo` module Docker layer — the composed image carries the 11.3 GB layer only for MACVO users
- Shared DDS-router configs moved out of the per-role `onboard_all/` tree up to `autonomy_bringup/config/`; the split-stack router config is now **generated** from the stack's `bridge.yaml` by `tools/gen_dds_router.py` (the generated config deliberately drops the legacy split's `set_trajectory_mode` crossing — doctor hard gate #2)
- Isaac launch scripts deduplicated onto a shared `pegasus_app.PegasusApp` base (`simulation/isaac-sim/launch_scripts/pegasus_app.py`): the six scripts become scenario declarations (~40–170 lines each, net −438 lines) with hooks for NatNet/scene-import extras; behavior verified by full system-test parity (liveliness, sensors, takeoff/hover/land on Isaac). `ISAAC_SIM_HEADLESS` and `ISAAC_SIM_LIVESTREAM` now work uniformly in **every** launch script (previously each was honored by only half of them)
- Launch-workflow docs corrected against actual behavior: `ISAAC_SIM_SCENE` (nonexistent) replaced by `ISAAC_SIM_SCRIPT_NAME`/`ISAAC_SIM_GUI`, getting-started reflects the paused-by-default sim and Foxglove UI, isaac docker.md defaults table matches `.env`, ms-airsim MAVROS ports/FOV/vehicle naming fixed, AGENTS.md uses the real `down`/`image-build` command names

- Unit-test documentation now matches the co-located layout: the `add-unit-tests` and `run-system-tests` skills and the testing docs record which runner each language uses (C++ gtests via `colcon test` under the `build_packages` mark; Python via the root harness, plus `colcon test` for `ament_python` packages), and stop instructing authors to write `@pytest.mark.unit` by hand — `conftest.py` applies it by file location
- Ephemeral CI GPU runners spawn via NVIDIA OSMO (not OpenStack); `system-tests.yml` / `docker-build.yml` still use `airstack-ephemeral`
- Default system-test `--sim` is `isaacsim`; pass `--sim msairsim` to opt in to Microsoft AirSim
- `-m build_packages` CI runs pull `cache_*` images instead of baking sim images
- `docker-build.yml` retags unchanged images on VERSION bumps (content fingerprint) instead of always rebuilding; floating `cache_*` tags still seed PR layer cache
- Automatic OSMO validation runs the pull-only `build_packages` gate whenever a PR is opened, updated, or reopened; GPU-intensive simulation campaigns (including OptiTrack) are selected through `/pytest` or `workflow_dispatch`
- `robot-l4t` compose service knobs are now env-overridable (`AUTONOMY_ROLE` (removed on this branch — see below), `FCU_URL`, and the rosbag path via `BAG_STORAGE_PATH`); `FCU_URL` unquoted so the literal serial path reaches MAVROS
- `zed-l4t` image: ZED SDK 4.2 → 5.2 with the coupled ROS deps (`zed_msgs` 5.2.1, `point_cloud_transport(_plugins)` 4.x, add `backward_ros`)
- Unit tests are defined by `tests/colcon_unit_test_packages.yaml`: `conftest.py` collects each listed package's co-located `test/` dir under `--import-mode=importlib` and marks it `unit` (ament lint files are skipped and run under `colcon test`)

### Removed

- The legacy `AUTONOMY_ROLE` launch dispatch and its per-role launch trees (`onboard_all/`, `onboard_local_offboard_global/`) — stacks are the only launch path; a set `AUTONOMY_ROLE` is a preflight error
- **MACVO extracted from trunk** to [castacks/asm_macvo](https://github.com/castacks/asm_macvo) (install with `airstack module add`)
- **OptiTrack/NatNet extracted from trunk** to [castacks/asm_optitrack](https://github.com/castacks/asm_optitrack): the `natnet_ros2` client + PX4 external-vision fusion, the NatNet server emulator and its Isaac wrapper, the `optitrack` e2e test, the host integration tests, and the `isaac-optitrack-simulation.env` / `l4t-optitrack-realrobot.env` overrides all live in the module repo now
- The stillborn `ensemble_planner` skeleton
- The legacy Gazebo parallel-bringup tree (8 dead files under `exploration/launch/robot_launch_gazebo/`)
- `tests/goldens/wiring/` — wiring baselines now live per-stack as `stacks/<name>/wiring.md`
- Pre-co-location unit-test scaffolding: the six per-layer stub READMEs under `tests/robot/` (which instructed authors to add tests in directories tests no longer live in) and `tests/sim/motive_emulator/README.md` (superseded by `simulation/isaac-sim/extensions/optitrack.natnet.emulator/` and `tests/integration/natnet/`)

### Fixed

- `barebones_pegasus_launch.py` (the documented template script) crashed with `NameError: os` on construction
- `isaac-sim-livestream` compose service silently produced a black stream when `ISAAC_SIM_SCRIPT_NAME` was a multi-drone script (livestream setup existed only in the single-drone scripts)
- `NATNET_BODY_NAME`/`NATNET_TARGET_NAME` env overrides documented by the single-drone NatNet script now actually work
- `airstack up` guards (one-simulator, URDF pairing) validated `.env` only and were bypassed by `--env-file overrides/...`; they now check the resolved configuration
- `pytest tests/` now collects the co-located unit tests before mark filtering. The old guard skipped injection whenever any path was on the command line, and `tests/` is a path — CI collected 97 of 252 items and the Python unit tests ran nowhere. Narrowing (`pytest tests/system/test_x.py`) still skips injection; repository-root and empty-path collection are rejected
- Empty CI pytest arguments no longer become `pytest tests/ ""` and recurse through the repository; collection/import, setup/teardown, partial, and interrupted artifacts are reported as non-comparable instead of false 0% simulation-policy results, and metric regression runs only for an identical simulation campaign fingerprint
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
- The NatNet emulator is now installed as a Kit extension: `Dockerfile.isaac-ros` pip-installs it editable into the Isaac python and bind-mounts the repo copy over it (the same pattern as `pegasus.simulator`), and the natnet launch scripts `enable_extension` it before importing. Being on a Kit `--ext-folder` search path only makes Kit *aware* of an extension — it does not put the package on `sys.path` — so the scripts previously died with `ModuleNotFoundError: No module named 'optitrack'`

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
