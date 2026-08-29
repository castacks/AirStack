# Environment Variable Reference (`.env`)

The top-level [`.env`](https://github.com/castacks/AirStack/blob/main/.env) file sets Docker Compose **interpolation variables** — image tags, profile selection, replica counts, and per-container launch switches. These variables do **not** automatically become environment variables inside the containers: a variable only reaches a container if a compose file forwards it through an `environment:` entry (see each *Consumed by* column). `airstack up` reads `.env` automatically; launch-intent flags (`--sim`, `--robots`, `--stack`, ...) override it by exporting the same variables before compose runs. This page is the complete schema; the [Docker guide](../docker/index.md#environment-variables) summarizes the subset forwarded into robot containers, and the [Configuration overview](index.md) explains where non-compose configuration lives.

## Project & Images

These variables assemble every image tag as `${PROJECT_DOCKER_REGISTRY}/${PROJECT_NAME}:v${VERSION}_<service-suffix>` (robot images additionally append `_${DOCKER_IMAGE_BUILD_MODE}`).

| Variable | Purpose | Default | Consumed by |
| -------- | ------- | ------- | ----------- |
| `PROJECT_NAME` | Repository name for Docker images and part of every image tag | `"airstack"` | `image:` tags in all compose files (`robot/docker/`, `gcs/docker/`, `simulation/*/docker/`) |
| `VERSION` | Semver image version; bumped per release, so the value in `.env` is always the current release (e.g. `0.21.0-dev.15`) | current release semver | `image:` tags in all compose files; CI version gate (`check-version-increment.yml`) |
| `DOCKER_IMAGE_BUILD_MODE` | Image-tag discriminator **only** — no Dockerfile consumes it. Keep `dev` (mounted code, built live); a real `prebuilt` workspace-baked stage is future work | `"dev"` | Tag suffix of the robot images (`robot/docker/docker-compose.yaml`) |
| `PROJECT_DOCKER_REGISTRY` | Registry to push/pull images from | `"airlab-docker.andrew.cmu.edu/airstack"` | `image:` tags in all compose files |
| `COMPOSE_PROFILES` | Default compose profiles when none are passed explicitly | `"desktop,isaac-sim"` | Docker Compose profile selection; rewritten by `airstack up --sim <sim>` (swaps the simulator profile) and `--fleet` (heterogeneous fleets swap `desktop` for `fleet`) |

## Launch Behavior

| Variable | Purpose | Default | Consumed by |
| -------- | ------- | ------- | ----------- |
| `AUTOLAUNCH` | If `false`, containers spawn idle with no launch command (tmux session still created) | `"true"` | Container `command:` of every robot service, `isaac-sim`, `ms-airsim`, and `gcs` (each gates its tmux autolaunch on it) |
| `NUM_ROBOTS` | Number of robot containers to launch (compose replicas) | `"1"` | `deploy.replicas` of `robot-desktop`/`robot-offboard`; forwarded into `isaac-sim`, `ms-airsim`, and `gcs` containers (drone spawn count / peer list). Overridden by `airstack up --robots N`; derived from the fleet file with `--fleet` |
| `RECORD_BAGS` | Start the bag recorder node with the stack (see [Rosbags](../logging/rosbags.md)) | `"false"` | `robot_base` and `gcs` `environment:` → `bag_recorder_pid` via the logging bringup |

## Isaac Sim

All four are forwarded into (or read by the `command:` of) the `isaac-sim` service in `simulation/isaac-sim/docker/docker-compose.yaml`.

| Variable | Purpose | Default | Consumed by |
| -------- | ------- | ------- | ----------- |
| `ISAAC_SIM_GUI` | USD scene path for the **non-standalone** launch path | `/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/simple_pegasus.scene.usd` | `ros2 launch isaacsim run_isaacsim.launch.py gui:=...` branch of the `isaac-sim` command (only when `ISAAC_SIM_USE_STANDALONE` is not `true`) |
| `ISAAC_SIM_USE_STANDALONE` | `true` = launch Isaac Sim via a standalone Python script; `false` = load the `ISAAC_SIM_GUI` USD file via `run_isaacsim.launch.py` | `"true"` | Branch selector in the `isaac-sim` container command |
| `ISAAC_SIM_SCRIPT_NAME` | Standalone launch script, resolved under `/AirStack/simulation/isaac-sim/launch_scripts/`. The default spawns exactly **one** drone; multi-robot needs `example_multi_px4_pegasus_launch_script.py` (auto-selected by `airstack up --robots N>1`) and fleets use `fleet_spawn.py` (auto-selected by `--fleet`) | `"example_one_px4_pegasus_launch_script.py"` | Standalone branch of the `isaac-sim` container command |
| `PLAY_SIM_ON_START` | Start the sim **playing** instead of paused (`airstack up --no-play` to come up paused) | `"true"` | `isaac-sim` `environment:` → the Pegasus launch scripts (`pegasus_app.py` and the example scripts); passed as `play_sim_on_start:=` in the USD launch path |

## Robot Identity & Description

| Variable | Purpose | Default | Consumed by |
| -------- | ------- | ------- | ----------- |
| `ROBOT_NAME_MAP_CONFIG_FILE` | Mapping file (in `robot/docker/robot_name_map/`) that resolves each container to `ROBOT_NAME` + `ROS_DOMAIN_ID` — see [Robot Identity](../docker/robot_identity.md) | `"default_robot_name_map.yaml"` | `robot_base` `environment:` → `resolve_robot_name.py`, run by `robot/docker/.bashrc` at container startup |
| `URDF_FILE` | Robot description, relative to the workspace `robot_descriptions/` install. Swapped automatically by `airstack up --sim isaac\|airsim` to the matching sensor URDF | `robot_descriptions/iris/urdf/iris_with_sensors.pegasus.robot.urdf` | `robot_base` `environment:` → `autonomy_bringup/launch/robot.launch.xml` (robot state publisher) |
| `DEBUG_RVIZ` | If `true`, launches RViz alongside the robot | `"false"` | `robot_base` `environment:` → `desktop_bringup/launch/robot.launch.xml` |

## Ports

| Variable | Purpose | Default | Consumed by |
| -------- | ------- | ------- | ----------- |
| `OFFBOARD_BASE_PORT` | Base UDP port for offboard (API-out) MAVLink streams; offset per robot so multi-agent FCU communication doesn't collide | `14540` | `robot_base` `environment:` → `interface_bringup/launch/interface.launch.py` (MAVROS `fcu_url` calculation) |
| `ONBOARD_BASE_PORT` | Base UDP port for onboard MAVLink streams, offset per robot | `14580` | Same as above |

## Variables Exported by `airstack up` (Not Set in `.env`)

`airstack up` parses its launch-intent flags (`parse_launch_intent` / `apply_launch_intent` in `airstack.sh`) and exports these before invoking compose. Set them by flag, not by editing `.env` — though explicit env / `--env-file` values take precedence (leaf-value precedence, with an override banner for fleet conflicts).

| Variable | Set by | Purpose | Consumed by |
| -------- | ------ | ------- | ----------- |
| `AIRSTACK_STACK_DIR` | `--stack <name>` (always exported; no stack = `/root/AirStack/stacks/full_default`) | Container path of the stack folder whose entry launch file defines the autonomy topology — see [Stacks](../../development/stacks.md) | `robot_base` `environment:` → `autonomy_bringup/launch/robot.launch.xml` dispatch |
| `AIRSTACK_STACK_ENTRY` | `--stack <name>:<entry>` (default `stack`) | Entry launch file name: `launch/<entry>.launch.xml` (split stacks use `onboard`/`offboard`) | Same as above |
| `FLEET_CONFIG_FILE` | `--fleet <name>` | Container path of the fleet file (`/root/AirStack/config/fleets/...`); empty = legacy `robot_name_map` resolution — see [Fleets](../../development/fleets.md) | `robot_base` and `isaac-sim` `environment:` → `robot/docker/.bashrc` (per-container identity/stack via `tools/fleet/resolve_fleet.py`) and `fleet_spawn.py` |
| `NUM_ROBOTS` | `--robots N`, or derived from the fleet file with `--fleet` (mutually exclusive flags) | Overrides the `.env` value above | As in Launch Behavior |
| `PLAY_SIM_ON_START` | `--play` / `--no-play` | Overrides the `.env` value above | As in Isaac Sim |
| `AUTOLAUNCH` | `--no-autolaunch` | Overrides the `.env` value above (sets `false`) | As in Launch Behavior |
| `ISAAC_SIM_HEADLESS` | `--headless` | Run Isaac Sim without a window | `isaac-sim` `environment:` (default `false`; forced `true` by the `isaac-sim-livestream` service) |
| `MS_AIRSIM_HEADLESS` | `--headless` | Run the UE4 binary off-screen | `ms-airsim` `environment:` |
| `QT_QPA_PLATFORM` | `--headless` (sets `offscreen`) | Keeps Qt tools (RViz etc.) from requiring a display | `robot_base` `environment:` |
| `COMPOSE_PROFILES` | `--sim isaac\|airsim\|simple` (swaps the simulator profile; `simple` also drops `desktop`), `--fleet` (heterogeneous: swaps `desktop` for `fleet`) | Overrides the `.env` value above | Docker Compose profile selection |
| `URDF_FILE` | `--sim isaac\|airsim` | Swaps to the simulator-matched sensor URDF | As in Robot Identity & Description |
| `ISAAC_SIM_SCRIPT_NAME` | `--robots` (one ↔ multi example script) and `--fleet` (→ `fleet_spawn.py`); an explicit env value always wins | Overrides the `.env` value above | As in Isaac Sim |
| `ISAAC_SIM_SCENE`, `ISAAC_SIM_STAGE_SCALE`, `MS_AIRSIM_SCENE` | `--scene <shortname>` via `simulation/resolve_scene.py` (`simulation/scenes.yaml` catalog) | Scene selection for the active simulator — see [Scenes](../../simulation/scenes.md) | `isaac-sim` / `ms-airsim` `environment:` → the launch scripts / `entrypoint.sh` |

## Notable Optional Variables

These appear commented-out in `.env` (or are read by compose with a built-in default) and are documented in full on their own pages.

| Variable | Purpose | Documented in |
| -------- | ------- | ------------- |
| `ISAAC_SIM_SCENE`, `ISAAC_SIM_STAGE_SCALE` | Direct scene override / stage scale for cm-authored stages (normally set via `--scene`) | [Scenes](../../simulation/scenes.md) |
| `MS_AIRSIM_SCENE`, `MS_AIRSIM_ENV_DIR`, `MS_AIRSIM_BINARY_PATH`, `MS_AIRSIM_HEADLESS`, `MS_AIRSIM_PX4_START_DELAY` | UE4 scene fetching/selection, binary override, off-screen rendering, PX4 start delay | [MS AirSim Docker](../../simulation/ms-airsim/docker.md) |
| `BAG_STORAGE_PATH` | Host directory mounted at `/bags` on the `l4t` profile (default `/media/airlab/Storage/airstack_collection`) | [Rosbags](../logging/rosbags.md) |
| `LOG_CONFIG` | Bag recorder topic-selection file in `logging_bringup/config/` (default `log.yaml`) | [Rosbags](../logging/rosbags.md) |
| `ISAAC_SIM_FOLLOW_CAM`, `ISAAC_SIM_FOLLOW_CAM_OFFSET`, `ISAAC_SIM_FOLLOW_CAM_LIGHT` | Viewport follow-camera: drone domain id to chase, world-frame offset, headlight for unlit interiors | [Isaac Sim](../../simulation/isaac_sim/index.md) (read by `pegasus_app.py`) |
| `SIM_IP` | Address robot containers use to reach the simulator (default `172.31.0.200`; every sim service binds this fixed address) | `robot/docker/docker-compose.yaml`, `simulation/isaac-sim/docker/docker-compose.yaml` |
| `FCU_URL` | MAVROS flight-controller URL on real hardware (default `/dev/ttyTHS4:115200` on `l4t`); desktop/sim derive it from the base ports instead | `robot/docker/docker-compose.yaml` (`robot-l4t`) |
| `CACHE_TAG` | Floating Docker-layer-cache image tag (default `cache`) — CI only | [CI/CD](../../development/intermediate/testing/ci_cd.md) |

## See Also

- [Docker Services — Environment Variables](../docker/index.md#environment-variables) — the subset forwarded into robot containers
- [Robot Configuration](index.md) — where non-compose configuration lives (stacks, module parameters, fleets)
- [AirStack CLI](../../development/beginner/airstack-cli/index.md) — the `airstack up` flags that export these variables
