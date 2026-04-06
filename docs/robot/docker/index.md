# Docker Services

AirStack's robot stack is containerized using Docker Compose. The compose files live under `robot/docker/` and are organized into a **base** layer shared by all targets and **profile-specific** services for each deployment target.

## File Structure

```
robot/docker/
├── robot-base-docker-compose.yaml   # Shared base service definition
├── docker-compose.yaml              # Platform-specific service definitions
└── .bashrc                          # Bash config mounted into every container
```

## Service Hierarchy

All robot services inherit from the shared `robot_base` service defined in `robot-base-docker-compose.yaml`. Platform-specific services then extend `robot_base` and override only what differs for that target.

```
robot_base  (robot-base-docker-compose.yaml)
│
├── robot-desktop     (profile: desktop)   x86-64 desktop / simulation
│   └── simple-robot  (profile: simple)   desktop + simple sim override
│   └── robot-test    (profile: test)     desktop + colcon test override
│
├── robot-voxl    (profile: voxl)      ModalAI VOXL platform
├── robot-l4t     (profile: l4t)       NVIDIA Jetson (Linux for Tegra)
└── zed-l4t       (profile: l4t)       ZED camera driver on Jetson
```

## Base Service (`robot_base`)

`robot_base` provides defaults that every robot container shares:

| Aspect | Details |
|---|---|
| **GPU access** | NVIDIA GPU reserved via the `deploy.resources` section |
| **Display** | `DISPLAY`, `QT_X11_NO_MITSHM`, and X11 socket mounted for GUI tools |
| **Source tree** | The entire `AirStack/` repo is bind-mounted at `/root/AirStack` inside the container |
| **ROS workspace** | `common/ros_packages` is mounted into the ROS 2 workspace `src/common` |
| **Shell config** | `.bashrc` and `inputrc` are bind-mounted so the developer experience is consistent across rebuilds |
| **Bags** | `robot/bags/` is mounted at `/bags` for recording and playback |
| **Launch variables** | All `*_LAUNCH_PACKAGE` / `*_LAUNCH_FILE` environment variables are forwarded from the host `.env` file |

## Platform Profiles

Select a profile by passing `--profile <name>` to `docker compose` (or via the `airstack` CLI).

### `desktop` — x86-64 development machine

This is the DEFAULT profile as specified by `COMPOSE_PROFILES=desktop` in the root level `.env` file. It will run by default if no profile is passed or with `airstack up --profile desktop`. Use this profile when developing or running simulations on an x86-64 Linux workstation.

- **Image:** `...:v<TAG>_robot-x86-64_<MODE>`
- **Base image:** `nvidia/cuda:13.0.2-base-ubuntu22.04`
- **Network:** isolated `airstack_network` bridge (prevents conflicts with other developers on the same LAN)
- **SSH:** host ports `2223–2243` forwarded to port `22` in each container, one port per robot replica
- **Scaling:** `NUM_ROBOTS` env var controls the number of replicas (default: 1)
- **Robot identity:** derived from the Docker container name → `ROBOT_NAME_SOURCE=container_name`
- **Auto-launch:** controlled by `AUTOLAUNCH` (default: `true`); starts a `tmux` session that runs the ROS 2 launch file

### `simple` — desktop + simple simulator

Runs with `airstack up --profile simple`. Extends `desktop` with `SIM_TYPE=simple`, intended for the lightweight built-in simulator.

### `voxl` — ModalAI VOXL

Runs with `airstack up --profile voxl`. Use this profile when deploying on a ModalAI VOXL flight computer.

- **Image:** `...:v<TAG>_robot-voxl_<MODE>`
- **Base image:** `ubuntu:22.04` (no CUDA; VOXL has its own compute stack)
- **Skipped components:** OpenVDB, MACVO, TensorRT
- **Network:** `host` (relies on the physical network for DDS discovery)
- **Robot identity:** derived from the device hostname → `ROBOT_NAME_SOURCE=hostname`

### `l4t` — NVIDIA Jetson (Linux for Tegra)

Runs with `airstack up --profile l4t`. Use this profile when deploying on an NVIDIA Jetson device running L4T.

- **Image:** `...:v<TAG>_robot-l4t_<MODE>`
- **Base image:** `nvcr.io/nvidia/l4t-jetpack:r36.4.0`
- **Network:** `host`
- **IPC:** `host` (needed for shared-memory DDS transports on Jetson)
- **Storage:** `/media/airlab/Storage/airstack_collection` mounted at `/bags`
- **Robot identity:** derived from the device hostname → `ROBOT_NAME_SOURCE=hostname`

A companion service `zed-l4t` (also under the `l4t` profile) runs only the ZED stereo camera driver in a separate container, keeping camera drivers isolated from the main autonomy stack.

### `test` — CI / colcon test runner

Extends `desktop` but overrides the container command to run `colcon test` and exit with the test result code. Useful in CI pipelines.

## Environment Variables

Key variables are set in the project's `.env` file and forwarded into the container:

| Variable | Description |
|---|---|
| `VERSION` | Image version tag |
| `DOCKER_IMAGE_BUILD_MODE` | Build mode (e.g. `release`, `dev`) |
| `PROJECT_DOCKER_REGISTRY` | Docker registry prefix |
| `PROJECT_NAME` | Project / image name |
| `NUM_ROBOTS` | Number of robot replicas (desktop only, default `1`) |
| `AUTOLAUNCH` | Whether to auto-start the ROS 2 stack on container start (default `true`) |
| `ROBOT_LAUNCH_PACKAGE` / `ROBOT_LAUNCH_FILE` | Top-level ROS 2 launch target |
| `OFFBOARD_BASE_PORT` / `ONBOARD_BASE_PORT` | MAVLink UDP port base values (desktop/sim only) |
| `ROBOT_NAME_MAP_CONFIG_FILE` | YAML mapping config used to resolve a name to `ROBOT_NAME` and `ROS_DOMAIN_ID` (default: `default_robot_name_map.yaml`) |

See [Robot Identity](robot_identity.md) for how `ROBOT_NAME` and `ROS_DOMAIN_ID` are derived automatically inside the container from these profiles.
