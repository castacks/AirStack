# Microsoft AirSim (legacy) Docker Configuration

Microsoft AirSim (legacy) runs in a Docker container with NVIDIA GPU support and full integration with the AirStack ecosystem.

## File Structure

```
simulation/ms-airsim/docker/
├── docker-compose.yaml       # Main service definition
├── Dockerfile                # Image definition
└── entrypoint.sh             # Container startup script

simulation/ms-airsim/
├── config/
│   ├── settings.json.j2      # Jinja2 template for AirSim settings
│   ├── settings.json         # Generated settings (git-ignored)
│   └── generate_settings.py  # Settings generator script
├── assets/scenes/            # Pre-built UE4 binaries (git-ignored), e.g. Blocks/, AirSimNH/
│   └── fetch_scene.sh        # Idempotent download + extract helper (tracked)
└── ros_ws/                   # ROS 2 bridge workspace
    └── src/
        └── ms_airsim_ros_bridge/  # Depth + camera_info bridge node
```

## Service Architecture

The Microsoft AirSim (legacy) service is defined in `simulation/ms-airsim/docker/docker-compose.yaml`.

**Key components:**

| Component | Purpose |
|-----------|---------|
| **AirSim UE4 binary** | Unreal Engine 4.27 physics simulation and rendering |
| **PX4 SITL** | Autopilot firmware running in software-in-the-loop mode |
| **ROS 2 Bridge** | Publishes depth images and camera info to ROS 2 |
| **GPU Acceleration** | NVIDIA GPU + Vulkan for UE4 rendering |

## Launch

### Starting Microsoft AirSim (legacy)

For the user-facing launch path (scene selection + `airstack up --sim airsim`), see the [Quick Start in the overview](index.md#quick-start). At the container level, the service is gated behind a Docker Compose profile:

```bash
# Explicit-profile form (equivalent to `airstack up --sim airsim`)
airstack up --profile ms-airsim --profile desktop
```

Alternatively, set `COMPOSE_PROFILES=ms-airsim,desktop` in `.env` and run `airstack up`.

### What happens on startup

The container runs `entrypoint.sh` (`simulation/ms-airsim/docker/entrypoint.sh`), which:

1. Generates `settings.json` from the Jinja2 template using current environment variables
2. Creates a tmux session named `ms-airsim` with a first window named `airsim`
3. Resolves the scene: an explicit `MS_AIRSIM_BINARY_PATH` wins (and must exist); otherwise `MS_AIRSIM_SCENE` (default `blocks`) selects a `fetch_scene.sh` key
4. Builds the ROS 2 bridge workspace (`colcon build`)
5. In the `airsim` window: auto-fetches the selected scene if it isn't downloaded yet (so progress is visible), then launches the UE4 binary as the `ms-airsim` user (UE4 refuses to run as root)
6. Creates one bridge window per robot (`robot_<i>_bridge`), each running the bridge node with `ROS_DOMAIN_ID=<robot_index>`
7. Waits for the AirSim API to become available (TCP port 41451)
8. Sleeps `MS_AIRSIM_PX4_START_DELAY` seconds (default 3) so AirSim sensors settle before PX4's EKF snapshots a local origin
9. Creates one PX4 SITL window per robot (`robot_<i>_px4`), each running `px4 ... -i <robot_index>`

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `AUTOLAUNCH` | `true` | Auto-start on container launch |
| `MS_AIRSIM_BINARY_PATH` | _(unset → auto-fetch Blocks)_ | Path to UE4 binary inside container. If unset, the entrypoint fetches Blocks into the mounted scenes dir and points at it. |
| `MS_AIRSIM_ENV_DIR` | `../assets/scenes` | Host path to extracted UE4 scenes |
| `MS_AIRSIM_HEADLESS` | `false` | Run UE4 without a window (`-RenderOffScreen -nosound`) |
| `MS_AIRSIM_SCENE` | _(empty → `blocks`)_ | Scene shortname (a `fetch_scene.sh` key, set by `airstack up --scene <shortname>`); ignored when `MS_AIRSIM_BINARY_PATH` is set |
| `MS_AIRSIM_PX4_START_DELAY` | `3` | Seconds to wait after AirSim becomes ready before starting PX4, so sensors settle before the EKF snapshots a local origin |
| `NUM_ROBOTS` | `1` | Number of vehicles and PX4 SITL instances |
| `SIM_IP` | `172.31.0.200` | Simulator IP on `airstack_network` |

The camera template variables (`AIRSIM_CAM_*`) and their defaults are documented in the [camera configuration reference](index.md#cameras); `AIRSIM_SPAWN_SPACING` (default `3.0`) sets the Y-axis spacing between spawned robots in meters.

**Example overrides:**

```bash
# Two robots
airstack up --sim airsim --robots 2

# Headless (no GUI, uses UE4's -RenderOffScreen)
airstack up --sim airsim --headless

# Custom scene binary (no flag equivalent)
MS_AIRSIM_ENV_DIR=/data/airsim_envs MS_AIRSIM_BINARY_PATH=/ms-airsim-env/CityEnviron/LinuxNoEditor/CityEnviron.sh airstack up --sim airsim
```

## Settings Generation

Microsoft AirSim (legacy) is configured via `settings.json`, which is generated from a Jinja2 template at container startup:

```
simulation/ms-airsim/config/settings.json.j2  →  generate_settings.py  →  settings.json
```

The template is mounted into the container at `/home/ms-airsim/Documents/AirSim/` (the standard AirSim config location). Camera and vehicle parameters are controlled entirely via environment variables — no manual JSON editing needed.

To preview or regenerate settings outside Docker:

```bash
cd simulation/ms-airsim/config
NUM_ROBOTS=2 python3 generate_settings.py
```

## Networking

**Network configuration:**

- **Network:** `airstack_network` (172.31.0.0/24)
- **Fixed IP:** 172.31.0.200
- **Purpose:** Communicate with robot containers via ROS 2 DDS and MAVLink UDP

**Port usage:**

| Port | Protocol | Purpose |
|------|----------|---------|
| 41451 | TCP | AirSim Python API |
| `4560 + i` | TCP | PX4 lockstep (`TcpPort`, one per robot `i` = 1..N) |
| `24540 + i` | UDP | AirSim MAVLink control channel, local (`ControlPortLocal`, one per robot) |
| `24580 + i` | UDP | AirSim MAVLink control channel, remote (`ControlPortRemote`, one per robot) |

## GPU Access

Microsoft AirSim (legacy) (UE4) requires NVIDIA GPU access with Vulkan support:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

The Vulkan ICD is mounted from the host:

```yaml
- /usr/share/vulkan/icd.d/nvidia_icd.json:/usr/share/vulkan/icd.d/nvidia_icd.json:ro
```

**Requirements:**

- NVIDIA GPU (GTX 1070+ recommended)
- NVIDIA Container Toolkit installed
- Vulkan-capable drivers on host

**Verify GPU and Vulkan access:**

```bash
# Inside container
nvidia-smi
vulkaninfo --summary
```

## Volume Mounts

### Display (X11)

```yaml
- /tmp/.X11-unix:/tmp/.X11-unix
```

Enables GUI rendering on the host display.

### AirSim Configuration

```yaml
- ../config:/home/ms-airsim/Documents/AirSim:rw
```

Mounts `simulation/ms-airsim/config/` to the AirSim config directory. `settings.json` is generated here at startup and read by UE4.

### ROS 2 Bridge Workspace

```yaml
- ../ros_ws:/root/ros_ws:rw
```

Mounts the bridge source so edits on the host are reflected after a rebuild inside the container.

### UE4 Scene

Mounts pre-built UE4 binaries. If `MS_AIRSIM_BINARY_PATH` is unset, the entrypoint auto-populates this directory with the default Blocks scene on first launch. Run `assets/scenes/fetch_scene.sh` to pre-fetch or pick a different scene, or set `MS_AIRSIM_ENV_DIR` in `.env` to point to an external scenes directory.

## Accessing the Container

### Via tmux Session

```bash
# Attach to the running tmux session
airstack connect ms-airsim
# Then inside the container:
tmux a -t ms-airsim
```

**Tmux windows layout** (`1 + 2*NUM_ROBOTS` windows, in creation order — see [What happens on startup](#what-happens-on-startup)):

| Window | Name | Contents |
|--------|------|----------|
| 0 | `airsim` | Scene fetch (if needed) + AirSim UE4 binary |
| 1..N | `robot_<i>_bridge` | ROS 2 bridge node for robot `i` = 1..N |
| N+1..2N | `robot_<i>_px4` | PX4 SITL instance for robot `i` = 1..N (created only after the AirSim API is ready + `MS_AIRSIM_PX4_START_DELAY`) |

**Useful tmux commands:**

- `Ctrl-b n` / `Ctrl-b p` — next / previous window
- `Ctrl-b d` — detach from session
- `Ctrl-b [` — scroll mode (arrow keys, `q` to exit)

### Via Logs

```bash
airstack logs ms-airsim
```

## Multi-Robot Support

Use `airstack up --robots N` (which sets `NUM_ROBOTS`) to spawn multiple
vehicles. Each robot gets:

- A named vehicle in `settings.json` (`robot_1`, `robot_2`, …)
- Its own PX4 SITL instance with unique ports
- Its own ROS 2 bridge node on `ROS_DOMAIN_ID=<index>`
- Vehicles are spawned 3 m apart along the Y-axis (configurable via `AIRSIM_SPAWN_SPACING`)

```bash
# Launch with 3 robots
airstack up --sim airsim --robots 3
```

## Image Management

### Pulling Pre-built Images

```bash
# Pull image (the AirLab registry is public — no login needed)
docker compose -f simulation/ms-airsim/docker/docker-compose.yaml pull
```

### Building from Source

```bash
# Build image
airstack images build --profile ms-airsim

# Or directly
docker compose -f simulation/ms-airsim/docker/docker-compose.yaml build
```

!!! note
    The image clones and compiles PX4 SITL from source (`v1.16.1`), which takes 15–30 minutes on first build.

## Development Workflow

### Editing the Bridge Node

The ROS workspace is mounted live, so you can edit on the host and rebuild inside the container:

```bash
# Rebuild bridge
docker exec ms-airsim bash -c "cd /root/ros_ws && colcon build --symlink-install"
```

### Iterating on Settings

Change camera or vehicle parameters via environment variables and restart the container — `settings.json` is regenerated each time.

```bash
AIRSIM_CAM_FOV=120 airstack up --sim airsim
```

## Troubleshooting

**AirSim binary won't launch:**

- Confirm the UE4 binary is executable: `chmod +x /ms-airsim-env/*.sh`
- Check GPU access: `nvidia-smi` inside container
- Verify `DISPLAY` is set and X11 socket is mounted: `echo $DISPLAY`, `xhost +local:docker`
- Check disk space: pre-built environments are 3–10 GB

**PX4 SITL won't connect:**

- Confirm `settings.json` was generated with correct `TcpPort` values (`4560 + i`)
- Check AirSim console for "Waiting for TCP connection" messages
- Verify the PX4 lockstep port is not blocked by a firewall

**ROS 2 topics not visible from robot container:**

- Confirm both containers are on `airstack_network`: `docker network inspect airstack_network`
- Check `ROS_DOMAIN_ID` is consistent between containers
- Verify DDS multicast is working: `ros2 topic list` from inside each container

For user-facing issues (bridge can't connect to AirSim, no depth images, MAVROS won't connect), see the [overview → Troubleshooting](index.md#troubleshooting).

## See Also

- [Microsoft AirSim (legacy) Overview](index.md) — quick start, settings/camera/bridge configuration, published topics
- [Simulation Overview](../index.md) — Choosing between simulators
- [Isaac Sim Docker](../isaac_sim/docker.md) — Isaac Sim container reference
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) — General Docker operations
