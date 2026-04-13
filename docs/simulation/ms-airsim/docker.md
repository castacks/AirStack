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
├── environments/             # Pre-built UE4 binaries (git-ignored)
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

Microsoft AirSim (legacy) is gated behind a Docker Compose profile:

```bash
# Start alongside the robot stack
airstack up --profile ms-airsim --profile desktop

# Build the image first
airstack image-build --profile ms-airsim
```

Alternatively, set `COMPOSE_PROFILES=ms-airsim,desktop` in `.env` and run `airstack up`.

### What happens on startup

The container runs `entrypoint.sh`, which:

1. Generates `settings.json` from the Jinja2 template using current environment variables
2. Creates a tmux session named `ms-airsim`
3. Launches the UE4 binary as the `ms-airsim` user (UE4 refuses to run as root)
4. Waits for the AirSim API to become available (TCP port 41451)
5. Spawns one PX4 SITL instance per robot, each in its own tmux window
6. Builds the ROS 2 bridge workspace (`colcon build`)
7. Launches one bridge node per robot, each with `ROS_DOMAIN_ID=<robot_index>`

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `AUTOLAUNCH` | `true` | Auto-start on container launch |
| `MS_AIRSIM_BINARY_PATH` | `/ms-airsim-env/Blocks.sh` | Path to UE4 binary inside container |
| `MS_AIRSIM_ENV_DIR` | `../environments` | Host path to extracted UE4 environment |
| `MS_AIRSIM_HEADLESS` | `false` | Run UE4 without a window (`-RenderOffScreen -nosound`) |
| `NUM_ROBOTS` | `1` | Number of vehicles and PX4 SITL instances |
| `SIM_IP` | `172.31.0.200` | Simulator IP on `airstack_network` |

**Camera template variables** (override in `.env` to regenerate `settings.json`):

| Variable | Default | Description |
|----------|---------|-------------|
| `AIRSIM_CAM_WIDTH` | `480` | Camera image width (px) |
| `AIRSIM_CAM_HEIGHT` | `300` | Camera image height (px) |
| `AIRSIM_CAM_FOV` | `90` | Camera horizontal FOV (degrees) |
| `AIRSIM_CAM_X` | `0.4` | Camera X offset from body center (m) |
| `AIRSIM_CAM_Y` | `0.06` | Camera Y half-baseline (m) |
| `AIRSIM_CAM_Z` | `0.0` | Camera Z offset from body center (m) |
| `AIRSIM_CAM_PITCH` | `0.0` | Camera pitch angle (degrees) |
| `AIRSIM_SPAWN_SPACING` | `3.0` | Y-axis spacing between robots (m) |

**Example overrides:**

```bash
# Two robots
NUM_ROBOTS=2 airstack up --profile ms-airsim

# Headless (no GUI, uses UE4's -RenderOffScreen)
MS_AIRSIM_HEADLESS=true airstack up --profile ms-airsim

# Custom environment binary
MS_AIRSIM_ENV_DIR=/data/airsim_envs MS_AIRSIM_BINARY_PATH=/ms-airsim-env/CityEnviron.sh airstack up --profile ms-airsim
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
| 4561–456N | TCP | PX4 lockstep (one per robot, N = robot index) |
| 24541–2454N | UDP | MAVLink offboard (one per robot) |
| 24581–2458N | UDP | MAVLink onboard (one per robot) |

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

### UE4 Environment

```yaml
- ${MS_AIRSIM_ENV_DIR:-../environments}:/ms-airsim-env:rw
```

Mounts pre-built UE4 binaries. Set `MS_AIRSIM_ENV_DIR` in `.env` to point to the extracted environment directory.

## Accessing the Container

### Via tmux Session

```bash
# Attach to the running tmux session
airstack connect ms-airsim
# Then inside the container:
tmux a -t ms-airsim
```

**Tmux windows layout:**

| Window | Contents |
|--------|---------|
| 0 | AirSim UE4 binary |
| 1..N | PX4 SITL instance for robot 1..N |
| N+1..2N | ROS 2 bridge node for robot 1..N |

**Useful tmux commands:**

- `Ctrl-b n` / `Ctrl-b p` — next / previous window
- `Ctrl-b d` — detach from session
- `Ctrl-b [` — scroll mode (arrow keys, `q` to exit)

### Via Logs

```bash
airstack logs ms-airsim
```

## Multi-Robot Support

Set `NUM_ROBOTS` to spawn multiple vehicles. Each robot gets:

- A named vehicle in `settings.json` (`robot_1`, `robot_2`, …)
- Its own PX4 SITL instance with unique ports
- Its own ROS 2 bridge node on `ROS_DOMAIN_ID=<index>`
- Vehicles are spawned 3 m apart along the Y-axis (configurable via `AIRSIM_SPAWN_SPACING`)

```bash
# Launch with 3 robots
NUM_ROBOTS=3 airstack up --profile ms-airsim --profile desktop
```

## Image Management

### Pulling Pre-built Images

```bash
# Login to AirLab registry
docker login airlab-docker.andrew.cmu.edu

# Pull image
docker compose -f simulation/ms-airsim/docker/docker-compose.yaml pull
```

### Building from Source

```bash
# Build image
airstack image-build --profile ms-airsim

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
AIRSIM_CAM_FOV=120 airstack up --profile ms-airsim
```

## Troubleshooting

**AirSim binary won't launch:**

- Confirm the UE4 binary is executable: `chmod +x /ms-airsim-env/*.sh`
- Check GPU access: `nvidia-smi` inside container
- Verify `DISPLAY` is set and X11 socket is mounted: `echo $DISPLAY`, `xhost +local:docker`
- Check disk space: pre-built environments are 3–10 GB

**Bridge can't connect to AirSim API:**

- Ensure AirSim binary started successfully (check tmux window 0)
- The entrypoint retries until the API is ready; check for connection errors in the container logs
- Verify `ms_airsim_ip` in `bridge.yaml` matches where AirSim is running (default: `127.0.0.1` — same container)

**PX4 SITL won't connect:**

- Confirm `settings.json` was generated with correct `TcpPort` values
- Check AirSim console for "Waiting for TCP connection" messages
- Verify the PX4 lockstep port is not blocked by a firewall

**MAVROS won't connect (robot container):**

- Verify `SIM_IP=172.31.0.200` is set in `.env`
- Ensure PX4 SITL has started (look for `[mavlink]` output in the PX4 tmux window)
- Check MAVLink ports match: offboard `24541+i`, onboard `24581+i`

**No depth images on ROS 2 topics:**

- Verify the camera names in `settings.json` match those in `bridge.yaml`
- Check bridge node output for connection errors (tmux bridge window)
- Echo the topic: `ros2 topic echo /robot_1/sensors/front_stereo/depth --once`

**ROS 2 topics not visible from robot container:**

- Confirm both containers are on `airstack_network`: `docker network inspect airstack_network`
- Check `ROS_DOMAIN_ID` is consistent between containers
- Verify DDS multicast is working: `ros2 topic list` from inside each container

## See Also

- [Microsoft AirSim (legacy) Overview](index.md) — capabilities, configuration, and architecture
- [Simulation Overview](../index.md) — Choosing between simulators
- [Isaac Sim Docker](../isaac_sim/docker.md) — Isaac Sim container reference
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) — General Docker operations
