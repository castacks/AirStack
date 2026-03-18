# Isaac Sim Docker Configuration

Isaac Sim runs in a Docker container with NVIDIA GPU support and full integration with the AirStack ecosystem.

## File Structure

```
simulation/isaac-sim/docker/
├── docker-compose.yaml           # Main service definition
├── Dockerfile.isaac-ros          # Image definition
├── .bashrc                       # Bash configuration
├── fastdds.xml                   # DDS configuration
├── omni_pass.env                 # Omniverse credentials (git-ignored)
├── omni_pass_TEMPLATE.env        # Template for credentials
├── omniverse.toml                # Omniverse settings
├── user.config.json              # Isaac Sim configuration (enables extensions)
└── user_TEMPLATE.config.json     # Template configuration
```

## Service Architecture

The Isaac Sim service is defined in `simulation/isaac-sim/docker/docker-compose.yaml`.

**Key components:**

| Component | Purpose |
|-----------|---------|
| **Isaac Sim Base** | NVIDIA Omniverse Isaac Sim with ROS 2 bridge |
| **Pegasus Extension** | Multi-rotor simulation extension |
| **ROS 2 Bridge** | Native ROS 2 topic publishing/subscribing |
| **GPU Acceleration** | NVIDIA GPU for rendering and physics |

## Launch Modes

Isaac Sim supports multiple launch modes:

### 1. Standard Launch (ROS 2 Integration)

Default mode with ROS 2 bridge:

```bash
airstack up isaac-sim
```

**What happens:**

- Launches Isaac Sim with ROS 2 bridge
- Loads configured scene (via `ISAAC_SIM_SCENE`)
- Publishes sensor topics to ROS 2
- Optionally auto-plays simulation (via `PLAY_SIM_ON_START`)

### 2. Standalone Python Launch

Launch with standalone Python script:

```bash
ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME=my_script.py airstack up isaac-sim
```

**Use cases:**

- Custom simulation logic
- Advanced scene setup
- Programmatic control

### 3. GUI-Only Mode

Launch just the Isaac Sim GUI (no ROS 2):

```bash
airstack up --profile isaac-sim-gui isaac-sim-gui
```

**Use cases:**

- Scene authoring
- Testing without robot stack
- Visual debugging

## Launch Configuration

The container command in docker-compose.yaml:

```yaml
command: >
  bash -c "
  tmux new -d -s isaac;
  if [ $$AUTOLAUNCH = 'true' ]; then
    if [ \"${ISAAC_SIM_USE_STANDALONE}\" = 'true' ]; then
      tmux send-keys -t isaac 'run_isaac_python /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/${ISAAC_SIM_SCRIPT_NAME}' ENTER
    else
      tmux send-keys -t isaac 'ros2 launch isaacsim run_isaacsim.launch.py install_path:=/isaac-sim gui:=\"${ISAAC_SIM_GUI}\" play_sim_on_start:=\"${PLAY_SIM_ON_START}\"' ENTER
    fi
  fi;
  sleep infinity"
```

**Launch sequence:**

1. Creates tmux session named `isaac`
2. If `AUTOLAUNCH=true`, launches Isaac Sim
3. Chooses standalone or ROS 2 mode based on `ISAAC_SIM_USE_STANDALONE`
4. Keeps container alive with `sleep infinity`

## Environment Variables

Key variables for Isaac Sim configuration:

| Variable | Description | Default |
|----------|-------------|---------|
| `AUTOLAUNCH` | Auto-start Isaac Sim on container launch | `false` |
| `ISAAC_SIM_SCENE` | Path to USD scene file | (set in `.env`) |
| `ISAAC_SIM_GUI` | Launch with GUI (`true`/`false`) | `true` |
| `PLAY_SIM_ON_START` | Auto-play simulation when GUI opens | `true` |
| `ISAAC_SIM_USE_STANDALONE` | Use standalone Python launch | `false` |
| `ISAAC_SIM_SCRIPT_NAME` | Standalone script filename | - |

**Example overrides:**

```bash
# Launch without GUI (headless)
ISAAC_SIM_GUI=false airstack up isaac-sim

# Don't auto-play simulation
PLAY_SIM_ON_START=false airstack up isaac-sim

# Launch with standalone script
ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME=custom_scene.py airstack up isaac-sim
```

## Networking

**Network configuration:**
- **Network:** `airstack_network` (172.31.0.0/24)
- **Fixed IP:** 172.31.0.200
- **Purpose:** Communicate with robot containers via ROS 2 DDS

**Why fixed IP?** Prevents conflicts with other Docker networks on the host.

## GPU Access

Isaac Sim requires NVIDIA GPU access:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

**Requirements:**

- NVIDIA GPU (RTX 3070+ recommended)
- NVIDIA Container Toolkit installed
- Host GPU drivers compatible with container

**Verify GPU access:**

```bash
# Inside Isaac Sim container
nvidia-smi
```

## Volume Mounts

Isaac Sim container mounts several directories:

### Display (X11)

```yaml
- $HOME/.Xauthority:/isaac-sim/.Xauthority
- /tmp/.X11-unix:/tmp/.X11-unix
```

Enables GUI display on host.

### Isaac Sim Cache & Data

```yaml
- $HOME/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw
- $HOME/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw
- $HOME/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw
- $HOME/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw
- $HOME/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw
- $HOME/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw
```

**Purpose:**

- Persist Isaac Sim settings and cache
- Improves startup performance
- Stores downloaded assets

### Pegasus Extension

```yaml
- ../extensions/PegasusSimulator/extensions/pegasus.simulator:/isaac-sim/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/:rw
```

Mounts the Pegasus multi-rotor simulator extension.

### AirStack Code

```yaml
- ../../..:/isaac-sim/AirStack:rw
```

Mounts entire AirStack repository for access to scenes, scripts, and launch files.

### Configuration Files

```yaml
- ./omniverse.toml:/isaac-sim/.nvidia-omniverse/config/omniverse.toml:rw
- ./user.config.json:/isaac-sim/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/user.config.json:rw
```

**user.config.json:** Enables Pegasus extension and other custom settings.

## Omniverse Credentials

Isaac Sim requires NVIDIA Omniverse credentials.

### Setup

1. **Create credentials file:**
   ```bash
   cp simulation/isaac-sim/docker/omni_pass_TEMPLATE.env simulation/isaac-sim/docker/omni_pass.env
   ```

2. **Edit with your credentials:**
   ```bash
   # omni_pass.env
   OMNI_USER=your_username
   OMNI_PASS=your_password
   ```

3. **File is git-ignored** (don't commit credentials!)

### Getting Credentials

1. Create account at [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Use your NVIDIA account credentials
3. Required for downloading assets and extensions

## Accessing Isaac Sim

### Via GUI (Default)

If `DISPLAY` is configured:

```bash
airstack up isaac-sim
# Isaac Sim GUI opens on host display
```

### Via tmux Session

Connect to the container and attach to tmux:

```bash
# Connect to container
airstack connect isaac-sim

# Attach to Isaac Sim tmux session
tmux a -t isaac
```

**Useful tmux commands:**

- `Ctrl-b d` - Detach from session
- `Ctrl-b [` - Scroll mode (arrow keys to scroll logs)
- `Ctrl-c` - Stop Isaac Sim

### Via Streaming (Headless)

For remote access, use Isaac Sim streaming:

**Native streaming:**
```bash
# Inside container
./runheadless.native.sh
```

Connect with [Omniverse Streaming Client](https://docs.omniverse.nvidia.com/streaming-client/latest/user-manual.html).

**WebRTC streaming:**
```bash
# Inside container
./runheadless.webrtc.sh
```

Access via web browser.

## Development Workflow

### Iterating on Scenes

1. **Edit scene files** on host (they're mounted):
   ```
   simulation/isaac-sim/scenes/my_scene.usd
   ```

2. **Reload in Isaac Sim:**
   - File → Open
   - Or restart container with new scene

3. **Changes persist** (files on host)

### Testing Standalone Scripts

1. **Create script:**
   ```
   simulation/isaac-sim/launch_scripts/test_script.py
   ```

2. **Launch:**
   ```bash
   ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME=test_script.py airstack up isaac-sim
   ```

3. **View output:**
   ```bash
   airstack logs isaac-sim
   ```

### Debugging

**Enable debug logging:**

Edit `user.config.json` to increase log verbosity.

**View logs:**

```bash
# Container logs
airstack logs isaac-sim

# Isaac Sim logs
ls $HOME/docker/isaac-sim/logs/
```

## Image Management

### Pulling Pre-built Images

```bash
# Login to AirLab registry
docker login airlab-docker.andrew.cmu.edu

# Pull Isaac Sim image
docker compose -f simulation/isaac-sim/docker/docker-compose.yaml pull
```

### Building from Source

```bash
# Build Isaac Sim image
docker compose -f simulation/isaac-sim/docker/docker-compose.yaml build

# Build with no cache
docker compose -f simulation/isaac-sim/docker/docker-compose.yaml build --no-cache
```

**Note:** Isaac Sim base image is large (~20GB). Initial build takes time.

## Troubleshooting

**Isaac Sim won't start:**

- Check GPU: `nvidia-smi` on host
- Verify NVIDIA Container Toolkit: `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`
- Check disk space: `df -h` (need 25GB+ free)
- Review logs: `airstack logs isaac-sim`

**GUI not displaying:**

- Check `DISPLAY`: `echo $DISPLAY` (should be `:0` or `:1`)
- Allow X11: `xhost +local:docker`
- Verify X11 socket mounted: Check docker-compose volumes

**ROS 2 topics not visible:**

- Verify containers on same network: `docker network inspect airstack_network`
- Check ROS 2 domain IDs match
- Inspect DDS: `fastdds.xml` configuration
- Test connection: `ros2 topic list` in Isaac Sim container

**Performance issues:**

- Reduce scene complexity
- Lower physics timestep
- Disable raytracing (Settings → Rendering)
- Close other GPU-intensive applications

**Omniverse login fails:**

- Verify credentials in `omni_pass.env`
- Check network connectivity
- Ensure NVIDIA account is active

**Extension not loading:**

- Verify `user.config.json` enables extension
- Check extension path in volume mounts
- Review Isaac Sim logs for extension errors

## Advanced Configuration

### Custom Extensions

Add custom Isaac Sim extensions:

1. Place extension in `simulation/isaac-sim/extensions/`
2. Mount in docker-compose.yaml
3. Enable in `user.config.json`

### Multi-GPU Setup

For multi-GPU systems:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          device_ids: ['0', '1']  # Use specific GPUs
          capabilities: [gpu]
```

### Persistent Nucleus Server

For team collaboration, set up persistent Nucleus server:

Edit `omniverse.toml` with your Nucleus server URL.

## See Also

- [Isaac Sim Overview](index.md) - Isaac Sim capabilities and features
- [Pegasus Scene Setup](pegasus_scene_setup.md) - Creating custom scenes
- [Simulation Overview](../index.md) - Main simulation documentation
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) - General Docker operations
