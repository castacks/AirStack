# Isaac Sim Container Workflows

Working procedures for the Isaac Sim container: launching it in different modes, accessing it, iterating on scenes and scripts, managing images, and troubleshooting.

**Reference for this container:** [Isaac Sim Docker Configuration](docker.md) — file structure, service architecture, launch configuration, environment variables, networking, GPU access, and volume mounts.

## Launch Modes

Isaac Sim supports multiple launch modes:

### 1. Standard Launch (ROS 2 Integration)

Default mode with ROS 2 bridge:

```bash
airstack up isaac-sim
```

**What happens:**

- Launches Isaac Sim with ROS 2 bridge
- Runs the launch script named by `ISAAC_SIM_SCRIPT_NAME` (or, with `ISAAC_SIM_USE_STANDALONE=false`, opens the USD in `ISAAC_SIM_GUI`)
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

### 3. GUI-Only Mode (`isaac-sim-gui`)

The `isaac-sim-gui` compose service opens Isaac Sim's **full GUI editor**
(`runapp.sh` — no Pegasus launch script, no drones) for USD/scene editing on
any asset:

```bash
airstack up --profile isaac-sim-gui isaac-sim-gui
```

(The service sits behind its own `isaac-sim-gui` profile, so both the
`--profile` flag and the service name are needed; it does not conflict with
the one-active-simulator rule.)

**Use cases:**

- Authoring/editing USD scenes and assets (see [Pegasus Scene Setup](pegasus_scene_setup.md))
- Inspecting assets without bringing up the robot stack

**Not for flying:** the service is deliberately **not** on `airstack_network`
(`networks: !reset null` in `simulation/isaac-sim/docker/docker-compose.yaml`),
so no DDS traffic reaches the robot containers and no PX4 is launched. To fly,
use `airstack up --sim isaac`.

### Example overrides

```bash
# Launch without a window (headless)
airstack up --sim isaac --headless

# Don't auto-play simulation
airstack up isaac-sim --no-play

# Launch with standalone script
ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME=custom_scene.py airstack up isaac-sim
```

See the [environment variables table](docker.md#environment-variables) for the full list of configuration variables.

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
   simulation/isaac-sim/assets/scenes/my_scene.usd
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
   airstack logs isaac-sim      # tmux pane output is mirrored to docker logs
   airstack connect isaac-sim   # attach to the tmux session interactively
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
# Pull Isaac Sim image (the AirLab registry is public — no login needed)
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

**`rclpy` / `_rclpy_pybind11` warnings when starting Kit with `python.sh`:**

!!! note "Why this happens: Kit-Python vs system Python ABI"
    Jazzy’s `setup.bash` puts **Python 3.12** ROS packages on `PYTHONPATH`. Isaac’s `python.sh` uses **Kit Python (~3.10)**. Importing system `rclpy` from the wrong interpreter causes ABI errors in the log (topics from Omnigraph may still work).

    Standalone launch uses `PYTHONPATH="$ISAAC_SIM_PYTHONPATH"` in the **tmux** command (`$$ISAAC_SIM_PYTHONPATH` in `docker-compose.yaml` so Compose does not treat it as a host variable). See container `.bashrc` and `docker-compose.yaml`: it drops `lib/python3.12/site-packages` and appends the bridge’s internal `rclpy` path.

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

## See Also

- [Isaac Sim Docker Configuration](docker.md) - Container reference: files, services, environment variables, mounts
- [Isaac Sim Overview](index.md) - Isaac Sim capabilities and features
- [Pegasus Scene Setup](pegasus_scene_setup.md) - Creating custom scenes
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) - General Docker operations
