# Isaac Sim Docker Configuration

Isaac Sim runs in a Docker container with NVIDIA GPU support and full integration with the AirStack ecosystem. This page is the **reference** for the container: file structure, service architecture, launch configuration, environment variables, networking, GPU access, and volume mounts.

**Working procedures:** [Isaac Sim Container Workflows](container_workflows.md) — launch modes, credentials setup, accessing Isaac Sim, development workflow, image management, and troubleshooting.

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

## Launch Configuration

The container command in `simulation/isaac-sim/docker/docker-compose.yaml` (excerpt — see the compose file for the full command):

```yaml
command: >
  bash -c "
  tmux new -d -s isaac;
  if [ $$AUTOLAUNCH = 'true' ]; then
    ...   # standalone: python.sh + ISAAC_SIM_SCRIPT_NAME
          # otherwise: ros2 launch isaacsim run_isaacsim.launch.py
  fi;
  sleep infinity"
```

**Launch sequence:**

1. Creates tmux session named `isaac`
2. If `AUTOLAUNCH=true`, launches Isaac Sim
3. Chooses standalone or ROS 2 mode based on `ISAAC_SIM_USE_STANDALONE`
4. Keeps container alive with `sleep infinity`

For the launch mode recipes (standard, standalone script, GUI-only editor), see [Container Workflows → Launch Modes](container_workflows.md#launch-modes).

## Environment Variables

Key variables for Isaac Sim configuration:

| Variable | Description | Default (`.env`) |
|----------|-------------|---------|
| `AUTOLAUNCH` | Auto-start Isaac Sim on container launch | `true` |
| `ISAAC_SIM_USE_STANDALONE` | `true`: run `ISAAC_SIM_SCRIPT_NAME`; `false`: open the `ISAAC_SIM_GUI` USD | `true` |
| `ISAAC_SIM_SCRIPT_NAME` | Standalone launch script in `simulation/isaac-sim/launch_scripts/` | `example_one_px4_pegasus_launch_script.py` |
| `ISAAC_SIM_GUI` | Path to a USD scene file (used only when `ISAAC_SIM_USE_STANDALONE=false`) | `simulation/isaac-sim/assets/scenes/simple_pegasus.scene.usd` |
| `PLAY_SIM_ON_START` | Auto-play simulation on start (`airstack up --play/--no-play`) | `true` |
| `ISAAC_SIM_HEADLESS` | Run without a window (`airstack up --headless`) | unset (`false`) |
| `PX4_PHYSICS_HZ` | Physics step rate for PX4 SITL — also sets PX4 `IMU_INTEG_RATE` | `100` |
| `PX4_RENDERING_HZ` | Rendering frame rate for PX4 profiles (independent of physics) | `30` |
| `ARDUPILOT_PHYSICS_HZ` | Physics step rate for ArduPilot SITL | `800` |
| `ARDUPILOT_RENDERING_HZ` | Rendering frame rate for ArduPilot profiles | `120` |

`PX4_PHYSICS_HZ` and `PX4_RENDERING_HZ` default to 100/30 in the isaac-sim compose file (the Pegasus code default is 250 Hz physics). AirStack runs PX4 at **100 Hz** for near-real-time performance. See [Pegasus Scene Setup → Physics Rate](pegasus_scene_setup.md) for valid values and the full configuration flow.

For example command-line overrides of these variables, see [Container Workflows → Launch Modes](container_workflows.md#launch-modes).

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

Mounts the Pegasus multi-rotor simulator extension. Custom Isaac Sim extensions follow the same pattern: place the extension in `simulation/isaac-sim/extensions/`, mount it in docker-compose.yaml, and enable it in `user.config.json`.

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

**omniverse.toml:** Omniverse settings. For team collaboration with a persistent Nucleus server, edit it with your Nucleus server URL.

## See Also

- [Isaac Sim Container Workflows](container_workflows.md) - Working procedures: launch modes, access, development, troubleshooting
- [Isaac Sim Overview](index.md) - Isaac Sim capabilities and features
- [Pegasus Scene Setup](pegasus_scene_setup.md) - Creating custom scenes
- [Simulation Overview](../index.md) - Main simulation documentation
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) - General Docker operations
