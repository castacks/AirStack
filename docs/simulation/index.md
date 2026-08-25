# Simulation

AirStack provides simulation environments for developing and testing autonomous systems before deploying to hardware. Simulation enables rapid iteration, safe testing of edge cases, and multi-robot scenarios.

Three simulators are supported because no single one covers every development need: **Isaac Sim** (primary) for high-fidelity rendering, physics, and full sensor suites; **Microsoft AirSim (legacy)** for native PX4-in-the-loop testing with pre-built Unreal Engine scenes and no Omniverse dependency; and **Simple Sim** for fast, lightweight iteration on planning and perception code without PX4 or a heavyweight GPU workload. Pick the lightest simulator that exercises what you're working on.

## Directory Structure

The simulation components are organized under `simulation/`:

```
simulation/
├── isaac-sim/
│   ├── docker/                    # Isaac Sim containerization
│   │   ├── docker-compose.yaml    # Main launch configuration
│   │   └── Dockerfile.isaac-ros   # Image definition
│   ├── assets/                    # Scenes, 3D models and props
│   ├── extensions/                # Custom Isaac Sim extensions
│   ├── launch_scripts/            # Python launch scripts
│   └── utils/                     # Shared helpers
├── ms-airsim/
│   ├── docker/                    # Microsoft AirSim (legacy) containerization
│   │   ├── docker-compose.yaml    # Launch configuration
│   │   └── Dockerfile             # Image definition
│   ├── config/                    # Microsoft AirSim (legacy) settings.json
│   └── ros_ws/                    # Depth bridge ROS package
└── simple-sim/
    ├── docker/                    # Simple simulator container
    │   └── docker-compose.yaml    # Launch configuration
    ├── models/                    # Lightweight simulation models
    └── ros_ws/                    # Simple sim ROS workspace
```

## Launch Structure

Simulation components are launched via Docker Compose. Each simulator has its own configuration:

- **Isaac Sim:** `simulation/isaac-sim/docker/docker-compose.yaml`
- **Microsoft AirSim (legacy):** `simulation/ms-airsim/docker/docker-compose.yaml`
- **Simple Sim:** `simulation/simple-sim/docker/docker-compose.yaml`

**Key launch points:**

- **Launch command:** `airstack up --sim isaac` (or `airstack up isaac-sim` to start only the sim service)
- **Main process:** The `command:` in docker-compose.yaml starts the simulator
- **Scene selection:** `airstack up --scene <shortname>` picks the environment for whichever simulator is active — see [Simulation Scenes](scenes.md). The standalone launch script named by `ISAAC_SIM_SCRIPT_NAME` (in `.env`) defines the drones and honors the selected scene; with `ISAAC_SIM_USE_STANDALONE=false`, `ISAAC_SIM_GUI` points at a USD file to open instead
- **Auto-play:** Controlled by `PLAY_SIM_ON_START` (or `airstack up --play`)

**Example:**
```bash
# Launch Isaac Sim with a custom launch script
ISAAC_SIM_SCRIPT_NAME=my_custom_scene.py airstack up --sim isaac

# Come up paused
airstack up --sim isaac --no-play
```

**Learn more:** [Docker Workflow](../development/beginner/airstack-cli/docker_usage.md)

## Simulation Platforms

### NVIDIA Isaac Sim (Primary)

Isaac Sim is our primary simulation platform, offering:

- **Photorealistic rendering** with ray-traced graphics
- **Accurate physics simulation** via NVIDIA PhysX
- **ROS 2 integration** through Pegasus extension
- **Multi-robot support** with independent namespaces
- **Sensor simulation** (cameras, depth, IMU, GPS, LiDAR)
- **Custom scene creation** with USD format

**Getting Started:**

- [Isaac Sim Overview](isaac_sim/index.md)
- [Pegasus Scene Setup](isaac_sim/pegasus_scene_setup.md)
- [Ascent SITL Extension](isaac_sim/ascent_sitl_extension.md)
- [Export from Unreal Engine](isaac_sim/export_stages_from_unreal.md)

### Microsoft AirSim (legacy) (Unreal Engine)

An open-source drone simulator built on Unreal Engine with native PX4 SITL integration.

**Use cases:**

- PX4-in-the-loop testing with photorealistic environments
- Depth-based obstacle avoidance testing (DROAN)
- Environments from the Unreal Engine ecosystem

**Launch:** `airstack up --sim airsim`

**Location:** `simulation/ms-airsim/`

### Simple Sim (Lightweight)

A lightweight kinematic simulator (single ROS 2 node, no PX4/MAVROS — it mocks the MAVROS interface directly) for fast iteration when full Isaac Sim fidelity isn't needed. Single robot only. See [Simple Sim](simple_sim/index.md).

**Use cases:**

- Quick algorithm prototyping (planning/control/stereo perception)
- Machines without an Isaac-class GPU or Omniverse credentials
- Faster iteration cycles

**Launch:** `airstack up --sim simple`

**Location:** `simulation/simple-sim/`

## Common Workflows

### Single Robot Simulation

1. **Launch the full stack:**
   ```bash
   airstack up
   ```

2. Isaac Sim starts with the configured scene
3. Robot autonomy stack connects and begins operation
4. Monitor via Ground Control Station

**See:** [Getting Started](../getting_started/index.md)

### Multi-Robot Simulation

1. **Launch multiple robots:**
   ```bash
   airstack up --sim isaac --robots 3
   ```
   (`--robots` sets `NUM_ROBOTS` **and** selects the multi-drone launch script.
   Plain `NUM_ROBOTS=3 airstack up` is rejected by preflight if
   `ISAAC_SIM_SCRIPT_NAME` is still the single-drone default, which spawns
   exactly one drone.)

2. Each robot gets an independent ROS 2 namespace and DDS domain
3. All robots visible in same Isaac Sim scene
4. Coordinate via ground control station

**Learn more:** [Docker Workflow](../development/beginner/airstack-cli/docker_usage.md#robot)

### Custom Scenes

Create custom Isaac Sim scenes with:

- Custom environments (buildings, forests, urban)
- Multiple robots
- Specific sensor configurations
- Dynamic obstacles

**See:** [Pegasus Scene Setup](isaac_sim/pegasus_scene_setup.md)

## Configuration

Key environment variables for simulation (set in `.env` or at runtime):

| Variable | Description | Default |
|----------|-------------|---------|
| `ISAAC_SIM_SCRIPT_NAME` | Standalone launch script (scene + drones) in `simulation/isaac-sim/launch_scripts/` | `example_one_px4_pegasus_launch_script.py` |
| `ISAAC_SIM_USE_STANDALONE` | `true`: run the launch script; `false`: open the USD in `ISAAC_SIM_GUI` | `true` |
| `ISAAC_SIM_GUI` | USD file to open when not using a standalone script | `simple_pegasus.scene.usd` |
| `PLAY_SIM_ON_START` | Auto-start simulation | `false` |
| `NUM_ROBOTS` | Number of robot containers (use `--robots` so the sim matches) | `1` |

**Example:**
```bash
# Custom launch script
ISAAC_SIM_SCRIPT_NAME=my_custom_scene.py airstack up --sim isaac

# Auto-play on start
airstack up --sim isaac --play
```

**Pre-built scenes:** Located in `simulation/isaac-sim/assets/scenes/` (e.g. `simple_pegasus.scene.usd`); standalone launch scripts in `simulation/isaac-sim/launch_scripts/` build scenes programmatically.

**Learn more:** [Docker Workflow](../development/beginner/airstack-cli/docker_usage.md#docker-compose-variable-overrides)

## Troubleshooting

**Isaac Sim won't start:**

- Check GPU requirements (RTX 3070+ recommended)
- Verify NVIDIA Container Toolkit installation
- Check disk space (25GB+ free required)

**ROS 2 communication issues:**

- Verify all containers on same Docker network (`docker network ls`)
- Check `ROS_DOMAIN_ID` settings in containers
- See [Docker Workflow](../development/beginner/airstack-cli/docker_usage.md)

**Performance issues:**

- Reduce scene complexity
- Lower rendering quality in Isaac Sim settings
- Close unnecessary applications
- Use simple-sim for lighter workloads

## Next Steps

- **[Getting Started](../getting_started/index.md)** - Complete setup and first simulation
- **[Isaac Sim Overview](isaac_sim/index.md)** - Learn Isaac Sim capabilities
- **[Pegasus Scene Setup](isaac_sim/pegasus_scene_setup.md)** - Create custom scenes
- **[Development Guide](../development/index.md)** - Develop autonomy algorithms