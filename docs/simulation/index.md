# Simulation

AirStack provides high-fidelity simulation environments for developing and testing autonomous systems before deploying to hardware. Simulation enables rapid iteration, safe testing of edge cases, and multi-robot scenarios.

## Directory Structure

The simulation components are organized under `simulation/`:

```
simulation/
├── isaac-sim/
│   ├── docker/                    # Isaac Sim containerization
│   │   ├── docker-compose.yaml    # Main launch configuration
│   │   └── Dockerfile.isaac-ros   # Image definition
│   ├── assets/                    # 3D models and props
│   ├── config/                    # Simulation configurations
│   ├── extensions/                # Custom Isaac Sim extensions
│   ├── launch_scripts/            # Python launch scripts
│   └── standalone_examples/       # Example scenes and scripts
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

- **Launch command:** `airstack up isaac-sim` or `airstack up simple-sim`
- **Main process:** The `command:` in docker-compose.yaml starts the simulator
- **Scene selection:** Set via `ISAAC_SIM_SCENE` environment variable (Isaac Sim)
- **Auto-launch:** Controlled by `PLAY_SIM_ON_START` variable

**Example:**
```bash
# Launch Isaac Sim with custom scene
ISAAC_SIM_SCENE=scenes/custom_scene.usd airstack up isaac-sim

# Launch simple simulator
airstack up simple-sim
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

**Launch:** Set `SIM_IP=172.31.0.201` in `.env`, then `docker compose --profile ms-airsim --profile desktop up`

**Location:** `simulation/ms-airsim/`

### Simple Sim (Lightweight)

A lightweight 2D/3D simulator for basic testing and development when full Isaac Sim fidelity isn't needed.

**Use cases:**

- Quick algorithm prototyping
- CI/CD testing
- Lower hardware requirements
- Faster iteration cycles

**Launch:** `airstack up simple-sim`

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
   NUM_ROBOTS=3 airstack up
   ```

2. Each robot gets independent ROS 2 namespace
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
| `ISAAC_SIM_SCENE` | Path to USD scene file | `simulation/isaac-sim/scenes/...` |
| `PLAY_SIM_ON_START` | Auto-start simulation | `true` |
| `NUM_ROBOTS` | Number of robots to spawn | `1` |

**Example:**
```bash
# Custom scene
ISAAC_SIM_SCENE=scenes/custom.usd airstack up isaac-sim

# Don't auto-play
PLAY_SIM_ON_START=false airstack up isaac-sim
```

**Pre-built scenes:** Located in `scenes/` directory

- `two_drone_fire_new.usd` - Fire academy scenario
- `two_drone_RetroNeighborhood.usd` - Urban neighborhood

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