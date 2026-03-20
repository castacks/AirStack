# Simple Sim

Simple Sim is a lightweight 2D/3D simulator for basic testing and development when full Isaac Sim fidelity isn't needed.

## Overview

Simple Sim provides a faster, more resource-efficient alternative to Isaac Sim for:

- **Quick algorithm prototyping** - Faster iteration cycles
- **CI/CD testing** - Lightweight enough for automated testing pipelines
- **Lower hardware requirements** - Works on systems without high-end GPUs
- **Basic flight dynamics** - Sufficient for many planning and control algorithms

**Trade-offs:**

- ✅ Faster startup and execution
- ✅ Lower computational requirements
- ✅ Simpler scene setup
- ❌ Less realistic physics
- ❌ Limited sensor simulation
- ❌ Basic graphics (no photorealism)

## Use Cases

### Algorithm Development

Test planning and control algorithms without full simulation overhead:

```bash
airstack up simple-sim robot
```

Your ROS 2 autonomy stack connects to Simple Sim just like Isaac Sim.

### Continuous Integration

Run automated tests in CI pipelines:

```bash
# In CI script
airstack up --profile simple simple-robot
# Run tests...
airstack down
```

### Resource-Constrained Environments

Develop on laptops or systems without high-end GPUs:

- Works with integrated graphics
- Lower RAM requirements (~4GB vs 16GB+)
- Faster container startup

## Architecture

Simple Sim is built on:

- **ROS 2 native** - Direct ROS 2 integration
- **Lightweight physics** - Basic dynamics simulation
- **2D/3D visualization** - RViz-compatible
- **Configurable dynamics** - Tune flight characteristics

### Comparison with Isaac Sim

| Feature | Isaac Sim | Simple Sim |
|---------|-----------|------------|
| **Graphics** | Photorealistic raytracing | Basic 3D rendering |
| **Physics** | NVIDIA PhysX (high-fidelity) | Simplified dynamics |
| **Sensors** | Full suite (cameras, LiDAR, etc.) | Basic sensors |
| **Startup time** | 30-60 seconds | 5-10 seconds |
| **GPU requirements** | RTX 3070+ | Integrated graphics OK |
| **RAM requirements** | 16GB+ | 4GB+ |
| **Scene authoring** | USD format (Omniverse) | Configuration files |
| **Multi-robot** | Full support | Full support |

## Quick Start

### Launch Simple Sim

```bash
# Launch Simple Sim only
airstack up --profile simple simple-sim

# Launch with robot stack
airstack up --profile simple simple-robot
```

### Verify Connection

```bash
# Check ROS 2 topics
docker exec airstack-simple-robot-1 bash -c "ros2 topic list | grep simple"
```

## Configuration

Simple Sim configuration is in `simulation/simple-sim/ros_ws/`:

```
simulation/simple-sim/
├── docker/                    # Docker configuration
│   ├── docker-compose.yaml
│   └── Dockerfile.sim
├── models/                    # 3D models
│   └── download.sh            # Model download script
└── ros_ws/                    # Simple Sim ROS workspace
    └── src/
        └── sim/               # Simulator package
            ├── config/        # Configuration files
            └── launch/        # Launch files
```

### Customizing Flight Dynamics

Edit configuration in `simulation/simple-sim/ros_ws/src/sim/config/`:

```yaml
# Example: drone dynamics parameters
mass: 1.5  # kg
max_thrust: 20.0  # N
drag_coefficient: 0.1
```

## Development Workflow

### Testing Algorithm Changes

1. **Start Simple Sim:**
   ```bash
   airstack up --profile simple simple-robot
   ```

2. **Make changes** to autonomy code on host

3. **Rebuild in container:**
   ```bash
   docker exec airstack-simple-robot-1 bash -c "bws --packages-select my_planner"
   ```

4. **Test immediately** (faster than Isaac Sim restart)

### Transitioning to Isaac Sim

Once algorithms work in Simple Sim, test in Isaac Sim:

```bash
# Stop Simple Sim
airstack down

# Start Isaac Sim
airstack up robot isaac-sim
```

Code changes are minimal - same ROS 2 topics and interfaces.

## Limitations

**What Simple Sim can't do:**

- Photorealistic rendering
- Complex sensor simulation (cameras, LiDAR)
- Accurate aerodynamic effects
- Detailed collision physics
- Custom 3D environments (limited to basic models)

**When to use Isaac Sim instead:**

- Visual perception algorithm development
- Realistic sensor simulation needed
- Complex environment interactions
- Final validation before hardware deployment

## See Also

- [Docker Configuration](docker.md) - Simple Sim container setup
- [Isaac Sim](../isaac_sim/index.md) - High-fidelity simulation alternative
- [Simulation Overview](../index.md) - Main simulation documentation
