# Simulation

AirStack provides high-fidelity simulation environments for developing and testing autonomous systems before deploying to hardware. Simulation enables rapid iteration, safe testing of edge cases, and multi-robot scenarios.

## Simulation Platforms

### NVIDIA Isaac Sim (Primary)

Isaac Sim is our primary simulation platform, offering:

- **Photorealistic rendering** with ray-traced graphics
- **Accurate physics simulation** via NVIDIA PhysX
- **ROS 2 integration** through Pegasus extension
- **Multi-robot support** with independent namespaces
- **Sensor simulation** (cameras, depth, IMU, GPS, LiDAR)
- **Custom scene creation** with USD format

**Getting Started**:

- [Isaac Sim Overview](isaac_sim/index.md)
- [Scene Setup Guide](isaac_sim/scene_setup.md)
- [Pegasus Scene Setup](isaac_sim/pegasus_scene_setup.md)
- [Ascent SITL Extension](isaac_sim/ascent_sitl_extension.md)

### Gazebo (Future)

We plan to support Gazebo for lighter-weight simulation needs.

## Docker Networking

All simulation components run in Docker containers on a shared network, enabling seamless communication between:

- Isaac Sim container
- Robot autonomy stack containers (multiple robots supported)
- Ground Control Station container

**Learn More**: [Docker Networking Guide](docker_network.md)

## Common Workflows

### Single Robot Simulation

1. Launch AirStack with default configuration:
   ```bash
   airstack up
   ```
2. Isaac Sim starts automatically with the configured scene
3. Robot autonomy stack connects and begins operation
4. Control via RQT GUI or Ground Control Station

See: [Getting Started Tutorial](../getting_started.md)

### Multi-Robot Simulation

1. Configure multiple robots in docker-compose
2. Launch with multi-robot support:
   ```bash
   airstack up
   ```
3. Each robot gets independent ROS 2 namespace
4. Coordinate via ground control station

See: [Multi-Robot Simulation Tutorial](../tutorials/multi_robot_simulation.md)

### Custom Scenes

Create custom Isaac Sim scenes with:

- Custom environments (buildings, forests, urban)
- Multiple robots
- Specific sensor configurations
- Dynamic obstacles

See: [Scene Setup Guide](isaac_sim/scene_setup.md) | [Pegasus Setup](isaac_sim/pegasus_scene_setup.md)

## Configuration

### Environment Variables

Simulation behavior is controlled via `.env` file:

- `ISAAC_SIM_SCENE` - Path to USD scene file
- `AUTOLAUNCH` - Auto-start simulation on container launch
- `ROS_DOMAIN_ID` - ROS 2 domain for each robot

### Scene Files

Pre-built scenes are located in `scenes/` directory:
- `two_drone_fire_new.usd` - Fire academy scenario
- `two_drone_RetroNeighborhood.usd` - Urban neighborhood

## Exporting Scenes

Import environments from Unreal Engine or other tools:

- [Export Stages from Unreal](isaac_sim/export_stages_from_unreal.md)

## Troubleshooting

**Isaac Sim won't start**:
- Check GPU requirements (RTX 3070+ recommended)
- Verify NVIDIA Container Toolkit installation
- Check disk space (25GB+ free required)

**ROS 2 communication issues**:
- Verify all containers on same Docker network
- Check ROS_DOMAIN_ID settings
- Review [Docker Networking](docker_network.md)

**Performance issues**:
- Reduce scene complexity
- Lower rendering quality in Isaac Sim settings
- Close unnecessary applications

## Next Steps

- **New Users**: Complete [Getting Started](../getting_started.md)
- **Multi-Robot**: Try [Multi-Robot Tutorial](../tutorials/multi_robot_simulation.md)
- **Custom Scenes**: Follow [Scene Setup](isaac_sim/scene_setup.md)
- **Development**: See [Developer Guide](../development/index.md)