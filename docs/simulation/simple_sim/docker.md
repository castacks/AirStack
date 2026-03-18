# Simple Sim Docker Configuration

Simple Sim runs in a lightweight Docker container optimized for quick startup and low resource usage.

## File Structure

```
simulation/simple-sim/docker/
├── docker-compose.yaml           # Service definition
├── Dockerfile.sim                # Image definition
├── bashrc                        # Bash configuration
└── inputrc                       # Input configuration
```

## Service Architecture

The Simple Sim service is defined in `simulation/simple-sim/docker/docker-compose.yaml`.

**Key components:**

| Component | Purpose |
|-----------|---------|
| **Simple Sim Core** | Lightweight 2D/3D simulator |
| **ROS 2 Native** | Direct ROS 2 topic integration |
| **Basic Physics** | Simplified flight dynamics |
| **Minimal Dependencies** | Fast startup, low overhead |

## Launch Configuration

The container command in docker-compose.yaml:

```yaml
command: >
  bash -c "ssh service restart;
  tmux new -d -s sim
  && tmux send-keys -t sim
  'cd /models && ./download.sh && cd ~/ros_ws/ && colcon build --symlink-install && source install/setup.bash && ROS_DOMAIN_ID=1 ros2 launch sim sim.launch.xml' ENTER
  && sleep infinity"
```

**Launch sequence:**

1. Restarts SSH service (for remote access)
2. Creates tmux session named `sim`
3. Downloads required models
4. Builds Simple Sim ROS workspace
5. Launches simulator with `ROS_DOMAIN_ID=1`
6. Keeps container alive

## Profiles

Simple Sim uses the `simple` profile:

```bash
# Launch Simple Sim standalone
airstack up --profile simple simple-sim

# Launch with robot (uses simple profile for robot too)
airstack up --profile simple simple-robot
```

**Profile configuration:**

- Activates Simple Sim service
- Configures robot for simple sim mode (if launched together)

## Networking

**Network configuration:**
- **Network:** `airstack_network` (172.31.0.0/24)
- **Fixed IP:** 172.31.0.200 (same as Isaac Sim - mutually exclusive)
- **ROS_DOMAIN_ID:** 1 (matches robot containers)

**Why same IP as Isaac Sim?** Simple Sim and Isaac Sim are mutually exclusive - only one runs at a time.

## GPU Access

Simple Sim supports GPU but doesn't require high-end hardware:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

**GPU usage:**

- **Optional** - Works with integrated graphics
- **Improves** visualization performance if available
- **Not required** for algorithm testing

**Test without GPU:**
```bash
# Works even without dedicated GPU
airstack up --profile simple simple-robot
```

## Volume Mounts

Simple Sim mounts minimal volumes for fast startup:

### Display (X11)

```yaml
- $HOME/.Xauthority:/root/.Xauthority
- /tmp/.X11-unix:/tmp/.X11-unix
```

Enables GUI visualization (optional).

### Configuration

```yaml
- ./bashrc:/root/.bashrc:rw
- /var/run/docker.sock:/var/run/docker.sock
```

**Docker socket:** Allows container to query its own name for `ROBOT_NAME`.

### Models

```yaml
- ../models:/models/:rw
```

3D models for visualization. Downloaded on first launch via `download.sh`.

### ROS Workspace

```yaml
- ../ros_ws:/root/ros_ws:rw
```

Simple Sim ROS 2 packages. Edit on host, rebuild in container.

## Environment Variables

Simple Sim uses minimal environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| `DISPLAY` | X11 display for GUI | (from host) |
| `ROS_DOMAIN_ID` | ROS 2 domain (hardcoded in command) | `1` |
| `NVIDIA_DRIVER_CAPABILITIES` | GPU capabilities | `all` |

**Note:** Most configuration is in ROS 2 launch files, not environment variables.

## Accessing Simple Sim

### Via tmux Session

Connect to the container and attach to tmux:

```bash
# Connect to container
airstack connect simple-sim

# Attach to Simple Sim tmux session
tmux a -t sim
```

**View logs:**
```bash
# Detach from tmux first (Ctrl-b d)
# Then view logs
airstack logs simple-sim
```

### Via RViz Visualization

If running with GUI, launch RViz to visualize:

```bash
# In robot container
docker exec airstack-simple-robot-1 bash -c "rviz2"

# Add visualization topics
```

## Development Workflow

### Modifying Simple Sim

1. **Edit simulator code** on host:
   ```
   simulation/simple-sim/ros_ws/src/sim/
   ```

2. **Rebuild in container:**
   ```bash
   docker exec airstack-simple-sim-1 bash -c "cd ~/ros_ws && colcon build --packages-select sim"
   ```

3. **Restart simulator:**
   ```bash
   docker exec airstack-simple-sim-1 bash -c "tmux send-keys -t sim C-c 'source install/setup.bash && ros2 launch sim sim.launch.xml' ENTER"
   ```

### Adding Models

1. **Place models** in `simulation/simple-sim/models/`

2. **Update download script:**
   ```bash
   # Edit models/download.sh
   ```

3. **Restart container** to download new models

### Testing with Robot Stack

**Full integration test:**

```bash
# Launch both Simple Sim and robot
airstack up --profile simple simple-robot

# Robot autonomy connects to Simple Sim automatically
```

## Image Management

### Pulling Pre-built Images

```bash
# Login to AirLab registry
docker login airlab-docker.andrew.cmu.edu

# Pull Simple Sim image
docker compose -f simulation/simple-sim/docker/docker-compose.yaml pull
```

### Building from Source

```bash
# Build Simple Sim image (fast - minimal dependencies)
docker compose -f simulation/simple-sim/docker/docker-compose.yaml build

# Build with no cache
docker compose -f simulation/simple-sim/docker/docker-compose.yaml build --no-cache
```

**Build time:** ~2-5 minutes (much faster than Isaac Sim)

## Troubleshooting

**Simple Sim won't start:**

- Check logs: `airstack logs simple-sim`
- Verify network available: `docker network ls | grep airstack`
- Check ROS 2 domain: Should be `ROS_DOMAIN_ID=1`

**No visualization:**

- Verify `DISPLAY` set: `echo $DISPLAY`
- Allow X11: `xhost +local:docker`
- Check X11 socket mounted in docker-compose

**ROS 2 topics not visible from robot:**

- Verify both on `airstack_network`: `docker network inspect airstack_network`
- Check `ROS_DOMAIN_ID=1` in both containers
- Test directly: `docker exec airstack-simple-sim-1 bash -c "ros2 topic list"`

**Models not downloading:**

- Check network connectivity
- Verify `/models` writable in container
- Run download manually: `docker exec airstack-simple-sim-1 bash -c "cd /models && ./download.sh"`

**Build fails:**

- Check ROS 2 dependencies in workspace
- Verify `simulation/simple-sim/ros_ws/src/sim/package.xml`
- Review colcon build output

## Performance Optimization

Simple Sim is already lightweight, but you can optimize further:

### Headless Mode

Disable GUI for faster execution:

```bash
# Don't mount display
# Edit docker-compose.yaml and comment out X11 volumes
```

### Reduce Visualization

In launch files, disable unnecessary visualization topics:

```xml
<!-- simulation/simple-sim/ros_ws/src/sim/launch/sim.launch.xml -->
<param name="enable_visualization" value="false"/>
```

### Increase Physics Rate

For faster simulation:

```yaml
# In sim config
physics_rate_hz: 100  # Increase from default
```

**Trade-off:** Higher CPU usage but faster-than-real-time simulation.

## Comparison with Isaac Sim Docker

| Aspect | Isaac Sim | Simple Sim |
|--------|-----------|------------|
| **Image size** | ~20GB | ~2GB |
| **Startup time** | 30-60 seconds | 5-10 seconds |
| **Build time** | 20-30 minutes | 2-5 minutes |
| **GPU requirement** | Required | Optional |
| **Volume mounts** | Many (cache, config, extensions) | Minimal |
| **Dependencies** | NVIDIA Omniverse stack | ROS 2 + basic physics |

## Advanced Configuration

### Custom Physics

Implement custom dynamics in `simulation/simple-sim/ros_ws/src/sim/`:

```python
# Example: Add wind effects
class DroneSimulator:
    def apply_wind(self, wind_vector):
        # Custom physics
        pass
```

### Multi-Robot Testing

Launch multiple Simple Sim instances:

```bash
# Requires docker-compose modifications
# Each instance needs unique IP and ROS_DOMAIN_ID
```

**Note:** Currently configured for single simulator instance.

### CI/CD Integration

Example GitHub Actions workflow:

```yaml
- name: Test with Simple Sim
  run: |
    airstack up --profile simple simple-robot &
    sleep 10  # Wait for startup
    docker exec airstack-simple-robot-1 bash -c "colcon test"
    airstack down
```

## See Also

- [Simple Sim Overview](index.md) - Features and use cases
- [Isaac Sim Docker](../isaac_sim/docker.md) - High-fidelity simulator
- [Simulation Overview](../index.md) - Main simulation documentation
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) - General Docker operations
