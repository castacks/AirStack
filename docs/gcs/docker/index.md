# GCS Docker Configuration

The Ground Control Station is containerized using Docker Compose, following the same patterns as the robot and simulation components.

## File Structure

```
gcs/docker/
├── gcs-base-docker-compose.yaml      # Shared base service definition
├── docker-compose.yaml               # GCS service definition  
├── Dockerfile.gcs                    # Image definition
├── .bashrc                           # Bash config mounted into container
├── Foxglove/                         # Foxglove Studio configuration
│   ├── layouts/                      # Pre-configured dashboard layouts
│   └── extensions/                   # Custom visualization extensions
└── resources/                        # GCS assets and configuration files
```

## Service Architecture

The GCS service extends a base configuration (`gcs_base`) defined in `gcs-base-docker-compose.yaml`.

**Key components:**

| Component | Purpose |
|-----------|---------|
| **RQT GCS Panel** | Main monitoring and control GUI |
| **Foxglove Studio** | Advanced visualization (web-based) |
| **ROS 2 Workspace** | GCS-specific ROS 2 nodes and tools |
| **TAK Integration** | Optional TAK protocol bridge |

## Base Service (`gcs_base`)

The `gcs_base` service provides defaults shared across GCS deployments:

| Aspect | Details |
|--------|---------|
| **Display** | `DISPLAY`, X11 socket mounted for GUI applications |
| **Source tree** | `AirStack/` repo bind-mounted at `/root/AirStack` |
| **ROS workspace** | `common/ros_packages` mounted into workspace `src/common` |
| **Shell config** | `.bashrc` bind-mounted for consistent developer experience |
| **Bags** | `gcs/bags/` mounted at `/bags` for mission recording |
| **Network** | Connected to `airstack_network` bridge for robot communication |

## Launch Configuration

The GCS container is launched with:

```bash
airstack up gcs
```

**What happens:**

1. Docker Compose starts the GCS container from `gcs/docker/docker-compose.yaml`
2. The `command:` attribute launches GCS ROS 2 nodes
3. RQT GUI opens automatically (if `DISPLAY` configured)
4. Foxglove Studio starts on port 8765

**Container command:**

```yaml
command: >
  bash -c "
  service ssh restart;
  tmux new -d -s gcs_session;
  if [ $$AUTOLAUNCH == 'true' ]; then
    tmux send-keys -t gcs_session 'bws && sws && ros2 launch gcs_bringup gcs.launch.xml' ENTER;
  fi;
  sleep infinity"
```

## Environment Variables

Key variables for GCS configuration:

| Variable | Description | Default |
|----------|-------------|---------|
| `DOCKER_IMAGE_TAG` | Image version tag | Set in `.env` |
| `DOCKER_IMAGE_BUILD_MODE` | Build mode (release/dev) | `dev` |
| `AUTOLAUNCH` | Auto-start GCS on container launch | `true` |
| `ROS_DOMAIN_ID` | ROS 2 domain (0 for GCS) | `0` |
| `FOXGLOVE_PORT` | Foxglove Studio web port | `8765` |

**Example overrides:**

```bash
# Don't auto-launch (for development)
AUTOLAUNCH=false airstack up gcs

# Custom Foxglove port
FOXGLOVE_PORT=9000 airstack up gcs
```

## Networking

The GCS container connects to the `airstack_network` bridge network, allowing communication with:

- Robot containers (via ROS 2 DDS)
- Isaac Sim container (for sensor visualization)
- External TAK servers (if configured)

**Network configuration:**

- **Network:** `airstack_network` (172.31.0.0/24)
- **ROS_DOMAIN_ID:** 0 (GCS shares domain 0)
- **Ports:** 8765 (Foxglove), 22 (SSH)

## Accessing the GCS

### Via X11 Forwarding

The GCS GUI (RQT) displays on the host's X11 server:

```bash
# Ensure DISPLAY is set on host
echo $DISPLAY

# Start GCS
airstack up gcs

# RQT opens automatically
```

### Via SSH

Connect to the GCS container via SSH:

```bash
# Get container IP
docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' airstack-gcs-1

# SSH into container
ssh root@<container_ip>
# Password: airstack
```

### Via Foxglove Web UI

Access Foxglove Studio in your web browser:

```
http://localhost:8765
```

Pre-configured layouts are available in `gcs/docker/Foxglove/layouts/`.

## Foxglove Configuration

### Custom Layouts

Create and save custom Foxglove layouts:

1. Open Foxglove at `http://localhost:8765`
2. Design your layout with panels for:
   - 3D visualization
   - Robot state displays
   - Sensor feeds
   - Custom plots
3. Export layout to `gcs/docker/Foxglove/layouts/`
4. Layout persists across container restarts

### Custom Extensions

Add custom Foxglove extensions:

```
gcs/docker/Foxglove/extensions/
└── your_extension/
    ├── package.json
    └── src/
```

Extensions are automatically loaded when Foxglove starts.

## Development Workflow

### Interactive Development

```bash
# Start GCS without auto-launch
AUTOLAUNCH=false airstack up gcs

# Connect to container
airstack connect gcs

# Inside container
cd /root/AirStack/gcs/ros_ws
bws --packages-select rqt_gcs
sws
ros2 launch gcs_bringup gcs.launch.xml
```

### Building GCS Packages

```bash
# Build from host
docker exec airstack-gcs-1 bash -c "bws --packages-select rqt_gcs"

# Build with debug symbols
docker exec airstack-gcs-1 bash -c "bws --packages-select rqt_gcs --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
```

### Viewing Logs

```bash
# View GCS container logs
airstack logs gcs

# Or
docker logs airstack-gcs-1

# Follow logs in real-time
docker logs -f airstack-gcs-1
```

## Image Management

### Pulling Pre-built Images

```bash
# Login to AirLab registry
docker login airlab-docker.andrew.cmu.edu

# Pull GCS image
docker compose pull gcs
```

### Building from Source

```bash
# Build GCS image
docker compose build gcs

# Build with no cache
docker compose build --no-cache gcs
```

## Troubleshooting

**RQT GUI won't display:**

- Check `DISPLAY` environment variable: `echo $DISPLAY`
- Allow X11 connections: `xhost +local:docker`
- Verify X11 socket mounted: Check docker-compose volumes

**Foxglove won't connect:**

- Verify Foxglove port not in use: `sudo lsof -i :8765`
- Check container networking: `docker inspect airstack-gcs-1`
- Access via container IP if localhost fails

**ROS 2 communication issues:**

- Verify `ROS_DOMAIN_ID=0` in GCS container
- Check all containers on `airstack_network`
- Inspect DDS discovery: `ros2 topic list` in GCS container

**SSH connection refused:**

- Wait for SSH service to start (few seconds after container launch)
- Check SSH port mapping: `docker port airstack-gcs-1`
- Verify SSH daemon running: `docker exec airstack-gcs-1 service ssh status`

## See Also

- [GCS Overview](../index.md) - Main GCS documentation
- [User Interface Guide](../usage/user_interface.md) - Using the RQT GCS panel
- [Docker Workflow](../../development/beginner/airstack-cli/docker_usage.md) - General Docker operations
- [Robot Docker Configuration](../../robot/docker/index.md) - Robot container setup
