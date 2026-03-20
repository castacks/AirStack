# Ground Control Station (GCS)

The Ground Control Station provides monitoring, control, and mission planning capabilities for AirStack robots. Operators use the GCS to:

- Monitor robot status and sensor feeds
- Send mission commands and waypoints
- Visualize robot poses and planned paths
- Coordinate multi-robot operations
- Record and analyze mission data

## Directory Structure

The GCS is organized under `gcs/`:

```
gcs/
├── docker/                           # GCS containerization
│   ├── docker-compose.yaml           # Main launch configuration
│   ├── gcs-base-docker-compose.yaml  # Shared base service
│   ├── Dockerfile.gcs                # Image definition
│   ├── Foxglove/                     # Foxglove Studio configuration
│   │   ├── layouts/                  # Pre-configured dashboard layouts
│   │   └── extensions/               # Custom Foxglove extensions
│   └── resources/                    # GCS assets and resources
├── ros_ws/                           # ROS 2 workspace
│   └── src/                          # Source packages
│       ├── rqt_gcs/                  # Main GCS GUI panel
│       ├── rqt_airstack_control_panel/ # Robot control panel
│       └── ros2tak_tools/            # TAK server integration (optional)
└── bags/                             # Recorded mission data
```

## Launch Structure

The GCS is launched via Docker Compose. The configuration is in `gcs/docker/docker-compose.yaml`.

**Key components:**

- **RQT GCS Panel:** Main monitoring and control interface
- **Foxglove Studio:** Advanced visualization and debugging  
- **TAK Integration:** Situational awareness via TAK protocol (optional)

**Launch command:**

```bash
# Start GCS container
airstack up gcs

# Access GCS interface
# - RQT panel opens automatically in container
# - Or connect to container: airstack connect gcs
```

The Docker `command:` launches the GCS ROS 2 nodes and opens the RQT interface.

**Learn more:** [Docker Configuration](docker/index.md)

## GCS Interfaces

### RQT GCS Panel

The main control interface built with RQT (ROS Qt GUI).

**Features:**

- Real-time robot status monitoring
- Mission planning and waypoint management
- Sensor feed visualization  
- Multi-robot coordination dashboard
- Emergency controls and safety monitoring

**See:** [User Interface Guide](usage/user_interface.md)

### Foxglove Studio

Advanced visualization and debugging interface for robotics data.

**Features:**

- **3D scene visualization:** View robot poses, paths, and point clouds
- **Custom layouts:** Pre-configured dashboards for different mission phases
- **Data recording:** Record and playback mission data
- **Remote monitoring:** Access via web browser from anywhere
- **Custom panels:** Extensible with custom visualization plugins

**Configuration:** Pre-configured layouts available in `gcs/docker/Foxglove/layouts/`

**Access:** Foxglove runs on port 8765 (configurable via docker-compose)

### TAK Integration (Optional)

Integration with TAK (Team Awareness Kit) servers for military and first responder workflows.

**Use cases:**

- Coordinate with TAK-enabled ground teams
- Share situational awareness data
- Integrate with existing TAK infrastructure

**See:** [WinTAK Installation](wintak/installation.md) | [Command Center](command_center/command_center.md)

## System Requirements

**Hardware:**

- **Hard Disk:** 60GB free space
- **RAM:** 8GB minimum, 16GB recommended
- **CPU:** 4 cores minimum, 8+ recommended
- **Network:** Access to robot containers (via airstack_network or direct connection)

**Software:**

- **OS:** Ubuntu 22.04/24.04 LTS
- **Docker:** Installed via `airstack install`
- **Display:** X11 display server (for RQT GUI)

## Quick Start

1. **Launch GCS container:**
   ```bash
   airstack up gcs
   ```

2. **Launch robots** (if not already running):
   ```bash
   airstack up robot-desktop
   # Or with multiple robots
   NUM_ROBOTS=3 airstack up robot-desktop
   ```

3. **Access GCS interface:**
   - RQT panel opens automatically in container
   - Or connect manually: `airstack connect gcs`
   - Foxglove: Open browser to `http://localhost:8765`

4. **Monitor and control** robots via the interface

**Full tutorial:** [Getting Started](../getting_started/index.md)

## Next Steps

- **[User Interface Guide](usage/user_interface.md)** - Learn the RQT GCS interface
- **[Command Center](command_center/command_center.md)** - Mission planning and execution
- **[Casualty Assessment](casualty_assessment/casualty_assessment.md)** - Emergency response features
- **[Docker Configuration](docker/index.md)** - Advanced GCS setup and deployment
- **[WinTAK Installation](wintak/installation.md)** - Optional TAK integration
