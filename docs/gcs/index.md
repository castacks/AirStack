# Ground Control Station (GCS)

The Ground Control Station provides monitoring, control, and mission planning capabilities for AirStack robots. Operators use the GCS to:

- Monitor robot status, camera/depth feeds, and sensor streams
- Send task commands (takeoff, land, navigate, fixed trajectory, search, exploration)
- Draw waypoint routes and polygon areas directly on the map
- Visualize robot poses, planned paths, and maps for the whole fleet
- Record mission data as ROS 2 bags

The main operator interface is **Foxglove Studio**, extended with custom AirStack panels. Everything runs inside one Docker container.

## Directory Structure

The GCS is organized under `gcs/`:

```
gcs/
├── docker/                           # GCS containerization
│   ├── docker-compose.yaml           # gcs (sim/dev) + gcs-real (field) services
│   ├── gcs-base-docker-compose.yaml  # Shared base service (command, env, mounts)
│   ├── Dockerfile.gcs                # Image: ROS 2 Jazzy + Foxglove Studio + DDS Router
│   ├── .bashrc                       # Shell config mounted into the container
│   └── Foxglove/                     # Foxglove Studio app state (mounted, gitignored)
├── foxglove_extensions/              # Custom Foxglove panels + layout tooling
│   ├── waypoint-editor/              # Click-to-place waypoint routes
│   ├── polygon-editor/               # Click-to-draw polygon areas
│   ├── robot-commands/               # "Robot Tasks" command panel
│   ├── install.py                    # Installs the panels into Foxglove on startup
│   ├── render_layout.py              # Renders the NUM_ROBOTS-matched layout
│   └── airstack_default.json         # Single-robot layout template
├── ros_ws/                           # ROS 2 workspace
│   └── src/
│       ├── action_relay/             # Bridges task actions from GCS domain 0 to each robot domain
│       ├── gcs_visualizer/           # Fleet markers/poses for Foxglove's 3D panel
│       └── common/                   # Mount of common/ros_packages (desktop_bringup, coordination, ...)
├── saves/                            # Persisted waypoint/polygon editor saves (mounted at /root/.airstack)
└── bags/                             # Recorded mission data (mounted at /bags)
```

## Launch Structure

The GCS is launched via Docker Compose (`gcs/docker/docker-compose.yaml`, extending `gcs-base-docker-compose.yaml`). On container start it:

1. Restarts the SSH daemon
2. Installs the custom Foxglove extensions (`foxglove_extensions/install.py`)
3. Renders the multi-robot Foxglove layout to `/root/airstack_layout_num_robots_<N>.json` (`render_layout.py`)
4. Opens a tmux session named `bringup`
5. If `AUTOLAUNCH=true`, runs `ros2 launch desktop_bringup gcs.launch.xml` in that session

`desktop_bringup/launch/gcs.launch.xml` (in `common/ros_packages`) starts:

| Component | Purpose |
|-----------|---------|
| **Foxglove Studio** (desktop app) | Main operator GUI, connects to `ws://localhost:8765` |
| **foxglove_bridge** | ROS 2 ⇄ Foxglove WebSocket bridge on port 8765 |
| **Gossip bridge** | Connects GCS domain 0 to the gossip bus (domain 99) |
| **gcs_visualizer** | Renders per-robot meshes/trajectories/maps in a shared global ENU frame |
| **action_relay** | One relay per robot: forwards task goals from Foxglove to each robot's domain |

**Launch command:**

```bash
# Start GCS container (usually alongside the rest of the stack)
airstack up gcs

# Attach to the container's tmux session
airstack connect gcs
```

**Learn more:** [Docker Configuration](docker/index.md)

## GCS Interfaces

### Foxglove Studio

The operator GUI. The container runs the Foxglove Studio desktop app on the host's X display; you can also open Foxglove on the host and connect to the bridged WebSocket.

**Features:**

- **3D scene visualization:** fleet poses, trajectories, global plans, VDB maps ([details](foxglove.md))
- **Robot Tasks panel:** send takeoff / land / navigate / trajectory / search / exploration commands per robot
- **Waypoint & polygon editors:** click-to-place routes and areas ([guide](waypoints_and_geofences.md))
- **Per-robot tabs:** the rendered layout creates one tab per robot for `NUM_ROBOTS` robots
- **Data recording and playback**

**Connection:** `ws://localhost:8765` inside the container, `ws://localhost:8766` from the host (the `gcs` service publishes 8766→8765). See [GCS Foxglove Visualization](foxglove.md) for the layout import flow.

## System Requirements

**Hardware:**

- **Hard Disk:** 60GB free space
- **RAM:** 8GB minimum, 16GB recommended
- **CPU:** 4 cores minimum, 8+ recommended
- **GPU:** NVIDIA GPU (the service reserves one for rendering)
- **Network:** access to robot containers (via `airstack_network` or the LAN for `gcs-real`)

**Software:**

- **OS:** Ubuntu 22.04/24.04 LTS
- **Docker:** installed via `airstack install`
- **Display:** X11 display server (for the Foxglove Studio window)

## Quick Start

1. **Launch the stack** (sim + robots + GCS — the `desktop` profile includes the `gcs` service):
   ```bash
   airstack up --sim isaac --robots 1
   ```

2. **Foxglove opens automatically** in the GCS container (when `AUTOLAUNCH=true`). Import the rendered layout and connect — see [GCS Foxglove Visualization](foxglove.md).

3. **Command robots** from the Robot Tasks panel; place waypoints with the editors.

**Full tutorial:** [Getting Started](../getting_started/index.md)

## Next Steps

- **[Docker Configuration](docker/index.md)** - GCS container internals, profiles, and development workflow
- **[GCS Foxglove Visualization](foxglove.md)** - Layout import, visualizer topics, extending markers
- **[Adding Waypoints and Geofences](waypoints_and_geofences.md)** - Interactive editors
- **[User Interface Guide](usage/user_interface.md)** - Interface walkthrough
