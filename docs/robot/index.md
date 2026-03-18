# Robot

The robot autonomy stack is the core of AirStack, providing layered autonomy from hardware interface through perception, planning, control, and high-level behavior.

## Directory Structure

The robot autonomy stack is organized under `robot/`:

```
robot/
├── docker/                           # Robot containerization
│   ├── docker-compose.yaml           # Main launch configuration (see Launch Structure)
│   ├── robot-base-docker-compose.yaml # Shared base service
│   ├── Dockerfile.robot              # Image definition
│   ├── .bashrc                       # Shell configuration (mounted into containers)
│   ├── robot_name_map/               # Robot identity mapping configs
│   │   └── default_robot_name_map.yaml
│   └── zed/                          # ZED camera Docker files
│       ├── Dockerfile.zed-l4t
│       └── ws/
├── ros_ws/                           # ROS 2 workspace
│   └── src/                          # Source packages (layered architecture)
│       ├── autonomy_bringup/         # Top-level launch orchestration
│       ├── interface/                # Hardware interface & safety
│       │   ├── interface_bringup/
│       │   ├── mavros_interface/
│       │   └── ...
│       ├── sensors/                  # Sensor integration
│       │   ├── sensors_bringup/
│       │   └── ...
│       ├── perception/               # State estimation & perception
│       │   ├── perception_bringup/
│       │   └── ...
│       ├── local/                    # Local planning, control, world models
│       │   ├── local_bringup/
│       │   ├── planners/
│       │   ├── c_controls/
│       │   └── world_models/
│       ├── global/                   # Global planning & mapping
│       │   ├── global_bringup/
│       │   ├── planners/
│       │   └── world_models/
│       └── behavior/                 # High-level decision making
│           ├── behavior_bringup/
│           └── ...
└── bags/                             # ROS 2 bag recordings
```

## Launch Structure

The robot autonomy stack is launched via Docker Compose. The configuration is in `robot/docker/docker-compose.yaml`.

### Multi-Profile Architecture

- **Base service:** `robot-base-docker-compose.yaml` defines shared configuration for all platforms
- **Platform profiles:** Different services for desktop/Jetson/VOXL/testing (see [Docker Services](docker/index.md))
- **Layered bringup:** ROS 2 launch files organized hierarchically

### Launch Command Hierarchy

The Docker `command:` attribute launches the top-level ROS 2 launch file, which cascades through autonomy layers:

```
robot.launch.xml                      # Entry point (robot_bringup)
  └── autonomy.launch.xml             # Autonomy orchestration (autonomy_bringup)
      ├── interface.launch.xml        # Hardware interface
      ├── sensors.launch.xml          # Sensor drivers
      ├── perception.launch.xml       # State estimation
      ├── local.launch.xml            # Local planning & control
      ├── global.launch.xml           # Global planning & mapping
      └── behavior.launch.xml         # Mission execution
```

Each `*_bringup` package contains launch files that orchestrate modules in that layer.

### Quick Reference

```bash
# Start robot container (desktop development)
airstack up robot-desktop

# Start without auto-launch (for development)
AUTOLAUNCH=false airstack up robot-desktop

# Multiple robots
NUM_ROBOTS=3 airstack up robot-desktop

# Different platforms (profiles)
airstack up --profile l4t        # NVIDIA Jetson
airstack up --profile voxl       # ModalAI VOXL
```

**Learn more:**

- [Docker Services](docker/index.md) - Detailed Docker configuration
- [Autonomy Modules](autonomy/index.md) - Layer-by-layer breakdown
- [System Architecture](autonomy/system_architecture.md) - Data flow diagrams

## Common Topics

Standard ROS 2 topics used across the autonomy stack:

| Topic | Type | Description |
|-------|------|-------------|
| `/$ROBOT_NAME/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/interfaces/msg/Odometry.html) | Best estimate of robot state |
| `/$ROBOT_NAME/global_plan` | [nav_msgs/Path](https://docs.ros.org/en/rolling/p/nav_msgs/interfaces/msg/Path.html) | Target global trajectory |
| `/$ROBOT_NAME/trajectory_controller/trajectory_override` | airstack_msgs/TrajectoryOverride | Direct trajectory commands |
| `/$ROBOT_NAME/trajectory_controller/look_ahead` | geometry_msgs/PointStamped | Look-ahead point for planning |

**See also:**

- [System Architecture](autonomy/system_architecture.md) - Complete data flow diagrams
- [Integration Checklist](autonomy/integration_checklist.md) - Full topic reference

## Next Steps

- **[Autonomy Modules](autonomy/index.md)** - Understand the layered architecture
- **[System Architecture](autonomy/system_architecture.md)** - See how components interact
- **[Integration Checklist](autonomy/integration_checklist.md)** - Add new modules
- **[Docker Services](docker/index.md)** - Configure deployment platforms