# Docker Folder Restructuring Summary

## Changes Made

Successfully moved docker folders back to their component-specific locations and updated all paths in docker-compose files.

## New Structure

```
AirStack/
├── robot/
│   └── docker/                    # Robot container configs
│       ├── Dockerfile.robot
│       ├── docker-compose.yaml
│       ├── robot-base-docker-compose.yaml
│       ├── .bashrc
│       ├── custom_rosdep.yaml
│       ├── wait_for_px4.py
│       └── zed/
│   └── bags/                      # Robot bag recordings (created)
├── gcs/
│   └── docker/                    # GCS container configs
│       ├── Dockerfile.gcs
│       ├── docker-compose.yaml
│       ├── gcs-base-docker-compose.yaml
│       ├── .bashrc
│       ├── Foxglove/
│       └── resources/
│   └── bags/                      # GCS bag recordings (existing)
├── simulation/
│   ├── isaac-sim/
│   │   └── docker/                # Isaac Sim container configs
│   │       ├── Dockerfile.isaac-ros
│   │       ├── docker-compose.yaml
│   │       ├── .bashrc
│   │       ├── fastdds.xml
│   │       ├── omniverse.toml
│   │       └── omni_pass_TEMPLATE.env
│   └── simple-sim/
│       └── docker/                # Simple sim container configs
│           ├── Dockerfile.sim
│           └── docker-compose.yaml
├── docs/
│   └── docker/                    # Documentation container configs
│       ├── Dockerfile
│       └── docker-compose.yaml
└── ros_ws/
    └── src/
        ├── autonomy/              # Autonomy stack packages
        ├── bringups/              # Platform-specific bringups (desktop, voxl, jetson)
        ├── gui/                   # GCS/GUI packages
        ├── logging/               # Logging packages
        └── robot_descriptions/    # Robot URDF descriptions
```

## ros_ws Structure (Simplified)

The ros_ws structure has been reorganized for clarity:

```
ros_ws/src/
├── autonomy/                      # Autonomy stack
│   ├── airstack_common/          # Common utilities
│   ├── airstack_msgs/            # Common message definitions
│   ├── interface/
│   │   └── interface_bringup/   # Interface module bringup
│   ├── sensors/
│   │   └── sensors_bringup/     # Sensors module bringup
│   ├── perception/
│   │   └── perception_bringup/  # Perception module bringup
│   ├── local/
│   │   └── local_bringup/       # Local planning bringup
│   ├── global/
│   │   └── global_bringup/      # Global planning bringup
│   └── behavior/
│       └── behavior_bringup/    # Behavior module bringup
├── bringups/                      # Platform-level orchestration
│   ├── desktop_bringup/          # Desktop/simulation platform
│   ├── voxl_bringup/             # VOXL hardware platform
│   └── jetson_bringup/           # Jetson hardware platform
├── gui/                           # GCS and GUI packages
│   ├── rqt_gcs/
│   ├── rqt_airstack_control_panel/
│   ├── rqt_behavior_tree/
│   └── ros2tak_tools/
├── logging/                       # Logging utilities
│   ├── logging_bringup/
│   └── bag_recorder_pid/
└── robot_descriptions/            # Robot descriptions
    ├── spirit_with_sensors_description/
    └── iris_with_sensors_description/
```

## Updated Files

### Main Configuration
- **docker-compose.yaml**: Updated include paths to reflect new docker locations

### Robot
- **robot/docker/robot-base-docker-compose.yaml**: 
  - Fixed bags path: `../../robot/bags` → `../bags`
  - Commented out common config files (.bash_profile, inputrc, .tmux.conf) that no longer exist

### GCS
- **gcs/docker/gcs-base-docker-compose.yaml**:
  - Fixed bags path: `../../gcs/bags` → `../bags`
  - Commented out common config files
- **gcs/docker/docker-compose.yaml**:
  - Fixed plot path: `../../plot` → `../plot`

### Simulation
- **simulation/isaac-sim/docker/docker-compose.yaml**:
  - Fixed extensions path: `../../../../simulation/isaac-sim/extensions/...` → `../../extensions/...`
  - Fixed AirStack mount: `../../../..` → `../../..`
  - Fixed .devcontainer paths: `../../../../.devcontainer/...` → `../../../.devcontainer/...`
  - Commented out common config files
- **simulation/simple-sim/docker/docker-compose.yaml**:
  - Commented out common config files and fastdds.xml mount

## Rationale

This structure maintains **machine-specific configs** in their respective component directories:

1. **Robot-specific**: ZED camera configs, PX4 waiting scripts, ROS dependencies
2. **GCS-specific**: Foxglove configs, FastRTPS profiles, domain bridge configs
3. **Isaac Sim-specific**: Omniverse credentials, user configs, FastDDS profiles

## Common Config Files

The following files were removed from the top-level docker/ directory as they are no longer needed:
- `.bash_profile`
- `inputrc`
- `.tmux.conf`

If these files are needed in the future, they should be:
1. Created in a `common/` directory at the repository root, OR
2. Copied individually to each docker/ subdirectory as needed

## Testing Recommendations

1. **Build test**: Verify docker-compose can parse all files
   ```bash
   docker-compose config --quiet
   ```

2. **Container test**: Start each container and verify mounts
   ```bash
   ./airstack.sh up robot
   docker exec airstack-robot-1 ls -la /root/AirStack
   docker exec airstack-robot-1 ls -la /bags
   ```

3. **Build test**: Test workspace builds correctly
   ```bash
   docker exec airstack-robot-1 bash -c "cd /root/AirStack/ros_ws && bws"
   ```

4. **Launch test**: Test launch files work with new structure
   ```bash
   docker exec airstack-robot-1 bash -c "bws && sws && ros2 launch desktop_bringup robot.launch.xml"
   ```

## Git Changes

All moves were done with `git mv` to preserve file history:
- Moved `docker/robot/` → `robot/docker/`
- Moved `docker/gcs/` → `gcs/docker/`
- Moved `docker/simulation/isaac-sim/` → `simulation/isaac-sim/docker/`
- Removed empty `docker/` directory and common config files

Total files affected: ~40 moved/renamed, ~10 modified, 3 deleted, 1 created (robot/bags/)

## Next Steps

1. Test the build and launch process
2. Update documentation to reflect new structure
3. Commit changes with descriptive message
4. Create PR for review
