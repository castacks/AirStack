# Development Environment Setup

Set up your IDE and tools for AirStack development. This guide assumes you've completed [Getting Started](../../getting_started/index.md) and understand [Key Concepts](key_concepts.md).

## System Requirements

**Host Machine:**

- **OS**: Ubuntu 22.04/24.04 LTS (recommended)
- **CPU**: 8+ cores
- **RAM**: 16GB minimum, 32GB+ recommended  
- **GPU**: NVIDIA RTX 3070+ for Isaac Sim
- **Storage**: 100GB+ free SSD space

**Software:**

- Docker with NVIDIA Container Toolkit (installed via `airstack install`)
- Git with SSH keys
- VSCode (recommended) or your preferred IDE

## IDE Configuration

### VSCode (Recommended)

VSCode provides excellent ROS 2 and C++ support. When you open the AirStack folder, VSCode will prompt you to install recommended extensions from `.vscode/extensions.json`.

**Key Extensions:**

- ROS extension (for ROS 2 development)
- C/C++ extension (Microsoft)
- Python extension
- Docker extension (for container management)
- CMake Tools
- YAML extension

**Settings:**

VSCode is pre-configured for AirStack in `.vscode/settings.json`. This includes:

- ROS 2 workspace setup
- C++ IntelliSense configuration
- CMake integration
- Python path configuration

### Debugging Setup

For debugging ROS 2 nodes inside containers, see [VSCode Debugging Guide](vscode/vscode_debug.md).

## Workspace Aliases

Inside robot containers, several aliases simplify development:

```bash
bws    # Build workspace (alias for colcon build)
sws    # Source workspace (alias for source install/setup.bash)
cws    # Clean workspace (removes build/install/log)
```

These are defined in `robot/.bashrc` and available in all robot containers.

## Development Tips

**Quick iteration:**
```bash
# Start without auto-launch
AUTOLAUNCH=false airstack up robot

# In another terminal, build and test
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package && sws && ros2 launch my_package test.launch.xml"
```

**Interactive development:**
```bash
# Connect to container
airstack connect robot

# Inside container, iterate quickly
cd /opt/ros_ws
bws --packages-select my_package
sws
ros2 launch my_package test.launch.xml
```

**Build with debug symbols:**
```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
```

See [Key Concepts](key_concepts.md) for the full development loop.

