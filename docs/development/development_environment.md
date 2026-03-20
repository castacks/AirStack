# Development Environment Setup

Configure your development environment for optimal AirStack development workflow. This guide covers IDE setup, debugging tools, and recommended configurations.

## Prerequisites

- Completed [Getting Started](../getting_started.md)
- Docker and Docker Compose installed
- Basic familiarity with ROS 2 and Linux

## Recommended Setup

### Host Machine

**Operating System**: Ubuntu 22.04/24.04 LTS (recommended)

**Hardware**:

- **CPU**: 8+ cores recommended
- **RAM**: 16GB minimum, 32GB+ recommended
- **GPU**: NVIDIA RTX 3070 or better for Isaac Sim
- **Storage**: SSD with 100GB+ free space

**Software**:

- Docker with NVIDIA Container Toolkit
- Git with SSH keys configured
- VSCode or your preferred IDE

### Docker-Based Development

All AirStack development happens inside Docker containers. This ensures:

- **Consistent environment** across developers
- **Isolated dependencies** don't affect host system
- **Easy cleanup** without residual files
- **Reproducible builds** and testing

## IDE Configuration

### VSCode (Recommended)

VSCode provides excellent ROS 2 and C++ support.

**Recommended Extensions**:

The recommended extensions can be installed automatically in VSCode. When you open the AirStack folder in VSCode, it should prompt you to install the recommended extensions from `.vscode/extensions.json`. Click "Install All" to get the necessary tools for development.

- ROS extension
- C/C++ extension (Microsoft)
- Python extension
- Docker extension
- CMake Tools
- YAML extension

## Development Workflow

### Container-Based Workflow

1. **Start containers without autolaunch**:
   ```bash
   AUTOLAUNCH=false airstack up robot-desktop
   ```

2. **Build your changes**:
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package"
   ```

3. **Source workspace**:
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "sws"
   ```

4. **Run/test**:
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch my_package my_launch.xml"
   ```

### Interactive Development

For frequent rebuilds during development:

```bash
# Connect to container interactively (careful with prompts!)
airstack connect airstack-robot-desktop-1 

# Inside container
cd /opt/ros_ws
bws --packages-select my_package
sws
ros2 launch my_package my_launch.xml
```

