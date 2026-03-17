# Development Environment Setup

Configure your development environment for optimal AirStack development workflow. This guide covers IDE setup, debugging tools, and recommended configurations.

## Prerequisites

- Completed [Getting Started](../getting_started.md)
- Docker and Docker Compose installed
- Basic familiarity with ROS 2 and Linux

## Recommended Setup

### Host Machine

**Operating System**: Ubuntu 22.04 LTS (recommended)

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

See: [VSCode Setup Guide](vscode/index.md)

**Recommended Extensions**:
- ROS extension
- C/C++ extension (Microsoft)
- Python extension
- Docker extension
- CMake Tools
- YAML extension

### CLion

JetBrains CLion works well for C++ development:

1. Install CLion ROS plugin
2. Import AirStack as CMake project
3. Configure remote interpreter to Docker container
4. Set up run configurations for ROS 2 nodes

### Vim/Emacs

For terminal-based development:

```bash
# Connect to running robot container
airstack connect robot

# Use tools inside container
vim robot/ros_ws/src/...
```

Install plugins for:
- Syntax highlighting (ROS msg/srv/action files)
- C++/Python language servers
- File navigation

## Development Workflow

### Container-Based Workflow

1. **Start containers without autolaunch**:
   ```bash
   AUTOLAUNCH=false airstack up robot
   ```

2. **Build your changes**:
   ```bash
   docker exec airstack-robot-1 bash -c "bws --packages-select my_package"
   ```

3. **Source workspace**:
   ```bash
   docker exec airstack-robot-1 bash -c "sws"
   ```

4. **Run/test**:
   ```bash
   docker exec airstack-robot-1 bash -c "sws && ros2 launch my_package my_launch.xml"
   ```

### Interactive Development

For frequent rebuilds during development:

```bash
# Connect to container interactively (careful with prompts!)
docker exec -it airstack-robot-1 bash

# Inside container
cd /opt/ros_ws
bws --packages-select my_package
sws
ros2 launch my_package my_launch.xml
```

**Warning**: Avoid interactive mode for automated workflows - you can get stuck on prompts.

## Debugging Tools

### GDB (C++ Debugging)

Build with debug symbols:
```bash
docker exec airstack-robot-1 bash -c "bws --packages-select my_package --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
```

Run with GDB:
```bash
docker exec -it airstack-robot-1 bash -c "gdb --args /opt/ros_ws/install/my_package/lib/my_package/my_node"
```

### ROS 2 Debugging Tools

**Topic inspection**:
```bash
# List topics
docker exec airstack-robot-1 bash -c "ros2 topic list"

# Echo topic
docker exec airstack-robot-1 bash -c "ros2 topic echo /robot1/odometry --once"

# Check publishing rate
docker exec airstack-robot-1 bash -c "ros2 topic hz /robot1/odometry"
```

**Node inspection**:
```bash
# List nodes
docker exec airstack-robot-1 bash -c "ros2 node list"

# Node info
docker exec airstack-robot-1 bash -c "ros2 node info /my_node"
```

**Parameter inspection**:
```bash
# List parameters
docker exec airstack-robot-1 bash -c "ros2 param list /my_node"

# Get parameter
docker exec airstack-robot-1 bash -c "ros2 param get /my_node my_param"
```

### Visualization Tools

**RViz2** (inside Docker):
```bash
docker exec airstack-robot-1 bash -c "rviz2"
```

**rqt** (GUI tools):
```bash
docker exec airstack-robot-1 bash -c "rqt"
```

**PlotJuggler** (time series plotting):
```bash
# Install on host or in container
docker exec airstack-robot-1 bash -c "apt-get install ros-jazzy-plotjuggler"
docker exec airstack-robot-1 bash -c "ros2 run plotjuggler plotjuggler"
```

## Code Quality Tools

### Linting and Formatting

**C++ (clang-format)**:
```bash
# Format a file
clang-format -i src/my_file.cpp

# Check formatting
clang-format --dry-run -Werror src/my_file.cpp
```

**Python (black, ruff)**:
```bash
# Format Python code
black my_package/

# Lint Python code
ruff check my_package/
```

**CMake (cmake-format)**:
```bash
cmake-format -i CMakeLists.txt
```

### Static Analysis

**C++ (clang-tidy)**:
```bash
clang-tidy src/my_file.cpp -- -I/opt/ros_ws/install/include
```

**Python (mypy)**:
```bash
mypy my_package/
```

## Testing in Development

### Unit Tests

Run unit tests for a package:
```bash
docker exec airstack-robot-1 bash -c "colcon test --packages-select my_package"
```

View test results:
```bash
docker exec airstack-robot-1 bash -c "colcon test-result --all --verbose"
```

### Integration Tests

Test with full stack:
```bash
# Launch autonomy stack
airstack up

# Run integration tests
docker exec airstack-robot-1 bash -c "ros2 launch my_package integration_test.launch.xml"
```

## Performance Profiling

### CPU Profiling

**perf** (Linux):
```bash
docker exec -it airstack-robot-1 bash
perf record -g ./my_node
perf report
```

**gprof**:
```bash
# Build with profiling
bws --cmake-args '-DCMAKE_CXX_FLAGS=-pg'

# Run node
./my_node

# Generate report
gprof my_node gmon.out > analysis.txt
```

### Memory Profiling

**Valgrind**:
```bash
docker exec -it airstack-robot-1 bash
valgrind --leak-check=full ./my_node
```

**Heaptrack**:
```bash
heaptrack ./my_node
heaptrack_gui heaptrack.my_node.PID.gz
```

## Environment Configuration

### Environment Variables

Key variables in Docker containers:

```bash
# ROS 2 configuration
ROS_DOMAIN_ID=0              # ROS 2 domain
ROS_LOCALHOST_ONLY=0         # Allow network communication
ROBOT_NAME=robot1            # Robot namespace

# Build configuration
MAKEFLAGS=-j$(nproc)         # Parallel build jobs
CMAKE_BUILD_TYPE=RelWithDebInfo  # Build type
```

### Aliases and Shortcuts

Add to `.bashrc` (inside container or persistent via Docker config):

```bash
# Build and source shortcuts
alias bws='colcon build --symlink-install && source install/setup.bash'
alias sws='source install/setup.bash'

# Quick package build
alias bp='colcon build --packages-select'

# ROS 2 shortcuts
alias rt='ros2 topic'
alias rn='ros2 node'
alias rp='ros2 param'
```

## Git Configuration

### Inside Container

Configure Git identity:
```bash
docker exec airstack-robot-1 bash -c "git config --global user.name 'Your Name'"
docker exec airstack-robot-1 bash -c "git config --global user.email 'your.email@example.com'"
```

### SSH Keys for GitHub

Mount SSH keys into container (already done in docker-compose):
```yaml
volumes:
  - ~/.ssh:/root/.ssh:ro  # Read-only SSH keys
```

## Troubleshooting

**Build fails with "not enough memory"**:
- Reduce parallel jobs: `MAKEFLAGS=-j4 bws`
- Increase Docker memory limit in Docker Desktop settings

**GUI apps don't display**:
- Run `xhost +` on host before launching containers
- Verify X11 socket is mounted: `ls -la /tmp/.X11-unix/`

**Code changes not reflected**:
- Ensure using `--symlink-install` in colcon build
- Rebuild package: `bws --packages-select my_package`
- Check that you're sourcing the correct workspace

**ROS 2 nodes can't find each other**:
- Verify ROS_DOMAIN_ID matches across containers
- Check Docker network configuration
- Review [Docker Networking](../simulation/docker_network.md)

## Best Practices

- **Use `--symlink-install`** for faster Python/launch file iteration
- **Build with debug symbols** during development
- **Run tests frequently** before committing
- **Use version control** commit small, logical changes
- **Document as you go** update README.md files
- **Profile early** catch performance issues during development

## See Also

- [VSCode Setup](vscode/index.md) - Detailed VSCode configuration
- [Testing Guide](testing/index.md) - Comprehensive testing workflows
- [Contributing](contributing.md) - Code standards and PR process
- [AirStack CLI](airstack-cli/index.md) - CLI tool usage
- [AI Agent Guide](ai_agent_guide.md) - Automated development workflows