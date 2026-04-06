# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AirStack is a comprehensive autonomous aerial robotics stack developed by AirLab CMU. It provides a complete end-to-end system for autonomous drone operations including simulation, robot control, and ground control station.

## Key Components

The repository is organized into four main components:

- **robot/**: Onboard autonomy stack for drones (ROS 2 based)
- **simulation/**: Isaac Sim integration and simulation environments
- **gcs/**: Ground Control Station for mission planning and monitoring
- **common/**: Shared ROS packages and utilities used across components

## Development Commands

### AirStack CLI Tool
The project uses a custom CLI tool `./airstack.sh` for common operations:

```bash
# Setup and installation
./airstack setup          # Configure AirStack and add to PATH
./airstack install        # Install Docker and dependencies

# Container management
./airstack up [service]    # Start services with Docker Compose
./airstack stop [service]  # Stop services
./airstack status          # Show container status
./airstack connect [name]  # Connect to running container (supports partial names)
./airstack logs [name]     # View container logs

# Development tasks
./airstack build          # Build the workspace
./airstack test           # Run tests
./airstack docs           # Build documentation
```

### ROS 2 Build Commands
Inside robot containers, use these aliases:
- `bws`: Build the ROS workspace (`colcon build`)
- `sws`: Source the workspace (`source install/setup.bash`)

### VSCode Integration
The repository includes VSCode tasks:
- **Build ros_ws Debug**: Builds ROS workspace in debug mode using `bws --cmake-args '-DCMAKE_BUILD_TYPE=Debug'`

## Architecture

### Robot Autonomy Stack (robot/ros_ws/src/autonomy/)

The autonomy stack follows a layered architecture with numbered modules:

- **0_interface/**: Robot hardware interface and safety monitors
  - `mavros_interface`: Communication with flight controller
  - `drone_safety_monitor`: Safety monitoring and emergency handling
  - `robot_interface`: High-level robot abstraction

- **1_sensors/**: Sensor integration and processing
  - `camera_param_server`: Camera calibration management
  - `gimbal_stabilizer`: Gimbal control and stabilization
  - `sensor_interfaces`: Common sensor message definitions

- **2_perception/**: State estimation and environment perception
  - `macvo_ros2`: Visual-inertial odometry system

- **3_local/**: Local planning and control
  - `a_world_models/`: Local world representation
    - `disparity_expansion`: Obstacle detection from stereo
    - `disparity_graph`: Graph-based obstacle representation
    - `cost_map_interface`: Cost map utilities
  - `b_planners/`: Local motion planning
    - `droan_local_planner`: DROAN obstacle avoidance planner
    - `takeoff_landing_planner`: Specialized takeoff/landing
    - `trajectory_library`: Trajectory generation utilities
  - `c_controls/`: Low-level control
    - `trajectory_controller`: Trajectory tracking controller
    - `attitude_controller`: Attitude control
    - `pid_controller`: PID control utilities

- **4_global/**: Global planning and mapping
  - `a_world_models/`: Global world representation
    - `vdb_mapping_ros2`: VDB-based 3D mapping
  - `b_planners/`: Global path planning
    - `random_walk`: Random exploration planner
    - `ensemble_planner`: Multi-planner coordination

- **5_behavior/**: High-level mission execution
  - `behavior_tree`: Behavior tree framework
  - `behavior_executive`: Mission execution engine
  - `rqt_behavior_tree_command`: GUI for behavior tree control

### Docker Architecture

Each component has its own Docker setup:
- `robot/docker/`: Robot container with ROS 2 and autonomy stack
- `simulation/isaac-sim/docker/`: Isaac Sim simulation environment
- `gcs/docker/`: Ground control station container
- `docs/docker/`: Documentation building container

The main `docker-compose.yaml` includes all component compose files for unified management.

## Common Patterns

### ROS 2 Launch Files
- Bringup packages (`*_bringup/`) contain launch files for each subsystem
- Launch files are organized by functionality and can be composed
- Example: `ros2 launch robot_bringup robot.launch.py`

### Configuration Management
- Parameters stored in `config/` directories within packages
- YAML files for node parameters and launch configurations
- Environment variables configured in `.env` file

### Testing
- Test files located in `test/` directories within packages
- Integration tests in `tests/` at repository root

## Important Notes

- The project uses ROS 2 Humble
- Primary development is done in Docker containers
- Documentation is built with MkDocs and hosted at docs.theairlab.org
- The autonomy stack supports both simulation (SITL) and hardware deployment
- Container networking uses custom bridge network (172.31.0.0/24)