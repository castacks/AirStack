# AirStack Configuration Directory

This directory contains centralized configuration files for the AirStack system.

## Config Propagation

Configuration files are mounted as read-only volumes in Docker containers and their paths are passed via environment variables defined in the top-level `.env` file.

### Environment Variables

- `CONFIG_MOUNT_PATH`: Path where config directory is mounted inside containers (`/config`)

### Docker Compose Integration

Both robot and simulation containers mount this config directory:
- **Robot containers**: Mount `../../config:${CONFIG_MOUNT_PATH}:ro`
- **Isaac Sim container**: Mount `../../../config:${CONFIG_MOUNT_PATH}:ro`

### Usage in Code

- **ROS2 launch files**: Use `$(env CAMERA_CONFIG_FILE /config/camera_config.yaml)` substitution
- **Python scripts**: Use `os.environ.get('CAMERA_CONFIG_FILE', '/config/camera_config.yaml')`

## Configuration Files

- `camera_config.yaml`: Camera parameters for stereo camera setup (baseline, intrinsics, etc.)
- `isaac_sim_config.yaml`: Isaac Sim parameters for the starting scene and extensions to enable.