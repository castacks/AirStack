# Robot Configuration

Configure robot-specific parameters, sensor calibrations, and system settings for AirStack deployment.

## Overview

Robot configuration includes:

- **Robot Identity**: Unique identification (name, ROS_DOMAIN_ID)
- **Sensor Configuration**: Calibration parameters and topic mappings
- **Network Settings**: Communication and connectivity
- **Hardware Parameters**: Platform-specific settings (Jetson, VOXL)
- **Autonomy Parameters**: Behavior and performance tuning

## Configuration Files

### Environment Variables

Key environment variables configured in `robot/docker/.env`:

```bash
# Robot Identity
ROBOT_NAME=robot1
ROS_DOMAIN_ID=0

# Launch Configuration
AUTOLAUNCH=true
AUTONOMY_MODE=onboard_all

# Sensor Configuration
ENABLE_CAMERA=true
ENABLE_LIDAR=false
CAMERA_TOPIC=/camera/image_raw
```

### ROS 2 Parameters

Module-specific parameters in YAML files:

**Location**: `robot/ros_ws/src/<layer>/<module>_bringup/config/`

**Example** (`perception_bringup/config/state_estimation.yaml`):
```yaml
state_estimator:
  ros__parameters:
    publish_rate: 100.0
    use_gps: true
    imu_topic: /imu/data
    gps_topic: /gps/fix
```

## Robot Identity

Each robot requires unique configuration for multi-robot scenarios.

See: [Robot Identity Guide](../docker/robot_identity.md)

**Key Settings**:
- **ROBOT_NAME**: Namespace for all topics (`/robot1/...`)
- **ROS_DOMAIN_ID**: Isolate ROS 2 communication (0-101)
- **Hostname**: Unique network identifier

## Sensor Configuration

### Camera Configuration

Configure camera parameters:

```yaml
camera:
  ros__parameters:
    frame_id: camera_link
    width: 1280
    height: 720
    fps: 30
    encoding: rgb8
```

### IMU Configuration

IMU calibration and orientation:

```yaml
imu:
  ros__parameters:
    frame_id: imu_link
    accel_stddev: 0.01
    gyro_stddev: 0.005
    orientation_covariance: [0.01, 0, 0,
                             0, 0.01, 0,
                             0, 0, 0.01]
```

### Depth Sensor Configuration

Depth camera/stereo parameters:

```yaml
depth_camera:
  ros__parameters:
    frame_id: depth_camera_link
    min_range: 0.5
    max_range: 10.0
    fov_horizontal: 87.0
    fov_vertical: 58.0
```

## Autonomy Configuration

### Local Planning Parameters

Tune local planner behavior:

```yaml
local_planner:
  ros__parameters:
    planning_horizon: 5.0
    max_velocity: 2.0
    max_acceleration: 1.0
    obstacle_margin: 0.5
```

### Global Planning Parameters

Configure global planner:

```yaml
global_planner:
  ros__parameters:
    planning_rate: 1.0
    goal_tolerance: 0.5
    path_resolution: 0.1
```

### Controller Parameters

Trajectory controller tuning:

```yaml
trajectory_controller:
  ros__parameters:
    kp_position: 1.0
    kd_position: 0.5
    kp_velocity: 0.8
    max_thrust: 20.0
```

## Network Configuration

### WiFi Configuration

For onboard computer (Jetson/VOXL):

```bash
# /etc/netplan/01-netcfg.yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: yes
      access-points:
        "YourSSID":
          password: "YourPassword"
```

### Static IP (Optional)

For reliable communication:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

## Platform-Specific Configuration

### NVIDIA Jetson

Power mode settings:

```bash
# Maximum performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Balanced mode
sudo nvpmodel -m 2
```

Configure fan control:

```bash
# /etc/systemd/system/jetson-fan.service
[Unit]
Description=Jetson Fan Control

[Service]
Type=simple
ExecStart=/usr/bin/jetson_fan.py

[Install]
WantedBy=multi-user.target
```

### ModalAI VOXL

VOXL-specific configuration via `voxl-configure-*` tools:

```bash
# Configure cameras
voxl-configure-cameras

# Configure MPA
voxl-configure-mpa

# Configure vision  
voxl-configure-vision
```

## Parameter Tuning Workflow

### 1. Baseline Configuration

Start with default parameters from reference implementation.

### 2. Simulation Testing

Test parameter changes in Isaac Sim:

```bash
# Launch with custom parameters
ros2 launch my_module_bringup my_module.launch.xml param_file:=config/tuned_params.yaml
```

### 3. HITL Validation

Validate on hardware-in-the-loop setup before field deployment.

See: [HITL Testing](../../real_world/HITL/index.md)

### 4. Field Tuning

Fine-tune based on real-world performance:

- Monitor performance metrics
- Adjust parameters incrementally
- Document changes and rationale
- Test thoroughly after each change

## Configuration Management

### Version Control

Track configuration files in Git:

```bash
git add robot/ros_ws/src/*/config/*.yaml
git commit -m "Tune planner parameters for outdoor operation"
```

### Robot-Specific Configs

For multiple robots with different configurations:

```
robot/ros_ws/src/my_module/config/
├── default.yaml          # Default parameters
├── robot1.yaml           # Robot 1 overrides
├── robot2.yaml           # Robot 2 overrides
└── outdoor.yaml          # Environment-specific
```

Load appropriate config:

```xml
<node pkg="my_module" exec="my_node">
    <param from="$(find-pkg-share my_module)/config/$(var robot_name).yaml" />
</node>
```

### Configuration Validation

Validate configuration before deployment:

```python
#!/usr/bin/env python3
import yaml

def validate_config(config_file):
    with open(config_file) as f:
        config = yaml.safe_load(f)
    
    # Check required parameters exist
    assert 'max_velocity' in config
    assert 'planning_horizon' in config
    
    # Check parameter ranges
    assert 0 < config['max_velocity'] <= 5.0
    assert config['planning_horizon'] > 0
    
    print(f"✓ Configuration {config_file} is valid")

if __name__ == "__main__":
    validate_config("config/my_params.yaml")
```

## Dynamic Reconfiguration

Some parameters can be changed at runtime without restart:

```bash
# Get current parameter value
ros2 param get /my_node my_parameter

# Set new parameter value
ros2 param set /my_node my_parameter 2.5

# Dump all parameters
ros2 param dump /my_node > current_params.yaml
```

## Troubleshooting

**Parameter changes not taking effect**:
- Verify parameter file path in launch file
- Check for typos in parameter names
- Rebuild package if C++ parameters changed
- Restart nodes after parameter changes

**Invalid parameter values**:
- Check parameter validation in node code
- Review error messages for allowed ranges
- Verify YAML syntax (indentation, types)

**Configuration conflicts**:
- Check for multiple parameter files being loaded
- Verify launch file parameter precedence
- Use `ros2 param dump` to see actual loaded values

## Best Practices

- **Document parameters**: Add comments in YAML files
- **Use reasonable defaults**: Safe, conservative values
- **Validate inputs**: Check parameter ranges in code
- **Version control**: Track configuration changes
- **Test incrementally**: Change one parameter at a time
- **Keep backups**: Save known-good configurations

## See Also

- [Robot Identity](../docker/robot_identity.md) - Configuring robot identification
- [Autonomy Modes](../../tutorials/autonomy_modes.md) - Different operation modes
- [HITL Testing](../../real_world/HITL/index.md) - Testing configuration on hardware
- [Integration Checklist](../autonomy/integration_checklist.md) - Module configuration requirements
