# Real World Deployment

Deploy AirStack to real hardware and fly autonomous missions in the field. This section covers hardware installation, configuration, testing, and field operation best practices.

## Overview

Real-world deployment involves:

- **Hardware Setup** - Installing AirStack on onboard computers (Jetson, VOXL)
- **Robot Configuration** - Configuring robot identity, sensors, and network
- **Hardware-in-the-Loop (HITL) Testing** - Testing with real hardware before field deployment
- **Field Operations** - Running missions and collecting data
- **Data Management** - Offloading and managing collected data

## Supported Hardware Platforms

### NVIDIA Jetson
- Jetson Orin series (recommended)
- Jetson Xavier NX
- Jetson TX2

### ModalAI VOXL
- VOXL 2
- VOXL Flight

## Getting Started with Hardware

### Prerequisites
- Completed [Getting Started Tutorial](../getting_started.md) in simulation
- Understanding of [System Architecture](../robot/autonomy/system_architecture.md)
- Access to supported hardware platform
- Familiarity with [Autonomy Modes](../tutorials/autonomy_modes.md)

### Deployment Process

1. **Install on Hardware**
   - Flash operating system
   - Install AirStack dependencies
   - Configure robot identity and network
   - See: [Installation Guide](installation/index.md)

2. **Hardware-in-the-Loop Testing**
   - Test autonomy stack with real hardware in controlled environment
   - Validate sensor integration
   - Tune controllers and parameters
   - See: [HITL Testing Guide](HITL/index.md)

3. **Field Deployment**
   - Follow safety protocols
   - Pre-flight checklist
   - Monitor during operation
   - Post-flight data collection
   - See: [Deploying to Hardware Tutorial](../tutorials/deploying_to_hardware.md)

4. **Data Management**
   - Offload ROS bags and logs
   - Sync to storage server
   - Archive and analyze
   - See: [Data Offloading](data_offloading/index.md)

## Safety Considerations

!!! warning "Safety First"
    Real-world operation requires adherence to safety protocols:
    
    - **Pre-flight checks**: Verify all systems functional before takeoff
    - **Kill switch**: Always have manual override capability
    - **Geofencing**: Configure safe operating boundaries
    - **Communication**: Maintain telemetry link throughout operation
    - **Emergency procedures**: Plan for failures (GPS loss, communication loss, etc.)
    - **Regulations**: Follow local aviation regulations and obtain necessary permits

## Hardware Configuration

### Robot Identity
Each robot needs unique configuration:
- Robot name/hostname
- ROS_DOMAIN_ID (for multi-robot operations)
- Network configuration
- Sensor calibration

See: [Robot Identity Configuration](../robot/docker/robot_identity.md)

### Network Setup
- WiFi or cellular connectivity for telemetry
- Optional 5G/LTE for high-bandwidth applications
- Local mesh networking for multi-robot teams

## Common Workflows

### Single Robot Field Mission
1. Power on robot and verify systems
2. Establish telemetry link
3. Run pre-flight checks
4. Arm and execute mission
5. Monitor via Ground Control Station
6. Land and collect data

### Multi-Robot Coordination
1. Configure unique ROS_DOMAIN_ID for each robot
2. Set up communication infrastructure
3. Launch Ground Control Station
4. Deploy robots sequentially
5. Monitor coordination via GCS
6. Manage data offload from multiple robots

## Autonomy Modes for Real World

AirStack supports multiple autonomy modes for different scenarios:

- **`onboard_all`**: All processing on robot (no ground station needed)
- **`onboard_local`**: Local planning onboard, global planning offboard
- **`offboard_global`**: Heavy computation on ground station

See: [Autonomy Modes Tutorial](../tutorials/autonomy_modes.md)

## Data Collection

### ROS Bag Recording
- Automatic recording of key topics
- Configurable topic selection
- Storage management on limited-capacity devices

See: [ROS Bags](../robot/logging/rosbags.md)

### Data Offloading
- Automatic sync to ground station or server
- Compression and transfer optimization
- Archive management

See: [Data Offloading](data_offloading/index.md)

## Troubleshooting

**No GPS fix**:
- Verify GPS antenna connection
- Check for interference
- Wait for satellite acquisition (can take several minutes)

**High latency on telemetry**:
- Check network signal strength
- Reduce publishing rates for non-critical topics
- Use compression for image data

**Poor localization**:
- Verify sensor calibration
- Check for sensor failures
- Ensure adequate visual features (for vision-based localization)

**Battery issues**:
- Monitor voltage and current draw
- Plan missions within battery capacity
- Consider cold weather effects on battery

## Resources

- [Installation on Hardware](installation/index.md)
- [HITL Testing](HITL/index.md)
- [Data Offloading](data_offloading/index.md)
- [Deploying to Hardware Tutorial](../tutorials/deploying_to_hardware.md)
- [Robot Configuration](../robot/configuration/index.md)

## Next Steps

- **New to hardware deployment?** Start with [Deploying to Hardware Tutorial](../tutorials/deploying_to_hardware.md)
- **Ready to install?** Follow [Installation Guide](installation/index.md)
- **Need to test safely?** Set up [HITL Testing](HITL/index.md)
- **Managing data?** Configure [Data Offloading](data_offloading/index.md)