# Interface Launch File Documentation

## Overview

This directory contains both XML and Python versions of the interface launch file:

- `interface.launch.xml` - Original XML launch file
- `interface.launch.py` - New Python launch file with dynamic FCU URL calculation

## Python Launch File Features

The Python launch file (`interface.launch.py`) implements the same functionality as the XML version but with the following enhancements:

### Dynamic FCU URL Calculation

The FCU URL for MAVROS is now calculated programmatically based on the `robot_id` (from `ROS_DOMAIN_ID` environment variable), following the same logic as the `px4_mavlink_backend.py` in the Pegasus simulator:

**Simulation Mode:**
- Connection format: `tcpin:localhost:PORT`
- Port calculation: `4560 + robot_id`
- Examples:
  - robot_id=0 → `tcpin:localhost:4560`
  - robot_id=1 → `tcpin:localhost:4561`
  - robot_id=2 → `tcpin:localhost:4562`

**Real Hardware Mode:**
- Connection: `/dev/ttyTHS4:115200` (unchanged from original)

### Usage

Launch the interface system:

```bash
# For simulation (sim=true)
ros2 launch interface_bringup interface.launch.py sim:=true

# For real hardware (sim=false, default)
ros2 launch interface_bringup interface.launch.py sim:=false

# With custom odometry topic
ros2 launch interface_bringup interface.launch.py \
    sim:=true \
    interface_odometry_in_topic:=/custom/robot/interface/mavros/local_position/odom
```

### Environment Variables Required

- `ROS_DOMAIN_ID`: Used as the robot_id for port calculation
- `ROBOT_NAME`: Used for topic remapping

### Integration with Pegasus Simulator

This launch file is designed to work seamlessly with the Pegasus Isaac Sim integration where:

1. Isaac Sim launches PX4 SITL instances with calculated ports
2. The PX4MavlinkBackend uses the same port calculation logic
3. MAVROS connects to the correct PX4 instance using the calculated FCU URL

### Node Configuration

The launch file starts the following nodes:

1. **MAVROS** - MAVLink communication with PX4
2. **robot_interface_node** - Robot interface abstraction
3. **position_setpoint_pub.py** - Position setpoint publisher
4. **odometry_conversion** - Converts odometry topics and publishes TF
5. **drone_safety_monitor** - Safety monitoring

All nodes maintain the same configuration and remapping as the original XML launch file.
