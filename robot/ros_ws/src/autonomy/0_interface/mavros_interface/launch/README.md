# MAVROS Connection Polling Launch File

This directory contains a Python launch file that continuously polls for a mavlink connection and launches `px4.launch` once the connection is established.

## File Overview

### `mavros_connection_poll.launch.py` (OpaqueFunction Approach)

This implementation uses ROS2's `OpaqueFunction` to handle connection polling and dynamic launch actions.

**Features:**
- Comprehensive URL parsing (TCP, UDP, Serial)
- Configurable polling intervals and timeouts
- Built-in connection checking for different protocols
- Thread-based polling to avoid blocking the launch process
- Automatic MAVROS launch when connection is established

**Usage:**
```bash
export MAVROS_FCU_URL="tcpin:localhost:4560"
ros2 launch mavros_interface mavros_connection_poll.launch.py

# With custom max wait time
ros2 launch mavros_interface mavros_connection_poll.launch.py max_wait_time:=120.0

# With multiple custom arguments
ros2 launch mavros_interface mavros_connection_poll.launch.py \
    max_wait_time:=90.0 \
    polling_interval:=0.5
```

## Supported FCU URL Formats

All implementations support the following FCU URL formats:

### TCP Connections
- `tcpin:localhost:4560` - TCP input connection
- `tcp://localhost:4560` - Standard TCP URL format

### UDP Connections  
- `udp://localhost:14540` - Simple UDP connection
- `udp://:14540@172.31.0.200:14580` - UDP with local and remote ports

### Serial Connections
- `/dev/ttyTHS4:115200` - Serial device connection
- `/dev/ttyUSB0:57600` - USB serial connection

## Environment Variables

### Optional
- `ROS_DOMAIN_ID` - Used for robot identification
- `ROBOT_NAME` - Used for namespacing

## Launch Arguments

- `polling_interval` (default: 1.0s) - How often to check connection
- `max_wait_time` (default: 60s) - Maximum time in seconds to wait for mavlink connection
- `MAVROS_FCU_URL` - The FCU connection URL (see formats above)

## Integration with AirStack

This launch file is designed to work with the AirStack interface system:

1. **Set the FCU URL**: Configure `MAVROS_FCU_URL` in your robot's `.bashrc`
2. **Launch Interface**: Use this launch file instead of directly launching MAVROS
3. **Connection Handling**: The system will wait for the mavlink endpoint to be available
4. **Automatic Launch**: MAVROS will be launched automatically when connection is established

## Example Usage Scenarios

### Scenario 1: Pegasus Isaac Sim Integration
```bash
# Robot 0
export ROS_DOMAIN_ID=0
ros2 launch mavros_interface mavros_connection_poll.launch.py fcu_url:="tcpin:localhost:4560"

# Robot 1  
export ROS_DOMAIN_ID=1
export MAVROS_FCU_URL="tcpin:localhost:4561"
ros2 launch mavros_interface mavros_connection_poll.launch.py
```

### Scenario 2: Real Hardware
```bash
ros2 launch mavros_interface mavros_connection_poll.launch.py connection_timeout:=30.0 fcu_url:="/dev/ttyTHS4:115200"
```

### Scenario 3: SITL Development
```bash
ros2 launch mavros_interface mavros_connection_poll.launch.py fcu_url:="$MAVROS_FCU_URL"
```

## Troubleshooting

### Connection Issues
1. Verify `MAVROS_FCU_URL` is set correctly
2. Check that the target endpoint is reachable
3. For serial connections, verify device permissions
4. Increase `connection_timeout` for slow-starting systems

### Launch Issues
1. Ensure MAVROS package is installed
2. Check ROS2 workspace is sourced
3. Verify launch file permissions
4. Review ROS2 logs for detailed error messages
