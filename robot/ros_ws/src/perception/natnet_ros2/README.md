# NatNet ROS 2 Wrapper

OptiTrack NatNet ROS 2 wrapper for motion capture integration in AirStack (optional). Receives rigid body pose data from an external Motive PC via NatNet UDP protocol and publishes into the AirStack perception layer.

**Note:** This module is only required if you intend to use OptiTrack Motive motion capture systems. If you do not plan to use OptiTrack, you can skip the NatNet SDK setup with `airstack setup --no-natnet`.

## Overview

This module provides a bridge between OptiTrack Motive motion capture systems and the AirStack autonomy stack. It:

- Receives **NatNet UDP packets** from an external Motive PC (configurable IP/port)
- **Decodes motion capture frames** containing rigid body positions and orientations
- **Publishes pose data** to the AirStack perception layer in standard ROS 2 formats
- **Supports multi-robot** via ROBOT_NAME namespacing
- **Optionally bridges** to MAVROS for PX4 external pose feedback
- **Respects OptiTrack licensing** by keeping the NatNet SDK external (host-side download with explicit consent)

## Architecture

```
Motive (External PC)
    ↓ NatNet UDP (port 1511)
    ↓
NatNet ROS 2 Node
    ├→ /robot_1/perception/optitrack/{body_name}    (PoseStamped)
    ├→ /robot_1/perception/optitrack/pose_cov       (PoseWithCovarianceStamped)
    └→ (Optional) /robot_1/mavros/vision_pose/pose  (for PX4)
```

## Interfaces

### Inputs

- **Network**: NatNet UDP stream from Motive PC (external network)
- **Configuration**: `natnet_config.yaml` with server IP, port, body names, covariance

### Outputs

#### Direct OptiTrack Poses
- **Topic**: `/{ROBOT_NAME}/perception/optitrack/{body_name}` (PoseStamped)
- **Type**: `geometry_msgs/PoseStamped`
- **Description**: Individual rigid body poses
- **Enabled by**: `publish_direct_optitrack: true` in config

#### Aggregated Pose with Covariance
- **Topic**: `/{ROBOT_NAME}/perception/optitrack/pose_cov` (PoseWithCovarianceStamped)
- **Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Description**: Primary pose with uncertainty
- **Enabled by**: `publish_to_mavros: true` or always available

#### MAVROS Vision Pose Bridge (Optional)
- **Topic**: `/{ROBOT_NAME}/mavros/vision_pose/pose` (PoseStamped)
- **Type**: `geometry_msgs/PoseStamped` (converted for MAVROS/PX4)
- **Description**: External pose feedback for PX4 EKF fusion
- **Enabled by**: `publish_to_mavros: true` in config

## Configuration

Edit `config/natnet_config.yaml`:

```yaml
natnet:
  # Motive PC network settings
  server_ip: "192.168.1.1"        # IP of Motive PC
  data_port: 1511                # Local UDP port bound for NatNet data stream
  
  # Rigid body tracking
  body_names:
    - "quad_1"
    - "marker_cloud_1"
  
  # Publishing behavior
  publish_direct_optitrack: true   # Publish individual body topics
  publish_to_mavros: false         # Bridge to MAVROS for PX4
  
  # Frame IDs
  frame_id: "world"
  child_frame_id: "base_link"
  
  # Covariance (default uncertainty)
  position_covariance: [0.1, 0.0, 0.0, ...]
  orientation_covariance: [0.01, 0.0, 0.0, ...]
```

## Launch

### Basic launch

Parameters come from `config/natnet_config.yaml` (network, body, covariance). Optional overrides:

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py \
  config_file:=/path/to/custom_natnet.yaml \
  vision_pose_config_file:=/path/to/custom_vision_pose.yaml \
  use_sim_time:=true
```

### MAVROS bridge

Set `publish_to_mavros: true` in `natnet_config.yaml`. The launch file reads `publish_to_mavros` and `body_name` from that YAML to decide whether to include `vision_pose_converter.launch.xml`.

### From perception bringup

With `LAUNCH_NATNET=true` in `.env`, `perception.launch.xml` includes `natnet_ros2.launch.py`.

## Dependencies

### Runtime
- `rclpy` — ROS 2 Python client
- `geometry_msgs` — Standard pose message types
- `tf_transformations` — Quaternion and rotation utilities
- `mavros_msgs` — Optional, for MAVROS bridge

### Required
- **OptiTrack NatNet SDK** (Linux SDK) — **REQUIRED**, downloaded via `airstack setup`

### Installation
To install the NatNet SDK and accept the license:
```bash
airstack setup
```
The SDK will be installed into `robot/ros_ws/src/perception/natnet_ros2/lib/` and `robot/ros_ws/src/perception/natnet_ros2/include/natnet/` after accepting the OptiTrack License Agreement.

## Implementation Details

### Protocol Support
- **NatNet Version**: 4.4+ (SDK handles protocol negotiation)
- **Packet Type**: Frame of Data with rigid bodies and markers
- **Transport**: UDP (configurable port, default 1511)
- **SDK**: OptiTrack NatNet SDK handles all protocol parsing

### Multi-Robot Support
Each container instance gets its own `ROBOT_NAME` and `ROS_DOMAIN_ID`:
- Topic published as: `/{ROBOT_NAME}/perception/optitrack/{body_name}`
- Supported via launch file argument forwarding

### Error Handling
- Invalid/malformed packets are skipped with debug logging
- Lost connectivity logs warnings; gracefully recovers when stream resumes
- Covariance in config allows tuning uncertainty per deployment

## Testing

### With Real Motive
1. Ensure Motive PC and robot are on same network
2. Configure server IP in `natnet_config.yaml`
3. Launch the node:
   ```bash
   ros2 launch natnet_ros2 natnet_ros2.launch.py
   ```
4. Verify topics:
   ```bash
   ros2 topic echo /robot_1/perception/optitrack/pose_cov
   ```

### Without Real Hardware (Mock)
TODO: Implement Motive simulator in Isaac Sim to generate fake NatNet packets

## Known Limitations

- Rigid body body name mapping currently uses generic `body_N` naming; TODO: load from config
- MAVROS bridge converter is a stub; needs coordinate frame transformation for PX4
- No support for skeleton tracking or labeled markers yet (future enhancement)

## References

- [OptiTrack NatNet Protocol Documentation](https://v20.wiki.optitrack.com/index.php?title=NatNet)
- [NatNet SDK Download](https://optitrack.com/software/natnet-sdk/)
- [MAVROS Vision Pose Plugin](https://github.com/mavlink/mavros/tree/master/mavros_plugins#vision_pose_estimation)

## Troubleshooting

### No data being received
- Check Motive PC IP address in config
- Verify UDP port is not blocked by firewall
- Use `ros2 topic hz` to check if data is arriving

### Topics not published
- Check `ros2 node list` — should see `natnet_ros2_node`
- Check `ros2 topic list | grep optitrack` — should see published topics
- Look at logs: `ros2 node info natnet_ros2_node`

### Low frame rate or dropped frames
- Reduce other network traffic
- Check NatNet streaming rate in Motive (default 120 Hz)
- Monitor CPU usage: `docker stats`

## License

Apache 2.0 — See [LICENSE](../../../LICENSE)

**Note on NatNet SDK Licensing**: The OptiTrack NatNet SDK is proprietary software governed by the OptiTrack Software License Agreement. Users download and install the SDK locally under their own license compliance. AirStack does not redistribute the SDK and remains fully open-source.
