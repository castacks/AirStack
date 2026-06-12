# NatNet ROS 2 Wrapper

OptiTrack NatNet ROS 2 wrapper for motion capture integration in AirStack (optional). Receives rigid body pose data from an external Motive PC via NatNet UDP protocol and publishes into the AirStack perception layer.

**Note:** This module is only required if you intend to use OptiTrack Motive motion capture systems. If you do not plan to use OptiTrack, you can skip the NatNet SDK setup with `airstack setup --no-natnet`.

### OptiTrack room calibration

If rigid bodies are jumping around or not tracking well, consider re-calibrating the capture volume in Motive. See the [OptiTrack Motive calibration guide](https://docs.optitrack.com/motive/calibration).

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
    ├→ /robot_1/perception/optitrack/{body_name}           (PoseStamped, optional)
    ├→ /robot_1/perception/optitrack/{body_name}/pose_cov  (PoseWithCovarianceStamped, always)
    └→ (Optional, publish_to_mavros: true)
         mavros_gp_origin_node
           └→ /robot_1/interface/mavros/global_position/set_gp_origin  (once, guarded)
         vision_pose_converter_node
           ├→ /robot_1/interface/mavros/vision_pose/pose
           └→ /robot_1/interface/mavros/vision_pose/pose_cov
```

## Interfaces

### Inputs

- **Network**: NatNet UDP stream from Motive PC (external network)
- **Configuration**: `natnet_config.yaml` with server IP, ports, `body_name`, and covariance

### Outputs

For each tracked rigid body `{body_name}` from Motive:

#### Direct OptiTrack pose (optional)

- **Topic**: `/{ROBOT_NAME}/perception/optitrack/{body_name}`
- **Type**: `geometry_msgs/PoseStamped`
- **Description**: Position and orientation only (no covariance)
- **Enabled by**: `publish_direct_optitrack: true` in config (default: `true`)

#### Pose with covariance (always)

- **Topic**: `/{ROBOT_NAME}/perception/optitrack/{body_name}/pose_cov`
- **Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Description**: Same pose as above plus a 6×6 covariance matrix (`position_covariance` and `orientation_covariance` from config). Published whenever the rigid body is tracked — independent of `publish_direct_optitrack` and `publish_to_mavros`.

#### MAVROS vision pose bridge (optional)

When `publish_to_mavros: true`, `vision_pose_converter_node` subscribes to `pose_cov` and republishes for PX4:

- **Topic**: `/{ROBOT_NAME}/interface/mavros/vision_pose/pose` — `geometry_msgs/PoseStamped` (pose extracted from the covariance message)
- **Topic**: `/{ROBOT_NAME}/interface/mavros/vision_pose/pose_cov` — `geometry_msgs/PoseWithCovarianceStamped` (full message, quaternion optionally canonicalized)
- **Enabled by**: `publish_to_mavros: true` in config
- **PX4 side**: set `PX4_PARAM_PROFILE=vision` in `.env` so Isaac SITL loads EKF2 external-vision params from `simulation/isaac-sim/docker/px4-profiles/vision.env`

##### Synthetic GPS origin (mocap / no-GNSS arming)

With GNSS disabled (`EKF2_GPS_CTRL=0`), PX4 fuses vision into a valid *local*
position but has **no global position**. Modes that require one — such as
`AUTO.LOITER` — then fail preflight and refuse to arm (`Arming denied: Resolve
system health failures first`). When `publish_to_mavros: true`,
`mavros_gp_origin_node` publishes a synthetic origin once at startup:

- **Topic**: `/{ROBOT_NAME}/interface/mavros/global_position/set_gp_origin` — `geographic_msgs/GeoPointStamped`
- **Guarded**: waits for `mavros/state.connected`, then publishes only if no
  origin already exists (it watches `…/global_position/gp_origin`), so a
  GNSS-equipped vehicle is left untouched.
- **Params** (`config/mavros_gp_origin.yaml`): `enabled` (default `true`),
  `latitude/longitude/altitude` (default PX4 SITL home, Zurich), `settle_sec`.
  Set `enabled: false` to rely on real GNSS.

## Configuration

Edit `config/natnet_config.yaml`:

```yaml
/**:
  ros__parameters:
    server_ip: "192.168.1.100"     # IP of the Motive PC
    client_ip: "0.0.0.0"
    command_port: 1510
    data_port: 1511
    connection_type: "unicast"      # or "multicast"

    body_name: "Drone"              # rigid body name in Motive (case-sensitive)
    body_id: -1                     # -1 = publish all bodies in the frame

    publish_direct_optitrack: true   # PoseStamped on …/optitrack/{body_name}
    publish_to_mavros: false        # include vision_pose_converter → MAVROS

    frame_id: "world"

    position_covariance: [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
    orientation_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
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
- Topics: `/{ROBOT_NAME}/perception/optitrack/{body_name}` and `/{ROBOT_NAME}/perception/optitrack/{body_name}/pose_cov`
- Supported via launch file argument forwarding

### Error Handling
- Invalid/malformed packets are skipped with debug logging
- Lost connectivity logs warnings; gracefully recovers when stream resumes
- Covariance in config allows tuning uncertainty per deployment
- **Connect retry:** the initial handshake is retried every 2 s until it
  succeeds, so the node tolerates the NatNet server starting *after* the robot
  (e.g. a Motive PC powered on later, or the Isaac Sim NatNet emulator which only
  binds ~100 s into sim boot). The retry timer cancels itself on first success.

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
   ros2 topic echo /robot_1/perception/optitrack/Drone/pose_cov
   ```

### Without Real Hardware (Mock)
TODO: Implement Motive simulator in Isaac Sim to generate fake NatNet packets

## Known Limitations

- When `body_id: -1`, all rigid bodies in the Motive frame get publishers; filter by subscribing to the `{body_name}` you care about
- MAVROS bridge applies frame_id override and quaternion canonicalization; full PX4 frame alignment may still need tuning per airframe
- No support for skeleton tracking or labeled markers yet (future enhancement)

## References

- [OptiTrack NatNet Protocol Documentation](https://docs.optitrack.com/developer-tools/natnet-sdk/natnet-4.0)
- [NatNet SDK Download](https://optitrack.com/software/natnet-sdk/)
- [MAVROS Vision Pose Plugin](https://docs.ros.org/en/melodic/api/mavros_extras/html/classmavros_1_1extra__plugins_1_1VisionPoseEstimatePlugin.html)

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

**Note on NatNet SDK Licensing**: The OptiTrack NatNet SDK is proprietary software governed by the OptiTrack Software License Agreement. Users download and install the SDK locally under their own license compliance. AirStack does not redistribute the SDK and remains fully open-source.
