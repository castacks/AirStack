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
- **Tracks multiple rigid bodies per robot** (e.g. a drone for state estimation plus a separate target), each mapped to its own topic
- **Supports multi-robot** via per-robot profiles selected by `ROBOT_NAME`
- **Optionally bridges** to MAVROS for PX4 external pose feedback (per-robot)
- **Respects OptiTrack licensing** by keeping the NatNet SDK external (host-side download with explicit consent)

## Architecture

```
Motive (External PC)
    ↓ NatNet UDP (port 1511)
    ↓
NatNet ROS 2 Node  (loads the ROBOT_NAME profile from natnet_config.yaml)
    │  per configured body (one or more):
    ├→ /{ROBOT_NAME}/{topic}           (PoseStamped, when pose: true)
    ├→ /{ROBOT_NAME}/{topic}/pose_cov  (PoseWithCovarianceStamped, when pose_cov: true)
    └→ (Optional, vision_pose.enabled: true)
         mavros_gp_origin_node
           └→ /{ROBOT_NAME}/interface/mavros/global_position/set_gp_origin
         px4_param_setter_node
           └→ /{ROBOT_NAME}/interface/mavros/param/set (external-vision PX4 params)
         vision_pose_converter_node   (reads input/output topics from the profile)
           ├→ /{ROBOT_NAME}/interface/mavros/vision_pose/pose
           └→ /{ROBOT_NAME}/interface/mavros/vision_pose/pose_cov
```

## Interfaces

### Inputs

- **Network**: NatNet UDP stream from Motive PC (external network)
- **Configuration**: `natnet_config.yaml` — generic `server` settings plus a `robots` map of per-robot profiles (body list + optional MAVROS `vision_pose` block). The launch file selects the profile matching `ROBOT_NAME`.

### Outputs

For each rigid body in the robot's profile, `topic` is a **relative** leaf namespaced
under `/{ROBOT_NAME}/` (it defaults to `perception/optitrack/{rigid_body_name}` when
omitted):

#### Direct OptiTrack pose

- **Topic**: `/{ROBOT_NAME}/{topic}`
- **Type**: `geometry_msgs/PoseStamped`
- **Description**: Position and orientation only (no covariance)
- **Enabled by**: `pose: true` on that body (per body)

#### Pose with covariance

- **Topic**: `/{ROBOT_NAME}/{topic}/pose_cov`
- **Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Description**: Same pose plus a 6×6 covariance matrix from that body's `position_covariance` / `orientation_covariance`.
- **Enabled by**: `pose_cov: true` on that body (per body)

#### MAVROS vision pose bridge (optional, per robot)

When the robot's `vision_pose.enabled: true`, `vision_pose_converter_node` subscribes to the configured `input_topic` (a body's `pose_cov`) and republishes for PX4 on the configured outputs:

- **Topic** (`output_pose_topic`): `/{ROBOT_NAME}/interface/mavros/vision_pose/pose` — `geometry_msgs/PoseStamped` (pose extracted from the covariance message)
- **Topic** (`output_pose_cov_topic`): `/{ROBOT_NAME}/interface/mavros/vision_pose/pose_cov` — `geometry_msgs/PoseWithCovarianceStamped` (full message, quaternion optionally canonicalized)
- **Enabled by**: `vision_pose.enabled: true` in the robot's profile
- **Retargetable**: change `input_topic` / `output_pose_topic` / `output_pose_cov_topic` (relative, namespaced) to bridge to other middleware
- **PX4 side**: set `SITL_PARAM_PROFILE=px4-vision` in `.env` so Isaac SITL loads EKF2 external-vision params from `simulation/isaac-sim/docker/sitl-files/px4-vision.env`

##### Synthetic GPS origin (mocap / no-GNSS arming)

With GNSS disabled (`EKF2_GPS_CTRL=0`), PX4 fused EKF has **no global position**. This fails preflight checks and refuse to arm. When `vision_pose.enabled: true`,
`mavros_gp_origin_node` publishes a synthetic origin once at startup:

- **Topic**: `/{ROBOT_NAME}/interface/mavros/global_position/set_gp_origin` — `geographic_msgs/GeoPointStamped`
- **Guarded**: waits for `mavros/state.connected`, then publishes only if no
  origin already exists (it watches `…/global_position/gp_origin`), so a
  GNSS-equipped vehicle is left untouched.
- **Params** (`config/mavros_gp_origin.yaml`): `enabled` (default `true`),
  `latitude/longitude/altitude` (default Lisbon — the AirStack shared world
  datum; **must match** the GCS origin in `gcs_visualizer/gcs_utils.py` and the
  sim's `gps_utils.py` so Foxglove waypoints transform 1:1), `settle_sec`.
  Set `enabled: false` to rely on real GNSS.

##### PX4 parameter enforcement (external-vision EKF2 setup)

When `vision_pose.enabled: true`, `px4_param_setter_node` pushes the PX4
parameter set for OptiTrack-only flight through the MAVROS param plugin at
startup, so the FCU doesn't need manual QGroundControl configuration:

- **Services used**: `/{ROBOT_NAME}/interface/mavros/param/get_parameters`
  (read current), `…/param/set` (`mavros_msgs/ParamSetV2`, set + verify readback)
- **Idempotent**: waits for `mavros/state.connected` + `settle_sec` (initial
  param-table pull), reads each param first, and skips ones already correct —
  PX4 persists parameters, so subsequent boots are a verify-only pass.
- **Reboot warning**: if any parameter actually changed, it logs a warning to
  reboot the FCU before flight so EKF2 restarts with a clean fusion config.
- **Params** (`config/px4_params.yaml`): `enabled`, `settle_sec`,
  `retry_period_sec`, `max_attempts`, and the `params.*` map of desired FCU
  values — external-vision fusion (`EKF2_EV_CTRL: 11`, `EKF2_HGT_REF: 3`),
  GPS/mag/baro disabled (`EKF2_GPS_CTRL: 0`, `EKF2_MAG_TYPE: 5`,
  `EKF2_BARO_CTRL: 0`), measured vision delay (`EKF2_EV_DELAY: 6.0` ms), and
  EV noise floors (`EKF2_EV_NOISE_MD: 1`, `EKF2_EVP_NOISE`, `EKF2_EVA_NOISE`).
  YAML type selects the MAVLink param type: write floats with a decimal point
  (`6.0`), integers bare. Values assume PX4 ≥ 1.14; for older firmware use
  `EKF2_AID_MASK: 24` / `EKF2_HGT_MODE: 3` instead.

## Configuration

`config/natnet_config.yaml` uses a custom `natnet:` schema (not a flat ROS 2 param
file): generic `server` settings shared by every agent, then a `robots` map of
per-robot profiles. The launch file parses it, selects the profile matching the
container's `ROBOT_NAME`, flattens the body list into node parameters, and brings up
the MAVROS bridge only when that robot's `vision_pose.enabled` is true.

```yaml
natnet:
  server:                              # generic across all agents
    server_ip: "$(env NATNET_SERVER_IP 172.31.0.200)"
    client_ip: "0.0.0.0"
    command_port: 1510
    data_port: 1511
    connection_type: "unicast"         # or "multicast"
    multicast_address: "239.255.42.99"
    frame_id: "world"
    debug: false
  robots:
    robot_1:
      vision_pose:                     # per-robot MAVROS bridge (omit/false to skip)
        enabled: true
        input_topic:           "perception/optitrack/drone/pose_cov"
        output_pose_topic:     "interface/mavros/vision_pose/pose"
        output_pose_cov_topic: "interface/mavros/vision_pose/pose_cov"
      bodies:                          # one or more tracked rigid bodies
        - rigid_body_name: "Drone"     # Motive name (case-sensitive)
          id: 1                        # Motive streaming id
          topic: "perception/optitrack/drone"  # relative → /{ROBOT_NAME}/<topic>
          pose: true                   # publish PoseStamped
          pose_cov: true               # publish PoseWithCovarianceStamped
          position_covariance:    [1.0e-6, 0, 0, 0, 1.0e-6, 0, 0, 0, 1.0e-6]
          orientation_covariance: [3.0e-6, 0, 0, 0, 3.0e-6, 0, 0, 0, 3.0e-6]
```

To track an additional body (e.g. a target) for a robot, add another entry under that
robot's `bodies`. To add a robot, add a new key under `robots`. The shipped file
includes commented scaffolding for a 3-drone fleet where `robot_1` and `robot_2` also
track a shared `Target` body and `robot_3` tracks only its drone.

## Launch

### Basic launch

Parameters come from `config/natnet_config.yaml` (server + the `ROBOT_NAME` profile). Optional overrides:

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py \
  config_file:=/path/to/custom_natnet.yaml \
  vision_pose_config_file:=/path/to/custom_vision_pose.yaml \
  use_sim_time:=true
```

### MAVROS bridge

Set `vision_pose.enabled: true` in the robot's profile. The launch file includes `vision_pose_converter.launch.xml` (plus `mavros_gp_origin.launch.xml` and `px4_param_setter.launch.xml`) and forwards the profile's `input_topic` / `output_pose_topic` / `output_pose_cov_topic`.

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
- The node loads the `robots[$ROBOT_NAME]` profile, so each robot tracks only the bodies (and runs the MAVROS bridge) configured for it.
- Topics are namespaced under `/{ROBOT_NAME}/` from each body's relative `topic`.
- Set `NUM_ROBOTS=N`; each replica resolves its own `ROBOT_NAME` (via `resolve_robot_name.py`) and auto-selects its profile — no per-robot env overrides.

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
4. Verify topics (default profile maps the `Drone` body to `perception/optitrack/drone`):
   ```bash
   ros2 topic echo /robot_1/perception/optitrack/drone/pose_cov
   ```

### Without Real Hardware (Mock)
TODO: Implement Motive simulator in Isaac Sim to generate fake NatNet packets

## Known Limitations

- The node publishes only bodies listed in the robot's profile (matched by `id`); bodies streamed by Motive but absent from the profile are ignored.
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
