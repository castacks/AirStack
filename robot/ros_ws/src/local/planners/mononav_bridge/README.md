# MonoNav Bridge

This package connects AirStack's ROS 2 graph to the separate MonoNav GPU runtime.
The split preserves the dependency sets used by AirStack (ROS 2 Jazzy) and MonoNav
(PyTorch, ZoeDepth, and Open3D CUDA).

The bridge subscribes to the simulated left RGB camera, optional simulator depth,
its calibration, and ground-truth odometry. For every sampled image it looks up `map -> camera_left`, so the pose
sent to MonoNav is the optical-camera pose expected by Open3D (RDF axes), not merely
the vehicle body pose. A small HTTP endpoint exposes the newest JPEG, intrinsics, and
4x4 camera pose. MonoNav sends the selected primitive back as world-frame waypoints.

## Interfaces

- Inputs: `image`, `depth_image` (optional), `camera_info`, `odometry`
- Output: `trajectory_segment` (`airstack_msgs/TrajectoryXYZVYaw`)
- Visualization: `trajectory_markers` (`visualization_msgs/MarkerArray`)
- Diagnostics: `status` (`std_msgs/String`)
- Service client: `set_trajectory_mode` (`airstack_msgs/TrajectoryMode`)
- HTTP: `GET /health`, `GET /frame`, `POST /trajectory`, `POST /pause`

Flight execution is disabled by default. With `execute_commands:=false`, received
primitives are still shown as green markers but are not sent to the trajectory
controller.

The optional simulator-depth A/B baseline expects
`/robot_1/sensors/front_stereo/left/depth_ground_truth`. The bridge packages the latest
float32 metric depth with each HTTP frame; ZoeDepth-only runs can leave the depth topic
absent.

```bash
ros2 launch mononav_bridge mononav_bridge.launch.xml \
  mononav_bridge_execute_commands:=false
```
