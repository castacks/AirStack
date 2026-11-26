# Sensor and bridge smoke test

Used the existing single-robot scene and the earlier [ROS observer](../evidence/observe_ros.py) and [bridge observer](../evidence/observe_foxglove.py). No fixture or flight command ran. The ROS windows were approximately 12 seconds after PX4 startup.

| Stream | Prior repaired runtime, domain 1 | New digest, domain 1 |
|---|---:|---:|
| Clock samples | 238 | 283 |
| Left RGB8 frames | 234 | 280 |
| Left CameraInfo messages | 239 | 285 |
| Right 32FC1 depth frames | 233 | 275 |
| Converted odometry messages | 256 | 256 |
| VDB map points in last marker | 853 | 987 |

Left RGB and CameraInfo report 480 × 300 and `camera_left`. CameraInfo retains fx and fy about 184, cx 240 and cy 150. Depth reports 480 × 300 and `camera_right`. The raw RTX LiDAR topic returned a point cloud width of 12,091. Clock showed zero duplicate stamps and zero regressions in the steady window. MAVROS yielded eight state messages and reported connected and disarmed. The inter-run counts are observations of two live windows and do not establish matched throughput or geometry equivalence.

The GCS bridge on pod TCP port 8766 negotiated HTTP 101 and delivered 247 left RGB frames, 285 CameraInfo frames, 233 depth frames, 252 odometry messages, 24 map markers and static TF. Domain 0 still did not deliver clock or dynamic TF in these probes. External tunnel stability and visual accuracy were outside this smoke check.

[Robot-domain data](evidence/ros-domain-1.json) · [GCS-domain data](evidence/ros-domain-0.json) · [LiDAR sample](evidence/lidar-width.txt) · [WebSocket data](evidence/foxglove.json)
