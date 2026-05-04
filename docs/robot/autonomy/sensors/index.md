We'll fill this with different things like the ZED-X package, LiDAR, etc

The `lidar_point_cloud_filter` package (under `robot/ros_ws/src/sensors/lidar_point_cloud_filter`) subscribes to a raw lidar `PointCloud2`, removes near-range and invalid (NaN or infinite) points for a cleaner cloud, and republishes for downstream consumers.

## Launch
Launch files are under `src/robot/autonomy/sensors/sensors_bringup/launch`.

The main launch command is `ros2 launch sensors_bringup sensors.launch.xml`.

