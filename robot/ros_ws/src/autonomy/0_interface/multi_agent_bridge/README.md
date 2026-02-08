# MULTI-AGENT-BRIDGE
A simple tool for distributed multi-agent system setup.
## ROS2 Topics
### Subscription:
- `$(ROBOT_NAME)/sensors/ouster/point_cloud`: Point Cloud from Ouster
- `$(ROBOT_NAME)/odometry_conversion/odometry`: Odometry data w.r.t. each robot’s map frame.

### Publication:
- `$(ROBOT_NAME)/global_pose`: Odometry data w.r.t global frame.
- `$(ROBOT_NAME)/sensors/filtered_pointcloud`: Filtered Point Cloud (ENABLE Masking Neighbor Agents)
- `neighbor_array_odom`: Neighbor Agents' Odometry
## User Parameters
- `local_map_frame_id`: Frame Id of Each Local Map Frame (e.g. "map")
- `local_mavros_frame_id`: Frame Id of MAVROS Frame (e.g "mavros_enu") 
- `global_map_frame_id`: Frame Id of Global Map Frame (e.g. "world")
- `neighbor_mask`: Choose whether mask neighbor agents in point cloud
- `x_size`: mask size (x-axis)
- `y_size`: mask size (y-axis)
- `z_size`: mask size (z-axis)
### Contact: yunwool@andrew.cmu.edu
