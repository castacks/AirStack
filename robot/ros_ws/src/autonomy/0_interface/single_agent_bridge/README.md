# SINGLE-AGENT-BRIDGE
A simple tool for distributed multi-agent system setup.
## ROS2 Topics
### Subscription:
- `$(ROBOT_NAME)/odometry_conversion/odometry`: Odometry data w.r.t. each robot’s map frame.
### Publication:
- `$(ROBOT_NAME)/global_pose`: Odometry data w.r.t global frame.
- `tf`: From `local_mavros_frame_id` to `local_map_frame_id`
## User Parameters
- `local_map_frame_id`: Frame Id of Each Local Map Frame (e.g. "map")
- `local_mavros_frame_id`: Frame Id of MAVROS Frame (e.g "mavros_enu") 
- `global_map_frame_id`: Frame Id of Global Map Frame (e.g. "world")
### Contact: yunwool@andrew.cmu.edu