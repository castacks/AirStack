
# State Estimation

Currently, the state estimation of our robot relies on [MAC-VO](https://mac-vo.github.io/), a learning-based sstereo visual odometry. This is purely camera based, and does not rely on additional sensors. On initialization, the node will load the model weights, and then allocate the required memory to store the model on first inference. This process may take some time. Once this process is complete, the inference time should be able to run at around 3 Hz. Documentation on the MAC-VO model can be found [here](https://mac-vo.github.io/wiki/category/extend-mac-vo/)

# Structure

The wrapper that is currently used for interfacing with the non-ROS MAC-VO logic is modified from the one provided [here](https://github.com/MAC-VO/MAC-VO-ROS2). For our purposes, we wanted modularity in how we can interface with the node, and so there is now two configuration files: 

- `interface_config.yaml`: This file specifies the desired camera name, the subscriber and publisher topics, and the size of the image when being fed through inference. This was designed specifically for Airstack
- `model_config.yaml`: This file is taken from the official MAC-VO ROS wrapper, and specifies the structure of how the MAC-VO model is created. It also specifies the location of the model weights, which are currently stored in /root/model_weights/MACVO_FrontendCov.pth in the docker.

## Parameters
| <div style="width:220px">Parameter</div>  | Description
|----------------------------|---------------------------------------------------------------
| `num_paths_to_generate`    | Number of straight-line paths to concatenate into a complete trajectory.|
| `max_start_to_goal_dist_m` | Maximum distance (in meters) from the start point to the goal point for each straight-line segment.|
| `checking_point_cnt`       | Number of points along each straight-line segment to check for collisions.|
| `max_z_change_m`           | Maximum allowed change in height (z-axis) between the start and goal points.|
| `collision_padding_m`      | Extra padding (in meters) added around a voxel's dimensions when checking for collisions.|
| `path_end_threshold_m`     | Distance threshold (in meters) for considering the current path completed and generating a new one.|
| `max_z_angle_change_rad`   | Maximum allowed change in angle (in radians) between consecutive straight-line segments to ensure a relatively consistent direction.|
| `robot_frame_id`           | The frame name for the robot's base frame to look up the transform from the robot position to the world.|

## Services
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/global_plan_toggle`     | std_srvs/Trigger | A toggle switch to turn on and off the random walk planner.|

## Subscriptions
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/sub_map_topic`     | visualization_msgs/Marker | Stores the map representation that is output from the world or local map topic; currently using vdb local map.|
| `~/tf`                | geometry_msgs/TransformStamped | Stores the transform from the robot to the world.|

## Publications
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/pub_global_plan_topic`  | nav_msgs/Path | Outputs the global plan that is generated from the random walk planner.|