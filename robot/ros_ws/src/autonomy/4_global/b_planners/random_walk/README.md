# Random Walk Global Planner Baseline

The Random Walk Planner serves as a baseline global planner for stress testing system autonomy. Unlike more informed and intelligent planners, the Random Walk Planner generates a series of random trajectories to evaluate system robustness. Using the map published by VDB, the planner will generate and publish multiple linked straight-line trajectories, checking for collisions along these paths.

## Functionality

Upon activation by the behavior tree, the Random Walk Planner will:

1. Generate a specified number of straight-line path segments.
2. Continuously monitor the robot's progress along the published path.
3. Once the robot completes the current path, a new set of paths will be generated.

This loop continues, allowing the system to explore various trajectories and stress test the overall autonomy stack.

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



