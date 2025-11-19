# Exploration Planner

The Exploration Planner is an optional global planner for autonomous flight. It combines the maintaining of an openvdb based voxel occupancy grid map, extract frontier from the map, and generates trajectories that enables the drone to explore previously undiscovered areas. With VDB mapping integrated, you can turn off the vdb_mapping node in the launch xml, and the planner will generate and publish multiple linked straight-line trajectories as a shortened RRT path to a selected viewpoint with collision check.

## Functionality

You can comment out the world model and random walk planning module in `global.launch.xml` and add the following line:

`<include file="$(find-pkg-share exploration_planner)/launch/exploration_launch.xml" />`:

Then when running the robot stack, after taking off, click the `Global Plan` and the exploration planner will work in the place of previous random walk planner. The working process is:

1. Create and maintain a voxel grid map with odometry and laser scan, to visualize, check topic `"~/vdb_viz"`, where `~` is the namespace.
2. Extract frontier and select viewpoints for exploration.
3. Generate RRT path, shorten and publish as global plan.
4. Continuously monitor the robot's progress along the published path.
5. Once the robot completes the current path, a new exploration path will be generated.

This loop continues, enabling the drone to keep explore in the entire space.

We're still cleaning old params of random walk planner, based on which this exploration is developed. We'll update parameter documentation later.

## Parameters
| <div style="width:220px">Parameter</div>  | Description
|----------------------------|---------------------------------------------------------------
| `num_paths_to_generate`    | Number of straight-line paths to concatenate into a complete trajectory.|
| `max_start_to_goal_dist_m` | Maximum distance (in meters) from the start point to the goal point for each straight-line segment.|
| `checking_point_cnt`       | Number of points along each straight-line segment to check for collisions.|
| `max_z_change_m`           | Maximum allowed change in height (z-axis) between the start and goal points.|
| `collision_padding_m`      | Extra padding (in meters) added around a voxel's dimensions when checking for collisions.|
| `path_end_threshold_m`     | Distance threshold (in meters) for considering the current path completed and generating a new one.|
| `max_yaw_change_degrees`   | Maximum allowed change in angle (in radians) between consecutive straight-line segments to ensure a relatively consistent direction.|
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



