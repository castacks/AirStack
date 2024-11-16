# Random Walk Global Planner Baseline

The Random Walk Planner serves as a baseline global planner for stress testing system autonomy. Unlike more informed and intelligent planners, the Random Walk Planner generates a series of random trajectories to evaluate system robustness. Using the map published by VDB, the planner will generate and publish multiple linked straight-line trajectories, checking for collisions along these paths.

## Functionality

Upon activation by the behavior tree, the Random Walk Planner will:

1. Generate a specified number of straight-line path segments.
2. Continuously monitor the robot's progress along the published path.
3. Once the robot completes the current path, a new set of paths will be generated.

This loop continues, allowing the system to explore various trajectories and stress test the overall autonomy stack.

## Parameters

The following parameters can be adjusted to control the behavior of the Random Walk Planner:

`num_paths_to_generate`: Number of straight-line paths to concatenate into a complete trajectory.

`max_start_to_goal_dist_m`: Maximum distance (in meters) from the start point to the goal point for each straight-line segment.

`checking_point_cnt`: Number of points along each straight-line segment to check for collisions.

`max_z_change_m`: Maximum allowed change in height (z-axis) between the start and goal points.

`collision_padding_m`: Extra padding (in meters) added around a voxel's dimensions when checking for collisions.

`path_end_threshold_m`: Distance threshold (in meters) for considering the current path completed and generating a new one.

`max_z_angle_change_rad`: Maximum allowed change in angle (in radians) between consecutive straight-line segments to ensure a relatively consistent direction.



