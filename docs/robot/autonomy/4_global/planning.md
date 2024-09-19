# Global Planning

Global planners output a high level, coarse trajectory for the robot to follow. 

A trajectory is a spatial path plus a schedule. 
This means each waypoint in the trajectory has a time associated with it, indicating when the robot should reach that waypoint.
These timestamps are fed to the local planner to determine speed and acceleration.

If a waypoint's header timestamp is empty, the local planner should assume there's no time constraint and follow the trajectory at its own pace.

The global planner should make a trajectory that is collision-free according to the global map.
However, avoiding fine obstacles is delegated to the local planner that operates at a faster rate.

For the structure of the package, the global planner node should not include any logic to generate the path. This should be located in a seperate logic class and be seperated from ROS. This will allow more modularity in the future for testing and easy interface changes.

## ROS Interfaces

### Actions Interface
The global planner should provide an action server for the topic `$(arg robot_name)/global_planner/plan` of message type `nav_msgs/GetPlan`. This should be triggered from the base station and should trigger a path generation based off a fed goal position and optionally a tolerance to the final goal, otherwise a default tolerance will be used. This will then trigger the global planner to generate a path and publish it to the gloabal trajectory topic. 

```
# The final pose of the goal position
geometry_msgs/PoseStamped goal

# How many meters the drone can be from the goal
float32 tolerance
---
# Response from the generated gloabl trajectory
nav_msgs/Path plan 
```

### Subscribers
In general, the global planner needs access to components of the world model.

There are two main requirments for the global planner to operate. The first essential Subscriber is the Occupancy Grids, which are published by the `map_server` node. This gives the global planner a global map to reference for planning, and should be agnostic to the robot. 

The second essential subscriber should be the odometry or pose of the specific robot it is planning for in relation to the gloabl map. This would be found on the topic `$(arg robot_name)/controls/robot_interface/odometry`. The global planning node should not start to plan without first recieving messages from these two topics, and should trigger a boolean to enable gloabl planning.

### Publishers

The global planner must publish a trajectory to the topic `$(arg robot_name)/global_planner/trajectory` of message type `nav_msgs/Path`.

The header of the `nav_msgs/Path` message should contain the coordinate frame of the trajectory, and the timestamp should indicate when the trajectory was generated.
Each waypoint's header should contain the coordinate frame of the waypoint, and the timestamp should indicate when the waypoint should be reached.

```
nav_msgs/Path.msg
    - std_msgs/Header header
        - time stamp: when the trajectory was generated
        - frame_id: the coordinate frame of the trajectory
    - geometry_msgs/PoseStamped[] poses: the trajectory
        - geometry_msgs/PoseStamped pose
            - std_msgs/Header header
                - time stamp: when the waypoint should be reached
                - string frame_id: the coordinate frame of the waypoint
            - geometry_msgs/Pose pose: the position and orientation of the waypoint
```

## Global Planners
### Random Walk Planner

The random walk planner is a naive global planning method as a stand in for a more intelligent global planner with set objectives. Currently, the planner randomly selects a goal point, constrained by the proximity to the start point and must not be located within a grid from the global map. Once a goal point is selected, intermediate points are checked for collision to ensure the path is free. This is done until a straight collision free path is created. The following parameters are taken in to specify the behavior of the random walk:

```
publish_visualizations: true # should trajectory visualizations be published

# Constants
max_start_to_goal_dist_m: 3.5 # how max distance the goal can be from the current robot position
checking_point_cnt: 1000 # how many points to collision check between the start and goal points
max_z_m: 0.1 # max height that the goal can deviate from the current height
collision_padding_m: 0.1 # how much space should the path have to any obstacles
path_end_threshold_m: 0.1 # how close to the goal will the path be considered complete
```

The global planner can do whatever it wants internally.


## Example Planners

### Random Walk planner

The random walk planner replans when the robot is getting close to the goal. The random walk planner is a trivial planner that generates a plan by randomly selecting a direction to move in. The random walk planner is useful for testing the robot's ability to follow a plan.
