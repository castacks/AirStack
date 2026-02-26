[//]: # "global"
# Planning

![global_trajectory_diagram](global_trajectory.png)

Global planners output a high level, coarse trajectory for the robot to follow. 

A **trajectory** is a spatial path plus a schedule. 
This means each waypoint in the trajectory has a time associated with it, indicating when the robot should reach that waypoint.
These timestamps are fed to the local planner and controller to determine velocity and acceleration.

If a waypoint's header timestamp is empty, the local planner should assume there's no time constraint and follow the trajectory at its own pace.

The global planner should make a trajectory that is collision-free according to the global map.
However, avoiding fine obstacles is delegated to the local planner that operates at a faster rate.

For the structure of the package, the global planner node should not include any logic to generate the path. This should be located in a seperate logic class and be seperated from ROS. This will allow more modularity in the future for testing and easy interface changes.

We intend the global planners to be modular. _AirStack_ implements a basic Random Walk planner as a baseline. 
Feel free to implement your own through the following interfaces.

## ROS Interfaces

Global planners are meant to be modules that can be swapped out easily. 
They can be thought of as different high level behaviors for the robot to follow.
Consider that multiple global planners may be run in parallel, for example by some ensemble planner node that chooses the best plan for the current situation.

As such, the global planner should be implemented as a ROS2 node that accepts runtime mission parameters in a custom `PlanRequest.msg` and 
publishes a plan to its local `~/global_plan` topic.

The best global plan should then be forwarded or remapped to `/$(env ROBOT_NAME)/global_plan` for the local planner to follow.

``` mermaid
sequenceDiagram
  autonumber
  Global Manager->>Global Planner: ~/plan_request (your_planner/PlanRequest.msg)
  loop Planning
      Global Planner-->>Global Manager: heartbeat feedback
  end
  Global Planner->>Global Manager: ~/global_plan (nav_msgs/Path.msg)
  Global Manager->>Local Planner: /$ROBOT_NAME/global_plan_reference (nav_msgs/Path.msg)
  Local Planner->>Global Manager: /$ROBOT_NAME/global_plan_eta (nav_msgs/Path.msg)
```

### Subscribe: Plan Request
Your custom `PlanRequest.msg` defines the parameters that your global planner needs to generate a plan. 
It will be sent on the `~/plan_request` topic.

Some common parameters may be the following:
```
# PlanRequest.msg
std_msgs/Duration timeout  # maximum time to spend planning
geometry_msgs/Polygon bounds # boundary that the plan must stay within
```


### Publish: Global Plan
The global planner must publish a message of type `nav_msgs/Path` to `~/global_plan`.
The message defines high level waypoints to reach by a given time.

The `nav_msgs/Path` message type contains a `header` field and `poses` field.

- The top level header of `nav_msgs/Path` message should contain the coordinate frame of the trajectory, and its timestamp should indicate when the trajectory was published.
- Within the `poses` field, each `geometry_msgs/PoseStamped`'s header should contain a timestamp that indicates when that waypoint should be reached

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
### Publish: Heartbeat
For long-running global planners, it's recommended to publish a heartbeat message to `~/heartbeat`. This way the calling node can know that the global planner is still running and hasn't crashed.

### Additional Subscribers
In general, the global planner needs to access components of the world model such as the map and drone state.

The most common map is Occupancy Grids that is published by {==TODO==} node.

The global planner can also access the robot's current state and expected state in the future. For example, if the global planner takes 20 seconds to plan a trajectory, 
it can query where the robot expects to be in 20 seconds. This ROS2 service is available under {==TODO==}.

The global planner can do whatever it wants internally with this information.

## Example Planners

### Random Walk planner

The random walk planner replans when the robot is getting close to the goal. The random walk planner is a trivial planner that generates a plan by randomly selecting a direction to move in. The random walk planner is useful for testing the robot's ability to follow a plan.

