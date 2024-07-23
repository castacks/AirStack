

Global planners output a high level, coarse trajectory for the robot to follow. 

A trajectory is a spatial path plus a schedule. 
This means each waypoint in the trajectory has a time associated with it, indicating when the robot should reach that waypoint.
These timestamps are fed to the local planner to determine speed and acceleration.

If a waypoint's header timestamp is empty, the local planner should assume there's no time constraint and follow the trajectory at its own pace.

The global planner should make a trajectory that is collision-free according to the global map.
However, avoiding fine obstacles is delegated to the local planner that operates at a faster rate.

## ROS Interfaces

### Actions Interface
The global planner should provide an action server for the topic `$(arg robot_name)/global_planner/plan` of message type `nav_msgs/GetPlan`.

```
```

### Subscribers
In general, the global planner needs access to components of the world model.

The most common one is the Occupancy Grids, which are published by the `map_server` node.

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



The global planner can do whatever it wants internally.