# Simple Global Navigator

A ROS2 global planner implementation that handles NavigationTask actions using RRT* algorithm for cost-optimal path planning.

## Features

- **PointCloud2 Cost Map Integration**: Subscribes to 3D cost maps where intensity values encode traversal costs
- **RRT* Path Planning**: Implements RRT* algorithm for cost-optimal path planning in 3D space
- **Multi-Goal Navigation**: Supports sequential navigation through multiple goal poses
- **Real-time Feedback**: Monitors robot odometry and provides progress feedback
- **Configurable Parameters**: Tunable RRT* parameters for different environments

## Architecture

### Core Components

1. **Cost Map Subscriber**: Processes `sensor_msgs/PointCloud2` messages with intensity-based costs
2. **RRT* Planner**: Implements sampling-based path planning with cost optimization
3. **Odometry Monitor**: Tracks robot position and determines current navigation goal
4. **Action Server**: Handles `NavigationTask` requests and provides feedback

### Key Classes

- `SimpleGlobalNavigator`: Main node class implementing the action server
- `RRTNode`: Tree node structure for RRT* algorithm
- `CostMapData`: Container for processed cost map information

## Topics

### Subscribed Topics

- `/cost_map` (`sensor_msgs/PointCloud2`): 3D cost map with intensity-encoded costs
- `/odom` (`nav_msgs/Odometry`): Robot odometry for position tracking

### Published Topics

- `/global_plan` (`nav_msgs/Path`): Computed global path to current goal

### Action Interface

- `/simple_navigator` (`task_msgs/NavigationTask`): Main navigation action interface

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rrt_max_iterations` | int | 5000 | Maximum RRT* iterations |
| `rrt_step_size` | double | 0.5 | Step size for tree expansion |
| `rrt_goal_tolerance` | double | 0.3 | Distance tolerance for goal reaching |
| `rrt_rewire_radius` | double | 1.0 | Radius for RRT* rewiring |
| `cost_map_topic` | string | "/cost_map" | Cost map topic name |
| `odom_topic` | string | "/odom" | Odometry topic name |

## Usage

### Launch the Node

```bash
ros2 launch simple_global_navigator simple_global_navigator.launch.py
```

### With Custom Parameters

```bash
ros2 launch simple_global_navigator simple_global_navigator.launch.py \
    rrt_max_iterations:=10000 \
    rrt_step_size:=0.3 \
    cost_map_topic:=/my_cost_map
```

### Send Navigation Goals

```bash
# Example using action client
ros2 action send_goal /simple_navigator task_msgs/action/NavigationTask \
    "{goal_poses: [{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 1.0}}}]}"
```

## Algorithm Details

### RRT* Implementation

1. **Sampling**: Random points sampled within cost map bounds
2. **Nearest Neighbor**: Efficient nearest node search in tree
3. **Steering**: Constrained expansion with configurable step size
4. **Collision Checking**: Line-of-sight validation using cost map
5. **Rewiring**: Cost optimization through tree restructuring
6. **Goal Bias**: Implicit goal-directed sampling

### Cost Integration

- **Point Costs**: Intensity values from PointCloud2 directly used as traversal costs
- **Path Costs**: Accumulated distance + point costs along trajectory
- **Collision Threshold**: High-cost points (>0.8) treated as obstacles

### Multi-Goal Handling

- **Sequential Processing**: Goals processed in order
- **Progress Tracking**: Current goal index updated based on proximity
- **Distance Calculation**: Remaining distance computed across all pending goals

## Dependencies

- ROS2 Humble
- rclcpp & rclcpp_action
- sensor_msgs, nav_msgs, geometry_msgs
- tf2 & tf2_geometry_msgs
- task_msgs (custom action definitions)

## Build Instructions

```bash
cd /path/to/ros_ws
colcon build --packages-select simple_global_navigator
source install/setup.bash
```

## Integration Notes

### Cost Map Requirements

- **Format**: `sensor_msgs/PointCloud2` with fields: x, y, z, intensity
- **Intensity Range**: 0.0 (free) to 1.0 (obstacle), >0.8 considered impassable
- **Coordinate Frame**: Should match odometry frame (typically 'map')

### Action Interface

The planner implements the `NavigationTask` action defined in `task_msgs`:

```
# Goal
geometry_msgs/PoseStamped[] goal_poses
float64 max_planning_seconds
string constraints

---
# Result  
bool success
string message
geometry_msgs/PoseStamped final_pose
float64 distance_traveled

---
# Feedback
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
float64 time_elapsed
string current_phase
```

## Performance Considerations

- **Computational Complexity**: O(n log n) per iteration for RRT*
- **Memory Usage**: Scales with tree size (max_iterations)
- **Real-time Performance**: 10Hz feedback rate, planning runs in separate thread
- **Scalability**: Suitable for environments up to ~100m³ with default parameters

## Future Enhancements

- **Adaptive Sampling**: Goal-biased and informed sampling strategies
- **Dynamic Replanning**: Real-time replanning for moving obstacles
- **Multi-Resolution**: Hierarchical planning for large environments
- **Smoothing**: Post-processing for smoother trajectories