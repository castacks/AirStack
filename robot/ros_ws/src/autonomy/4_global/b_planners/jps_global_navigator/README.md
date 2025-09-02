# JPS Global Navigator

A ROS 2 global path planner that uses Jump Point Search (JPS) algorithm with path smoothing for efficient pathfinding in 3D environments.

## Overview

This package implements a global path planner based on the Jump Point Search algorithm, which is an optimization of A* that reduces the number of nodes explored by "jumping" over intermediate nodes in straight lines and diagonals. The planner includes path smoothing to create more natural, flyable paths.

## Features

- **Jump Point Search Algorithm**: Efficient pathfinding with reduced node exploration
- **3D Planning**: Full 3D pathfinding capability with 26-directional movement
- **Path Smoothing**: Post-processing to create smooth, natural paths
- **ROS 2 Action Server**: Compatible with existing navigation interfaces
- **Visualization**: Debug visualization of search process and final paths
- **Configurable Parameters**: Tunable algorithm parameters for different environments

## Algorithm Details

### Jump Point Search (JPS)
JPS is an optimization of A* that:
1. Identifies "jump points" - nodes that must be explored due to forced neighbors
2. Skips intermediate nodes along straight lines and diagonals
3. Significantly reduces the search space while maintaining optimality

### Path Smoothing
The path smoothing algorithm:
1. Attempts to create shortcuts by connecting non-adjacent waypoints
2. Verifies collision-free paths using line-of-sight checks
3. Iteratively refines the path over multiple passes
4. Maintains safety while improving path quality

## Usage

### Launch the Planner
```bash
ros2 launch jps_global_navigator jps_global_navigator.launch.py
```

### With Custom Parameters
```bash
ros2 launch jps_global_navigator jps_global_navigator.launch.py \
    config_file:=/path/to/your/params.yaml \
    enable_debug_visualization:=true
```

### Action Interface
The planner provides a ROS 2 action server at `/jps_navigator` with the interface:
- **Goal**: List of target poses to navigate through
- **Feedback**: Current position, distance remaining, planning phase
- **Result**: Success status, final pose, distance traveled

### Send Navigation Goals

```bash
# Basic navigation goal
ros2 action send_goal /jps_navigator task_msgs/action/NavigationTask \
    "{goal_poses: [{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 5.0}}}]}"

# Navigation with time constraint (30 seconds max)
ros2 action send_goal /jps_navigator task_msgs/action/NavigationTask \
    "{goal_poses: [{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 5.0}}}], max_planning_seconds: 30.0}"

# Multi-goal navigation with time limit
ros2 action send_goal /jps_navigator task_msgs/action/NavigationTask \
    "{goal_poses: [
        {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 5.0}}},
        {header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 6.0, z: 5.5}}}
    ], max_planning_seconds: 60.0}"
```

## Parameters

### Core Algorithm Parameters
- `jps_max_iterations`: Maximum search iterations (default: 10000)
- `jps_goal_tolerance`: Goal reach tolerance in meters (default: 2.0)
- `cost_threshold`: Obstacle cost threshold (default: 50.0)
- `resolution`: Grid resolution in meters (default: 0.5)

### Path Smoothing Parameters
- `smoothing_iterations`: Number of smoothing passes (default: 5)
- `smoothing_step_size`: Smoothing step size (default: 0.1)

### Topics
- `cost_map_topic`: Input cost map (default: "/cost_map")
- `odom_topic`: Robot odometry (default: "/odom")

### Visualization
- `enable_debug_visualization`: Enable search visualization (default: true)

## Topics

### Subscribed Topics
- `/cost_map` (sensor_msgs/PointCloud2): 3D cost map with optional intensity field
- `/odom` (nav_msgs/Odometry): Robot odometry for current position

### Published Topics
- `/global_plan` (nav_msgs/Path): Planned path as sequence of poses
- `/jps_search_markers` (visualization_msgs/MarkerArray): Debug visualization

### Action Servers
- `/jps_navigator` (task_msgs/NavigationTask): Main planning action interface

## Comparison with RRT*

| Aspect | JPS | RRT* |
|--------|-----|------|
| **Optimality** | Optimal on grid | Asymptotically optimal |
| **Speed** | Fast (reduced search space) | Variable (depends on sampling) |
| **Memory** | Grid-based (predictable) | Tree-based (grows with time) |
| **Path Quality** | Grid-constrained, then smoothed | Naturally smooth |
| **Determinism** | Deterministic | Probabilistic |
| **Environment** | Works well in structured spaces | Better for complex obstacles |

## Performance Tuning

### For Faster Planning
- Decrease `resolution` (coarser grid)
- Reduce `jps_max_iterations`
- Disable `enable_debug_visualization`

### For Better Path Quality
- Increase `smoothing_iterations`
- Decrease `resolution` (finer grid)
- Adjust `cost_threshold` for your environment

### For Complex Environments
- Increase `jps_max_iterations`
- Fine-tune `cost_threshold`
- Consider hybrid approach with RRT* for very complex spaces

## Building

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select jps_global_navigator
source install/setup.bash
```

## Testing

### Unit and Integration Tests

Unit and integration tests are provided to verify functionality:

```bash
# Build and run tests
colcon build --packages-select jps_global_navigator
colcon test --packages-select jps_global_navigator

# View test results
colcon test-result --verbose

# Run tests directly
cd /path/to/ros_ws
source install/setup.bash
./build/jps_global_navigator/test_jps_global_navigator_unit
./build/jps_global_navigator/test_jps_global_navigator_integration
```

### Manual Testing

Use the provided test script to verify planning functionality:

```bash
# Terminal 1: Start the navigator
ros2 launch jps_global_navigator jps_global_navigator.launch.py

# Terminal 2: Run test script
python3 src/autonomy/4_global/b_planners/jps_global_navigator/scripts/test_jps_planner.py
```

The test script creates a simple 3D environment with obstacles and tests the planner's ability to find paths around them.

## Dependencies

- ROS 2 (Humble or later)
- Standard ROS 2 navigation messages
- task_msgs (for action interface)
- Visualization messages for debug output

## License

Apache 2.0 License