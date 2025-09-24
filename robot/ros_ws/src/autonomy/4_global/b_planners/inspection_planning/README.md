# Inspection Planning Package

The Inspection Planning package provides semantic-aware inspection planning capabilities for autonomous robots. It subscribes to VDB map data and semantic information (both observed and predicted) to generate optimal inspection paths.

## Overview

This package serves as a global planner that:
- Subscribes to VDB map visualization data
- Receives observed semantic points (ground truth)
- Receives predicted semantic points (from ML models)
- Generates inspection paths to visit semantic points of interest
- Provides visualization of the inspection plan

## Functionality

Upon activation, the Inspection Planner will:

1. **Map Integration**: Process VDB map data for environment understanding
2. **Semantic Processing**: Collect and process observed and predicted semantic points
3. **Path Generation**: Create inspection paths to visit semantic points
4. **Visualization**: Publish visual markers for debugging and monitoring
5. **Service Control**: Toggle inspection planning on/off via service calls

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `vdb_map_visualization` | `visualization_msgs/MarkerArray` | VDB map visualization data |
| `observed_semantics` | `geometry_msgs/PointStamped` | Observed semantic points |
| `predicted_semantics` | `geometry_msgs/PointStamped` | Predicted semantic points |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `~/global_plan` | `nav_msgs/Path` | Generated inspection path |
| `~/grid_viz` | `visualization_msgs/MarkerArray` | VDB map visualization |
| `~/observed_semantics_viz` | `visualization_msgs/MarkerArray` | Observed semantics visualization |
| `~/predicted_semantics_viz` | `visualization_msgs/MarkerArray` | Predicted semantics visualization |
| `~/inspection_plan_viz` | `visualization_msgs/MarkerArray` | Inspection plan visualization |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `~/inspection_toggle` | `std_srvs/Trigger` | Toggle inspection planning on/off |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_frame_id` | string | "base_link" | Robot base frame |
| `map_frame_id` | string | "map" | Map frame |
| `pub_global_plan_topic` | string | "~/global_plan" | Global plan topic |
| `pub_grid_viz_topic` | string | "~/grid_viz" | Grid visualization topic |
| `pub_observed_semantics_viz_topic` | string | "~/observed_semantics_viz" | Observed semantics viz topic |
| `pub_predicted_semantics_viz_topic` | string | "~/predicted_semantics_viz" | Predicted semantics viz topic |
| `pub_inspection_plan_viz_topic` | string | "~/inspection_plan_viz" | Inspection plan viz topic |
| `sub_vdb_map_topic` | string | "vdb_map_visualization" | VDB map topic |
| `sub_observed_semantics_topic` | string | "observed_semantics" | Observed semantics topic |
| `sub_predicted_semantics_topic` | string | "predicted_semantics" | Predicted semantics topic |
| `srv_inspection_toggle_topic` | string | "~/inspection_toggle" | Toggle service topic |

## Usage

### Launch the package:
```bash
ros2 launch inspection_planning inspection_launch.xml
```

### Toggle inspection planning:
```bash
ros2 service call /inspection_planner/inspection_toggle std_srvs/srv/Trigger
```

### Publish semantic points:
```bash
# Observed semantics
ros2 topic pub /observed_semantics geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 1.0, y: 2.0, z: 0.5}}"

# Predicted semantics
ros2 topic pub /predicted_semantics geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 3.0, y: 4.0, z: 1.0}}"
```

## Visualization

The package publishes several visualization markers:
- **Green spheres**: Observed semantic points
- **Red spheres**: Predicted semantic points
- **Blue line strip**: Inspection path
- **VDB map**: Original map data

## Dependencies

- ROS2 (Humble/Iron)
- rclcpp
- geometry_msgs
- nav_msgs
- visualization_msgs
- sensor_msgs
- tf2
- vdb_mapping
- PCL
- Eigen3

## Future Enhancements

- Advanced path planning algorithms (RRT, A*, etc.)
- Collision avoidance integration
- Multi-robot coordination
- Dynamic replanning
- Semantic point prioritization
- Time-optimal path generation
