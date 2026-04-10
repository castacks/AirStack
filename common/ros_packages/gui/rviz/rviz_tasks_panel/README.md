# RViz Tasks Panel

RViz2 panel plugin for dispatching and monitoring ROS 2 task action goals. Provides a tabbed GUI where operators can parameterize, execute, and cancel tasks on any discovered robot, with live feedback and result display.

## Overview

The Tasks Panel replaces the need for CLI-based action goal dispatch by providing a graphical interface for all 9 AirStack task types. Each task type gets its own tab with auto-generated parameter widgets, an executor selector, and a feedback/result view.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Tasks Panel                                       Robot: [robot_1]  │
├──────────────────────────────────────────────────────────────────────┤
│ [Takeoff] [Land] [Navigate] [Exploration] [Coverage] [ObjectSearch]  │
├──────────────────────────────────────────────────────────────────────┤
│  ┌─ Goal Parameters ────────┐  ┌─ Feedback & Result ─────────────┐   │
│  │ Executor: [/robot_1/...] │  │  Feedback:                      │   │
│  │ target_altitude_m: [5.0] │  │  [live feedback stream]         │   │
│  │ velocity_m_s:      [1.0] │  │                                 │   │
│  │                          │  │  Result:                        │   │
│  │ [Cancel]      [Execute]  │  │  [goal outcome]                 │   │
│  └──────────────────────────┘  └─────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────┘
```

## Features

- **9 task tabs** with auto-generated goal parameter widgets
- **Executor discovery** -- scans ROS 2 topics every 5 seconds to find running action servers
- **Robot namespace selector** -- auto-populated from discovered action server namespaces
- **Polygon input** -- integrates with `rviz_polygon_selection_tool` to capture 3D polygon selections from the RViz viewport
- **Waypoint input** -- subscribes to the `waypoints` topic to receive paths from the 3D Waypoint tool
- **Fixed Trajectory editor** -- type dropdown with auto-populated default attributes in an editable key-value table
- **Live feedback** -- timestamped feedback messages stream in real time
- **Result display** -- color-coded status (green for succeeded, red for aborted/canceled)
- **Config persistence** -- robot and executor selections are saved/restored with the RViz config
- **Active Tab Status** -- tab text color reflect active/running task status, GUI only allows one action to execute at a time per robot to prevent conflicts

## Supported Task Types

| Tab | Action Type | Key Parameters |
|-----|-------------|----------------|
| Takeoff | `task_msgs/action/TakeoffTask` | `target_altitude_m`, `velocity_m_s` |
| Land | `task_msgs/action/LandTask` | `velocity_m_s` |
| Navigate | `task_msgs/action/NavigateTask` | `global_plan` (Path), `goal_tolerance_m` |
| Exploration | `task_msgs/action/ExplorationTask` | `search_bounds` (Polygon), altitude/speed limits, `time_limit_sec` |
| Coverage | `task_msgs/action/CoverageTask` | `coverage_area` (Polygon), altitude/speed limits, `line_spacing_m`, `heading_deg` |
| Object Search | `task_msgs/action/ObjectSearchTask` | `object_class`, `search_area` (Polygon), `target_count`, `time_limit_sec` |
| Object Counting | `task_msgs/action/ObjectCountingTask` | `object_class`, `count_area` (Polygon), altitude/speed limits |
| Semantic Search | `task_msgs/action/SemanticSearchTask` | `query`, `search_area` (Polygon), `confidence_threshold`, `time_limit_sec` |
| Fixed Trajectory | `task_msgs/action/FixedTrajectoryTask` | `trajectory_spec` (FixedTrajectory), `loop` |

## Widget Type Mapping

Goal fields are mapped to Qt widgets based on their ROS type:

| ROS Type | Widget | Notes |
|----------|--------|-------|
| `float32` / `float64` | `QDoubleSpinBox` | Range and default from task registry |
| `int32` | `QSpinBox` | Range and default from task registry |
| `string` | `QLineEdit` | Free-text input |
| `bool` | `QCheckBox` | Toggle |
| `geometry_msgs/Polygon` | `QPushButton` | Calls `get_selection` service on `rviz_polygon_selection_tool` |
| `nav_msgs/Path` | Status label | Displays latest path from `waypoints` subscription |
| `airstack_msgs/FixedTrajectory` | `QComboBox` + `QTableWidget` | Type selector with editable attribute table |

## Dependencies

- `rviz_common` -- RViz2 panel base class
- `pluginlib` -- plugin loading
- `rclcpp` / `rclcpp_action` -- ROS 2 node and action client
- `task_msgs` -- action definitions for all 9 task types
- `airstack_msgs` -- `FixedTrajectory` message
- `geometry_msgs` / `nav_msgs` -- standard ROS 2 message types
- `diagnostic_msgs` / `action_msgs` -- status and action introspection
- `rviz_polygon_selection_tool` -- polygon selection service
- Qt5 (Core, Widgets, Gui)

## Build

```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select rviz_tasks_panel"
```

## Usage

1. Launch RViz2
2. Go to **Panels > Add New Panel**
3. Select **rviz_tasks_panel / TasksPanel**
4. The panel auto-discovers running task action servers and populates the Robot dropdown
5. Select a tab, configure goal parameters, and click **Execute**
6. Monitor feedback in the right pane; click **Cancel** to abort a running goal

### Polygon Selection

For tasks requiring a polygon boundary (Exploration, Coverage, Object Search, etc.):

1. Activate the **Polygon Selection Tool** in the RViz toolbar
2. Draw a polygon in the 3D viewport
3. In the Tasks Panel, click the **Get Polygon from RViz** button for the polygon field
4. The panel calls the `get_selection` service and displays the captured point count

### Waypoint Navigation

For the Navigate task:

1. Use the **3D Waypoint Tool** in RViz to place waypoints
2. The Tasks Panel subscribes to the `waypoints` topic and caches the latest path
3. Click **Get Waypoints from RViz** to capture the current path

### Fixed Trajectory

1. Select a trajectory type from the dropdown (e.g., `circle`, `lemniscate`, `fixed_trajectory`)
2. Default attributes are pre-populated in the key-value table
3. Edit attribute values as needed, then click **Execute**

## Executor Discovery

The panel scans `node->get_topic_names_and_types()` for topics matching the pattern `*/<action_topic_suffix>/_action/status`. For each match, it extracts the robot namespace prefix (e.g., `/robot_1`) and populates:

- The top-level **Robot** dropdown with discovered namespaces
- Each tab's **Executor** dropdown with the full action server topic

Discovery runs automatically every 5 seconds and can be triggered manually with the **Refresh** button.

## Architecture

- **Compile-time task registry**: `getTaskDefs()` returns a static vector of `TaskTypeDef` structs defining all 9 task types, their action topic suffixes, and goal field definitions.
- **Type-erased action clients**: Since `rclcpp_action::Client` is templated, the panel uses `std::any` to store type-erased clients and goal handles per tab, with a 9-way switch in `onExecuteClicked()` / `onCancelClicked()` for dispatch.
- **Thread safety**: ROS 2 action callbacks arrive on the ROS executor thread. All Qt widget updates from callbacks use `QMetaObject::invokeMethod(this, lambda, Qt::QueuedConnection)` to marshal back to the Qt main thread.

## License

MIT -- Carnegie Mellon University
