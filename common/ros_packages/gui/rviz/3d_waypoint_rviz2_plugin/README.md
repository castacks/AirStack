# 3D Waypoint RViz2 Plugin

An interactive RViz2 tool plugin for creating, manipulating, and managing 3D waypoints directly in the RViz2 viewport. Waypoint state is managed by a shared `WaypointManager` singleton so that both this tool (3D interaction) and the Tasks Panel's Navigate tab (UI controls) operate on the same data.

<img width="899" height="664" alt="image" src="https://github.com/user-attachments/assets/655d8e5d-ee15-4daa-aff8-b45f28d3403a" />

## Architecture

```
 WaypointTool (rviz_common::Tool)        Tasks Panel Navigate tab
   mouse click in 3D viewport              UI: height, X/Y/Z/Yaw,
         |                                 Clear, Save, Load, Execute
         v                                        |
     +------------------------------------------+
     |          WaypointManager (singleton)      |
     |                                          |
     |  - InteractiveMarkerServer               |
     |  - Ogre SceneNode map (waypoint visuals) |
     |  - add / remove / clear / reorder        |
     |  - save / load (.db3 bag files)          |
     |  - getPath() -> nav_msgs/Path            |
     |  - Qt signals for UI updates             |
     +------------------------------------------+
```

- **WaypointTool** (`rviz_common::Tool`) -- handles mouse events in the 3D viewport. Left-click to add, right-click near an existing waypoint to delete. Delegates all state to `WaypointManager`.
- **WaypointManager** (`QObject` singleton) -- owns all waypoint state: the interactive marker server, the Ogre scene nodes, persistence, and publishing. Emits Qt signals (`waypointCountChanged`, `selectedMarkerChanged`, `waypointsCleared`) so external UI can stay in sync.
- The **Tasks Panel** (separate `rviz_tasks_panel` package) holds a `shared_ptr` to the same `WaypointManager` instance and provides the full control UI in its Navigate tab.

## Features

- **Interactive Waypoint Creation**: Click to add waypoints at any position in the 3D environment
- **3D Manipulation**: Drag and move waypoints using interactive markers (MOVE_ROTATE, MOVE_AXIS)
- **Orientation Control**: Set and adjust waypoint yaw via interactive markers or Navigate tab spinboxes
- **Visual Feedback**: Real-time axis indicator meshes at each waypoint
- **Waypoint Management**:
  - Add waypoints with left-click
  - Delete waypoints with right-click or context menu
  - "Set manual" context menu selects the waypoint for editing in the Navigate tab
  - Automatic reordering after deletion (no gaps in numbering)
  - Clear all waypoints at once
- **Data Persistence**: Save and load waypoint configurations using ROS 2 bag files (.db3)
- **Publishing**: Publish waypoints as `nav_msgs/msg/Path` messages
- **Configurable**: Default height, frame ID, and topic name (persisted in RViz config)

## Requirements

- ROS 2 Jazzy (or Humble and above)
- RViz2
- Qt5
- C++17

## Dependencies

- rclcpp
- rviz_common
- rviz_default_plugins
- rviz_rendering
- rviz_ogre_vendor
- geometry_msgs
- visualization_msgs
- interactive_markers
- nav_msgs
- tf2
- tf2_geometry_msgs
- rosbag2_cpp
- pluginlib

## Building

```bash
colcon build --packages-select waypoint_rviz2_plugin
```

## Usage

### Adding the Tool to RViz2

1. Open RViz2
2. In the toolbar, click the "+" icon to add a new tool
3. Select **waypoint_rviz2_plugin / WaypointTool** from the list
4. The tool appears in the toolbar with keyboard shortcut **1**

### Placing Waypoints

1. **Select the Tool**: Click the Waypoint Tool icon or press **1**
2. **Add Waypoints**: Left-click anywhere in the 3D view
3. **Move Waypoints**: Add the interactive marker display (Add > By topic > `/waypoint_plugin/update/InteractiveMarkers`), then drag markers to reposition
4. **Delete Waypoints**: Right-click near a waypoint, or use the context menu "delete"
5. **Select for Editing**: Right-click a marker and choose "set manual" to select it in the Navigate tab

### Navigate Tab Controls (in Tasks Panel)

All waypoint management UI lives in the Tasks Panel's **Navigate** tab:

- **Default Height** -- Z-height for newly placed waypoints
- **X / Y / Z / Yaw spinboxes** -- edit the currently selected waypoint's pose
- **Clear All** -- remove all waypoints
- **Save / Load** -- persist waypoints to/from `.db3` bag files
- **Waypoint count** and **selected waypoint** labels update in real time

### Listening to Published Waypoints

```bash
ros2 topic echo /waypoints
```

## File Structure

```
waypoint_rviz2_plugin/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml
├── README.md
├── include/
│   └── waypoint_rviz2_plugin/
│       ├── waypoint_manager.hpp   # Shared singleton (state + operations)
│       └── waypoint_tool.hpp      # RViz Tool (mouse events only)
├── src/
│   ├── waypoint_manager.cpp
│   └── waypoint_tool.cpp
├── media/
│   └── axis.dae                   # 3D axis indicator mesh
└── launch/
    └── rviz2.launch.py
```

## License

Apache 2.0

## Credits

Original ROS 1 version by KoKoLates (the21515@gmail.com).
ROS 2 port and WaypointManager refactor for AirStack.

## References

- [RViz2 Plugin Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Advanced/RViz-Plugins.html)
- [Interactive Markers](https://docs.ros.org/en/jazzy/Tutorials/Advanced/RViz/Interactive-Markers.html)
