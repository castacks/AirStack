# 3D Waypoint RViz2 Plugin

An interactive RViz2 plugin for ROS2 Humble (and above) that allows users to create, manipulate, and manage 3D waypoints directly in the RViz2 visualization environment.

<img width="899" height="664" alt="image" src="https://github.com/user-attachments/assets/655d8e5d-ee15-4daa-aff8-b45f28d3403a" />



## Features

- **Interactive Waypoint Creation**: Click to add waypoints at any position in the 3D environment
- **3D Manipulation**: Drag and move waypoints using interactive markers
- **Orientation Control**: Set and adjust waypoint orientation (yaw angle)
- **Visual Feedback**: Real-time visual representation of waypoints with axis indicators
- **Waypoint Management**:
  - Add waypoints with left-click
  - Delete waypoints with right-click or context menu
  - Manually adjust position and orientation via UI controls
  - Clear all waypoints at once
- **Data Persistence**: Save and load waypoint configurations using ROS2 bag files (.db3)
- **Publishing**: Publish waypoints as `nav_msgs/msg/Path` messages
- **Configurable**: Set default height, frame ID, and topic name

## Requirements

- ROS2 Humble or above
- RViz2
- Qt5
- C++14 or higher

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

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> waypoint_rviz2_plugin
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select waypoint_rviz2_plugin
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launching RViz2 with the Plugin

```bash
ros2 launch waypoint_rviz2_plugin rviz2.launch.py
```

Or start RViz2 manually:
```bash
rviz2
```

### Adding the Plugin to RViz2

1. Open RViz2
2. Click on the "Panels" menu and select "Add New Panel"
3. In the toolbar, click the "+" icon to add a new tool
4. Select "waypoint_rviz2_plugin/WaypointTool" from the list
5. The waypoint tool should now appear in the toolbar

### Using the Plugin

1. **Select the Tool**: Click on the Waypoint Tool icon in the RViz2 toolbar (or press '1' as shortcut)

2. **Add Waypoints**: 
   - Left-click anywhere in the 3D view to place a waypoint
   - An axis indicator will appear showing the waypoint position

3. **Move Waypoints**:
   - In the Displays panel, make sure to click Add > By topic > /waypoint_plugin/update/InteractiveMarkers
   - Click and drag the interactive markers to reposition waypoints
   - Use the rotation controls on the markers to adjust orientation

5. **Delete Waypoints**:
   - Right-click on a waypoint to delete it
   - Or right-click on the interactive marker and select "delete" from the menu

6. **Manual Adjustment**:
   - Use the panel controls to manually set X, Y, Z position and Yaw orientation
   - Select "set manual" from the marker context menu to apply manual changes

7. **Configuration**:
   - **Topic**: Set the topic name for publishing waypoints (default: `/waypoints`)
   - **Frame**: Set the reference frame for waypoints (default: `map`)
   - **Default Height**: Set the Z-height for new waypoints

8. **Save/Load**:
   - Click "Save Waypoints" to save current waypoints to a .db3 bag file
   - Click "Load Waypoints" to load waypoints from a .db3 bag file

9. **Publish**:
   - Click "Publish Waypoints" to publish all waypoints as a `nav_msgs/msg/Path` message
   - To visualize, in the Displays panel click Add > /waypoints > Path

10. **Clear All**:
   - Click "Clear All" to remove all waypoints from the scene

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
│       ├── waypoint_tool.hpp
│       └── waypoint_widget.hpp
├── src/
│   ├── waypoint_tool.cpp
│   └── waypoint_widget.cpp
├── ui/
│   └── waypoint_plugin.ui
├── media/
│   └── axis.dae
└── launch/
    └── rviz2.launch.py
```

## Migration from ROS1

This plugin is a port of the ROS1 Noetic waypoint plugin. Key changes include:

- Updated to use `rclcpp` instead of `roscpp`
- Updated to use `rviz_common` instead of `rviz`
- Updated interactive markers API for ROS2
- Changed from `rosbag` to `rosbag2_cpp` API
- Updated TF library from `tf` to `tf2`
- Changed plugin export macros for ROS2
- Updated build system from `catkin` to `ament_cmake`

## Known Issues

- Saved bag files use the `.db3` format (SQLite3) instead of the ROS1 `.bag` format
- The 6D checkbox in the UI is currently not implemented

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## License

Apache 2.0

## Credits

Original ROS1 version by KoKoLates (the21515@gmail.com)
ROS2 port for Humble and above

## References

- [RViz2 Plugin Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/RViz-Plugins.html)
- [Interactive Markers](https://docs.ros.org/en/humble/Tutorials/Advanced/RViz/Interactive-Markers.html)
