# RViz Behavior Tree Panel

A custom RViz panel for visualizing behavior trees in real-time. This panel subscribes to behavior tree data published in Graphviz xdot format and renders interactive behavior tree visualizations directly within RViz.

## Features

- **Real-time visualization**: Automatically updates when new behavior tree data is published
- **Interactive display**: Zoom, pan, and navigate through behavior tree graphs
- **Seamless RViz integration**: No need for external windows or applications
- **Compatible with existing systems**: Uses the same `behavior_tree_graphviz` topic as the RQT version

## Installation

This package is part of the AirStack robotics framework. Build it using colcon:

```bash
cd /path/to/your/ros_workspace
colcon build --packages-select rviz_behavior_tree_panel
source install/setup.bash
```

## Usage

### Adding the Panel to RViz

1. Launch RViz2:
   ```bash
   rviz2
   ```

2. Add the Behavior Tree Panel:
   - Go to **Panels** → **Add New Panel**
   - Select **BehaviorTreePanel** from the list
   - Click **OK**

3. The panel will automatically subscribe to the `behavior_tree_graphviz` topic and display behavior trees when data is available.

### Topic Interface

The panel subscribes to:
- **Topic**: `behavior_tree_graphviz`
- **Message Type**: `std_msgs/String`
- **Content**: Graphviz xdot format describing the behavior tree structure

### Publishing Behavior Tree Data

Your behavior tree system should publish String messages containing Graphviz xdot data to the `behavior_tree_graphviz` topic. Example:

```cpp
// C++ example
auto publisher = node->create_publisher<behavior_tree_msgs::msg::GraphVizXdot>("behavior_tree_graphviz", 10);
behavior_tree_msgs::msg::GraphVizXdot msg;
msg.header.stamp = node->now();
msg.xdot.data = "digraph G { root [label=\"Root\"]; action [label=\"Action\"]; root -> action; }";
publisher->publish(msg);
```

```python
# Python example
import rclpy
from behavior_tree_msgs.msg import GraphVizXdot

publisher = node.create_publisher(GraphVizXdot, 'behavior_tree_graphviz', 10)
msg = GraphVizXdot()
msg.header.stamp = node.get_clock().now().to_msg()
msg.xdot.data = 'digraph G { root [label="Root"]; action [label="Action"]; root -> action; }'
publisher.publish(msg)
```

## Panel Status Indicators

The panel displays status information at the bottom:
- **Green**: "Waiting for behavior tree data..." (no data received yet)
- **Blue**: "Behavior tree updated" (successfully displaying current data)
- **Red**: "Error parsing behavior tree: [error message]" (invalid data received)

## Testing

Test publishers are included in the `scripts/` directory:

### Simple Test Publisher
```bash
python3 scripts/simple_test_publisher.py
```
Publishes a basic 3-node behavior tree with cycling states every 5 seconds.

### Complex Test Publisher  
```bash
python3 scripts/test_behavior_tree_publisher.py
```
Publishes more complex behavior tree scenarios with multiple node types and states.

## Architecture

### Core Components

- **BehaviorTreePanel**: Main panel class inheriting from `rviz_common::Panel`
- **xdot_cpp**: Integrated C++ library for rendering Graphviz xdot data in Qt
- **Topic Subscription**: ROS 2 subscription to `behavior_tree_graphviz` topic

### Dependencies

- **ROS 2**: Core ROS 2 functionality
- **RViz2**: Panel framework and integration
- **Qt5**: GUI framework (Core, Widgets, Gui)
- **xdot_cpp**: Custom Graphviz rendering library
- **std_msgs**: String message type

### File Structure

```
rviz_behavior_tree_panel/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata
├── rviz_common_plugins.xml          # Plugin registration
├── README.md                         # This documentation
├── include/rviz_behavior_tree_panel/
│   └── behavior_tree_panel.hpp      # Panel header
├── src/
│   └── behavior_tree_panel.cpp      # Panel implementation
├── icons/classes/
│   └── BehaviorTreePanel.png        # Panel icon
├── scripts/
│   ├── simple_test_publisher.py     # Basic test publisher
│   └── test_behavior_tree_publisher.py  # Complex test publisher
└── xdot_cpp/                        # Graphviz rendering library
    ├── CMakeLists.txt
    ├── include/
    └── src/
```

## Compatibility

This panel is designed to be compatible with existing behavior tree systems that use the RQT behavior tree visualization. Simply ensure your system publishes to the `behavior_tree_graphviz` topic with the same Graphviz xdot format.

## Troubleshooting

### Panel Not Appearing
- Ensure the package is built and sourced: `source install/setup.bash`
- Check that RViz2 can find the plugin: look for BehaviorTreePanel in the Add New Panel dialog

### No Visualization
- Verify data is being published: `ros2 topic echo behavior_tree_graphviz`
- Check panel status message for error details
- Ensure published data is valid Graphviz xdot format

### Performance Issues
- Large behavior trees may impact rendering performance
- Consider reducing update frequency if needed
- The panel automatically filters duplicate data to avoid unnecessary updates

## License

This package is part of the AirStack robotics framework. See LICENSE file for details.
