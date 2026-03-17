---
name: add-behavior-tree-node
description: Create behavior tree nodes for high-level mission logic and decision-making. Use when implementing actions, conditions, or decorators for behavior trees. Covers BT node types, registration, and integration with behavior executive.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Add a Behavior Tree Node

## When to Use

When implementing high-level mission logic, decision-making, or orchestrating autonomy behaviors through behavior trees.

## Prerequisites

- Understanding of behavior tree concepts (nodes, ticks, status)
- Familiarity with the AirStack behavior tree framework
- Knowledge of the behavior you want to implement
- Understanding of required interactions with autonomy stack

## Behavior Tree Overview

AirStack uses behavior trees for high-level mission execution:

- **Behavior Tree Framework:** `robot/ros_ws/src/behavior/behavior_tree`
- **Behavior Executive:** `robot/ros_ws/src/behavior/behavior_executive`
- **Example Nodes:** `robot/ros_ws/src/behavior/behavior_tree_example`

### Node Types

1. **Action Nodes:** Perform actions (e.g., "Navigate to Waypoint", "Take Photo")
2. **Condition Nodes:** Check conditions (e.g., "Battery OK?", "At Goal?")
3. **Decorator Nodes:** Modify child behavior (e.g., "Retry", "Timeout")
4. **Control Nodes:** Control flow (Sequence, Fallback, Parallel) - usually pre-defined

## Steps

### 1. Understand Behavior Tree Framework

Study the existing framework:

```bash
# View behavior tree package
ls -la robot/ros_ws/src/behavior/behavior_tree/

# Study example nodes
cat robot/ros_ws/src/behavior/behavior_tree_example/src/example_action.cpp
cat robot/ros_ws/src/behavior/behavior_tree_example/src/example_condition.cpp
```

Key components:
- **BT Node base classes:** Action, Condition, Decorator
- **Node registration:** Factory registration macros
- **XML description:** Nodes described in XML for behavior executive

### 2. Decide Node Type

Choose the appropriate node type:

| Node Type | When to Use | Returns |
|-----------|-------------|---------|
| **Action** | Performs an operation | SUCCESS, FAILURE, RUNNING |
| **Condition** | Checks a condition | SUCCESS, FAILURE |
| **Decorator** | Wraps another node | Depends on implementation |

**Examples:**
- Action: "TakeoffAction", "NavigateToWaypointAction", "CaptureImageAction"
- Condition: "BatteryOKCondition", "AtGoalCondition", "ObstacleDetectedCondition"
- Decorator: "RetryDecorator", "TimeoutDecorator"

### 3. Create Node Package (if needed)

If creating multiple related nodes, create a package:

```bash
# Create package for your behavior nodes
cd robot/ros_ws/src/behavior/

# C++ package
ros2 pkg create your_behavior_nodes \
    --build-type ament_cmake \
    --dependencies rclcpp behavior_tree airstack_msgs

# Python package
ros2 pkg create your_behavior_nodes \
    --build-type ament_python \
    --dependencies rclpy behavior_tree airstack_msgs
```

Or add to existing package (e.g., `behavior_tree_example` for testing).

### 4. Implement Action Node (C++)

**File:** `src/your_action_node.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include "behavior_tree/action_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace your_namespace
{

/**
 * @brief Action node that performs your specific action
 * 
 * This node demonstrates a typical action that:
 * - Has input/output ports for data
 * - Subscribes to ROS topics
 * - Publishes commands
 * - Returns RUNNING while executing, then SUCCESS/FAILURE
 */
class YourActionNode : public BT::StatefulActionNode
{
public:
  YourActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    // Get ROS node from blackboard
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Create ROS interfaces
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10,
      std::bind(&YourActionNode::odomCallback, this, std::placeholders::_1));
    
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "goal", 10);
    
    RCLCPP_INFO(node_->get_logger(), "YourActionNode initialized");
  }
  
  /**
   * @brief Define input/output ports
   * 
   * Ports allow data exchange between nodes in the tree
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Point>("target_position",
                                                "Target position to reach"),
      BT::InputPort<double>("tolerance", 0.5,
                            "Distance tolerance (m)"),
      BT::OutputPort<double>("distance_to_goal",
                            "Remaining distance to goal")
    };
  }
  
  /**
   * @brief Called when node is first ticked
   * @return NodeStatus (RUNNING, SUCCESS, FAILURE)
   */
  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(), "Starting YourAction");
    
    // Get input from ports
    auto target = getInput<geometry_msgs::msg::Point>("target_position");
    if (!target) {
      RCLCPP_ERROR(node_->get_logger(), "Missing target_position input");
      return BT::NodeStatus::FAILURE;
    }
    target_ = target.value();
    
    tolerance_ = getInput<double>("tolerance").value();
    
    // Publish goal command
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position = target_;
    goal_msg.pose.orientation.w = 1.0;
    goal_pub_->publish(goal_msg);
    
    action_started_ = true;
    return BT::NodeStatus::RUNNING;
  }
  
  /**
   * @brief Called on each tick while RUNNING
   * @return NodeStatus
   */
  BT::NodeStatus onRunning() override
  {
    if (!current_pose_) {
      // Waiting for odometry
      return BT::NodeStatus::RUNNING;
    }
    
    // Compute distance to goal
    double dx = target_.x - current_pose_->position.x;
    double dy = target_.y - current_pose_->position.y;
    double dz = target_.z - current_pose_->position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Update output port
    setOutput("distance_to_goal", distance);
    
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "Distance to goal: %.2f m", distance);
    
    // Check if goal reached
    if (distance < tolerance_) {
      RCLCPP_INFO(node_->get_logger(), "Goal reached!");
      return BT::NodeStatus::SUCCESS;
    }
    
    // Check timeout (optional)
    auto elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > max_duration_) {
      RCLCPP_WARN(node_->get_logger(), "Action timeout");
      return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
  }
  
  /**
   * @brief Called when node is halted (interrupted)
   */
  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "YourAction halted");
    // Clean up resources if needed
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
    if (!action_started_) {
      start_time_ = node_->now();
    }
  }
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  
  geometry_msgs::msg::Point target_;
  std::shared_ptr<geometry_msgs::msg::Pose> current_pose_;
  double tolerance_ = 0.5;
  double max_duration_ = 60.0;  // seconds
  rclcpp::Time start_time_;
  bool action_started_ = false;
};

}  // namespace your_namespace

// Register node with factory
#include "behavior_tree/bt_factory.hpp"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<your_namespace::YourActionNode>("YourAction");
}
```

### 5. Implement Condition Node (C++)

**File:** `src/your_condition_node.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include "behavior_tree/condition_node.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace your_namespace
{

/**
 * @brief Condition node that checks a specific condition
 * 
 * Condition nodes typically:
 * - Check state from topics or ports
 * - Return SUCCESS if condition true, FAILURE if false
 * - Are stateless (re-evaluated on each tick)
 */
class YourConditionNode : public BT::ConditionNode
{
public:
  YourConditionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 10,
      std::bind(&YourConditionNode::batteryCallback, this, std::placeholders::_1));
  }
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("threshold", 20.0,
                           "Battery percentage threshold")
    };
  }
  
  /**
   * @brief Evaluate condition
   * @return NodeStatus (SUCCESS if condition true, FAILURE otherwise)
   */
  BT::NodeStatus tick() override
  {
    if (!battery_percentage_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "No battery data received");
      return BT::NodeStatus::FAILURE;
    }
    
    double threshold = getInput<double>("threshold").value();
    
    if (*battery_percentage_ >= threshold) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "Battery low: %.1f%% (threshold: %.1f%%)",
                          *battery_percentage_, threshold);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    battery_percentage_ = std::make_shared<double>(msg->percentage * 100.0);
  }
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::shared_ptr<double> battery_percentage_;
};

}  // namespace your_namespace

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<your_namespace::YourConditionNode>("YourCondition");
}
```

### 6. Implement Python Node (Alternative)

**File:** `your_behavior_nodes/your_action_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from behavior_tree_py import ActionNode, NodeStatus, PortsList, Port
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math

class YourActionNode(ActionNode):
    """
    Python action node implementation
    """
    
    @staticmethod
    def provided_ports():
        return PortsList([
            Port.Input("target_position", Point, "Target position to reach"),
            Port.Input("tolerance", float, 0.5, "Distance tolerance (m)"),
            Port.Output("distance_to_goal", float, "Remaining distance")
        ])
    
    def __init__(self, name, config):
        super().__init__(name, config)
        self.node = config.blackboard.get("node")
        
        # ROS interfaces
        self.odom_sub = self.node.create_subscription(
            Odometry, 'odometry', self.odom_callback, 10)
        self.goal_pub = self.node.create_publisher(
            PoseStamped, 'goal', 10)
        
        self.current_pose = None
        self.target = None
        self.tolerance = 0.5
        
    def on_start(self):
        """Called when node starts"""
        self.node.get_logger().info("Starting YourAction")
        
        # Get inputs
        self.target = self.get_input("target_position")
        if self.target is None:
            self.node.get_logger().error("Missing target_position")
            return NodeStatus.FAILURE
        
        self.tolerance = self.get_input("tolerance", 0.5)
        
        # Publish goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = self.target
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        
        self.start_time = self.node.get_clock().now()
        return NodeStatus.RUNNING
    
    def on_running(self):
        """Called each tick while RUNNING"""
        if self.current_pose is None:
            return NodeStatus.RUNNING
        
        # Compute distance
        dx = self.target.x - self.current_pose.position.x
        dy = self.target.y - self.current_pose.position.y
        dz = self.target.z - self.current_pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Update output
        self.set_output("distance_to_goal", distance)
        
        # Check if reached
        if distance < self.tolerance:
            self.node.get_logger().info("Goal reached!")
            return NodeStatus.SUCCESS
        
        # Check timeout
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > 60.0:
            self.node.get_logger().warn("Action timeout")
            return NodeStatus.FAILURE
        
        return NodeStatus.RUNNING
    
    def on_halted(self):
        """Called when node is interrupted"""
        self.node.get_logger().info("YourAction halted")
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
```

### 7. Update CMakeLists.txt

Add your node to the build system:

```cmake
# Find behavior tree dependency
find_package(behavior_tree REQUIRED)

# Add your node library
add_library(your_behavior_nodes SHARED
  src/your_action_node.cpp
  src/your_condition_node.cpp
)

target_include_directories(your_behavior_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(your_behavior_nodes
  rclcpp
  behavior_tree
  airstack_msgs
  nav_msgs
  geometry_msgs
)

# Install library
install(TARGETS your_behavior_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/)
```

### 8. Register Nodes with Behavior Executive

Create XML description for the behavior executive:

**File:** `config/node_manifest.xml`

```xml
<?xml version="1.0"?>
<root>
  <TreeNodesModel>
    <!-- Your Action Node -->
    <Action ID="YourAction">
      <input_port name="target_position" type="geometry_msgs::Point">
        Target position to reach
      </input_port>
      <input_port name="tolerance" default="0.5" type="double">
        Distance tolerance (m)
      </input_port>
      <output_port name="distance_to_goal" type="double">
        Remaining distance to goal
      </output_port>
    </Action>
    
    <!-- Your Condition Node -->
    <Condition ID="YourCondition">
      <input_port name="threshold" default="20.0" type="double">
        Battery percentage threshold
      </input_port>
    </Condition>
  </TreeNodesModel>
</root>
```

### 9. Create Example Behavior Tree

Create an XML file demonstrating your node usage:

**File:** `trees/example_tree.xml`

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
  <BehaviorTree ID="ExampleMission">
    <Sequence name="Main Sequence">
      
      <!-- Check preconditions -->
      <YourCondition name="Check Battery" threshold="30.0"/>
      
      <!-- Execute action -->
      <YourAction name="Navigate to Waypoint"
                  target_position="{waypoint}"
                  tolerance="0.5"
                  distance_to_goal="{remaining_dist}"/>
      
      <!-- Log result -->
      <LogInfo message="Waypoint reached. Distance was: {remaining_dist}"/>
      
    </Sequence>
  </BehaviorTree>
</root>
```

### 10. Test Node Standalone

Create a test executable:

**File:** `test/test_your_node.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include "behavior_tree/bt_factory.hpp"
#include "behavior_tree/loggers/bt_cout_logger.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_test_node");
  
  // Create BT factory and register nodes
  BT::BehaviorTreeFactory factory;
  RegisterYourNodes(factory);  // Your registration function
  
  // Create blackboard and add node
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  
  // Create tree from XML
  auto tree = factory.createTreeFromFile("trees/example_tree.xml", blackboard);
  
  // Add logger
  BT::StdCoutLogger logger(tree);
  
  // Run tree
  rclcpp::Rate rate(10);  // 10 Hz
  while (rclcpp::ok()) {
    auto status = tree.tickRoot();
    
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Tree completed successfully");
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "Tree failed");
      break;
    }
    
    rclcpp::spin_some(node);
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}
```

Build and run:
```bash
docker exec airstack-robot-1 bash -c "bws --packages-select your_behavior_nodes"
docker exec airstack-robot-1 bash -c "sws && ros2 run your_behavior_nodes test_your_node"
```

### 11. Integrate with Behavior Executive

Add your nodes to the behavior executive configuration:

**File:** Update `behavior_executive` config to load your nodes

The behavior executive should auto-discover nodes from registered plugins.

### 12. Document Your Node

Create README documenting the node:

**File:** `README.md`

```markdown
# Your Behavior Nodes

## Nodes

### YourAction
**Type:** Action
**Description:** Performs [specific action]

#### Ports
- **Input:**
  - `target_position` (geometry_msgs::Point): Target position
  - `tolerance` (double, default=0.5): Distance tolerance
- **Output:**
  - `distance_to_goal` (double): Remaining distance

#### Example Usage
```xml
<YourAction name="Navigate" target_position="{goal}" tolerance="1.0"/>
```

### YourCondition
**Type:** Condition
**Description:** Checks [specific condition]

#### Ports
- **Input:**
  - `threshold` (double, default=20.0): Threshold value

#### Example Usage
```xml
<YourCondition name="Check" threshold="30.0"/>
```

## Example Trees
See `trees/example_tree.xml` for usage examples.
```

## Common Patterns

### Retry Pattern
```xml
<RetryUntilSuccessful num_attempts="3">
  <YourAction name="Attempt Action" target_position="{goal}"/>
</RetryUntilSuccessful>
```

### Timeout Pattern
```xml
<Timeout msec="30000">
  <YourAction name="Action with Timeout" target_position="{goal}"/>
</Timeout>
```

### Fallback Pattern
```xml
<Fallback name="Try alternatives">
  <YourAction name="Preferred Method" target_position="{goal}"/>
  <AlternativeAction name="Backup Method" target_position="{goal}"/>
</Fallback>
```

## Common Pitfalls

- ❌ **Not checking port inputs**
  - ✅ Always validate port inputs exist before use
- ❌ **Blocking in tick()**
  - ✅ Return RUNNING for long operations, continue on next tick
- ❌ **Not cleaning up in onHalted()**
  - ✅ Release resources when node is interrupted
- ❌ **Forgetting to register node**
  - ✅ Use BT_REGISTER_NODES macro

## References

- **Behavior Trees:**
  - [BehaviorTree.CPP](https://www.behaviortree.dev/)
  - [Introduction to Behavior Trees](https://www.behaviortree.dev/docs/intro)

- **AirStack Behavior:**
  - Framework: `robot/ros_ws/src/behavior/behavior_tree/`
  - Executive: `robot/ros_ws/src/behavior/behavior_executive/`
  - Examples: `robot/ros_ws/src/behavior/behavior_tree_example/`

- **Related Skills:**
  - [add-ros2-package](../add-ros2-package)
  - [test-in-simulation](../test-in-simulation)
