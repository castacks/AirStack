---
name: add-ros2-package
description: Create a new ROS 2 package for the AirStack autonomy stack. Use when implementing a new algorithm module (planner, controller, perception, world model, behavior node). Covers package structure, CMakeLists.txt, package.xml, launch files, and configuration.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Add a New ROS 2 Package to AirStack

## When to Use

Adding a new algorithm module (planner, controller, perception, world model, behavior node, etc.) to the autonomy stack.

## Prerequisites

- Understand which layer the module belongs to (interface/sensors/perception/local/global/behavior)
- Know the intended input/output topics for the module
- Have algorithm requirements and dependencies defined
- Familiarity with ROS 2 package structure

## Steps

### 1. Choose Package Location

Determine where the package should live based on its function:

**Local Layer:**
- Local planner: `robot/ros_ws/src/local/planners/<package_name>`
- Local controller: `robot/ros_ws/src/local/c_controls/<package_name>`
- Local world model: `robot/ros_ws/src/local/world_models/<package_name>`

**Global Layer:**
- Global planner: `robot/ros_ws/src/global/planners/<package_name>`
- Global world model: `robot/ros_ws/src/global/world_models/<package_name>`

**Other Layers:**
- Perception: `robot/ros_ws/src/perception/<package_name>`
- Sensors: `robot/ros_ws/src/sensors/<package_name>`
- Behavior: `robot/ros_ws/src/behavior/<package_name>`
- Interface: `robot/ros_ws/src/interface/<package_name>`

### 2. Create Package Structure

Use the template from `assets/package_template/` or create the following structure manually:

```
<package_name>/
├── CMakeLists.txt          # For C++ packages
├── setup.py                # For Python packages
├── package.xml             # Package metadata and dependencies
├── config/
│   └── <package_name>.yaml # Default parameters
├── launch/
│   └── <package_name>.launch.xml  # Launch file with remapping
├── include/<package_name>/  # C++ headers (for C++ packages)
│   └── <node_name>.hpp
├── src/                    # Source code
│   └── <node_name>.cpp     # or .py for Python
├── test/                   # Unit and integration tests
│   └── test_<node_name>.cpp
└── README.md               # Documentation
```

**Create the directory:**
```bash
mkdir -p <full_path_to_package>
cd <full_path_to_package>
```

### 3. Create package.xml

Define package metadata and dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>your_package_name</name>
  <version>0.0.1</version>
  <description>Brief description of your module</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Common AirStack dependencies -->
  <depend>rclcpp</depend>  <!-- For C++; use rclpy for Python -->
  <depend>airstack_msgs</depend>
  <depend>airstack_common</depend>
  
  <!-- Standard ROS 2 message packages -->
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <!-- Transforms -->
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  
  <!-- Add other dependencies as needed -->
  <!-- <depend>pluginlib</depend> -->
  <!-- <depend>trajectory_controller</depend> -->

  <!-- Testing -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Reference:** `robot/ros_ws/src/local/planners/droan_local_planner/package.xml`

### 4. Create CMakeLists.txt (for C++)

```cmake
cmake_minimum_required(VERSION 3.8)
project(your_package_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# Add other dependencies as needed

# Create executable
add_executable(your_node_name src/your_node.cpp)

target_include_directories(your_node_name PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(your_node_name PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  your_node_name
  "rclcpp"
  "airstack_msgs"
  "airstack_common"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
)

# Install executable
install(TARGETS your_node_name
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

**Reference:** `robot/ros_ws/src/local/planners/droan_local_planner/CMakeLists.txt`

### 5. Create setup.py (for Python)

For Python-only packages:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'your_package_name'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Brief description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'your_node_name = your_package_name.your_node:main'
        ],
    },
)
```

### 6. Implement the Module

Create your ROS 2 node with proper topic interfaces:

**C++ Example Structure:**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class YourNode : public rclcpp::Node
{
public:
  YourNode() : Node("your_node_name")
  {
    // Declare parameters
    this->declare_parameter<double>("param_name", 1.0);
    
    // Create subscriptions (use topic names without namespaces - remapped in launch)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10, std::bind(&YourNode::odomCallback, this, std::placeholders::_1));
    
    // Create publishers
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Create timer for periodic processing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&YourNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Your node initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Process incoming data
  }
  
  void timerCallback()
  {
    // Periodic processing and publishing
    auto cmd_msg = geometry_msgs::msg::Twist();
    // ... populate message
    cmd_pub_->publish(cmd_msg);
  }
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YourNode>());
  rclcpp::shutdown();
  return 0;
}
```

**Python Example Structure:**
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node_name')
        
        # Declare parameters
        self.declare_parameter('param_name', 1.0)
        
        # Create subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry',
            self.odom_callback,
            10)
        
        # Create publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Your node initialized')
    
    def odom_callback(self, msg):
        # Process incoming data
        pass
    
    def timer_callback(self):
        # Periodic processing and publishing
        cmd_msg = Twist()
        # ... populate message
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7. Create Configuration File

Create `config/<package_name>.yaml` with default parameters:

```yaml
/**:
  ros__parameters:
    # Algorithm parameters
    param_name: 1.0
    another_param: "value"
    
    # Timing parameters
    update_rate: 10.0
    
    # Thresholds
    threshold: 0.5
```

### 8. Create Launch File

Create `launch/<package_name>.launch.xml` with topic remapping:

```xml
<launch>
  <!-- Launch arguments for topic remapping -->
  <arg name="odometry_topic" default="/robot/odometry" />
  <arg name="output_topic" default="/robot/cmd_vel" />
  <arg name="config_file" default="$(find-pkg-share your_package_name)/config/your_package_name.yaml" />

  <node pkg="your_package_name" 
        exec="your_node_name" 
        name="your_node_name" 
        output="screen">
    
    <!-- Load parameters -->
    <param from="$(var config_file)" allow_substs="true" />
    
    <!-- Remap topics -->
    <remap from="odometry" to="$(var odometry_topic)" />
    <remap from="cmd_vel" to="$(var output_topic)" />
  </node>
</launch>
```

**Key points:**
- Use `allow_substs="true"` to enable environment variable substitution in config files
- Define launch arguments for all topic names (enables flexible remapping)
- Use `$(var arg_name)` to reference launch arguments
- Use `$(env VAR_NAME)` for environment variables in configs

### 9. Create Module README.md

Document your module using the template structure:

```markdown
# Your Module Name

## Overview
Brief description of what this module does.

## Algorithm
Explanation of the algorithm or approach used.

## Dependencies
- ROS 2 packages
- External libraries

## Interfaces

### Subscribed Topics
- `odometry` (nav_msgs/Odometry): Description

### Published Topics
- `cmd_vel` (geometry_msgs/Twist): Description

### Parameters
- `param_name` (double, default: 1.0): Description

## Configuration
Explanation of config file options.

## Usage
```bash
ros2 launch your_package_name your_package_name.launch.xml
```

## Testing
How to test the module.
```

**See:** `assets/package_template/README.md` for full template.

### 10. Build the Package

Build and test your package:

```bash
# From outside the container
AUTOLAUNCH=false airstack up robot-desktop

# Build the specific package
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package_name"

# Source the workspace
docker exec airstack-robot-desktop-1 bash -c "sws"

# Test launch
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch your_package_name your_package_name.launch.xml"
```

### 11. Verify Package Works

Check that the node is running and topics are connected:

```bash
# List nodes
docker exec airstack-robot-desktop-1 bash -c "ros2 node list | grep your_node"

# List topics
docker exec airstack-robot-desktop-1 bash -c "ros2 topic list | grep your_topic"

# Check topic publishing rate
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /your/topic"

# Echo topic data
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /your/topic --once"
```

## Next Steps

After creating the package:

1. **Integrate into layer bringup:** Follow [integrate-module-into-layer](../integrate-module-into-layer)
2. **Update documentation:** Follow [update-documentation](../update-documentation)
3. **Test in simulation:** Follow [test-in-simulation](../test-in-simulation)

## Common Pitfalls

### Build Errors
- ❌ Missing dependencies in `package.xml`
  - ✅ Add all ROS 2 packages and external libraries used
- ❌ Not installing launch/config files in `CMakeLists.txt`
  - ✅ Use `install(DIRECTORY ...)` for launch and config directories
- ❌ Header file not found
  - ✅ Ensure `include/` directory is properly configured in `target_include_directories`

### Runtime Errors
- ❌ Topics not connecting
  - ✅ Use `ros2 topic info <topic>` to check publishers/subscribers
  - ✅ Verify topic remapping in launch file
- ❌ Parameters not loading
  - ✅ Check config file path with `$(find-pkg-share ...)`
  - ✅ Ensure `allow_substs="true"` is set in param tag
- ❌ Node crashes on start
  - ✅ Check logs: `docker logs airstack-robot-desktop-1`
  - ✅ Verify all dependencies are built and sourced

### Launch File Issues
- ❌ Hardcoded topic names
  - ✅ Use launch arguments for all topics
- ❌ Missing robot namespace
  - ✅ Use `$(env ROBOT_NAME)` for multi-robot support
- ❌ Environment variables not substituting in config
  - ✅ Add `allow_substs="true"` to param tag

## References

- **ROS 2 Documentation:**
  - [Creating a Package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
  - [Writing a Simple Publisher/Subscriber](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
  - [Using Parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
  
- **AirStack Examples:**
  - Reference local planner: `robot/ros_ws/src/local/planners/droan_local_planner`
  - Reference global planner / behavior manager: `robot/ros_ws/src/global/planners/raven_nav` (raven_nav — behavior-driven global planner)
  - Reference controller: `robot/ros_ws/src/local/c_controls/trajectory_controller`
  - Reference perception package: `common/rayfronts` (Rayfronts — now a first-class ROS 2 package with its own `package.xml`, launched from `robot/ros_ws/src/perception/perception_bringup/launch/rayfronts.launch.xml`)
  - Package template: `assets/package_template/`

- **Next Skills:**
  - [integrate-module-into-layer](../integrate-module-into-layer)
  - [update-documentation](../update-documentation)
  - [debug-module](../debug-module)
