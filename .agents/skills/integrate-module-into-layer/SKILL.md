---
name: integrate-module-into-layer
description: Integrate a ROS 2 module into the appropriate layer bringup package. Use after creating a package to add it to the autonomy stack launch flow. Covers topic remapping, namespace configuration, and bringup package integration.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Integrate Module into Layer Bringup

## When to Use

After creating a new ROS 2 package, integrate it into the appropriate layer's bringup package so it launches automatically with the autonomy stack.

## Prerequisites

- Module package created and tested standalone
- Package builds successfully with `bws --packages-select <package_name>`
- Know which layer the module belongs to
- Understand required topic connections to other modules

## Bringup Package Overview

Each layer in the autonomy stack has a corresponding bringup package that orchestrates launching all modules in that layer with proper topic remapping:

| Layer | Bringup Package | Location |
|-------|----------------|----------|
| Interface | interface_bringup | `robot/ros_ws/src/interface/interface_bringup` |
| Sensors | sensors_bringup | `robot/ros_ws/src/sensors/sensors_bringup` |
| Perception | perception_bringup | `robot/ros_ws/src/perception/perception_bringup` |
| Local | local_bringup | `robot/ros_ws/src/local/local_bringup` |
| Global | global_bringup | `robot/ros_ws/src/global/global_bringup` |
| Behavior | behavior_bringup | `robot/ros_ws/src/behavior/behavior_bringup` |

The top-level orchestration is in `autonomy_bringup` which calls each layer's bringup.

## Steps

### 1. Identify the Correct Bringup Package

Based on your module type, identify the bringup package to modify:

```bash
# List bringup packages
ls -la robot/ros_ws/src/*/launch/*.launch.xml
ls -la robot/ros_ws/src/*/*_bringup/launch/*.launch.xml
```

For example:
- New local planner → `local_bringup`
- New global planner → `global_bringup`
- New perception module → `perception_bringup`

### 2. Study the Existing Launch File

Before adding your module, understand the current structure:

```bash
# View the main launch file for your layer
cat robot/ros_ws/src/local/local_bringup/launch/local.launch.xml
```

**Key patterns to observe:**
- Launch arguments for topic remapping
- Namespace usage with `push-ros-namespace`
- Topic remapping to AirStack standard topics
- Parameter file loading with `allow_substs="true"`
- Environment variable usage (`$(env ROBOT_NAME)`)

**Example from local.launch.xml:**
```xml
<launch>
    <!-- Launch arguments for shared topics -->
    <arg name="local_odometry_in_topic" default="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
    <arg name="local_global_plan_topic" default="/$(env ROBOT_NAME)/global_plan" />
    
    <!-- Existing module -->
    <group>
        <push-ros-namespace namespace="existing_planner" />
        <node pkg="existing_planner" exec="planner_node" output="screen">
            <param from="$(find-pkg-share existing_planner)/config/config.yaml" allow_substs="true" />
            <remap from="odometry" to="$(var local_odometry_in_topic)" />
            <remap from="global_plan" to="$(var local_global_plan_topic)" />
        </node>
    </group>
</launch>
```

### 3. Add Your Module to the Launch File

Edit the layer's main launch file to include your module:

```xml
<!-- Your New Module -->
<group>
  <push-ros-namespace namespace="your_module_namespace" />
  
  <node pkg="your_package_name" 
        exec="your_node_name" 
        name="your_node_name"
        output="screen">
    
    <!-- Load parameters from config -->
    <param from="$(find-pkg-share your_package_name)/config/your_package_name.yaml" 
           allow_substs="true" />
    
    <!-- Remap input topics to AirStack standard topics -->
    <remap from="odometry" to="$(var local_odometry_in_topic)" />
    <remap from="global_plan" to="/$(env ROBOT_NAME)/global_plan" />
    
    <!-- Remap output topics to controllers/other modules -->
    <remap from="local_trajectory" 
           to="/$(env ROBOT_NAME)/trajectory_controller/trajectory_segment_to_add" />
  </node>
</group>
```

**Important remapping patterns:**

For **local planners**:
```xml
<!-- Inputs -->
<remap from="odometry" to="$(var local_odometry_in_topic)" />
<remap from="global_plan" to="/$(env ROBOT_NAME)/global_plan" />
<remap from="look_ahead" to="/$(env ROBOT_NAME)/trajectory_controller/look_ahead" />

<!-- Outputs -->
<remap from="trajectory_segment" 
       to="/$(env ROBOT_NAME)/trajectory_controller/trajectory_segment_to_add" />
```

For **global planners**:
```xml
<!-- Inputs -->
<remap from="odometry" to="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
<remap from="global_map" to="/$(env ROBOT_NAME)/mapping/global_map" />

<!-- Outputs -->
<remap from="global_plan" to="/$(env ROBOT_NAME)/global_plan" />
```

For **controllers**:
```xml
<!-- Inputs -->
<remap from="odometry" to="$(var local_odometry_in_topic)" />
<remap from="trajectory" to="/$(env ROBOT_NAME)/local_planner/trajectory" />

<!-- Outputs -->
<remap from="cmd_vel" to="/$(env ROBOT_NAME)/interface/cmd_vel" />
```

### 4. Add Launch Arguments (if needed)

If your module needs configurable inputs, add launch arguments at the top of the file:

```xml
<launch>
    <!-- Existing arguments -->
    <arg name="local_odometry_in_topic" default="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
    
    <!-- Your new arguments -->
    <arg name="your_module_input_topic" default="/$(env ROBOT_NAME)/default/input" />
    <arg name="your_module_config" default="$(find-pkg-share your_package_name)/config/default.yaml" />
    
    <!-- ... rest of launch file ... -->
    
    <!-- Use the argument in your module -->
    <node pkg="your_package_name" exec="your_node" output="screen">
        <param from="$(var your_module_config)" allow_substs="true" />
        <remap from="input" to="$(var your_module_input_topic)" />
    </node>
</launch>
```

### 5. Update Bringup Package Dependencies

Edit the bringup package's `package.xml` to add your module as a dependency:

```bash
# Edit the bringup package.xml
vi robot/ros_ws/src/local/local_bringup/package.xml
```

Add your package:
```xml
<package format="3">
  <name>local_bringup</name>
  <!-- ... existing content ... -->
  
  <!-- Existing dependencies -->
  <depend>droan_local_planner</depend>
  <depend>trajectory_controller</depend>
  
  <!-- Add your package -->
  <depend>your_package_name</depend>
  
  <!-- ... rest of package.xml ... -->
</package>
```

### 6. Consider Conditional Launching (Future/Optional)

For implementing a plugin-style architecture where users can select which module to use, add conditional launching:

```xml
<!-- Launch argument to enable/disable module -->
<arg name="use_your_planner" default="false" />

<!-- Conditional group -->
<group if="$(var use_your_planner)">
  <push-ros-namespace namespace="your_planner" />
  <node pkg="your_package_name" exec="your_node" output="screen">
    <!-- ... configuration ... -->
  </node>
</group>

<!-- Alternative module (mutually exclusive) -->
<group unless="$(var use_your_planner)">
  <push-ros-namespace namespace="default_planner" />
  <node pkg="default_planner" exec="planner_node" output="screen">
    <!-- ... configuration ... -->
  </node>
</group>
```

This pattern allows runtime selection of implementations.

### 7. Rebuild the Bringup Package

After modifying the launch file:

```bash
# Rebuild bringup package (picks up new launch file)
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select local_bringup"

# Rebuild your module too (if needed)
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package_name"
```

### 8. Test Full Autonomy Launch

Launch the complete autonomy stack to test integration:

```bash
# Stop any running containers
airstack stop

# Launch with full autonomy
AUTOLAUNCH=true airstack up robot

# Or launch specific components
AUTOLAUNCH=true airstack up robot isaac-sim
```

### 9. Verify Integration

Check that your module is running and connected:

```bash
# List all running nodes (should see your node)
docker exec airstack-robot-desktop-1 bash -c "ros2 node list | grep your_node"

# Check your module's topics
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot_name/namespace/your_node"

# Verify topic connections
docker exec airstack-robot-desktop-1 bash -c "ros2 topic info /robot_name/your/output/topic"

# Check data flow
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /robot_name/your/output/topic"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot_name/your/output/topic --once"
```

### 10. Test with Other Modules

Verify your module integrates correctly with the rest of the system:

```bash
# Check end-to-end data flow
# For a planner: verify it receives odometry and publishes trajectories
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /robot_name/odometry"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /robot_name/trajectory_controller/trajectory_segment_to_add"

# Visualize in RViz (if GCS is running)
# Topics should appear in RViz topic list
```

### 11. Update Layer Documentation

Document the integration in the layer's overview:

Edit `docs/robot/autonomy/<layer>/index.md`:

```markdown
## Available Modules

### Your Module Name
Brief description of your module and its purpose.
- **Location:** `robot/ros_ws/src/<layer>/<category>/your_package_name`
- **Documentation:** [Your Module README](../../../robot/ros_ws/src/<layer>/<category>/your_package_name/README.md)
```

## Common Integration Patterns

### Pattern 1: Parallel Processing Modules

Multiple modules process data independently:

```xml
<!-- Module A -->
<group>
  <push-ros-namespace namespace="module_a" />
  <node pkg="module_a" exec="node_a" output="screen">
    <remap from="input" to="$(var shared_input)" />
    <remap from="output" to="/robot/module_a/output" />
  </node>
</group>

<!-- Module B (independent) -->
<group>
  <push-ros-namespace namespace="module_b" />
  <node pkg="module_b" exec="node_b" output="screen">
    <remap from="input" to="$(var shared_input)" />
    <remap from="output" to="/robot/module_b/output" />
  </node>
</group>
```

### Pattern 2: Sequential Processing Pipeline

Modules process data in sequence:

```xml
<!-- Stage 1 -->
<node pkg="stage1" exec="processor1" output="screen">
  <remap from="input" to="/robot/raw_data" />
  <remap from="output" to="/robot/processed_stage1" />
</node>

<!-- Stage 2 (depends on Stage 1 output) -->
<node pkg="stage2" exec="processor2" output="screen">
  <remap from="input" to="/robot/processed_stage1" />
  <remap from="output" to="/robot/final_output" />
</node>
```

### Pattern 3: Optional/Conditional Module

Module only launches under certain conditions:

```xml
<arg name="enable_optional_module" default="$(env ENABLE_MODULE false)" />

<group if="$(var enable_optional_module)">
  <node pkg="optional_module" exec="optional_node" output="screen">
    <!-- ... configuration ... -->
  </node>
</group>
```

## Common Pitfalls

### Topic Connection Issues
- ❌ **Hardcoded topic names in module code**
  - ✅ Use generic names in code, remap in launch file
- ❌ **Wrong topic remapping direction**
  - ✅ `<remap from="local_name" to="global_name" />` (from = in code, to = actual topic)
- ❌ **Missing robot namespace**
  - ✅ Always include `$(env ROBOT_NAME)` in topic paths for multi-robot support

### Launch File Issues
- ❌ **Forgetting `allow_substs="true"`**
  - ✅ Required for environment variable substitution in config files
- ❌ **Not using `push-ros-namespace`**
  - ✅ Properly namespace your nodes to avoid conflicts
- ❌ **Missing `output="screen"`**
  - ✅ Add to see node logs in terminal/docker logs

### Build Issues
- ❌ **Not rebuilding bringup package**
  - ✅ Launch files are installed at build time, must rebuild after changes
- ❌ **Missing dependency in bringup package.xml**
  - ✅ Add your package to bringup's dependencies
- ❌ **Not sourcing workspace**
  - ✅ Source after every build: `sws`

### Integration Issues
- ❌ **Module launches but doesn't receive data**
  - ✅ Check topic remapping with `ros2 topic info`
  - ✅ Verify publishers exist: `ros2 topic hz <topic>`
- ❌ **Multiple nodes publishing to same topic**
  - ✅ Check for topic conflicts with `ros2 topic info`
  - ✅ Use namespaces to separate nodes
- ❌ **Module crashes on launch**
  - ✅ Check docker logs: `docker logs airstack-robot-desktop-1`
  - ✅ Verify dependencies are built

## Debugging Integration

If integration fails, use systematic debugging:

```bash
# 1. Verify your node is in the node list
docker exec airstack-robot-desktop-1 bash -c "ros2 node list"

# 2. Check node info (topics, services, actions)
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot_name/namespace/your_node"

# 3. Verify expected input topics exist and are publishing
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /expected/input/topic"

# 4. Check output topics are being published
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /your/output/topic"

# 5. Inspect actual topic data
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /your/output/topic --once"

# 6. Check for errors in logs
docker logs airstack-robot-desktop-1 2>&1 | grep -i error

# 7. View full launch output
docker logs airstack-robot-desktop-1
```

See [debug-module](../debug-module) for comprehensive debugging strategies.

## Next Steps

After successful integration:

1. **Update documentation:** Follow [update-documentation](../update-documentation)
2. **Test in simulation:** Follow [test-in-simulation](../test-in-simulation)
3. **Create integration tests:** Add tests to verify module works with full stack

## References

- **AirStack Launch Files:**
  - Top-level: `robot/ros_ws/src/autonomy_bringup/launch/robot.launch.xml`
  - Local layer: `robot/ros_ws/src/local/local_bringup/launch/local.launch.xml`
  - Global layer: `robot/ros_ws/src/global/global_bringup/launch/global.launch.xml`
  
- **ROS 2 Launch Documentation:**
  - [ROS 2 Launch Files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
  - [Launch XML Format](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
  
- **Related Skills:**
  - [add-ros2-package](../add-ros2-package) - Creating the module
  - [debug-module](../debug-module) - Debugging integration issues
  - [update-documentation](../update-documentation) - Documenting integration
