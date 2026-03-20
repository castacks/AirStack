---
name: debug-module
description: Systematically debug ROS 2 modules with autonomous diagnostic strategies. Use when a module is not working as expected. Covers node status, topic connections, data flow analysis, parameter checking, and performance profiling.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Autonomously Debug a ROS 2 Module

## When to Use

When a module is not working as expected and you need to systematically diagnose and fix issues. This skill is designed for **autonomous debugging** by AI agents.

## Debugging Philosophy

1. **Systematic approach:** Work through diagnostic steps methodically
2. **Collect evidence:** Gather logs, topic data, and system state before forming hypotheses
3. **Isolate the problem:** Narrow down whether issue is in the module, integration, or environment
4. **Test hypotheses:** Make targeted changes and verify effects
5. **Document findings:** Record root cause and solution

## Debugging Strategy

### Level 1: Is the Module Running?
### Level 2: Are Topics Connected?
### Level 3: Is Data Flowing Correctly?
### Level 4: Is the Algorithm Working?
### Level 5: Is the Integration Correct?

## Steps

### Level 1: Verify Module is Running

First, confirm your module's node is actually running:

```bash
# List all running nodes
docker exec airstack-robot-desktop-1 bash -c "ros2 node list"

# Expected output should include:
# /robot_name/namespace/your_node_name

# If node not found, check if container is running
airstack status

# Check docker logs for startup errors
docker logs airstack-robot-desktop-1 2>&1 | tail -50

# Look for your node's initialization message
docker logs airstack-robot-desktop-1 2>&1 | grep -i "your_node\|your_package"

# Check for crash/error messages
docker logs airstack-robot-desktop-1 2>&1 | grep -i "error\|fail\|crash\|segmentation"
```

**Common issues at this level:**
- Node name mismatch (check launch file vs. node constructor)
- Package not built or not sourced
- Missing dependencies causing import failures
- Node crashing immediately on startup

**Fix strategies:**
- Rebuild package: `docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package"`
- Check build output for errors
- Verify dependencies in package.xml
- Test node standalone: `docker exec airstack-robot-desktop-1 bash -c "sws && ros2 run your_package your_node"`

### Level 2: Check Topic Connections

Verify that topics are properly connected:

```bash
# Get detailed info about your node
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot_name/namespace/your_node_name"

# This shows:
# - Subscribers (input topics)
# - Publishers (output topics)
# - Services
# - Actions

# Check specific topic publishers/subscribers
docker exec airstack-robot-desktop-1 bash -c "ros2 topic info /your/topic/name"

# Expected output:
# Publisher count: X
# Subscription count: Y

# List all topics (to find misnamed topics)
docker exec airstack-robot-desktop-1 bash -c "ros2 topic list | grep -i relevant_keyword"
```

**Common issues at this level:**
- Topic name typo or incorrect remapping
- Missing publisher for subscribed topic
- Multiple publishers causing conflicts
- Wrong namespace

**Fix strategies:**
- Verify topic remapping in launch file
- Check for typos in topic names
- Ensure upstream nodes are running
- Verify namespace with `$(env ROBOT_NAME)`

### Level 3: Check Data Flow

Verify data is actually flowing through topics:

```bash
# Check publishing rate
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /your/topic/name"

# Expected output:
# average rate: X.XXX
#         min: X.XXX s max: X.XXX s std dev: X.XXXXX s window: N

# If rate is 0, no data is flowing

# Echo topic to see actual data
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /your/topic/name --once"

# Check input topics are publishing
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /input/topic/name"

# Monitor multiple topics simultaneously
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /topic1 /topic2 /topic3"
```

**Common issues at this level:**
- Input topic not publishing (upstream node issue)
- Publishing rate too slow/fast
- Data format incorrect
- Timing/synchronization issues

**Fix strategies:**
- Verify upstream nodes are working
- Check callback frequency in code
- Verify topic types match: `ros2 topic type /topic`
- Check for dropped messages in logs

### Level 4: Inspect Data Quality

Examine the actual data being published/received:

```bash
# Echo and save topic data for analysis
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /your/topic/name --once" > topic_data.txt

# For continuous monitoring (save to file in container)
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /your/topic/name" > /tmp/topic_log.txt
# Then Ctrl+C after collecting data
# docker cp airstack-robot-desktop-1:/tmp/topic_log.txt ./topic_log.txt

# Check message structure
docker exec airstack-robot-desktop-1 bash -c "ros2 interface show sensor_msgs/msg/Image"

# Look for specific values in topic data
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /your/topic --once" | grep -i "field_name"
```

**Analyze data for:**
- NaN or infinity values
- Values out of expected range
- Constant values (should be changing)
- Coordinate frame mismatches
- Timestamp issues

**Common issues at this level:**
- Uninitialized variables (NaN values)
- Wrong coordinate frame transformations
- Incorrect units (meters vs. millimeters)
- Timestamp not set correctly

### Level 5: Review Node Parameters

Check if parameters are loaded correctly:

```bash
# List all parameters for your node
docker exec airstack-robot-desktop-1 bash -c "ros2 param list /robot_name/namespace/your_node"

# Get specific parameter value
docker exec airstack-robot-desktop-1 bash -c "ros2 param get /robot_name/namespace/your_node param_name"

# Dump all parameters to YAML
docker exec airstack-robot-desktop-1 bash -c "ros2 param dump /robot_name/namespace/your_node" > params.yaml
```

**Common issues at this level:**
- Parameters not loading from YAML
- Default values used instead of config values
- Parameter type mismatch
- Missing `allow_substs="true"` for env variables

**Fix strategies:**
- Verify config file path in launch file
- Check YAML syntax (indentation, formatting)
- Ensure parameter names match between code and YAML
- Add `allow_substs="true"` to param tag

### Level 6: Check Transforms (if applicable)

For modules using coordinate frames:

```bash
# Check transform between frames
docker exec airstack-robot-desktop-1 bash -c "ros2 run tf2_ros tf2_echo source_frame target_frame"

# Generate TF tree
docker exec airstack-robot-desktop-1 bash -c "ros2 run tf2_tools view_frames"
# This creates frames.pdf - copy out:
# docker cp airstack-robot-desktop-1:/frames.pdf ./frames.pdf

# List available frames
docker exec airstack-robot-desktop-1 bash -c "ros2 run tf2_ros tf2_monitor"
```

**Common issues at this level:**
- Missing transform between required frames
- Transform not updating (static when should be dynamic)
- Wrong transform parent/child relationship
- Frame_id in messages doesn't match TF tree

### Level 7: Add Instrumentation

If the problem isn't obvious, add debugging outputs:

**C++ instrumentation:**
```cpp
// Add to your node
RCLCPP_INFO(this->get_logger(), "Debug: processing callback");
RCLCPP_WARN(this->get_logger(), "Value out of range: %f", value);
RCLCPP_ERROR(this->get_logger(), "Critical error occurred");

// Add debug publisher
debug_pub_ = this->create_publisher<std_msgs::msg::Float64>("debug/value", 10);

// Publish debug data
auto msg = std_msgs::msg::Float64();
msg.data = debug_value;
debug_pub_->publish(msg);
```

**Python instrumentation:**
```python
# Add to your node
self.get_logger().info(f'Debug: processing callback')
self.get_logger().warn(f'Value out of range: {value}')
self.get_logger().error(f'Critical error occurred')

# Add debug publisher
self.debug_pub = self.create_publisher(Float64, 'debug/value', 10)

# Publish debug data
msg = Float64()
msg.data = debug_value
self.debug_pub.publish(msg)
```

Rebuild and test:
```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package"
docker logs airstack-robot-desktop-1 -f | grep -i debug
```

### Level 8: Use GDB for C++ Crashes

If node crashes or segfaults:

```bash
# Rebuild with debug symbols
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"

# Run with GDB
docker exec -it airstack-robot-desktop-1 bash
gdb --args ros2 run your_package your_node

# In GDB:
(gdb) run
# Wait for crash
(gdb) backtrace  # Show call stack
(gdb) frame N    # Examine specific frame
(gdb) print variable_name  # Inspect variables
```

**Common crash causes:**
- Null pointer dereference
- Array out of bounds
- Memory corruption
- Uninitialized variables

### Level 9: Compare with Reference Implementation

Study a working similar module:

```bash
# Find reference implementation
# Example: if debugging a planner, study droan_local_planner

# Compare:
# 1. Package structure
ls -la robot/ros_ws/src/local/planners/droan_local_planner/
ls -la robot/ros_ws/src/local/planners/your_planner/

# 2. Topic structure
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot/droan/droan_planner"
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot/your_planner/your_node"

# 3. Message timing
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /robot/droan/output"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /robot/your_planner/output"

# 4. Data ranges
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot/droan/output --once"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot/your_planner/output --once"
```

### Level 10: Create Minimal Reproduction Test

Isolate the problem with a minimal test:

```cpp
// test/test_your_module.cpp
#include <gtest/gtest.h>
#include "your_package/your_node.hpp"

TEST(YourNodeTest, BasicFunctionality) {
  rclcpp::init(0, nullptr);
  
  // Create node
  auto node = std::make_shared<YourNode>();
  
  // Create mock input
  auto input_msg = sensor_msgs::msg::Image();
  // ... populate with test data
  
  // Trigger processing
  node->process(input_msg);
  
  // Verify output
  EXPECT_TRUE(node->has_output());
  EXPECT_GT(node->get_output().size(), 0);
  
  rclcpp::shutdown();
}
```

Build and run test:
```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select your_package"
docker exec airstack-robot-desktop-1 bash -c "sws && colcon test --packages-select your_package"
docker exec airstack-robot-desktop-1 bash -c "colcon test-result --test-result-base build/your_package"
```

## Systematic Debugging Workflow

**For AI Agents - follow this sequence:**

1. **Check node status**
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "ros2 node list | grep your_node"
   ```

2. **If node not running, check logs**
   ```bash
   docker logs airstack-robot-desktop-1 2>&1 | grep -A 10 -B 10 "your_package\|your_node"
   ```

3. **If node running, check topics**
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "ros2 node info /robot/namespace/your_node"
   ```

4. **Check data flow**
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /output/topic"
   docker exec airstack-robot-desktop-1 bash -c "ros2 topic hz /input/topic"
   ```

5. **Inspect data**
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /output/topic --once"
   ```

6. **Form hypothesis** based on evidence collected

7. **Test fix** with targeted code changes

8. **Verify fix** with same diagnostic commands

## Common Issues and Solutions

| Symptom | Likely Cause | Diagnostic | Solution |
|---------|--------------|------------|----------|
| Node not in node list | Build/launch failure | Check docker logs | Rebuild package, check dependencies |
| Topic not publishing | No subscribers or logic error | `ros2 topic hz` | Check callback, add debug prints |
| Wrong topic name | Remapping error | `ros2 topic list` | Fix launch file remapping |
| NaN values | Uninitialized variables | `ros2 topic echo` | Initialize all variables |
| Slow publishing | Heavy computation | `ros2 topic hz` | Profile code, optimize |
| Segfault | Memory error | Use GDB | Check pointers, array bounds |
| Missing frames | TF not published | `tf2_echo` | Verify transforms |
| Wrong output | Algorithm error | Echo and analyze | Add instrumentation, test logic |

## Advanced Debugging: Performance Profiling

### CPU Profiling

```bash
# Install profiling tools (if not in container)
docker exec -it airstack-robot-desktop-1 bash
apt-get update && apt-get install -y linux-tools-generic

# Profile node
perf record -g ros2 run your_package your_node
perf report
```

### Memory Profiling

```bash
# Use valgrind for memory leaks
docker exec -it airstack-robot-desktop-1 bash
apt-get install -y valgrind

valgrind --leak-check=full ros2 run your_package your_node
```

## Debugging Integration Issues

When module works standalone but fails in full stack:

1. **Check topic conflicts**
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "ros2 topic info /conflicting/topic"
   # Should show exactly 1 publisher
   ```

2. **Check namespace isolation**
   ```bash
   docker exec airstack-robot-desktop-1 bash -c "ros2 node list"
   # All nodes should have proper namespaces
   ```

3. **Check message ordering**
   - Record bag: `ros2 bag record -a`
   - Analyze timing in bag file

4. **Check resource usage**
   ```bash
   docker stats airstack-robot-desktop-1
   # Monitor CPU, memory usage
   ```

## Agent Debugging Workflow

As an AI agent, follow this systematic approach:

```
1. RUN diagnostic commands → COLLECT evidence
2. ANALYZE evidence → IDENTIFY patterns
3. FORM hypothesis → What could cause this?
4. TEST hypothesis → Make targeted change
5. VERIFY fix → Re-run diagnostics
6. DOCUMENT → Record root cause and solution
```

**Example debugging session:**

```
Agent: Let me check if the node is running...
[runs: ros2 node list]
Evidence: Node not in list

Agent: Checking docker logs for errors...
[runs: docker logs ... | grep package_name]
Evidence: ImportError: No module named 'scipy'

Hypothesis: Missing Python dependency

Test: Add scipy to package.xml and rebuild
[rebuilds package]

Verify: Node now appears in ros2 node list ✓
Solution: Added scipy dependency
```

## References

- **ROS 2 Debugging:**
  - [ROS 2 Troubleshooting](https://docs.ros.org/en/jazzy/How-To-Guides/Troubleshooting.html)
  - [ROS 2 Command Line Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)

- **GDB:**
  - [GDB Tutorial](https://www.sourceware.org/gdb/documentation/)
  - [Debugging with GDB](https://docs.ros.org/en/jazzy/Tutorials/Debugging/Debugging-CPP.html)

- **AirStack:**
  - [Integration Checklist](../docs/robot/autonomy/integration_checklist.md)
  - [System Architecture](../docs/robot/autonomy/system_architecture.md)

- **Related Skills:**
  - [add-ros2-package](../add-ros2-package)
  - [integrate-module-into-layer](../integrate-module-into-layer)
  - [test-in-simulation](../test-in-simulation)
