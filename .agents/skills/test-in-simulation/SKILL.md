---
name: test-in-simulation
description: Test modules in Isaac Sim simulation environment end-to-end. Use after implementing and integrating a module to verify functionality. Covers test scenarios, monitoring, recording, and analysis of simulation tests.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Test Module in Simulation

## When to Use

After implementing and integrating a module, test it in the full Isaac Sim simulation environment to verify end-to-end functionality.

## Prerequisites

- Module built and integrated into autonomy stack
- Isaac Sim container accessible
- Understanding of expected module behavior
- Test scenario planned

## Testing Strategy

1. **Unit tests:** Verify module logic in isolation (covered in module development)
2. **Integration tests:** Test module with mock inputs
3. **Simulation tests:** Full autonomy stack in Isaac Sim (this skill)
4. **Hardware tests:** Deploy to real robot (separate workflow)

This skill focuses on **simulation testing**.

## Steps

### 1. Prepare Test Environment

Ensure containers are ready:

```bash
# Check container status
airstack status

# If not running, start services
airstack up isaac-sim robot

# Or with specific scene
ISAAC_SIM_SCRIPT_NAME="example_one_px4_pegasus_launch_script.py" airstack up isaac-sim robot
```

### 2. Verify Module is Launched

Check that your module is running in the autonomy stack:

```bash
# List running nodes
docker exec airstack-robot-1 bash -c "ros2 node list | grep your_module"

# Expected output:
# /drone1/namespace/your_node

# Get node info
docker exec airstack-robot-1 bash -c "ros2 node info /drone1/namespace/your_node"
```

### 3. Monitor Key Topics

Set up topic monitoring to verify data flow:

```bash
# Monitor input topics (verify module receives data)
docker exec airstack-robot-1 bash -c "ros2 topic hz /drone1/odometry"
docker exec airstack-robot-1 bash -c "ros2 topic hz /drone1/global_plan"

# Monitor output topics (verify module publishes)
docker exec airstack-robot-1 bash -c "ros2 topic hz /drone1/trajectory_controller/trajectory_segment_to_add"

# Monitor multiple topics simultaneously
docker exec airstack-robot-1 bash -c "ros2 topic hz /drone1/odometry /drone1/trajectory_controller/trajectory_segment_to_add"
```

### 4. Visualize in Ground Control Station

If GCS is running, use it for visualization:

```bash
# Start GCS if not already running
airstack up gcs

# Access GCS web interface
# http://localhost:8080 (or configured port)
```

In GCS:
- View drone telemetry
- Monitor autonomy status
- Send waypoints
- Visualize trajectories

### 5. Use RViz for Detailed Visualization

RViz provides detailed topic visualization:

```bash
# Launch RViz (from your host, if GCS has RViz)
# or from within robot container with display forwarding

# Key topics to visualize:
# - /drone1/odometry (PoseStamped or Odometry)
# - /drone1/global_plan (Path)
# - /drone1/local_planner/trajectory (Path or TrajectoryArray)
# - /drone1/local_planner/cost_map (CostMap or PointCloud)
# - /drone1/sensors/camera/image (Image)
# - TF tree (shows all coordinate frames)
```

**RViz configuration:**
1. Add → By topic → Select topics
2. Fixed Frame: `map` or `world`
3. Save configuration for reuse

### 6. Design Test Scenario

Plan specific tests for your module:

#### For Local Planners:
- **Obstacle avoidance:** Spawn obstacles, verify path goes around
- **Goal reaching:** Set waypoint, verify convergence
- **Dynamic obstacles:** Moving obstacles, verify reactive behavior
- **Edge cases:** Narrow passages, dead ends

#### For Global Planners:
- **Path generation:** Set distant goal, verify feasible path
- **Replanning:** Block path, verify replanning
- **Multi-waypoint:** Multiple waypoints, verify order

#### For Controllers:
- **Tracking accuracy:** Follow trajectory, measure error
- **Stability:** Aggressive maneuvers, verify stability
- **Disturbance rejection:** Apply external forces in sim

#### For Perception:
- **Accuracy:** Compare estimates with ground truth
- **Latency:** Measure processing time
- **Robustness:** Various lighting, textures

### 7. Execute Test Scenario

#### Method 1: Manual Commands via Behavior Tree GUI

```bash
# If behavior tree GUI is available
# Send commands through rqt_behavior_tree_command

# Typical test sequence:
# 1. Arm drone
# 2. Takeoff to altitude
# 3. Send waypoint/goal
# 4. Monitor execution
# 5. Land
```

#### Method 2: Publish Test Commands

```bash
# Publish test waypoint
docker exec airstack-robot-1 bash -c "ros2 topic pub --once /drone1/global_plan nav_msgs/Path '
header:
  frame_id: \"map\"
poses:
  - pose:
      position: {x: 5.0, y: 0.0, z: 2.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - pose:
      position: {x: 10.0, y: 5.0, z: 2.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
'"

# Trigger module service (if applicable)
docker exec airstack-robot-1 bash -c "ros2 service call /drone1/your_module/trigger std_srvs/Trigger"
```

#### Method 3: Scripted Test Sequence

Create a test script:

**File:** `scripts/test_your_module.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

class ModuleTester(Node):
    def __init__(self):
        super().__init__('module_tester')
        self.plan_pub = self.create_publisher(Path, '/drone1/global_plan', 10)
        time.sleep(1)  # Wait for connections
        
    def test_scenario_1(self):
        """Test basic waypoint following"""
        self.get_logger().info("Running test scenario 1...")
        
        # Create test path
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add waypoints
        for x, y, z in [(5, 0, 2), (10, 5, 2), (10, 10, 2)]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        # Publish path
        self.plan_pub.publish(path)
        self.get_logger().info("Published test path")

def main():
    rclpy.init()
    tester = ModuleTester()
    
    # Run tests
    tester.test_scenario_1()
    
    # Keep node alive to see results
    rclpy.spin(tester)
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run test:
```bash
docker exec airstack-robot-1 bash -c "sws && python3 scripts/test_your_module.py"
```

### 8. Record Test Data

Record ROS bag for later analysis:

```bash
# Record all topics
docker exec airstack-robot-1 bash -c "ros2 bag record -a -o /tmp/test_run_1"

# Or record specific topics
docker exec airstack-robot-1 bash -c "ros2 bag record \
    /drone1/odometry \
    /drone1/global_plan \
    /drone1/trajectory_controller/trajectory_segment_to_add \
    /drone1/your_module/debug \
    -o /tmp/test_run_1"

# Stop recording with Ctrl+C

# Copy bag out of container
docker cp airstack-robot-1:/tmp/test_run_1 ./test_data/
```

### 9. Analyze Test Results

#### Check Topic Data

```bash
# Play back bag
ros2 bag play test_data/test_run_1

# Info about bag
ros2 bag info test_data/test_run_1

# Extract specific topic to CSV
ros2 bag play test_data/test_run_1 --topics /drone1/odometry | tee odom_data.txt
```

#### Compute Metrics

Create analysis script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import numpy as np

class TestAnalyzer(Node):
    def __init__(self):
        super().__init__('test_analyzer')
        self.odom_positions = []
        self.planned_positions = []
        
        self.odom_sub = self.create_subscription(
            Odometry, '/drone1/odometry', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(
            Path, '/drone1/global_plan', self.plan_callback, 10)
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.odom_positions.append([pos.x, pos.y, pos.z])
    
    def plan_callback(self, msg):
        self.planned_positions = [[p.pose.position.x, 
                                   p.pose.position.y, 
                                   p.pose.position.z] for p in msg.poses]
    
    def compute_metrics(self):
        if not self.odom_positions or not self.planned_positions:
            self.get_logger().warn("Insufficient data")
            return
        
        # Tracking error
        # ... compute metrics
        
        self.get_logger().info(f"Mean tracking error: {error_mean:.3f} m")

# Run analysis on bag playback
```

#### Visual Analysis

Plot trajectories:

```python
import matplotlib.pyplot as plt

# Plot actual vs planned trajectory
plt.figure()
plt.plot(actual_x, actual_y, label='Actual')
plt.plot(planned_x, planned_y, label='Planned', linestyle='--')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid(True)
plt.savefig('trajectory_comparison.png')
```

### 10. Verify Expected Behavior

Create a checklist for your module:

**For Local Planner:**
- [ ] Receives odometry at expected rate (>10 Hz)
- [ ] Receives global plan when published
- [ ] Publishes trajectory at expected rate
- [ ] Avoids obstacles in environment
- [ ] Reaches goal position (within threshold)
- [ ] Smooth trajectory (no jerky movements)
- [ ] Reasonable computation time (<100 ms per cycle)

**For Global Planner:**
- [ ] Generates path from start to goal
- [ ] Path is collision-free (in known map)
- [ ] Path is smooth and efficient
- [ ] Replans when environment changes
- [ ] Computation time acceptable

**For Controller:**
- [ ] Tracks trajectory with acceptable error
- [ ] Stable (no oscillations)
- [ ] Responds to trajectory updates
- [ ] Publishes control commands at expected rate

### 11. Test Edge Cases

Don't just test the happy path:

```bash
# Test 1: No input (verify graceful handling)
# - Don't publish global plan
# - Verify module doesn't crash

# Test 2: Invalid input (verify error handling)
# - Publish malformed message
# - Verify module handles gracefully

# Test 3: High rate input (verify performance)
# - Publish inputs at high frequency
# - Monitor CPU usage

# Test 4: Sensor failures (verify robustness)
# - Stop sensor topics
# - Verify fallback behavior
```

### 12. Multi-Robot Testing (if applicable)

If module supports multi-robot:

```bash
# Launch multi-robot simulation
NUM_ROBOTS=2 airstack up isaac-sim robot

# Verify each robot runs independently
docker exec airstack-robot-1 bash -c "ros2 node list | grep drone"

# Expected:
# /drone0/...
# /drone1/...

# Test coordination or conflicts
```

### 13. Performance Profiling

Monitor system resources:

```bash
# Monitor CPU/memory usage
docker stats airstack-robot-1

# Find resource-intensive nodes
docker exec airstack-robot-1 bash -c "top -b -n 1 | grep ros"

# Profile specific node (if needed)
docker exec airstack-robot-1 bash -c "perf record -g ros2 run your_package your_node"
```

### 14. Document Test Results

Create test report:

**File:** `test_reports/your_module_test_YYYYMMDD.md`

```markdown
# Test Report: Your Module

## Test Date
YYYY-MM-DD

## Test Environment
- Isaac Sim version: X.Y.Z
- AirStack version: X.Y.Z
- Docker image: airstack:tag
- Scene: example_one_px4_pegasus_launch_script.py

## Test Scenarios

### Scenario 1: Basic Waypoint Following
**Description:** Single waypoint navigation

**Expected:** Drone reaches waypoint within 0.5m

**Result:** ✓ PASS
- Final error: 0.23m
- Time to goal: 15.3s

**Data:** test_data/scenario_1.bag

### Scenario 2: Obstacle Avoidance
**Description:** Navigate around static obstacle

**Expected:** Path avoids obstacle with >1m clearance

**Result:** ✗ FAIL
- Minimum clearance: 0.4m (below threshold)
- Issue: Cost map underestimated obstacle size

**Action:** Increase expansion radius in config

## Performance Metrics
- Average CPU: 15%
- Average latency: 45ms
- Publish rate: 10 Hz (as expected)

## Issues Found
1. Issue description
   - Reproduction steps
   - Proposed fix

## Conclusion
Summary of test results and next steps.
```

## Automated Testing

For CI/CD, create automated test scripts:

```bash
#!/bin/bash
# scripts/run_simulation_tests.sh

set -e

# Start simulation
docker-compose up -d isaac-sim robot

# Wait for initialization
sleep 30

# Run test scenarios
docker exec airstack-robot-1 bash -c "sws && python3 scripts/test_scenario_1.py"
docker exec airstack-robot-1 bash -c "sws && python3 scripts/test_scenario_2.py"

# Check results
if [ -f "/tmp/test_results_pass" ]; then
    echo "Tests PASSED"
    exit 0
else
    echo "Tests FAILED"
    exit 1
fi
```

## Common Issues

### Simulation Issues
- ❌ **Isaac Sim not responding**
  - ✅ Check GPU availability, restart container
- ❌ **PX4 not connecting**
  - ✅ Check MAVLink ports, verify PX4 processes
- ❌ **Drone not moving**
  - ✅ Check arm/offboard commands, verify mode

### Integration Issues
- ❌ **Module not receiving data in sim**
  - ✅ Check topic remapping, verify sim publishes topics
- ❌ **Commands not reaching PX4**
  - ✅ Verify control chain: planner → controller → mavros → PX4

### Performance Issues
- ❌ **Slow simulation**
  - ✅ Reduce sensor resolution, decrease update rates
- ❌ **High latency**
  - ✅ Profile module, optimize algorithm

## Tips for Effective Testing

1. **Start simple:** Test basic functionality before complex scenarios
2. **One change at a time:** Isolate what you're testing
3. **Use visualization:** RViz/GCS help identify issues quickly
4. **Record everything:** Bags are invaluable for debugging
5. **Automate:** Write scripts for repeatable tests
6. **Document:** Record what you tested and results

## References

- **ROS 2 Testing:**
  - [ROS 2 Testing Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html)
  - [ROS 2 Bag](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

- **Isaac Sim:**
  - [Isaac Sim Testing](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)

- **AirStack:**
  - [System Architecture](../docs/robot/autonomy/system_architecture.md)
  - [Integration Checklist](../docs/robot/autonomy/integration_checklist.md)

- **Related Skills:**
  - [debug-module](../debug-module) - Debugging issues found in testing
  - [write-isaac-sim-scene](../write-isaac-sim-scene) - Creating custom test scenarios
