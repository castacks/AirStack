# Module Integration Checklist

This document provides a comprehensive checklist and guidelines for integrating new modules into the AirStack autonomy stack.

## Overview

When adding a new module to AirStack, proper integration ensures:

- **Interoperability:** Module works correctly with other modules
- **Discoverability:** Module is documented and easy to find
- **Maintainability:** Module follows conventions and is easy to update
- **Testability:** Module can be tested in isolation and in full stack

## Standard Topic Patterns

AirStack uses standardized topic naming conventions to ensure consistent communication between modules.

### Topic Naming Convention

Topics follow this pattern:
```
/[robot_name]/[layer]/[module]/[data_type]
```

Examples:
- `/drone1/perception/macvo/odometry`
- `/drone1/local_planner/droan/trajectory`
- `/drone1/trajectory_controller/tracking_point`

### Common Standard Topics

These topics are used across multiple modules and should be used when applicable:

| Topic | Type | Purpose | Layer |
|-------|------|---------|-------|
| `/[robot]/odometry` | nav_msgs/Odometry | Primary state estimate | Perception → All |
| `/[robot]/global_plan` | nav_msgs/Path | Global waypoint path | Global → Local |
| `/[robot]/trajectory_controller/trajectory_segment_to_add` | airstack_msgs/TrajectorySegment | Local trajectory commands | Local Planner → Controller |
| `/[robot]/trajectory_controller/trajectory_override` | airstack_msgs/TrajectoryOverride | Direct trajectory override | Behavior → Controller |
| `/[robot]/trajectory_controller/look_ahead` | geometry_msgs/PointStamped | Look-ahead point for planning | Controller → Local Planner |
| `/[robot]/trajectory_controller/tracking_point` | geometry_msgs/PointStamped | Current tracking point | Controller → All |
| `/[robot]/trajectory_controller/trajectory_completion_percentage` | std_msgs/Float32 | Trajectory progress | Controller → Planners |
| `/[robot]/interface/mavros/cmd/takeoff` | mavros_msgs/CommandTOL | Takeoff command | Behavior → Interface |
| `/[robot]/interface/cmd_vel` | geometry_msgs/Twist | Low-level velocity commands | Controller → Interface |

### Layer-Specific Topic Patterns

#### Interface Layer
- **Inputs:** Commands from control layer
- **Outputs:** Robot state, sensor raw data
- **Topics:**
  - `/[robot]/interface/mavros/state`
  - `/[robot]/interface/mavros/local_position/pose`
  - `/[robot]/interface/battery_state`

#### Sensors Layer
- **Inputs:** Raw sensor data from interface
- **Outputs:** Processed sensor data
- **Topics:**
  - `/[robot]/sensors/[sensor_name]/[data_type]`
  - Example: `/[robot]/sensors/front_stereo/left/image`
  - Example: `/[robot]/sensors/front_stereo/disparity`

#### Perception Layer
- **Inputs:** Sensor data
- **Outputs:** Odometry, environment understanding
- **Topics:**
  - `/[robot]/perception/[module]/odometry`
  - `/[robot]/perception/[module]/depth`
  - `/[robot]/odometry` (aggregated/primary odometry)

#### Local Layer
- **Inputs:** Odometry, local sensor data, global plan
- **Outputs:** Local trajectories, cost maps
- **Topics:**
  - World Models: `/[robot]/local/[module]/cost_map`
  - Planners: `/[robot]/local/[module]/trajectory`
  - Controllers: `/[robot]/trajectory_controller/cmd`

#### Global Layer
- **Inputs:** Global map, robot pose, goal
- **Outputs:** Global plan, map updates
- **Topics:**
  - Mapping: `/[robot]/global/[module]/map`
  - Planning: `/[robot]/global_plan`

#### Behavior Layer
- **Inputs:** Mission commands, autonomy state
- **Outputs:** High-level commands, mode changes
- **Topics:**
  - `/[robot]/behavior/mission_state`
  - `/[robot]/behavior/bt_status`

## Integration Checklist

Use this checklist when integrating a new module:

### 1. Package Structure

- [ ] Package created in correct location based on layer
- [ ] `package.xml` complete with all dependencies
- [ ] `CMakeLists.txt` or `setup.py` properly configured
- [ ] Launch files installed correctly
- [ ] Config files installed correctly
- [ ] README.md present with comprehensive documentation

### 2. Topic Interfaces

- [ ] Input topics use generic names (remapped in launch)
- [ ] Output topics use generic names (remapped in launch)
- [ ] Topic types match expected interfaces
- [ ] Topics documented in README.md
- [ ] Topic remapping tested and verified

### 3. Parameters

- [ ] All parameters declared in code
- [ ] Default configuration file created (`config/`)
- [ ] Parameters documented in README.md with types and defaults
- [ ] Parameter validation implemented (range checks, etc.)
- [ ] Parameters loaded correctly with `allow_substs="true"`

### 4. Launch Integration

- [ ] Module launch file created with topic remapping arguments
- [ ] Module added to appropriate layer bringup package
- [ ] Launch arguments use `$(env ROBOT_NAME)` for multi-robot support
- [ ] Module namespace properly configured
- [ ] Module included in `autonomy_bringup` launch flow

### 5. Dependencies

- [ ] All dependencies listed in `package.xml`
- [ ] Bringup package depends on your package
- [ ] External dependencies documented in README
- [ ] Dependencies available in Docker image (or documented for addition)

### 6. Code Quality

- [ ] Code follows ROS 2 best practices
- [ ] Error handling implemented
- [ ] Logging uses appropriate levels (DEBUG, INFO, WARN, ERROR)
- [ ] No hardcoded values (use parameters instead)
- [ ] Proper resource cleanup in destructors/shutdown

### 7. Testing

- [ ] Unit tests created (if applicable)
- [ ] Integration tests planned/created
- [ ] Module tested standalone
- [ ] Module tested in full autonomy stack
- [ ] Module tested in Isaac Sim simulation
- [ ] Edge cases tested

### 8. Documentation

- [ ] Module README.md complete (use template)
- [ ] README added to `mkdocs.yml` navigation
- [ ] Layer overview page updated (`docs/robot/autonomy/<layer>/index.md`)
- [ ] Algorithm/approach documented
- [ ] Architecture diagram included (mermaid)
- [ ] Usage examples provided
- [ ] Known issues/limitations documented

### 9. Visualization (if applicable)

- [ ] Debug topics published for visualization
- [ ] RViz markers published for 3D visualization
- [ ] RViz config file provided
- [ ] Visualization documented

### 10. Multi-Robot Support

- [ ] Topics include `$(env ROBOT_NAME)` namespace
- [ ] No hardcoded robot names
- [ ] Node names properly namespaced
- [ ] Tested with multiple robot instances

## Integration Testing Checklist

After integration, verify the module works correctly:

### Node Status

```bash
# Verify node is running
docker exec airstack-robot-1 bash -c "ros2 node list | grep your_node"

# Check node info
docker exec airstack-robot-1 bash -c "ros2 node info /robot/namespace/your_node"
```

**Expected:**
- [ ] Node appears in node list
- [ ] All expected subscriptions present
- [ ] All expected publications present

### Topic Connections

```bash
# Check topic publishers/subscribers
docker exec airstack-robot-1 bash -c "ros2 topic info /robot/your/topic"

# Check topic publishing rate
docker exec airstack-robot-1 bash -c "ros2 topic hz /robot/your/output/topic"
```

**Expected:**
- [ ] Input topics have publishers
- [ ] Output topics have subscribers (if other modules depend on them)
- [ ] Publishing rates match expectations

### Data Flow

```bash
# Echo input topic
docker exec airstack-robot-1 bash -c "ros2 topic echo /robot/input/topic --once"

# Echo output topic
docker exec airstack-robot-1 bash -c "ros2 topic echo /robot/output/topic --once"
```

**Expected:**
- [ ] Input data has reasonable values
- [ ] Output data has reasonable values
- [ ] Data types are correct

### Parameters

```bash
# List parameters
docker exec airstack-robot-1 bash -c "ros2 param list /robot/namespace/your_node"

# Check parameter values
docker exec airstack-robot-1 bash -c "ros2 param get /robot/namespace/your_node param_name"
```

**Expected:**
- [ ] All parameters loaded correctly
- [ ] Parameter values match config file

### Logs

```bash
# Check logs for errors
docker logs airstack-robot-1 2>&1 | grep -i "error\|fail"

# Check module-specific logs
docker logs airstack-robot-1 2>&1 | grep -i "your_module"
```

**Expected:**
- [ ] No errors or warnings (unless expected)
- [ ] Initialization messages present
- [ ] Module is processing data

### Performance

```bash
# Monitor resource usage
docker stats airstack-robot-1
```

**Expected:**
- [ ] CPU usage reasonable (<X%, depends on module)
- [ ] Memory usage reasonable (<Y MB, depends on module)
- [ ] No memory leaks over time

## Common Integration Issues

### Issue: Node Not Starting

**Symptoms:**
- Node not in `ros2 node list`
- Container logs show errors on startup

**Possible Causes:**
- Missing dependencies
- Build errors
- Import errors (Python)
- Segfault on initialization

**Debug Steps:**
1. Check docker logs for errors
2. Try running node standalone
3. Verify all dependencies are installed
4. Check for typos in launch file

### Issue: Topics Not Connected

**Symptoms:**
- `ros2 topic info` shows no publishers or subscribers
- Data not flowing

**Possible Causes:**
- Incorrect topic remapping
- Wrong topic names in launch file
- Namespace issues

**Debug Steps:**
1. List all topics: `ros2 topic list`
2. Check node info for actual topic names
3. Verify remapping in launch file
4. Check for namespace conflicts

### Issue: Parameters Not Loading

**Symptoms:**
- Parameters have default values instead of config values
- Warnings about undeclared parameters

**Possible Causes:**
- Config file path incorrect
- Missing `allow_substs="true"`
- Parameter names mismatch
- YAML syntax errors

**Debug Steps:**
1. Verify config file path in launch file
2. Check parameter declaration in code
3. Validate YAML syntax
4. Test parameter loading with `ros2 param get`

### Issue: Poor Performance

**Symptoms:**
- High CPU usage
- High latency
- Slow update rates

**Possible Causes:**
- Inefficient algorithm
- Too high update rate
- Heavy computation in callbacks
- Not using multithreading when needed

**Debug Steps:**
1. Profile the code
2. Check update rate configuration
3. Move heavy computation to separate threads
4. Optimize algorithm

## Best Practices

### Topic Remapping

✅ **DO:**
- Use generic topic names in code (`odometry`, `output`, etc.)
- Remap in launch files
- Use launch arguments for flexibility

❌ **DON'T:**
- Hardcode full topic paths in code
- Assume specific namespaces

### Parameters

✅ **DO:**
- Declare all parameters with defaults
- Validate parameter values
- Document parameters in README

❌ **DON'T:**
- Use magic numbers in code
- Skip parameter validation
- Use undocumented parameters

### Error Handling

✅ **DO:**
- Check for null pointers/None values
- Handle exceptions gracefully
- Log errors with context

❌ **DON'T:**
- Crash on invalid input
- Silently fail
- Log errors without context

### Documentation

✅ **DO:**
- Use the README template
- Include diagrams
- Document edge cases and limitations

❌ **DON'T:**
- Skip documentation
- Assume behavior is obvious
- Leave TODOs unfilled

## References

- [System Architecture](system_architecture.md) - Overall system design
- [Add ROS 2 Package Skill](../../../.agents/skills/add_ros2_package.md) - Creating packages
- [Integration Skill](../../../.agents/skills/integrate_module_into_layer.md) - Integration workflow
- [Testing Skill](../../../.agents/skills/test_in_simulation.md) - Testing procedures
- [Package Template](../../../../../.agents/skills/add-ros2-package/assets/package_template/) - Template to follow
