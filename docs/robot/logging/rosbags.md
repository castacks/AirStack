# ROS Bag Recording

ROS bags are the primary method for recording data during robot operation. They capture ROS 2 topics for later analysis, debugging, and algorithm development.

## Overview

ROS bag recording in AirStack:

- **Automatic recording** via bag_recorder_pid package
- **Selective topic recording** to manage storage
- **Integration with logging infrastructure**
- **Support for onboard and offboard recording**

## Quick Start

### Manual Recording

Record specific topics:
```bash
ros2 bag record /robot1/odometry /robot1/camera/image_raw
```

Record all topics:
```bash
ros2 bag record -a
```

Record with storage limit:
```bash
ros2 bag record -a --max-bag-size 1000000000  # ~1 GB per file (bytes)
```

### Managed Recording

AirStack manages recording with the [bag_recorder_pid](../../../common/ros_packages/logging/bag_recorder_pid/README.md) package: launch the stack with `RECORD_BAGS=true` to start the recorder node, then toggle recording via its `bag_record/set_recording_status` topic. See the [Logging overview](index.md) for the full workflow.

## Configuration

### Topic Selection

Choose topics based on mission objectives. The recorder's config file (selected with `LOG_CONFIG`, in `logging_bringup/config/`) groups topics into named **sections**; relative topic names are prefixed with the robot namespace:

**Minimal set** (state and commands):
```yaml
sections:
  state:
    mcap_qos: mcap_qos.yaml
    args: []
    topics:
      - odometry_conversion/odometry
      - global_plan
      - trajectory_controller/trajectory_segment_to_add
```

**Full set** (everything for debugging — record all topics except an exclude list):
```yaml
sections:
  everything:
    mcap_qos: mcap_qos.yaml
    args: []
    exclude:
      - /tf
      - /tf_static
```

### Storage Management

On resource-constrained platforms (Jetson, VOXL):

- **Limit bag size**: Use `--max-bag-size` to split large bags
- **Selective recording**: Only record topics needed for mission
- **Compression**: Use compressed image topics when available
- **Automatic offload**: Configure [data offloading](data_offloading.md) to free space

## Storage Locations

### Development (Docker)
- Bags stored in mounted volume: `robot/bags/` (mounted at `/bags` in the container)
- Persists across container restarts

### Hardware Deployment
- Jetson (`l4t` profile): `/media/airlab/Storage/airstack_collection` on the device is mounted at `/bags`
- The recorder's target directory is its `output_dir` parameter (set in `logging.launch.xml`, default `/bags`)

## Playback and Analysis

### Basic Playback

Play back a recorded bag:
```bash
ros2 bag play path/to/bagfile
```

Play at different speed:
```bash
ros2 bag play path/to/bagfile --rate 0.5  # Half speed
```

Play in loop:
```bash
ros2 bag play path/to/bagfile --loop
```

### Bag Information

Get bag metadata:
```bash
ros2 bag info path/to/bagfile
```

Example output:
```
Files:             state_20260317_143000/state_20260317_143000_0.mcap
Bag size:          1.2 GB
Storage id:        mcap
Duration:          300.5s
Start:             Mar 17 2024 14:30:00.123
End:               Mar 17 2024 14:35:00.623
Messages:          45123
Topic information:
  Topic: /robot1/odometry | Type: nav_msgs/msg/Odometry | Count: 3005 | Serialization Format: cdr
  Topic: /robot1/camera/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 1500 | Serialization Format: cdr
  ...
```

### Extract Specific Topics

Convert to a new bag with only specific topics using `ros2 bag convert` with an output spec:
```bash
ros2 bag convert -i input_bag -o out_spec.yaml
```

```yaml
# out_spec.yaml
output_bags:
  - uri: output_bag
    topics: [/robot_1/odometry_conversion/odometry, /robot_1/global_plan]
```

## Common Workflows

### Debug Mission Issues

1. Record full topic set during mission
2. Play back locally in simulation
3. Analyze behavior with rviz or custom tools
4. Iterate on algorithms offline

### Algorithm Development

1. Record sensor data in real environment
2. Play back during development
3. Test new algorithms against real data
4. Validate before hardware deployment

### Performance Analysis

1. Record timestamped topics
2. Analyze latencies and frequencies
3. Identify bottlenecks
4. Optimize performance

## Best Practices

- **Test recording setup** before important missions
- **Monitor disk space** during operation
- **Use compression** for image topics
- **Document bag contents** with descriptive names
- **Archive important bags** with mission metadata
- **Regular cleanup** of old/unnecessary bags

## Troubleshooting

**Bag recording fails to start**:

- Check disk space availability
- Verify write permissions to bag directory
- Check if bag_recorder_pid is running

**Bags too large**:

- Use topic filtering to record only necessary data
- Enable compression for image topics
- Use `--max-bag-size` to split files
- Consider reducing sensor publishing rates

**Playback issues**:

- Ensure ROS 2 version matches recording system
- Check topic names and types match expectations
- Verify clock synchronization settings

**Missing data in bags**:

- Verify topics were being published during recording
- Check bag info to confirm topics recorded
- Ensure recording started before mission began

## Integration with Data Offloading

For automatic transfer of bags from robot to ground station or storage server:

See: [Data Offloading Guide](data_offloading.md)

## See Also

- [bag_recorder_pid Package](../../../common/ros_packages/logging/bag_recorder_pid/README.md) - Automatic recording package
- [Data Offloading](data_offloading.md) - Transfer bags from robot
- [Logging Overview](index.md) - AirStack logging infrastructure
- [ROS 2 Bag Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)