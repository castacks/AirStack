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
ros2 bag record -a --max-bag-size 1000  # 1GB per file
```

### Automatic Recording

AirStack can automatically record bags using the [bag_recorder_pid](../../../common/ros_packages/bag_recorder_pid/README.md) package.

Configure in the robot launch files to automatically start recording when the autonomy stack launches.

## Configuration

### Topic Selection

Choose topics based on mission objectives:

**Minimal set** (state and commands):
```yaml
topics:
  - /{robot_name}/odometry
  - /{robot_name}/global_plan
  - /{robot_name}/trajectory_controller/trajectory_segment_to_add
```

**Standard set** (add sensor data):
```yaml
topics:
  - /{robot_name}/odometry
  - /{robot_name}/global_plan
  - /{robot_name}/camera/image_raw/compressed
  - /{robot_name}/depth/image_raw
  - /{robot_name}/imu/data
```

**Full set** (everything for debugging):
```yaml
topics:
  - ".*"  # Record all topics
```

### Storage Management

On resource-constrained platforms (Jetson, VOXL):

- **Limit bag size**: Use `--max-bag-size` to split large bags
- **Selective recording**: Only record topics needed for mission
- **Compression**: Use compressed image topics when available
- **Automatic offload**: Configure [data offloading](data_offloading.md) to free space

## Storage Locations

### Development (Docker)
- Bags stored in mounted volume: `robot/bags/`
- Persists across container restarts

### Hardware Deployment
- Default location: `/opt/airstack/bags/` or local SSD
- Configure via environment variable: `ROSBAG_DIR`

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
Files:             rosbag2_2024_03_17-14_30_00.db3
Bag size:          1.2 GB
Storage id:        sqlite3
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

Convert to a new bag with only specific topics:
```bash
ros2 bag filter input_bag -o output_bag --topics /robot1/odometry /robot1/camera/image_raw
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

- [bag_recorder_pid Package](../../../common/ros_packages/bag_recorder_pid/README.md) - Automatic recording package
- [Data Offloading](data_offloading.md) - Transfer bags from robot
- [Logging Overview](index.md) - AirStack logging infrastructure
- [ROS 2 Bag Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)