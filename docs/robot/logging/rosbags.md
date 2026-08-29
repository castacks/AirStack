# ROS Bag Recording

ROS bags are the primary method for recording data during robot operation. They capture ROS 2 topics for later analysis, debugging, and algorithm development.

## Managed Recording

AirStack manages recording with the [bag_recorder_pid](../../../common/ros_packages/logging/bag_recorder_pid/README.md) package: launch the stack with `RECORD_BAGS=true` to start the recorder node, then toggle recording via its `bag_record/set_recording_status` topic (`std_msgs/Bool`, in the robot namespace). See the [Logging overview](index.md) for the full workflow.

## Configuration

### Topic Selection

Choose topics based on mission objectives. The recorder's config file (selected with `LOG_CONFIG`, in `logging_bringup/config/`, default `log.yaml`) groups topics into named **sections**; relative topic names are prefixed with the robot namespace:

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

Section `args` are passed through to `ros2 bag record`, so storage limits like
`-b 4000000000` (split at ~4 GB) or `--max-cache-size` go there — see the
shipped `log.yaml` for a working example.

## Storage Locations

### Development (Docker)
- Bags stored in mounted volume: `robot/bags/` (mounted at `/bags` in the container)
- Persists across container restarts

### Hardware Deployment
- Jetson (`l4t` profile): `${BAG_STORAGE_PATH}` on the device (default `/media/airlab/Storage/airstack_collection`) is mounted at `/bags`
- The recorder's target directory is its `output_dir` parameter (set in `logging.launch.xml`, default `/bags`)

## Manual Recording and Playback

For one-off captures the standard CLI works as usual inside the robot container — see the [ROS 2 bag documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) for the full reference:

```bash
ros2 bag record /robot_1/odometry_conversion/odometry /robot_1/global_plan
ros2 bag play path/to/bagfile
ros2 bag info path/to/bagfile
```

Topics are namespaced by robot name (`/robot_1/...` with the default robot
name map).

To extract specific topics into a new bag, use `ros2 bag convert` with an output spec:
```bash
ros2 bag convert -i input_bag -o out_spec.yaml
```

```yaml
# out_spec.yaml
output_bags:
  - uri: output_bag
    topics: [/robot_1/odometry_conversion/odometry, /robot_1/global_plan]
```

## Integration with Data Offloading

For transferring bags from robot to a storage server, see the [Data Offloading Guide](data_offloading.md).

## See Also

- [bag_recorder_pid Package](../../../common/ros_packages/logging/bag_recorder_pid/README.md) - Automatic recording package
- [Data Offloading](data_offloading.md) - Transfer bags from robot
- [Logging Overview](index.md) - AirStack logging infrastructure
- [ROS 2 Bag Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
