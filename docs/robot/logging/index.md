# Logging

## Bag Recording

AirStack provides automated bag recording capabilities for capturing ROS 2 topic data during flights. The main node for bag recording is located at [common/ros_packages/logging/bag_recorder_pid](../../../common/ros_packages/logging/bag_recorder_pid/README.md). For detailed configuration options and implementation details, please consult the README in that directory.

### Enabling Bag Recording

To start the recorder node, prepend `RECORD_BAGS=true` to the airstack up command:

```bash
RECORD_BAGS=true airstack up robot-desktop
```

The stack entry file includes `logging_bringup/launch/logging.launch.xml`, which starts the `bag_record` node only when `RECORD_BAGS=true`. The topic set to record is selected with `LOG_CONFIG` (a filename in `logging_bringup/config/`, default `log.yaml`).

The recorder starts **idle**. Toggle recording at runtime by publishing to its control topic (bridged to the GCS by the DDS router):

```bash
# Start recording
ros2 topic pub --once /$ROBOT_NAME/bag_record/set_recording_status std_msgs/msg/Bool "{data: true}"

# Stop recording
ros2 topic pub --once /$ROBOT_NAME/bag_record/set_recording_status std_msgs/msg/Bool "{data: false}"

# Watch recording status (published at 2 Hz)
ros2 topic echo /$ROBOT_NAME/bag_record/bag_recording_status
```

Recorded bags (MCAP format) are written to `/bags` inside the container, which is the mounted `./robot/bags` directory on the host.
