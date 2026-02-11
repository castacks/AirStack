# Logging

## Bag Recording

AirStack provides automated bag recording capabilities for capturing ROS2 topic data during flights. The main node for bag recording is located at [common/ros_packages/bag_recorder_pid](../../common/ros_packages/bag_recorder_pid). For detailed configuration options and implementation details, please consult the README in that directory.

### Enabling Bag Recording

To enable bag recording, prepend `RECORD_BAGS=true` to the airstack up command:

```bash
RECORD_BAGS=true airstack up robot
```

The BehaviorTree will automatically trigger topic recording once the drone takes off. Recorded bags will appear in the `./robot/bags` directory. 