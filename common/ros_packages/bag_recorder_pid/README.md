# bag_record_pid

A flexible ROS2 node for recording bag files with runtime control, multi-section recording, and namespace support.

## Overview

`bag_record_pid` provides an advanced interface for recording ROS2 topics to MCAP bag files. The node can be dynamically started and stopped via ROS topics, supports multiple concurrent recording sections with different configurations, and automatically handles namespaced topics.

### Key Features

- **Runtime Control**: Start/stop recording via ROS topic messages
- **Multi-Section Recording**: Record different topic groups with independent configurations
- **Namespace Support**: Automatic prefixing of topics with node namespace
- **Flexible Topic Selection**: Include specific topics or exclude patterns from all topics
- **MCAP Format**: Records to MCAP format with configurable QoS and compression
- **Status Publishing**: Publishes recording status for monitoring
- **Configurable Storage**: Set bag size limits, cache sizes, and compression options

## Installation

This package is part of the Airstack framework. Ensure you have ROS2 installed and your workspace is properly configured.

```bash
cd <workspace>/src
colcon build --packages-select bag_record_pid
source install/setup.bash
```

## Usage

### Basic Launch

Launch with default configuration:

```bash
ros2 launch bag_record_pid bag_record_pid.launch.py
```

### With Custom Configuration

Specify custom config file and output directory:

```bash
ros2 launch bag_record_pid bag_record_pid.launch.py \
    cfg_path:=/path/to/config.yaml \
    output_dir:=/path/to/logging \
    mcap_qos_dir:=/path/to/qos/configs
```

### Namespaced Launch

Launch with a specific namespace (e.g., for multi-robot systems):

```bash
ros2 launch bag_record_pid bag_record_pid_namespaced.launch.py \
    robot_prefix:=robot1 \
    cfg_path:=/path/to/config.yaml \
    output_dir:=/logging
```

### Controlling Recording

Start recording:

```bash
ros2 topic pub --once /bag_record_pid/set_recording_status std_msgs/msg/Bool "{data: true}"
```

Stop recording:

```bash
ros2 topic pub --once /bag_record_pid/set_recording_status std_msgs/msg/Bool "{data: false}"
```

Check recording status:

```bash
ros2 topic echo /bag_record_pid/bag_recording_status
```

## Configuration

### Configuration File Structure

The configuration file (`cfg.yaml`) defines recording sections with their respective topics and settings:

```yaml
sections:
  section_name:
    mcap_qos: mcap_qos.yaml  # Optional: MCAP QoS profile filename
    args:
      - -b
      - 4000000000            # Bag size limit (~4GB)
      - --max-cache-size
      - 1073741824            # Cache size (1GB)
    topics:
      - /tf
      - /tf_static
      - sensor/data
      - robot/state
```

### Topic Specification Options

#### Option 1: Explicit Topics List

Specify exact topics to record:

```yaml
sections:
  my_section:
    mcap_qos: ""
    args: []
    topics:
      - /tf                    # Absolute topic path
      - /tf_static
      - camera/image           # Relative topic (namespace will be prefixed)
```

#### Option 2: Exclude Pattern

Record all topics except specified ones:

```yaml
sections:
  my_section:
    mcap_qos: ""
    args: []
    exclude:
      - /tf
      - /tf_static
```

**Note**: Cannot mix `topics` and `exclude` in the same section.

### MCAP QoS Configuration

Configure MCAP storage settings (`mcap_qos.yaml`):

```yaml
noChunkCRC: false
noChunking: false
noMessageIndex: false
noSummary: false
chunkSize: 10485760          # 10MB chunks
compression: "Zstd"
compressionLevel: "Fastest"
forceCompression: false
```

### Common Arguments

Arguments passed to `ros2 bag record`:

- `-b <bytes>`: Maximum bag file size in bytes
- `--max-cache-size <bytes>`: Maximum cache size
- `--storage-config-file <path>`: Path to MCAP QoS config (automatically added)
- Additional `ros2 bag record` arguments can be added

## Example Configurations

### Example 1: Recording Specific Topics

```yaml
sections:
  sensors:
    mcap_qos: mcap_qos.yaml
    args:
      - -b
      - 2000000000           # 2GB limit
      - --max-cache-size
      - 524288000            # 500MB cache
    topics:
      - /tf
      - /tf_static
      - gps/position
      - lidar/points
      - camera/image
```

### Example 2: Multi-Section Recording

Record different topic groups separately (useful for managing large data volumes):

```yaml
sections:
  # Low-frequency but critical data
  state_and_gps:
    mcap_qos: mcap_qos.yaml
    args:
      - -b
      - 1000000000
    topics:
      - /tf
      - /tf_static
      - gps/position
      - robot/state
  
  # High-bandwidth camera data
  cameras:
    mcap_qos: mcap_qos.yaml
    args:
      - -b
      - 4000000000
    topics:
      - camera/front/image
      - camera/back/image
      - camera/left/image
      - camera/right/image
```

### Example 3: Exclude Pattern

Record everything except specific topics:

```yaml
sections:
  all_except_diagnostics:
    mcap_qos: mcap_qos.yaml
    args:
      - -b
      - 5000000000
    exclude:
      - /diagnostics
      - /debug/verbose_output
      - /tf              # Often excluded to reduce bag size
      - /tf_static
```

## Launch File Parameters

### bag_record_pid.launch.py

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cfg_path` | string | `<pkg_dir>/config/cfg.yaml` | Path to configuration file |
| `output_dir` | string | `/logging` | Directory for bag files |
| `mcap_qos_dir` | string | `<pkg_dir>/config` | Directory containing MCAP QoS configs |

### bag_record_pid_namespaced.launch.py

All parameters from `bag_record_pid.launch.py` plus:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_prefix` | string | `robot1` | Namespace prefix for the node |

## Topics

### Subscribed

- `<namespace>/bag_record_pid/set_recording_status` (`std_msgs/Bool`): Control recording on/off

### Published

- `<namespace>/bag_record_pid/bag_recording_status` (`std_msgs/Bool`): Current recording status (published at 2Hz)

## Output Files

Bag files are saved to the specified `output_dir` with the naming convention:

```
<section_name>_<timestamp>/
```

Where:
- `section_name`: Name from configuration file
- `timestamp`: Format `YYYYMMDD_HHMMSS`

Example:
```
/logging/sensors_20260210_143052/
```

## Namespace Behavior

When topics are specified in the configuration file:

- **Absolute topics** (starting with `/`): Used as-is
- **Relative topics** (no leading `/`): Automatically prefixed with the node's namespace

For example, with namespace `/robot1`:
- `/tf` → `/tf` (unchanged)
- `camera/image` → `/robot1/camera/image` (prefixed)

## Advanced Usage

### Programmatic Control (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BagController(Node):
    def __init__(self):
        super().__init__('bag_controller')
        self.pub = self.create_publisher(
            Bool, 
            '/bag_record_pid/set_recording_status', 
            10
        )
        self.status_sub = self.create_subscription(
            Bool,
            '/bag_record_pid/bag_recording_status',
            self.status_callback,
            10
        )
    
    def start_recording(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
    
    def stop_recording(self):
        msg = Bool()
        msg.data = False
        self.pub.publish(msg)
    
    def status_callback(self, msg):
        if msg.data:
            self.get_logger().info('Recording is active')
        else:
            self.get_logger().info('Recording is stopped')
```

### Programmatic Control (C++)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class BagController : public rclcpp::Node {
public:
    BagController() : Node("bag_controller") {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/bag_record_pid/set_recording_status", 10);
        
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/bag_record_pid/bag_recording_status", 10,
            std::bind(&BagController::status_callback, this, std::placeholders::_1));
    }
    
    void start_recording() {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        publisher_->publish(msg);
    }
    
    void stop_recording() {
        auto msg = std_msgs::msg::Bool();
        msg.data = false;
        publisher_->publish(msg);
    }

private:
    void status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Recording is active");
        } else {
            RCLCPP_INFO(this->get_logger(), "Recording is stopped");
        }
    }
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};
```

## Troubleshooting

### Output Directory Does Not Exist

**Error**: "Output directory /logging does not exist"

**Solution**: Ensure the output directory exists before launching:
```bash
mkdir -p /logging
```

### Topics Not Being Recorded

**Issue**: Specified topics aren't appearing in bag files

**Checklist**:
1. Verify topics exist: `ros2 topic list`
2. Check topic names in config (relative vs absolute paths)
3. Verify namespace settings if using namespaced launch
4. Ensure recording is started: `ros2 topic echo /bag_record_pid/bag_recording_status`

### Bag Files Too Large

**Solution**: Adjust the bag size limit in configuration:
```yaml
args:
  - -b
  - 1000000000  # Reduce to 1GB
```

Or split into multiple sections with different topics.

### Cannot Mix Topics and Exclude

**Error**: "Cannot mix exclude with topics"

**Solution**: Choose either `topics` or `exclude` for each section, not both.

## Dependencies

- ROS2 (Humble or later recommended)
- Python 3
- `rclpy`
- `std_msgs`
- `ros2 bag` CLI tools
- MCAP storage plugin

## Node Details

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cfg_path` | string | `<pkg_root>/config/cfg.yaml` | Configuration file path |
| `output_dir` | string | `/logging/` | Output directory for bag files |
| `mcap_qos_dir` | string | `""` | Directory containing MCAP QoS configs |

### Timer Callbacks

- **Status Publisher**: 2Hz (0.5s interval) - publishes current recording status

### Signal Handling

The node sends `SIGINT` to bag recording processes on shutdown or when stopping recording, ensuring graceful termination and proper bag file finalization.

## Best Practices

1. **Storage Management**: Monitor disk space when recording large amounts of data
2. **Section Organization**: Group topics by data type or frequency for easier management
3. **MCAP Compression**: Use compression to reduce disk usage (Zstd recommended)
4. **Namespace Consistency**: Use namespaced launch for multi-robot deployments
5. **Testing**: Verify configuration with a short test recording before long-duration runs
6. **Timestamps**: Use the automatic timestamp naming to avoid overwriting previous recordings

## Contributing

Contributions are welcome! Please follow the Airstack development guidelines.

## License

TODO: License declaration

## Maintainer

**Airlab**  
Email: mmayank74567@gmail.com

## See Also

- [ROS2 Bag Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [MCAP Format](https://mcap.dev/)
- Airstack Documentation: [docs/](../../docs/)
