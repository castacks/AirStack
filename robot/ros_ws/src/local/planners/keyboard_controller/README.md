# Keyboard Controller

Teleop local planner that moves the drone incrementally in response to keyboard input. Supports two input modes simultaneously:

- **Terminal mode** — raw terminal keystrokes read directly in a background thread
- **GUI/Foxglove mode** — topic-based input from the Foxglove `keyboard-control` extension panel

## Key Bindings

| Key | Action |
|-----|--------|
| W / S | Forward / Backward (+X / -X in body frame) |
| A / D | Left / Right (+Y / -Y in body frame) |
| C / Z | Up / Down (+Z / -Z) |
| Q / E | Yaw left / Yaw right |
| O / P | Decrease / Increase XYZ step |
| K / L | Decrease / Increase yaw step |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `odometry` | `nav_msgs/Odometry` | In | Current drone pose |
| `trajectory_override` | `airstack_msgs/TrajectoryXYZVYaw` | Out | 3-waypoint trajectory to desired pose |
| `keyboard_input` | `std_msgs/String` | In | Single-character key from GUI |
| `keyboard_control_enable` | `std_msgs/Bool` | In | Enable/disable GUI keyboard mode |
| `tracking_point_vis` | `geometry_msgs/PoseStamped` | Out | Desired pose for visualization |

## Remapped Topics (with bringup)

| Remapped to | Description |
|------------|-------------|
| `/{robot_name}/trajectory_controller/trajectory_override` | Trajectory controller input |
| `/{robot_name}/keyboard_controller/keyboard_input` | GUI key input (bridged from GCS) |
| `/{robot_name}/keyboard_controller/keyboard_control_enable` | GUI enable toggle (bridged from GCS) |

## CLI Test

```bash
# Enable keyboard control from terminal
ros2 topic pub --once /robot_1/keyboard_controller/keyboard_control_enable std_msgs/Bool "{data: true}"

# Send a key
ros2 topic pub --once /robot_1/keyboard_controller/keyboard_input std_msgs/String "{data: w}"
```
