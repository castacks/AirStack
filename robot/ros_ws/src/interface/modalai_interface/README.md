# ModalAI Interface

RobotInterface plugin for the [ModalAI VOXL2](https://www.modalai.com/products/voxl-2) flight computer.

## Overview

The VOXL2 runs PX4 internally and exposes it via the MicroDDS bridge — the same `/fmu/in/*` and `/fmu/out/*` topic interface used by `px4_interface`. The only meaningful difference is the **odometry source**: instead of reading PX4's own `out/vehicle_odometry`, this interface subscribes to VOXL's `qvio` topic (visual-inertial odometry from onboard cameras) and converts it from NED/FRD → ENU/FLU for the rest of AirStack.

## Architecture

```
AirStack autonomy stack
        │  cmd_velocity / cmd_pose / cmd_attitude_thrust / ...
        ▼
 ModalAIInterface (pluginlib plugin)
        │                            ▲
        │  /fmu/in/trajectory_setpoint    qvio (PoseStamped, NED/FRD)
        │  /fmu/in/vehicle_command        └── from voxl-mpa-to-ros2 (hardware)
        │  /fmu/in/offboard_control_mode      or sim_qvio_bridge (simulation)
        ▼                            │
   PX4 via MicroDDS ────────────────┘
        │  /fmu/out/vehicle_status (arm state, nav state)
```

## Topics

### Subscribed (from AirStack)
| Topic | Type | Description |
|-------|------|-------------|
| `cmd_pose` | `geometry_msgs/PoseStamped` | Position setpoint (ENU) |
| `cmd_velocity` | `geometry_msgs/TwistStamped` | Velocity setpoint (ENU) |
| `cmd_attitude_thrust` | `mav_msgs/AttitudeThrust` | Attitude + thrust |
| `cmd_rate_thrust` | `mav_msgs/RateThrust` | Body rate + thrust |
| `cmd_roll_pitch_yawrate_thrust` | `mav_msgs/RollPitchYawrateThrust` | RPY rate + thrust |
| `qvio` | `geometry_msgs/PoseStamped` | VOXL visual-inertial odometry (NED/FRD) |
| `visual_odometry_in` | `nav_msgs/Odometry` | Optional: forward AirStack VIO → PX4 EKF |

### Published (to AirStack)
| Topic | Type | Description |
|-------|------|-------------|
| `odometry` | `nav_msgs/Odometry` | Robot odometry (ENU/FLU), converted from qvio |

### Published (to PX4 via MicroDDS)
| Topic | Type |
|-------|------|
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` |
| `/fmu/in/vehicle_attitude_setpoint` | `px4_msgs/VehicleAttitudeSetpoint` |
| `/fmu/in/vehicle_rates_setpoint` | `px4_msgs/VehicleRatesSetpoint` |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` |

## Usage

### Hardware (VOXL2)

```xml
<group>
  <push-ros-namespace namespace="fmu" />
  <include file="$(find-pkg-share modalai_interface)/launch/modalai_interface.launch.xml">
    <arg name="voxl_qvio_topic" value="/qvio" />
  </include>
</group>
```

Adjust `voxl_qvio_topic` to match your `voxl-mpa-to-ros2` namespace config (common values: `/qvio`, `/voxl/qvio`).

### Simulation (Isaac Sim)

1. Launch Isaac Sim with the VOXL2 scene script:
   ```bash
   python launch_scripts/modalai_voxl2_pegasus_launch_script.py
   ```

2. Launch the ROS stack with the sim qvio bridge enabled:
   ```xml
   <include file="$(find-pkg-share modalai_interface)/launch/modalai_interface.launch.xml">
     <arg name="use_sim_qvio_bridge" value="true" />
   </include>
   ```

   `use_sim_qvio_bridge` starts `sim_qvio_bridge.py`, which converts PX4's `out/vehicle_odometry` → `geometry_msgs/PoseStamped` on `/qvio`, mimicking what `voxl-mpa-to-ros2` does on real hardware. **Do not enable this on real hardware.**

## Frame Conventions

| Frame | World | Body |
|-------|-------|------|
| AirStack / ROS | ENU (x=East, y=North, z=Up) | FLU (x=Forward, y=Left, z=Up) |
| PX4 / VOXL qvio | NED (x=North, y=East, z=Down) | FRD (x=Forward, y=Right, z=Down) |

Position conversion: `N=ENU_y, E=ENU_x, D=-ENU_z`

## USD Model Note

The simulation launch script uses the Iris quadrotor as a visual placeholder. To swap in a different drone model, update `DRONE_USD` in `modalai_voxl2_pegasus_launch_script.py` to point to your USD file.

## Building

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   cd /root/AirStack/robot/ros_ws && \
   colcon build --packages-select modalai_interface"
```
