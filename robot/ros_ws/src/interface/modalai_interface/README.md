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
   PX4 via MicroDDS ─────────────────┘
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

### Simulation

Verifies the ModalAI interface against a simulated VOXL2 drone in Isaac Sim. The sim spawns a drone using the Pegasus PX4 backend — you should see the drone sitting in the Isaac Sim scene. A `sim_qvio_bridge` node replaces the real VOXL2's qvio output, feeding simulated odometry into the interface so it behaves identically to hardware. The interface is working correctly when odometry is flowing on `/robot_1/odometry`.

**Terminal 1 — Build and launch the ROS interface:**

> All ROS commands run inside the robot container — **not on your host machine**. Your host uses ROS Humble; the container uses ROS Jazzy.

```bash
# Start the robot container
AUTOLAUNCH=false airstack up robot-desktop

# Build (only needed once, or after code changes)
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   cd /root/AirStack/robot/ros_ws && \
   colcon build --packages-select modalai_interface"

# Launch the ROS interface
docker exec airstack-robot-desktop-1 bash -c \
  "export ROBOT_NAME=robot_1 && \
   source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 launch modalai_interface modalai_sim.launch.xml"
```

You should see all 4 nodes start:
```
[INFO] [robot_interface_node-1]: process started with pid [...]
[INFO] [odometry_conversion-2]: process started with pid [...]
[INFO] [sim_qvio_bridge.py-3]: process started with pid [...]
[INFO] [drone_safety_monitor-4]: process started with pid [...]
[INFO] [fmu.sim_qvio_bridge]: sim_qvio_bridge started
```

**Terminal 2 — Launch Isaac Sim:**

> Isaac Sim runs in its own container. Pass the env vars inline so they don't get saved to `.env`.
> If Isaac Sim is already running from a previous session, kill it first with `docker rm -f isaac-sim`.

```bash
ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME=modalai_voxl2_pegasus_launch_script.py AUTOLAUNCH=true airstack up isaac-sim
```

Isaac Sim takes **3-5 minutes** to fully load. The GUI will appear and the drone will be visible in the scene. To check for errors:
```bash
docker exec isaac-sim bash -c "tmux capture-pane -t isaac -p"
```

**Terminal 3 — Once the drone is visible in the scene, start the MicroXRCE-DDS agent:**

> This bridges PX4's internal DDS topics to ROS 2. Without it, no `/fmu/out/*` topics will appear.

```bash
docker exec -d isaac-sim bash -c \
  "cd /tmp/Micro-XRCE-DDS-Agent/build && \
   ./MicroXRCEAgent udp4 -p 8888 > /tmp/uxrce.log 2>&1"
```

Then hit **Play** in the Isaac Sim GUI.

**Verify the interface is working** — after hitting play, run:
```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 topic echo /robot_1/odometry_conversion/odometry --once"
```
Expected output (drone sitting at origin before takeoff):
```
A message was lost!!!
        total count change:1
        total count: 1---
header:
  stamp:
    sec: 1780080965
    nanosec: 920918910
  frame_id: map
child_frame_id: base_link
pose:
  pose:
    position:
      x: 0.0009052683017216623
      y: -0.00046574819134548306
      z: 0.00101470947265625
    orientation:
      x: 0.0006589159270852206
      y: -0.029113688983817186
      z: -0.0007315428496036391
      w: 0.9995756062680728
  covariance:
  - 0.0
  ...
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 0.0
  ...
---
```

The `A message was lost` warning is normal — it just means the subscriber connected mid-stream. If the command hangs with no output, the interface is not receiving qvio data — check that the sim is playing and no errors appear in `tmux capture-pane -t isaac -p`.

See [modalai_sim.launch.xml](launch/modalai_sim.launch.xml) and [sim_qvio_bridge.py](scripts/sim_qvio_bridge.py).

---

### Hardware (VOXL2)

Runs the ModalAI interface against a real VOXL2 flight computer. The VOXL2 publishes visual-inertial odometry on `/qvio` via `voxl-mpa-to-ros2`, which the interface converts from NED/FRD to ENU/FLU and publishes as `/robot_1/odometry` for the rest of the autonomy stack. The interface is working correctly when odometry is flowing on `/robot_1/odometry`.

**Step 1 — Build the package** (once, or after any code changes):

```bash
# Start the robot container if it isn't already running
AUTOLAUNCH=false airstack up robot-desktop

# Build
docker exec airstack-robot-desktop-1 bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   cd /root/AirStack/robot/ros_ws && \
   colcon build --packages-select modalai_interface"
```

**Step 2 — On the VOXL2, make sure `voxl-mpa-to-ros2` is running** and publishing on `/qvio`.

**Step 3 — Launch the ROS interface:**

> All ROS commands run inside the robot container — **not on your host machine**.

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "export ROBOT_NAME=robot_1 && \
   source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 launch modalai_interface modalai_hardware.launch.xml"
```

If your VOXL2 uses a different topic namespace (e.g. `/voxl/qvio`), edit `voxl_qvio_topic` in [modalai_hardware.launch.xml](launch/modalai_hardware.launch.xml).

**Verify the interface is working:**
```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 topic echo /robot_1/odometry_conversion/odometry --once"
```
You should see an odometry message with position and orientation data. If it hangs, confirm `voxl-mpa-to-ros2` is running on the VOXL2 and publishing on `/qvio`.

---

## Frame Conventions

| Frame | World | Body |
|-------|-------|------|
| AirStack / ROS | ENU (x=East, y=North, z=Up) | FLU (x=Forward, y=Left, z=Up) |
| PX4 / VOXL qvio | NED (x=North, y=East, z=Down) | FRD (x=Forward, y=Right, z=Down) |

Position conversion: `N=ENU_y, E=ENU_x, D=-ENU_z`

## USD Model Note

The simulation launch script uses the Iris quadrotor as a visual placeholder. To swap in a different drone model, update `DRONE_USD` in [modalai_voxl2_pegasus_launch_script.py](../../../../simulation/isaac-sim/launch_scripts/modalai_voxl2_pegasus_launch_script.py) to point to your USD file.

The workspace is mounted into the container at `/root/AirStack/robot/ros_ws`, so any edits you make on the host are immediately reflected inside — no need to restart the container. See [modalai_interface.cpp](src/modalai_interface.cpp) for the main implementation.
