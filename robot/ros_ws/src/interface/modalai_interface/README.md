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

Verifies the ModalAI interface against a simulated VOXL2 drone in Isaac Sim. The sim spawns a drone using the Pegasus PX4 backend — you should see the drone sitting in the Isaac Sim scene. A `sim_qvio_bridge` node replaces the real VOXL2's qvio output, feeding simulated odometry into the interface so it behaves identically to hardware. The interface is working correctly when you see the props spin.

#### Step 1 — Run the launch script

From the AirStack repo root:

```bash
cd robot/ros_ws/src/interface/modalai_interface
./scripts/run_sim.sh
```

The script does the following automatically:
1. Kills any existing `isaac-sim` and `airstack-robot-desktop-1` containers from previous sessions
2. Starts the robot container and launches the ROS interface (AirStack side) in the background
3. Starts the Isaac Sim container with the VOXL2 drone scene
4. Starts the MicroXRCE-DDS agent, which bridges PX4's internal topics to ROS 2

If you've made code changes and need to rebuild first:
```bash
REBUILD=true ./scripts/run_sim.sh
```

#### Step 2 — Wait for Isaac Sim to load (3-5 minutes)

The Isaac Sim GUI will appear on your screen with the drone sitting in the scene. Once fully loaded, the launch script automatically hits Play, arms the drone, and spins the props for 15 seconds to verify the full communication chain is working end-to-end.

#### Step 3 — Confirm the props spun

```bash
docker exec isaac-sim bash -c "tmux capture-pane -t isaac:0.0 -p | tail -30"
```

Look for this line in the output:
```
PropSpinTest: props spinning for 15s — interface verified!
```

If you see it, the interface is working correctly. If you don't, see Troubleshooting below.

- To skip the prop spin test: set `PROP_SPIN_TEST=false`
- To change the duration: set `PROP_SPIN_DURATION=<seconds>`

---

**Troubleshooting**

Watch the ROS interface logs:
```bash
docker exec airstack-robot-desktop-1 bash -c "tail -f /tmp/modalai_interface.log"
```

Check Isaac Sim for errors:
```bash
docker exec isaac-sim bash -c "tmux capture-pane -t isaac:0.0 -p | tail -30"
```

Manually verify odometry is flowing:
```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 topic echo /robot_1/odometry_conversion/odometry --once"
```

Expected output (drone sitting at origin):
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

The `A message was lost` warning is normal — it just means the subscriber connected mid-stream. If the command hangs with no output, confirm the sim is playing and check `tmux capture-pane -t isaac -p` for errors.

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

Two ModalAI models are defined in [modalai_voxl2_pegasus_launch_script.py](../../../../simulation/isaac-sim/launch_scripts/modalai_voxl2_pegasus_launch_script.py):

| Constant | Model |
|----------|-------|
| `STARLING2_MAX` | Starling 2 Max (default) |
| `STARLING2` | Starling 2 |

To switch models, change the `DRONE_USD` line at the top of the script:
```python
DRONE_USD = STARLING2      # or STARLING2_MAX
```

Both are loaded from the AirLab Nucleus server (`airlab-nucleus.andrew.cmu.edu`) — Isaac Sim must be connected to Nucleus for the USD to load.

The workspace is mounted into the container at `/root/AirStack/robot/ros_ws`, so any edits you make on the host are immediately reflected inside — no need to restart the container. See [modalai_interface.cpp](src/modalai_interface.cpp) for the main implementation.
