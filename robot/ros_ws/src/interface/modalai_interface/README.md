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

## Simulation Test

This test verifies that the full command path through the interface works end-to-end in Isaac Sim. The flow is: the AirStack `robot_command` service sends arm and offboard mode requests to `modalai_interface`, which translates them into PX4 uXRCE-DDS messages and forwards them to PX4 SITL running inside Isaac Sim. We first confirm that odometry is flowing back correctly through the interface (PX4 vehicle odometry converted by `sim_qvio_bridge` into the `/qvio` format that the real VOXL2 hardware produces, then converted by `modalai_interface` into AirStack's standard odometry topic). We then run [prop_spin_test.py](scripts/prop_spin_test.py), which arms the drone through the interface and confirms the props visibly spin in the Isaac Sim GUI.

### Step 1 — Clean up any existing containers

Kill stale containers so there are no leftover ROS graphs or PX4 instances.

```bash
docker rm -f isaac-sim airstack-robot-desktop-1 2>/dev/null || true
```

### Step 2 — Start robot container and build

Start the robot container without auto-launching the autonomy stack, then build `px4_msgs` and `modalai_interface`. `px4_msgs` must exactly match PX4 v1.16.1 — the message files in `robot/ros_ws/src/local/controls/px4_msgs/msg/` are pinned to the upstream `release/1.16` branch.

```bash
AUTOLAUNCH=false airstack up robot-desktop

docker exec airstack-robot-desktop-1 bash -c \
  "source ~/.bashrc && bws --packages-select px4_msgs modalai_interface"
```

### Step 3 — Start Isaac Sim container and patch the airframe

Start the Isaac Sim container, then add `COM_RCL_EXCEPT 4` to the Iris airframe file if it isn't already there. This parameter exempts offboard mode from the RC-loss failsafe, which otherwise blocks arming without a physical RC transmitter.

```bash
AUTOLAUNCH=false airstack up isaac-sim

docker exec isaac-sim bash -c "
  grep -q 'COM_RCL_EXCEPT' /isaac-sim/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris || \
  echo 'param set-default COM_RCL_EXCEPT 4' >> /isaac-sim/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris"
```

### Step 4 — Start MicroXRCE-DDS Agent

Start the agent in the background. It bridges PX4's uXRCE-DDS client to ROS 2 DDS on domain 0.

> **Note:** If this is the first run on a fresh container and `MicroXRCEAgent` isn't on PATH, build it first:
> ```bash
> docker exec isaac-sim bash -c "
>   cd /tmp && git clone --depth=1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
>   cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
>   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
>   make -j\$(nproc) && make install && ldconfig"
> ```
> The `ldconfig` step is required — a bare binary copy without registering the shared library causes a `libmicroxrcedds_agent.so` error at runtime.

```bash
docker exec isaac-sim bash -c "rm -f /tmp/uxrce.log"
docker exec -d isaac-sim bash -c \
  "ROS_DOMAIN_ID=0 MicroXRCEAgent udp4 -p 8888 -d 0 > /tmp/uxrce.log 2>&1"
```

### Step 5 — Launch Isaac Sim + PX4 SITL

Send the command to the `isaac` tmux session to start Isaac Sim with the prop-spin test scene. This spawns the Iris drone and starts PX4 SITL.

```bash
docker exec isaac-sim bash -c \
  "tmux send-keys -t isaac 'source /isaac-sim/.bashrc && ROS_DOMAIN_ID=0 \
   run_isaac_python /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/modalai_voxl2_pegasus_launch_script.py \
   --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts' ENTER"
```

### Step 6 — Wait for PX4 to connect (1–3 min)

Poll the MicroXRCEAgent log for `establish_session` — the line it prints when PX4 successfully opens a DDS session. Then give PX4 15 s to finish EKF initialisation.

```bash
until docker exec isaac-sim bash -c "grep -q 'establish_session' /tmp/uxrce.log 2>/dev/null"; do
  sleep 3; echo "  waiting for PX4..."
done
echo "PX4 connected."
sleep 15
```

### Step 7 — Configure FastDDS UDP-only in robot container

The robot container defaults to FastDDS shared-memory transport, which cannot reach the Isaac Sim container. Switching to UDP-only fixes cross-container DDS discovery.

```bash
docker exec airstack-robot-desktop-1 bash -c 'mkdir -p /root/.ros && cat > /root/.ros/fastdds.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF'
```

### Step 8 — Launch modalai_interface (sim mode)

Start the interface using `modalai_sim.launch.xml` with `ROS_DOMAIN_ID=0` and the UDP-only FastDDS profile. This launch file sets `fmu_namespace=px4_1/fmu` and starts the `sim_qvio_bridge` node, which converts PX4's `VehicleOdometry` → `PoseStamped` on `/qvio`, feeding odometry into the interface exactly as real VOXL2 hardware would.

```bash
docker exec -d airstack-robot-desktop-1 bash -c \
  "export ROBOT_NAME=robot_1 ROS_DOMAIN_ID=0 FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml && \
   source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 launch modalai_interface modalai_sim.launch.xml > /tmp/modalai_interface.log 2>&1"

sleep 10
```

### Step 9 — Verify interface odometry

Confirm the full inbound data path is working: PX4 SITL → MicroXRCEAgent → domain 0 → sim_qvio_bridge → `/qvio` (PoseStamped, NED/FRD) → modalai_interface `on_qvio()` → ENU/FLU conversion → AirStack topic.

If nothing is published here, check `/tmp/modalai_interface.log` — the most common cause is `sim_qvio_bridge` not receiving `vehicle_odometry` (usually a `px4_msgs` version mismatch).

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ROS_DOMAIN_ID=0 timeout 5 ros2 topic echo /robot_1/interface/odometry --once"
```

### Step 10 — Run PropSpinTest

Run the test script. Watch the Isaac Sim GUI — props should begin spinning when PX4 arms (~20 s after the script starts).

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && \
   FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml \
   ROS_DOMAIN_ID=0 PX4_NAMESPACE=px4_1 ROBOT_NAME=robot_1 \
   python3 /root/AirStack/robot/ros_ws/src/interface/modalai_interface/scripts/prop_spin_test.py 15"
```

**What the test verifies (full interface path):**
1. `robot_command` service reachable → modalai_interface is up
2. `REQUEST_CONTROL` → modalai_interface calls `DO_SET_MODE (OFFBOARD)` → PX4
3. `ARM` → modalai_interface calls `COMPONENT_ARM_DISARM (1)` → PX4
4. Zero-velocity `TwistStamped` on `cmd_velocity` → modalai_interface converts to NED `trajectory_setpoint` → keeps offboard heartbeat alive
5. `vehicle_control_mode.flag_armed = true` (read directly from PX4 DDS for ground-truth confirmation)

### Step 11 — Tear down

```bash
airstack down
```

---

## Verified Issues and Their Fixes

| Issue | Symptom | Fix |
|-------|---------|-----|
| `px4_msgs` version mismatch | Odometry never flows; payload 207 vs 220 bytes | Replace all msg files from `PX4/px4_msgs release/1.16` |
| `target_system = 1` | All vehicle_commands silently ignored | Changed to `0` (broadcast); PX4 SITL sets `MAV_SYS_ID = vehicle_id+1 = 2` |
| No `trajectory_setpoint` with heartbeat | PX4 rejects offboard mode switch | `publish_offboard_heartbeat()` now publishes both `offboard_control_mode` and `trajectory_setpoint` |
| FastDDS shared-memory in robot container | Cross-container DDS discovery fails | UDP-only FastDDS profile injected before launching modalai_interface |
| Wrong FMU namespace (`fmu` vs `px4_1/fmu`) | Interface publishes/subscribes to wrong topics in sim | Added `fmu_namespace` arg to launch file; `modalai_sim.launch.xml` sets `px4_1/fmu` |
| Missing `vehicle_status_v1` remap | Interface never receives arm/mode state from PX4 | Added remap `out/vehicle_status` → `out/vehicle_status_v1` (PX4 v1.15+ rename) |

---

## Hardware Test (Starling 2 Max / VOXL2)

Arms the drone through `modalai_interface` and confirms props spin on real hardware.
Not testing flight — just verifying the full AirStack → PX4 command path works.

- **`--force`**: bypasses EKF2 yaw alignment preflight check (needed on bench because OpenVINS timestamps are stale → VIO never reaches PX4). For actual flight this must be fixed.
- **`--network host`**: required because VOXL2's internal Docker bridge conflicts with the desktop's bridge (both `172.31.0.0/24`), breaking bidirectional DDS. Host-network bypasses all Docker NAT.

### Run the script

```bash
# USB cable (workstation on different subnet from drone's WiFi)
./robot/ros_ws/src/interface/modalai_interface/scripts/run_hw_test.sh

# WiFi (workstation already on same network as drone — Linux only, requires sshpass)
./robot/ros_ws/src/interface/modalai_interface/scripts/run_hw_test.sh --wifi

# Leave interface running without spinning props (for manual testing)
./robot/ros_ws/src/interface/modalai_interface/scripts/run_hw_test.sh --up-only

# Custom spin duration (default 10s) or override VOXL2 WiFi IP
./robot/ros_ws/src/interface/modalai_interface/scripts/run_hw_test.sh --duration 5
./robot/ros_ws/src/interface/modalai_interface/scripts/run_hw_test.sh --wifi --voxl-ip <ip>
```

USB mode requires `sudo` + `adb`. WiFi mode requires `sshpass` (`sudo apt install sshpass`).
For first-time USB setup or manual steps see [VOXL2_USB_ETHERNET.md](VOXL2_USB_ETHERNET.md).

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ping 192.168.123.2` fails | USB NCM not set up or cable reconnected | Re-run usb-ncm + usb-rebind services, assign workstation IP (see VOXL2_USB_ETHERNET.md). |
| DDS topics listed but no data flows | `wlan0` still up; microdds-agent advertising unreachable WiFi IP | `adb shell "ip link set wlan0 down"` then restart microdds-agent. |
| `robot_command` service not found | Container not started or still initializing | `docker logs modalai-hw-test`; wait a few more seconds. |
| PX4 did not arm within 20s | `cs_yaw_align: False` — VIO not flowing | Use `--force`. Expected on bench. |
| `voxl-mpa-ros2.service` failed (status=250) | `HOME` not set in systemd unit | Add `Environment=HOME=/root` and `Environment=ROS_LOG_DIR=/tmp/ros2_log` to unit file. |
| `voxl-vision-hub` logs `VIO time X.Xs too old` | OpenVINS timestamps ~0.9s behind wall clock | Known issue. Use `--force` for bench. Fix clock domain mismatch in OpenVINS config for real flights. |
| `voxl-open-vins-server` logs `In init too long, retry RESET` | IMU overheated or needs recalibration | Power off 10 min, recalibrate via QGroundControl → Sensors. |

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
