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

## Hardware Test (VOXL2)

This test does the same thing as the simulation test — arms the drone through the interface and confirms the props spin — but against the real VOXL2 hardware instead of Isaac Sim. The robot container still runs on your workstation. The VOXL2 just needs to be reachable over the network so that DDS topics flow between them.

---

### Step 1 — Physical setup

Power on the drone and make sure it is sitting on a flat surface. **Remove the propellers** before running any arming test for the first time. You can always reattach them once you have confirmed the arm command works.

Connect the VOXL2 to the same network as your workstation. The easiest way is a direct Ethernet cable between the drone and your router, or connecting it to the same WiFi network your workstation is on. DDS (the ROS 2 communication layer) uses UDP multicast, which only works if both machines are on the same network segment — it will not work over ADB USB alone.

---

### Step 2 — Connect via ADB and verify the drone's internal services

Plug a USB cable from the VOXL2 into your workstation. ADB lets you open a shell directly on the drone without needing SSH or a password.

```bash
# Confirm the VOXL2 is visible
adb devices
```

You should see one device listed. If you see nothing, check the USB cable and that ADB is installed (`sudo apt install adb`).

```bash
# Open a shell on the drone
adb shell
```

Once inside, check that the two services the interface depends on are running:

```bash
# The MicroXRCE-DDS bridge — bridges PX4's internal topics to ROS 2
systemctl status voxl-microdds-agent

# PX4 flight controller
systemctl status voxl-px4
```

Both should show `active (running)`. If either is not:

```bash
systemctl start voxl-microdds-agent
systemctl start voxl-px4
```

> **Note:** On Starling 2 Max firmware, `voxl-inspect-services` and `voxl-start-services` do not exist — use `systemctl` instead. The commands below are kept for reference on older firmware only.
>
> ```bash
> # (older firmware only — may not work)
> # voxl-inspect-services | grep mpa-to-ros
> # voxl-inspect-services | grep px4
> # voxl-start-services voxl-mpa-to-ros2
> # voxl-start-services voxl-px4-to-ros
> ```

Next, start `voxl-mpa-to-ros2`. On Starling 2 Max firmware it is not a systemd service — run it manually:

```bash
source /opt/ros/foxy/mpa_to_ros2/install/setup.bash
nohup ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node > /tmp/mpa_to_ros2.log 2>&1 &
```

> **Note:** On this firmware VIO is published on `/vvhub_body_wrt_fixed/pose` instead of `/qvio`. `modalai_hardware.launch.xml` has been updated accordingly. If you are on firmware that does publish `/qvio`, revert the `voxl_qvio_topic` arg in that launch file.

Find the VOXL2's IP address and ROS domain ID:

```bash
ip addr show wlan0   # use eth0 if ethernet-connected
printenv ROS_DOMAIN_ID   # empty means domain 0
```

Write down the IP and domain ID. Then exit the ADB shell:

```bash
exit
```

**Optional — VOXL Portal (web UI):** Forward the portal port over ADB, then open it in your browser:

```bash
# On your workstation:
adb forward tcp:8080 tcp:80
# Then open: http://localhost:8080
```

---

### Step 3 — Confirm network connectivity from your workstation

```bash
ping <VOXL2_IP>
```

You should get replies. If not, the drone and workstation are not on the same network segment and DDS will not work.

---

### Step 4 — Set COM_RCL_EXCEPT on the drone

This PX4 parameter allows arming in offboard mode without a physical RC transmitter connected. From the ADB shell, open the PX4 console and set it:

```bash
voxl-px4-shell
# Now at the PX4 prompt (pxh>):
param set COM_RCL_EXCEPT 4
exit
```

---

### Step 5 — Build modalai_interface on your workstation

Only needed once, or after any code changes.

```bash
AUTOLAUNCH=false airstack up robot-desktop

docker exec airstack-robot-desktop-1 bash -c \
  "source ~/.bashrc && bws --packages-select px4_msgs modalai_interface"
```

---

### Step 6 — Configure FastDDS UDP-only in the robot container

The robot container defaults to shared-memory DDS transport, which only works within the same machine. Switching to UDP-only allows it to discover the VOXL2's topics over the network.

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

---

### Step 7 — Launch modalai_interface

Use `modalai_hardware.launch.xml`. This is the same as the sim launch except it does not start `sim_qvio_bridge` — the real VOXL2 publishes VIO directly. Replace `<VOXL_DOMAIN>` with the domain ID you found in Step 2 (use `0` if `ROS_DOMAIN_ID` was empty).

```bash
docker exec -d airstack-robot-desktop-1 bash -c \
  "export ROBOT_NAME=robot_1 ROS_DOMAIN_ID=<VOXL_DOMAIN> FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml && \
   source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 launch modalai_interface modalai_hardware.launch.xml > /tmp/modalai_interface.log 2>&1"

sleep 10
```

---

### Step 8 — Verify odometry is flowing from the drone

This confirms the full path: VOXL2 publishes VIO (`/vvhub_body_wrt_fixed/pose` on Starling 2 Max firmware) → modalai_interface converts NED/FRD to ENU/FLU → `/robot_1/interface/odometry`. If the drone is sitting still you should see near-zero position values.

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ROS_DOMAIN_ID=<VOXL_DOMAIN> timeout 5 ros2 topic echo /robot_1/interface/odometry --once"
```

If nothing comes back, check `/tmp/modalai_interface.log` and confirm `voxl-mpa-to-ros2` is publishing `/qvio` on the drone (re-run Step 2).

---

### Step 9 — Run the prop spin test

This is identical to the simulation test. The interface receives the arm command, forwards it to PX4 on the VOXL2, and PX4 arms. If you left propellers on, they will spin.

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "source /root/AirStack/robot/ros_ws/install/setup.bash && \
   FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml \
   ROS_DOMAIN_ID=<VOXL_DOMAIN> ROBOT_NAME=robot_1 \
   python3 /root/AirStack/robot/ros_ws/src/interface/modalai_interface/scripts/prop_spin_test.py 15"
```

The script will print `interface verified` if PX4 armed successfully, then disarm after 15 seconds.

---

### Step 10 — Tear down

```bash
airstack down
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `voxl-open-vins-server` repeatedly logs `In init too long, timeout, retry RESET` | IMU overheated or needs recalibration | Power off the drone for 10 minutes to cool, then retry. If it persists, recalibrate IMU/accelerometer via QGroundControl (Sensors tab). |
| `voxl-open-vins-server` logs `Cannot initialize FRD to IMU transform--too much drift` | Same as above — IMU bias too high for VIO to initialize | Same fix: cool down + recalibrate. |
| `voxl-camera-server` logs `preview buffer pool has 0 free, skipping request` | Too many consumers of the tracking camera stream | Kill any extra `voxl_mpa_to_ros2_node` instances (`pkill -f voxl_mpa_to_ros2_node`) and restart camera/VIO services. |
| `ros2 topic hz /vvhub_body_wrt_fixed/pose` returns nothing | `voxl_mpa_to_ros2_node` not running or died after shell exit | Re-run the `nohup ros2 run ...` command from Step 2 in a new ADB shell. |
| Prop spin test times out waiting for PX4 to arm | VIO not initialized → PX4 EKF unhealthy | Ensure `voxl-open-vins-server` is running cleanly and `/vvhub_body_wrt_fixed/pose` is publishing before running the test. |

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
