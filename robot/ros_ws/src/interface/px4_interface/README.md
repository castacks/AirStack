# PX4 Interface

ROS 2 plugin (`robot_interface::RobotInterface`) that connects AirStack to PX4 SITL via **uXRCE-DDS** — no MAVROS required.

## Architecture

```
AirStack autonomy stack
        │  robot_command service (arm/disarm/takeoff/land/request_control)
        │  cmd_velocity / cmd_pose  (ENU/FLU setpoints)
        ▼
  px4_interface node        ← THIS PACKAGE
        │  ENU↔NED + FLU↔FRD frame conversion
        │  offboard heartbeat (offboard_control_mode + trajectory_setpoint @ 10 Hz)
        │  VehicleCommand (arm, mode switch, takeoff, land)
        ▼
  MicroXRCE-DDS Agent  (UDP port 8888, domain 0)
        ▼
  PX4 SITL (inside Isaac Sim container)
```

Odometry flows the reverse direction: PX4 `vehicle_odometry` (NED/FRD) → converted to `nav_msgs/Odometry` (ENU/FLU) → published on `/{ROBOT_NAME}/interface/odometry`.

---

## Simulation Test (PropSpinTest)

The PropSpinTest proves the **full interface command path** works — from the AirStack `robot_command` service through px4_interface all the way to PX4 arming in Isaac Sim (visible as props spinning in the GUI).

### Step 1 — Clean up any existing containers

Kill stale containers so there are no leftover ROS graphs or PX4 instances.

```bash
docker rm -f isaac-sim airstack-robot-desktop-1 2>/dev/null || true
```

### Step 2 — Start robot container and build

Start the robot container without auto-launching the autonomy stack, then build `px4_msgs` and `px4_interface`. `px4_msgs` must exactly match PX4 v1.16.1 — the message files in `robot/ros_ws/src/local/controls/px4_msgs/msg/` are pinned to the upstream `release/1.16` branch.

```bash
AUTOLAUNCH=false airstack up robot-desktop

docker exec airstack-robot-desktop-1 bash -c \
  "source ~/.bashrc && bws --packages-select px4_msgs px4_interface"
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
   run_isaac_python /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/px4_prop_spin_test.py \
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

### Step 8 — Launch px4_interface

Start the interface node with `ROS_DOMAIN_ID=0` (PX4 topics are on domain 0) and the UDP-only FastDDS profile. The launch file pushes the `px4_1/fmu` namespace and remaps `vehicle_status` → `vehicle_status_v1` for PX4 v1.15+ compatibility.

```bash
docker exec -d airstack-robot-desktop-1 bash -c \
  "export ROBOT_NAME=robot_1 ROS_DOMAIN_ID=0 FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml && \
   source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 launch px4_interface px4_interface.launch.xml > /tmp/px4_interface.log 2>&1"

sleep 10
```

### Step 9 — Verify interface odometry

Confirm the full inbound data path is working: PX4 SITL → MicroXRCEAgent → domain 0 → px4_interface `on_vehicle_odometry()` → ENU/FLU conversion → AirStack topic.

If nothing is published here, the most likely cause is a `px4_msgs` version mismatch — FastDDS silently drops messages with the wrong payload size.

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
   ROS_DOMAIN_ID=0 PX4_NAMESPACE=px4_1 \
   python3 /root/AirStack/robot/ros_ws/src/interface/px4_interface/scripts/prop_spin_test.py 15"
```

**What the test verifies (full interface path):**
1. `robot_command` service reachable → px4_interface is up
2. `REQUEST_CONTROL` → px4_interface calls `DO_SET_MODE (OFFBOARD)` → PX4
3. `ARM` → px4_interface calls `COMPONENT_ARM_DISARM (1)` → PX4
4. Zero-velocity `TwistStamped` on `cmd_velocity` → px4_interface converts to NED `trajectory_setpoint` → keeps offboard heartbeat alive
5. `vehicle_control_mode.flag_armed = true` (read directly from PX4 DDS for ground-truth confirmation)

---

## Verified Issues and Their Fixes

| Issue | Symptom | Fix |
|-------|---------|-----|
| `px4_msgs` version mismatch | Odometry never flows; payload 207 vs 220 bytes | Replace all msg files from `PX4/px4_msgs release/1.16` |
| `target_system = 1` | All vehicle_commands silently ignored | Changed to `0` (broadcast); PX4 SITL sets `MAV_SYS_ID = vehicle_id+1 = 2` |
| No `trajectory_setpoint` with heartbeat | PX4 rejects offboard mode switch | `publish_offboard_heartbeat()` now publishes both `offboard_control_mode` and `trajectory_setpoint` |
| FastDDS shared-memory in robot container | Cross-container DDS discovery fails | UDP-only FastDDS profile injected before launching px4_interface |
| MicroXRCEAgent shared library missing | `libmicroxrcedds_agent.so.3.0` not found at runtime | Build with `make install && ldconfig`, not just binary copy |

---

## Frame Conventions

| | Position | Attitude |
|-|----------|----------|
| AirStack / ROS | ENU (x=East, y=North, z=Up) | FLU (x=Fwd, y=Left, z=Up) |
| PX4 uXRCE-DDS | NED (x=North, y=East, z=Down) | FRD (x=Fwd, y=Right, z=Down) |

Conversions: `NED_x = ENU_y`, `NED_y = ENU_x`, `NED_z = -ENU_z`

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `COM_RCL_EXCEPT` | `4` | Exempt offboard mode from RC-loss failsafe |
| `MAV_SYS_ID` | `vehicle_id + 1` | PX4 SITL system ID — px4_interface uses `target_system=0` (broadcast) to match any instance |
| MicroXRCEAgent port | `8888 UDP` | Bridge between PX4 uXRCE-DDS and ROS 2 DDS |
| `ROS_DOMAIN_ID` | `0` | PX4 topics are always on domain 0 |
