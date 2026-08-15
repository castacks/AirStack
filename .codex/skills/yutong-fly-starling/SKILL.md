---
name: yutong-fly-starling
description: Real-lab workflow for testing or flying Yutong's ModalAI Starling 2 Max drones with AirStack SVG ground control, OptiTrack Motive/NatNet mocap, VOXL/PX4, QGroundControl, Micro XRCE-DDS, ROS 2 Jazzy, Docker, and rosbag recording. Use when the user asks to set up, verify, debug, or fly a Starling/VOXL drone in the lab; asks about Motive/NatNet, drone_2, drone_3, drone_4, voxl-bs, voxl-by, voxl-bv, mocap pose, qvio/VIO, QGC, MicroXRCEAgent, ROS_DOMAIN_ID, or real-flight checklists; or wants help recording/visualizing soccer ball or drone mocap data.
---

# Yutong Fly Starling

Use this skill as the operating checklist for Yutong's Starling 2 Max lab fleet. BS, BY, and BV are all valid potential active drones; the labels do not imply age, preference, or replacement order. Be conservative: do not arm or fly until the safety gates pass and the user explicitly confirms the physical setup is ready.

For a new Ubuntu ground-controller computer, read
[ONBOARDING.md](ONBOARDING.md). For copy-paste flight and policy commands, read
[QUICK_REFERENCE.md](QUICK_REFERENCE.md).

## Fixed Assumptions

Treat these as the known-good lab values unless the user says they changed:

- AirStack repo: `/home/yutongw/Desktop/AirStack`, branch `yikuan/SVG_ground_control`.
- Known robot-name / Motive rigid-body mapping: BS and BY use `drone_2`; BV currently uses `drone_3`.
- Known VOXL SSH aliases:
  - `voxl-bs`: BS Starling/VOXL at `192.168.50.73`, SSH key `~/.ssh/id_voxl_m0054`.
  - `voxl-by`: BY Starling/VOXL at `192.168.50.6`, SSH key `~/.ssh/id_voxl_by`; it may still report hostname `m0054`.
  - `voxl-bv`: BV Starling/VOXL at `192.168.50.12`, SSH key `~/.ssh/id_voxl_bv`; it reports hostname `m0054`.
- Ground Ubuntu PC IP on router LAN: `192.168.50.139`.
- Motive PC IP: `192.168.50.5`.
- Router subnet: `192.168.50.x`.
- NatNet multicast: `239.255.42.99`; command port `1510`; data port `1511`.
- Current BV ROS domain: `ROS_DOMAIN_ID=1`.
- Current BV Micro XRCE-DDS agent UDP port: `8892`.
- QGC runs on the ground Ubuntu PC or another PC that can reach the drone.
- The AirStack Docker `robot-desktop` service should use host networking for real hardware.

At the start of a session, choose the active drone from the known mappings:

| Drone | `VOXL_ALIAS` | `VOXL_IP` | `DRONE_NAME` | ROS domain | DDS port |
|---|---|---|---|---|---|
| BS | `voxl-bs` | `192.168.50.73` | `drone_2` | verify | verify |
| BY | `voxl-by` | `192.168.50.6` | `drone_2` | verify | verify |
| BV | `voxl-bv` | `192.168.50.12` | `drone_3` | `1` | `8892` |

For example, to select BV:

```bash
VOXL_ALIAS=voxl-bv
VOXL_IP=192.168.50.12
DRONE_NAME=drone_3
ROS_DOMAIN_ID=1
DDS_AGENT_IP=192.168.50.139
DDS_AGENT_PORT=8892
```

Verify:

```bash
ssh "$VOXL_ALIAS" 'hostname; ip -brief addr | grep 192.168.50 || true'
```

Do not treat the Linux hostname as the flight identity. For `voxl-by`, hostname can still print `m0054`; the SSH alias and current IP are the operational identifiers.

## Safety Rules

- Never suggest arming or takeoff while RC/safety path is missing, mocap is not live, estimator validity is unknown, or the flight area is not physically clear.
- If RC is unavailable, keep tests disarmed: mocap hand-carry, QGC inspector, PX4 estimator checks, command plumbing, rosbag recording, and RViz visualization are still useful.
- If QGC shows yellow "not ready", do not assume arming will make it green. Check the concrete preflight failure.
- Indoor mocap flight intentionally has no compass. The known PX4 setup uses external vision/mocap and disables compass checks.
- Do not disable External Vision when using mocap; mocap enters PX4 through the visual odometry path.
- **BV / `drone_3` is no-fly after the 2026-07-23 runaway** until its external-vision frame is corrected and revalidated disarmed. With `px4_vio_frame: "enu_to_ned"`, mocap position derivative and PX4 estimated horizontal velocity disagreed, producing horizontal positive feedback before a roll failsafe and wall impact. The PX4 ULog is `/data/px4/log/2026-07-23/20_07_41.ulg`.
- For BV, an RViz carry test is insufficient: carry North and verify
  `vehicle_visual_odometry.position[0]` increases, carry East and verify
  `position[1]` increases, lift and verify `position[2]` decreases, and verify
  PX4 heading matches the physical nose. On 2026-07-30, `modalai_flip` failed
  this test: East increased PX4 `x`, North decreased PX4 `y`, and Up decreased
  PX4 `z`. This proves its world-position mapping is wrong for the current
  Motive frame. Although `enu_to_ned` supplies the required `[y, x, -z]`
  position mapping, its quaternion conversion must also pass the nose-heading
  test; its prior use preceded BV's 2026-07-23 runaway.
- The 2026-07-30 BV retest confirmed that Motive currently publishes ENU world
  coordinates and `drone_3` rigid-body `+X` is BV's physical nose: with the
  nose East, the raw Motive quaternion was near identity. Use
  `px4_vio_frame: "enu_to_ned"` for this configuration; it maps position to
  `[y, x, -z]` and an East-facing identity Motive attitude to approximately
  `+90 deg` PX4 NED yaw. Re-run all translation and nose-heading carry checks
  after every Motive calibration or rigid-body redefinition.

## One-Time Repo/Container Setup

Verify the local AirStack configuration before the lab run:

```bash
cd /home/yutongw/Desktop/AirStack
git branch --show-current
./airstack.sh status
```

Expected AirStack edits from the previous setup:

- `.env`: `COMPOSE_PROFILES="desktop"`, `AUTOLAUNCH="false"`, `NUM_ROBOTS="1"`.
- `robot/docker/docker-compose.yaml` and `robot/docker/.bashrc`: `robot-desktop` uses host networking and `ROS_DOMAIN_ID=1`.
- `robot/ros_ws/src/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_fmu_hover.yaml` configures the BV direct-FMU hover pipeline and mocap bridge without `swarm_commander`.
- `robot/ros_ws/src/interface/robot_interface/src/robot_interface_node.cpp` loads the configured interface plugin, so `px4_interface::PX4Interface` can load instead of the old hardcoded MAVROS interface.

If source changes were made or pulled, rebuild the relevant packages inside the container:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; bws --packages-select robot_interface px4_interface svg_ground_control'
```

## Start the Lab Stack

Start AirStack:

```bash
cd /home/yutongw/Desktop/AirStack
./airstack.sh up robot-desktop
```

Use a plain shell when needed:

```bash
docker exec -it airstack-robot-desktop-1 bash
```

Do not rely on `./airstack.sh connect robot-desktop` when the user wants a normal shell; it may attach to tmux directly.

Start the Micro XRCE-DDS agent on the ground PC:

```bash
docker run -d --rm --name micro_ros_agent_jazzy \
  --network host \
  -e ROS_DOMAIN_ID=1 \
  microros/micro-ros-agent:jazzy \
  udp4 --port 8892 -v4
```

Check it:

```bash
docker logs --tail 80 micro_ros_agent_jazzy
```

For BV, expect the agent on port `8892` to establish a session from `192.168.50.12`.

## Configure VOXL/PX4

The drone-side setup script should already exist at `/usr/bin/voxl_setup_real_drone.sh`. Run it on the active VOXL when the ground PC IP, ROS domain, port, or drone name must be re-applied:

```bash
ssh "$VOXL_ALIAS" "voxl_setup_real_drone.sh $DRONE_NAME $DDS_AGENT_IP $ROS_DOMAIN_ID $DDS_AGENT_PORT"
```

This configures `/usr/bin/voxl-px4-start` to set:

```bash
param set XRCE_DDS_DOM_ID 1
microdds_client start -t udp -h 192.168.50.139 -p 8892 -n drone_3
```

It also disables the onboard `voxl-microdds-agent` and restarts `voxl-px4`.

On BV, rerunning the provisioning script rewrites
`svg_microdds_watchdog.sh`, but an already-running watchdog can retain its old
agent address in memory. Restart it after provisioning, then stop the client
once and verify that the watchdog restores the new address:

```bash
ssh "$VOXL_ALIAS" 'systemctl restart svg-microdds-watchdog; px4-microdds_client stop; sleep 4; px4-microdds_client status'
```

Verify:

```bash
ssh "$VOXL_ALIAS" 'voxl-inspect-services | grep -E "voxl-px4|voxl-microdds-agent|voxl-qvio|voxl-dfs"'
ssh "$VOXL_ALIAS" 'px4-microdds_client status'
```

Expected:

- `voxl-px4` enabled/running.
- `voxl-microdds-agent` disabled/not running.
- `voxl-qvio-server` and `voxl-dfs-server` disabled/not running for mocap-only testing.
- MicroDDS client connected to agent IP `192.168.50.139`, port `8892`, with nonzero tx/rx.

## Start Runtime ROS Sessions

Run these inside the AirStack container or through `docker exec`. Keep them alive in separate tmux sessions.

NatNet bridge:

```bash
tmux new-session -d -s natnet "bash -lc 'sws; ros2 launch natnet_ros2 natnet_ros2.launch.py serverIP:=192.168.50.5 clientIP:=192.168.50.139 activate:=true pub_rigid_body:=true'"
```

If NatNet returns error code `3`, confirm Motive Data Streaming is enabled and
check `ip route get 239.255.42.99`. The multicast route must use the lab Wi-Fi
interface/address, not an unrelated Ethernet interface. Switching the bridge
to unicast does not help when the Motive NatNet server itself is unavailable.

BV direct-FMU interfaces:

```bash
tmux new-session -d -s real_interfaces "bash -lc 'sws; ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_3'"
tmux new-session -d -s mocap_bridge "bash -lc 'sws; ros2 run svg_ground_control mocap_bridge --ros-args --params-file $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_fmu_hover.yaml'"
```

The current Motive ball rigid body is `SoccerBall`. The real-hardware mocap
configs publish its shared Kalman state on `/SoccerBall/mocap_odometry` with
position noise `0.004 m`, acceleration noise `0.1 m/s²`, and initial velocity
noise `1.7 m/s`. Soccer policies use that ball odometry, while drone velocity
continues to come from PX4/onboard-EKF odometry.

Do not run `swarm_commander` or `ground_control.launch.py` with this direct-FMU path. Start `trajectory_commander` only after the disarmed estimator and frame checks pass.

Manage tmux:

```bash
tmux ls
tmux attach -t natnet
tmux attach -t real_interfaces
tmux attach -t mocap_bridge
```

Detach with `Ctrl-b d`. If a session was killed, restart only that session.

NatNet logs should show Motive server IP `192.168.50.5`, client IP `192.168.50.139`, the `drone_3` rigid body, and mocap framerate near `120.00`.

## Core Checks Before Any Arm

Check domain and PX4 topics:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID; sws >/dev/null; ros2 topic list | grep ^/drone_3/fmu'
```

Check duplicate nodes. Duplicates usually mean another setup is on the same ROS domain.

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; ros2 node list | sort | uniq -c | grep -E "natnet|mocap|trajectory_commander|drone_3"'
```

Expected before starting the trajectory publisher: one each for `/natnet_ros`, `/mocap_bridge`, `/drone_3/fmu/px4_interface`, and `/drone_3/px4_micro_xrce_dds`; no `/swarm_commander`.

Check mocap pose:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; ros2 topic echo /drone_3/pose --field pose.position'
```

Check mocap-to-PX4 visual odometry:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; ros2 topic echo /drone_3/fmu/in/vehicle_visual_odometry --qos-reliability best_effort'
```

Expected: `quality: 100`, position values from mocap, and timestamp `0` if using the current SVG bridge behavior.

Check AirStack odometry:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; ros2 topic echo /drone_3/odometry_conversion/odometry --field pose.pose.position'
```

Check PX4 estimator:

```bash
ssh "$VOXL_ALIAS" 'px4-listener vehicle_visual_odometry -n 1 | head -80'
ssh "$VOXL_ALIAS" 'px4-listener vehicle_local_position -n 1 | grep -E "x:|y:|z:|eph:|epv:|xy_valid|z_valid|v_xy_valid|v_z_valid|dead_reckoning"'
ssh "$VOXL_ALIAS" 'px4-listener estimator_status_flags -n 1 | grep -E "cs_ev_pos|cs_ev_yaw|cs_ev_hgt|reject_hor_pos|reject_ver_pos|reject_yaw|cs_inertial_dead_reckoning"'
ssh "$VOXL_ALIAS" 'px4-listener failsafe_flags -n 1 | grep -E "local_altitude_invalid|local_position_invalid|local_velocity_invalid|manual_control_signal_lost|offboard_control_signal_lost|gcs_connection_lost"'
```

Expected before considering flight:

- `vehicle_visual_odometry` age is small, ideally under `0.1s`.
- `xy_valid`, `z_valid`, `v_xy_valid`, and `v_z_valid` are true.
- `dead_reckoning` is false.
- `cs_ev_pos`, `cs_ev_yaw`, and `cs_ev_hgt` are true.
- EV reject flags are false.
- Local altitude/position/velocity invalid flags are false.

Manual/GCS/offboard failsafe flags may be true when RC, GCS, or offboard command stream is not ready; treat those as flight blockers until intentionally resolved.

## PX4/QGC Parameters

Known indoor mocap/no-compass values:

- `EKF2_EV_CTRL=11`.
- `EKF2_HGT_REF=3`.
- `EKF2_GPS_CTRL=0`.
- `EKF2_EV_DELAY=50`.
- `SYS_HAS_MAG=0`.
- `EKF2_MAG_TYPE=5`.

Use QGC MAVLink Inspector / Analyze Tools to watch `LOCAL_POSITION_NED`, `ODOMETRY`, `ATTITUDE`, and `HEARTBEAT`. Hand-carry the drone: QGC local position should change with physical motion. Sending commands while disarmed will not make the estimated position move unless the drone physically moves.

## RViz Hand-Carry Test

Run RViz from inside the container with the runtime sessions still alive:

```bash
sws
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```

Hand-carry the drone in the mocap volume. Confirm the `drone_3` marker moves in the correct direction and the arena/fence coordinates make sense.

For terminal checks, avoid raw high-rate echo backlog. Use one-sample polling:

```bash
while true; do clear; ros2 topic echo /drone_3/odometry_conversion/odometry --once --field pose.pose.position; sleep 0.2; done
```

If the terminal appears several seconds behind, suspect echo backlog first; NatNet at 100-120 Hz can overwhelm the terminal.

## Disarmed Command Plumbing

If no RC is available, do only disarmed command-path checks. Publish a zero velocity command:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; ros2 topic pub --once /drone_3/fmu/velocity_command geometry_msgs/msg/TwistStamped "{header: {frame_id: map}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}}"'
```

Then check PX4 trajectory setpoint:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'sws >/dev/null; ros2 topic echo /drone_3/fmu/in/trajectory_setpoint --qos-reliability best_effort --once'
```

If `/trajectory_setpoint` has a publisher but no messages, it usually means no upstream command has been received yet.

## Rosbag Recording

If Motive has a rigid body named `SoccerBall`, NatNet should publish `/SoccerBall/pose` after the NatNet bridge is restarted or refreshed.

For flight/test data, start recording before arming and stop after landing/disarming. Use the repo helper from the AirStack root:

```bash
./scripts/record_drone_flight_bag.sh
```

The recorder defaults to BV's `drone_3` identity and current `SoccerBall` rigid body. For BS or BY, pass `--drone drone_2` and the appropriate ball rigid-body name. It records the PX4/AirStack state topics used for flight review, writes a timestamped MCAP bag under `/bags` in the container, and stops with `Ctrl-C`. Docker Compose bind-mounts `/bags` to `robot/bags` on the host.

Before a flight, check topic visibility without recording:

```bash
./scripts/record_drone_flight_bag.sh --check-only
```

Useful variants:

```bash
./scripts/record_drone_flight_bag.sh --ball SoccerBall
./scripts/record_drone_flight_bag.sh --no-ball
./scripts/record_drone_flight_bag.sh --drone drone_3 --ball SoccerBall
```

After recording, the script prints the bag path and writes a latest-bag marker in the container. Inspect the latest SoccerBall bag with:

```bash
docker exec airstack-robot-desktop-1 bash -lc 'BAG=$(cat /bags/latest_drone_2_SoccerBall_bag.txt); ros2 bag info "$BAG"'
```

If using the default `/bags` output directory, bags are visible on the host under:

```text
/home/yutongw/Desktop/AirStack/robot/bags/<bag_folder_name>/
```

For ball-only tests, use standard `ros2 bag record` inside the container:

```bash
docker exec -it airstack-robot-desktop-1 bash
mkdir -p /bags
sws
ros2 bag record /SoccerBall/pose -s mcap -o /bags/soccerball_pose
```

## Flight Readiness Gate

Before allowing any real hover/takeoff recommendation, confirm all of the following in the same session:

- The physical flight area is clear, props are safe, and the drone is restrained until the final moment.
- Motive is streaming the correct `drone_3` rigid body with low latency.
- AirStack and Micro XRCE-DDS are on `ROS_DOMAIN_ID=1`, with no duplicate critical nodes.
- VOXL MicroDDS is connected to `192.168.50.139:8892`.
- QGC is connected and preflight errors are understood.
- PX4 local position and estimator EV flags are valid.
- The RViz/QGC hand-carry direction check matches the lab coordinate frame.
- Bounds and hover offset in `trajectory_commander_drone3_fmu_hover.yaml` are inside the actual mocap volume.
- RC or an agreed equivalent safety/kill path is available and tested.
- The user explicitly says they are ready for arm/takeoff guidance.

If any item fails, keep debugging at the verification level and do not move to arming.
