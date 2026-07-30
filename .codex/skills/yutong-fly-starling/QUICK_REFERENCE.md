# Starling Flight Quick Reference

Copy-paste commands for BV (`drone_4`) real-flight operation. Run flight commands only after mocap, PX4 estimation, QGC, RC/kill path, and the flight area have been checked.

There are three separate workflows:

1. Direct SVG goal — requires `swarm_commander`.
2. SVG scripted trajectory — requires `swarm_commander`.
3. Direct-FMU trajectory — bypasses `swarm_commander`; do not run both together.

Start bag recording first in a separate host terminal:

```bash
cd /home/yutongw/Desktop/AirStack

./scripts/record_drone_flight_bag.sh --check-only

./scripts/record_drone_flight_bag.sh
```

Enter the robot container:

```bash
docker exec -it airstack-robot-desktop-1 bash

sws
```

## 1. Direct SVG goal — requires swarm commander

Start `swarm_commander`:

```bash
ros2 launch svg_ground_control ground_control.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/swarm_real_single_goal_drone4.yaml use_mocap:=true
```

```bash
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger # take off

ros2 service call /swarm_commander/start std_srvs/srv/Trigger # enable goal tracking

ros2 topic pub --once /svg/drone_4/goal_command geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}}}" # send goal

ros2 service call /swarm_commander/hold std_srvs/srv/Trigger # hold

ros2 service call /swarm_commander/land std_srvs/srv/Trigger # land
```

## 2. SVG scripted trajectory — requires swarm commander

Start `swarm_commander`:

```bash
ros2 launch svg_ground_control ground_control.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/swarm_real_single_goal_drone4.yaml use_mocap:=true
```

Take off and start goal tracking:

```bash
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger

ros2 service call /swarm_commander/start std_srvs/srv/Trigger
```

Launch the SVG trajectory in another terminal:

```bash
ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone4_circle.yaml
```

Land through `swarm_commander`:

```bash
ros2 service call /swarm_commander/hold std_srvs/srv/Trigger

ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```

## 3. Direct-FMU trajectory — no swarm commander

Do not run `ground_control.launch.py`, `swarm_commander`, or another FMU setpoint publisher. Keep `real_interfaces` running and start only the mocap bridge:

```bash
ros2 run svg_ground_control mocap_bridge --ros-args --params-file $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/swarm_real_single_goal_drone4.yaml
```

Launch exactly one trajectory in a separate terminal while disarmed:

```bash
ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone4_fmu_hover.yaml

ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone4_fmu_square.yaml

ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone4_fmu_figure8_aggressive.yaml
```

Run the Direct-FMU command sequence from another terminal:

```bash
ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 0}" # offboard

ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 1}" # arm

ros2 service call /trajectory_commander/start std_srvs/srv/Trigger # start trajectory motion

ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 4}" # land

ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 2}" # disarm after landing
```

Keep the trajectory commander running through landing and disarm. Stop it with `Ctrl-C` only after disarm is confirmed.

## 4. PPO policy (direct FMU) — drone_soccer deploy

Do **not** run `ground_control.launch.py`, `swarm_commander`, or `trajectory_commander` at the same time.

One-time in the robot container (after mount at `/root/drone_soccer`):

```bash
pip install -e /root/drone_soccer
pip install -r /root/AirStack/robot/ros_ws/src/svg_ground_control/requirements-policy.txt
bws --packages-select svg_ground_control
```

Lab stack (same as §3 Direct-FMU): NatNet, `real_interfaces`, `mocap_bridge` with
`swarm_real_single_goal_drone4.yaml` (includes `extra_body_names: [VolleyBall]` and
`/{name}/mocap_odometry` for the ball).

Launch policy (disarmed; pass your checkpoint):

```bash
ros2 launch svg_ground_control policy_commander.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/policy_commander_drone4.yaml \
  model_path:=/path/to/ppo_final.zip
```

### Disarmed checks (before arming)

```bash
ros2 topic echo /drone_4/odometry_conversion/odometry --once --field twist.twist.linear
ros2 topic echo /VolleyBall/mocap_odometry --once --field twist.twist.linear
ros2 service call /policy_commander/start std_srvs/srv/Trigger
ros2 topic echo /policy_commander/obs --once
ros2 topic echo /drone_4/fmu/pose_command --once --field pose.position
```

Carry the drone and ball in mocap: obs and pose setpoints should update smoothly.
Call `/policy_commander/stop` to pause; the node holds the last waypoint.

### Armed sequence

Same as §3 Direct-FMU, but replace trajectory start with policy start:

```bash
ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 0}" # offboard
ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 1}" # arm
ros2 service call /policy_commander/start std_srvs/srv/Trigger
ros2 service call /policy_commander/stop std_srvs/srv/Trigger   # pause before land
ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 4}" # land
ros2 service call /drone_4/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 2}" # disarm
```
