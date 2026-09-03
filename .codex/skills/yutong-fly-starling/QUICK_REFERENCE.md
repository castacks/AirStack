# Starling Flight Quick Reference

Copy-paste commands for BV (`drone_3`) real-flight operation. Run flight commands only after mocap, PX4 estimation, QGC, RC/kill path, and the flight area have been checked.

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

Or bring up the complete policy-support stack in one host tmux session:

```bash
./scripts/start_drone_soccer_lab_tmux.sh
```

This starts the idle robot container and Micro XRCE-DDS agent, then creates
tmux **inside the robot container**. Its `runtime` window runs NatNet,
`real_interfaces`, and `mocap_bridge`; its `policy` window prepopulates the
policy launch/start/stop and goal-topic commands without executing them; and
its `shell` window is sourced for additional ROS commands. Detach with
`Ctrl-b d`; reattach from the host with
`docker exec -it airstack-robot-desktop-1 tmux attach-session -t drone_soccer_lab`.
Stop the session, DDS agent, and robot container with
`./scripts/start_drone_soccer_lab_tmux.sh --stop`.
The launcher checks `stable_baselines3` and `gymnasium` first and installs the
pinned policy requirements automatically when a recreated image lacks them.

Enter the robot container:

```bash
docker exec -it airstack-robot-desktop-1 bash

sws
```

## 1. Direct SVG goal — requires swarm commander

Start `swarm_commander`:
position
```bash
ros2 launch svg_ground_control ground_control.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/swarm_real_single_goal_drone3.yaml use_mocap:=true
```

```bash
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger # take off

ros2 service call /swarm_commander/start std_srvs/srv/Trigger # enable goal tracking

ros2 topic pub --once /svg/drone_3/goal_command geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}}}" # send goal

ros2 service call /swarm_commander/hold std_srvs/srv/Trigger # hold

ros2 service call /swarm_commander/land std_srvs/srv/Trigger # land
```

## 2. SVG scripted trajectory — requires swarm commander

Start `swarm_commander`:

```bash
ros2 launch svg_ground_control ground_control.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/swarm_real_single_goal_drone3.yaml use_mocap:=true
```

Take off and start goal tracking:

```bash
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger

ros2 service call /swarm_commander/start std_srvs/srv/Trigger
```

Launch the SVG trajectory in another terminal:

```bash
ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_circle.yaml
```

Land through `swarm_commander`:

```bash
ros2 service call /swarm_commander/hold std_srvs/srv/Trigger

ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```

## 3. Direct-FMU trajectory — no swarm commander

Do not run `ground_control.launch.py`, `swarm_commander`, or another FMU setpoint publisher. Keep `real_interfaces` running and start only the mocap bridge:

```bash
ros2 run svg_ground_control mocap_bridge --ros-args --params-file $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/swarm_real_single_goal_drone3.yaml
```

Launch exactly one trajectory in a separate terminal while disarmed:

```bash
ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_fmu_hover.yaml

ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_fmu_square.yaml

ros2 launch svg_ground_control trajectory_commander.launch.py config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_fmu_figure8_aggressive.yaml
```

Run the Direct-FMU command sequence from another terminal:

```bash
ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 0}" # offboard

ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 1}" # arm

ros2 service call /trajectory_commander/start std_srvs/srv/Trigger # start trajectory motion

ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 4}" # land

ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 2}" # disarm after landing
```

Keep the trajectory commander running through landing and disarm. Stop it with `Ctrl-C` only after disarm is confirmed.

## 4. PPO policy (direct FMU) — drone_soccer deploy

Do **not** run `ground_control.launch.py`, `swarm_commander`, or `trajectory_commander` at the same time.

When using `start_drone_soccer_lab_tmux.sh`, no separate dependency step is
needed: the launcher verifies and, if necessary, installs the pinned policy
requirements before creating or attaching to tmux. For a manual bringup, run:

```bash
pip install --break-system-packages -r /root/AirStack/robot/ros_ws/src/svg_ground_control/requirements-policy.txt
bws --packages-select svg_ground_control
```

The read-only `/root/drone_soccer` mount is already included in the robot
container's `PYTHONPATH`; do not install it editable. `bws`/`colcon` alone
does not install `stable_baselines3` or the package's `install_requires`.

Lab stack (same as §3 Direct-FMU): NatNet, `real_interfaces`, `mocap_bridge` with
`swarm_real_single_goal_drone3.yaml` (includes `extra_body_names: [SoccerBall]` and
`/{name}/mocap_odometry` for the ball). The production ball filter is Kalman
with position noise `0.004 m`, acceleration noise `0.1 m/s²`, and initial
velocity noise `1.7 m/s`; policy drone velocity remains PX4 EKF odometry.

Launch policy (disarmed; pass your checkpoint):

```bash
ros2 launch svg_ground_control policy_commander.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/policy_commander_drone3.yaml \
  model_path:=/root/drone_soccer/drone_soccer/saved_policies/safe.zip
```

### Disarmed checks (before arming)

```bash
ros2 topic echo /drone_3/odometry_conversion/odometry --once --field twist.twist.linear
ros2 topic echo /SoccerBall/mocap_odometry --once --field twist.twist.linear
ros2 service call /policy_commander/start std_srvs/srv/Trigger
ros2 topic echo /policy_commander/obs --once
ros2 topic echo /policy_commander/goal --once
ros2 topic echo /drone_3/fmu/in/trajectory_setpoint --once --field position
ros2 topic echo /drone_3/fmu/in/offboard_control_mode --once
```

Carry the drone and ball in mocap: observations and trajectory setpoints should
update smoothly. For the initial planar test, every trajectory setpoint must
have NED `position[2] == -1.0`, corresponding to absolute ENU altitude `1.0 m`.
Policy ENU X/Y waypoints remain unconstrained while the drone is more than
`geofence_buffer_m` (default `0.5 m`) inside every horizontal fence face. At
or inside that buffer, or when already outside the fence, X/Y snaps to the
configured bounds. ENU Z is always clamped to its configured floor/ceiling.
Call `/policy_commander/stop` to pause; the node holds the last waypoint.

The default `policy_commander_drone3.yaml` uses arrival-triggered random goals.
It samples one XY goal at a time inside the ENU rectangle configured by
`goal_spawn_min_xy` and `goal_spawn_max_xy`, then resamples when the drone is
within `goal_arrival_radius`. The current two-element XY goal is continuously
published on `/policy_commander/goal`; `record_drone_flight_bag.sh` includes
that topic even when the policy node starts after rosbag.

### Armed sequence

Same as §3 Direct-FMU, but replace trajectory start with policy start:

```bash
ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 0}" # offboard
ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 1}" # arm
ros2 service call /policy_commander/start std_srvs/srv/Trigger
ros2 service call /policy_commander/stop std_srvs/srv/Trigger   # pause before land
ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 4}" # land
ros2 service call /drone_3/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 2}" # disarm
```
