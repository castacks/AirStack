# Teleop

Flying one drone by hand with a gamepad, instead of from a scenario policy.

`safe_teleop` reads `sensor_msgs/Joy` and the drone's odometry, and publishes
`geometry_msgs/TwistStamped` (world-frame ENU velocity) on
`/svg/{drone}/teleop_command`. The swarm commander treats that like any other
nominal velocity, so it passes through the CBF filter unless the drone is also
listed in `cbf_exempt_drones`.

There is also `keyboard_teleop`, which maps keys to velocity directly and
needs no odometry. See the bottom of this file.

## Controls

| control | effect |
|---------|--------|
| right stick | horizontal velocity. Release and it stops. |
| left stick up/down | raises and lowers a target altitude. Release and the target stays where it is. |
| left bumper | locks the left stick, so the target cannot move |

The right stick is a direct mapping: stick position is velocity. The left
stick sets a *rate* — hold it and the target climbs, let go and it stops
climbing but keeps the height it reached. The vertical velocity sent is
computed from the gap between the target and the drone's measured altitude, so
the height is actively held rather than left to drift.

`vx` and `vy` are room-fixed, not nose-relative. The drone's heading does not
affect which way the sticks move it. Standing behind the drone keeps the
controls aligned with what you see.

## Prerequisites

This is a git worktree, and `airstack.sh` mounts the `robot/ros_ws` next to
itself. So the container has to be brought up from **this** directory, not
from `/home/kayla/AirStack`:

```bash
cd /home/kayla/airstack-xbox-teleop && AUTOLAUNCH=false ./airstack.sh up robot-desktop
```

Compose names the project after the directory, so this worktree's container is
`airstack-xbox-teleop-robot-desktop-1`. A container started from
`/home/kayla/AirStack` is called `airstack-robot-desktop-1` and mounts that
checkout instead — execing into it will not find this code.

Confirm the container sees it:

```bash
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "ls ~/AirStack/robot/ros_ws/src/svg_ground_control/svg_ground_control/safe_teleop/"
```

`~/AirStack` inside the container is the mount target and is correct there; it
is this worktree, not `/home/kayla/AirStack`.

## Test in simulation

Each block is complete — paste it as-is. Steps 1 to 5 each need their own
terminal; step 6 can go in any of them.

### 1. Isaac Sim, one drone

```bash
./airstack.sh connect isaac-sim --command=bash
```

At the container prompt:

```bash
NUM_ROBOTS=1 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```

Wait for `Spawning 1 drone(s) on ROS domain 1` and `PX4 Autolaunch: True`.

### 2. Build and start the drone interface

```bash
./airstack.sh connect robot --command=bash
```

At the container prompt:

```bash
cd ~/AirStack/robot/ros_ws && bws --packages-select svg_ground_control && sws
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 1
```

Odometry takes about 30 s to appear while EKF2 converges. Check it:

```bash
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 topic hz /drone_1/odometry_conversion/odometry"
```

### 3. Ground controller

```bash
./airstack.sh connect robot --command=bash
```

At the container prompt:

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/teleop_single.yaml
```

`teleop_single.yaml` puts `drone_1` in `teleop_drones` and leaves
`cbf_exempt_drones` empty.

### 4. joy_node, on the host where the pad is plugged in

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=1
ros2 run joy joy_node
```

It prints `Opened joystick: <name>`. `ROS_DOMAIN_ID=1` is required — the robot
container forces domain 1, and the two sides will not see each other on
different domains. The container uses host networking, so nothing else is
needed to connect them.

Check the container can see the topic:

```bash
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic hz /joy"
```

### 5. The teleop node

```bash
./airstack.sh connect robot --command=bash
```

At the container prompt:

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 run svg_ground_control safe_teleop --ros-args -p drone:=drone_1
```

### 6. Fly

```bash
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger"
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 service call /swarm_commander/start std_srvs/srv/Trigger"
```

The sticks do nothing until `start` — before that the drone holds its takeoff
position. To stop:

```bash
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 service call /swarm_commander/hold std_srvs/srv/Trigger"
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 service call /swarm_commander/land std_srvs/srv/Trigger"
```

### What to check

- right stick forward moves the drone one consistent direction across the room
- left stick up climbs; release and the altitude holds instead of sagging
- left bumper: the node logs `left stick locked`, and the left stick stops
  changing the altitude
- Ctrl-C on the teleop node publishes a zero velocity before exiting

Watch the commanded velocity while flying:

```bash
docker exec airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 topic echo /svg/drone_1/teleop_command"
```

## Parameters

`ros2 run svg_ground_control safe_teleop --ros-args -p <name>:=<value>`

| param | default | meaning |
|-------|---------|---------|
| `drone` | `drone_1` | which drone this instance drives |
| `max_speed_mps` | `1.0` | horizontal speed at full right stick |
| `climb_rate_mps` | `0.5` | how fast the target altitude moves at full left stick |
| `altitude_gain` | `1.0` | target-to-measured gap converted to vertical velocity |
| `max_climb_speed_mps` | `0.8` | cap on the vertical velocity sent |
| `min_altitude_m` | `0.3` | lower clamp on the target altitude |
| `max_altitude_m` | `2.5` | upper clamp on the target altitude |
| `deadzone` | `0.15` | stick slop ignored around center, rescaled so full deflection still reaches 1.0 |
| `joy_timeout_s` | `0.5` | zero the command if `/joy` goes quiet |
| `odometry_timeout_s` | `0.5` | zero the command if odometry goes quiet |
| `forward_axis` / `left_axis` / `climb_axis` | `4` / `3` / `1` | axis index per direction |
| `lock_button` | `4` | button that locks the left stick |
| `forward_sign` / `left_sign` / `climb_sign` | `-1.0` | flip an axis that runs backwards |

The altitude clamps are the only floor and ceiling limit in the mapping. The
CBF filter constrains drone-to-drone separation only; it has no model of the
floor, ceiling, walls, or people. The geofence (`fence_min` / `fence_max` in
the config) freezes all drones after a breach rather than braking before one.

## Safety

A teleop drone's command goes through the CBF filter like any autonomous
drone's, so a stick pushed at another drone gets projected onto the safe set
before it reaches the vehicle. Add the drone to `cbf_exempt_drones` to leave
its command uncorrected — for example when it is the moving obstacle the other
drones are supposed to dodge.

This stack bypasses `drone_safety_monitor`. PX4 failsafes and the RC kill
switch are the safety net.

Both timeouts publish zero velocity rather than holding the last command, so
an unplugged pad or lost odometry stops the drone instead of latching whatever
it was last told.

---

## Keyboard teleop

Maps keys straight to velocity. No odometry, no altitude hold.

```bash
docker exec -it airstack-xbox-teleop-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_1"
```

`w`/`s` = ±x, `a`/`d` = ±y, `r`/`f` = ±z, space = stop, `+`/`-` = speed step,
`q` = quit. It puts the TTY in raw mode, so give it its own terminal.

## Checking the controller

These run on the host, outside ROS, reading `/dev/input` directly. Use them
when the pad itself is in question, before `joy_node` is involved.

Live table of axes and buttons, for finding your axis numbers:

```bash
cd /home/kayla/airstack-xbox-teleop
PYTHONPATH=robot/ros_ws/src/svg_ground_control python3 -m svg_ground_control.safe_teleop.view
```

Push one control at a time and read the number off. Left bumper freezes the
left stick's rows so you can read a value without holding the stick.

The same mapping applied to velocity, against a stand-in drone, so the
ramp-and-hold behaviour is visible without flying anything:

```bash
cd /home/kayla/airstack-xbox-teleop
PYTHONPATH=robot/ros_ws/src/svg_ground_control python3 -m svg_ground_control.safe_teleop.velocity
```

Left column is the pad, right column is the velocity that would be published.

Analog triggers rest at full scale. Picking one as a velocity axis commands
full speed with nothing held; on the climb axis that is an immediate full-speed
descent. Both tools show triggers as a squeeze percentage.

If a device exists but is not readable, add yourself to the `input` group and
log out and back in.

## Troubleshooting

**`Package 'svg_ground_control' not found`** — the package only exists inside
the robot container. Only `joy_node` runs on the host.

**No `safe_teleop` executable** — either you are in the container started
from `/home/kayla/AirStack` (see Prerequisites), or the package has not been
rebuilt. `ros2 pkg executables svg_ground_control` should list `safe_teleop`,
`pad_view` and `velocity_preview`.

**`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`** — conda is
shadowing the system Python on the host. Use `/usr/bin/python3`, or
`conda deactivate` first.

**Nothing on `/joy`** — mismatched `ROS_DOMAIN_ID` between the host and the
container. Check `ros2 topic list` on both sides.

**Sticks move but the drone does not** — check the drone is in
`teleop_drones`, and that `/swarm_commander/start` was called.

**`odometry stale, holding zero velocity`** — the interface layer is not
running or EKF2 has not converged. Check
`ros2 topic hz /drone_1/odometry_conversion/odometry`.

**Altitude drifts down slowly** — `altitude_gain` too low for the sag, or
`max_climb_speed_mps` is clamping the correction. Raise the gain first.
