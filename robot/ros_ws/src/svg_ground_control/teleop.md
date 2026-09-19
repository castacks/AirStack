# Teleop

Flying a drone by hand with a gamepad, instead of from a scenario policy.

`safe_teleop` reads `sensor_msgs/Joy` and the drone's odometry, and publishes
`geometry_msgs/TwistStamped` (world-frame ENU velocity) on
`/svg/{drone}/teleop_command`. The swarm commander treats that like any other
nominal velocity, so it passes through the CBF filter unless the drone is also
listed in `cbf_exempt_drones`.

```
pad -> joy_node -> /joy -> safe_teleop -> /svg/{drone}/teleop_command -> swarm_commander -> PX4
```

There is also `keyboard_teleop`, which maps keys to velocity directly and
needs no odometry. See the bottom of this file.

## Controls

| control | effect |
|---------|--------|
| right stick | horizontal velocity. Release and it stops. |
| left stick up/down | raises and lowers a target altitude. Release and the target stays where it is. |
| left stick left/right | yaw rate — turns the drone in place. Release and it stops turning. |
| left bumper | locks the left stick, so neither altitude nor yaw can move |

The right stick is a direct mapping: stick position is velocity. The left
stick sets a *rate* — hold it and the target climbs, let go and it stops
climbing but keeps the height it reached. The vertical velocity sent is
computed from the gap between the target and the drone's measured altitude, so
the height is actively held rather than left to drift.

Yaw bypasses the CBF: the filter constrains drone-to-drone distance, which
turning in place cannot change.

`vx` and `vy` are room-fixed, not nose-relative. The drone's heading does not
affect which way the sticks move it.

## One-command bring-up

`scripts/svg_teleop.sh` runs every step below for you — Isaac, interfaces,
ground controller, `joy_node`, `safe_teleop` and RViz — each in its own tmux
session inside the containers, so none of them needs a terminal.

Pick one of the three experiments. They are described under
[Experiments](#experiments):

```bash
cd ~/AirStack/robot/ros_ws/src/svg_ground_control/scripts

./svg_teleop.sh solo       # 1 drone, alone — does the pad move it
./svg_teleop.sh squeeze    # 3 drones — you fly the intruder, holders yield
./svg_teleop.sh hover      # 3 drones — you fly at them, the CBF pushes you back
```

Add `--headless` to any of them to skip the Isaac viewport. There is also
`./svg_teleop.sh real` — one **real** drone, no sim; read
[Real drone](#real-drone) before running it.

Then fly it:

```bash
./svg_teleop.sh takeoff
./svg_teleop.sh start      # sticks do nothing until this
./svg_teleop.sh land
```

Other commands:

```bash
./svg_teleop.sh status         # what is running, odometry rates, drone roles
./svg_teleop.sh logs isaac     # isaac | iface | commander | teleop | joy | rviz
./svg_teleop.sh hold           # stop where you are, mid-flight
./svg_teleop.sh reset-fence    # clear a geofence breach
./svg_teleop.sh stop           # kill everything, leave the containers up
./svg_teleop.sh --help
```

To watch one of the processes live:

```bash
docker exec -it airstack-robot-desktop-1 tmux attach -t commander
```

## What to check

**`solo`** — the teleop mapping itself:

- right stick forward moves the drone one consistent direction
- left stick up climbs; release and the altitude holds instead of sagging
- left bumper: the node logs `left stick locked`, and the altitude stops moving
- Ctrl-C on the teleop node publishes a zero velocity before exiting

**`squeeze`** — the holders yield to you:

- fly at the gap and the holders part as you close, settling back onto their
  posts once you are through
- they never let your center inside `2 * cbf_safety_radius_m` (1.1 m) of
  either of them
- the holders do the yielding, not you — drone_3 is CBF-exempt, so its command
  goes out uncorrected
- nothing stops you ramming a holder; see [Safety](#safety)

**`hover`** — the filter corrects you:

- hold the stick straight at drone_1 and you stop short rather than reaching it
- drone_1 does not move out of your way

The commander logs `CBF active on: <drones> (residual ...)` whenever the
filter is correcting someone.

## Real drone

The teleop node is transport-agnostic — the commander routes each drone's
(CBF-filtered) command to MAVROS in sim or px4_interface (uXRCE-DDS) on
hardware, and the sticks neither know nor care. One command brings up the
whole real single-drone stack — agent, px4_interface, NatNet, mocap bridge,
commander (`config/teleop_real.yaml`), joy, teleop, RViz:

```bash
./svg_teleop.sh real
```

It assumes experiment.md Part B is already done on this rig: VOXL provisioned
(`voxl_setup_real_drone.sh`), EKF2 external-vision params set and verified
(B4b, including the frame hand-check), and `natnet_ros2` built. The script
gates on the agent session, `/drone_1/pose` from mocap, and odometry out of
the EKF before starting teleop, and tells you which stage to debug when a gate
fails.

### Ground check — before the first takeoff

Everything up to the sticks is checkable on the ground. With the stack up and
**takeoff not called**:

```bash
./svg_teleop.sh monitor                          # sticks vs published command
ros2 topic echo /svg/drone_1/teleop_command      # or raw, in the container
```

- right stick: `vx`/`vy` follow the sticks, correct directions, zero at rest
- carry the drone up and down by hand: the altitude target seeds from the
  measured height, and `vz` pushes back toward the target
- the drone's marker tracks in RViz as you carry it (the B5 preflight)
- unplug the pad: horizontal zeros, the altitude hold stays — that is the
  intended dead-pad behavior (see [Safety](#safety))

**Yaw cannot be ground-checked.** The real path negates the yaw rate where the
sim path does not (`px4_interface` vs `mavros_interface`), so sim flights
prove every axis except yaw sign. First flight: test yaw slowly, low, with a
thumb on the RC kill switch. If it turns the wrong way, flip `yaw_sign`.

Known limitation: the commander's takeoff is fixed-time staging (request
control, arm, ascend) and does not confirm against `vehicle_status` — if PX4
refuses to arm (EKF not ready, preflight failure), the drone simply stays put;
check the drone's own status via QGC or `px4-listener`.

## Hardware and hybrid

Multi-drone hardware configs take teleop the same way, but the bring-up
differs (per-drone uXRCE agents on separate ports, more interfaces). Read each
config's own header before running.

| config | setup |
|--------|-------|
| `teleop_real.yaml` | ONE real drone, teleop — what `./svg_teleop.sh real` uses |
| `hybrid_squeeze.yaml` | real holders + sim intruder |
| `squeeze_rc_intruder.yaml` | all real, intruder on RC — `external_drones`, not teleop |
| `swarm_real.yaml` | three real drones, hover |
| `goal_single.yaml` / `goal_tracking.yaml` | real, goal-tracking |

`squeeze_rc_intruder.yaml` cannot be used with teleop — its intruder is
`external_drones`, flown on its own RC link and merely tracked. A drone cannot
be both external and teleop; the commander rejects that.

Any config can take a hand-flown drone by adding `teleop_drones:=<name>` to
the launch line, whatever the YAML says.

## Reading the pad

Two live tables, same layout. Both need an interactive terminal.

**The device**, on the host, no ROS involved — use it when the pad itself is
in question:

```bash
cd ~/AirStack
PYTHONPATH=robot/ros_ws/src/svg_ground_control python3 -m svg_ground_control.safe_teleop.view
```

**The topic**, in the container, needs `joy_node` running — use it when the
pad is fine but the drone is not moving:

```bash
docker exec -it airstack-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 run svg_ground_control joy_topic_view"
```

Together they separate "the pad is wrong" from "the messages are not
arriving". `ros2 topic hz /joy` is a rate meter only — it never shows axis
values, and prints nothing unless the sticks are moving, because `joy_node`
publishes on change.

`joy_node` **negates every axis**, so a stick pushed right reads positive on
the device and negative on `/joy`. That is why the `*_sign` parameters exist —
after the axis-convention fix only `left_sign` defaults to `-1.0`; the rest
are `+1.0` (the [Axis signs](#axis-signs) table below is the authority). It
also applies its own deadzone, and reports the triggers resting at `+1.0`
rather than the device's `-1.0`. The two tables will disagree on sign for the
same stick position, and that is correct.

The velocity preview, showing the mapping against a stand-in drone without
flying anything:

```bash
cd ~/AirStack
PYTHONPATH=robot/ros_ws/src/svg_ground_control python3 -m svg_ground_control.safe_teleop.velocity
```

Analog triggers rest at full scale. Picking one as a velocity axis commands
full speed with nothing held; on the climb axis that is an immediate
full-speed descent. Both tools show triggers as a squeeze percentage.

If a device exists but is not readable, add yourself to the `input` group and
log out and back in.

### joy_node on the host

Do **not** run `joy_node` on the host unless the host has the same ROS distro
as the container (Jazzy). A Humble host talking to a Jazzy container connects
at the DDS level but cannot deserialize the messages — you get a stream of
`sequence size exceeds remaining buffer` and `/joy` never arrives. Ubuntu
22.04 hosts only have Humble, so use the container.

## Parameters

`ros2 run svg_ground_control safe_teleop --ros-args -p <name>:=<value>`

The node's own defaults are conservative. `svg_teleop.sh` passes
`max_speed_mps:=2.0` for the sim experiments (with matching 2.0
`teleop_max_speed_mps` / `cbf_max_speed_mps` in the sim configs — the
commander clamps to the smallest of the three, so all must agree). The real
mode stays at `teleop_real.yaml`'s slower caps.

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
| `yaw_rate_rad_s` | `1.0` | yaw rate at full left-stick deflection |
| `forward_axis` / `left_axis` / `climb_axis` / `yaw_axis` | `4` / `3` / `1` / `0` | axis index per direction |
| `lock_button` | `4` | button that locks the left stick |
| `forward_sign` / `climb_sign` / `yaw_sign` | `1.0` | flip an axis that runs backwards |
| `left_sign` | `-1.0` | as above |

### Axis signs

| control | axis | sign |
|---------|------|------|
| right stick up = forward | 4 | `+1.0` |
| right stick right = right | 3 | `-1.0` |
| left stick up = climb | 1 | `+1.0` |
| left stick left = yaw | 0 | `+1.0` |

The signs apply to `/joy`, not to the raw device. `joy_node` negates every
axis, so `/dev/input` reports the opposite sign to the topic for the same stick
position. The device-side `view` tool and the topic-side `joy_topic_view` will
disagree for that reason.

If a direction is backwards after a pad or driver change, flip that one sign.
`./svg_teleop.sh monitor` shows raw axis, signed value and published velocity
in one view.

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

The squeeze intruder is exempt (`squeeze_intruder_cbf_exempt`, default true).
That means **nothing stops you ramming a holder** — the holders dodge, but
corner one against the geofence and contact is possible. The same warning
applies on hardware; see `squeeze_rc_intruder.yaml`.

This stack bypasses `drone_safety_monitor`. PX4 failsafes and the RC kill
switch are the safety net.

The two timeouts fail differently, on purpose. A stale pad (unplugged, dead
battery) zeros horizontal and yaw but **keeps the altitude hold**, so the
drone parks in the air at its held height rather than latching whatever it was
last told — bring it down with `land` (or `hold`), or the RC kill switch.
Stale odometry zeros everything including the vertical command — without a
height measurement the altitude hold would be flying blind — and drops the
altitude target, which re-seeds from the measured height when odometry
returns. Neither case is a position hold: a drone commanded zero velocity
stays roughly put but can drift.

## Keyboard teleop

Maps keys straight to velocity. No odometry, no altitude hold.

```bash
docker exec -it airstack-robot-desktop-1 bash -lc "cd ~/AirStack/robot/ros_ws && sws && ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_1"
```

`w`/`s` = ±x, `a`/`d` = ±y, `r`/`f` = ±z, space = stop, `+`/`-` = speed step,
`q` = quit. It puts the TTY in raw mode, so give it its own terminal.
