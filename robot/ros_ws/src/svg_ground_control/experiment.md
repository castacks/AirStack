# SVG Ground Control — Guide & Command Reference

> **Maintained file.** Canonical, copy-pasteable reference for the SVG
> multi-drone CBF experiments, updated whenever the package or workflow
> changes. Every command block assumes a **fresh terminal**.

## Contents
1. [How AirStack is structured](#1-how-airstack-is-structured)
2. [How SVG ground control is structured](#2-how-svg-ground-control-is-structured)
3. [Topic & service wiring](#3-topic--service-wiring)
4. [Conventions (domain, tmux, rebuilds)](#4-conventions)
5. [Part A — Simulation](#part-a--simulation)
6. [Part B — Component tests](#part-b--component-tests-build-up-to-the-real-thing)
7. [Part C — Hybrid real+sim](#part-c--hybrid-realsim)
8. [Part D — Real Starlings + OptiTrack](#part-d--real-starlings--optitrack-mocap)
9. [RViz visualization](#rviz-visualization)
10. [Geofence](#geofence)
11. [Recording rosbags / monitoring](#recording-rosbags)
12. [Automated tests](#automated-tests)
13. [Troubleshooting](#troubleshooting)

---

## 1. How AirStack is structured

AirStack is a layered ROS 2 (Jazzy) autonomy stack that runs in Docker
containers. The pieces relevant to us:

```
┌─────────────────────────────────────────────────────────────────────┐
│ Docker containers (started by ./airstack.sh up, one bridge network)   │
│                                                                       │
│  isaac-sim ───── PX4 SITL (one per drone) ── MAVLink UDP / uXRCE-DDS  │
│   (Isaac Sim + Pegasus; physics, sensors, flight dynamics)            │
│                                                                       │
│  robot-desktop-1 ── the ROS 2 workspace (robot/ros_ws), where our     │
│                     nodes run. /AirStack is bind-mounted from host.    │
└─────────────────────────────────────────────────────────────────────┘
```

**The autonomy workspace** (`robot/ros_ws/src`) is organized in layers; the
ones we touch:

- `interface/` — talks to the flight controller. Two interchangeable plugins
  behind the same ROS API (`robot_interface_node`):
  - `mavros_interface` → PX4 over **MAVROS/MAVLink** (used for SIM/SITL).
  - `px4_interface` → PX4 over **uXRCE-DDS** (`/fmu/*` topics, used for
    HARDWARE). It converts ENU↔NED, runs the 10 Hz offboard heartbeat, and
    accepts `velocity_command` / `pose_command` / `robot_command`.
  - `odometry_conversion` — both plugins feed this; it republishes the
    canonical `…/odometry_conversion/odometry` (ENU `nav_msgs/Odometry`) and
    the `map→base_link` TF that everything downstream consumes.
- `perception/natnet_ros2` — OptiTrack Motive → ROS bridge (hardware mocap).
- `svg_ground_control/` — **our package** (this one). It sits *on top of*
  the interface layer: it reads each drone's odometry and writes each drone's
  velocity command, and otherwise ignores the stock AirStack planners.

Normally AirStack auto-launches the full autonomy stack; we run with
`AUTOLAUNCH=false` and launch only our nodes, so nothing fights us for
control.

---

## 2. How SVG ground control is structured

Five executables (`robot/ros_ws/src/svg_ground_control/svg_ground_control/`):

| Script | Node | What it does |
|---|---|---|
| `swarm_commander.py` | `swarm_commander` | **The brain.** 20 Hz loop: build each drone's *nominal* velocity (from the scenario or teleop) → run the **CBF safety filter** → publish a per-drone velocity command. Owns takeoff/start/hold/land/reset_fence services, the geofence, and the RViz markers. |
| `scenarios.py` | (library) | Nominal-velocity policies: `hover`, `goal`, `random_walk`, `random_goals`, `head_on`, `antipodal`, `squeeze`. Pure NumPy, ported from `~/drone_soccer`. |
| `cbf_filter.py` | (library) | The velocity-CBF collision filter (`filter_velocities`), a verbatim port of `drone_soccer/cbf.py`. |
| `mocap_bridge.py` | `mocap_bridge` | Hardware only: `/{name}/pose` (mocap) → `/{name}/fmu/visual_odometry_in` for the PX4 EKF. |
| `keyboard_teleop.py` | `keyboard_teleop` | Drives one teleop drone (`-p drone:=drone_3`) with the keyboard. |

**Data flow inside `swarm_commander` each tick:**

```
 per-drone odometry  ──► (add drone_position_offsets → shared world frame)
        │
        ▼
   scenario.nominal_velocity()   ── OR ──  teleop / goal-command input
        │  (per-drone desired velocity, ENU)
        ▼
   cbf_filter.filter_velocities()   ◄── sees ALL drones' world positions
        │  (collision-safe velocities; obstacle/teleop rows restored after)
        ▼
   geofence check (latch + freeze all if any drone outside the box)
        │
        ▼
   publish /{name}/<iface>/velocity_command   +   /svg/viz/markers (RViz)
```

**Roles** (`teleop_drones`, `external_drones` params): `auto` (scenario-driven,
CBF-filtered), `teleop` (operator-driven, **CBF-exempt** moving obstacle),
`external` (tracked for the CBF, never commanded — e.g. RC-flown).

**Modes** (`drone_modes` param): `sim` (commands via MAVROS `/interface/…`)
or `real` (commands via px4_interface `/fmu/…`). Mixed per run → hybrid.

**Lifecycle services** (`std_srvs/Trigger`):
`~/takeoff` (arm+offboard+ascend to the scenario's initial layout, then hold)
→ `~/start` (scenario goes live) → `~/hold` (panic freeze) →
`~/land` (descend+disarm). Plus `~/reset_fence` (clear a geofence latch).

---

## 3. Topic & service wiring

For each drone `{name}` (e.g. `drone_1`):

| Topic / service | Dir | Type | Who |
|---|---|---|---|
| `/{name}/odometry_conversion/odometry` | in | `nav_msgs/Odometry` | from interface layer → commander & RViz |
| `/{name}/interface/velocity_command` (sim) | out | `geometry_msgs/TwistStamped` | commander → MAVROS interface |
| `/{name}/fmu/velocity_command` (real) | out | `geometry_msgs/TwistStamped` | commander → px4_interface |
| `/{name}/interface/robot_command` or `/{name}/fmu/robot_command` | call | `airstack_msgs/srv/RobotCommand` | commander → arm/offboard/disarm |
| `/svg/{name}/teleop_command` | in | `geometry_msgs/TwistStamped` | keyboard_teleop → commander (teleop drones) |
| `/svg/{name}/goal_command` | in | `geometry_msgs/PoseStamped` | you → commander (`goal` scenario) |
| `/svg/{name}/speed_command` | in | `std_msgs/Float32` | you → commander (`goal` scenario) |
| `/{name}/pose` | in | `geometry_msgs/PoseStamped` | mocap → mocap_bridge (hardware) |
| `/{name}/fmu/visual_odometry_in` | out | `nav_msgs/Odometry` | mocap_bridge → px4_interface (hardware) |
| `/svg/viz/markers` | out | `visualization_msgs/MarkerArray` | commander → RViz (all drones, world frame) |
| `/swarm_commander/{takeoff,start,hold,land,reset_fence}` | call | `std_srvs/Trigger` | you → commander |

The state topic is the same for sim and real; only the command topic/service
namespace changes (`/interface/` vs `/fmu/`), which is exactly what
`drone_modes` selects per drone.

---

## 4. Conventions

**ROS domain = 1 everywhere.** The robot container auto-derives 1 from its
name; manually-started containers and the mocap PC need
`export ROS_DOMAIN_ID=1`. Check `echo $ROS_DOMAIN_ID` in every shell — a
mismatch shows up as "service unavailable" / missing topics.

**tmux** (when you `./airstack.sh connect robot` without `--command=bash`):
`Ctrl-b c` new window · `Ctrl-b n/p` or `Ctrl-b 0..9` switch · `Ctrl-b ,`
rename · `Ctrl-b %`/`"` split · `Ctrl-b x` close pane · `Ctrl-b [` scroll
(`q` exits) · `Ctrl-b d` detach (keeps running). Every new window is a fresh
shell: re-run `cd ~/AirStack/robot/ros_ws && sws`.

**Rebuild after edits.** `ros2 launch` reads the *installed* copy. After
editing any `.py`/`.yaml`/`.rviz` in the package, run `bws` (or pass
`config:=` pointing straight at the source file under `src/.../config/`).

---

# Part A — Simulation

The standard demo: 3 SITL drones, scenario from the config.

### A1. Containers (host)

```bash
cd ~/AirStack
git checkout yikuan/SVG_ground_control
# .env: COMPOSE_PROFILES="desktop,isaac-sim", AUTOLAUNCH="false", NUM_ROBOTS="1"
grep -E '^(COMPOSE_PROFILES|AUTOLAUNCH|NUM_ROBOTS)' .env
./airstack.sh up
./airstack.sh status        # robot-desktop-1 and isaac-sim Up
```

### A2. Isaac Sim — spawn drones (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect isaac-sim --command=bash
```
Inside (`PLAY_SIM_ON_START=true` is REQUIRED — PX4 SITL only launches when the
timeline plays):
```bash
NUM_ROBOTS=3 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```
Expect `Spawning 3 drone(s) on ROS domain 1` then `PX4 Autolaunch: True` per
drone. Drones spawn at x = −2, 0, +2 (this is why the sim configs set
`drone_position_offsets: [-2,0,0, 0,0,0, 2,0,0]`).

### A3. Build + per-drone MAVROS interfaces (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
echo $ROS_DOMAIN_ID                       # 1
cd ~/AirStack/robot/ros_ws && bws && sws  # bws first time / after edits
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 3
```
Verify (any other shell): `ros2 topic echo /drone_1/interface/mavros/state
--once` → `connected: true`, then `ros2 topic hz
/drone_1/odometry_conversion/odometry` (~30 Hz after EKF converges, ~30 s).

### A4. Ground controller (fresh terminal)

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py            # default (hover + teleop)
# or pick a scenario:
ros2 launch svg_ground_control ground_control.launch.py scenario:=head_on
# or the squeeze profile:
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_3drone.yaml
```

### A5. Teleop (fresh terminal, only if a drone has role teleop)

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_3
# w/s=±x  a/d=±y  r/f=up/down  space=stop  +/-=speed  q=quit
```

### A6. Fly (fresh terminal)

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger   # arm+ascend+hold
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger   # scenario live
ros2 service call /swarm_commander/hold    std_srvs/srv/Trigger   # PANIC freeze
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger   # descend+disarm
```

---

# Part B — Component tests (build up to the real thing)

Three incremental tests. Each runs in pure sim first; the same commands work
on hardware by swapping the config (Part D). All assume A1–A3 are up (sim +
interfaces). Launch the commander with the test's config in one terminal,
drive it from another.

### B1. Single-drone goal tracking

Validates: one drone flies to a goal you set, at a speed you set.

```bash
# commander terminal:
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_single.yaml

# control terminal:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
# send a goal (world ENU, metres):
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 1.4}}}"
# set the speed (m/s) live:
ros2 topic pub --once /svg/drone_1/speed_command std_msgs/msg/Float32 "{data: 0.8}"
ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```
(Goal needs the `goal` scenario — `goal_single.yaml` sets it. For a 1-drone
sim, spawn with `NUM_ROBOTS=1` in A2 and `./launch_sim_interfaces.sh 1` in A3.)

### B2. Multi-drone goal tracking (live, per-drone speed)

Validates: assign different goals to different drones while flying; the CBF
keeps them apart when paths cross.

```bash
# commander terminal:
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_tracking.yaml

# control terminal — takeoff + start, then retarget any drone any time:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_2/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_2/speed_command std_msgs/msg/Float32 "{data: 1.0}"
```

### B3. Hybrid real/sim — see [Part C](#part-c--hybrid-realsim).

---

# Part C — Hybrid real+sim

The headline feature: decide per drone whether it is **real** (commands go to
hardware) or **sim** (flies in Isaac), in one run, under one CBF, all visible
together in RViz. Example: squeeze with **real holders + simulated intruder**.

This is selected by `drone_modes` in `hybrid_squeeze.yaml`:
```yaml
drone_modes: "real,real,sim"     # drone_1,drone_2 = real ; drone_3 = sim
```
- `real` drones → commander publishes `/{name}/fmu/velocity_command` and calls
  `/{name}/fmu/robot_command` (px4_interface, Part D bringup).
- `sim` drones → `/{name}/interface/velocity_command` + `/{name}/interface/robot_command`
  (MAVROS/SITL, Part A bringup).
- The state topic is identical, so **one CBF sees all drones**.
- `drone_position_offsets`: real drones are mocap-anchored (offset 0); the sim
  drone reports in its SITL local frame, so set ITS offset to its Isaac spawn.

### Bring-up (mixed)

You run BOTH interface stacks side by side, then the commander once:

```bash
# real drones (Part D B1–B2): uXRCE agent + px4_interface for drone_1,drone_2
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1,drone_2
# sim drone (Part A A2–A3): Isaac SITL for drone_3 + its MAVROS interface
ROBOT_NAME=drone_3 FCU_URL='udp://:14543@<sim_ip>:14583' TGT_SYSTEM=4 \
  ros2 launch svg_ground_control sim_drone_interface.launch.xml drone_name:=drone_3
# (or just run launch_sim_interfaces.sh for all and ignore the holders' SITL)

# one commander for all three:
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/hybrid_squeeze.yaml
```

"See all three in the sim" = open RViz (below). The commander publishes every
drone's offset-corrected WORLD position to `/svg/viz/markers`, so real
(red) and simulated (cyan) drones appear together in one `map` frame, with
their safety spheres and the geofence box.

> Pure-sim dry run of the routing: the functional test
> `test/functional_hybrid_test.py` fakes the real+sim drones on their
> respective topics and asserts each drone's commands land on the correct
> namespace — run it before trusting a real flight.

---

# Part D — Real Starlings + OptiTrack mocap

### D0. One-time setup

Per drone (QGC / VOXL): `uxrce_dds_client start -n drone_1` (own name, ground
PC IP, port 8888) with PX4 param `UXRCE_DDS_DOM_ID = 1`; `EKF2_EV_CTRL` to
fuse external vision (GPS off indoors); RC kill switch + offboard-loss
failsafe. Motive: one rigid body per drone named `drone_1`, `drone_2`, …;
enable NatNet streaming. NatNet SDK (once): `./robot/ros_ws/src/perception/
natnet_ros2/scripts/download-natnet-sdk.sh`, then set `server_ip`, `body_id:
-1` in `natnet_ros2/config/natnet_config.yaml`.

### D1. Host-network container + uXRCE agent (fresh terminal)

```bash
cd ~/AirStack
docker run --rm -it --network host --name svg_ground \
  -v ~/AirStack:/home/robot/AirStack \
  airlab-docker.andrew.cmu.edu/airstack/airstack:v0.18.0_robot-x86-64_dev bash
# inside:
export ROS_DOMAIN_ID=1
MicroXRCEAgent udp4 -p 8888
```

### D2. ALL per-drone interfaces — one command (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && bws && sws
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1,drone_2,drone_3
```

### D3. NatNet bridge (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && sws
ros2 launch natnet_ros2 natnet_ros2.launch.py
ros2 topic list | grep -i optitrack     # note the per-body pose path
```
Set `mocap_topic_template` in `swarm_real.yaml` to that path with `{name}`.

### D4. Commander + mocap bridge (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(pwd)/src/svg_ground_control/config/swarm_real.yaml use_mocap:=true
```

### D5. Preflight + fly (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && sws
ros2 topic hz   /drone_1/pose                                  # mocap arriving?
ros2 topic echo /drone_1/odometry_conversion/odometry --once   # tracks hand movement?
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger
```
First hardware flight: ONE drone (`drone_names: ["drone_1"]`, scenario hover),
thumb on the kill switch. Then two. Then the demo.

---

## RViz visualization

The commander publishes all drones' **world** positions (offset-corrected, so
real + simulated share one frame) as a `MarkerArray` on `/svg/viz/markers`:
solid sphere per drone (red=real, cyan=sim, yellow=teleop, gray=external,
orange=frozen-on-breach), translucent safety sphere (2r), name/mode/role
label, goal points, and the geofence box.

```bash
# any shell on domain 1, workspace sourced (needs an X display):
cd ~/AirStack/robot/ros_ws && sws
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```
The config sets fixed frame `map` and adds the MarkerArray display. If you
open a bare `rviz2`: set Fixed Frame = `map`, Add → By topic →
`/svg/viz/markers`. This is the unified "see all drones" view for hybrid runs.

---

## Geofence

A safety latch in `swarm_commander`. If any **airborne, holding/active** drone
leaves the box `[fence_min, fence_max]` (world ENU), the commander latches a
breach: every drone freezes at its current position, the scenario stops, and
`start` is refused until you call `~/reset_fence`. Climb-out and landing pass
through the floor on purpose and are exempt from detection.

Config (per profile):
```yaml
fence_enabled: true
fence_min: [-2.5, -2.5, 0.3]    # x,y,z lower limits (world ENU, m)
fence_max: [ 2.5,  2.5, 2.5]    # x,y,z upper limits
```
Recover:
```bash
ros2 service call /swarm_commander/reset_fence std_srvs/srv/Trigger
```
This is a freeze-in-place, not a motor cutoff — the RC kill switch remains the
true cutoff. The fence box is drawn in RViz (green normally, red when latched).

---

## Recording rosbags

Record INTO the mounted workspace (`~/AirStack/robot/ros_ws/...`); paths
outside the bind mounts stay trapped in the container. Ctrl-C stops AND writes
`metadata.yaml`.

```bash
ros2 bag record -o ~/AirStack/robot/ros_ws/bags/run_$(date +%H%M%S) \
  /drone_1/odometry_conversion/odometry /drone_2/odometry_conversion/odometry \
  /drone_3/odometry_conversion/odometry \
  /drone_1/interface/velocity_command /drone_2/interface/velocity_command \
  /drone_3/interface/velocity_command
ros2 bag info <bag_dir>          # sanity
```
Live position of one drone: `ros2 topic echo
/drone_1/odometry_conversion/odometry --field pose.pose.position`. To rescue a
bag from a non-mounted path: `docker cp
airstack-robot-desktop-1:/root/AirStack/<path> ~/AirStack/...`.

---

## Automated tests

```bash
cd ~/AirStack/robot/ros_ws/src/svg_ground_control
# unit (pure numpy, no ROS):
python3 -m pytest test/test_cbf.py test/test_scenarios.py -q

# closed-loop functional (fake drones; launch the matching commander first):
#   ground_control.launch.py config:=<share>/config/<cfg>.yaml   then:
python3 test/functional_single_goal_test.py     # cfg: goal_single.yaml
python3 test/functional_multi_goal_test.py      # cfg: goal_tracking.yaml
python3 test/functional_hybrid_test.py          # cfg: hybrid_squeeze.yaml
python3 test/functional_squeeze_test.py         # cfg: squeeze_3drone.yaml
python3 test/functional_squeeze_lag_test.py     # cfg: squeeze_3drone.yaml (PX4-lag model)
python3 test/functional_fence_test.py           # cfg: goal_single.yaml
```
The fake-drone tests integrate the commander's velocity commands and publish
odometry in per-drone local frames, so they also exercise the offset
correction and (for hybrid) the command routing. Give the commander ~5 s to
come up before starting a test.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `[ERROR] Docker daemon is not running` but it is | user not in `docker` group: `sudo usermod -aG docker $USER`, then a real logout/login (lock screen doesn't count), or `newgrp docker` per shell |
| `airstack connect` shows no prompt | it attaches to the container tmux; `Ctrl-b c` new window, `Ctrl-b d` detach — or use `--command=bash` |
| 3 robot containers appear | `.env NUM_ROBOTS` also scales container replicas; keep it `"1"`, pass drone count inline to the sim script |
| service `waiting for service to become available…` forever | `ROS_DOMAIN_ID` mismatch between shells; also `ros2 daemon stop` |
| MAVROS `connected: false`, no odometry | PX4 SITL not launched: Isaac timeline not playing (`PLAY_SIM_ON_START=true`, or press Play) |
| takeoff returns success=false right after launch | commander hasn't received odometry yet — wait a few seconds and retry |
| `The parameter 'X' is not initialized` | empty YAML list can't override a typed param — `teleop_drones`/`external_drones`/`drone_modes` are comma-separated STRINGS (`""` = none) |
| `start` says "not all drones holding yet" | drones still converging to takeoff targets; retry after a few seconds |
| `start` says "geofence breached" | a drone left the box; `ros2 service call /swarm_commander/reset_fence std_srvs/srv/Trigger` after recovering |
| drones fly right *shapes* in wrong *places*; intruder misses the gap | per-drone PX4 local origins: `drone_position_offsets` must equal the sim spawn positions (`x = 2*(i-1) - (N-1)` → `[-2,0,0, 0,0,0, 2,0,0]` for 3). Zeros only for mocap-anchored hardware |
| hybrid: a "real" drone never moves in sim | nothing is consuming `/{name}/fmu/velocity_command` — in pure sim only MAVROS (`/interface/`) exists; real-mode drones need px4_interface (Part D) or are validated via `functional_hybrid_test.py` |
| RViz empty | Fixed Frame must be `map`; check `ros2 topic hz /svg/viz/markers`; needs an X display (`echo $DISPLAY`) |
