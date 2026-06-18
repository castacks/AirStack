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
6. [Part B — Bring in a real drone (connect + verify)](#part-b--bring-in-a-real-drone-connect--verify)
7. [Part C — Tasks: any drone in any mode](#part-c--tasks-any-drone-in-any-mode)
8. [Part D — Real hardware: first flight & reference](#part-d--real-hardware-first-flight--reference)
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
        │  (collision-safe velocities; cbf_exempt rows restored after)
        ▼
   geofence check (latch + freeze all if any drone outside the box)
        │
        ▼
   publish /{name}/<iface>/velocity_command   +   /svg/viz/markers (RViz)
```

**Three independent per-drone axes** — set any combination in *any* task
config (see [Part C](#part-c--tasks-any-drone-in-any-mode)):

- **Mode** (`drone_modes`): `sim` (commands via MAVROS `/interface/…`) or
  `real` (commands via px4_interface `/fmu/…`). A `real` drone also shows up in
  the Isaac viewport at its live pose via an avatar (Part A `DRONE_MODES`).
  Mixed per run → hybrid.
- **Role** (`teleop_drones`, `external_drones`): `auto` (scenario-driven),
  `teleop` (operator-driven via a teleop topic), `external` (tracked for the
  CBF but never commanded — e.g. RC-flown). Unlisted = `auto`.
- **CBF-exempt** (`cbf_exempt_drones`): the filter still *sees* these drones
  (so everyone else avoids them) but leaves their *own* command uncorrected —
  they play the moving obstacle. Independent of role: a policy-driven (`auto`)
  drone or a `teleop` drone can be exempt. Teleop is **not** auto-exempt; list
  it here if you want its manual commands left unfiltered. (The `squeeze`
  scenario additionally self-designates its intruder via
  `squeeze_intruder_cbf_exempt`; the two union.)

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
timeline plays; `ISAAC_SIM_HEADLESS=true` is REQUIRED unless you specifically
need the Isaac window — see note below):
```bash
NUM_ROBOTS=3 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```
Expect `Spawning 3 drone(s) on ROS domain 1` then `PX4 Autolaunch: True` per
drone. Drones spawn at x = −2, 0, +2 (this is why the sim configs set
`drone_position_offsets: [-2,0,0, 0,0,0, 2,0,0]`).

> **Real-drone avatars (`DRONE_MODES`).** For a hybrid run, tell the sim which
> drones are real so it spawns a SITL body only for the sim ones and a
> visual-only **avatar** for each real one — the avatar is teleported every
> step to that drone's `…/odometry_conversion/odometry`, so a real (mocap)
> drone appears in the Isaac viewport at its live pose. Add
> `DRONE_MODES="real,real,sim"` (length `NUM_ROBOTS`, matching the commander's
> `drone_modes`) to the launch, and run with a **GUI** viewport
> (`ISAAC_SIM_HEADLESS=false`) so you can see it. Used by the hybrid squeeze in
> [Part C](#part-c--tasks-any-drone-in-any-mode). The avatar's rclpy node joins
> the drones' domain automatically (it sets `ROS_DOMAIN_ID=SVG_DOMAIN_ID`); for
> true hardware the Isaac container must also be able to reach the real drones'
> DDS traffic (host networking / discovery server) — see Troubleshooting.

> **Run headless.** For SVG ground control you never need the Isaac viewport —
> physics, PX4 SITL, and the ROS topics all run headless, and you watch the
> drones in RViz (`/svg/viz/markers`) instead. The launcher defaults to GUI
> mode (`ISAAC_SIM_HEADLESS` unset → `false`), which opens a viewport window;
> running headless avoids the viewport entirely and is the right default. Pass
> `ISAAC_SIM_HEADLESS=true`. (When launched via `./airstack.sh up` with
> `AUTOLAUNCH=true`, set `ISAAC_SIM_HEADLESS=true` in `.env` instead.)
>
> ⚠️ **Headless does NOT fix an RTX renderer segfault.** If Isaac crashes with a
> `Segmentation fault` whose backtrace is in `librtx.scenedb.plugin.so` /
> `libcarb.scenerenderer-rtx.plugin.so` at `carbOnPluginStartup` — and it still
> crashes headless, and even a bare empty `SimulationApp({"headless":True})`
> crashes the same way — that is a **GPU driver ↔ Isaac Sim version
> incompatibility**, not an AirStack bug. Seen on RTX 5080 / Blackwell with
> NVIDIA driver 595.x and Isaac Sim 5.1.0: the app boots to `app ready`, then
> the RTX renderer faults on the first frame. Clearing the shader cache does
> not help. Fix = run a driver Isaac Sim 5.1 supports (Linux **580.65.06**, or
> **591.74** which a Blackwell user confirmed works — driver **595.x crashes**),
> or move to a newer Isaac Sim release. See Troubleshooting below.

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

# Part B — Bring in a real drone (connect + verify)

Get one real drone talking to the stack and confirm it is tracked — the
hardware analogue of Part A's sim bring-up, with **no flight**. Once this
passes, a real drone is just `drone_modes: "...real..."` in any
[Part C](#part-c--tasks-any-drone-in-any-mode) task. Do it per session.

The state topic is identical to sim (`…/odometry_conversion/odometry`); only
the source changes (mocap+px4_interface instead of SITL+MAVROS).

### B1. One-time per-drone setup (PX4 + Motive)

Per drone (QGC / VOXL): `uxrce_dds_client start -n drone_1` (own name, ground
PC IP, port 8888) with PX4 param `UXRCE_DDS_DOM_ID = 1`; `EKF2_EV_CTRL` to
fuse external vision (GPS off indoors); RC kill switch + offboard-loss
failsafe. Motive: one rigid body per drone named `drone_1`, `drone_2`, …;
enable NatNet streaming. NatNet SDK (once): `./robot/ros_ws/src/perception/
natnet_ros2/scripts/download-natnet-sdk.sh`, then set `server_ip`, `body_id:
-1` in `natnet_ros2/config/natnet_config.yaml`.

### B2. Host-network container + uXRCE agent (fresh terminal)

```bash
cd ~/AirStack
docker run --rm -it --network host --name svg_ground \
  -v ~/AirStack:/home/robot/AirStack \
  airlab-docker.andrew.cmu.edu/airstack/airstack:v0.18.0_robot-x86-64_dev bash
# inside:
export ROS_DOMAIN_ID=1
MicroXRCEAgent udp4 -p 8888
```

### B3. Per-drone px4_interface (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && bws && sws
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1   # add ,drone_2,...
```

### B4. NatNet mocap — launch + UNIT-TEST (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && sws
ros2 launch natnet_ros2 natnet_ros2.launch.py
ros2 topic list | grep -i optitrack          # note the per-body pose path
```
Then verify mocap is actually streaming (this is the unit-test — do NOT skip):
```bash
ros2 topic hz   /drone_1/pose                 # ~120 Hz (or your Motive rate)
ros2 topic echo /drone_1/pose --once          # sane x,y,z = where the drone sits
# move the drone by hand: position must change smoothly, no NaNs / jumps
```
- No topic / 0 Hz → Motive not streaming, wrong `server_ip`, or the rigid body
  isn't named `drone_1`. Fix before going further.
- Set `mocap_topic_template` in `swarm_real.yaml` to the path from `grep`
  (with `{name}`), so `mocap_bridge` forwards mocap → PX4 visual odometry.

### B5. See the drone in RViz (no flight)

Bring up the commander **without taking off** + the mocap bridge, then watch
the drone's marker track as you carry it. Confirms mocap → odometry → world
before anything arms.

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(pwd)/src/svg_ground_control/config/swarm_real.yaml use_mocap:=true
# in another shell — does odometry track your hand?
ros2 topic echo /drone_1/odometry_conversion/odometry --once
```
Open RViz (see [RViz visualization](#rviz-visualization)) and move the drone by
hand: its **red** sphere should follow on `/svg/viz/markers`. (To also see it in
the Isaac 3D viewport, launch Isaac with `DRONE_MODES` set — see Part A and the
flagship in C4.) Do **not** call `takeoff` here — this is preflight only.

---

# Part C — Tasks: any drone in any mode

One framework, not "sim tests vs hardware tests". Every task is a config; each
config exposes the **three per-drone axes** (see §2) and you pick them freely:

```yaml
drone_modes:        "sim,sim,sim"   # per drone: sim -> SITL/MAVROS, real -> hardware/fmu
teleop_drones:      ""              # operator-driven (else scenario-driven)
external_drones:    ""              # tracked by CBF, never commanded
cbf_exempt_drones:  ""              # CBF won't correct these (still obstacles)
```

"Pure sim", "all real", and "hybrid" are just different `drone_modes` vectors on
the **same** task. To make a drone real: set its slot to `real` (commands route
to `/fmu/…`; it must be connected per [Part B](#part-b--bring-in-a-real-drone-connect--verify),
and Isaac shows it as an avatar). Nothing else in the task changes.

All-sim tasks assume Part A (A1–A3) is up; any `real` drone assumes Part B.

### C1. Single-drone goal (`goal_single.yaml`)

One drone flies to a goal you set, at a speed you set. `drone_modes: "sim"`
(flip to `"real"` to fly the goal on hardware). For a 1-drone sim, spawn with
`NUM_ROBOTS=1` in A2 and `./launch_sim_interfaces.sh 1` in A3.

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_single.yaml
# control terminal:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 1.4}}}"
ros2 topic pub --once /svg/drone_1/speed_command std_msgs/msg/Float32 "{data: 0.8}"
ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```

### C2. Multi-drone goal (`goal_tracking.yaml`)

Assign different goals to different drones while flying; the CBF keeps them
apart when paths cross. Mix modes with e.g. `drone_modes: "real,sim,sim"`.

```bash
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_tracking.yaml
# takeoff + start, then retarget any drone any time:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_2/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_2/speed_command std_msgs/msg/Float32 "{data: 1.0}"
```

### C3. Squeeze (`squeeze_3drone.yaml`)

Holders (drone_1,2) hold their posts; the intruder (drone_3) shuttles through
the gap. drone_3 is **CBF-exempt** (`cbf_exempt_drones: "drone_3"`) so it
presses through and the holders alone yield. Drive the intruder by hand instead
with `teleop_drones: "drone_3"` (it stays exempt — it's in the exempt list; see
[A5](#a5-teleop-fresh-terminal-only-if-a-drone-has-role-teleop) for keyboard
teleop).

```bash
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_3drone.yaml
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
```

### C4. Flagship — hybrid squeeze: real holders + sim intruder (`hybrid_squeeze.yaml`)

The target run: **drone_1,2 real** (mocap hardware), **drone_3 sim** (Isaac
SITL) and **CBF-exempt + policy-controlled** (not teleop). All three appear in
the Isaac viewport — real holders as live avatars, the intruder as its SITL
body — and the real holders react (via the CBF) to the virtual intruder
squeezing through. Config already set: `drone_modes: "real,real,sim"`,
`cbf_exempt_drones: "drone_3"`.

```bash
# 0. real holders connected + verified — Part B (px4_interface + NatNet + mocap)
#    for drone_1,drone_2.

# 1. Isaac (GUI): SITL for the sim intruder + avatars for the real holders.
#    DRONE_MODES matches the commander's drone_modes.  [isaac-sim container]
NUM_ROBOTS=3 DRONE_MODES="real,real,sim" SVG_DOMAIN_ID=1 \
PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=false \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

# 2. MAVROS interface for the sim intruder (drone_3) only.  [robot container]
ROBOT_NAME=drone_3 FCU_URL='udp://:14543@<sim_ip>:14583' TGT_SYSTEM=4 \
  ros2 launch svg_ground_control sim_drone_interface.launch.xml drone_name:=drone_3

# 3. ONE commander for all three (use_mocap feeds the real holders' EKFs).
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/hybrid_squeeze.yaml \
  use_mocap:=true
```
One CBF sees all three (the state topic is identical for real and sim), so the
real holders dodge the simulated intruder. In RViz the holders are **red**, the
intruder **cyan**, all in one `map` frame.

> **Dry-run the routing first (no hardware).** `test/functional_hybrid_test.py`
> fakes the real+sim drones on their respective topics and asserts each drone's
> commands land on the correct namespace and the squeeze still works — run it
> before trusting a real flight (see [Automated tests](#automated-tests)).

---

# Part D — Real hardware: first flight & reference

Once [Part B](#part-b--bring-in-a-real-drone-connect--verify) confirms tracking,
fly. The flight services are identical to sim ([A6](#a6-fly-fresh-terminal)) —
only the config (real modes) and the safety discipline differ.

### D1. Preflight + fly (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1 && cd ~/AirStack/robot/ros_ws && sws
ros2 topic hz   /drone_1/pose                                  # mocap arriving?
ros2 topic echo /drone_1/odometry_conversion/odometry --once   # tracks reality?
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger
```

### D2. First-flight safety

- **One drone first.** `drone_names: ["drone_1"]`, `drone_modes: "real"`,
  scenario `hover`, thumb on the **RC kill switch**. Then two. Then the demo.
- The geofence is a freeze-in-place, **not** a motor cutoff — the RC kill
  switch is the true cutoff ([Geofence](#geofence)).
- Fit `arena_*` and `fence_*` to your capture volume before arming.
- Keep `cbf_max_speed_mps` conservative on hardware (`swarm_real.yaml` uses
  1.0).

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

**Hand-carry / preflight (no flight needed).** The markers come from
`swarm_commander`, not the drones directly, so the chain is: interface layer
→ `/{name}/odometry_conversion/odometry` → commander → `/svg/viz/markers` →
RViz. To watch drones move by hand with nothing armed:
1. bring up the per-drone interfaces (Part A A3 for sim, or Part B for
   hardware: px4_interface + NatNet + mocap bridge) so odometry flows;
2. launch `ground_control.launch.py` but **do NOT call takeoff** — the
   commander idles in IDLE, publishes zero commands, and still publishes
   markers every tick;
3. launch RViz.
Now move each drone by hand and its sphere tracks live — the ideal hardware
preflight to confirm mocap→odometry matches reality before arming. If RViz is
empty: `ros2 topic hz /svg/viz/markers` (should be ~20 Hz; if silent the
commander isn't running) and `ros2 topic echo
/{name}/odometry_conversion/odometry --once` (a drone with no odometry is
skipped in the markers).

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
# node-level (needs rclpy; constructs the commander, no launch/interfaces):
python3 -m pytest test/test_exempt.py -q          # cbf_exempt_drones / teleop decoupling

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
| Isaac Sim segfaults at startup, backtrace in `librtx.scenedb.plugin.so` / `libcarb.scenerenderer-rtx.plugin.so` at `carbOnPluginStartup` — **also crashes headless**, and a bare empty `SimulationApp({"headless":True})` crashes identically | GPU driver ↔ Isaac Sim RTX incompatibility, NOT an AirStack bug. App boots to `app ready` then the RTX renderer faults on the first frame. Confirmed on RTX 5080 / Blackwell + NVIDIA driver **595.x** + Isaac Sim 5.1.0. Headless and clearing the shader cache do **not** help (the renderer plugin loads at app init regardless; there is no renderer-less path through Kit). **Fix:** install a driver Isaac Sim 5.1 supports — Linux **580.65.06**, or **591.74** (a Blackwell user's confirmed-good version) — using the *open* kernel module variant required for RTX 50-series; or upgrade to a newer Isaac Sim release. ([NVIDIA forum report](https://forums.developer.nvidia.com/t/isaac-sim-5-1-gui-crash-access-violation-on-rtx-5070-ti-blackwell-fixed-by-driver-downgrade-to-591-74/365335)) |
| MAVROS `connected: false`, no odometry | PX4 SITL not launched: Isaac timeline not playing (`PLAY_SIM_ON_START=true`, or press Play) |
| takeoff returns success=false right after launch | commander hasn't received odometry yet — wait a few seconds and retry |
| `The parameter 'X' is not initialized` | empty YAML list can't override a typed param — `teleop_drones`/`external_drones`/`drone_modes`/`cbf_exempt_drones` are comma-separated STRINGS (`""` = none) |
| teleop drone gets shoved around / won't act as the obstacle | teleop is **no longer auto-CBF-exempt** — add it to `cbf_exempt_drones` to leave its commands uncorrected (and let others dodge it). Conversely, drop it from the list to have the filter protect your manual commands |
| commander rejects config: `"X" is in both external_drones and cbf_exempt_drones` | external drones are never commanded, so they can't be "exempt" — remove the name from one of the two lists |
| Isaac avatar (real drone) doesn't appear / doesn't move | (a) launch Isaac with a **GUI** viewport (`ISAAC_SIM_HEADLESS=false`) and `DRONE_MODES` listing that drone as `real`; (b) the avatar tracks `…/odometry_conversion/odometry` — confirm it's flowing (`ros2 topic hz`); (c) domain: the script sets `ROS_DOMAIN_ID=SVG_DOMAIN_ID`, so the drone must publish on that domain; (d) **true hardware**: the real drone's odometry is on a host-network container while Isaac is on the bridge network — DDS must cross them (run Isaac with host networking or a discovery server). Pure-sim dry-run: publish a synthetic odom or run that drone as a throwaway SITL |
| `start` says "not all drones holding yet" | drones still converging to takeoff targets; retry after a few seconds |
| `start` says "geofence breached" | a drone left the box; `ros2 service call /swarm_commander/reset_fence std_srvs/srv/Trigger` after recovering |
| drones fly right *shapes* in wrong *places*; intruder misses the gap | per-drone PX4 local origins: `drone_position_offsets` must equal the sim spawn positions (`x = 2*(i-1) - (N-1)` → `[-2,0,0, 0,0,0, 2,0,0]` for 3). Zeros only for mocap-anchored hardware |
| hybrid: a "real" drone never moves | nothing is consuming `/{name}/fmu/velocity_command` — real-mode drones need px4_interface up (Part B); validate the routing first with `functional_hybrid_test.py` |
| RViz empty | Fixed Frame must be `map`; check `ros2 topic hz /svg/viz/markers`; needs an X display (`echo $DISPLAY`) |
