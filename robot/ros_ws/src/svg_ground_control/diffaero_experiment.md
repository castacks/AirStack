# DiffAero Ground Control — Guide & Command Reference

> **Maintained file.** Canonical, copy-pasteable reference for flying the
> **single-drone DiffAero learned policy** on AirStack, via the two commanders:
> the **attitude+thrust** commander (`diffaero_commander`) and the
> **velocity-command** commander (`diffaero_velocity_commander`). Every command
> block assumes a **fresh terminal**. For the *why* behind the frame/timing/
> controller details, see the companion [`DIFFAERO.md`](DIFFAERO.md); this file
> is the *how-to-run*. The multi-drone CBF workflow is a separate doc
> ([`experiment.md`](experiment.md)).

## Contents
1. [How AirStack is structured](#1-how-airstack-is-structured)
2. [How DiffAero ground control is structured](#2-how-diffaero-ground-control-is-structured)
3. [Topic & service wiring](#3-topic--service-wiring)
4. [Conventions (domain, tmux, rebuilds)](#4-conventions)
5. [Part A — Simulation](#part-a--simulation)
6. [Part B — Real hardware (px4_interface + mocap)](#part-b--real-hardware-px4_interface--mocap)
7. [Choosing & tuning the policy](#choosing--tuning-the-policy)
8. [Perception / ToF](#perception--tof)
9. [Geofence](#geofence)
10. [RViz visualization](#rviz-visualization)
11. [Recording rosbags](#recording-rosbags)
12. [Troubleshooting](#troubleshooting)

---

## 1. How AirStack is structured

AirStack is a layered ROS 2 (Jazzy) autonomy stack that runs in Docker
containers. The pieces relevant to us:

```
┌─────────────────────────────────────────────────────────────────────┐
│ Docker containers (started by ./airstack.sh up, one bridge network)   │
│                                                                       │
│  isaac-sim ───── PX4 SITL (one drone) ── MAVLink UDP / uXRCE-DDS      │
│   (Isaac Sim + Pegasus; physics, sensors, flight dynamics)            │
│                                                                       │
│  robot-desktop-1 ── the ROS 2 workspace (robot/ros_ws), where our     │
│                     nodes run. /AirStack is bind-mounted from host.    │
└─────────────────────────────────────────────────────────────────────┘
```

**The autonomy workspace** (`robot/ros_ws/src`) is layered; the parts we touch:

- `interface/` — talks to the flight controller. Two interchangeable plugins
  behind the same ROS API (`robot_interface_node`):
  - `mavros_interface` → PX4 over **MAVROS/MAVLink** (used for SIM/SITL).
  - `px4_interface` → PX4 over **uXRCE-DDS** (`/fmu/*` topics, used for
    HARDWARE). Converts ENU↔NED, runs the 10 Hz offboard heartbeat, and accepts
    `velocity_command` / `pose_command` / `attitude_thrust_command` /
    `robot_command`.
  - `odometry_conversion` — both plugins feed this; it republishes the canonical
    `…/odometry_conversion/odometry` (ENU `nav_msgs/Odometry`) and the
    `map→base_link` TF.
- `svg_ground_control/` — **our package**. The DiffAero commanders sit *on top
  of* the interface layer: they read the drone's odometry, run the learned
  policy, and write the drone's command (attitude+thrust **or** velocity).

Normally AirStack auto-launches the full autonomy stack; we run with
`AUTOLAUNCH=false` and launch only our nodes, so nothing fights us for control.

---

## 2. How DiffAero ground control is structured

DiffAero is a **single-drone, learned cruise controller** (SHA2C, point-mass)
exported as a self-contained TorchScript actor. Two commander executables wrap
it, differing only in **what the policy outputs and how it's published**:

| Script | Node | Policy output | Command topic (sim) | Best for |
|---|---|---|---|---|
| `diffaero_commander.py` | `diffaero_commander` | **attitude + normalized thrust** (`mav_msgs/AttitudeThrust`) | `/{name}/interface/attitude_thrust_command` | aggressive/agile flight; the FCU only does rate control |
| `diffaero_velocity_commander.py` | `diffaero_velocity_commander` | **velocity setpoint** (`geometry_msgs/TwistStamped`) | `/{name}/interface/velocity_command` | tamer flight; the FCU owns attitude + velocity control |

Both share the **same** lifecycle, geofence, RViz markers, scenario plumbing,
and policy core. The wrapper cores live in
`svg_ground_control/diffaero/`:
- `diffaero_core.py` → `DiffAeroPolicy` (attitude+thrust). Actor returns
  `(acc_cmd, quat_xyzw, acc_norm)`; obs is a **9-vector**
  `[target_vel_local, uz, v_local]`.
- `diffaero_vel_core.py` → `DiffAeroVelPolicy` (velocity). Actor returns a single
  world-ENU velocity setpoint; obs is a **6-vector** `[target_vel_local,
  v_local]` (no body up-axis). Yaw follows travel direction; the commander turns
  the policy's desired heading into a **yaw-rate** (`twist.angular.z`).
- `perception_builder.py` → optional depth→`9×16` grid encoder (see
  [Perception / ToF](#perception--tof)).

**Data flow each tick (both commanders):**

```
  /{name}/odometry_conversion/odometry  (ENU; twist rotated body→world)
        │
        ▼
   build DiffAeroObs:  target_vel = (goal − pos) saturated to max_vel,
        │              projected into the yaw frame; + perception grid
        ▼
   policy.compute()
        │   attitude commander → AttitudeThrust (ENU/FLU; interface → NED/FRD)
        │   velocity commander → TwistStamped   (sim: world→body rotate; +yaw-rate)
        ▼
   geofence latch  ──►  publish command   +   /svg/viz/markers (RViz)
```

**Lifecycle** (`std_srvs/Trigger` services), identical for both:

```
IDLE ─takeoff→ ARMING ─→ ASCEND ─→ FACE_GOAL ─→ ACTIVE ─start→ (cruise) ─arrive→ HOLD ─land→ LANDING ─→ IDLE
```

| State | Behavior |
|---|---|
| `ARMING` | Timed: request offboard (1.0 s), arm (1.5 s), done (2.5 s); streams current pose to satisfy PX4 offboard-entry. |
| `ASCEND` | Pose command to `hover_positions`; transitions within `arrival_threshold_m`. |
| `FACE_GOAL` | Yaw in place to point at `goal_position` (avoids dragging a leg on launch). Transitions when yaw error < `face_goal_threshold_rad`. |
| `ACTIVE` | `start` → runs the policy toward `goal_position`; on arrival (`goal_arrival_threshold_m`) hands off to a stable **pose-hold** (the policy is a cruise, not a hover, controller — see DIFFAERO.md §6a). Otherwise idles in pose-hold. |
| `LANDING` | Velocity-down until `land_complete_altitude_m`, then disarm. |

Services (per commander): `~/takeoff`, `~/start`, `~/hold` (panic freeze),
`~/land`, `~/reset_fence`.

---

## 3. Topic & service wiring

For drone `{name}` (default `drone_1`). `drone_mode` (`sim`|`real`) switches the
command/service namespace (`/interface/` ↔ `/fmu/`); the state topic is the same.

| Topic / service | Dir | Type | Who |
|---|---|---|---|
| `/{name}/odometry_conversion/odometry` | in | `nav_msgs/Odometry` | interface layer → commander & RViz |
| `/{name}/perception/tof` | in | `std_msgs/Float32MultiArray` | (optional) pre-encoded `9×16` grid → policy |
| `/{name}/interface/attitude_thrust_command` (sim) | out | `mav_msgs/AttitudeThrust` | **attitude** commander → MAVROS |
| `/{name}/fmu/attitude_thrust_command` (real) | out | `mav_msgs/AttitudeThrust` | **attitude** commander → px4_interface |
| `/{name}/interface/velocity_command` (sim) | out | `geometry_msgs/TwistStamped` | **velocity** commander (+ both for landing) → MAVROS |
| `/{name}/fmu/velocity_command` (real) | out | `geometry_msgs/TwistStamped` | **velocity** commander → px4_interface |
| `/{name}/interface/pose_command` or `/{name}/fmu/pose_command` | out | `geometry_msgs/PoseStamped` | both (ascend / hold / fence) |
| `/{name}/interface/robot_command` or `/{name}/fmu/robot_command` | call | `airstack_msgs/srv/RobotCommand` | both → arm / offboard / disarm |
| `/svg/{name}/goal_command` | in | `geometry_msgs/PoseStamped` | you → commander (`goal` scenario) |
| `/svg/{name}/speed_command` | in | `std_msgs/Float32` | you → commander (`goal` scenario) |
| `/svg/viz/markers` | out | `visualization_msgs/MarkerArray` | commander → RViz |
| `/diffaero_commander/{takeoff,start,hold,land,reset_fence}` | call | `std_srvs/Trigger` | you → **attitude** commander |
| `/diffaero_velocity_commander/{takeoff,start,hold,land,reset_fence}` | call | `std_srvs/Trigger` | you → **velocity** commander |

> ⚠️ **Velocity-command frame differs by mode.** The policy emits world-ENU
> velocity. The **sim** MAVROS interface interprets `velocity_command` as a
> **body** frame (FRAME_BODY_NED → FLU on the ROS side), so the velocity
> commander rotates world→body before publishing in `sim`; the **real**
> px4_interface consumes world ENU directly. The yaw-rate (`angular.z`, ENU
> CCW+) is correct for both. (See `publish_velocity` /
> `_world_to_body` in `diffaero_velocity_commander.py`.)

---

## 4. Conventions

**ROS domain = 1 everywhere.** The robot container's
[`.bashrc`](../../../docker/.bashrc) hard-pins `ROS_DOMAIN_ID=1`, so every shell
you open in it is already on domain 1 (rebuild the image after pulling — see
[A1](#a1-containers-host)). Check `echo $ROS_DOMAIN_ID` in every shell — a
mismatch shows up as "service unavailable" / missing topics.

**Single drone.** DiffAero is single-drone: keep `.env NUM_ROBOTS="1"` (the
default Isaac script `example_one_px4_pegasus_launch_script.py` spawns one drone;
the SVG single-domain script also works with `NUM_ROBOTS=1`). `drone_1` spawns at
the origin, so `drone_position_offset: [0,0,0]` (raw odom == world).

**tmux** (when you `./airstack.sh connect robot` without `--command=bash`):
`Ctrl-b c` new window · `Ctrl-b n/p` switch · `Ctrl-b d` detach. Every new window
is a fresh shell: re-run `cd ~/AirStack/robot/ros_ws && sws`.

**Rebuild after edits.** `ros2 launch` reads the *installed* copy. After editing
any `.py`/`.yaml` in the package, run `bws` (or pass `config:=` pointing straight
at the source file under `src/.../config/`).

**Checkpoints.** The commanders load a TorchScript actor from `checkpoint_path`
(a directory containing `checkpoints/exported_actor.pt2` + `.hydra/config.yaml`).
The default configs point at `robot/ros_ws/checkpoints/diffaero/<run>/` (inside
the container, `/root/AirStack/robot/ros_ws/checkpoints/...`), bind-mounted from
the host. The velocity commander **validates** the checkpoint is a
`velocity_pointmass` (`action_is_velocity=true`) run and errors otherwise.

---

# Part A — Simulation

One SITL drone, one commander. Pick **attitude** or **velocity** in A4.

### A1. Containers (host)

```bash
cd ~/AirStack
./airstack.sh image-build robot-desktop     # after pulling a branch that changed the image
# .env: COMPOSE_PROFILES="desktop,isaac-sim", AUTOLAUNCH="false", NUM_ROBOTS="1"
grep -E '^(COMPOSE_PROFILES|AUTOLAUNCH|NUM_ROBOTS)' .env
./airstack.sh up
./airstack.sh status        # robot-desktop-1 and isaac-sim Up
```

### A2. Isaac Sim — spawn the drone (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect isaac-sim --command=bash
```
Inside (`PLAY_SIM_ON_START=true` REQUIRED — PX4 SITL only launches when the
timeline plays; run headless unless you need the viewport):
```bash
NUM_ROBOTS=1 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```
Expect `Spawning 1 drone(s) on ROS domain 1` then `PX4 Autolaunch: True`.
`drone_1` spawns at the origin (matches `drone_position_offset: [0,0,0]`).

### A3. Build + MAVROS interface (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
echo $ROS_DOMAIN_ID                       # 1
cd ~/AirStack/robot/ros_ws && bws && sws  # bws first time / after edits
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 1
```
Verify (another shell): `ros2 topic echo /drone_1/interface/mavros/state --once`
→ `connected: true`, then `ros2 topic hz /drone_1/odometry_conversion/odometry`
(~30 Hz after the EKF converges, ~30 s).

> **Why before the commander:** the commander reads state from
> `/{name}/odometry_conversion/odometry`, produced by the interface. Without it
> `~/takeoff` returns `no odometry yet`.

### A4. DiffAero commander (fresh terminal) — pick ONE

```bash
cd ~/AirStack/robot/ros_ws && sws
```

**Attitude+thrust policy** (`config/diffaero_sim.yaml`):
```bash
ros2 launch svg_ground_control diffaero_single.launch.py
#   override scenario:  scenario:=goal
```

**Velocity-command policy** (`config/diffaero_vel_sim.yaml`):
```bash
ros2 launch svg_ground_control diffaero_velocity_single.launch.py
#   override scenario:  scenario:=goal
```

Watch for `DiffAero … policy loaded from …` and `… policy warmed up` at startup
(the warm-up avoids a stall on the first control tick — DIFFAERO.md §7).

### A5. Fly (fresh terminal)

The service namespace matches the commander you launched in A4.

**Attitude commander:**
```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 service call /diffaero_commander/takeoff std_srvs/srv/Trigger   # arm+ascend+face goal+hold
ros2 service call /diffaero_commander/start   std_srvs/srv/Trigger   # cruise to goal, auto-hold on arrival
ros2 service call /diffaero_commander/hold    std_srvs/srv/Trigger   # PANIC freeze
ros2 service call /diffaero_commander/land    std_srvs/srv/Trigger   # descend+disarm
```

**Velocity commander:**
```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 service call /diffaero_velocity_commander/takeoff std_srvs/srv/Trigger
ros2 service call /diffaero_velocity_commander/start   std_srvs/srv/Trigger
ros2 service call /diffaero_velocity_commander/hold    std_srvs/srv/Trigger
ros2 service call /diffaero_velocity_commander/land    std_srvs/srv/Trigger
```

> **Sequencing:** `takeoff` → drone ascends, yaws to face the goal, logs
> `… → ACTIVE` and holds → `start` (begins cruising to `goal_position`) → on
> arrival auto-holds at the goal → `land`. `start` is rejected until the drone is
> holding in `ACTIVE`, and is blocked while a geofence breach is latched
> (`~/reset_fence` clears it).

### A6. (Optional) set the goal live with the `goal` scenario

Launch with `scenario:=goal` in A4, then:
```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_1/speed_command std_msgs/msg/Float32 "{data: 1.0}"
```

---

# Part B — Real hardware (px4_interface + mocap)

The DiffAero policy flies a real drone over **uXRCE-DDS** (`/fmu/*`) with mocap
feeding PX4's EKF. The hardware bring-up (VOXL comms, the uXRCE-DDS agent, NatNet
mocap, the EKF2 external-vision params, frame hand-checks) is **identical to the
SVG workflow** — follow [`experiment.md` Part B](experiment.md#part-b--bring-in-a-real-drone-connect--verify)
verbatim (B0–B4b). The only DiffAero-specific differences:

1. **Config with `drone_mode: real`.** There is no `diffaero_real.yaml` checked
   in yet — copy `diffaero_sim.yaml` (or `diffaero_vel_sim.yaml`), set
   `drone_mode: "real"` and `drone_position_offset: [0,0,0]` (mocap is already
   world-anchored), and adjust `goal_position`/`fence_*` to your capture volume.
   Then launch with `config:=<that file>` (and `use_mocap:=true`).

2. **px4_interface up first** (the analogue of A3), so state flows on
   `…/odometry_conversion/odometry`:
   ```bash
   ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1
   ```

3. **Commander + mocap bridge:**
   ```bash
   cd ~/AirStack/robot/ros_ws && sws
   ros2 launch svg_ground_control diffaero_velocity_single.launch.py \
     config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/<your_real>.yaml \
     use_mocap:=true
   #   attitude policy → diffaero_single.launch.py the same way
   ```
   `use_mocap:=true` starts `mocap_bridge` (mocap → `/{name}/fmu/in/
   vehicle_visual_odometry`), the only way EKF2 fuses a position indoors.

4. **Fly** with the matching service namespace (B/A5). **First-flight safety:**
   thumb on the RC kill switch (the geofence is a freeze, not a cutoff), `hover`
   goal first, conservative `max_vel`. On the real path the velocity commander
   publishes world-ENU velocity (no body rotation) and px4_interface converts to
   NED — verify a metre of commanded `+x` moves the drone East before trusting a
   full cruise.

> **Hover-throttle (attitude commander only).** The attitude policy's
> normalized thrust is anchored to `max_accel`; if the drone slowly climbs/sinks
> at hover, match the FCU hover param to `g/max_accel` (DIFFAERO.md §8). The
> velocity commander sidesteps this — PX4 owns thrust.

---

## Choosing & tuning the policy

| Want… | Use | Key params |
|---|---|---|
| Agile / aggressive, FCU does rate control only | **attitude** (`diffaero_commander`) | `max_accel` (hover-throttle anchor), `max_acc_xy` (tilt cap → `atan(max_acc_xy/g)`), `max_vel` |
| Tamer, FCU owns attitude+velocity, easier to trust | **velocity** (`diffaero_velocity_commander`) | `max_vel` (target-vel saturation), `max_vel_xy`/`max_vel_z` (actor action limits; `-1` = checkpoint default), `yaw_kp`/`yaw_rate_max` |

Shared cruise/landing params (`max_vel`, `goal_arrival_threshold_m`,
`control_rate_hz: 30` to match training `dt`, `arrival_threshold_m`,
`land_*`, `fence_*`) behave the same in both. Full rationale: DIFFAERO.md §8.

Tuning order for a new site: (1) confirm hover is stable (attitude:
`max_accel`), (2) set a modest `max_vel`, (3) tame the launch (attitude:
`max_acc_xy`; velocity: `max_vel_xy`), (4) widen `goal_arrival_threshold_m` if it
overshoots before the hold catches.

---

## Perception / ToF

The policy can consume a `9×16` obstacle grid on `/{name}/perception/tof`
(`Float32MultiArray`, `0 = clear`, `1 = obstacle`). **No node in this repo
currently publishes it**, so by default the policy receives a zeros grid (older
than `tof_timeout_s` → dropped to zeros) and flies **as if there are no
obstacles** — fine for open-arena goal-seeking, but it will **not** avoid
anything until a ToF producer is wired up.

To enable obstacle avoidance you must publish the pre-encoded grid (e.g. from a
depth camera through `perception_builder.PerceptionBuilder`, or a real ToF
sensor) on `/{name}/perception/tof` at ≥ the control rate. The encoding and the
raw-depth path are documented in DIFFAERO.md §5. Confirm it's live before relying
on it: `ros2 topic hz /drone_1/perception/tof` and watch the commander log
`tof_fresh=True`.

---

## Geofence

A latching safety box in the commander. If the drone (in `FACE_GOAL`/`ACTIVE`)
leaves `[fence_min, fence_max]` (world ENU), it freezes in place, the mission
stops, and `start` is refused until `~/reset_fence`. Climb-out/landing are
exempt.

```yaml
fence_enabled: true
fence_min: [-3.0, -3.0, 0.0]
fence_max: [ 3.0,  3.0, 3.0]
```
```bash
ros2 service call /diffaero_commander/reset_fence std_srvs/srv/Trigger
#   velocity commander → /diffaero_velocity_commander/reset_fence
```
This is a freeze-in-place, not a motor cutoff — the RC kill switch remains the
true cutoff. The box is drawn in RViz (green normally, red when latched).

---

## RViz visualization

The commander publishes the drone's world pose, heading arrow, goal, and the
fence box as a `MarkerArray` on `/svg/viz/markers`:

```bash
# from a robot-container shell (./airstack.sh connect robot --command=bash):
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```
Fixed Frame = `map`; Add → By topic → `/svg/viz/markers` if starting bare. The
drone sphere is **cyan** in sim, **red** in real, **orange** when a fence breach
is latched. The yellow arrow is the heading (FLU forward).

---

## Recording rosbags

Record INTO the mounted workspace so the bag survives the container. Ctrl-C
stops and writes `metadata.yaml`.

```bash
# attitude commander
ros2 bag record -o ~/AirStack/robot/ros_ws/bags/diffaero_$(date +%H%M%S) \
  /drone_1/odometry_conversion/odometry \
  /drone_1/interface/attitude_thrust_command \
  /drone_1/perception/tof  /svg/viz/markers

# velocity commander
ros2 bag record -o ~/AirStack/robot/ros_ws/bags/diffaero_vel_$(date +%H%M%S) \
  /drone_1/odometry_conversion/odometry \
  /drone_1/interface/velocity_command \
  /drone_1/perception/tof  /svg/viz/markers
ros2 bag info <bag_dir>
```

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `takeoff` → `no odometry yet` | the interface isn't up (A3) or odom hasn't started — `ros2 topic hz /drone_1/odometry_conversion/odometry`; in sim wait ~30 s for EKF convergence |
| `start` → `not holding yet (state=…)` | drone hasn't reached `ACTIVE` — it's still ascending or yaw-facing the goal; wait for `… → ACTIVE`, or loosen `arrival_threshold_m`/`face_goal_threshold_rad` |
| `start` → `geofence breached` | drone left the box → `~/reset_fence` (matching namespace) after recovering, and widen `fence_*` |
| `FileNotFoundError: exported_actor.pt2` / `.hydra/config.yaml` | `checkpoint_path` wrong, or the checkpoint dir isn't bind-mounted into the container. It must contain `checkpoints/exported_actor.pt2` (+ `.hydra/config.yaml` for the velocity commander) |
| velocity commander: `Expected dynamics.name=velocity_pointmass` | you pointed the **velocity** commander at an **attitude** checkpoint (or vice-versa). The velocity commander only loads `action_is_velocity=true` runs; use `diffaero_commander` for attitude checkpoints |
| `checkpoint_path not set — … nominal fallback` | the config's `checkpoint_path` is empty → ACTIVE uses the scenario's nominal velocity, not the policy. Set the path and rebuild/`config:=` |
| drone lunges / over-tilts on launch | attitude: lower `max_acc_xy` (tilt cap); velocity: lower `max_vel_xy`. Both: lower `max_vel` |
| attitude: drone slowly climbs/sinks at hover | `max_accel` ≠ FCU hover throttle. Match `MPC_THR_HOVER ≈ g/max_accel` (DIFFAERO.md §8) |
| oscillates / "goes crazy" at the goal | the policy is a cruise, not a hover, controller — ensure `goal_arrival_threshold_m` is wide enough to hand off to pose-hold before overshoot (DIFFAERO.md §6a) |
| velocity (sim): obstacle-avoidance / lateral motion looks **mirrored** | the world→body rotation sign — flip the convention in `_world_to_body` (DIFFAERO.md / §3 note). Forward motion is unaffected |
| runs straight into the wall / reported `vel` sign opposite to motion | body-vs-world velocity bug — the twist must be rotated body→world in `odometry_callback` (DIFFAERO.md §3a). Check `ros2 topic echo /drone_1/odometry_conversion/odometry --field twist.twist.linear` |
| drone never avoids obstacles | nothing publishes `/{name}/perception/tof` → zeros grid (see [Perception / ToF](#perception--tof)). Wire up a ToF/depth producer; confirm `tof_fresh=True` in the log |
| `odometry stale, holding position` right at `start` | first `policy.compute()` JIT stall on a single-threaded executor — confirm `… policy warmed up` printed at startup (DIFFAERO.md §7); a one-off at handoff is benign, recurring mid-flight is a real stream gap |
| service `waiting for service to become available…` forever | `ROS_DOMAIN_ID` mismatch between shells (must be 1); `ros2 daemon stop` |
| MAVROS `connected: false`, no odometry | PX4 SITL not launched: Isaac timeline not playing (`PLAY_SIM_ON_START=true`, or press Play) |
| `package 'svg_ground_control' not found` / `bws`/`sws` missing | not in a robot-container shell — `./airstack.sh connect robot --command=bash` (its `.bashrc` sets the domain + sources the workspace) |
| Isaac Sim segfault at startup (`librtx.scenedb.plugin.so` …) | GPU driver ↔ Isaac Sim RTX incompatibility, not an AirStack bug — see [`experiment.md` Troubleshooting](experiment.md#troubleshooting) |
