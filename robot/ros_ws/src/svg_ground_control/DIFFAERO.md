# DiffAero on AirStack

This documents how the **DiffAero** learned flight policy (SHA2C, continuous
point-mass) is wired into AirStack, and — more importantly — **why** each
non-obvious detail is the way it is. DiffAero ships as **two commander
executables** that wrap the *same* policy core but differ in what the network
outputs and how it reaches the flight controller:

| Commander | Node | Policy output | Source | FCU owns |
|-----------|------|---------------|--------|----------|
| **attitude+thrust** | `diffaero_commander` | attitude quaternion + normalized thrust (`mav_msgs/AttitudeThrust`) | `diffaero_commander.py` + `diffaero/diffaero_core.py` | rate loop only |
| **velocity** | `diffaero_velocity_commander` | world-ENU velocity setpoint (`geometry_msgs/TwistStamped`) | `diffaero_velocity_commander.py` + `diffaero/diffaero_vel_core.py` | attitude + velocity |

The attitude commander gives aggressive/agile flight (the network closes the
attitude loop); the velocity commander is tamer and easier to trust (PX4 owns
attitude and thrust). They share lifecycle, geofence, RViz markers, scenario
plumbing, warm-up, and the perception path — only the **action space** and its
**output plumbing** differ. Most of these notes exist because the attitude
policy was first deployed standalone against PX4 over MAVLink in
`superfly/starling-deployment`, and porting it onto AirStack's `robot_interface`
abstraction surfaced a series of frame / timing / controller mismatches. Each is
recorded below so the next person doesn't re-derive them.

> **How-to-run lives in [`diffaero_experiment.md`](diffaero_experiment.md);**
> this file is the *why*. The full copy-pasteable command sequence is §10 below.

---

## 1. What DiffAero is (and is not)

- **Type:** a learned **cruise** controller exported as a self-contained
  TorchScript actor (`checkpoints/.../exported_actor.pt2`).
  - **Attitude actor** bakes in `tanh → rescale → Rz @ action → point_mass_quat`
    and returns `(acc_cmd, quat_xyzw_cmd, acc_norm)`. The action is a
    **world-frame thrust acceleration**; gravity is handled by the point-mass
    model, so we do **not** add `g` when forming the attitude/thrust setpoint.
  - **Velocity actor** (exported with `dynamics=velocity_pointmass`,
    `action_is_velocity=true`) returns a single **world-ENU velocity** setpoint
    `[vx, vy, vz]` — no accel→attitude conversion, because the FCU owns attitude
    in velocity mode.
- **Inputs (obs_frame = local, point-mass):**
  - Attitude → a **9-vector** `[target_vel_local(3), uz(3), v_local(3)]`
    (`uz` is the body up-axis in world).
  - Velocity → a **6-vector** `[target_vel_local(3), v_local(3)]` — **no `uz`**.

  Both also take an optional `9×16` depth "perception" grid (the velocity actor
  only consumes it if the checkpoint was trained with `env=obstacle_avoidance`).
  `target_vel` and `v` are expressed in the **yaw-only (local) frame**.
- **Output frame:** attitude → ENU/FLU quaternion `[x,y,z,w]` (the interface
  converts to NED/FRD); velocity → world-ENU velocity (the velocity commander
  rotates to body in sim — see §3c).
- **Trained at** `dt = 0.0333 s` → **30 Hz**. Run the control loop at 30 Hz
  (`control_rate_hz: 30.0`) so the policy sees the timestep it expects. (The
  parameter *default* is 20 Hz; both shipped configs override to 30.)

> **Key consequence (both commanders):** it is **not** a position-hold
> controller. At the goal, `target_vel → 0`, but the network has nothing stable
> to track and will overshoot/oscillate. We must hand off to a pose-hold on
> arrival (see §6a).

The policy core (`diffaero/diffaero_core.py`, `diffaero/diffaero_vel_core.py`,
`diffaero/perception_builder.py`) is **byte-for-byte identical** to the standalone
deployments. Everything that differs lives in the *plumbing* around it —
documented below.

---

## 2. Data flow

```
                      AirStack robot_interface              this package
  ┌─────────────┐    ┌────────────────────────┐    ┌──────────────────────────────┐
  │  MAVROS /   │    │ mavros_interface /     │    │ diffaero_commander       (att)│
  │  PX4 (SITL) │───▶│ px4_interface →        │───▶│   odometry_callback           │
  │             │    │ odometry_conversion    │    │   → DiffAeroObs (9-vec)        │
  └─────────────┘    │  /{name}/odometry_     │    │   → DiffAeroPolicy.compute     │
                     │  conversion/odometry   │    │   → AttitudeThrust out         │
                     │  (ENU nav_msgs/Odom)   │    │ ── OR ────────────────────────│
                     │                        │    │ diffaero_velocity_commander   │
                     │                        │───▶│   odometry_callback           │
                     │                        │    │   → DiffAeroObs (6-vec)        │
                     │                        │    │   → DiffAeroVelPolicy.compute  │
                     │                        │    │   → TwistStamped + yaw-rate out│
                     └────────────────────────┘    └──────────────────────────────┘
   perception: /{name}/perception/tof  (Float32MultiArray, pre-encoded 9×16)  [both]
   commands (att): /{name}/<iface>/attitude_thrust_command  (mav_msgs/AttitudeThrust)
   commands (vel): /{name}/<iface>/velocity_command         (geometry_msgs/TwistStamped)
   commands (both): /{name}/<iface>/pose_command            (ascend / hold / fence)
                    /{name}/<iface>/velocity_command         (landing, both)
   services (both): /{name}/<iface>/robot_command           (arm / offboard / disarm)
```

`drone_mode` selects the interface topic templates (`<iface>` above) for **both**
commanders:
- `sim`  → `/{name}/interface/*` (MAVROS/SITL)
- `real` → `/{name}/fmu/*` (px4_interface / uXRCE-DDS)

---

## 3. Frame conventions (the most important section)

The policy lives entirely in **ENU world / FLU body**:

| Quantity            | Frame expected by policy            |
|---------------------|-------------------------------------|
| `position_enu`      | ENU world                           |
| `velocity_enu`      | **ENU world** (measured, not finite-differenced) |
| `R_enu`             | FLU-body → ENU-world rotation matrix |
| `goal_enu`          | ENU world                           |
| output attitude     | ENU/FLU quaternion `[x,y,z,w]` (attitude commander) |
| output velocity     | ENU world `[vx,vy,vz]` (velocity commander) |

### 3a. Velocity must be rotated body → world  ⚠ (the runaway-into-the-wall bug) — BOTH commanders

`nav_msgs/Odometry` follows **REP-145**: `twist` is expressed in
`child_frame_id` (the **body** frame, `base_link` / FLU), while `pose` is in
`header.frame_id` (world). `mavros/local_position/odom` — the source feeding
`odometry_conversion` in sim — does exactly this: **its linear velocity is in
the body FLU frame.** `odometry_conversion.cpp` only rewrites frame IDs; it
never rotates the twist.

The starling deployment never hit this because it read velocity straight from
the raw MAVLink `LOCAL_POSITION_NED` (world NED) and converted NED→ENU world.

**Symptom if you forget:** at hover with the drone yawed (e.g. 130° after
FACE_GOAL), the body-frame velocity reads rotated ~130° from the true world
velocity. Inside the policy `v_local = Rz.T @ v_world` then double-rotates it,
so the velocity-damping term points the wrong way → positive feedback →
the drone accelerates away (we saw reported velocity with the *opposite sign*
of actual world displacement, ending in a geofence breach).

**Fix** — applied identically in **both** commanders' `odometry_callback`: rotate
the twist into world ENU using the orientation the message already carries (pose
orientation is FLU→ENU):

```python
R_flu_to_enu = Rotation.from_quat(self.drone.orientation).as_matrix()
self.drone.velocity = R_flu_to_enu @ np.array([v.x, v.y, v.z])
```

> The hardware `px4_interface` path happens to publish **world-ENU** velocity
> (`px4_interface.cpp` converts NED-world → ENU-world), so this *input* bug is
> MAVROS/sim-specific — but the rotation above is correct for both, because a
> true world velocity rotated by `R_flu_to_enu` would just be wrong. If you
> ever swap the odom source, **re-verify the twist frame** (see §10).

### 3b. Attitude output path (attitude commander)

- The policy returns both `attitude_ned_frd_wxyz` (PX4-ready) and
  `attitude_enu_flu_xyzw` (ENU/FLU).
- starling sent `attitude_ned_frd_wxyz` straight over MAVLink
  `SET_ATTITUDE_TARGET`.
- AirStack publishes **`attitude_enu_flu_xyzw`** to `AttitudeThrust` and lets
  the **interface** convert ENU/FLU → NED/FRD. Keep the conversion in one place
  (the interface) rather than duplicating it here.

### 3c. Velocity output path (velocity commander) ⚠ — frame differs by mode

The velocity actor emits **world-ENU** velocity, but the two interfaces consume
different frames, so `publish_velocity` branches on `drone_mode`:

- **`sim` (MAVROS):** `velocity_command` is interpreted as a **body** frame
  (`FRAME_BODY_NED` → FLU on the ROS side). The commander rotates world-ENU →
  yaw-aligned body FLU with `_world_to_body` (a yaw-only `Rz(yaw)^T`, so vertical
  speed maps straight to body z) before publishing; `header.frame_id =
  base_link`.
- **`real` (px4_interface):** `velocity_command` is consumed as world ENU
  directly — published unchanged, `header.frame_id = map`.

The **yaw-rate** (`twist.angular.z`, ENU CCW+) is frame-agnostic and correct for
both. The velocity actor does not emit a yaw setpoint; the commander tracks the
policy's desired heading (from the velocity EMA, §4) and runs a P-controller
(`yaw_kp`, clamped to `yaw_rate_max`) to produce the yaw-rate (`_yaw_rate_to`).

> **Mirrored lateral motion** in sim is the tell-tale sign of a `_world_to_body`
> sign error; forward motion is unaffected. Flip the convention there if obstacle
> avoidance looks mirrored.

### 3d. ENU↔NED constants (in `diffaero_core.py`, same as Pegasus)

- ENU inertial → NED inertial: `Rotation.from_quat([0.70711, 0.70711, 0, 0])`
- FLU body → FRD body: `+π` about X = `Rotation.from_quat([1, 0, 0, 0])`

---

## 4. Observation construction (`compute`)

Both cores share steps 1–3; they differ only in the assembled state vector and
(attitude only) the `uz` term.

1. `Rz` = yaw-only frame from `R_enu` (strip pitch/roll; columns = `[fwd, left, up]` in ENU).
2. `target_vel_world = (goal − pos) / max(dist/max_vel, 1)` → saturates to `max_vel`.
3. Project into yaw frame: `target_vel_local = Rz.T @ target_vel_world`, `v_local = Rz.T @ v_world`.
4. Assemble the state:
   - Attitude → `state9 = [target_vel_local, uz, v_local]`.
   - Velocity → `state6 = [target_vel_local, v_local]` (no `uz`).
5. **vel-EMA → yaw orientation fed to the actor.** Initialized from the
   *heading direction* (`Rz[:,0]`), not raw velocity, so transient drift or
   post-interruption velocity doesn't corrupt yaw on the first tick after a
   reset. Falls back to the forward axis when nearly stationary (`norm < 0.3`).
   - In the **velocity** core the same EMA also yields `desired_yaw_enu` (the
     heading the drone should nose into), which the commander turns into a
     yaw-rate (§3c). The actor was trained with `align_yaw_with_vel_ema=true`.

`policy.reset()` clears the vel-EMA — call it whenever continuity breaks (both
cores expose it; both commanders call it at `~/start` and on interruption).

---

## 5. Perception (both commanders)

Both commanders use the **`perception_encoded`** path: the `/{name}/perception/tof`
topic already carries a pre-encoded `9×16` grid (`0 = clear`, `1 = obstacle`),
so `PerceptionBuilder` is bypassed and intrinsics are dummies. `PerceptionBuilder`
(planar depth → crop to 86° FOV → planar-to-Euclidean → min-pool to 9×16 →
`1 − r/max_dist`) is retained for the raw-depth path. Encoding: **1 = surface at
the lens, 0 = nothing within `max_dist` (5 m)**. ToF older than `tof_timeout_s`
(0.5 s) is dropped (`perception_encoded=None` → zeros grid).

> **The attitude actor always consumes the grid; the velocity actor only consumes
> it when its checkpoint was trained with `env=obstacle_avoidance`** (e.g.
> `sha2c_vel_cmd_oa`). A state-only velocity checkpoint ignores perception
> entirely. Either way, **nothing in this repo publishes the ToF topic**, so by
> default both fly the zeros grid (no avoidance). See
> [`diffaero_experiment.md`](diffaero_experiment.md) for how to publish a fake
> all-clear grid or wire a real ToF/depth producer.

The velocity commander additionally republishes the grid as a colormapped
`sensor_msgs/Image` on `/svg/{name}/tof_image` (red = near, green = clear) for an
RViz/Foxglove Image panel.

---

## 6. Lifecycle state machine (identical for both)

`IDLE → ARMING → ASCEND → FACE_GOAL → ACTIVE → (LANDING)`

| State      | Behavior |
|------------|----------|
| `ARMING`   | Timed: request offboard (1.0 s), arm (1.5 s), done (2.5 s). Streams current pose to satisfy PX4's offboard-entry "must already be receiving setpoints" precondition. |
| `ASCEND`   | Pose command to `hover_positions`; transitions when within `arrival_threshold_m`. |
| `FACE_GOAL`| Yaw in place to point at `goal_position`, slewed at `face_goal_yaw_rate_max` so it rotates smoothly instead of snapping. Rotating *before* moving avoids digging a skid/leg into the ground and tripping a sim collision — same reasoning as starling's separate YAW phase. Transitions when yaw error < `face_goal_threshold_rad`. |
| `ACTIVE`   | Runs the policy (`mission_active`) or holds pose (idle). See §6a. |
| `LANDING`  | Velocity-down (`land_speed_mps`) until `land_complete_altitude_m`, then disarm. Both commanders land with a velocity command. |

Services (both): `~/takeoff`, `~/start`, `~/hold` (panic freeze), `~/land`,
`~/reset_fence`. The service namespace is the node name —
`/diffaero_commander/*` for the attitude commander, `/diffaero_velocity_commander/*`
for the velocity one.

### 6a. Goal handoff — policy is a cruise controller, not a hover controller ⚠ (both)

`~/start` pins `policy_goal = goal_position` (a real target), so
`target_vel = (goal − pos)` drives the drone toward it.

**On arrival we must leave the policy.** When within `goal_arrival_threshold_m`
(0.4 m) of the goal, the commander drops `mission_active`, sets
`hold_target = goal` / `hold_orientation = current`, and from then on streams a
**pose-hold** at the goal (stable PX4 position control) — via `pose_command` in
both commanders. This mirrors the starling deployment, which switched to PX4
`LAND` within 0.5 m. Without this, the policy overshoots the goal and oscillates
("goes crazy at the goal").

> A `goal` scenario lets you retarget live by publishing
> `/svg/{name}/goal_command` (PoseStamped) and `/svg/{name}/speed_command`
> (Float32) — handled identically by both commanders.

### 6b. Interruption handling (both)

If the policy was interrupted > 0.2 s (stale odom, fence, hold), `vel_ema` is
reset so the heading-direction init (§4.5) kicks in instead of accumulating
stale drift. We intentionally **do not** re-anchor the goal to the current
position here (that would make it hover wherever it got interrupted mid-cruise).

---

## 7. Policy warm-up (single-threaded executor) ⚠ (both)

Each node runs on `rclpy.spin` (**single-threaded executor**): the control-loop
timer and the odometry subscription share one thread. The **first**
`policy.compute()` pays ~0.5 s of TorchScript/CUDA JIT warm-up, which blocks the
thread → the odometry callback can't run → odom looks "stale" at the exact
moment of policy handoff → spurious `odometry stale` hold + `vel_ema` reset on
the very first tick of flight.

**Fix:** both commanders run a few dummy `compute()` calls (then `reset()`) in
`__init__`, while still constructing the node, so the first real tick is fast.
Look for `DiffAero … policy warmed up` at startup and the absence of an
`interrupted …s` line right after `scenario "…" running`.

---

## 8. Parameters and why they're set the way they are

### 8a. Shared (both commanders)

| Param | Value | Why |
|-------|-------|-----|
| `control_rate_hz` | 30.0 | Matches training `dt = 0.0333 s`. (Param default is 20; both configs set 30.) |
| `drone_position_offset` | `[0,0,0]` | `NUM_ROBOTS=1` → `drone_1` spawns at origin, so raw odom == world. Set to the spawn position for shifted spawns; leave zero for mocap. A zero offset logs a warning in sim precisely because it's wrong for a shifted spawn. |
| `goal_position` | ENU | Cruise target. Keep inside the geofence. |
| `max_vel` | 1.5 | Cruise speed cap: `target_vel = (goal−pos)` saturated to this. Training sampled **3–6 m/s** — far too fast/violent for a short indoor hop. Caps *cruise* speed, **not** from-rest acceleration. |
| `goal_arrival_threshold_m` | 0.4 | Distance at which to leave the cruise policy for a pose-hold (§6a). |
| `arrival_threshold_m` | 0.15 | ASCEND→FACE_GOAL transition radius. |
| `face_goal_threshold_rad` | 0.05 | FACE_GOAL→ACTIVE yaw-error gate. |
| `land_speed_mps` / `land_complete_altitude_m` | 0.3 / 0.15 | Descent rate and disarm altitude. |
| `tof_timeout_s` / `state_timeout_s` | 0.5 / 0.5 | Drop stale perception / fall back to stale-hold. |
| `fence_*` | box | Latching geofence: a breach freezes the drone and blocks `~/start` until `~/reset_fence`. |

### 8b. Attitude commander only (`config/diffaero_sim.yaml`)

| Param | Value | Why |
|-------|-------|-----|
| `max_accel` | 30.0 (config) / 20.0 (default) | Thrust-accel that maps to full throttle; hover throttle = `g/max_accel`. **Must match what the FCU expects** — starling set `MPC_THR_HOVER = g/max_accel` explicitly. If the drone slowly climbs/sinks at hover, tune this. |
| `max_acc_xy` | 6.0 (tamed) | Caps the policy's horizontal action scale → max tilt ≈ `atan(max_acc_xy/g)` ≈ **31°** (vs ~64° at the 20.0 default). Tames the launch lunge. **Deviates from the action limits the policy was exported with**, so tracking may be slightly off — raise toward 20 if it feels sluggish. |
| `max_acc_z` | 40.0 | Vertical action scale (z action limit). |

### 8c. Velocity commander only (`config/diffaero_vel_sim.yaml`)

| Param | Value | Why |
|-------|-------|-----|
| `max_vel_xy` | 1.0 (`-1` = checkpoint default) | Actor **action** limit on horizontal velocity — rescales the actor's `[-1,1]` tanh output. Distinct from `max_vel`, which only caps the *target* toward the goal. `-1` falls back to the value baked into the checkpoint's training config. |
| `max_vel_z` | 0.5 (`-1` = default) | Actor action limit on vertical velocity. |
| `vel_ema_factor` | `-1` = default (0.1) | EMA blend toward measured velocity that drives the desired-yaw heading (§4). |
| `yaw_kp` | 1.5 | P gain (1/s) from current heading toward the policy's desired heading → yaw-rate. |
| `yaw_rate_max` | 1.5 | Clamp (rad/s) on the commanded yaw-rate. |
| `face_goal_yaw_rate_max` | 0.8 | Slew limit (rad/s) while turning to face the goal in FACE_GOAL. |
| `vel_arrow_scale_s` | 0.5 | RViz: velocity-command arrow length = `vel_cmd · this` (where the command would carry the drone in 0.5 s). |

> **Checkpoint validation.** The velocity commander **loads only**
> `velocity_pointmass` / `action_is_velocity=true` checkpoints and raises
> `Expected dynamics.name=velocity_pointmass` otherwise — point an attitude
> checkpoint at `diffaero_commander` and vice-versa.

Tuning order for a new site: (1) confirm hover is stable (attitude: `max_accel`;
velocity: PX4 owns thrust, so skip), (2) set a modest `max_vel`, (3) tame the
launch (attitude: `max_acc_xy`; velocity: `max_vel_xy`), (4) widen
`goal_arrival_threshold_m` if it overshoots before the hold catches.

---

## 9. Differences vs `starling-deployment` (why the port needed work)

| Aspect | starling (worked) | AirStack |
|--------|-------------------|----------|
| Transport | direct MAVLink (pymavlink) | `robot_interface` (MAVROS/px4_interface) over ROS 2 |
| Velocity source | `LOCAL_POSITION_NED` (world NED → world ENU) | `mavros/local_position/odom` twist (**body FLU** → must rotate, §3a) |
| Attitude cmd | `SET_ATTITUDE_TARGET` (NED/FRD) | `AttitudeThrust` (ENU/FLU); interface converts (§3b) |
| Velocity cmd | n/a (attitude only) | `TwistStamped`; world→body rotate in sim, yaw-rate P-controller (§3c) |
| Arrival | switch to PX4 `LAND` < 0.5 m | switch to pose-hold < `goal_arrival_threshold_m` (§6a) |
| Executor | dedicated control loop thread | single-threaded `rclpy.spin` → needs warm-up (§7) |
| Hover throttle | `MPC_THR_HOVER` set explicitly | attitude: relies on `max_accel`; velocity: PX4 owns thrust |

---

## 10. Running and verifying

### 10.0 Full command sequence (sim) — containers → service calls

The pieces run in **separate terminals**: (A) Isaac Sim container, (B) the MAVROS
interface stack, (C) the commander, (D) the operator service calls. Terminals
B–D all run **inside the robot container** (via `./airstack.sh connect robot
--command=bash`). The interface script and the Isaac Sim launch script are a
matched pair (PX4 SITL ports / sysids), so start the sim first.

`.env` before you begin:
```
COMPOSE_PROFILES="desktop,isaac-sim"
AUTOLAUNCH="false"
NUM_ROBOTS="1"
```

**Terminal A — Isaac Sim (fresh terminal, host):**
```bash
cd ~/AirStack && ./airstack.sh up
./airstack.sh status        # robot-desktop-1 and isaac-sim Up
./airstack.sh connect isaac-sim --command=bash
```
Inside the isaac-sim container (`PLAY_SIM_ON_START=true` REQUIRED):
```bash
NUM_ROBOTS=1 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```
Wait for `Spawning 1 drone(s) on ROS domain 1` and `PX4 Autolaunch: True` before
proceeding.

**Terminal B — MAVROS interface (fresh terminal, host):**
```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
echo $ROS_DOMAIN_ID                       # 1
cd ~/AirStack/robot/ros_ws && bws && sws  # bws first time / after edits
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 1
```
Verify (another shell): `ros2 topic echo /drone_1/interface/mavros/state --once`
→ `connected: true`, then `ros2 topic hz /drone_1/odometry_conversion/odometry`
(~30 Hz after EKF converges, ~30 s).

**Terminal C — DiffAero commander (fresh terminal) — pick ONE:**
```bash
cd ~/AirStack/robot/ros_ws && sws
```
Attitude+thrust policy (`config/diffaero_sim.yaml`):
```bash
ros2 launch svg_ground_control diffaero_single.launch.py
#   override scenario:    scenario:=goal
#   hardware:             config:=<share>/config/diffaero_real.yaml use_mocap:=true
```
Velocity-command policy (`config/diffaero_vel_sim.yaml`):
```bash
ros2 launch svg_ground_control diffaero_velocity_single.launch.py
#   override scenario:    scenario:=goal
```
Watch for `DiffAero … policy loaded from …` and `… policy warmed up` at startup.

**Terminal D — fly (fresh terminal):**
```bash
cd ~/AirStack/robot/ros_ws && sws
```
Attitude commander (`/diffaero_commander/*`):
```bash
ros2 service call /diffaero_commander/takeoff std_srvs/srv/Trigger   # arm+ascend+face goal+hold
ros2 service call /diffaero_commander/start   std_srvs/srv/Trigger   # cruise to goal, auto-hold on arrival
ros2 service call /diffaero_commander/hold    std_srvs/srv/Trigger   # PANIC freeze
ros2 service call /diffaero_commander/land    std_srvs/srv/Trigger   # descend+disarm
```
Velocity commander (`/diffaero_velocity_commander/*`):
```bash
ros2 service call /diffaero_velocity_commander/takeoff std_srvs/srv/Trigger
ros2 service call /diffaero_velocity_commander/start   std_srvs/srv/Trigger
ros2 service call /diffaero_velocity_commander/hold    std_srvs/srv/Trigger
ros2 service call /diffaero_velocity_commander/land    std_srvs/srv/Trigger
```

> **Sequencing:** `takeoff` → drone ascends, yaws to face the goal, logs
> `… → ACTIVE` and holds → `start` (begins cruising to `goal_position`) → on
> arrival it auto-holds at the goal (§6a) → `land`. `start` is rejected until the
> drone is holding in `ACTIVE`, and is blocked while a geofence breach is latched.

**Terminal E — fake ToF publisher (fresh terminal, optional):**

The policy expects a `9×16` all-clear grid on `/{name}/perception/tof`. Nothing
publishes it in this repo, so the commander substitutes zeros automatically after
`tof_timeout_s` — but publishing explicitly keeps `tof_fresh=True` in the log
and avoids the per-tick warning. Must be running before `start`.

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
ros2 topic pub --rate 10 /drone_1/perception/tof std_msgs/msg/Float32MultiArray \
  "{data: [$(python3 -c 'print(",".join(["0.0"]*144))')]}"
```
Confirm: `ros2 topic hz /drone_1/perception/tof` → ~10 Hz.

**Terminal F — RViz (fresh terminal, optional):**

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```
Fixed Frame = `map`; shows the drone sphere (cyan in sim, red on hardware),
heading arrow, goal point, and the geofence box (green normally, red when
latched). The velocity commander also draws an orange velocity-command arrow. If
starting bare: Add → By topic → `/svg/viz/markers`.

**Sanity checks** (any shell in the robot container):

- **Velocity frame (§3a):** with the drone yawed and drifting toward world `+x`,
  the policy log's `vel=[...]` must read mostly `+x`. If its sign is opposite to
  the actual motion, the twist is body-frame and the §3a rotation is missing.
  ```bash
  ros2 topic echo /drone_1/odometry_conversion/odometry --field twist.twist.linear --once
  ```
- **Warm-up (§7):** expect `DiffAero … policy warmed up` at startup and **no**
  `odometry stale / interrupted` line right at handoff.
- **Launch tilt (attitude, §8b):** first `att_euler_rpy` pitch ≈ 30°, not 40°+.
- **Velocity output (vel, §3c):** the `[policy] vel_cmd_enu=…` log should point
  toward the goal; in sim, lateral commands that look mirrored mean a
  `_world_to_body` sign error.
- **Arrival (§6a):** `reached goal … → HOLD`, then a steady hover.
- **Odom rate:** `ros2 topic hz /drone_1/odometry_conversion/odometry` should be
  steady near the control rate; recurring mid-flight gaps (not the startup one)
  indicate a real stream problem, not warm-up.

---

## 11. Known-open / watch items

- **`max_acc_xy` cap deviates from training (attitude)** — fine for gentle indoor
  hops, but if obstacle avoidance is later enabled the policy may need its full
  action authority back. Treat 6.0 as a comfort/safety clamp, not a permanent
  value. The velocity analogue is `max_vel_xy`.
- **Hover-throttle matching (attitude)** — `max_accel` vs the FCU's hover param is
  the most likely cause of slow altitude drift at hover; verify per airframe. The
  velocity commander sidesteps this entirely — PX4 owns thrust.
- **Velocity world→body sign (sim)** — `_world_to_body` is yaw-only; if obstacle
  avoidance / lateral motion ever looks mirrored, that rotation's sign is the
  first suspect (§3c).
- **Single goal only (both)** — `policy_goal` is a fixed point. Multi-waypoint /
  moving goals would need a carrot/waypoint feeder in the control loop.
- **No `diffaero_real.yaml` checked in** — hardware runs copy a sim config and set
  `drone_mode: real` (see [`diffaero_experiment.md`](diffaero_experiment.md)
  Part B).
