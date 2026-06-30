# DiffAero on AirStack

This documents how the **DiffAero** learned flight policy (SHA2C, continuous
point-mass) is wired into AirStack, and — more importantly — **why** each
non-obvious detail is the way it is. DiffAero ships as **two commander
executables** that wrap the *same* policy core but differ in what the network
outputs and how it reaches the flight controller:


| Commander           | Node                          | Policy output                                                       | Source                                                             | FCU owns            |
| ------------------- | ----------------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------------ | ------------------- |
| **attitude+thrust** | `diffaero_commander`          | attitude quaternion + normalized thrust (`mav_msgs/AttitudeThrust`) | `diffaero_commander.py` + `diffaero/diffaero_core.py`              | rate loop only      |
| **velocity**        | `diffaero_velocity_commander` | world-ENU velocity setpoint (`geometry_msgs/TwistStamped`)          | `diffaero_velocity_commander.py` + `diffaero/diffaero_vel_core.py` | attitude + velocity |


The attitude commander gives aggressive/agile flight (the network closes the
attitude loop); the velocity commander is tamer and easier to trust (PX4 owns
attitude and thrust). They share lifecycle, geofence, RViz markers, scenario
plumbing, warm-up, and the perception path — only the **action space** and its
**output plumbing** differ. Most of these notes exist because the attitude
policy was first deployed standalone against PX4 over MAVLink in
`superfly/starling-deployment`, and porting it onto AirStack's `robot_interface`
abstraction surfaced a series of frame / timing / controller mismatches. Each is
recorded below so the next person doesn't re-derive them.

> **How-to-run:** §10 below — three modes in one place. **Why** each non-obvious
> detail is the way it is lives in §§1–9. Read **§6c before any hardware session**
> (what arms the drone, kill switch, emergency stops).

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

- `sim`  → `/{name}/interface/`* (MAVROS/SITL)
- `real` → `/{name}/fmu/*` (px4_interface / uXRCE-DDS)

---

## 3. Frame conventions (the most important section)

The policy lives entirely in **ENU world / FLU body**:


| Quantity        | Frame expected by policy                            |
| --------------- | --------------------------------------------------- |
| `position_enu`  | ENU world                                           |
| `velocity_enu`  | **ENU world** (measured, not finite-differenced)    |
| `R_enu`         | FLU-body → ENU-world rotation matrix                |
| `goal_enu`      | ENU world                                           |
| output attitude | ENU/FLU quaternion `[x,y,z,w]` (attitude commander) |
| output velocity | ENU world `[vx,vy,vz]` (velocity commander)         |


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
- AirStack publishes `**attitude_enu_flu_xyzw`** to `AttitudeThrust` and lets
the **interface** convert ENU/FLU → NED/FRD. Keep the conversion in one place
(the interface) rather than duplicating it here.

### 3c. Velocity output path (velocity commander) ⚠ — frame differs by mode

The velocity actor emits **world-ENU** velocity, but the two interfaces consume
different frames, so `publish_velocity` branches on `drone_mode`:

- `**sim` (MAVROS):** `velocity_command` is interpreted as a **body** frame
(`FRAME_BODY_NED` → FLU on the ROS side). The commander rotates world-ENU →
yaw-aligned body FLU with `_world_to_body` (a yaw-only `Rz(yaw)^T`, so vertical
speed maps straight to body z) before publishing; `header.frame_id = base_link`.
- `**real` (px4_interface):** `velocity_command` is consumed as world ENU
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

Both commanders use the `**perception_encoded`** path: the `/{name}/perception/tof`
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
> `[diffaero_experiment.md](diffaero_experiment.md)` for how to publish a fake
> all-clear grid, or wire a real Starling ToF via the **UDP pipeline** in
> `~/tof_streamer/README.md` (`tof_udp_stream.cpp` on the VOXL →
> `ground/tof_udp_bridge.py` on the ground PC).
> **Do not** subscribe from Jazzy on `ROS_DOMAIN_ID=0` to the VOXL's Foxy DDS
> topics — that crashes `voxl_mpa_to_ros2` with `deserialize_change` /
> `std::bad_alloc` (§10.2 / §10.4).

The velocity commander additionally republishes the 9×16 grid as a colormapped
`sensor_msgs/Image` on `/svg/{name}/tof_image` (hot/cold: red = close, blue = far,
black = no return / 0.0; upscaled for RViz). Wire live ToF via UDP (§10.2 or §10.3);

---

## 6. Lifecycle state machine (identical for both)

`IDLE → ARMING → ASCEND → FACE_GOAL → ACTIVE → (LANDING)`


| State       | Behavior                                                                                                                                                                                                                                                                                                                            |
| ----------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ARMING`    | Timed: request offboard (1.0 s), arm (1.5 s), done (2.5 s). Streams current pose to satisfy PX4's offboard-entry "must already be receiving setpoints" precondition.                                                                                                                                                                |
| `ASCEND`    | Pose command to `hover_positions`; transitions when within `arrival_threshold_m`.                                                                                                                                                                                                                                                   |
| `FACE_GOAL` | Yaw in place to point at `goal_position`, slewed at `face_goal_yaw_rate_max` so it rotates smoothly instead of snapping. Rotating *before* moving avoids digging a skid/leg into the ground and tripping a sim collision — same reasoning as starling's separate YAW phase. Transitions when yaw error < `face_goal_threshold_rad`. |
| `ACTIVE`    | Runs the policy (`mission_active`) or holds pose (idle). See §6a.                                                                                                                                                                                                                                                                   |
| `LANDING`   | Velocity-down (`land_speed_mps`) until `land_complete_altitude_m`, then disarm. Both commanders land with a velocity command.                                                                                                                                                                                                       |


Services (both): `~/takeoff`, `~/start`, `~/hold` (panic freeze), `~/land`,
`~/reset_fence`. The service namespace is the node name —
`/diffaero_commander/*` for the attitude commander, `/diffaero_velocity_commander/*`
for the velocity one.

> **Only `~/takeoff` arms the drone** (via timed `REQUEST_CONTROL` + `ARM`).
> See §6c for the full safety breakdown.

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

### 6c. Safety — what arms the drone and when motors can spin ⚠ (read before hardware)

DiffAero bypasses AirStack's stock `drone_safety_monitor`. Treat every step
below as a preflight checklist. **The RC kill switch is the true motor cutoff**
(geofence and `~/hold` only freeze setpoints — they do not disarm).

#### What does **not** arm (safe to run for bring-up / tracking checks)

These can (and should) be running **before** you are ready to fly. None of them
call `RobotCommand.ARM` or spin propellers by themselves:


| Action                                                                                                  | Why it is safe                                                                                                                                                                                             |
| ------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `MicroXRCEAgent udp4 -p 8888`                                                                           | DDS bridge only; no flight commands                                                                                                                                                                        |
| `ros2 launch natnet_ros2 natnet_ros2.launch.py`                                                         | Publishes mocap poses only                                                                                                                                                                                 |
| `ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1`                              | `px4_interface` publishes an offboard **heartbeat** (`in/offboard_control_mode` @ 10 Hz) but **does not arm** and does not send setpoints until something publishes on `velocity_command` / `pose_command` |
| `ros2 launch … diffaero_*_single.launch.py use_mocap:=true` (commander **without** calling `~/takeoff`) | Commander stays in `IDLE` — the control loop does **nothing** (no setpoints published). `mocap_bridge` only feeds external vision into PX4 EKF2                                                            |
| `rviz2 -d …/svg_drones.rviz`                                                                            | Visualization only                                                                                                                                                                                         |
| `ros2 topic pub … /drone_1/perception/tof …`                                                            | Perception input only; commander ignores it until armed + `~/start`                                                                                                                                        |
| Hand-carry test: move drone, watch `/svg/viz/markers` or `…/odometry_conversion/odometry`               | Confirms mocap → EKF → interface chain with **motors disarmed**                                                                                                                                            |


> **Preflight pattern:** bring up agent + NatNet + `real_interfaces` + commander
> with `use_mocap:=true`, open RViz, carry the drone — its marker should track.
> **Do not call `~/takeoff` until this passes.**

#### What **does** arm (or can spin motors once armed)


| Trigger                                                                                               | When                    | Effect                                                                                 |
| ----------------------------------------------------------------------------------------------------- | ----------------------- | -------------------------------------------------------------------------------------- |
| `**ros2 service call /diffaero_commander/takeoff …`** or `**/diffaero_velocity_commander/takeoff …**` | You explicitly call it  | Starts the arming sequence (see timeline below). **This is the intended arming path.** |
| `**ros2 service call /{name}/fmu/robot_command` with `command: 1` (ARM)**                             | Manual / scripted       | Arms immediately — bypasses DiffAero lifecycle. **Avoid unless debugging.**            |
| `**ros2 service call /{name}/fmu/robot_command` with `command: 0` (REQUEST_CONTROL)**                 | Manual / scripted       | Requests offboard only; still needs ARM to spin motors                                 |
| **RC transmitter arm switch/stick**                                                                   | Operator on the radio   | Arms PX4 independently of DiffAero — configure a **kill switch** before flying         |
| **QGroundControl / `px4-param` arm**                                                                  | Ground-station operator | Same as RC — outside DiffAero's control                                                |


**Automatic arming timeline** (after a successful `~/takeoff` service call):

```
t = 0.0 s   state → ARMING; pose setpoints stream at current position
              (satisfies PX4 offboard-entry precondition)
t = 1.0 s   RobotCommand.REQUEST_CONTROL  →  px4_interface requests OFFBOARD
t = 1.5 s   RobotCommand.ARM              →  px4_interface sends ARM  ⚠ PROPS CAN SPIN
t = 2.5 s   state → ASCEND; pose setpoints climb toward hover_positions
…           FACE_GOAL → ACTIVE (hold at goal heading)
```

Once `**ARM` succeeds** (t ≥ 1.5 s), PX4 will track whatever setpoints the
commander publishes (`pose_command` during ascend/hold, `velocity_command` during
cruise/land). `**~/start` does not arm** — it only enables the cruise policy
after the drone is already airborne and holding in `ACTIVE`.

#### Motor activity after arming (not separate arming steps, but dangerous)


| Service / state                                     | Motor behavior                                                                    |
| --------------------------------------------------- | --------------------------------------------------------------------------------- |
| `~/takeoff` (post-ARM)                              | Climb via `pose_command` to `hover_positions`                                     |
| `~/start`                                           | Cruise policy publishes `velocity_command` — **active flight**                    |
| `~/hold`                                            | Freezes at current pose (`pose_command`); **still armed**                         |
| Geofence breach                                     | Same as hold — **still armed**, frozen in place                                   |
| `~/land`                                            | Descending `velocity_command` until `land_complete_altitude_m`, then `**DISARM`** |
| Stale odometry (> `state_timeout_s`) while airborne | Pose-hold at last target — **still armed**                                        |


`**~/land` is the normal disarm path.** On touchdown the commander sends
`RobotCommand.DISARM`. If landing aborts mid-descent, the drone stays armed.

#### Emergency stops (in priority order)

1. **RC kill switch** — cuts motors (configure before first flight).
2. `**ros2 service call /diffaero_*_commander/hold std_srvs/srv/Trigger`** — freeze
  setpoints; does **not** disarm.
3. `**ros2 service call /{name}/fmu/robot_command airstack_msgs/srv/RobotCommand "{command: 2}"`**
  — direct **DISARM** via px4_interface (hardware).
4. Physical power-off / battery disconnect.

#### Sim vs real arming note

In **sim**, the same lifecycle and `~/takeoff` → `REQUEST_CONTROL` → `ARM`
sequence applies via MAVROS (`/{name}/interface/robot_command`). The sim stack
is not “safer” — treat `~/takeoff` in sim as arming a real vehicle when
validating hardware workflows.

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


| Param                                         | Value      | Why                                                                                                                                                                                                                          |
| --------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `control_rate_hz`                             | 30.0       | Matches training `dt = 0.0333 s`. (Param default is 20; both configs set 30.)                                                                                                                                                |
| `drone_position_offset`                       | `[0,0,0]`  | `NUM_ROBOTS=1` → `drone_1` spawns at origin, so raw odom == world. Set to the spawn position for shifted spawns; leave zero for mocap. A zero offset logs a warning in sim precisely because it's wrong for a shifted spawn. |
| `goal_position`                               | ENU        | Cruise target. Keep inside the geofence.                                                                                                                                                                                     |
| `max_vel`                                     | 1.5        | Cruise speed cap: `target_vel = (goal−pos)` saturated to this. Training sampled **3–6 m/s** — far too fast/violent for a short indoor hop. Caps *cruise* speed, **not** from-rest acceleration.                              |
| `goal_arrival_threshold_m`                    | 0.4        | Distance at which to leave the cruise policy for a pose-hold (§6a).                                                                                                                                                          |
| `arrival_threshold_m`                         | 0.15       | ASCEND→FACE_GOAL transition radius.                                                                                                                                                                                          |
| `face_goal_threshold_rad`                     | 0.05       | FACE_GOAL→ACTIVE yaw-error gate.                                                                                                                                                                                             |
| `land_speed_mps` / `land_complete_altitude_m` | 0.3 / 0.15 | Descent rate and disarm altitude.                                                                                                                                                                                            |
| `tof_timeout_s` / `state_timeout_s`           | 0.5 / 0.5  | Drop stale perception / fall back to stale-hold.                                                                                                                                                                             |
| `fence_*`                                     | box        | Latching geofence: a breach freezes the drone and blocks `~/start` until `~/reset_fence`.                                                                                                                                    |


### 8b. Attitude commander only (`config/diffaero_sim.yaml`)


| Param        | Value                          | Why                                                                                                                                                                                                                                                                                    |
| ------------ | ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `max_accel`  | 30.0 (config) / 20.0 (default) | Thrust-accel that maps to full throttle; hover throttle = `g/max_accel`. **Must match what the FCU expects** — starling set `MPC_THR_HOVER = g/max_accel` explicitly. If the drone slowly climbs/sinks at hover, tune this.                                                            |
| `max_acc_xy` | 6.0 (tamed)                    | Caps the policy's horizontal action scale → max tilt ≈ `atan(max_acc_xy/g)` ≈ **31°** (vs ~64° at the 20.0 default). Tames the launch lunge. **Deviates from the action limits the policy was exported with**, so tracking may be slightly off — raise toward 20 if it feels sluggish. |
| `max_acc_z`  | 40.0                           | Vertical action scale (z action limit).                                                                                                                                                                                                                                                |


### 8c. Velocity commander only (`config/diffaero_vel_sim.yaml`)


| Param                    | Value                           | Why                                                                                                                                                                                                                                         |
| ------------------------ | ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `max_vel_xy`             | 1.0 (`-1` = checkpoint default) | Actor **action** limit on horizontal velocity — rescales the actor's `[-1,1]` tanh output. Distinct from `max_vel`, which only caps the *target* toward the goal. `-1` falls back to the value baked into the checkpoint's training config. |
| `max_vel_z`              | 0.5 (`-1` = default)            | Actor action limit on vertical velocity.                                                                                                                                                                                                    |
| `vel_ema_factor`         | `-1` = default (0.1)            | EMA blend toward measured velocity that drives the desired-yaw heading (§4).                                                                                                                                                                |
| `yaw_kp`                 | 1.5                             | P gain (1/s) from current heading toward the policy's desired heading → yaw-rate.                                                                                                                                                           |
| `yaw_rate_max`           | 1.5                             | Clamp (rad/s) on the commanded yaw-rate.                                                                                                                                                                                                    |
| `face_goal_yaw_rate_max` | 0.8                             | Slew limit (rad/s) while turning to face the goal in FACE_GOAL.                                                                                                                                                                             |
| `vel_arrow_scale_s`      | 0.5                             | RViz: velocity-command arrow length = `vel_cmd · this` (where the command would carry the drone in 0.5 s).                                                                                                                                  |


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


| Aspect          | starling (worked)                            | AirStack                                                              |
| --------------- | -------------------------------------------- | --------------------------------------------------------------------- |
| Transport       | direct MAVLink (pymavlink)                   | `robot_interface` (MAVROS/px4_interface) over ROS 2                   |
| Velocity source | `LOCAL_POSITION_NED` (world NED → world ENU) | `mavros/local_position/odom` twist (**body FLU** → must rotate, §3a)  |
| Attitude cmd    | `SET_ATTITUDE_TARGET` (NED/FRD)              | `AttitudeThrust` (ENU/FLU); interface converts (§3b)                  |
| Velocity cmd    | n/a (attitude only)                          | `TwistStamped`; world→body rotate in sim, yaw-rate P-controller (§3c) |
| Arrival         | switch to PX4 `LAND` < 0.5 m                 | switch to pose-hold < `goal_arrival_threshold_m` (§6a)                |
| Executor        | dedicated control loop thread                | single-threaded `rclpy.spin` → needs warm-up (§7)                     |
| Hover throttle  | `MPC_THR_HOVER` set explicitly               | attitude: relies on `max_accel`; velocity: PX4 owns thrust            |


---

## 10. Running DiffAero — three modes

Pick one mode. All commands assume `NUM_ROBOTS=1`, `drone_1`, `ROS_DOMAIN_ID=1`,
and the **robot container** (`./airstack.sh connect robot --command=bash`) unless
noted.

### Mode overview


|                      | **A — Sim only**                          | **B — Sim + real ToF**                     | **C — Real world**                 |
| -------------------- | ----------------------------------------- | ------------------------------------------ | ---------------------------------- |
| **What flies**       | Isaac Sim + PX4 SITL                      | Same as A                                  | Physical drone                     |
| **Odometry**         | MAVROS → `…/odometry_conversion/odometry` | Same as A                                  | OptiTrack → EKF2 → same topic      |
| **ToF / perception** | None (zeros grid) or fake publisher       | Live 9×16 from **bench** Starling over UDP | Optional UDP from onboard ToF      |
| `**drone_mode`**     | `sim` (default configs)                   | `sim`                                      | `real` (copy config, §10.3)        |
| **Arms**             | SITL in Isaac (`~/takeoff`)               | SITL only — bench drone stays passive      | **Real motors** (`~/takeoff`, §6c) |
| **Section**          | §10.1                                     | §10.2                                      | §10.3                              |


**Commander choice:** velocity policy is the usual path (`diffaero_velocity_single.launch.py`,
`config/diffaero_vel_sim.yaml`). Attitude policy:
`diffaero_single.launch.py` + `config/diffaero_sim.yaml`.

**Obstacle avoidance:** velocity checkpoint must be trained with
`env=obstacle_avoidance` (e.g. `sha2c_vel_cmd_oa`) for ToF to affect the policy.

**Fly sequence (all modes)** — service namespace is `/diffaero_velocity_commander/`*
(or `/diffaero_commander/*` for attitude):

```bash
ros2 service call /diffaero_velocity_commander/takeoff std_srvs/srv/Trigger   # arm → ascend → face goal → hold
ros2 service call /diffaero_velocity_commander/start   std_srvs/srv/Trigger   # cruise (after ACTIVE)
ros2 service call /diffaero_velocity_commander/hold    std_srvs/srv/Trigger   # panic freeze (still armed)
ros2 service call /diffaero_velocity_commander/land    std_srvs/srv/Trigger   # descend → disarm
```

`~/start` is rejected until the drone is holding in `ACTIVE`. On goal arrival the
commander auto-holds (§6a). `**~/takeoff` arms at t ≈ 1.5 s** — see §6c.

---

### 10.1 Mode A — Sim only

Isaac Sim provides state and the simulated FCU. No real hardware. ToF is absent
unless you add the fake publisher (Terminal E) — the policy then flies with a
zeros grid (no avoidance).

#### `.env` (before starting containers)

```
COMPOSE_PROFILES="desktop,isaac-sim"
AUTOLAUNCH="false"
NUM_ROBOTS="1"
```

#### Terminals


| ID    | Where               | Command                                                             |
| ----- | ------------------- | ------------------------------------------------------------------- |
| **A** | isaac-sim container | Start sim (see below)                                               |
| **B** | robot container     | `launch_sim_interfaces.sh 1`                                        |
| **C** | robot container     | `ros2 launch svg_ground_control diffaero_velocity_single.launch.py` |
| **D** | robot container     | Fly services (above)                                                |
| **E** | robot container     | *Optional* fake ToF publisher                                       |
| **F** | robot container     | *Optional* RViz                                                     |


**A — Isaac Sim** (host: `./airstack.sh up`, then connect to isaac-sim):

```bash
NUM_ROBOTS=1 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```

Wait for `Spawning 1 drone(s) on ROS domain 1` before continuing.

**B — MAVROS interface** (robot container):

```bash
cd ~/AirStack/robot/ros_ws && bws && sws
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 1
```

Verify: `ros2 topic echo /drone_1/interface/mavros/state --once` → `connected: true`;
`ros2 topic hz /drone_1/odometry_conversion/odometry` → ~30 Hz (after ~30 s EKF settle).

**C — Commander:**

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control diffaero_velocity_single.launch.py
# scenario:=goal   — live goal retargeting
```

Watch for `DiffAero … policy loaded` and `… policy warmed up` at startup (§7).

**E — Fake ToF** (optional; skip if using Mode B):

```bash
ros2 topic pub --rate 10 /drone_1/perception/tof std_msgs/msg/Float32MultiArray \
  "{data: [$(python3 -c 'print(",".join(["0.0"]*144))')]}"
```

**F — RViz** (optional):

```bash
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```

Fixed frame `map`; topics `/svg/viz/markers`, `/svg/drone_1/tof_image` (velocity commander).

#### Sim sanity checks


| Check                                                 | Pass                               |
| ----------------------------------------------------- | ---------------------------------- |
| `ros2 topic hz /drone_1/odometry_conversion/odometry` | ~30 Hz, steady in flight           |
| Policy log `vel=[…]` sign vs motion (§3a)             | Matches world direction when yawed |
| No `odometry stale` at first policy tick (§7)         | Warm-up ran at startup             |
| `reached goal … → HOLD` (§6a)                         | Stable hover at goal               |


---

### 10.2 Mode B — Sim + real ToF (perception only)

Everything from **§10.1 Mode A**, plus a **bench Starling** streaming ToF over
Wi-Fi. Isaac still flies the sim drone; the real drone runs **sensor software only**
(no uXRCE flight link, no mocap, no ground commands to the real FCU).

```
BENCH DRONE (props safe)                    GROUND PC (domain 1)
┌──────────────────────────┐               ┌─────────────────────────────────┐
│ voxl-tof-server            │               │ Isaac + MAVROS + DiffAero (sim) │
│ tof_udp_stream.cpp         │  Wi-Fi UDP    │ tof_udp_bridge.py               │
│  → 9×16 TOF2 (~592 B)      │ ── :5600 ──▶  │  → /drone_1/perception/tof      │
└──────────────────────────┘               └─────────────────────────────────┘
```

**Why UDP, not ROS on the link:** VOXL is Foxy; ground is Jazzy. Jazzy on
`ROS_DOMAIN_ID=0` crashes `voxl_mpa_to_ros2`. See §10.4 if you need `GROUND_IP`.

#### Extra terminals (on top of §10.1 A–D)


| ID          | Where           | What                |
| ----------- | --------------- | ------------------- |
| **T-ToF-D** | VOXL (SSH)      | `tof_udp_stream`    |
| **T-ToF-G** | robot container | `tof_udp_bridge.py` |


Skip §10.1 Terminal E (fake ToF). Use obstacle-avoidance checkpoint
(`sha2c_vel_cmd_oa` in `diffaero_vel_sim.yaml`).

**T-ToF-D — on the VOXL** (set `GROUND_IP` first — §10.4):

```bash
export GROUND_IP=<ground_pc_wifi_ip>
scp ~/tof_streamer/tof_udp_stream.cpp root@$VOXL_IP:/home/root/
ssh root@$VOXL_IP
systemctl status voxl-tof-server && voxl-inspect-pipe tof
cd /home/root
g++ -std=c++14 -O2 tof_udp_stream.cpp \
    -o tof_udp_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm
./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100
```

**Starling 2 crop:** default center crop includes ceiling in the 9×16 grid. Bench-tuned
on Starling 2 Max (240×180, `pmd-tof-liow2`): **bottom anchor, `v_shift=-100`**
(see `~/tof_streamer/ground/tof_crop_tune.py` to re-tune per mount). Use the command
above for Mode B and Mode C unless you re-run crop tuning.

`flip-h` + `flip-v` are **on by default** (Starling 2). Pass `--no-flip-h` /
`--no-flip-v` only if the image still looks mirrored.

**T-ToF-G — robot container:**

```bash
cd ~/tof_streamer
ROS_DOMAIN_ID=1 python3 ground/tof_udp_bridge.py --port 5600 \
  --topic /drone_1/perception/tof
ros2 topic hz /drone_1/perception/tof    # ~10–30 Hz, 144 floats
```

#### Do **not** run on the bench drone


| Blocked                                                                | Why                                       |
| ---------------------------------------------------------------------- | ----------------------------------------- |
| `MicroXRCEAgent`, `real_interfaces`, mocap                             | Would connect real FCU to ground stack    |
| `drone_mode: real` or commands to `/drone_1/fmu/`* for the bench drone | Would arm the real airframe               |
| `ROS_DOMAIN_ID=0` probing from Jazzy                                   | Crashes Foxy bridge on VOXL               |
| QGC / RC arm on bench drone                                            | Independent of DiffAero — keep props safe |


Sim `~/takeoff` arms **SITL only** (§6c).

#### Mode B checklist


| Step                                        | Pass                                             |
| ------------------------------------------- | ------------------------------------------------ |
| `./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100` on VOXL  | Running, no crash                                |
| `/drone_1/perception/tof` @ domain 1        | ~10–30 Hz                                        |
| Sim odom @ ~30 Hz                           | From SITL                                        |
| After `~/start`, log shows `tof_fresh=True` | Not stale zeros                                  |
| Wave hand at **real** sensor                | `/svg/drone_1/tof_image` or sim avoidance reacts |


---

### 10.3 Mode C — Real world

Physical drone flies via **uXRCE-DDS** (`/{name}/fmu/*`) with **OptiTrack mocap**
feeding PX4 EKF2. DiffAero commanders use `drone_mode: real` and publish to
`/{name}/fmu/velocity_command` (not MAVROS).

> **One-time setup:** [experiment.md Part B](experiment.md#part-b--bring-in-a-real-drone-connect--verify) (B0–B4b: VOXL Wi-Fi, `voxl_setup_real_drone.sh`, EKF2 params, Motive rigid body `drone_1`, frame check). **Read §6c** before arming.

#### Checked-in files (Mode C)

| File | Purpose |
|------|---------|
| [`config/diffaero_vel_real.yaml`](config/diffaero_vel_real.yaml) | `drone_mode: real`, fence, mocap_bridge, OA checkpoint |
| [`launch/diffaero_real.launch.py`](launch/diffaero_real.launch.py) | NatNet + `real_interfaces` + mocap_bridge + velocity commander |
| [`scripts/preflight_diffaero_mode_c.sh`](scripts/preflight_diffaero_mode_c.sh) | Automated pre-arm topic checks |

Edit **`diffaero_vel_real.yaml`** for your capture volume (`fence_*`, `goal_position`, `hover_positions`).

#### Data flow

```
Motive → natnet_ros2 → /drone_1/pose → mocap_bridge → /drone_1/fmu/in/vehicle_visual_odometry
  → PX4 EKF2 → px4_interface → /drone_1/odometry_conversion/odometry → diffaero_velocity_commander
When armed: commander → /drone_1/fmu/velocity_command | pose_command | robot_command
Optional ToF: VOXL tof_udp_stream → UDP :5600 → tof_udp_bridge.py → /drone_1/perception/tof
```

---

#### Phase 0 — One-time (per drone / lab)

Do once per airframe or after re-flash. Details in **experiment.md Part B**.

| Step | Where | Action | Verify |
|------|--------|--------|--------|
| **0a** | VOXL | Wi-Fi on drone subnet (`voxl-wifi`, DHCP) | `ip addr show wlan0` → e.g. `192.168.123.167` |
| **0b** | VOXL | `voxl_setup_real_drone.sh drone_1 <GROUND_IP> 1 8888` | `px4-microdds_client status` → connected |
| **0c** | QGC / `px4-param` | `EKF2_EV_CTRL=11`, `EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0`, RC kill switch | Params saved |
| **0d** | Motive | Rigid body named **`drone_1`**, Up axis Z, NatNet streaming | Motive shows live track |
| **0e** | Ground | `bws --packages-select natnet_ros2 svg_ground_control && sws` | Packages build |
| **0f** | VOXL (optional ToF) | `voxl-camera-server` running, `pmd-tof-liow2` in conf | `timeout 3s ~/tof_stream` → `240x180` |

**Do not** run `MicroXRCEAgent`, mocap, or DiffAero on the VOXL for Mode C — the ground PC runs the stack; the VOXL runs only PX4 + uXRCE client (+ optional `tof_udp_stream`).

---

#### Phase 1 — Ground container (every session)

Robot container must use **host network** (NatNet multicast + drone Wi-Fi + uXRCE UDP).

```bash
# Host (.env): COMPOSE_PROFILES includes robot-desktop, AUTOLAUNCH=false, NUM_ROBOTS=1
cd ~/AirStack && AUTOLAUNCH=false airstack up robot-desktop
./airstack.sh connect robot --command=bash
export ROS_DOMAIN_ID=1
cd ~/AirStack/robot/ros_ws && sws
```

| Check | Command | Pass |
|-------|---------|------|
| Domain | `echo $ROS_DOMAIN_ID` | `1` |
| Workspace | `ros2 pkg prefix svg_ground_control` | path under `install/` |

---

#### Phase 2 — Bring-up terminals (no flight until Phase 4 passes)

Open **separate tmux panes**. Nothing here arms motors (§6c).

| ID | Terminal | Command | Check before next step |
|----|----------|---------|------------------------|
| **T0** | robot | `MicroXRCEAgent udp4 -p 8888 -v4` | Log: `session established` + VOXL IP |
| **T1** | robot | See **T1 option A or B** below | All nodes running |
| **T2** | robot (opt.) | RViz: `rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz` | Fixed frame `map` |
| **T-ToF-G** | robot (opt.) | `cd ~/tof_streamer && ROS_DOMAIN_ID=1 python3 ground/tof_udp_bridge.py --port 5600 --topic /drone_1/perception/tof` | Bridge listening :5600 |
| **T-ToF-D** | VOXL SSH | `./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100` (after T-ToF-G) | `streaming … Hz (9x16 perception)` |

**T1 option A — single launch (recommended):**

```bash
ros2 launch svg_ground_control diffaero_real.launch.py
# Override Motive IPs if needed:
#   natnet_server_ip:=192.168.123.199 natnet_client_ip:=192.168.123.134
```

**T1 option B — manual (same as A, split for debugging):**

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py serverIP:=192.168.123.199 clientIP:=192.168.123.134
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1
ros2 launch svg_ground_control diffaero_velocity_single.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/diffaero_vel_real.yaml \
  use_mocap:=true
```

**T0 verify (ground PC, new shell):**

```bash
export ROS_DOMAIN_ID=1 && sws
ros2 topic list | grep drone_1/fmu
ros2 topic echo /drone_1/fmu/out/vehicle_status --once --qos-reliability best_effort
# armed: false
```

**T1 verify — mocap:**

```bash
ros2 topic hz /drone_1/pose                    # ~120–180 Hz
ros2 topic echo /drone_1/pose --once           # sane x,y,z at rest
```

**T1 verify — EKF2 fusion (critical — arm blocker if missing):**

```bash
ros2 topic hz /drone_1/fmu/in/vehicle_visual_odometry --qos-reliability best_effort
ros2 topic hz /drone_1/fmu/out/vehicle_odometry --qos-reliability best_effort   # must be >0
```

If `in/…` streams but `out/…` is silent → fix **EKF2_EV_CTRL** (Phase 0c) or `px4_vio_frame` in `diffaero_vel_real.yaml` (`enu_to_ned` vs `modalai_flip`).

**T1 verify — commander input:**

```bash
ros2 topic hz /drone_1/odometry_conversion/odometry   # ~30 Hz after EKF settles
```

**Optional ToF verify:**

```bash
ros2 topic hz /drone_1/perception/tof    # ~10–30 Hz
# or: CHECK_TOF=1 bash .../preflight_diffaero_mode_c.sh drone_1
```

---

#### Phase 3 — Preflight (hand-carry, still disarmed)

**Required before any `~/takeoff`.** RC kill switch in hand.

```bash
bash $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/scripts/preflight_diffaero_mode_c.sh drone_1
# With ToF required:
CHECK_TOF=1 bash $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/scripts/preflight_diffaero_mode_c.sh drone_1
```

**Manual checks:**

1. **Carry test:** Move drone by hand — RViz **red** marker on `/svg/viz/markers` must follow smoothly (no jumps/NaNs).
2. **North axis (B4b):** Carry ~1 m toward agreed **North** — `ros2 topic echo /drone_1/fmu/out/vehicle_odometry --once --qos-reliability best_effort` → `position[0]` increases. If wrong, set `px4_vio_frame: "modalai_flip"` in YAML and restart mocap_bridge.
3. **Disarmed:** `armed: false` on `vehicle_status`.
4. **Fence:** `goal_position` and `hover_positions` inside `fence_min` / `fence_max` in YAML.

---

#### Phase 4 — First flight (arms real motors)

> **Timeline:** `~/takeoff` sends **ARM at t ≈ 1.5 s** (§6c). Props can spin. Kill switch ready.

**Stage 1 — hover only (skip `~/start` on first outing):**

```bash
ros2 service call /diffaero_velocity_commander/takeoff std_srvs/srv/Trigger
# Wait: ARMING → ASCEND → FACE_GOAL → ACTIVE (pose hold at hover_positions)
# Confirm stable hover ~1.2 m; no drift runaway
ros2 service call /diffaero_velocity_commander/land std_srvs/srv/Trigger
```

**Stage 2 — policy cruise (after hover is stable):**

```bash
ros2 service call /diffaero_velocity_commander/takeoff std_srvs/srv/Trigger
# wait until ACTIVE + holding hover
ros2 service call /diffaero_velocity_commander/start std_srvs/srv/Trigger
# policy flies toward goal_position; auto-holds on arrival (§6a)
ros2 service call /diffaero_velocity_commander/hold std_srvs/srv/Trigger   # panic freeze (still armed)
ros2 service call /diffaero_velocity_commander/land std_srvs/srv/Trigger
```

**In-flight monitors:**

```bash
ros2 topic echo /diffaero_velocity_commander/status  # if exposed; else watch commander logs
# Log should show tof_fresh=True when ToF wired; vel=[…] sane vs motion
```

**Direction sanity:** After hover, a small commanded +x velocity should move the drone **East** in the mocap/map frame (verify once before `~/start`).

---

#### Phase 5 — Shutdown

| Step | Action |
|------|--------|
| 1 | `~/land` (or kill switch if emergency) |
| 2 | Confirm disarm in logs / `vehicle_status` |
| 3 | Ctrl-C `tof_udp_stream` on VOXL, then bridge on ground |
| 4 | Ctrl-C launch / agent terminals |

---

#### Mode C checklist (summary)

| Check | Pass |
|-------|------|
| `px4-microdds_client status` (VOXL) | Connected to ground agent IP |
| `MicroXRCEAgent` log | Session established |
| `/drone_1/pose` | ~120–180 Hz |
| `/drone_1/fmu/in/vehicle_visual_odometry` | ~mocap rate (best_effort) |
| `/drone_1/fmu/out/vehicle_odometry` | Streaming (EKF fusing) |
| `/drone_1/odometry_conversion/odometry` | ~30 Hz, tracks hand-carry |
| `preflight_diffaero_mode_c.sh` | Exit 0 |
| Armed before `~/takeoff` | **Disarmed** |
| ToF (if wired) | `/drone_1/perception/tof` ~10–30 Hz, `tof_fresh=True` after start |

#### Attitude commander variant

Copy [`config/diffaero_sim.yaml`](config/diffaero_sim.yaml) → `diffaero_real.yaml`, set `drone_mode: "real"`, same mocap section as `diffaero_vel_real.yaml`, launch:

```bash
ros2 launch svg_ground_control diffaero_single.launch.py \
  config:=/path/to/diffaero_real.yaml use_mocap:=true
```

Services under `/diffaero_commander/*` instead of `/diffaero_velocity_commander/*`.

### 10.4 Shared reference

#### Finding `GROUND_IP` (Modes B & C ToF)

The VOXL sends UDP to your **laptop Wi-Fi IP on the drone subnet** (not localhost,
not Docker bridge `172.x`). Robot container uses host networking.

```bash
export VOXL_IP=192.168.123.167
export GROUND_IP=$(ip -4 route get "$VOXL_IP" | awk '{print $7; exit}')
echo "GROUND_IP=$GROUND_IP"
ping -c 1 $GROUND_IP   # from VOXL
```

Start `tof_udp_bridge.py` on the ground **before**
`./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100`.

Full ToF build notes: `~/tof_streamer/README.md` (separate repo on the ground PC).

#### Perception encoding

`/{name}/perception/tof` carries a pre-encoded **9×16** grid: **1 = obstacle near,
0 = clear**. Stale ToF (> `tof_timeout_s`) → zeros. Velocity commander republishes
a hot/cold debug image on `/svg/{name}/tof_image` (red = close, blue = far,
black = 0.0).

#### Legacy ROS/DDS ToF path (avoid)

`voxl_mpa_to_ros2` → domain 0 → `domain_bridge` is **unsafe** with Jazzy on the
ground (`config/tof_real_to_sim_bridge.yaml` is reference only). Use UDP (§10.2).

#### Detailed sanity checks (sim)

- **Velocity frame (§3a):** policy log `vel=[…]` sign matches world motion when yawed.
- **Warm-up (§7):** no `odometry stale` at first policy tick.
- **Arrival (§6a):** `reached goal … → HOLD`.

---

## 11. Known-open / watch items

- `**max_acc_xy` cap deviates from training (attitude)** — fine for gentle indoor
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
`drone_mode: real` (template in §10.3; VOXL/EKF2 details in
`[experiment.md](experiment.md)` Part B).

