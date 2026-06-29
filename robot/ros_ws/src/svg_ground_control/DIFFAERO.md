# DiffAero on AirStack

This documents how the **DiffAero** learned flight policy (SHA2C, continuous
point-mass) is wired into AirStack via `diffaero_commander.py`, and — more
importantly — **why** each non-obvious detail is the way it is. Most of these
notes exist because the policy was first deployed standalone against PX4 over
MAVLink in `superfly/starling-deployment`, and porting it onto AirStack's
`robot_interface` abstraction surfaced a series of frame / timing / controller
mismatches. Each is recorded below so the next person doesn't re-derive them.

---

## 1. What DiffAero is (and is not)

- **Type:** a learned **cruise** controller exported as a self-contained
  TorchScript actor (`checkpoints/.../exported_actor.pt2`). It bakes in
  `tanh → rescale → Rz @ action → point_mass_quat` and returns
  `(acc_cmd, quat_xyzw_cmd, acc_norm)`.
- **Inputs (obs_frame = local, point-mass):** a 9-vector
  `[target_vel_local(3), uz(3), v_local(3)]` plus a `9×16` depth "perception"
  grid. `target_vel` and `v` are expressed in the **yaw-only (local) frame**;
  `uz` is the body up-axis in world.
- **Output (action_frame = local):** a **world-frame thrust acceleration**
  command. Gravity is handled by the point-mass model, so we do **not** add `g`
  when forming the attitude/thrust setpoint.
- **Trained at** `dt = 0.0333 s` → **30 Hz**. Run the control loop at 30 Hz
  (`control_rate_hz: 30.0`) so the policy sees the timestep it expects.

> **Key consequence:** it is **not** a position-hold controller. At the goal,
> `target_vel → 0`, but the network has nothing stable to track and will
> overshoot/oscillate. We must hand off to a pose-hold on arrival (see §6).

---

## 2. Data flow

```
                      AirStack robot_interface                  this package
  ┌─────────────┐    ┌───────────────────────────┐    ┌──────────────────────────┐
  │  MAVROS /   │    │ mavros_interface /         │    │ diffaero_commander       │
  │  PX4 (SITL) │───▶│ px4_interface →            │───▶│  odometry_callback       │
  │             │    │ odometry_conversion        │    │  → DiffAeroObs           │
  └─────────────┘    │   /{name}/odometry_        │    │  → DiffAeroPolicy.compute│
                     │   conversion/odometry (ENU)│    │  → AttitudeThrust out    │
                     └───────────────────────────┘    └──────────────────────────┘
        perception:  /{name}/perception/tof  (Float32MultiArray, pre-encoded 9×16)
        commands:    /{name}/interface/attitude_thrust_command  (mav_msgs/AttitudeThrust)
                     /{name}/interface/pose_command             (ascend / hold / fence)
                     /{name}/interface/velocity_command         (landing only)
        services:    /{name}/interface/robot_command            (arm / offboard / disarm)
```

`drone_mode` selects the interface topic templates:
- `sim`  → `/{name}/interface/*` (MAVROS/SITL)
- `real` → `/{name}/fmu/*` (px4_interface / uXRCE-DDS)

The policy code (`diffaero/diffaero_core.py`, `diffaero/perception_builder.py`)
is **byte-for-byte identical** to the starling deployment. Everything that
differs between the two deployments lives in the *plumbing* around it —
documented below.

---

## 3. Frame conventions (the most important section)

The policy lives entirely in **ENU world / FLU body**:

| Quantity            | Frame expected by policy            |
|---------------------|-------------------------------------|
| `position_enu`      | ENU world                           |
| `velocity_enu`      | **ENU world** (measured, not finite-differenced) |
| `R_enu`             | FLU-body → ENU-world rotation matrix |
| `goal_enu`          | ENU world                           |
| output attitude     | ENU/FLU quaternion `[x,y,z,w]`      |

### 3a. Velocity must be rotated body → world  ⚠ (the runaway-into-the-wall bug)

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

**Fix (`odometry_callback`):** rotate the twist into world ENU using the
orientation the message already carries (pose orientation is FLU→ENU):

```python
R_flu_to_enu = Rotation.from_quat(self.drone.orientation).as_matrix()
self.drone.velocity = R_flu_to_enu @ np.array([v.x, v.y, v.z])
```

> The hardware `px4_interface` path happens to publish **world-ENU** velocity
> (`px4_interface.cpp` converts NED-world → ENU-world), so this bug is
> MAVROS/sim-specific — but the rotation above is correct for both, because a
> true world velocity rotated by `R_flu_to_enu` would just be wrong. If you
> ever swap the odom source, **re-verify the twist frame** (see §10).

### 3b. Attitude output path

- The policy returns both `attitude_ned_frd_wxyz` (PX4-ready) and
  `attitude_enu_flu_xyzw` (ENU/FLU).
- starling sent `attitude_ned_frd_wxyz` straight over MAVLink
  `SET_ATTITUDE_TARGET`.
- AirStack publishes **`attitude_enu_flu_xyzw`** to `AttitudeThrust` and lets
  the **interface** convert ENU/FLU → NED/FRD. Keep the conversion in one place
  (the interface) rather than duplicating it here.

### 3c. ENU↔NED constants (in `diffaero_core.py`, same as Pegasus)

- ENU inertial → NED inertial: `Rotation.from_quat([0.70711, 0.70711, 0, 0])`
- FLU body → FRD body: `+π` about X = `Rotation.from_quat([1, 0, 0, 0])`

---

## 4. Observation construction (`compute`)

1. `Rz` = yaw-only frame from `R_enu` (strip pitch/roll; columns = `[fwd, left, up]` in ENU).
2. `target_vel_world = (goal − pos) / max(dist/max_vel, 1)` → saturates to `max_vel`.
3. Project into yaw frame: `target_vel_local = Rz.T @ target_vel_world`, `v_local = Rz.T @ v_world`.
4. `state9 = [target_vel_local, uz, v_local]`.
5. **vel-EMA → yaw orientation fed to the actor.** Initialized from the
   *heading direction* (`Rz[:,0]`), not raw velocity, so transient drift or
   post-interruption velocity doesn't corrupt yaw on the first tick after a
   reset. Falls back to the forward axis when nearly stationary (`norm < 0.3`).

`policy.reset()` clears the vel-EMA — call it whenever continuity breaks.

---

## 5. Perception

The commander uses the **`perception_encoded`** path: the `/{name}/perception/tof`
topic already carries a pre-encoded `9×16` grid (`0 = clear`, `1 = obstacle`),
so `PerceptionBuilder` is bypassed and intrinsics are dummies. `PerceptionBuilder`
(planar depth → crop to 86° FOV → planar-to-Euclidean → min-pool to 9×16 →
`1 − r/max_dist`) is retained for the raw-depth path. Encoding: **1 = surface at
the lens, 0 = nothing within `max_dist` (5 m)**. ToF older than `tof_timeout_s`
(0.5 s) is dropped (`perception_encoded=None` → zeros grid).

---

## 6. Lifecycle state machine

`IDLE → ARMING → ASCEND → FACE_GOAL → ACTIVE → (LANDING)`

| State      | Behavior |
|------------|----------|
| `ARMING`   | Timed: request offboard (1.0 s), arm (1.5 s), done (2.5 s). Streams current pose to satisfy PX4's offboard-entry "must already be receiving setpoints" precondition. |
| `ASCEND`   | Pose command to `hover_positions`; transitions when within `arrival_threshold_m`. |
| `FACE_GOAL`| Yaw in place to point at `goal_position` (rotating *before* moving avoids digging a skid/leg into the ground and tripping a sim collision — same reasoning as starling's separate YAW phase). Transitions when yaw error < `face_goal_threshold_rad`. |
| `ACTIVE`   | Runs the policy (`mission_active`) or holds pose (idle). See §6a. |
| `LANDING`  | Velocity-down until `land_complete_altitude_m`, then disarm. |

Services: `~/takeoff`, `~/start`, `~/hold`, `~/land`, `~/reset_fence`.

### 6a. Goal handoff — policy is a cruise controller, not a hover controller ⚠

`~/start` pins `policy_goal = goal_position` (a real target), so
`target_vel = (goal − pos)` drives the drone toward it.

**On arrival we must leave the policy.** When within `goal_arrival_threshold_m`
(0.4 m) of the goal, the commander drops `mission_active`, sets
`hold_target = goal` / `hold_orientation = current`, and from then on streams a
**pose-hold** at the goal (stable PX4 position control). This mirrors the
starling deployment, which switched to PX4 `LAND` within 0.5 m. Without this,
the policy overshoots the goal and oscillates ("goes crazy at the goal").

> Earlier the goal was pinned to the *current* position for a hover test
> (`policy_goal = drone.position`) → `target_vel ≈ 0` → it holds in place. That
> was the smoke test that proved the velocity-frame fix (§3a) before enabling
> real goal-seeking.

### 6b. Interruption handling

If the policy was interrupted > 0.2 s (stale odom, fence, hold), `vel_ema` is
reset so the heading-direction init (§4.5) kicks in instead of accumulating
stale drift. We intentionally **do not** re-anchor the goal to the current
position here (that would make it hover wherever it got interrupted mid-cruise).

---

## 7. Policy warm-up (single-threaded executor) ⚠

The node runs on `rclpy.spin` (**single-threaded executor**): the control-loop
timer and the odometry subscription share one thread. The **first**
`policy.compute()` pays ~0.5 s of TorchScript/CUDA JIT warm-up, which blocks the
thread → the odometry callback can't run → odom looks "stale" at the exact
moment of policy handoff → spurious `odometry stale` hold + `vel_ema` reset on
the very first tick of flight.

**Fix:** run a few dummy `compute()` calls (then `reset()`) in `__init__`, while
still constructing the node, so the first real tick is fast. Look for
`DiffAero policy warmed up` at startup and the absence of an `interrupted …s`
line right after `scenario "… " running`.

---

## 8. Parameters and why they're set the way they are

(`config/diffaero_sim.yaml`)

| Param | Value | Why |
|-------|-------|-----|
| `control_rate_hz` | 30.0 | Matches training `dt = 0.0333 s`. |
| `drone_position_offset` | `[0,0,0]` | `NUM_ROBOTS=1` → `drone_1` spawns at origin, so raw odom == world. Set to the spawn position for multi-robot / shifted spawns; leave zero for mocap. A zero offset logs a warning in sim precisely because it's wrong for a shifted spawn. |
| `goal_position` | `[2,-2,1.2]` | Cruise target (ENU). Keep inside the geofence. |
| `max_vel` | 1.5 | Cruise speed cap: `target_vel = (goal−pos)` saturated to this. Training sampled **3–6 m/s** — far too fast/violent for a short indoor hop. Caps *cruise* speed, **not** from-rest acceleration (see `max_acc_xy`). |
| `max_acc_xy` | 6.0 | Caps the policy's horizontal action scale → max tilt ≈ `atan(max_acc_xy/g)` ≈ **31°** (vs ~64° at the 20.0 default). Tames the launch lunge. **Deviates from the action limits the policy was exported with**, so tracking may be slightly off — raise toward 20 if it feels sluggish. |
| `goal_arrival_threshold_m` | 0.4 | Distance at which to leave the cruise policy for a pose-hold (§6a). |
| `max_accel` | 20.0 | Thrust-accel that maps to full throttle; hover throttle = `g/max_accel ≈ 0.49`. **Must match what the FCU expects** — starling set `MPC_THR_HOVER = g/max_accel` explicitly. If the drone slowly climbs/sinks at hover, tune this. |
| `tof_timeout_s` | 0.5 | Drop stale perception. |
| `state_timeout_s` | 0.5 | Odom freshness window; older → stale-hold fallback. |
| `fence_*` | `[-3,3]³` | Latching geofence: a breach freezes the drone and blocks `~/start` until `~/reset_fence`. |

Tuning order for a new site: (1) confirm hover is stable (`max_accel`), (2) set
a modest `max_vel`, (3) tame the launch with `max_acc_xy`, (4) widen
`goal_arrival_threshold_m` if it overshoots before the hold catches.

---

## 9. Differences vs `starling-deployment` (why the port needed work)

| Aspect | starling (worked) | AirStack |
|--------|-------------------|----------|
| Transport | direct MAVLink (pymavlink) | `robot_interface` (MAVROS/px4_interface) over ROS 2 |
| Velocity source | `LOCAL_POSITION_NED` (world NED → world ENU) | `mavros/local_position/odom` twist (**body FLU** → must rotate, §3a) |
| Attitude cmd | `SET_ATTITUDE_TARGET` (NED/FRD) | `AttitudeThrust` (ENU/FLU); interface converts |
| Arrival | switch to PX4 `LAND` < 0.5 m | switch to pose-hold < `goal_arrival_threshold_m` (§6a) |
| Executor | dedicated control loop thread | single-threaded `rclpy.spin` → needs warm-up (§7) |
| Hover throttle | `MPC_THR_HOVER` set explicitly | relies on `max_accel`; verify FCU hover param |

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
latched). If starting bare: Add → By topic → `/svg/viz/markers`.

**Sanity checks** (any shell in the robot container):

- **Velocity frame (§3a):** with the drone yawed and drifting toward world `+x`,
  the policy log's `vel=[...]` must read mostly `+x`. If its sign is opposite to
  the actual motion, the twist is body-frame and the §3a rotation is missing.
  ```bash
  ros2 topic echo /drone_1/odometry_conversion/odometry --field twist.twist.linear --once
  ```
- **Warm-up (§7):** expect `DiffAero policy warmed up` at startup and **no**
  `odometry stale / interrupted` line right at handoff.
- **Launch tilt (§8):** first `att_euler_rpy` pitch ≈ 30°, not 40°+.
- **Arrival (§6a):** `reached goal … → HOLD`, then a steady hover.
- **Odom rate:** `ros2 topic hz /drone_1/odometry_conversion/odometry` should be
  steady near the control rate; recurring mid-flight gaps (not the startup one)
  indicate a real stream problem, not warm-up.

---

## 11. Known-open / watch items

- **`max_acc_xy` cap deviates from training** — fine for gentle indoor hops, but
  if obstacle avoidance is later enabled the policy may need its full action
  authority back. Treat 6.0 as a comfort/safety clamp, not a permanent value.
- **Hover-throttle matching** — `max_accel` vs the FCU's hover param is the most
  likely cause of slow altitude drift at hover; verify per airframe.
- **Single goal only** — `policy_goal` is a fixed point. Multi-waypoint / moving
  goals would need a carrot/waypoint feeder in the control loop.
</content>
</invoke>
