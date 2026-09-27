# SVG Ground Control

Central multi-drone ground controller for mocap flight with a **CBF
collision safety filter** (velocity-CBF with hybrid Dykstra projection,
ported from `~/drone_soccer` where it is MuJoCo-validated against the
Starling 2 Max airframe).

> **Commands:** see [experiment.md](experiment.md) — the maintained,
> copy-pasteable command reference for sim and hardware.

## Architecture

```
 OptiTrack Motive ──▶ natnet_ros2 ──▶ /{name}/pose   (hardware only)
                                          │
                          ┌───────────────▼────────────────────────────┐
                          │ mocap_bridge → /{name}/fmu/visual_odometry │
                          └────────────────────────────────────────────┘
                          ┌────────────────────────────────────────────┐
 /{name}/odometry_        │ swarm_commander  (20 Hz)                   │
 conversion/odometry ──▶  │  scenario nominal | per-drone teleop       │
 /svg/{name}/teleop ───▶  │  → cbf_filter.filter_velocities()  [REAL]  │
                          │  → real: /{name}/fmu/trajectory_command    │
                          │          (reference + velocity + accel)    │
                          │    sim:  /{name}/interface/velocity_command│
                          │  services: takeoff / start / hold / land   │
                          └────────────────────────────────────────────┘
                                   │ per-drone robot_interface
                          sim: MAVROS          real: px4_interface (uXRCE-DDS)
```

Sim and hardware differ **only in the topic templates** in the config YAML
(`config/swarm_sim.yaml` vs `config/swarm_real.yaml`).

## Scenarios (`scenario:=` launch arg)

Ported from drone_soccer plus goal-tracking and a squeeze profile:

- `hover` — hold configured positions
- `goal` — each drone seeks a per-drone goal you set live via
  `/svg/{name}/goal_xyzt` (`[x, y, z, theta_deg]`, theta 0 = +X, clockwise)
  or `/svg/{name}/goal_command` (PoseStamped) + `/svg/{name}/speed_command`
  (Float32); backs the single- and multi-drone tracking tests
- `random_walk` — fixed-speed drift with wall bounces
- `random_goals` — random goal seeking, resampled on arrival
- `head_on` — two facing groups swap sides repeatedly
- `antipodal` — sphere-to-antipode crossings through the center
- `squeeze` — **3-drone CBF showcase** ([config/squeeze_3drone.yaml](config/squeeze_3drone.yaml)):
  two holders goal-track explicit posts; the intruder shuttles through the
  gap; the holders must yield and return. Order: `[holder, holder, intruder]`.

`teleop_drones` (comma-separated string) lists operator-driven drones — empty
= fully autonomous. Teleop is a control-source role, not a safety exemption:
a teleop drone's commanded velocity is still passed through the CBF filter
like any autonomous drone unless it is also listed in `cbf_exempt_drones`
(separate, opt-in, empty by default). `external_drones` are tracked for the
filter but never commanded (e.g. RC-flown).

The maintained way to hand-fly a drone is the **`safe_teleop`** gamepad driver
(sticks = velocity, flown in position mode by the commander so released
sticks hold position; stick lock), brought up end to end by `scripts/svg_teleop.sh` — sim experiments
(`solo`/`squeeze`/`hover`) and one real drone (`real`,
[config/teleop_real.yaml](config/teleop_real.yaml)). See
**[teleop.md](teleop.md)** for controls, pad diagnostics, axis signs, and the
real-drone ground check. `teleop.launch.py` starts the input driver and
`safe_teleop` in their own terminal (printing the pad reading, so it is
checked before the commander comes up); the device is the
`teleop_controller` parameter (`dragonrise_usb`, `xbox_usb`; registry in
[safe_teleop/controllers.py](svg_ground_control/safe_teleop/controllers.py)).
The old keyboard teleop has been removed; `xbox_teleop` (direct stick-to-
velocity, no altitude hold) remains as an ad-hoc utility.

## Hybrid sim/real, geofence, RViz

- **Per-drone sim/real routing** (`drone_modes: "real,real,sim"`): each drone's
  commands route to MAVROS (`/{name}/interface/…`, sim) or px4_interface
  (`/{name}/fmu/…`, hardware), all under one CBF. See
  [config/hybrid_squeeze.yaml](config/hybrid_squeeze.yaml).
- **Geofence**: `fence_enabled` + `fence_min`/`fence_max`, watched for every
  role. `fence_behavior: hold_all` — any airborne drone leaving the box
  latches a swarm-wide freeze until `~/reset_fence`; `keep_in` — commanded
  drones are braked at the walls and pushed back in, nobody stops. The wall
  is a braking envelope (`fence_brake_accel_mps2`, `fence_keep_in_gain`:
  cruise until the true braking distance, then a firm brake sent to PX4 as
  the acceleration feedforward — the old `gain × distance` cap overshot by
  0.5 m at 6 m/s, bag `run_045417`). A separate, smaller **teleop fence**
  (`teleop_fence_enabled`, `teleop_fence_min`/`max`, inside the geofence)
  bounds hand-flown drones the same way whatever `fence_behavior` is. The
  boxes and a fence-clipped ground grid (`fence_grid_cell_m`, world-aligned,
  whole metres brighter) are published with the drone markers.
- **Position hold and trajectories** (`trajectory.py`, `position_hold.py`):
  every commanded drone has a reference point (where it was told to be),
  which real drones receive as PX4's position setpoint together with the
  velocity and acceleration feedforward, so PX4 holds position onboard and
  tracks like its own Position mode. Scenario drones fly an
  acceleration-limited profile with PX4's braking law toward their goal
  (`goal_accel_mps2`, `goal_settle_s`, `goal_lead_m`); hand-flown drones move
  the reference with the sticks, so released sticks hold position on all
  axes (`teleop_lead_m`; horizontal and vertical lead leashed separately, so
  x-y lag never moves the altitude reference). A CBF-corrected command keeps
  a feedforward — the rate of change of the published command, capped at
  `goal_accel_mps2` — so an evasion is flown with it rather than ~0.5 s
  behind it (`command_feedforward`; bag `run_020444`). The climb and the
  non-mission hold evaluate their braking law at the reference point too
  (`profile_point`), so the setpoint settles on the spot instead of
  swinging ±0.15 m around it at ~4 s (bags `run_042957`, `run_042433`). Measured on drone_2: the old `1.5 × distance`
  velocity P-law overshot a 5 m/s leg by 1 m; see experiment.md C1.
- **Heading**: real drones are told an absolute yaw with every setpoint —
  the goal's `theta` in the goal scenario, nose on +X everywhere else
  (0° = +X, clockwise positive). Teleop yaws at the stick's rate (all-zero
  rotation in the setpoint) and holds the measured heading when it is
  centred. The stick velocity is ramped at `teleop_accel_mps2` with the
  acceleration fed forward, like PX4's own Position mode.
- **RViz**: all drones' world positions on `/svg/viz/markers`
  (`rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz`).
- **Status snapshot** (`status_topic`, default `/svg/commander_status`,
  `std_msgs/String` JSON at `status_rate_hz` = 5 Hz): mission state
  (`mission_active`, `mission_ever_started`, `mission_started_at`,
  `fence_breached`), the outcome of the last lifecycle service
  (`last_command` + a `command_seq` counter), the live CBF gains and which
  drones the CBF is correcting, and per drone its `FlightState`, world
  position, speed, odometry freshness, DDS reception counters
  (`odom_rx_total` / `odom_lost_total` from the reader's `message_lost`
  event — a measured drop count, not a timing guess) and the result of its
  last `robot_command` (offboard / arm / disarm). Built by `build_status()`; the
  [SVG Basestation Foxglove panel](foxglove/svg-basestation/README.md)
  (in this package's `foxglove/` directory, with the `svg_basestation.json`
  layout and its `install.py`) uses it to confirm Start really took effect
  and to show numeric positions.
- **Runtime tuning**: the CBF gains (`cbf_alpha`, `cbf_safety_radius_m`,
  `cbf_max_speed_mps`) can be changed while flying and apply on the next
  control tick (`ros2 param set /swarm_commander cbf_alpha 4.0`, or the
  panel's CBF sliders via `set_parameters`); non-positive / non-finite
  values are rejected. The speed and tracking gains (`scenario_speed_mps`,
  `teleop_max_speed_mps`, `goal_accel_mps2`, `goal_settle_s`, `goal_lead_m`,
  `goal_velocity_only_settle_s`, `teleop_kp`, `teleop_lead_m`, `hover_kp`,
  `hold_lead_m`, `takeoff_speed_mps`) are live too. Everything else is read
  once at startup; a `ros2 param set` on it is refused with a reason.

Full how-to for all of the above: **[experiment.md](experiment.md)**.

## Update 2026-09-27 — what the squash onto `yikuan/SVG_ground_control` contains

One commit carrying 40 commits of `yikuan/SVG_ground_control_dev`
(2026-08-24 … 2026-09-27; 68 files, +15057 / −404). Everything below was
flown on the three Starlings in the mocap room; the bag names are the
evidence and live in `experiment.md`'s troubleshooting table.

### 1. Real drones fly a PX4-style trajectory

- **Output changed.** A real drone (`drone_modes: real`) now receives one
  `trajectory_msgs/MultiDOFJointTrajectory` point at 20 Hz on
  `/{name}/fmu/trajectory_command`: the **reference point** (PX4 position
  setpoint), the **velocity**, the **acceleration feedforward** and an
  absolute **yaw**. `px4_interface` gained that subscription, an ENU→NED
  conversion for all three vectors and the `TRAJECTORY` control mode
  (`offboard_control_mode` position + velocity + acceleration). Sim / MAVROS
  drones still get a bare `TwistStamped`. Parameters: `real_command_mode`
  (`trajectory` | `velocity`), `real_trajectory_command_topic_template`.
  **Rebuild `px4_interface` and `svg_ground_control` and relaunch the
  interfaces** — an old interface ignores the new topic and the drone hovers.
- **Go-to-goal law** (`trajectory.py`): an acceleration-limited profile with
  PX4's own braking law `v = −aL + sqrt((aL)² + 2ad)` evaluated at the
  reference point and re-attached to the velocity actually published
  (`ReferenceTracker`). Parameters, all live: `goal_accel_mps2`,
  `goal_settle_s`, `goal_lead_m` (leash: how far the reference may run ahead
  of the drone), `goal_velocity_only_settle_s` (the stateless law sim
  drones fly). The plant identified from drone_2's ULogs (link delay 0.15 s,
  velocity loop P 1.8 / I 0.4, attitude lag 0.1 s, |a| ≤ 8 m/s², position
  P 0.95) is in `test/test_trajectory.py`; the old `1.5 × distance` P-law
  overshot a 5 m/s leg by 1 m, the profile does not.
- **Takeoff**: a climb profile at `takeoff_speed_mps` with the reference on
  the short `hold_lead_m` leash (0.2 m) while ascending / landing / holding
  (bag `C1_0920_203148`: the old 2 m leash let PX4's stiff altitude loop
  dash to 1.94 m for a 1 m target).
- **Climb and hold evaluated at the reference** (`profile_point`): the
  setpoint converges onto the spot. Evaluated at the drone and integrated
  into the reference it stopped up to 0.2 m off the spot and PX4's pull
  toward it re-opened the error — a ~4 s ±0.15 m limit cycle around every
  hover before `/start` (bags `run_042957`, `run_042433`, `run_041532`).
- **`/hold` brakes to a predicted stop point** `v²/(2a) + v/hover_kp` ahead
  (clamped into the fence) instead of flying back to the call position
  (bag `run_041842`: a 1.5 m bounce from 6 m/s). The reply says
  `braking, stops X m ahead`.
- **Heading**: goals are `[x, y, z, theta]` on `/svg/{name}/goal_xyzt`
  (`Float64MultiArray`, theta in degrees, 0 = +X, clockwise positive);
  `/svg/{name}/goal_command` (PoseStamped) still works and takes the yaw
  from its quaternion. Every other scenario keeps the nose on +X. Teleop
  yaws at the stick's rate (all-zero rotation = yaw-rate mode in
  `px4_interface`) and holds the measured heading when centred.

### 2. CBF changes

- **Feedforward kept through corrections** (`command_feedforward`): a
  CBF-corrected command carries the rate of change of the published
  command, capped at `goal_accel_mps2`, so an evasion is flown with
  feedforward instead of by PX4's velocity loop alone, ~0.5 s behind. Bag
  `run_020444`: every close pass (0.52, 0.65, 0.79 m against 1.1–1.3 m
  required) had the commanded closing speed already at zero where the
  barrier says; the drones were not flying the command. The fence's braking
  feedforward now replaces only the axes the wall limited.
- **Short leash only on CBF corrections**, not on fence speed clips (bag
  `run_035852`: leashing on a clip turned every fence-limited cruise into a
  bare velocity setpoint, 2.5–3.3 m/s actual for 4.7–5.9 commanded).
- `/svg/cbf_active` (`String`, comma-separated names every tick) for the
  LEDs and the panel; `cbf_alpha`, `cbf_safety_radius_m`, `cbf_max_speed_mps`
  settable in flight.
- Known, not yet changed: `cbf_alpha` 2.5 assumes no tracking lag, and
  `random_goals` samples goals with no separation from other drones or their
  goals (100 of 221 legs in `run_020444` had another drone within 1.5 m of
  the goal, one arrival took 68 s). See experiment.md.

### 3. Geofence

- `fence_behavior`: `hold_all` (any airborne drone outside the box latches a
  swarm-wide freeze until `~/reset_fence`) or `keep_in` (commanded drones are
  braked at the walls and pushed back in, nobody stops).
- `keep_in` is a **braking envelope** (`fence_brake_accel_mps2`,
  `fence_keep_in_gain`, `fence_margin_m`): cruise until the true braking
  distance, then a firm brake whose deceleration goes to PX4 as feedforward
  on that axis (`fence.py`: `wall_speed`, `keep_in_velocity`,
  `keep_in_acceleration`). The old `gain × distance` cap overshot 0.5 m at
  6 m/s (bag `run_045417`). Set the brake to what the airframe delivers at
  its tilt limit (8 m/s² at 45°); 8 with a 1 m-past-the-wall goal at 9 m/s
  still overran 1.9 m in `log_141` because the goal sat on the wall.
- **Teleop fence** (`teleop_fence_enabled`, `teleop_fence_min` / `max`)
  inside the geofence for hand-flown drones, whatever `fence_behavior` is.
  Fence boxes and a ground grid (`fence_grid_cell_m`) are published as
  markers.

### 4. Commander hygiene

- **Takeoff resets stored goals** to the takeoff points and goals are
  accepted before `/start` (the stored goal is drawn, dimmer, while idle).
- **Twin commanders**: a second `swarm_commander` on the same domain is
  detected (`get_node_names_and_namespaces` + `/proc` scan); takeoff / start
  are refused while a twin exists; at start-up idle twins are killed
  (SIGTERM, then SIGKILL) — `takeover:=false` / `takeover_twins` to disable.
  A twin with a drone in the air is never killed.
- **Status snapshot** on `/svg/commander_status` (`status_topic`,
  `status_rate_hz`): mission state, last service outcome, fences, CBF
  gains and active set, per-drone state / position / speed / odometry
  freshness and loss counters, `pid`.
- **Live parameters**: `scenario_speed_mps`, the goal law, `teleop_*`,
  `hover_kp`, `hold_lead_m`, `takeoff_speed_mps`, the fence dynamics and
  the CBF gains; everything else is refused with a reason.
- `stop_point`, `advance_reference`, `leash`, `ramp_velocity`,
  `command_feedforward`, `profile_point` are small pure functions with
  their own tests.

### 5. Teleop (`safe_teleop/`, `teleop.launch.py`, `teleop.md`)

- Gamepad teleop for `teleop_drones`: `teleop_controller` selects a device
  profile from `safe_teleop/controllers.py` (`xbox_usb`, `dragonrise_usb`),
  rate-controlled altitude with lock, `joy_map` / `joy_view` / `monitor`
  diagnostics, the pad started and checked in its own terminal first.
- **Position-mode sticks**: the sticks move a leashed reference
  (`teleop_lead_m`, horizontal and vertical leashed separately), released
  sticks hold position; the stick velocity is ramped at `teleop_accel_mps2`
  with the ramp's acceleration fed forward (PX4 `MPC_ACC_HOR_MAX` style);
  `teleop_max_speed_mps` caps the stick and warns when it does;
  `teleop_kp` is the velocity-output (sim) gain only.

### 6. Foxglove basestation

- `foxglove/svg-basestation`: CBF alpha / gamma / vmax and a parameter
  drop-down, numeric drone positions, Start-status check from the snapshot,
  a 3D grid matched to the launched geofence; `foxglove_bridge` starts with
  the commander (`use_foxglove_bridge`, `foxglove_port`,
  `use_foxglove_studio`).

### 7. Configs and docs

- `goal_single.yaml`, `goal_tracking.yaml`, `squeeze_rc_intruder.yaml`,
  `swarm_real.yaml`, `hybrid_squeeze.yaml`: goal law 10 / 0.2 / 2.0,
  `keep_in` fence with brake 8 and gain 2, +y wall at 5.0 (the mocap volume
  ends at y ≈ 5.5 — `log_141` lost the body there three times), takeoff and
  land 1.0 m/s, CBF radius 0.55 / alpha 2.5 / cap 10, `teleop_*` and
  `safe_teleop` blocks. New: `cbf_sim.yaml`, `teleop_real.yaml`,
  `teleop_single.yaml`.
- `experiment.md`: C1 "how it flies", top speed in the room (8 m/s on a
  10 m runway at 45° tilt; 10 m/s needs 11 m or 60°), heading, LED service,
  CBF safety, and troubleshooting rows for the hold bounce, wall goals, twin
  commanders, takeoff reset, hover swing, the failure detector vs
  `MPC_TILTMAX_AIR` (the `log_141` termination) and the mocap volume edge.
- PX4 params that go with this: `MPC_TILTMAX_AIR` 45 (60 tripped the
  failure detector at `FD_FAIL_R` 60), `EKF2_EV_DELAY` 50 ms on every drone.

### 8. Tests

140 tests: `test_trajectory.py` (plant model: goal legs, takeoff, hold
swing), `test_fence_and_position_hold.py`, `test_feedforward.py`,
`test_live_speed.py`, `test_runtime_params.py`, `test_teleop_controllers.py`,
`test_scenarios.py`, plus the unchanged CBF suite. Run inside the robot
container:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider test/`.

## CBF filter

`svg_ground_control/cbf_filter.py` is a verbatim port of
`drone_soccer/cbf.py`: pairwise barrier `h = ||p_i−p_j||² − (2r)²`,
constraint `ḣ + αh ≥ 0` (linear in velocities), least-squares projection via
parallel Dykstra + Gauss-Seidel polish, constraint pruning, and an emergency
push-apart fallback when the QP is infeasible. `cbf_alpha` is the class-K
gain: lower = gentler (yields earlier, softer corrections), higher = more
aggressive (approaches closer, corrects harder). Tests:
[test/test_cbf.py](test/test_cbf.py) (kinematic suite from drone_soccer),
[test/test_scenarios.py](test/test_scenarios.py) (includes a kinematic
squeeze rollout), [test/test_runtime_params.py](test/test_runtime_params.py)
(runtime `cbf_alpha` set/reject and the status snapshot), and
[test/functional_squeeze_test.py](test/functional_squeeze_test.py)
(closed-loop ROS test against fake drones — barrier held at exactly 2r).

## Safety notes

- Teleop drones are CBF-protected by default, same as autonomous ones — the
  filter corrects an operator's command like any other drone's. Add a drone
  to `cbf_exempt_drones` if you deliberately want it uncorrected (e.g. it
  should act as the moving obstacle the others dodge); in that case the
  operator becomes the safety authority for it instead of the filter.
- This stack bypasses `drone_safety_monitor`; PX4 failsafes and the RC kill
  switch are the safety net. Configure them before flying.
- Stale odometry (> `state_timeout_s`) → zero-velocity command. A stale
  *teleop topic* (> `teleop_timeout_s`, i.e. the teleop node died) or a dead
  *gamepad* (safe_teleop publishes zeros) both mean "sticks at rest": the
  commander keeps holding the drone at its position-mode target — see
  teleop.md "Safety". `~/hold` is the panic button.
- `CBF emergency push-apart engaged` in the log means the QP went infeasible
  (drones inside each other's safety spheres) — land and investigate.
