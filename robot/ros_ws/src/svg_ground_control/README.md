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
  x-y lag never moves the altitude reference). Measured on drone_2: the old `1.5 × distance`
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
  `hold_lead_m`, `takeoff_speed_mps`) are live too, and the snapshot's
  `tuning` block reports `teleop_max_speed_mps`, `goal_accel_mps2`,
  `goal_settle_s` and `scenario_speed_mps` so the panel's gain dropdown can
  edit the first three next to the CBF gains. `teleop_max_speed_mps` and
  `safe_teleop`'s `max_speed_mps` are one number kept equal at runtime:
  the pad follows the commander's snapshot, a `ros2 param set
  /safe_teleop max_speed_mps` is pushed to the commander, and the panel's
  Teleop vmax row sets both. Everything else is read once at startup; a
  `ros2 param set` on it is refused with a reason.

Full how-to for all of the above: **[experiment.md](experiment.md)**.

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
