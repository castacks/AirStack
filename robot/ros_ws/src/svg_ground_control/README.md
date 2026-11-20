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
                          │  → /{name}/.../velocity_command            │
                          │  services: takeoff / start / hold / land   │
                          └────────────────────────────────────────────┘
                                   │ per-drone robot_interface
                          sim: MAVROS          real: px4_interface (uXRCE-DDS)
```

Sim and hardware differ **only in the topic templates** in the config YAML
(`config/swarm_sim.yaml` vs `config/swarm_real.yaml`).

## Ball mocap velocity filtering

`mocap_bridge` can publish a filtered world-frame velocity with each rigid
body's pose on `/{name}/mocap_odometry`. The PX4 direct external-vision path
remains pose-only; these velocities are for the soccer policy, logging, and
debugging.

```mermaid
flowchart LR
    pose[PoseStamped mocap] --> raw[Finite difference]
    pose --> lp[Low-pass derivative]
    pose --> kf[Position/velocity Kalman filter]
    raw --> select{velocity_filter_type}
    lp --> select
    kf --> select
    select --> odom[Odometry twist]
    odom --> policy[Soccer policy]
```

The supported `velocity_filter_type` values are:

| Value | Behavior | Typical use |
|---|---|---|
| `finite_difference` | Unfiltered position difference | Noise baseline only |
| `low_pass` | First-order filter over the derivative | Simple, predictable smoothing |
| `kalman` | Six-state constant-velocity filter updated by position | Recommended for ball mocap |

Important parameters:

| Parameter | Default | Tuning effect |
|---|---:|---|
| `velocity_filter_type` | `low_pass` | Selects the bridge estimator; the SoccerBall hardware configs override this to `kalman`. |
| `velocity_low_pass_cutoff_hz` | `0.0` | Positive values enable a rate-independent cutoff; lower is smoother but slower. At `0`, `velocity_filter_alpha` is used. |
| `velocity_filter_alpha` | `0.4` | Legacy low-pass weight; lower is smoother. |
| `velocity_kalman_position_stddev_m` | `0.005` | Expected mocap noise; increasing it smooths more. |
| `velocity_kalman_acceleration_stddev_mps2` | `10.0` | Expected ball acceleration; increasing it responds faster to kicks but passes more noise. |
| `velocity_kalman_initial_velocity_stddev_mps` | `2.0` | Initial velocity uncertainty. |
| `velocity_filter_max_dt_s` | `0.5` | A longer sample gap resets the estimate to zero. |

Tune the Kalman estimator in a rolling Matplotlib window:

```bash
ros2 run svg_ground_control plot_mocap_velocity --ros-args \
  -p pose_topic:=/SoccerBall/pose \
  -p history_s:=15.0 \
  -p velocity_kalman_position_stddev_m:=0.004 \
  -p velocity_kalman_acceleration_stddev_mps2:=0.1 \
  -p velocity_kalman_initial_velocity_stddev_mps:=1.7
```

The blue curve is the Kalman estimate computed from the mocap pose samples.
Sliders tune its position measurement noise, acceleration process noise, and
initial velocity uncertainty in real time. Changing a slider restarts the plot
estimate; it does not modify the running `mocap_bridge`. Use **Reset sliders**
to restore the startup values. Close the window or press `Ctrl-C` to exit. The
The production bridge publishes the tuned estimate on
`/SoccerBall/mocap_odometry`. The `policy_commander`,
`drone_soccer_depl/inference_soccer_pos_sp`, and legacy `pos_sp_soccer`
deployment paths consume or prefer that shared ball state. Their drone state
remains sourced from PX4 odometry, including the onboard EKF's velocity
estimate.

## Scenarios (`scenario:=` launch arg)

Ported from drone_soccer plus goal-tracking and a squeeze profile:

- `hover` — hold configured positions
- `goal` — each drone seeks a per-drone goal you set live via
  `/svg/{name}/goal_command` (PoseStamped) + `/svg/{name}/speed_command`
  (Float32); backs the single- and multi-drone tracking tests
- `random_walk` — fixed-speed drift with wall bounces
- `random_goals` — random goal seeking, resampled on arrival
- `head_on` — two facing groups swap sides repeatedly
- `antipodal` — sphere-to-antipode crossings through the center
- `squeeze` — **3-drone CBF showcase** ([config/squeeze_3drone.yaml](config/squeeze_3drone.yaml)):
  two holders goal-track explicit posts; the intruder shuttles through the
  gap; the holders must yield and return. Order: `[holder, holder, intruder]`.

`teleop_drones` (comma-separated string) lists operator-driven, **CBF-exempt**
drones (the moving obstacles) — empty = fully autonomous. `external_drones`
are tracked for the filter but never commanded (e.g. RC-flown). Drive a
teleop drone with `ros2 run svg_ground_control keyboard_teleop --ros-args -p
drone:=drone_3` (one instance per teleop drone).

## Hybrid sim/real, geofence, RViz

- **Per-drone sim/real routing** (`drone_modes: "real,real,sim"`): each drone's
  commands route to MAVROS (`/{name}/interface/…`, sim) or px4_interface
  (`/{name}/fmu/…`, hardware), all under one CBF. See
  [config/hybrid_squeeze.yaml](config/hybrid_squeeze.yaml).
- **Geofence**: `fence_enabled` + `fence_min`/`fence_max`; any airborne drone
  leaving the box latches a swarm-wide freeze until `~/reset_fence`.
- **RViz**: all drones' world positions on `/svg/viz/markers`
  (`rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz`).

Full how-to for all of the above: **[experiment.md](experiment.md)**.

## CBF filter

`svg_ground_control/cbf_filter.py` is a verbatim port of
`drone_soccer/cbf.py`: pairwise barrier `h = ||p_i−p_j||² − (2r)²`,
constraint `ḣ + αh ≥ 0` (linear in velocities), least-squares projection via
parallel Dykstra + Gauss-Seidel polish, constraint pruning, and an emergency
push-apart fallback when the QP is infeasible. Tests:
[test/test_cbf.py](test/test_cbf.py) (kinematic suite from drone_soccer),
[test/test_scenarios.py](test/test_scenarios.py) (includes a kinematic
squeeze rollout), and [test/functional_squeeze_test.py](test/functional_squeeze_test.py)
(closed-loop ROS test against fake drones — barrier held at exactly 2r).

## PPO policy deploy (drone_soccer)

Inference-only: `policy_commander` loads an SB3 `.zip`, builds the 18-dim
observation from PX4 odometry plus ball mocap, and publishes PX4
`TrajectorySetpoint` and `OffboardControlMode` messages directly to
`/{name}/fmu/in/trajectory_setpoint` and
`/{name}/fmu/in/offboard_control_mode`. The policy's full 3D waypoint is used.
While the drone is more than `geofence_buffer_m` (default `0.5 m`) from every
horizontal geofence face, the resulting ENU X/Y waypoint is left unconstrained.
Once the drone is within that buffer—or is already outside the fence—the X/Y
waypoint is clamped to `bounds_min` and `bounds_max`. Altitude is always
clamped to the configured Z bounds.

The ball goal supplied to the policy observation can be `fixed`, `circle`,
`figure8`, or `random`. `target_x` and `target_y` specify the fixed goal or
periodic-curve center. `goal_radius`, `goal_period_s`, and `goal_phase_rad` configure the
periodic modes. `goal_rotation_rad` then rotates the complete curve
counter-clockwise in ENU around its center. Calling `/policy_commander/start`
resets the reference phase. Before rotation, phase zero starts at
`(target_x + goal_radius, target_y)`. The current XY reference is published on
`<debug_topic_prefix>/goal` as
`std_msgs/Float32MultiArray`.

In `random` mode, one goal is sampled uniformly from the axis-aligned ENU
rectangle defined by `goal_spawn_min_xy` and `goal_spawn_max_xy`. A new goal
is sampled when the drone's XY position (or the ball's, with
`goal_arrival_reference: ball`) comes within `goal_arrival_radius` of the
current goal, or after `goal_timeout_s` if that is positive. `goal_random_seed: -1` uses nondeterministic sampling; a
non-negative seed makes test runs repeatable. Starting the policy samples a
fresh first goal. The current goal topic remains active even when
`publish_debug` is false and is included by `record_drone_flight_bag.sh`.

Policies train on goals at most 4 m from the ball, so the goal-minus-ball
offset in the observation is capped at `max_goal_distance_m` (default `4.0`)
with `cap_observation_goal` from `drone_soccer.deploy.observation`: a farther
goal is shown pulled in along its true direction. The `<debug_topic_prefix>/goal`
topic still carries the uncapped goal; `<debug_topic_prefix>/obs` shows the
capped observation the policy acted on.

The circle moves counter-clockwise in ENU. The figure eight is a Gerono
lemniscate with `goal_radius` as its half-width and half that value as its
maximum vertical displacement:

```text
x = target_x + goal_radius * cos(theta)
y = target_y + goal_radius * sin(theta) * cos(theta)
theta = goal_phase_rad + 2*pi*time/goal_period_s
[x_rot]   [cos(rotation) -sin(rotation)] [x - target_x]
[y_rot] = [sin(rotation)  cos(rotation)] [y - target_y]
```

```yaml
target_x: 0.0
target_y: 0.0
goal_trajectory: "circle"
goal_radius: 1.5
goal_period_s: 8.0
goal_phase_rad: 0.0
goal_rotation_rad: 0.0
```

Select the figure eight without changing the other parameters:

```yaml
goal_trajectory: "figure8"
goal_rotation_rad: 1.5707963267948966  # long axis along ENU Y
```

Select arrival-triggered random goals inside a rectangular spawn region:

```yaml
goal_trajectory: "random"
goal_spawn_min_xy: [-1.5, -1.5]
goal_spawn_max_xy: [1.5, 1.5]
goal_arrival_radius: 0.35
goal_random_seed: -1
```

Install in the robot container:

```bash
pip install --break-system-packages -r /root/AirStack/robot/ros_ws/src/svg_ground_control/requirements-policy.txt
bws --packages-select svg_ground_control
```

The standard `scripts/start_drone_soccer_lab_tmux.sh` launcher performs the
policy dependency import check and installs `requirements-policy.txt`
automatically when an older or recreated robot container is missing it.
Running `bws`/`colcon` by itself does not install Python `install_requires`.

The read-only `/root/drone_soccer` mount is included in the robot container's
`PYTHONPATH`, so an editable install is neither needed nor writable.

See [experiment.md](experiment.md) (topic table) and
[QUICK_REFERENCE §4](../../../../.codex/skills/yutong-fly-starling/QUICK_REFERENCE.md)
for disarmed checks and flight sequence.

### Online policy-memory updates (`online_memory_learner`)

`online_memory_learner` improves the flying policy in rounds with drone_soccer's
policy-memory fine-tune (`drone_soccer/policy_memory/online.py`):

1. It buffers the live flight of the active policy (mocap poses, commander goal,
   `<prefix>/action` ticks, PX4 setpoints) from 0.5 s after the commander announces
   that policy on `<prefix>/active_policy`.
2. Every 2 s it cuts the buffer into 3 s chunks with the offline `real-chunks` filters
   (tracking loss, goal switch, landed, idle, barrier margin) plus a pause filter
   (chunks need policy ticks throughout). When `min_kept_chunks` pass (60 = 30 s),
   it fine-tunes the active checkpoint in a background thread (about 3-5 s on CPU);
   samples arriving meanwhile are dropped.
3. It saves `policy.zip`, `update.json`, `real_chunks.npz` and `flight_states.npz` under
   `output_root/<session>/update_<n>/` and publishes the policy path on
   `<prefix>/load_policy`. The commander loads it off its control loop, swaps it in at
   the start of a tick and announces it, which starts the next round with an empty buffer.

```bash
ros2 launch svg_ground_control policy_commander.launch.py enable_policy_hot_swap:=true
ros2 launch svg_ground_control online_memory_learner.launch.py
```

Without `enable_policy_hot_swap:=true` the commander ignores load requests and the
learner keeps re-sending its first update. Set `run_dir` / `memory_dir` in
`config/drone_soccer/online_memory_learner_drone3.yaml` to the training run whose
policy memory should drive the updates, and `barrier_dataset` to the day's barrier
polygon in drone_soccer's `BallResidualConfig`. The commander must keep
`publish_debug: true` (the learner counts policy ticks on `<prefix>/action`).
Updated policies have not been evaluated in sim; watch the first swaps closely.

### Online goal-retargeting updates

drone_soccer's goal-conditioned policy update (`drone_soccer/goal_retarget/`,
method and settings in its skill reference `goal-retargeting.md`) can update the
flown policy during a flight. Flown twice on 2026-11-19 (about 10 min each; results
in drone_soccer `experiments/goal_retarget_hw_20261119/`). Two processes
share one run directory:

```mermaid
flowchart LR
    C[policy_commander<br/>robot container] -- "ticks.jsonl<br/>(one row per flown tick)" --> D[(robot/bags/goal_retarget/&lt;run&gt;)]
    D -- ticks --> L[goal_retarget_learner.py<br/>host, gpu conda env]
    L -- "policies/policy_n.zip<br/>updates.jsonl, manifest.json" --> D
    D -- "manifest.json changed" --> C
```

- The **commander** (`goal_retarget_enabled: true`) creates
  `/bags/goal_retarget/<stamp>_<drone>/` at launch, writes `session.json`, and appends
  every flown tick (true goal, drone and ball state, action, waypoint, flying
  iterate) to `ticks.jsonl`. Ticks are grouped in segments: a stop or a tick skipped
  for stale inputs ends one, since the learner matches each tick with the ball
  `t_g` later. Each tick it checks `manifest.json`; a new iterate is loaded in a
  background thread and swapped in between ticks.
- The **learner** (`scripts/run_goal_retarget_learner.sh`) runs on the host, because
  the forward simulation needs a CUDA GPU and the drone_soccer `gpu` conda env (no
  rclpy). Every 5 s it reads new ticks, simulates `t_g` in Warp, judges ticks, and
  after every 200 accepted ticks updates pi_F, saves `policies/policy_<n>.zip`,
  appends the diagnostics to `updates.jsonl`, and rewrites `manifest.json`.
  It refuses a commander whose `policy_dt`, goal cap, policy type or action box differ
  from training, or whose starting policy file on the host does not match the
  commander's (SHA-256).
- **Resuming:** pi_E is always `saved_policies/motor_dynamics_full.zip` (`--expert`
  overrides it); pi_F starts from the commander's `model_path`. Pointing `model_path`
  at an earlier run's last iterate continues from it (fresh LoRA adapters on top of
  it). The goal-retarget YAML currently starts from the 11/19 run's `policy_014.zip`.
- **pi_T is locked to the expert pi_E** by default (the learner sets
  `target_refresh_period_s = inf` and `stagnation_action_change_m = 0`): every label
  is the expert's action at the miss-shifted goal, and pi_F's LoRA adapters keep
  accumulating on pi_E. `--refresh-target` restores the refresh pi_T <- pi_F on
  stagnation or every 120 s of flight. `--replay-all-accepted` fits each update on
  every tick accepted so far instead of the newest 200 (the best variant in
  drone_soccer's sim test; not flown yet).

`policy_commander_drone3_goal_retarget.yaml` is `policy_commander_drone3.yaml` plus
learning, and goals that switch only when the **ball** is within 0.3 m
(`goal_arrival_reference: ball`; the default `drone` keeps the old behavior). There
is no time limit (`goal_timeout_s: 0.0`): the drone keeps trying for a goal until
the ball reaches it. drone_soccer's sim test also replaced goals after 10 s;
set `goal_timeout_s: 10.0` to match it. Learning requires
`policy_type: standard`, `policy_dt: 0.1`, `deterministic: true`, and
`max_goal_distance_m: 4.0`; the commander refuses to start otherwise.

```bash
# robot container (after bws --packages-select svg_ground_control)
ros2 launch svg_ground_control policy_commander.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/policy_commander_drone3_goal_retarget.yaml
# host, AirStack checkout: attaches to the newest run; pass the current barrier polygon
scripts/run_goal_retarget_learner.sh --barrier-dataset data_20261119
#   or --barrier-corners x1 y1 x2 y2 ... (counter-clockwise ball-centre polygon)
```

`start_drone_soccer_lab_tmux.sh --goal-retarget` types both commands (policy launch
with this YAML in the container, the learner in a host tmux session
`drone_soccer_learner`) without running them.

**Watching policy updates.** The consoles show one line per event; the details go
to log files in the run directory.

Commander console: one `WARN` line at the tick the new weights start flying (the
action change is between the old and new policy on that exact state; then how long
the old iterate flew and how many goals it reached):

```text
POLICY UPDATE 2 -> 3 | action change 0.19 m | iterate 2: 48 s, 6 goals
```

Learner console: one line per update (ending in `| pi_T refreshed` when a refresh
happened, only with `--refresh-target`) and a progress line every 30 s
(`--status-period-s`):

```text
18:26:59 UPDATE 3 published | loss 0.615 -> 0.136 | action change 0.64 m | median miss 2.94 m
18:27:20   161 s flown | 17/200 toward update 4 | 27% of judged ticks accepted
```

Healthy updates: some ticks accepted, loss falling, and the action change shrinking
within a pi_T period. Load failures and configuration warnings still appear on the
console.

Detailed logs, `<run>/commander.log` and `<run>/learner.log`: every segment start
and end, load request, and learner pass (ticks read, accepted/pending counts,
verdicts), the full update record, and for each swap:

```text
POLICY UPDATE: now flying learner iterate 3 (policy_003.zip), replacing iterate 2; first tick: segment 1 tick 812
  learner update 3: loss 0.5120 -> 0.2330 | batch action change 0.310 m RMS | label shift 0.640 m RMS | median miss 0.62 m | pi_T refreshed False (1 total) | at 182 s of flight
  action on this state: iterate 2 [0.410, -1.200, 0.930] -> iterate 3 [0.520, -1.050, 0.950] (|change| 0.188 m)
  iterate 2 flew 48.0 s: 6 goals reached, 0 timed out
```

Follow them live with `tail -f robot/bags/goal_retarget/<run>/commander.log` on the host.
The swap is also published (latched, JSON with the iterate, its first tick and the
learner's update record) on `<debug_topic_prefix>/policy_update`, next to the flying
checkpoint's path on `<debug_topic_prefix>/active_policy`; `record_drone_flight_bag.sh`
records both, and each `ticks.jsonl` row carries the iterate that flew it, so a bag
or the run directory can be cut by iterate. Policy-memory and goal-retargeting swaps
share one loader: with both `enable_policy_hot_swap` and `goal_retarget_hot_swap`
on, a policy from either source replaces the flying one; a policy-memory checkpoint flies as iterate
`-1` (`external` in the console line) in `ticks.jsonl` and the logs. With
`goal_retarget_hot_swap: false` the commander records published iterates but keeps
flying the starting policy.

The run directory keeps every iterate (`policy_000.zip` is the expert), the
per-tick rows, `updates.jsonl` and `learner_summary.json` (written on Ctrl-C), so a
flight can be replayed through a fresh learner offline.

## Safety notes

- Teleop drones are CBF-exempt by design — the autonomous drones do the
  dodging. The operator (you) is the safety authority for the obstacle.
- This stack bypasses `drone_safety_monitor`; PX4 failsafes and the RC kill
  switch are the safety net. Configure them before flying.
- Stale odometry (> `state_timeout_s`) → zero-velocity command; stale teleop
  input → zero. `~/hold` is the panic button.
- `CBF emergency push-apart engaged` in the log means the QP went infeasible
  (drones inside each other's safety spheres) — land and investigate.
