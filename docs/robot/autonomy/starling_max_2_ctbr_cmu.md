# CMU Starling Max 2 Gamma-Zero backend

ROS development and deployment use the standard AirStack desktop container on the ground computer. Training, evaluation and policy export remain on lecar-S2. This package consumes a frozen exported actor and input manifest; inference runs locally over a private Unix socket, never over the VPN.

## Ground computer startup

Use AirStack's existing `desktop` profile (`robot-desktop` and `gcs`), without a
simulator profile. This worktree sets `AUTOLAUNCH=false`, so startup creates idle
development containers and installs the Foxglove panels; it does not launch flight
or policy processes. Standard container networking and hardware access apply.

```sh
cd /home/dronesoccer/Desktop/AirStack_starling_max_2_ctbr_cmu
airstack up
airstack status
airstack connect robot-desktop
```

If the shell has not loaded the `airstack` function, run `source ~/.bashrc` or use
`./airstack.sh up`. The function selects the checkout from the current directory.
Inside the robot container, use the normal workspace helpers:

```sh
bws --packages-up-to gamma_zero_controller px4_interface svg_ground_control
sws
ros2 pkg prefix gamma_zero_controller
```

Builds, installs, and logs live in `robot/ros_ws/{build,install,log}`. The standard
Compose mounts expose that workspace at `/root/AirStack/robot/ros_ws`. Exported
policy assets and their CPU Python environment live in the Git-ignored
`robot/ros_ws/.starling/` directory, outside `src` so colcon does not discover
Python dependencies as ROS packages. Historical reports and recordings are
preserved under `.starling/validation-archive`; active offline fixtures use
`.starling/validation` (override with `STARLING_VALIDATION_DIR`).

The original AirStack checkout already owns `172.31.0.0/24`. This worktree sets
`AIRSTACK_NETWORK_SUBNET=172.30.0.0/24` to avoid Docker's overlapping-pool error.
Do not reuse the original checkout's Compose project name. If enabling Isaac Sim
later, align its static address and `SIM_IP` with the selected subnet first.

### Coexistence with the original checkout

Compose project names, source mounts, build/install/log directories, bag paths,
and Foxglove configuration directories are separate. This worktree uses GCS SSH
port **2232** and Foxglove bridge port **8776**; the original uses 2222 and 8766.
Use the full container name with `airstack connect` when both projects run,
because the CLI's partial-name search includes all Docker containers.

This is not complete runtime isolation. Both robot containers use host networking,
and `robot/docker/.bashrc` forces ROS domain 1 on this branch. Running controllers
in both can create duplicate publishers, services, and shared aircraft command
paths. Both also share the host GPU/devices and Docker image tags: rebuilding or
pulling a tag from one checkout can change what the other uses on recreation.
Worktrees share Git objects/refs/configuration, though source and build outputs
are separate. Keep `AUTOLAUNCH=false`; do not run both aircraft-control stacks
at once. No changes to the original checkout's source/configuration are required
for this setup.

The existing `offboard` profile is for AirStack's split global-planning role; it
is not a selector for this custom CTBR controller. Start the controller explicitly
only after the configuration and hardware checks below.

## Controller components

`gamma_zero_controller` owns backend selection, policy state/history, reference tracking and fault handling. `gamma_zero_msgs/SelectedCommand` carries exactly one conventional trajectory or normalized FLU CTBR command per tick. The managed PX4Interface rejects legacy command callbacks, forwards the selected command and acknowledges its generation/mode. Only that interface sends Offboard heartbeats; they expire without fresh selected commands.

SVG remains the operator/lifecycle frontend. Its optional `gamma_enabled` hooks route Start to the tested trajectory, Hold to the selected backend, and Land through an acknowledged conventional handback. SVG retains its original lifecycle when this option is false. The PX4 interface now has a configurable command-age watchdog (100 ms by default), including when `gamma_managed=false`; validate this timeout against existing conventional command rates as well. Foxglove's mission status includes Gamma mode, acknowledged owner, reference source, latency and fault. The existing recorder captures all topics, including `/gamma/{svg_reference,selected_command,interface_status,status,trace}` and PX4 input/output topics.

## Tested references, not SVG defaults

| Pattern | X amplitude | Y coefficient | Vertical amplitude | Period | Laps | Settle | Phase ramp |
|---|---:|---:|---:|---:|---:|---:|---:|
| flown | 1 m | 0.5 m | 0.2 m | 40 s | 2 | 10 s | 3 s |
| large | 2 m | 1 m | 0.3 m | 24 s | 2 | 10 s | 3 s |

The original formula is x=Ax sin(theta), y=0.5 Ay sin(2 theta), z=Az sin(theta). Coordinates are relative to the captured airborne origin and heading. The smoothed runs last 93/61 seconds plus the evaluated 2-second terminal hold, then request conventional handback. The separate fault-handback hold is bounded to 30 seconds. `ramp_s=0` reproduces the original abrupt benchmark timing; the default 3-second ramps reproduce the separately evaluated deployment references. `test/fixtures/trajectory_golden.json` contains 404 samples generated by the original S2 code, including both timing variants.

The position and heading loops use the evaluated gains 1 and 1.5, with velocity caps 0.8/1 m/s and yaw-rate cap 1 rad/s. Final body commands are also clipped to the exported training envelope. Acceleration feedforward is not consumed by the policy. Fence braking limits are applied after position correction; their physical effectiveness still requires validation.

## Simple routines

Select one at launch using `trajectory_pattern:=hover`, `forward_backward`, `yaw`, or `altitude`. An empty override uses `trajectory.pattern` from the config (still `flown` by default). Restart the controller to change selection; this is not an in-flight switching service. Keep `reference_mode:=tested_trajectory` so SVG Start selects this routine. Status reports `trajectory_pattern`.

| Pattern | Reference relative to captured start pose |
|---|---|
| hover | Hold position and heading for 20 seconds. |
| forward_backward | Smoothly move 0 → +0.5 → 0 → −0.5 → 0 m along the captured forward axis. |
| yaw | Hold position; smoothly turn 0 → +20° → 0 → −20° → 0 relative to captured heading. |
| altitude | Hold horizontal position/heading; smoothly move 0 → +0.15 → 0 → −0.15 → 0 m vertically. |

Defaults add 10 seconds of settling before the 20-second routine and a 2-second terminal hold (32 seconds total), then use the existing conventional handback path. The four moving legs use quintic smoothstep with zero velocity/acceleration at each waypoint. These are bounded position/heading routines, not the earlier pilot evaluator's abrupt velocity steps. Hover is position hold, not merely a zero-velocity command.

Optional `trajectory` config fields: `simple_duration_s` (20), `forward_distance_m` (0.5), `altitude_distance_m` (0.15), `yaw_amplitude_deg` (20). The simple routines ignore `laps` and `ramp_s`; existing figure-eight timing is unchanged. Translational commands are capped at 0.3 m/s, including correction; configurations with faster nominal references are rejected. Yaw combines wrapped heading error with analytic rate feedforward, capped by `max_yaw_rate`. Planned positional extent plus the tracking margin must fit the configured fence before Start is accepted.

Conventional takeoff → explicit Engage → SVG Start remains the sequence. Engage alone does not start the routine. Hold retains the selected backend; handback disengages the policy. Completion uses the existing latched conventional-hold path, not automatic landing or policy re-engagement. The existing fault-hold timeout still applies. These software routines are not hardware-qualified flight procedures.

## Policy installation and offline checks

On this computer the frozen actor, inputs JSON, and CPU environment have been
copied into `robot/ros_ws/.starling/`. JAX 0.7.2 and Flax 0.12.0 run inside the
normal robot container. On a new computer, obtain the same actor and matching
inputs JSON, then create the CPU environment with these pinned dependencies.

Inside the robot container:

```sh
sws
export STARLING_ASSETS_DIR=/root/AirStack/robot/ros_ws/.starling
export JAX_PLATFORMS=cpu
export PYTHONPATH=/root/AirStack/robot/ros_ws/src/gamma_zero_controller:$PYTHONPATH
"$STARLING_ASSETS_DIR/policy-env/bin/python" -m gamma_zero_controller.policy_worker \
  --bundle "$STARLING_ASSETS_DIR/policy/starling_cmu_301989888.model" \
  --assets "$STARLING_ASSETS_DIR/policy/inputs.json" \
  --socket "$STARLING_ASSETS_DIR/policy.sock"
```

The worker checks the model/actor hashes and warms inference. Use one worker per
controller. It runs only when explicitly started. After an unclean stop, inspect
socket ownership before removing a leftover socket file.

For isolated software-only ROS checks, set `ROS_DOMAIN_ID=87` and
`ROS_LOCALHOST_ONLY=1` in **every** participating shell before starting any nodes.
In a second shell, with the same `STARLING_ASSETS_DIR`:

```sh
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
ros2 launch gamma_zero_controller starling_cmu.launch.py \
  assets:="$STARLING_ASSETS_DIR/policy/inputs.json" \
  policy_socket:="$STARLING_ASSETS_DIR/policy.sock" \
  log:="$STARLING_ASSETS_DIR/unique-sortie.jsonl" bag_dir:=/bags
```

The default `simulation:=true` uses `/starling_cmu_test/interface`; it is not a
real-state shadow mode. This launch needs synthetic state for offline testing.
Use a fresh log filename for each run. The standard desktop container itself is
not network-isolated; never treat a topic namespace alone as network isolation.

## Operator sequence

1. Start recording via `/bag_recorder/record` (`std_srvs/SetBool`).
2. Use SVG Takeoff, wait for ACTIVE holding and healthy state.
3. Call `/gamma_controller/engage` (`std_srvs/Trigger`). Confirm CTBR ownership and no pending handoff in status.
4. SVG Start requests the configured tested trajectory; inspect Gamma status/trace for acceptance. SVG Hold captures a hold on the selected backend.
5. SVG Land requests conventional handback, waits for its matching acknowledgement, then enters LANDING. A handoff failure refuses landing and reports an error; it does not silently descend under CTBR.
6. Stop recording and verify the bag metadata before moving logs.

`/gamma_controller/handback` requests conventional hold without landing. After an ordinary handback, `/gamma_controller/use_svg` explicitly resumes SVG references; it never re-engages the policy. After an invalid-state fault, restart the backend before further operation; see
the unresolved recovery gap below. No fault automatically resumes learned control.

For explicitly selected SVG goals or gamepad operation, use `reference_mode:=svg` (and `teleop_drones:=drone_3` for gamepad operation); configure/start SVG's existing teleop node separately. Default Start never selects an SVG built-in trajectory.

## Live boundary

`simulation:=false` is refused unless an explicitly supplied configuration has verified transforms, thrust calibration and firmware flags. `config/live.example.json` deliberately has null bounds and lever arm. Populate measured values and verify the clock/frame convention before attended props-off testing. Launch the managed interface instead of an existing legacy interface, never alongside it.

Takeoff and landing remain conventional. Policy failures with valid state request conventional hold; invalid state, failed handoff or expired fault hold stops selected output so the interface watchdog stops heartbeats. Actual PX4 Offboard-loss behavior, radio takeover, thrust mapping and controller transitions must be verified on the intended firmware. The simulator and synthetic-FCU tests do not validate these hardware behaviors.

## Current test readiness

The CPU policy, standard-workspace build, synthetic-state lifecycle, recording,
and offline trajectory routines are available for software testing. The bounded
`hover`, `forward_backward`, `yaw`, and `altitude` routines also completed the
archived native MuJoCo checks with zero and 100 ms additional command delay.
Those checks do not qualify aircraft behavior.

Four issues reproduced during offline review remain open before flight:

- Handback can clear a restart-required state fault after fresh state returns.
- A gap in state can count toward the one-second stable-hold engagement gate.
- Hold advances command generation without requiring its acknowledgement.
- A zero hover calibration is accepted and maps force commands to zero collective.

The corresponding report and reproductions are preserved locally under
`.starling/validation-archive/offline-review-20261020/`. Ordinary passing unit
tests do not resolve these findings. Live transforms, thrust calibration,
firmware compatibility, Offboard-loss behavior, and attended props-off checks
also remain unverified. Real-state shadow operation still needs explicit output
isolation and wiring.

## Source provenance

Private port from loco_mjx commit `db888431fb69a9edfd9569fd371cfd370b0a59f3`: the then-current standalone CTBR runtime, `deployment/booster_t1/internal/policy.py` architecture, and the `figure_eight_test` reference functions. The current package names were introduced later. Inputs and checkpoints stay outside Git. No new distribution license is asserted for the copied upstream code; resolve upstream licensing before external redistribution.

Offline fixtures live under `internal/starling_max_2_ctbr_cmu/checks/`. They use `.starling/validation` in this checkout, or `STARLING_VALIDATION_DIR` when supplied. Place the matching `starling_cmu_inputs.json`, `policy_golden.json`, and worker `policy.sock` there before running them. Run ROS fixtures in an isolated test environment; they publish synthetic aircraft state and commands.

File logging uses a bounded background queue; overflow or disk failure latches a runtime failure. Diagnostic trace QoS is best-effort, while mandatory file logging and selected-command acknowledgement remain checked. A missed 20 ms preparation deadline still requests handback; no hard real-time guarantee is claimed for Linux scheduling.
