# `mtl_search` — multi-agent target localization search stack

A team of multirotors searches a prior-weighted area for ground targets. Each
robot flies its own row of a team search plan from `mtl::planner`, which is vendored
unchanged inside `mtl_search_planner`. A native earth-stabilised gimbal camera
points along the planned boresight. A metrics logger scores detections online
against the scenario's ground truth, using the Moon et al. sigmoid and a
miss-product accumulation.

Full walkthrough: [MTL target localization tutorial](../../docs/tutorials/mtl_target_localization.md).

## What it launches

`launch/stack.launch.xml` runs these per robot:

| Block | Module | Notes |
|---|---|---|
| Interface | `interface_bringup/interface.launch.py` | wrapped by design |
| Perception | `topic_keepalive` | keeps Isaac on-demand publishers alive |
| Tasks / behavior | `takeoff_landing_planner`, `drone_safety_monitor` | canonical |
| Global | `mtl_search_planner` | `/<robot>/search_mission` action, `search/plan`, `search/planned_trajectory`, `search/planned_boresight` |
| Local | `mtl_trajectory_follower` | carrot pursuit (L = 1.2·R_min) + gimbal law, 20 Hz, owns `trajectory_controller/tracking_point` |
| Local | `trajectory_controller`, `pid_controller` | tracking point moved to `tracking_point_nominal`; stack-local PID clamp |
| Logging | `mtl_metrics_logger` | `runs/<run_id>/<robot>/{telemetry.csv,detection.json,report.html}` |
| Extras | DDS router (stack-local allowlist), gossip | |

**The one wiring deviation.** The trajectory controller's tracking point is
remapped to `tracking_point_nominal`. The follower owns the canonical
`trajectory_controller/tracking_point`:

- **No search active** (takeoff, land, hover): it forwards the nominal point.
- **Searching**: it publishes its carrot.
- **Search ends or aborts**: it hands control back with
  `set_trajectory_mode(ROBOT_POSE)`.

`config/pid_controller_mtl.yaml` is the stock gain file with one change: the
horizontal position-loop clamp is ±6.5 m/s instead of ±3 m/s, so the aircraft
can reach the planned 6 m/s.

## Configuration

| File | What |
|---|---|
| `config/mission.yaml` | **Source of truth**: search area, prior bumps, targets, aircraft, sensor/detection model, sim gimbal, render |
| `config/scenario.json`, `ground_truth.json`, `belief.png` | Generated bundle, read by the planner, the logger, the Isaac scene and the analysis script |
| `config/pid_controller_mtl.yaml` | PID gains (speed clamp) |
| `config/dds_router_mtl_search.yaml` | Robot↔GCS allowlist: the shared list plus the search and gimbal topics and actions |
| `config/rosbag_mcap_storage.yaml` | MCAP writer options (zstd) for the sortie rosbag |
| `scripts/mtl_sortie.sh` | Per-robot sortie: preflight, rosbag, takeoff, goal watchdog. Runs inside the robot container |
| `../../config/fleets/mtl_search_fleet.yaml` | `robot_1..3` spawns (= scenario agent homes), vehicle `quad_gimbal` |

After editing `mission.yaml` or the fleet spawns, regenerate the bundle. CI
runs the same command with `--check` to confirm the bundle is up to date:

```bash
python3 scripts/mtl_generate_scenario.py            # writes stacks/mtl_search/config/*
python3 scripts/mtl_generate_scenario.py --check    # exit 1 if stale
```

## How to run

```bash
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet mtl_search_fleet --stack mtl_search --play --wait
airstack ready
bash scripts/mtl_start_mission.sh              # takeoff + /robot_N/search_mission on all robots, then analyze
python3 scripts/analyze_mtl_run.py --run-dir runs/latest
```

`ISAAC_SIM_SCRIPT_NAME` must be set explicitly. Otherwise `--fleet` selects
the generic `fleet_spawn.py`, and that script gives each drone a rigid ZED
camera instead of the gimbal, with no search area and no targets.

`mtl_start_mission.sh` runs `scripts/mtl_sortie.sh` from this folder inside every
robot container. That script:

- checks that the robot's planner is up (the robot does not take off otherwise);
- records a rosbag (MCAP) of the sortie into `runs/<run_id>/<robot>/bag/`;
- takes off;
- sends the goal, with an acceptance watchdog and retries.

When all robots are done, the launcher writes the team `report.html` and a team Foxglove file,
`runs/<run_id>/foxglove/<run_id>.mcap`, with an importable `mtl_layout.json`. The Foxglove file
needs `pip install -r scripts/requirements-mtl-viz.txt` on the host.

The tutorial covers the manual per-robot commands, topic checks and the Foxglove topics.

## Known limits

- Each robot plans the whole team problem independently, and the result is
  deterministic because the scenario and seed are identical. Nothing
  re-plans in flight: a robot that aborts leaves its cells unsearched.
- Detection is model-based: the Moon et al. sigmoid on slant range and
  off-boresight angle. Nothing runs a detector on `gimbal/rgb`; the image
  stream is there for inspection and future perception work.
- ROS domain IDs and MAVLink ports come from the fleet order (the fleet
  schema forbids declaring them). The fleet file's comments document them.
- The interface layer is a wrapped include; `wiring.md` shows the observed
  graph.

## wiring.md

This stack does not have a `wiring.md` yet. Generate it on a GPU host, then
commit it. Use either the observed graph of a live bring-up (started as
above) or the wiring test:

```bash
airstack doctor --snapshot --stack mtl_search
airstack test -m wiring --stack mtl_search --fleet mtl_search_fleet --sim isaacsim
```

Only the robot graph is recorded. With the wiring test, the sim side is the
generic fleet spawner, so the `gimbal/*` publishers come from the scene in
the live snapshot only.
