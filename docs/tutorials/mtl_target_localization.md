# Multi-Agent Target Localization (MTL) search in Isaac Sim

Three PX4 multirotors search a 400 m × 400 m area for 15 ground targets. The search is
weighted by a prior belief map. Each robot:

- flies its own row of a **team** search plan from `mtl::planner`;
- points a **native, ROS 2-driven gimbal camera** along the planned boresight.

A logger scores every target's detection probability online. After the flight,
one script fuses the three robots into a team report. The prior is a probability mass
function: it sums to 1 over the area. The number to compare planners on is the **residual
belief mass**, the probability that the search missed the target. Lower is better.

| Piece | Where |
|---|---|
| Stack | [`stacks/mtl_search`](../../stacks/mtl_search/README.md) |
| Planner | [`mtl_search_planner`](../../robot/ros_ws/src/global/planners/mtl_search_planner/README.md) |
| Follower + gimbal law | [`mtl_trajectory_follower`](../../robot/ros_ws/src/local/controls/mtl_trajectory_follower/README.md) |
| Scorer / report | [`mtl_metrics_logger`](../../robot/ros_ws/src/behavior/mtl_metrics_logger/README.md) |
| Messages | [`mtl_msgs`](../../common/ros_packages/msgs/mtl_msgs/README.md) |
| Isaac scene + gimbal | `simulation/isaac-sim/launch_scripts/search_mission_scene.py` ([gimbal docs](../robot/autonomy/sensors/gimbal.md#native-mtl-gimbal-search_mission_scenepy)) |
| Fleet / vehicle | `config/fleets/mtl_search_fleet.yaml`, `config/vehicles/quad_gimbal/` |

## Frames in one paragraph

The scenario is written in **mission NED** (`n, e, d`). The Isaac world is **ENU**:
`x = e`, `y = n`, `z = −d`. Each robot's `map` frame is its PX4/MAVROS local origin, which is
its spawn point, so `p_map = p_world − home`. The planner publishes each robot's track in
that robot's `map` frame, so no robot needs to know where the others are. Gimbal angles
are earth-frame (ENU) Z-Y-X: `x = roll`, `y = pitch`, `z = yaw`, with `pitch > 0` looking
down.

## 0. (Optional) change the mission

`stacks/mtl_search/config/mission.yaml` is the single source of truth. It defines:

- the area, prior bumps and targets (the prior is normalised to sum to 1 after the bumps
  are summed, capped and floored);
- the valid-cell threshold `mapping.minimum_belief_mass`: a 20 m cell is kept when the
  probability that the target is in it is greater than this. It is a per-cell probability,
  so work it out again if you change the area, the cell size or the bumps;
- the aircraft;
- the sensor and detection model;
- the budget.

After editing it, or the spawns in the fleet file, regenerate the bundle that the planner,
the logger and the Isaac scene read:

```bash
python3 scripts/mtl_generate_scenario.py           # -> stacks/mtl_search/config/{scenario.json,ground_truth.json,belief.png}
python3 scripts/mtl_generate_scenario.py --check   # verify the bundle is current
```

You can also rehearse the mission kinematically, with no Docker, ROS or Isaac. This needs
the `mtl_search_plan` CLI built on the host (see the script's `--help`):

```bash
python3 scripts/mtl_offline_mission.py --run-id offline    # -> runs/offline/{robot_N/,report.html,...}
```

## 1. Build the packages (first time, or after changing them)

The robot containers share `robot/ros_ws`, so building in one container builds for all of
them:

```bash
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet mtl_search_fleet --stack mtl_search --play --wait
docker exec airstack-robot-desktop-1 bash -ic \
  "bws --packages-up-to mtl_msgs mtl_search_planner mtl_trajectory_follower mtl_metrics_logger"
airstack down && ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet mtl_search_fleet --stack mtl_search --play --wait
```

`bws` and `sws` are shell functions defined in the container's `~/.bashrc`. They exist only in
an interactive shell: use `bash -ic` from the host, or type them directly inside
`airstack connect`. A plain `bash -c` reports `bws: command not found`.

!!! note "Set `ISAAC_SIM_SCRIPT_NAME` explicitly"
    With `--fleet`, `airstack up` otherwise picks the generic `fleet_spawn.py`. That script
    gives each drone a rigid ZED camera instead of the gimbal, with no search area and no
    targets.

## 2. Bring up and wait for flight-ready

```bash
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet mtl_search_fleet --stack mtl_search --play --wait
airstack ready
```

`robot_N` runs in ROS domain `N` with PX4 MAVLink ports `14540+N` and `14580+N`. These
are derived from the fleet order, and the fleet file's comments tabulate them.

Sanity checks, on robot 1 for example:

```bash
docker exec airstack-robot-desktop-1 bash -ic "sws; ros2 topic hz /robot_1/gimbal/rgb"            # ~15 Hz
docker exec airstack-robot-desktop-1 bash -ic "sws; ros2 topic echo --once /robot_1/gimbal/camera_info"
docker exec airstack-robot-desktop-1 bash -ic "sws; ros2 topic echo --once /robot_1/gimbal/state"  # parked: y ~ 1.047 (60 deg)
docker exec airstack-robot-desktop-1 bash -ic "sws; ros2 topic echo --once /robot_1/search/plan --field plan_id"   # boot preview plan
```

Move the gimbal by hand. Only do this while no search is running, because the follower
re-parks the gimbal at 20 Hz:

```bash
docker exec airstack-robot-desktop-1 bash -ic "sws; ros2 topic pub --once /robot_1/gimbal/cmd_pitch_yaw geometry_msgs/msg/Vector3 '{x: 0.0, y: 1.5708, z: 0.0}'"
```

## 3. Fly the team search

This one command does everything for all robots:

```bash
bash scripts/mtl_start_mission.sh            # -n 3 -a 30 by default; --dry-run plans without flying
```

For each robot, in parallel, the script runs `stacks/mtl_search/scripts/mtl_sortie.sh`
inside that robot's container. The in-container script does this:

1. **Preflight.** The robot's `mtl_search_planner` node and its `/robot_N/search_mission`
   action must be up. Otherwise the robot **stays on the ground** and the log shows why.
2. **Record.** It starts a rosbag (MCAP, zstd) of everything worth replaying into
   `runs/<run_id>/robot_N/bag/`: TF, odometry, MAVROS pose/GPS/state, gimbal camera + info +
   state + command, plan, follower status, carrot/aim, footprint, detection markers,
   metrics, tracking points, action feedback.
3. **Take off.** It waits for a healthy state estimate, then takes off to 30 m. The
   takeoff must report `success: true`.
4. **Start the search.** It sends `/robot_N/search_mission` with the shared `run_id`. If
   the planner doesn't accept the goal within 20 s, the script prints diagnostics (nodes,
   action servers, last plan, follower state) and retries, up to 3 attempts.
5. **Finish.** It streams feedback until the result, then closes the bag.

When all robots are done, the launcher writes the team report (`analyze_mtl_run.py`) and the
team Foxglove file (`mtl_foxglove.py`, see step 5), and points `runs/latest` at the run.

Options: `--no-images` records without the ~14 MB/s-per-robot raw camera stream,
`--no-record` records nothing, and `--no-foxglove` skips the export.

The same thing by hand, for robot `N`:

```bash
docker exec -e ROS_DOMAIN_ID=N airstack-robot-desktop-N bash -ic "sws; \
  ros2 action send_goal /robot_N/tasks/takeoff task_msgs/action/TakeoffTask '{target_altitude_m: 30.0, velocity_m_s: 2.0}' && \
  ros2 action send_goal --feedback /robot_N/search_mission mtl_msgs/action/SearchMission '{start_mission: true, run_id: \"my_run\"}'"
```

While it flies:

- `/robot_N/search/follower_status` shows `SEARCH`, progress and cross-track error.
- `search/carrot` and `search/aim_point` are the pursuit and gimbal targets.
- `search/detection_markers` colours the targets by P_det.

Cancelling a goal aborts that robot's search. The follower then hands control back to the
trajectory controller, and the drone holds its position.

## 4. Analyze

```bash
python3 scripts/analyze_mtl_run.py --run-dir runs/latest
```

The script writes these into the run folder:

- the **team** `telemetry.csv`, `detection.json`, `residual_belief.csv` and `report.html`;
- a printed summary. It starts with the **residual belief mass**, flown and planned, then
  lists targets found, mean time to detect, the valid cells the team reached versus the plan,
  and the distance flown.

**Residual belief.** Every pixel `x` of the prior gets the same Bayes update as a target
standing there. That is the same footprint gate and Moon et al. sigmoid, with the
`dt / dt_ref` exponent:

```
residual(x)  = prior(x) · Π_looks (1 − P(z|x))^(dt/dt_ref)
residualMass = Σ_x residual(x) = P(the search missed the target)      # lower is better
```

`1 − residualMass` is how much of the prior the team actually searched. The report shows the
prior and the residual side by side on one colour scale: swept belief goes dark, and
unsearched belief stays bright. It also plots the residual over time against the planned
value. `residual_belief.csv` holds both maps per 10 m block (`x, y, prior, residual`, world
ENU, block sums). Compare two planners only on the same prior, the same sensor parameters
(`fov`, `a`, `b`, `c`, `beta`, `p_out_of_range`) and the same `dt_ref`.

Runs recorded before this change can still be scored again. Their `scenario.json` has the
prior bumps, and the analysis normalises the prior itself. Their cell masses stay in the old
raw units.

Each robot also has its own report, which the logger writes at the end of its search:
`runs/<run_id>/robot_N/report.html`. See [`runs/README.md`](../../runs/README.md) for the layout.

## 5. Replay the whole team in Foxglove

```bash
pip install -r scripts/requirements-mtl-viz.txt              # once, on the host
python3 scripts/mtl_foxglove.py --run-dir runs/latest        # the launcher already ran this
# -> runs/<run_id>/foxglove/<run_id>.mcap  and  runs/<run_id>/foxglove/mtl_layout.json
```

Open the `.mcap` in Foxglove (desktop app or app.foxglove.dev, **Open local file**). Then
use **Layouts → Import from file** and choose `mtl_layout.json`.

The robots' own bags all reuse the frame names `map` and `base_link`, each with a different
origin. The export puts the whole team in one `world` frame:
`world → robot_N/map → robot_N/base_link → robot_N/gimbal → robot_N/camera_optical`.

| Topic | Shows |
|---|---|
| `/world/area`, `/world/belief` | search boundary, valid cells (shade = prior mass), homes; the prior as a colour-mapped grid (densest block = 1) |
| `/world/residual` | the residual belief left so far, at 1 Hz, on the prior's colour scale. It is hidden in the layout: show it and hide `/world/belief` to watch the search sweep the belief away |
| `/world/targets` | ground-truth targets, coloured by their **live** P_det (red → green), labelled with discovery time and finder |
| `/robot_N/model`, `/robot_N/frustum` | the drone and its camera frustum, following the recorded pose and gimbal |
| `/robot_N/plan`, `/robot_N/trail` | planned track and planned boresight ground track; flown trail |
| `/robot_N/sensor` | camera footprint circle, boresight ray, aim point, carrot, status label (phase, progress, speed, altitude) |
| `/robot_N/camera/image` + `/calibration` | the gimbal camera (JPEG), in the Image panels and projectable in 3D |
| `/robot_N/gps` | Map panel track |
| `/robot_N/telemetry`, `/team/metrics` | plottable speed, altitude, XTE, progress, gimbal cmd vs measured, slant, footprint, pointing error; residual belief (`residual_mass`), targets found, valid-cell mass covered, distance, per-target P_det |
| `/events` | phase changes and target discoveries (Log panel) |
| `/raw/robot_N/...` | every recorded ROS topic, unchanged, for Raw Messages and Plot |

Runs recorded before the rosbag existed still export from `telemetry.csv`: trails, footprints,
targets and plots, but no camera. Each robot's bag also opens on its own in Foxglove.

## 6. Record the observed wiring (once, on a GPU host)

```bash
airstack doctor --snapshot --stack mtl_search    # with the stack running (step 2)
# commit the generated stacks/mtl_search/wiring.md
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `search_mission` fails: *agent 'robot_N' is not in the scenario team* | `ROBOT_NAME` doesn't match a `team.agents[].name`; the fleet and bundle are out of sync, so rerun `mtl_generate_scenario.py` |
| Isaac log: *spawns at (…) but the scenario home is (…)* | same as above; the robots would fly offset tracks |
| No `/robot_N/gimbal/*` topics | the scene wasn't `search_mission_scene.py` (see the note in step 1), or the timeline isn't playing |
| Drone flies the track at ~3 m/s | the stack-local `pid_controller_mtl.yaml` isn't loaded (±3 m/s stock clamp) |
| Search aborts at start | stale odometry or a safety-monitor timeout; check `search/follower_status.state_name` and the follower log |
| A robot takes off and then hovers, never searching | its planner never accepted the goal. The sortie log (`runs/<run_id>/_launcher_logs/robot_N.log`) now shows the preflight / watchdog diagnostics. If the planner node is missing, look at its output in that robot's launch tmux (`airstack connect`) |
| `mtl_foxglove: missing dependency` | `pip install -r scripts/requirements-mtl-viz.txt` on the host |
