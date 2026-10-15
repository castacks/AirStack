# AirStack × S.A.F.E. Evaluation

**S.A.F.E. (Safe Agile Function Everywhere)** is a task-agnostic benchmark that
evaluates whether autonomous agents behave safely under uncertainty. This
`eval/` folder is AirStack's S.A.F.E. **integration** (integration-author role,
per the S.A.F.E. README): it depends on the installed `safe-benchmark` package
(`safe_core` only) and wraps **the real AirStack system** — Isaac Sim through
AirStack's own Pegasus bring-up, PX4 SITL, and the full autonomy stack
(perception → planner → trajectory controller → MAVROS) in the robot
container. S.A.F.E. does not fly the drone or replace any internals; it sets
up scenarios, hands the planner a real global plan, observes ground truth, and
scores.

Current port status and the AirStack-native parity gates are in [PORT_STATUS.md](PORT_STATUS.md). The benchmark loops **K domains × N scenarios × P perturbations × T episodes**
and produces CVaR-aggregated risk metrics (RE, RP, RET, RPT) and SAFE
certification properties (P0–P3b).

## Architecture

```
host (plain python + safe_core + docker CLI)
│ eval/run.py            AirstackRunner(BaseRunner)
│ eval/adapter.py        level sets, taxonomy, costs (unchanged S.A.F.E. math)
│ eval/envs/AirstackStackEnv.py
│     reset(): airstack up → land/teleport → place obstacles → takeoff action
│              → publish /robot_1/global_plan
│     step():  wait env_dt of SIM time → sample ground truth → obs/collision
│ eval/stack/compose.py  airstack up/down + readiness gates (mirrors tests/)
│ eval/stack/bridge.py   host handle on the in-container bridge
│
├── robot container (unmodified stack, AUTONOMY_ROLE=full)
│     eval/stack/container_bridge.py   (docker exec -i, rclpy)
│        publishes:  /robot_1/global_plan, …/trajectory_override
│        subscribes: /robot_1/odometry_conversion/odometry
│     takeoff/land via ros2 action send_goal /robot_1/tasks/{takeoff,land}
│
└── isaac-sim container
      simulation/isaac-sim/launch_scripts/safe_eval_launch_script.py
         = AirStack's standard single-drone PX4+ZED bring-up
         + ScenarioManager: obstacle prims, vehicle teleport, ground-truth
           stream + analytic collision — JSON lines over TCP :8899 on the
           docker network (the Kit interpreter cannot host an rclpy node,
           so this link is plain TCP, like PX4 SITL's own MAVLink link;
           eval/stack/scenario_client.py is the host side)
```

SAFE source policies are imported from the separate SAFE checkout. The main-study
set has 16 AirStack adapters: source velocity baselines, learned policies, and
ROS sidecar planners. `droan` remains AirStack's own planner. The SAFE planner
sidecars are launched by SAFE's hooks, while their outputs go through the
AirStack ROS/PX4 command bridge; the AirStack PID output is gated during direct
external control. The registered adapters and CPU tests are **not flight
validation**. Build the updated robot workspace and run a one-episode Isaac/PX4
smoke test before using their results; see [PORT_STATUS.md](PORT_STATUS.md).

## Prerequisites

```bash
pip install -e /path/to/benchmark    # provides safe_core (safe-benchmark pkg)
airstack build                       # robot + isaac-sim images built
```

No Isaac python, no ROS on the host — everything ROS-side runs in containers
via `docker exec`.

## Running

S.A.F.E. is a first-class AirStack command (`.airstack/modules/safe.sh`):

Every run knob lives in an experiment yaml (keys are the argparse dest
names: `agents`, `domains`, `scenarios`, `perturbations`, `episodes`,
`max_episode_time`, `env_dt`, `simple`, `keep_up`, `goal_tolerance`,
`takeoff_velocity`, `epsilon_unsafe`, `save_videos`); the CLI carries only
actions and session facts (`--config`, `-y`, `--headless`, `--list`,
`--resume`, `--compare`, `-v`):

```bash
# One DROAN episode through the full stack on the empty grid domain
airstack safe --config eval/experiments/droan_empty.yaml -y

# `keep_up: true` in the yaml leaves the stack running after a run for
# inspection; a new run restarts it
airstack safe --list                                 # agents and domains
```

(`python eval/run.py ...` is the same entry point without the CLI wrapper;
`--config` takes a path, or a name resolved against `eval/experiments/`. The
yaml must set `robot: iris`, since AirStack's bring-up spawns the Pegasus Iris.)

Timing expectations: Isaac + PX4 bring-up is minutes; each episode adds
land/teleport/takeoff overhead (~30 s) plus the flight itself. Budget
accordingly when picking K/N/P/T. Domain switches restart the stack.

## Agents

`safeaggressive`, `safeconservative`, `safeadaptive`, the nine `safe*`
planner wrappers (SAFE's main study flies eight of them; panther is left out
there), and the five `safe*` learned wrappers cover every policy in SAFE's
`paper/main_moving.yaml`. The `safe` prefix distinguishes source
policies from AirStack's older `aggressive` and `conservative` baselines.
`safevitfly` also starts the source VitFly ROS sidecar. `airstack safe --list`
shows exact selectable names. Learned profiles read each source policy's camera
contract and pass it to the AirStack Isaac launcher. Planner profiles enable
RTX lidar where their source sensor contract requires it.

Policy registration and CPU tests pass. Actual sensor frames, camera geometry,
sidecar startup, controller ownership, and PX4 behavior need an Isaac flight
check before benchmark results can be claimed.

## Domains

The AirStack-native path now reads SAFE's Isaac domain registry from a
read-only SAFE source mount. The launcher calls SAFE's world builder for the
USD/procedural scene and its CCT controller for walking humans when
`humans: true` is set. The source scenario sampler now supplies seeded placements; the contact
check still needs Isaac flight validation; see
[PORT_STATUS.md](PORT_STATUS.md). Set `AIRSTACK_SAFE_LEGACY_DOMAINS=1` only to
inspect the prior five-domain AirStack registry.

## Safety definitions

| Symbol | Meaning |
|--------|---------|
| `r_failure` = 0.61 m | 0.31 m drone contact radius + 0.30 m obstacle radius |
| `r_unsafe` = 0.91 m | contact radius plus a 0.30 m clearance buffer |
| `δ` = 0.3 m | risk-proximate shell width (`--epsilon-unsafe`) |
| `l_failure` / `l_unsafe` | horizontal center clearance to spawned obstacles and arena walls, minus the corresponding threshold |

Collision ground truth is the ScenarioManager's analytic contact check
(latched per episode), consistent with the same SDFs used for scoring.

## Outputs

`runs/airstack/<experiment_tag>_<MMDD>/` under the working directory, in SAFE's
`RunLayout`: `report.html`, `meta/episodes.csv`, and per episode
`raw/<domain>/scenario_N/perturbation_M/trajectories/epT.npz`. Episode video comes
from live capture: a `debug:` profile that captures (`default`) makes the Isaac
launcher render an overview and a drone-tracking camera, written to
`raw/<domain>/scenario_N/perturbation_M/epT_<outcome>/episode_composite.mp4` at
1/env_dt frames per second. There is no post-hoc replay renderer for this
integration, so keep `save_videos: none`.

## Known limitations (v1)

- **SAFE parity**: source-backed worlds and walking humans are wired in but
  have not been validated in Isaac; AirStack PX4 dynamics differ from SAFE.
  See [PORT_STATUS.md](PORT_STATUS.md).

- **Frames**: goals published to the stack are converted from Isaac world to
  the stack's local frame using a measured (ground truth − EKF) offset; EKF
  drift during long episodes shifts goal placement accordingly.
- **Crash recovery**: after a collision the harness lands/disarms/teleports;
  if PX4 refuses to re-arm after a hard crash the run needs a stack restart.
- **Scene-mesh obstacles** (shelves, walls of the USD scenes) are not in the
  scoring SDF — only spawned cylinders, pedestrians, and arena walls. Choose
  arena bounds over open floor.
- Non-warehouse Nucleus domains have untuned arena origins/scales.
