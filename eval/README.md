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
certification properties (P1–P6).

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
validation**. Single-episode Isaac flights have run for every policy; see
[PORT_STATUS.md](PORT_STATUS.md) for their results and what still needs checking.

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

SAFE's policies keep their SAFE names: `aggressive`, `conservative`, `adaptive`, the nine
planner wrappers (SAFE's main study flies eight of them; panther is left out there), and the
five learned wrappers cover every policy in SAFE's `paper/main_moving.yaml`. Their profiles in
`eval/agents/agents.py` carry `"source": "safe"`, which selects the 60 Hz state stream and the
one-tick lockstep. AirStack's own older baselines are `airstackrandom`, `airstackaggressive` and
`airstackconservative`. `vitfly` also starts the source VitFly ROS sidecar. `airstack safe --list`
shows exact selectable names. Learned profiles read each source policy's camera
contract and pass it to the AirStack Isaac launcher. Planner profiles enable
RTX lidar where their source sensor contract requires it.

Policy registration and CPU tests pass, and each policy has flown one Isaac episode. Sensor
frames, camera geometry, sidecar startup, controller ownership and PX4 behaviour still need
checking per policy before benchmark results can be claimed.

DROAN flies at AirStack's `max_velocity: 1.0` m/s from `droan.yaml`, which cannot cover SAFE's
60 m paper goal inside its 60 s budget.

## Domains

The AirStack-native path now reads SAFE's Isaac domain registry from a
read-only SAFE source mount. The launcher calls SAFE's world builder for the
USD/procedural scene and its CCT controller for walking humans when
`humans: true` is set, the floating domain included. The source scenario sampler supplies
seeded placements; see [PORT_STATUS.md](PORT_STATUS.md). Set `AIRSTACK_SAFE_LEGACY_DOMAINS=1` only to
inspect the prior five-domain AirStack registry.

## Safety definitions

| Symbol | Meaning |
|--------|---------|
| Drone collider box | the box around iris.usd's collision meshes in the body frame, with each rotor counted as the disk it sweeps; half extents about 0.26 x 0.35 x 0.06 m |
| `l_failure` | clearance from the oriented collider box to the nearest surface; contact at 0 |
| `l_unsafe` | `l_failure` minus the rulebook's 0.30 m `r_unsafe_offset_m` |
| `δ` = 0.3 m | risk-proximate shell width (`--epsilon-unsafe`) |

The ScenarioManager measures the clearance in Isaac every tick. It grows the
box with PhysX `overlap_box` until it touches a collider, to 2 cm resolution
and up to SAFE's `CLEARANCE_SEARCH_REACH_M`. Every PhysX collider counts except
the drone, ground and boundary prims. People count as their character
controller capsules. Arena walls and AirStack's colliderless pool columns are
measured exactly from the box corners. The grown box is a cube margin, so away
from contact the clearance can read up to 1.7 times closer than the true
distance near box edges. Contact is clearance 0, latched per episode, and a
wall touch counts only above 0.3 m. The value travels as `body_clearance` in the
state line. Records without it fall back to the centre geometry less the body
radius.

## Outputs

`runs/airstack/<experiment_tag>_<MMDD>/` under the working directory, in SAFE's
`RunLayout`: `report.html`, `meta/episodes.csv`, and per episode
`raw/<domain>/scenario_N/perturbation_M/trajectories/epT.npz` in SAFE's navigation schema,
which SAFE's replay renderer reads. A `debug:` profile that captures (`default`) also makes
the Isaac launcher render an overview and a drone-tracking camera live, written to
`raw/<domain>/scenario_N/perturbation_M/epT_<outcome>/episode_composite.mp4`. Live capture
slows Isaac to about 0.1x real time, where PX4 loses control, so long runs use
`debug: quiet` with `save_videos: none`.

## Known limitations (v1)

- **SAFE parity**: source-backed worlds and walking humans fly in Isaac, but
  AirStack's PX4 dynamics and collider box differ from SAFE's.
  See [PORT_STATUS.md](PORT_STATUS.md).

- **Frames**: goals published to the stack are converted from Isaac world to
  the stack's local frame using a measured (ground truth − EKF) offset; EKF
  drift during long episodes shifts goal placement accordingly.
- **Crash recovery**: after a collision the harness lands, disarms and teleports,
  and the launcher relaunches PX4 so its EKF starts at the new pose. When Pegasus
  did not autolaunch PX4 it cannot be relaunched, the arm can fail, and the run
  needs a stack restart.
- **Scene-mesh obstacles**: shelves, walls and other stage meshes with PhysX
  colliders are scored, on photoreal stages too. A stage floor counts as an
  obstacle unless its prim path names it ground, floor, plane, terrain, road or
  pavement.
- The legacy AirStack domains behind `AIRSTACK_SAFE_LEGACY_DOMAINS=1` other than
  warehouse have untuned arena origins and scales. SAFE's domains are unaffected.
