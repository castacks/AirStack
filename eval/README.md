# AirStack × S.A.F.E. Evaluation

**S.A.F.E. (Safe Agile Function Everywhere)** is a task-agnostic benchmark that
evaluates whether autonomous agents behave safely under uncertainty. This
`eval/` folder is AirStack's S.A.F.E. **integration** (integration-author role,
per the S.A.F.E. README). It imports SAFE's core and its navigation task
(`safe_core`, `safe_navigation`, `safe_isaac`, `safe_sim`) from the benchmark
checkout and wraps **the real AirStack system**: Isaac Sim through AirStack's
own Pegasus bring-up, PX4 SITL, and the full autonomy stack (perception →
planner → trajectory controller → MAVROS) in the robot container. S.A.F.E.
does not replace any AirStack internals. It builds SAFE's worlds and
scenarios in AirStack's Isaac, hands the policy under test its goal, observes
ground truth, and scores.

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
│     step():  advance Isaac one tick (SAFE policies) or wait env_dt of sim
│              time (DROAN, airstack* baselines) → sample ground truth → obs
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
         + ScenarioManager: SAFE's world and people, vehicle teleport, ground-truth
           stream + PhysX clearance — JSON lines over TCP :8899 on the
           docker network (the Kit interpreter cannot host an rclpy node,
           so this link is plain TCP, like PX4 SITL's own MAVLink link;
           eval/stack/scenario_client.py is the host side)
```

SAFE's policies are imported from the benchmark checkout, never copied. The SAFE planner
sidecars are launched by SAFE's hooks, while their outputs go through the AirStack ROS/PX4
command bridge; the AirStack PID output is gated during direct external control. `droan`
is AirStack's own planner. See [PORT_STATUS.md](PORT_STATUS.md) for the flights so far.

## Prerequisites

```bash
pip install -e /home/kayla/workspace/benchmark
export PYTHONPATH=/home/kayla/workspace/benchmark:/home/kayla/AirStack
airstack build                       # robot + isaac-sim images built
```

No Isaac python, no ROS on the host — everything ROS-side runs in containers
via `docker exec`.

## Running

S.A.F.E. is a first-class AirStack command (`.airstack/modules/safe.sh`):

Every run knob lives in an experiment yaml (`robot`, `agents`, `env_domains`,
`num_scenarios`, `num_perturbations`, `max_episodes`, `max_episode_time`,
`goal_distance`, `env_dt`, `humans`, `simple`, `keep_up`, `workers`, `debug`,
`save_videos`, as in `eval/experiments/`); the CLI carries only actions and
session facts (`--config`, `-y`, `--headless`, `--list`, `--resume`, `--compare`, `-v`):

```bash
# One DROAN episode through the full stack in SAFE's empty room
airstack safe --config eval/experiments/droan_empty.yaml -y --headless

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

SAFE's policies keep their SAFE names. The velocity baselines (`random`, `aggressive`,
`conservative`, `adaptive`, `adaptiveapf`), the nine planner wrappers and the five learned
wrappers cover every policy in SAFE's `paper/isaac_moving_primitive.yaml`. Their profiles in
`eval/agents/agents.py` carry `"source": "safe"`, which selects the 60 Hz state stream and the
one-tick lockstep. Velocity policies fly at the m/s they command, with no ceiling. AirStack's
own older baselines are `airstackrandom`, `airstackaggressive` and `airstackconservative`.
`vitfly` also starts the source VitFly ROS sidecar. `airstack safe --list` shows exact
selectable names. Learned profiles read each source policy's camera contract and pass it to
the AirStack Isaac launcher. Planner profiles enable RTX lidar where their source sensor
contract requires it.

AirStack runs DROAN as `droan_gl`, whose candidate trajectories head out at a fixed 1 m/s
written in `droan_gl/src/gl_interface.cpp` (it has no speed parameter), which cannot cover
SAFE's 60 m paper goal inside its 60 s budget.

## Domains

The domains are SAFE's Isaac domains, read from a read-only mount of the benchmark
checkout. The launcher calls SAFE's world builder for the USD or procedural scene, loads
SAFE's baked stages (every mesh given a static collider) and stage exports, and runs SAFE's
character controller for walking people when `humans: true` is set, the floating domain
included. SAFE's scenario sampler supplies the seeded placements.

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
`raw/<domain>/scenario_N/perturbation_M/trajectories/epT.npz` in SAFE's navigation schema.
Episode video comes only from live capture: a `debug:` profile that captures (`default`)
makes the Isaac launcher render an overview and a drone-tracking camera, written to
`raw/<domain>/scenario_N/perturbation_M/epT_<outcome>/episode_composite.mp4`. Live capture
slows Isaac to about 0.1x real time, where PX4 loses control, so long runs use
`debug: quiet` with `save_videos: none` and have no video.

## Notes

- **Control path**: episodes fly AirStack's PX4, Pegasus Iris and trajectory
  controller, so they form their own experiment beside SAFE's runs even with
  matching names and seeds.
- **Goal frame**: goals reach the stack in its EKF odometry frame through the
  offset between ground truth and EKF, measured once per episode after takeoff.
- **Between episodes**: the harness lands, disarms and teleports the drone, and
  the launcher relaunches PX4 so its EKF starts at the new pose.
- **Stage meshes**: every PhysX collider on a stage is an obstacle, as in SAFE.
  Prims whose path names them ground, floor, plane, terrain, road or pavement
  are treated as the floor.
- **Timing**: DROAN and the `airstack*` baselines run against Isaac on its own
  clock, as the full stack runs on a real drone, so their repeats vary with
  timing. SAFE's policies step Isaac one tick at a time and repeat exactly.
