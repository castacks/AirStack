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

The benchmark loops **K domains × N scenarios × P perturbations × T episodes**
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

There are **no custom planner bridges**: DROAN runs exactly as `airstack up`
launches it and is commanded through the same `global_plan` interface the
global planner uses. Velocity baselines (`random`, `aggressive`,
`conservative`) are executed as `trajectory_override` segments on the real
trajectory controller, so every agent flies through the same interface layer.

## Prerequisites

```bash
pip install -e /path/to/benchmark    # provides safe_core (safe-benchmark pkg)
airstack build                       # robot + isaac-sim images built
```

No Isaac python, no ROS on the host — everything ROS-side runs in containers
via `docker exec`.

## Running

S.A.F.E. is a first-class AirStack command (`.airstack/modules/safe.sh`):

```bash
# Smoke test — one episode, DROAN through the full stack, empty grid domain
airstack safe --agent droan --domains empty --simple -y

# DROAN vs baselines in the warehouse, full SAFE stats
airstack safe --agents droan aggressive conservative \
    --domains warehouse -N 5 -P 5 -T 10

# Iterating? Keep the stack up between runs (skips the ~5 min Isaac boot
# when the domain and agent profile don't change)
airstack safe --agent droan --domains empty --simple -y --keep-up
```

(`python eval/run.py ...` is the same entry point without the CLI wrapper.)

Timing expectations: Isaac + PX4 bring-up is minutes; each episode adds
land/teleport/takeoff overhead (~30 s) plus the flight itself. Budget
accordingly when picking K/N/P/T. Domain switches restart the stack.

## Agents

| Agent | Mode | What flies |
|-------|------|------------|
| `droan` | stack | Full stack; DROAN (droan_gl) follows the published global plan |
| `aggressive` / `conservative` / `random` | velocity | Baseline policy → `trajectory_override` on the real trajectory controller |

Profiles (bring-up env, post-up commands, control mode) live in
[`eval/agents/agents.py`](agents/agents.py) `PROFILES`. A SuperPlanner profile
needs a `local_bringup` variant that swaps the local planner — see the TODO
there.

## Domains

Registered in [`eval/sim/environment_config.py`](sim/environment_config.py);
arena bounds are centered on the Isaac world origin.

| Domain | Scene | Dynamic obstacles | Notes |
|--------|-------|-------------------|-------|
| `empty` | default grid | 0 | sanity/smoke domain |
| `warehouse` | Simple Warehouse (Nucleus) | 30 | `stage_scale=0.01` |
| `hospital` / `office` / `outdoor` | Nucleus scenes | 25 / 20 / 40 | arena origins untuned — verify before trusting results |

Obstacles are cylinder prims spawned by the ScenarioManager (visible to the
drone's ZED/lidar); "pedestrians" walk waypoint patterns, with speeds
resampled ±20% per perturbation (`perturb_scenario`).

## Safety definitions

| Symbol | Meaning |
|--------|---------|
| `r_failure` = 0.31 m | drone body contacts obstacle — catastrophic failure |
| `r_unsafe` = 0.61 m | safety constraint violated, no contact yet |
| `δ` = 0.3 m | risk-proximate shell width (`--epsilon-unsafe`) |
| `l_failure` / `l_unsafe` | min(obstacle SDF, wall SDF) − radius |

Collision ground truth is the ScenarioManager's analytic contact check
(latched per episode), consistent with the same SDFs used for scoring.

## Outputs

`eval/runs/airstack/<run_id>/` — per-agent RE/RP/CVaR plots, `episodes.csv`,
text summary; `comparison.*` for multi-agent runs.

## Known limitations (v1)

- **Frames**: goals published to the stack are converted from Isaac world to
  the stack's local frame using a measured (ground truth − EKF) offset; EKF
  drift during long episodes shifts goal placement accordingly.
- **Crash recovery**: after a collision the harness lands/disarms/teleports;
  if PX4 refuses to re-arm after a hard crash the run needs a stack restart.
- **Scene-mesh obstacles** (shelves, walls of the USD scenes) are not in the
  scoring SDF — only spawned cylinders, pedestrians, and arena walls. Choose
  arena bounds over open floor.
- Non-warehouse Nucleus domains have untuned arena origins/scales.
