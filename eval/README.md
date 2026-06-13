# AirStack × S.A.F.E. Evaluation

**S.A.F.E. (Safe Agile Function Everywhere)** is a task-agnostic benchmark that evaluates whether autonomous agents behave safely under uncertainty. This `eval/` folder integrates AirStack's Isaac Sim stack into S.A.F.E. — it wraps what AirStack already has running rather than replacing any internals.

The benchmark loops over **K domains × N scenarios × P perturbations × T episodes** and produces CVaR-aggregated risk metrics (RE, RP, RET, RPT) and SAFE validation properties (P0–P3b) comparing AirStack's planners side-by-side under identical conditions.

---

## Start here: simple mode

If you just want per-domain **success / collision / timeout rates** with no safety math, use `--simple`. This skips level-set functions and CVaR — no safety definitions required:

```bash
# Conservative baseline, simple outcome rates
~/isaacsim/python.sh eval/run.py --agent conservative --domains warehouse --simple -q

# DROAN planner, simple outcome rates
~/isaacsim/python.sh eval/run.py --agent airstackdroan --domains warehouse --simple
```

`--simple` outputs an outcome bar chart and a performance metrics table. It's the fastest way to verify the integration is wired up correctly before running a full SAFE evaluation.

---

## Folder structure

```
eval/
├── __init__.py
├── run.py                    AirstackRunner(BaseRunner) — CLI entry point
├── adapter.py                l_failure / l_unsafe safety level sets, episode context,
│                             taxonomy (HVC/LVC/AR/SHP/SR), CVaR-ready metrics
├── launch.py                 Isaac Sim SimulationApp bootstrap + Pegasus extension loading
├── agents/
│   └── agents.py             5 Policy subclasses:
│                               AirstackDROAN, AirstackSuperPlanner (TCP ROS sidecars)
│                               Random, Aggressive, Conservative (pure-Python baselines)
├── agent_hooks/
│   ├── airstackdroan.sh      brings up robot-desktop container + launches DROAN planner
│   ├── airstacksuperplanner.sh  same for Super Planner
│   └── airstack_ros_sidecar.py  ROS2 Jazzy bridge running inside Docker; publishes
│                               Odometry/PoseStamped/PointCloud2, subscribes to vel output
├── envs/
│   └── AirstackEnv.py        gymnasium.Env wrapping AirStack's SimulatorManager —
│                             defers vehicle control/sensor reads to existing stack
└── sim/
    ├── environment_config.py  domain registry: warehouse, hospital, office, outdoor, empty
    └── scenario_config.py     configure_scenario(), perturb_scenario(), dynamic obstacle motion
```

---

## Prerequisites

**1. Install the S.A.F.E. benchmark package:**
```bash
pip install -e /path/to/benchmark
```

**2. Isaac Sim** must be installed at `~/isaacsim` (or set `ISAAC_PATH`).

**3. The AirStack robot Docker image** must be built:
```bash
airstack build
```

---

## Running

### Smoke test — one episode, DROAN planner, warehouse domain
```bash
~/isaacsim/python.sh eval/run.py \
  --agent airstackdroan \
  --domains warehouse \
  --num_scenarios 1 --num_perturbations 1 --max_episodes 1 \
  --headless
```

### Full comparison — DROAN vs Super Planner, two domains
```bash
~/isaacsim/python.sh eval/run.py \
  --agents airstackdroan airstacksuperplanner \
  --domains warehouse hospital \
  --num_scenarios 5 --num_perturbations 5 --max_episodes 10 \
  --headless
```

### Pure-Python baselines (no Isaac Sim or Docker needed)
```bash
python eval/run.py --agent conservative --domains warehouse --simple -q
```

### Simple mode (outcome rates only, no safety level-set math)
```bash
~/isaacsim/python.sh eval/run.py --agent airstackdroan --domains warehouse --simple
```

---

## How the ROS bridge works

Each `AirstackDROAN` / `AirstackSuperPlanner` policy communicates with a sidecar process over newline-delimited JSON on TCP:

```
benchmark process (Isaac Sim Python)
        │  {"type": "step", "pos": [...], "target": [...], "obstacles": [...]}
        ▼
airstack_ros_sidecar.py  (inside robot Docker container, ROS 2 Jazzy)
        │  publishes: nav_msgs/Odometry, geometry_msgs/PoseStamped, sensor_msgs/PointCloud2
        │  subscribes: /drone/planning/pos_cmd (TwistStamped)
        ▼
AirStack DROAN / Super Planner (running in same container)
        │  publishes velocity command
        ▼
airstack_ros_sidecar.py  returns: {"cmd": {"velocity": [vx, vy, vz]}}
```

The hook scripts (`airstackdroan.sh`, `airstacksuperplanner.sh`) start the Docker container, build and launch the planner, then start the sidecar. The benchmark connects automatically when `AirstackDROAN.act()` is first called.

**Ports:**

| Agent | Port env var | Default |
|-------|-------------|---------|
| DROAN | `AIRSTACK_DROAN_PORT` | 8780 |
| Super Planner | `AIRSTACK_SUPER_PORT` | 8781 |

---

## Domains

| Domain | Scene | Dynamic obstacles | Flight height |
|--------|-------|-------------------|--------------|
| `warehouse` | Simple Warehouse | 30 pedestrians | 1.5 m |
| `hospital` | Hospital | 25 pedestrians | 1.5 m |
| `office` | Office | 20 pedestrians | 1.5 m |
| `outdoor` | Rivermark Community | 40 pedestrians | 2.5 m |
| `empty` | Procedural flat floor | 0 | 1.5 m |

---

## Safety definitions

| Symbol | Meaning |
|--------|---------|
| `r_failure` = 0.31 m | Drone body contacts obstacle — catastrophic failure |
| `r_unsafe` = 0.61 m | Safety constraint violated but no contact yet |
| `δ` = 0.3 m | Risk-proximate shell width (tunable via `--epsilon-unsafe`) |
| `l_failure` | min(obstacle SDF, wall SDF) − r_failure |
| `l_unsafe` | min(obstacle SDF, wall SDF) − r_unsafe |

---

## Outputs

Results are written to `runs/airstack/<run_id>/`:
- `<agent>/` — RE/RP/CVaR plots, `episodes.csv`, text summary
- `comparison.png` / `comparison_summary.txt` — multi-agent side-by-side (when multiple agents)

---

## Follow-up engineering notes

1. **Verify planner output topic** — `airstack_ros_sidecar.py` subscribes to `/drone/planning/pos_cmd` as `TwistStamped`. Check what DROAN / Super Planner actually publish and update the message type if different.
2. **Per-call socket timeout** — `_TCPSidecarClient` defaults to 1 s. If the planner is slow to respond, increase via the `call_timeout` constructor arg.
3. **Add more domains** — register new `AirstackDomainCfg` entries in `sim/environment_config.py` pointing to AirStack's Isaac Sim USD stages.
4. **Parallel scenarios** — `AirstackEnv` supports single-drone runs only. For parallel lockstep (multiple drones tiled in Isaac), follow the `_run_lockstep` pattern in `integrations/aerial_nav/envs/IsaacSimEnv.py`.
