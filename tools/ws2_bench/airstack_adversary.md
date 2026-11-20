# CLAUDE.md — airstack_adversary

Project context for Claude Code. Read this first, then
`airstack_attack_sim_implementation.md` (the full implementation brief).

## What this is

A ROS 2 package of adversarial attacks against the **AirStack** autonomy stack,
run in **IsaacSim** simulation, for **defensive red-teaming**: implement each attack
from the drone-autonomy-stack threat model, pair it with a success metric, and use it
to test detectors and hardening (consistency gates, estimator monitors, comms auth).

## Hard rules (do not violate)

1. **Simulation only.** No code that targets real RF, real airframes, or hardware.
   Every "physical" (black-box) attack is a software perturbation of *simulated*
   sensor data or the USD scene. Do not add RF, transmitter, or real-GPS logic.
2. **Keep the AirStack tree unmodified.** All attack code lives in
   `airstack_adversary/`. Realize attacks via add-on nodes + launch-time topic
   remaps, not edits to AirStack source. The only exceptions are the white-box
   attacks (W1, W2, W5, W6, W7), which inherently need node internals — when one of
   those requires touching AirStack, isolate it behind a clearly labeled patch under
   `white_box_patches/` and flag it in the PR, never edit AirStack in place.
3. **Confirm names before coding.** Topic/node names in the brief are anchored to the
   system diagram, NOT verified against source. Before implementing any attack,
   resolve the item in the brief's §8 checklist against the real tree
   (`ros2 topic list`, `ros2 node info`, grep the AirStack source). Treat every
   `code-font` AirStack topic name as a hypothesis until confirmed.
4. **Config over code.** Switching or tuning an attack must be a YAML edit under
   `experiments/`, not a code change. Don't hardcode targets, rates, or triggers.
5. **Every attack ships with a metric.** No attack is "done" until it emits a
   `summary.json` (effect + detection + fidelity note + defense exercised) per the
   brief's §6.

## Layout

```
airstack_adversary/
├── airstack_adversary/
│   ├── interposer_node.py        # generic sub → perturb → pub (the core pattern)
│   ├── perturbations/            # bias_walk, replay_buffer, flood, phantom_injector, dropper, blob_fuzzer
│   ├── triggers/                 # time, geofence, vehicle_state, msg_count
│   └── metrics/                  # recorder.py, summarize.py
├── usd_hooks/                    # IsaacSim scene scripts (B2, B3, B9)
├── sensor_bridge_patches/        # faithful B1/B4/B5 (Isaac sensor output edits)
├── white_box_patches/            # isolated W-series patches, labeled, never in-place
├── launch/<attack_id>.launch.py  # per-attack launch with the right remaps
├── experiments/<attack_id>.yaml  # the configs
└── results/<run_id>/             # bag + summary.json
```

## The core pattern (MITM interposer)

Attack a topic by owning its name: remap the real producer to a shadow name, have
the interposer subscribe to the shadow, apply a perturbation, and publish on the
real name so consumers receive the tampered data.

```
<producer> --(remap)--> /topic__truth → [interposer: perturb] → /topic → <consumers>
```

Prefer this for all gray-box (R-series) and the shortcut path of sensor attacks. Use
`sensor_bridge_patches/` (faithful) when the defense under test lives below
perception, and `usd_hooks/` when the attack must exercise real raycasting/rendering.

## Build & run

ROS 2 workspace (confirm distro against the AirStack container; likely Humble):

```bash
# build (from workspace root)
colcon build --packages-select airstack_adversary
source install/setup.bash

# run one experiment
ros2 launch airstack_adversary <attack_id>.launch.py config:=experiments/<attack_id>.yaml

# baseline vs attack, emits results/<run_id>/summary.json
./scripts/run_experiment.sh <attack_id>
```

Match `ROS_DOMAIN_ID` and the DDS/container network to the AirStack + IsaacSim
bringup, or interposer nodes won't see the graph. Record a clean baseline run before
any attack run so metrics have a reference.

## Workflow

- Pick the next attack from the brief's phased roadmap (§5). Order:
  Phase 0 scaffolding → Phase 1 gray box → Phase 2/3 black box → Phase 4 white box →
  Phase 5 defenses.
- For each attack: resolve §8 names → write `experiments/<id>.yaml` → reuse the
  interposer + a perturbation (add one only if none fits) → write
  `launch/<id>.launch.py` with remaps → run baseline + attack → check `summary.json`.
- First task: scaffold the package, build `interposer_node` + `bias_walk` + time
  trigger, then implement **R1 setpoint injection** and **R2 state spoof**.

## Conventions

- Python, ROS 2 `rclpy`. Node names prefixed `adv_` (e.g. `adv_interposer`).
- Keep perturbations pure and testable: `perturb(msg, t, params) -> msg`.
- Determinism: seed every stochastic perturbation; record the seed in `summary.json`.
- Don't silently widen scope — if an attack needs more access than the brief's matrix
  lists, stop and flag it rather than editing AirStack or adding capabilities.
