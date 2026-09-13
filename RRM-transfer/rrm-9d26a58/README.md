> **SCRUM-8 continuation (2026-09-13):** Read the [current allocation, contracts and verification status](docs/scrum-8/README.md). The material below records the original prototype; its hardware/model prescriptions and oracle-ceiling claims do not override the Phase 1 baseline. Existing mock passes are not SIL compliance.

# RRM-1 — Robotics Reasoning Model

A modular embodied-reasoning architecture. Perception feeds a persistent semantic
world model, a reasoner plans over symbols, a deterministic verifier gates every
action, and a VLA policy converts verbs into motion.

**The deliverable is a reproducible benchmark and reference architecture — not a robot
demo.** The research question is whether an explicit world model, predictive planner
and independent safety verifier measurably improve task success, recovery and safety
over end-to-end VLA control.

```
Isaac Sim ──► Perception ──► WORLD MODEL ──► Reasoner ──► Safety #1
                              (semantic          │
                               belief)     verb table
                                                 ▼
                              Safety #2 ◄──── GR00T ◄── embodiment adapter
                                  │
                                ROS 2 ──► robot ──► back to perception
```

## Run it now — no GPU, no models, no Isaac Sim

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python scripts/oracle_loop.py --suite
```

```
task  result   replans  actions   cycles   unsafe   recovery
T1    PASS           0        1        3        0       100%
T2    PASS           0        2        6        0       100%
T6    PASS           3        0        0        4       100%
T8    PASS           1        3       12        0       100%
T9    PASS           3        4       24        0         0%
5/5 passed
```

Other entry points:

```bash
.venv/bin/python scripts/oracle_loop.py --fail-grasp        # divergence + recovery
.venv/bin/python scripts/oracle_loop.py --human             # safety rejection
.venv/bin/python scripts/oracle_loop.py --suite --trace-dir traces/
.venv/bin/python simulation/isaac_backend.py                # relation-inference test
```

## Layout

| Path | |
|---|---|
| `rrm/` | the architecture — schema, verb table, planner, safety, loop |
| `rrm/schema.py` | every payload crossing a component boundary |
| `rrm/verbs.py` | verb table: preconditions and effects, authored not generated |
| `rrm/loop.py` | outer planning loop + inner execution loop |
| `scripts/oracle_loop.py` | CLI |
| `scripts/setup_instance.sh` | A10G provisioning |
| `simulation/isaac_backend.py` | what does *not* transfer from the mock |
| `docs/architecture.md` | design and decisions on record |
| `docs/benchmarks.md` | task suite, metrics, reproducibility rules |

## Deploying to the A10G

```bash
./scripts/setup_instance.sh check     # report, change nothing
./scripts/setup_instance.sh base      # drivers, CUDA, ROS 2 Jazzy, venv
./scripts/setup_instance.sh models    # weights → EBS
./scripts/setup_instance.sh verify    # smoke tests
```

Four things that will bite otherwise:

1. **Weights go on EBS, not instance store.** The `g5.2xlarge` NVMe is wiped on
   stop/start, and stopping when idle is the main cost control at $1.21/hr.
2. **The A10G is Ampere (sm_86) — no FP8.** Use AWQ/GPTQ INT4 for the reasoner.
3. **Cap `--gpu-memory-utilization` when serving the reasoner.** vLLM's 0.9 default
   claims ~21 GB and starves Isaac Sim.
4. **Use the Franka Panda.** GR00T's `LIBERO_PANDA` is pre-registered, so no
   fine-tuning — which matters because fine-tuning wants 40 GB+ and you have 24.

## Design commitments

- **The reasoner emits verbs over object IDs, never coordinates.** Grounding is
  deterministic, so a hallucinating model can produce a wrong *plan* but never a wrong
  *coordinate*, and every failure is attributable to a named component.
- **Preconditions and effects come from a static verb table, not the model.** A
  hallucinated effect would silently corrupt divergence detection, which is the one
  subsystem meant to catch errors.
- **`WorldState` is authoritative semantic *belief*, not ground truth.** Physics owns
  where things are; the controller owns the robot's configuration. Divergence detection
  is how belief and reality get reconciled.
- **Deterministic orchestration, no agent framework.** ROS 2 is already the
  orchestrator, safety must be a mandatory edge rather than a tool the model may elect
  to call, and planning latency is a published result.
- **`ScriptedOracle` is permanent.** It plans perfectly within the verb vocabulary but
  cannot parse language. When a learned reasoner trails it, the gap is reasoning; when
  it fails, the bug is elsewhere.

## Status

| | |
|---|---|
| Schema, verb table, planner, safety, divergence, replan, metrics, tracing | done |
| Benchmark suite | 5 of 10 tasks, passing on `Oracle` |
| Isaac Sim, GR00T, learned reasoner | not started — needs the A10G |

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).

Apache 2.0 rather than MIT for two reasons: it carries an explicit **patent
grant**, which matters in robotics where the patent landscape is dense, and it is
what the entire dependency stack already uses (ROS 2, LeRobot, PyTorch tooling,
NVIDIA's Isaac-GR00T and Cosmos code, Qwen3, SmolVLA) — so there is no license
friction for anyone reproducing this work or building on it commercially.

**Model weights are not covered.** GR00T and Cosmos weights carry the NVIDIA Open
Model License, and that follows any checkpoint you fine-tune from them. See
[NOTICE](NOTICE) before publishing a derivative model.
