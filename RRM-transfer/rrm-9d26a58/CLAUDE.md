> **SCRUM-8 continuation (2026-09-13):** Read the [current allocation, contracts and verification status](docs/scrum-8/README.md). The material below records the original prototype; its hardware/model prescriptions and oracle-ceiling claims do not override the Phase 1 baseline. Existing mock passes are not SIL compliance.

# RRM-1 — context for Claude Code

## What this is

A modular embodied-reasoning architecture for robotics. **The deliverable is a
reproducible benchmark and reference architecture, not a robot demo.** The research
question is whether an explicit world model, predictive planning and an independent
safety verifier measurably improve task success, recovery and safety over end-to-end
VLA control.

Read `docs/architecture.md` (design + decisions on record) and `docs/benchmarks.md`
(tasks, metrics, reproducibility rules) before proposing changes. Those are the source
of truth; this file exists so you do not re-derive them.

## Commands

```bash
python3 scripts/oracle_loop.py --suite                  # benchmark suite + metrics
python3 scripts/oracle_loop.py --fail-grasp             # divergence + recovery
python3 scripts/oracle_loop.py --human                  # safety rejection
python3 scripts/oracle_loop.py --suite --trace-dir traces/
python3 simulation/isaac_backend.py                     # relation-inference self-test
./scripts/setup_instance.sh preflight                   # run LOCALLY before paying
./scripts/setup_instance.sh {check|base|models|verify}  # on the instance
```

The whole loop runs with **no GPU, no Isaac Sim, no model weights**. That is
deliberate: it is the CI target, and it keeps loop debugging off paid instances. If you
find yourself debugging a `Predicate` or a replan on a running GPU instance, stop and
move it local.

## Layout

| Path | |
|---|---|
| `rrm/` | the architecture — importable package |
| `rrm/schema.py` | every payload crossing a component boundary; `WorldBackend`/`ActionPolicy` protocols |
| `rrm/verbs.py` | verb table — preconditions and effects, authored not generated |
| `rrm/reasoning.py` | backward-chaining `ScriptedOracle` |
| `rrm/safety.py` | Safety #1 symbolic, Safety #2 numeric |
| `rrm/loop.py` | outer planning loop + inner execution loop |
| `simulation/isaac_backend.py` | the work that does NOT transfer from the mock |
| `scripts/setup_instance.sh` | provisioning + preflight |

## Invariants — do not break these without an explicit decision

1. **The reasoner emits verbs over `ObjectID`s, never coordinates.** Grounding is
   deterministic, so a hallucinating model can produce a wrong *plan* but never a wrong
   *coordinate*, and failures stay attributable to a named component.
2. **Preconditions and effects come from `VERB_TABLE`, never from a model.** A
   hallucinated effect would silently corrupt divergence detection — the one subsystem
   meant to catch errors.
3. **`WorldState` is authoritative semantic *belief*, not ground truth.** Physics owns
   where things are; the controller owns joint reality. `confidence`, `observed_by` and
   `uncertainty` exist because it can be wrong. Divergence detection is the
   reconciliation mechanism.
4. **Both safety verifiers are mandatory edges in the state machine**, not tools a model
   may elect to call. Nothing reaches the trajectory controller without Safety #2.
5. **Subtasks terminate on effect satisfaction**, not timeout. Timeout is the failure
   path.
6. **The inner loop absorbs transient failure.** Only failures the policy cannot fix
   itself escalate to divergence and replanning, which keeps replan count meaningful as
   a metric.
7. **Deterministic orchestration — no LangChain/LangGraph.** ROS 2 is already the
   orchestrator, and planning latency is a published result.
8. **`ScriptedOracle` is permanent**, not scaffolding. It is the reference ceiling: it
   plans perfectly within the verb vocabulary but cannot parse language. When a learned
   reasoner trails it, the gap is reasoning; when it fails, the bug is elsewhere.

## Hardware — constraints that are easy to violate

Target: AWS `g5.2xlarge` / Brev A10G — **22.35 GiB usable VRAM**, 8 vCPU, 32 GiB RAM.

| | |
|---|---|
| Budget | Isaac Sim 6–10 + GR00T 7–8 + reasoner INT4 ~3 = 16–21 GiB of 22.35 |
| Reasoner precision | **INT4 required.** bf16 is ~8 GB and will not fit |
| FP8 | **unavailable** — A10G is Ampere sm_86; FP8 needs Ada/Hopper |
| Embodiment | **`LIBERO_PANDA`** — GR00T fine-tuning needs 40 GB+, so use a pre-registered one |
| Perception | **Isaac Sim ground truth through Phase 5** — no VRAM for detector + depth, and it isolates the reasoning variable |
| vLLM | cap `--gpu-memory-utilization 0.15`; the 0.9 default takes ~20 GiB |
| Weights | must land on storage surviving stop/start — `setup_instance.sh` probes for it |

`UNITREE_G1` is also pre-registered, so a humanoid needs no fine-tuning. Still start
with Panda: cross-embodiment transfer is a *result* (hold RRM fixed, swap
`EmbodimentAdapter`), and that needs a simpler embodiment to transfer from.

## Status

Built and passing: schema, verb table, planner, both safety layers, divergence, replan,
two-loop execution, JSONL tracing, benchmark harness (5 of 10 tasks, 5/5 on `Oracle`).

Not started, in dependency order:

1. `IsaacWorldBackend.observe()/apply()` — see the TODOs in `simulation/isaac_backend.py`
2. `GR00TPolicy` + `EmbodimentAdapter`
3. `LocalReasoner` (Qwen3-4B INT4 via vLLM, `guided_json`)

Geometric relation inference is written and self-tested — Isaac gives poses, not
`on(cup, table)`, so that computation is required and its quality caps world-state
accuracy.

## Working notes

- Cost discipline matters: the instance bills while running. Prefer `brev exec` over an
  interactive session for setup, and stop the instance when idle.
- Traces must capture full context from the first real run; retrofitting means
  discarding every episode recorded before the change.
- No composite benchmark score until the weights are validated — report the per-metric
  table.
- Safety is not tradeable: any hard violation fails the run regardless of task success.
