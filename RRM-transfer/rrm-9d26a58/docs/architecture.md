> **SCRUM-8 continuation (2026-09-13):** Read the [current allocation, contracts and verification status](scrum-8/README.md). The material below records the original prototype; its hardware/model prescriptions and oracle-ceiling claims do not override the Phase 1 baseline. Existing mock passes are not SIL compliance.

# RRM-1 Architecture — Schema and Agentic Reasoning

Status: draft spec. Reference implementation of §3–§4 against a mock world in
`scripts/oracle_loop.py`.

## 0. Decisions on record

| Decision | Choice | Rationale |
|---|---|---|
| Reasoner | `ScriptedOracle` now; small local model when missions stop being scripted. **No API.** | Everything runs on the A10G |
| Orchestration | Deterministic state machine — **no** LangChain/LangGraph | See §1 |
| Schema/validation | Pydantic (JSON Schema export) | Not a framework; feeds `response_format` directly |
| Transport | ROS 2 (Jazzy on Ubuntu 24.04) | Already the orchestration layer |
| Action policy | GR00T N1.7-3B, local on A10G, via LeRobot harness | ~6.93 GB checkpoint; swap to SmolVLA if VRAM-bound |
| Simulation | Isaac Sim 5.1, headless | A10G 24 GB |

Compute: AWS `g5.2xlarge` — A10G 24 GB VRAM, 8 vCPU, 32 GiB RAM. **Everything is local;
no external API is in the loop.** VRAM is therefore the binding constraint on every
model choice — see §8.

### 0.1 Why the reasoner is a separate model from GR00T

**GR00T does reason.** Its Action Cascade architecture runs a Cosmos-Reason2-2B backbone
that processes images and language into high-level action tokens — NVIDIA describes task
decomposition and multi-step reasoning as happening there — which a 32-layer diffusion
transformer then denoises into motor commands. Any claim that GR00T "cannot reason" is
simply wrong.

The distinction that matters is that **its reasoning is latent, not externalized.**
Output is continuous-value action vectors mapped to the robot's degrees of freedom: no
text, no chain-of-thought, no readable trace. Four consequences:

- **Not symbolic.** Latent action tokens cannot populate `WorldState`. Divergence
  checking compares propositions (`holding(cup)`); embeddings do not supply them.
- **Not auditable.** §6 requires an independent deterministic verifier. Reasoning that
  cannot be inspected cannot be verified — this is precisely why safety is a separate
  component rather than a property we trust the policy to have.
- **Short horizon.** It reasons to emit a 16-step action chunk (63.9 ms on an L40,
  bf16). That is real-time motor reasoning, not a minute-long conditional task graph.
- **Not persistent.** No object identity across time, no relations, no temporal state;
  each inference conditions on the current observation and proprioception.

So GR00T answers *how do I move to execute this instruction* — very well, and with
genuine reasoning inside. It does not answer, in any form RRM can read, verify, or
accumulate:

| Question | Answered by |
|---|---|
| What is the mission asking for? | Reasoner |
| What are the steps, in what order? | Reasoner |
| Does observation match expectation? | RRM code (deterministic) |
| Why did that fail, what next? | Reasoner |
| How do I move the joints? | **GR00T** |

That GR00T's backbone is itself a Cosmos model is an implementation detail of GR00T,
not a reasoning interface RRM can call.

**This is a hypothesis, not an axiom.** If GR00T's internal reasoning turns out to be
sufficient for the task suite, Baseline A (VLA → robot) will match or beat RRM-1, and
that is a legitimate and publishable result. The §21 comparison exists to settle this
empirically rather than by assertion. Build the separated architecture because it is the
only way to *measure* the question — not because the answer is assumed.

### 0.2 The reasoner probably does not need vision

Perception converts pixels into `WorldState` — objects, relations, poses — before the
reasoner is invoked. A reasoner that reads `WorldState` is reasoning over symbols, not
images, and a small **text-only** model does that more reliably and far more cheaply
than a VLM re-deriving the same facts from frames.

Default to a text reasoner over `WorldState`. Escalate to a VLM only for the specific
case where the reasoner must catch something the world model missed — and treat that as
a measured ablation, not an assumption.

## 1. Orchestration: deterministic, not an agent framework

LangChain/LangGraph is rejected for the control path. Reasons, in order of weight:

1. **ROS 2 is already the orchestrator.** Adding a second orchestration layer means two
   systems own control flow, scheduling, and failure semantics.
2. **§20 measures planning latency.** Framework overhead is confounding noise in a
   number that is a published result.
3. **The safety verifier must be deterministic and auditable.** A framework deciding
   when to invoke it defeats its purpose. Safety is a mandatory edge in the state
   machine, not a tool the model may elect to call.
4. **The RRM loop is a fixed state machine, not open-ended deliberation.** The model
   does not choose the next stage; the stages are known in advance (§4).
5. **Reproducibility over the project's lifetime.** Framework version churn against a
   multi-year benchmark is a liability.
6. **Generalizability comes from the schema, not the framework.** See §5.

What replaces it: Pydantic models for every payload, a plain enumerated state machine
for the loop, ROS 2 topics/services for transport, and structured logging for tracing.

## 2. Model boundary — what the reasoner may and may not emit

**The reasoner emits verbs over object references. It never emits coordinates, poses,
joint angles, or motor commands.**

This is the load-bearing safety property. Grounding an `ObjectID` to a pose is done by
perception plus the world model, both deterministic. A hallucinating reasoner can
therefore produce a *wrong plan* but never a *wrong coordinate*; the damage is bounded
by the safety verifier and the grounding layer, and every failure is attributable to a
specific component.

```
Reasoner        ──emits──>  TaskGraph of AbstractActions (symbolic)
                                        │
World Model      ──binds──>  ObjectID → pose, extent, confidence
                                        │
Safety Verifier  ──gates──>  deterministic, may only reject
                                        │
VLA / Policy     ──emits──>  trajectories (numeric)
```

### 2.1 Data flow — two paths out of the simulator

Perception and the reasoner never exchange messages; the world model sits between them
and owns all writes. The policy does not read `WorldState` at all — it takes raw pixels
and proprioception, because the visual detail a VLA needs for contact and grasping is
precisely what perception discards when it collapses a scene into `{id, class, pose}`.

```
                        Isaac Sim
                            │
         ┌──────────────────┴──────────────────┐
         │ RGB-D, poses, segmentation          │ RGB + joint state
         ▼                                     │   (raw, unprocessed)
    Perception  (ground-truth shim, Ph.3–5)    │
         │                                     │
         ▼                                     │
   ┌───────────────┐                           │
   │  WORLD MODEL  │  authoritative *semantic* │
   │   WorldState  │  belief — not ground truth│
   └───────┬───────┘                           │
           │ reads                             │
           ▼                                   │
     Reasoner (text)        ~1 Hz  ◄── OUTER LOOP
           │ {verb, target}                    │
           ▼                                   │
      Verb table ──> preconditions + effects   │
           │                                   │
           ▼                                   │
   ► SAFETY #1  symbolic: is this permitted?   │
           │                                   │
           ▼                                   │
   Embodiment adapter                          │
           │ "pick up the red cup"             │
           ▼                                   ▼
           └──────────────► GR00T ◄────────────┘
                              │  control rate  ◄── INNER LOOP
                              │  16-step chunk, 63.9 ms (L40)
                              ▼
                   ► SAFETY #2  numeric: joints, velocity, path
                              │
                              ▼
                            ROS 2 ──► robot ──► back to Isaac Sim
```

Why the world model must sit in the middle:

- **Persistence.** Perception is per-frame and stateless. The world model accumulates
  identity across time and retains objects that are currently occluded. A reasoner
  reading perception directly would forget the cup when the camera turns away.
- **Provenance.** `observed_by` only works if one component owns writes.
- **Divergence.** `expected_effects` are checked against `WorldState`, not against a
  perception message. The comparison needs a stable snapshot on both sides of an action.

### 2.1.1 What `WorldState` is authoritative *for*

`WorldState` is **the single authoritative semantic state of the environment and task**.
It is emphatically *not* the source of truth about physical reality. Three distinct
authorities coexist:

| Authority | Owns | Accessible via |
|---|---|---|
| Physics (sim or world) | where things actually are | sensors only, never directly |
| `WorldState` | semantic belief: identity, relations, history, provenance | the world model |
| Controller / proprioception | the robot's actual configuration and executed trajectory | ROS 2 feedback |

This distinction is load-bearing, not pedantic. The obvious challenge to the
architecture is *"how can `WorldState` be the source of truth if the robot is actually
somewhere else?"* — and the answer is that it never claimed to be. `WorldState` is a
**belief with confidence and provenance attached**, which is exactly why `WorldObject`
carries `confidence`, `observed_by`, and `last_observed`, and why `WorldState` carries
an aggregate `uncertainty`. A genuine source of truth would need none of those fields.

It follows that `RobotState` inside `WorldState` is a **cached view** of proprioception,
not the authority for it. The controller owns joint reality. This is why GR00T reads raw
proprioception directly rather than `WorldState.robot` — a policy must never act on a
stale belief about where its own joints are.

**Divergence detection is the reconciliation mechanism.** Belief and physics drift apart
during execution; comparing `expected_effects` against fresh observation is precisely
how the system detects that drift and corrects the belief. A system whose world model
could not be wrong would have no need to check.

That makes this a **closed semantic control loop** rather than a perception-to-policy
pipeline: symbolic intent flows down, physical consequence flows back up as observation,
and the belief is corrected each cycle.

**Why not feed the policy `WorldState` and the mission directly?** Three reasons, the
first of which is not negotiable:

1. **The modality is fixed.** GR00T's `modality_config` declares images, a language
   instruction, and proprioceptive state. There is no encoder for a symbolic scene
   graph. Adding one is retraining, which needs 40 GB+ VRAM (§8.0).
2. **That configuration is Baseline A.** `mission + camera → VLA → robot` is the
   control condition of §21. Building it and calling it RRM-1 removes the thing under
   test.
3. **Symbolic safety disappears.** Safety #1 works because a discrete action exists to
   inspect before motion. With only trajectories available, `GRASP(knife)` cannot be
   rejected as an *intent* — only its joint angles can be checked after the fact.

There is a real gap the question exposes, though: the adapter currently renders
`GRASP(obj_003)` to the string `"pick up the red cup"` and discards the grounding the
world model just computed. With two cups present, the string is ambiguous where the
`ObjectID` was not. The fix is to pass the resolved target pose through GR00T's numeric
state channel alongside the instruction — enriching the adapter's output within the
accepted modality, not bolting a new one on.

**Two safety checkpoints, not one.** Symbolic safety runs before the policy and asks
whether the action is permitted. Numeric safety runs after and asks whether the emitted
joint targets are admissible. The `joint_limit`, `velocity_limit` and `force_limit`
checks of §3.5 are trajectory-level and cannot run until the policy has produced numbers.
Neither check subsumes the other.

### 2.2 Subtask termination: effect satisfaction

GR00T emits no completion signal, so something must decide when a subtask is done. The
world model polls the action's instantiated `expected_effects` each inner-loop cycle and
terminates when they hold. Timeout is the failure path, not the normal one.

```
dispatch(action):
    each cycle:
        if all expected_effects hold          -> COMPLETE
        traj = policy.step(action, observation)
        if numeric safety rejects traj        -> UNSAFE
        apply(traj)
    cycle budget spent                        -> TIMEOUT
```

This falls out of the verb table for free: the effects were already declared, so no new
machinery is needed to know what "done" means.

It also creates a useful separation of concerns. **The inner loop absorbs transient
failure.** A policy that fumbles a grasp and recovers within its cycle budget has
genuinely succeeded, and the outer loop should never hear about it. Only failures the
policy cannot fix itself escalate to divergence and replanning — which keeps replan
counts meaningful as a metric rather than counting motor noise.

## 3. Core schema

Design constraints:

- Flat over nested. Minimal required fields.
- Object references are opaque string IDs, never inline object copies.
- Every model-authored structure must be validatable, repairable, and replayable.

### 3.1 Identity and world state

```
ObjectID    := string, stable across time, assigned by the tracker, never by the model

WorldObject
  id            : ObjectID
  class         : string              # "cup", "cabinet", "person"
  pose          : Pose | null         # null = known to exist, position unknown
  extent        : BBox3 | null
  properties    : map<string, value>  # color, graspable, open, occupied
  confidence    : float [0,1]
  last_observed : timestamp
  observed_by   : enum {sim_ground_truth, detector, inferred, asserted_by_reasoner}

Relation
  subject   : ObjectID
  predicate : enum {on, in, near, held_by, occludes, supports, blocked_by}
  object    : ObjectID
  confidence: float [0,1]

RobotState
  embodiment_id : string
  base_pose     : Pose
  joint_state   : map<string, float>
  gripper       : enum {open, closed, holding}
  holding       : ObjectID | null

WorldState
  t          : timestamp
  objects    : list<WorldObject>
  relations  : list<Relation>
  robot      : RobotState
  uncertainty: float [0,1]            # aggregate; drives replan thresholds
```

`observed_by` is not decoration. It is how §15's baseline-vs-RRM ablation stays honest:
state the reasoner asserted must never be silently promoted to state the sensors saw.

Temporal state (§3 of the brief) is a **append-only delta log** plus a current snapshot.
The log is what makes prediction accuracy measurable after the fact.

### 3.2 Actions

Closed verb vocabulary. Adding a verb is a deliberate, versioned act — the vocabulary
is the contract that makes cross-embodiment transfer possible (§5).

**Preconditions and effects are owned by a static verb table, not by the model.** Each
verb's semantics are hand-written once and are identical on every invocation. The model
selects a verb and a target; the system fills in the rest.

```
Verb := LOCATE | NAVIGATE_TO | GRASP | RELEASE | PLACE
      | OPEN | CLOSE | INSPECT | WAIT | ABORT

VerbSpec                          # static, authored by us, never model-generated
  verb             : Verb
  arity            : int
  preconditions    : list<PredicateTemplate>
  expected_effects : list<PredicateTemplate>

# e.g. GRASP(x):
#   preconditions    = [graspable(x), reachable(x), gripper_empty()]
#   expected_effects = [holding(x), ¬on(x, *)]

AbstractAction                    # instantiated by the planner, not parsed from the model
  id        : string
  verb      : Verb
  targets   : list<ObjectID>
  params    : map<string, value>  # e.g. {relation: "on"} for PLACE
  rationale : string              # free text from the model, logged, never executed
```

This matters for robustness. Asking a natural-language model to emit correct
preconditions and effects invites a class of failure where the plan looks valid but its
declared consequences are wrong — and since divergence detection compares against those
declarations, a hallucinated effect silently corrupts recovery. Deriving them from the
verb table makes that failure mode structurally impossible, and shrinks what the model
must produce to roughly a verb plus an object reference.

**Instantiated `expected_effects` remain the mechanism that makes recovery possible.**
§17 requires detecting `expected state ≠ observed state`; the effects give you the
left-hand side. The change here is only *who authors them* — the table, deterministically,
rather than the model, per-call.

### 3.3 The model-facing payload (kept deliberately thin)

The canonical structures above are internal, deterministic, and as rich as they need to
be. **What the model is asked to produce is much smaller.** These are two different
schemas joined by a parser, and conflating them is the main way this design fails.

What we ask the reasoner for looks roughly like:

```json
{
  "task": "pick_red_cup",
  "observations": ["red cup on table", "robot arm is idle"],
  "next_action": {"verb": "GRASP", "target": "obj_003"},
  "confidence": 0.91,
  "rationale": "cup is visible, reachable, gripper is empty"
}
```

Flat. Few required fields. No predicates, no poses, no graph topology, no effects.
Everything else — preconditions, expected effects, graph edges, grounding — is derived
deterministically from the verb table and the world model.

```
Reasoner response (loose)
      ↓
  Parser            # tolerant: strips prose, extracts JSON, coerces types
      ↓
  Validator         # Pydantic; rejects unknown verbs and unbound ObjectIDs
      ↓ invalid
  Repair / retry    # bounded; validation error fed back once
      ↓ still invalid
  Fail closed       # never reaches world model or safety verifier
      ↓ valid
  Canonical RRM state
```

The parser must tolerate a model that wraps JSON in prose or reasons aloud before
answering — normal behavior for a reasoning model, and more reliably handled in the
parser than fought with prompt engineering. Constrained decoding (§7) reduces this but
does not remove the need for the parser.

### 3.4 Task graph

```
TaskGraph
  mission_id : string
  mission_text: string
  nodes      : list<AbstractAction>
  edges      : list<Edge>        # (from, to, condition: on_success|on_failure|always)
  version    : int               # increments on every replan; never mutated in place
```

Replanning produces a new `TaskGraph` with an incremented version. The old one is
retained. The version history *is* the recovery record for §20's recovery-rate metric.

### 3.5 Safety and divergence

```
SafetyVerdict
  action_id  : string
  verdict    : enum {PASS, FAIL}
  violations : list<Violation>   # empty iff PASS
  checked    : list<string>      # which checks ran — audit trail

Violation
  check    : enum {collision, human_proximity, joint_limit, velocity_limit,
                   force_limit, stability, workspace, mission_constraint}
  severity : enum {hard, soft}
  detail   : string

Divergence
  action_id      : string
  expected       : list<Predicate>
  observed       : list<Predicate>
  unmet          : list<Predicate>      # expected ∧ ¬observed
  surprise       : list<Predicate>      # observed ∧ ¬expected
  magnitude      : float [0,1]
```

`surprise` matters as much as `unmet`: a grasp that succeeded while also knocking over
a second object is a partial success the world model must record.

## 4. The agentic loop

Explicit states. Every transition is logged with a timestamp and the payload that
caused it.

```
        ┌──────────────────────────────────────────────┐
        ▼                                              │
   PERCEIVE ──> UPDATE_WORLD ──> [replan needed?] ──no──┤
                                       │yes             │
                                       ▼                │
                                    REASON              │
                                       │                │
                                       ▼                │
                              VALIDATE_SCHEMA           │
                                  │        │fail        │
                                  │pass    └─> REPAIR ──┐(bounded retry)
                                  ▼                     │
                                GROUND                  │
                                  │                     │
                                  ▼                     │
                            VERIFY_SAFETY               │
                              │        │FAIL            │
                              │PASS    └──> REPLAN ─────┤
                              ▼                         │
                            EXECUTE                     │
                              │                         │
                              ▼                         │
                       CHECK_DIVERGENCE ────────────────┘
                              │
                        [mission done?] ──> REPORT
```

Replan triggers, all deterministic thresholds, none model-decided:

- `SafetyVerdict.verdict == FAIL`
- `Divergence.magnitude > θ_div`
- `WorldState.uncertainty > θ_unc`
- action precondition unsatisfied at dispatch time
- executor timeout
- human enters workspace (immediate STOP, not replan)

Replan budget is bounded. On exhaustion: `ABORT`, report, do not retry indefinitely.
An unbounded replan loop is the most likely way this system burns wall-clock during
an overnight benchmark run.

## 5. Generalizability — where it actually comes from

The stated goal is a reusable intelligence layer across embodiments (§25). That
property lives in exactly one place: **the boundary between `AbstractAction` and the
embodiment.**

```
AbstractAction (embodiment-agnostic, stable)
        │
        ▼
EmbodimentAdapter  ← the only component that changes per robot
        │
        ▼
Concrete trajectory / VLA prompt / ROS 2 action goal
```

An adapter declares:

- which `Verb`s it supports (a mobile base without an arm rejects `GRASP`)
- workspace bounds and kinematic limits
- how each verb maps to a policy call or motion-planner goal
- observation format expected by its VLA

Nothing above the adapter mentions arms, wheels, joints, or grippers. If a schema
change requires touching the reasoner to support a new robot, the abstraction has
leaked and the §25 claim is false. That is a testable invariant, and it should be
enforced by a test.

## 6. Backends

The reasoner is a swappable interface, not a commitment. This is required by the §21
experiment design — "which reasoner" must be a row in a results table, not an
architectural fact.

```
ReasonerBackend
  .plan(mission_text, WorldState, capabilities) -> TaskGraph
  .replan(mission_text, WorldState, TaskGraph, Divergence) -> TaskGraph

  ├── ScriptedOracle        # deterministic, no model, 0 VRAM — build this FIRST
  └── LocalReasoner         # small text model over WorldState, served by vLLM
```

**Build `ScriptedOracle` first.** Hand-written correct task graphs for the benchmark
tasks let the world model, safety verifier, grounding, and recovery loop be developed
and tested with zero model variance. Every failure in Phases 4–6 is then unambiguously
a bug in our code rather than a bad generation. It is also the zero-cost path for CI,
and it remains the control condition in the §21 comparison permanently.

`LocalReasoner` is deliberately unspecified as to model. It reads `WorldState` and emits
the payload of §3.3; which model fills that slot is an experimental variable (§0.2), not
an architectural commitment.

| Candidate | VRAM (INT4) | Why |
|---|---|---|
| Qwen3-4B-Instruct-2507 | ~3 GB | Default. Apache 2.0, strong structured output |
| SmolLM3-3B | ~2 GB | Alternate. Fully open weights *and* data |
| **Cosmos-Reason2-2B** | ~2 GB | **Ablation arm.** Open weights, runs locally |

Cosmos-Reason2-2B is not excluded — the decision against an API removed the hosted 8B,
not the model family. It is not the default because the reasoner reads symbolic
`WorldState` rather than pixels, so a VLM's vision tower is unused weight and its long
chain-of-thought costs latency on every plan. But it is the one candidate trained
specifically for embodied physical reasoning, which makes "does a physically-trained VLM
plan better than a general text model of similar size?" a cheap and worthwhile
experiment once this interface exists.

## 7. Structured output

Serving the reasoner locally with vLLM makes constrained decoding **available**:
`guided_json`, `guided_choice` and `guided_grammar` are supported, backed by xgrammar or
outlines. This is a real advantage of running locally rather than against a hosted
endpoint, where the Qwen-family VLM containers document those parameters as unsupported.

Export the JSON Schema from the Pydantic model of §3.3 and pass it as `guided_json`.
Constrained decoding also tends to be *faster* than free generation under load, since
tokens outside the grammar need not be sampled.

The parser stays regardless. Constrained decoding guarantees the output *parses*; it
does not guarantee the content is sane — a schema-valid plan can still name a
nonexistent `ObjectID` or an unreachable target. So:

1. Keep the emitted schema flat and small (§3.3). Prefer several narrow calls over one
   wide call that must be perfect.
2. Validate semantics after parsing: every `ObjectID` must bind to a real object, every
   verb must be in `VERB_TABLE`, arity must match.
3. On failure: one bounded repair attempt with the validation error fed back, then fail
   closed. Never let an unvalidated structure reach the safety verifier or world model.
4. **Log the full call context** — `(WorldState snapshot, prompt hash, raw response,
   parse outcome)`. Replaying raw generations makes prompt and schema changes measurable
   offline without re-running simulation, and the parse-outcome field yields a
   schema-conformance rate that is a reportable result about the model.

Prompts are versioned artifacts. The verb vocabulary section must be **generated from
`VERB_TABLE`** rather than written by hand, or the two drift and the model will emit
verbs that no longer exist. Hash the prompt and log the hash with every trace; a silent
prompt edit invalidates every prior benchmark number.

## 8. Resource budget

Two target configurations. The budget below is sized for the tighter one; the
larger removes VRAM as a constraint entirely.

| | A10G (`g5.2xlarge`) | L40S (`g6e.2xlarge`) |
|---|---|---|
| VRAM | 24 GB | 44.7 GiB usable of 48 |
| System RAM | 32 GiB — Isaac Sim *minimum* | 64 GiB — its "good" tier |
| vCPU | 8 | 8 |
| Compute capability | sm_86 Ampere — **no FP8** | sm_89 Ada — FP8 available |
| Reasoner precision | INT4 required | bf16 or FP8 |
| GR00T fine-tuning | blocked (needs 40 GB+) | possible |

**vCPU count matters more than it appears.** Isaac Sim physics stepping, USD traversal
and ROS 2 DDS serialization are all CPU work, competing with the RRM loop and a vLLM
server. Four cores is Isaac Sim's stated minimum and is not enough once GR00T and the
reasoner run concurrently — prefer 8+ even at the cost of a smaller GPU.

With no API in the loop, every model is resident simultaneously.

Selected target: `g5.2xlarge` — A10G, **22.35 GiB usable** (24 GB nominal, less
driver and ECC overhead), 8 vCPU, 32 GiB RAM.

| Component | VRAM | Notes |
|---|---|---|
| Isaac Sim (headless, moderate scene) | 6–10 GB | |
| GR00T N1.7-3B | ~7–8 GB | 6.93 GB checkpoint + activations |
| Reasoner, 4B **INT4 (AWQ/GPTQ)** | ~3 GB | 0 GB while on `ScriptedOracle` |
| Perception (detector + depth) | 2–4 GB | **Does not fit** — see below |
| **Total** | **16–21 GiB of 22.35** | |

At the top of that range the margin is ~1.3 GiB. Two triggers to watch, with the
response for each already decided:

| Symptom | Response |
|---|---|
| Isaac Sim scene pushes past ~10 GB | swap GR00T → SmolVLA (~1–2 GB) |
| OOM during model load | check host RAM, not VRAM — 32 GiB is the minimum and checkpoints stage through it |

**The reasoner must be quantized.** A 4B model at bf16 is ~8 GB, which puts the total at
21–26 GB and breaks the budget before perception exists. INT4 brings it to ~3 GB.

**Do not use FP8 checkpoints on the A10G.** GA102 is compute capability 8.6; FP8
requires Ada (8.9) or Hopper (9.0). On Ampere the options are AWQ or GPTQ INT4. FP8
becomes available only if the instance is upgraded to `g6e` (L40S, Ada).

Model downloads (`HF_HOME` on the EBS volume, never instance store):

| Repo | Size | Role |
|---|---|---|
| `nvidia/GR00T-N1.7-3B` | 6.93 GB | Action policy |
| `Qwen/Qwen3-4B-Instruct-2507` (INT4 build) | ~3 GB | Reasoner, Apache 2.0 |
| `lerobot/smolvla_base` | ~1 GB | Fallback policy |

Take **Instruct**, not Thinking/reasoning variants: §20 publishes planning latency and
thinking traces spend hundreds of tokens per call. `SmolLM3-3B` is the alternate if the
Qwen quant is unavailable or misbehaves — fully open weights and training data, which
helps reproducibility claims.

Two consequences, neither optional:

1. **Take perception from Isaac Sim ground truth through Phase 5.** There is no VRAM
   for detector and depth models alongside the rest. This was previously a
   methodological preference (cleaner ablation); it is now also a hard requirement.
2. **Fall back to SmolVLA (~1–2 GB) if the budget breaks.** The LeRobot harness makes
   this a config change. GR00T is the better policy but it is also the largest
   discretionary consumer here.

The reasoner and GR00T do not peak simultaneously — the reasoner runs at plan time,
GR00T at control rate — but both stay resident, since load/unload cycling costs far more
than the memory it reclaims. Budget for concurrent residency, not concurrent compute.

### 8.0 Embodiment choice is a VRAM decision

**Use a Franka Panda in Isaac Sim and take GR00T's pre-registered `LIBERO_PANDA`
embodiment.** This is not an aesthetic preference; it determines whether the A10G is
sufficient.

GR00T ships pre-registered embodiments (`LIBERO_PANDA`, `OXE_WIDOWX`, `UNITREE_G1`,
DROID with zero-shot support). Any other robot uses the `NEW_EMBODIMENT` tag with a
custom modality config and **requires fine-tuning** — SO-100 is documented as needing it
rather than working zero-shot.

Note that `UNITREE_G1` is a full humanoid and is pre-registered, so a humanoid is
reachable on this hardware. Start with Panda regardless, for experimental reasons
rather than hardware ones:

- **Attribution.** A humanoid adds balance, whole-body control and locomotion failure
  modes. RRM-1 exists to attribute failures to a named component; an embodiment that
  can fall over blurs exactly the signal being measured.
- **The task suite is tabletop.** T1–T10 are manipulation, and the verb table contains
  no locomotion verbs. Extra DOF contribute nothing and cost simulation budget.
- **Cross-embodiment is a result, not a starting point.** §25's claim is demonstrated by
  holding RRM fixed and swapping only `EmbodimentAdapter`. Because `UNITREE_G1` needs no
  fine-tuning, that experiment costs one adapter — but only if there is a simpler
  embodiment to transfer *from*. Starting on the humanoid yields one data point and no
  transfer story.

And fine-tuning does not fit on this GPU:

| Fine-tune mode | VRAM |
|---|---|
| NVIDIA stated minimum | 40 GB+ |
| Default (projector + diffusion head, backbone frozen) | peak ~35 GB |
| `--tune-llm` / `--tune-visual` | 80 GB+ recommended |

24 GB is short of all of these. LoRA and gradient accumulation are documented
workarounds and success is reported on 32 GB cards, but the supported path needs 40 GB+.

**Consequence:** choosing an unregistered arm converts the `g6e` upgrade from a Phase 6
convenience into a Phase 2 blocker. Choosing `LIBERO_PANDA` avoids fine-tuning entirely
through Phase 7.

Dataset trap for later: GR00T reads **LeRobot v2.1** format, while the LeRobot framework
now records **v3.0**. Any data collection needs a conversion step.

### 8.1 Upgrade path

The budget above fits Phases 3–5 because perception is Isaac Sim ground truth. **The
predictable break point is Phase 6**, when the first real detector and depth model add
2–4 GB and push the total to 19–27 GB. Do not upgrade before then.

| Instance | GPU | vCPU / RAM | $/hr |
|---|---|---|---|
| `g5.2xlarge` (current) | A10G 24 GB | 8 / 32 GiB | $1.21 |
| **`g6e.2xlarge`** (recommended) | L40S 48 GB | 8 / 64 GiB | $2.24 |
| `g6e.4xlarge` | L40S 48 GB | 16 / 128 GiB | $3.00 |

`g6e.2xlarge` doubles VRAM and gives roughly 2× Isaac Sim performance, with no
architectural change. Take `g6e.4xlarge` only if profiling shows a CPU bound — the
2xlarge stays at 8 vCPUs, so it will not move a CPU bottleneck.

**Do not upgrade to p4d/p5 (A100/H100).** Those GPUs have no RT cores and Isaac Sim
does not support them; G6e is the practical ceiling for this workload. Skip `g6`
(L4 24 GB) as well — cheaper than g5, but a 72 W part and weaker for rendering than the
A10G.

Profile GPU utilization against CPU saturation before spending. The failure mode is
paying 85% more and not moving the bottleneck.

**Storage:** the instance-store NVMe on `g5.2xlarge` is ephemeral and is wiped on
stop/start. Model weights and the Hugging Face cache belong on the EBS root volume, or
they must be re-pulled every session — which matters because stopping the instance when
idle is the main cost control.

For Phases 3–5, take segmentation, depth, and object poses from Isaac Sim ground truth
and run **no perception models at all**. This isolates the reasoning research from
perception error, frees VRAM, and makes the Phase 6 introduction of real perception a
clean, measurable ablation rather than a confound present from day one.

## 9. The Isaac Sim seam

Two Python protocols define everything a real backend must provide. Nothing above them
changes when Isaac Sim arrives.

```python
class WorldBackend(Protocol):
    def observe(self) -> WorldState: ...
    def apply(self, action: AbstractAction, traj: Trajectory) -> None: ...
    def begin_dispatch(self, action: AbstractAction) -> None: ...

class ActionPolicy(Protocol):
    def step(self, action: AbstractAction, ws: WorldState) -> Trajectory: ...
    def reset(self, action_id: str) -> None: ...
```

`observe()` returns semantic belief, so the backend owns the perception step. In
Phases 3–5 that is a ground-truth shim reading poses directly from the simulator (§8).

### 9.1 Node granularity — start with one node, not six

The brief sketches one ROS 2 node per component. **Do not start there.** Six nodes means
six serialization boundaries, six lifecycles, and distributed debugging, in exchange for
distribution you do not yet need. Every node boundary also adds latency to a number
(§20 planning latency) that is a published result.

Run the whole RRM loop — world model, reasoner, planner, both safety verifiers — as a
**single node**, with ROS 2 at the two boundaries that genuinely cross a process:

```
/isaac_sim  ──sensors──►  ┌──────────────────┐
                          │   /rrm_node      │  world model, reasoner,
/isaac_sim  ◄──commands──│   (single proc)  │  planner, safety #1 + #2
                          └──────────────────┘
                                   │
                          /groot_node (separate: owns the GPU policy)
```

GR00T gets its own node because it owns a GPU context and has a different lifecycle
from the reasoning loop — that split is real, not organizational.

This preserves what actually matters for the physical-robot migration: the *boundaries*
are ROS 2-shaped, so `/isaac_sim` swaps for a real driver without touching RRM. Split
further only when a measurement demands it.

### 9.2 Topic contract

| Direction | Topic | Type | Notes |
|---|---|---|---|
| sim → rrm | `/camera/rgb`, `/camera/depth` | `sensor_msgs/Image` | raw; also consumed by `/groot_node` |
| sim → rrm | `/joint_states` | `sensor_msgs/JointState` | proprioception, authoritative |
| sim → rrm | `/ground_truth/objects` | custom | Ph.3–5 perception shim |
| rrm → groot | `/rrm/instruction` | custom | rendered verb + resolved target pose (§2.1) |
| groot → rrm | `/rrm/trajectory` | `trajectory_msgs/JointTrajectory` | gated by Safety #2 |
| rrm → sim | `/joint_trajectory_controller/command` | `trajectory_msgs/JointTrajectory` | post-verification only |

**Nothing reaches `/joint_trajectory_controller/command` without passing Safety #2.**
That is the one invariant the node layout must make structurally impossible to violate —
which is a second reason to keep the verifier in-process with the publisher rather than
in a node that could be bypassed.

### 9.3 Traces

`scripts/oracle_loop.py --suite --trace-dir traces/` writes one JSONL per episode:
`run_start`, `plan`, `safety1`, `safety2`, `apply`, `dispatch`, `divergence`, `replan`,
`episode_end`. Every metric in benchmarks.md §3 is recomputable from these records
without re-running simulation. `RUN_META` carries backend, policy, reasoner, seed,
prompt version, and threshold values — fill `prompt_version` and `seed` in as soon as a
real reasoner and a non-deterministic backend exist.

## 10. Open items

- Verify the three-model VRAM budget of §8 empirically on the A10G. This is the
  highest-risk open item: if Isaac Sim plus GR00T plus a reasoner does not fit, the
  fallbacks are SmolVLA or `g6e.2xlarge`. **Week 1.**
- Cap vLLM `gpu_memory_utilization` explicitly when serving the reasoner. The default
  (0.9) will claim the whole GPU and starve Isaac Sim.
- Smoke-test one `guided_json` generation before building on it. Qwen3 Instruct-2507
  has reported failures to terminate under vLLM structured output, running to the
  context limit — that is exactly this stack. Fallback is `SmolLM3-3B`. **Week 1.**
- Put model weights and the HF cache on the EBS root volume, not the ephemeral
  instance-store NVMe, which is wiped on stop/start.
- Make safety checks verb-scoped. `LOCATE` is currently rejected for human proximity
  despite moving nothing (observed in `scripts/oracle_loop.py --human`).
- Fix `CYCLE_BUDGET` empirically once GR00T is in the loop. Too low turns slow-but-
  working grasps into spurious replans; too high delays genuine failure detection.
- Give the oracle a safety-aware replan. On a `SAFETY #1` rejection it currently
  returns the same plan and retries into the same violation until the budget aborts —
  correct fail-closed behaviour, but it means the `--human` scenario exercises the
  budget rather than any recovery strategy.
- Choose the `LocalReasoner` model. Text-only over `WorldState` is the default (§0.2);
  a VLM variant is an ablation, not the baseline.
- Fix θ_div, θ_unc, and the replan budget empirically; they are currently unspecified.
- Confirm Isaac Sim 5.1 performance on A10G at target scene complexity. Fallback is
  `g6e.2xlarge` (L40S 48 GB) with no architectural change.
