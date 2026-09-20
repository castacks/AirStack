> **Current continuation (2026-09-20):** Read [HANDOFF.md](HANDOFF.md) first. The
> active OSMO path is a two-GPU workflow: one Isaac/AirStack workspace and one private
> warm Cosmos worker. The [localhost command console](docs/scrum-8/command-console.md)
> captures checksum-bound live observations and uses that worker only for shadow-only
> proposals. The console recovers the worker's private URL from the OSMO workspace init
> environment when a Remote-SSH process does not inherit it. No worker port is public,
> and no proposal can dispatch a drone. PSC material below is historical/offline
> reference only; it is not the live control-loop path.

The repository now contains the full injected observe/propose/ground/check/admit/
dispatch/verify/replan composition and a narrow read-only AirStack corridor provider.
The remaining route to validated model-to-drone operation is an explicitly supervised
end-to-end simulator trial from a compatible already-airborne state, followed by
takeoff/route-planning integration and broader scenario evidence. The GUI remains
proposal-only. Detailed safety conditions and current blockers are in `HANDOFF.md`.

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
bash scripts/test_rrm.sh
```

This is the standard RRM test entry point. It creates or reuses the gitignored
`.venv/`, installs `requirements.txt` only when Pydantic v2 is unavailable, and runs
the complete unit suite. On a minimal host without Python `venv` support, it instead
uses the gitignored `.rrm-deps/` directory through `PYTHONPATH`; neither path modifies
system Python. Do not run the suite with bare host `python3`: a host Python without
Pydantic will fail before test collection. The script does not start or alter Docker,
OSMO, Isaac, GPUs, or robot state.

To run just the original CPU-only Oracle demonstration after bootstrap, use the
interpreter selected by the bootstrap:

```bash
# Normal venv-capable host:
.venv/bin/python scripts/oracle_loop.py --suite

# Minimal host where the bootstrap reported .rrm-deps fallback:
PYTHONPATH=.rrm-deps python3 scripts/oracle_loop.py --suite
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
# Use .venv/bin/python on a venv-capable host, or prefix these with
# PYTHONPATH=.rrm-deps python3 on the minimal-host fallback.
.venv/bin/python scripts/oracle_loop.py --fail-grasp        # divergence + recovery
.venv/bin/python scripts/oracle_loop.py --human             # safety rejection
.venv/bin/python scripts/oracle_loop.py --suite --trace-dir traces/
.venv/bin/python simulation/isaac_backend.py                # relation-inference test
```

## RRM Command Console & two-GPU Cosmos workflow (OSMO)

The RRM GUI (Command Console) is used for live visual intake and proposal review.
The active workflow pairs Isaac with a private warm Cosmos worker; it is not a batch
submission path. The GUI presents one task-level proposal at a time and stays
shadow-only: review and outcome records never send a drone command.

**Prerequisite:** Isaac Sim and AirStack must be running so the console can pull live
camera images and robot odometry.

**Start the live console:**
```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_command_console.sh
```

This mode uses the checked-in Office C01/C02/C03 template and scene manifest. It can
capture/save live requests and call the configured private Cosmos worker, but it
loads no historical proposal and cannot dispatch a flight command.

### Start a new two-GPU Office workflow

Use the two-GPU workflow for a fresh Isaac workspace plus warm Cosmos worker. The
catalog shortname is `office`, but direct environment configuration must use the
resolved Pegasus key `Office` (capital `O`). **Run this from your local machine or
an authenticated OSMO control terminal, not from the Remote-SSH shell inside an
already-running OSMO workspace.** That workspace intentionally does not carry your
personal OSMO client login or `~/.ssh/id_ed25519.pub`.

```bash
cd /root/AirStack
osmo workflow submit osmo/workflows/airstack-live-replan.yaml \
  --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)" \
  --set-env "ISAAC_SIM_SCENE=Office" \
  --set-env "ISAAC_SIM_STAGE_SCALE=1.0"
```

`airstack-dev.yaml` remains the one-GPU developer workflow; do not use it for the
live Cosmos replan path. Do not run `airstack.sh up --sim isaac --scene office` merely
to change the scene of an already-running workspace: it can recreate simulator
containers. For a local fresh launch, that command remains the supported shortname
form and resolves `office` through `simulation/scenes.yaml`.

After submission, attach with your local OSMO helper/port-forward workflow. Inside the
remote workspace, use the already-running AirStack stack and start only the console:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_command_console.sh
```

Historical PSC bundles can still be imported for reference review with
`scripts/rrm_office_fetch_import.sh`; they are not exposed in the live GUI and do not
participate in the warm-worker cycle.

### Live action → replan workflow (foundation)

`rrm/live_replan.py` is the provider-neutral, shadow-only coordinator for the future
continuous workflow. It records a fresh camera image plus separately verified live
scene state before each provider request; binds the provider result to that exact
observation; exposes only the first action of a multi-action plan for review; and then
requires a reviewed, independently verified outcome before it will accept another
observation/replan. It has no ROS or dispatch dependency; the private warm worker is
one proposal provider.

`rrm/authorized_live_mission.py` is the separate, explicitly invoked composition
boundary for a simulator mission. It accepts injected per-frame entity-verifier
C02 context, C05 provider, deterministic embodiment compiler, dynamic C03 feasibility
evaluator, single-use admission, and independently verified public-action outcome
adapter. It enforces the bounded task/revision/verb/target/action authorization and
halts on any failed grounding, feasibility, admission, compilation, or effect
verification. Learned feasibility evidence is advisory; `grounding`, `body_limits`,
`physics`, `controller`, `resources`, and `stop_channel` must all have authoritative
PASS evidence for the exact observation and command. The core has no worker, console,
OSMO, Docker, or ROS dependency; the explicitly started runner injects those
boundaries. A worker starting successfully therefore cannot acquire a flight path.

### Explicit simulator mission runner

The private worker now also exposes `/v1/verify-entities`, which returns only
checksum-bound, catalog-limited visual C02 evidence with `execution_dispatch: false`.
It must be rebuilt and deployed before an existing worker instance can serve that
endpoint; this repository change does not restart a worker or workflow.

The mission runner is separate from both the worker and the console. Create a bounded
authorization file that exactly matches the immutable context, for example:

```json
{
  "task_id": "office-nav-001",
  "task_revision": "office-task-v1",
  "allowed_verbs": ["NAVIGATE_TO"],
  "allowed_targets": ["blue_marker"],
  "max_actions": 2
}
```

First use the default shadow path; it captures, visually verifies, and proposes one
action, but never creates an ActionClient:

If this checkout does not already have its isolated Python dependencies, run
`bash scripts/test_rrm.sh` once. The standalone mission runner and bundled feasibility
provider resolve the repository and `.rrm-deps` themselves; they do not rely on a
Remote-SSH shell inheriting `PYTHONPATH`.

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
python3 scripts/rrm_authorized_live_mission.py \
  --context examples/office_visual_eval/navigation_context.json \
  --scene-manifest examples/office_visual_eval/scene_manifest.json \
  --entity-catalog examples/office_visual_eval/entity_catalog.json \
  --authorization /secure/operator/office-mission.json \
  --worker-url "$RRM_COSMOS_WORKER_URL" \
  --run-dir /root/AirStack/.rrm-artifacts/live-missions/<new-run-id>
```

The public AirStack task-action adapter is unavailable unless the operator adds both
`--execute --simulator-only` and configures an executable
`--feasibility-provider`. The provider receives the exact C03 query on stdin, may
query embodiment-specific simulator physics and planning, and must return a typed,
short-lived result on stdout. The bundled Office profile is
`scripts/airstack_drone_feasibility_provider.py`. It combines fresh read-only vehicle,
controller, planner and action-graph state with a map-frame Ouster point-cloud corridor
check; it stores checksum-bound inline evidence and fails closed on stale channels,
unknown coverage, obstacles, grounded/no-control state, or a missing stop path. It is
limited to an already-airborne, nearly stationary vehicle and one straight `NAVIGATE`
waypoint; it is not a global route planner or a takeoff sequencer. These execution
flags are deliberately not shown as a routine startup command: use them only after
the worker image deployment, fresh simulator readiness/reconciliation, independent
observer coverage, and a supervised review.

The configured profile is intentionally exact: embodiment `aerial-eval`, capability
revision `office-airframe-v1`, and limits reference `office-bounded-nav-v1`. The
corridor start must agree with the checksum-bound camera observation within 0.25 m,
its target must exactly match the compiled waypoint, and its odometry must be
`map -> base_link`. Any changed profile, stale channel, detached corridor, or
insufficient point/range coverage blocks admission. The current GUI does not invoke
this provider and has no execution control.

The warm OSMO Cosmos worker is the active proposal provider and is connected to the
console's shadow cycle. PSC batch jobs remain an offline evaluation path and must not
be used as the per-action control loop. The coordinator is not connected to a flight
dispatcher, so it cannot move the drone.

### Persistent Cosmos worker on OSMO

The two-GPU workflow is [`osmo/workflows/airstack-live-replan.yaml`](../../osmo/workflows/airstack-live-replan.yaml):
one GPU is the existing Isaac workspace and one is a private warm Cosmos worker. They
are in one OSMO workflow group, so the workspace receives the internal worker URL
through `RRM_COSMOS_WORKER_URL`; port `8090` must not be port-forwarded to a browser.

The worker image is published at
`airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker:latest`. Before
submitting, accept the Cosmos model terms in Hugging Face and create the user-owned
OSMO generic credential `rrm-huggingface-read` described in
[`osmo/cosmos-worker/README.md`](../../osmo/cosmos-worker/README.md). At every new
workflow start, only the worker task downloads the approved pinned
`nvidia/Cosmos-Reason2-8B` snapshot into its own ephemeral storage, unsets the token,
then loads the model once. No model is stored in Git, the worker image, the registry,
or the Isaac workspace. Once that credential exists, submit a new workflow (do not
restart the shared pool):

```bash
cd /root/AirStack
osmo workflow submit osmo/workflows/airstack-live-replan.yaml \
  --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)" \
  --set-env "ISAAC_SIM_SCENE=Office" \
  --set-env "ISAAC_SIM_STAGE_SCALE=1.0"
```

This requests two GPUs, 24 CPU cores, and 96 GiB memory total. It is a shadow-only
model service; the GUI can record proposals and evidence, but it cannot dispatch a
flight command.

### Attach AirStack helpers to a manually submitted OSMO workflow

`./airstack.sh osmo up` records its submitted workflow ID in
`~/.airstack/osmo-state`. If a workflow was instead submitted directly with
`osmo workflow submit`, helper commands such as `ide`, `logs`, `webrtc`, and
`foxglove` may still target an older cancelled workflow.

For one command, override the saved workflow ID without changing it:

```bash
AIRSTACK_OSMO_WF=airstack-live-replan-1 ./airstack.sh osmo ide
```

To make that workflow the normal target for all helper commands, update the saved
state, then attach:

```bash
printf '%s\n' 'airstack-live-replan-1' > ~/.airstack/osmo-state
./airstack.sh osmo ide
```

Replace `airstack-live-replan-1` with the actual workflow ID returned by your
submission. This only changes the local helper's selected workflow; it does not
restart, cancel, or otherwise modify the running workflow.

## Loading an Isaac Sim Scene (AirStack)

When running AirStack (either locally or via OSMO), you can specify which Isaac Sim scene to load using the `--scene` flag or the equivalent environment variable `ISAAC_SIM_SCENE`.

In the two-GPU command console, the optional **Isaac scene launcher** is catalog-driven:
it exposes every `isaac` entry in `simulation/scenes.yaml`, including custom USD
stages. Choosing one restarts only the inner Isaac and robot services; it does not
submit a new OSMO workflow or restart the warm Cosmos worker. It is a convenience,
not the owner of the stage: edits made in Isaac Sim and scene parameters supplied from
the terminal remain valid. Capture a fresh observation after any external change.

**Locally (via airstack CLI):**
```bash
./airstack.sh up --sim isaac --scene office
```

**Remotely (via OSMO workflow):**
For the active two-GPU Cosmos workflow, use the `airstack-live-replan.yaml` command
above. The one-GPU `airstack-dev.yaml` workflow remains valid for ordinary development,
but a direct environment override must use the resolved Pegasus key `Office`, not the
catalog shortname `office`. Submit it from a local/authenticated OSMO control terminal,
not from inside a Remote-SSH workspace:

```bash
osmo workflow submit osmo/workflows/airstack-dev.yaml \
  --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)" \
  --set-env "ISAAC_SIM_SCENE=Office" \
  --set-env "ISAAC_SIM_STAGE_SCALE=1.0"
```
*(Other configurable parameters like `NUM_ROBOTS=1` or `ISAAC_SIM_HEADLESS=1` can also be passed this way. Refer to the AirStack documentation and `airstack.sh --help` for the full list of parameters).*

### OSMO WebRTC streaming

The standard `isaac-sim` profile does not publish WebRTC ports, so a remote Isaac
streaming client will be blank even when the simulator itself is healthy. For an
OSMO/VS Code remote session, launch the livestream profile instead:

```bash
cd /root/AirStack
COMPOSE_PROFILES=desktop,isaac-sim-livestream \
  ./airstack.sh up --sim isaac --scene office
```

This starts `isaac-sim-livestream` and publishes TCP port `49100` for WebRTC
signaling and UDP port `49099` for media. Forward TCP `49100` in VS Code's **Ports**
panel before connecting the streaming client. Switching from an already running
standard `isaac-sim` container requires stopping/removing that container first, then
starting the livestream profile; the scene will load again. `airstack ready` only
reports readiness—it does not make Isaac load faster.

## AirStack drone shadow mode (OSMO)

`scripts/airstack_shadow.py` is the first SIL adapter. It observes canonical MAVROS
odometry/state, `map -> base_link` TF, and existing task-status topics and writes an
append-only evidence bundle. It creates subscriptions and a timer only: it has no ROS
action client, publisher, service client, trajectory, or PX4 command path. A fresh
bundle permits RRM evaluation in shadow mode; it never permits task dispatch.

Run it inside `airstack-robot-desktop-1` after the stack is ready, with RRM and its
declared `pydantic` dependency available in that container:

```bash
source /root/AirStack/robot/ros_ws/install/local_setup.bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
PYTHONPATH=/tmp/rrm-canonical-deps:${PYTHONPATH} \
  python3 scripts/airstack_shadow.py \
  --robot-name robot_1 --duration-s 30 --output-dir /root/AirStack/.rrm-artifacts/evidence
```

The output has `manifest.json`, `events.jsonl`, and `replay-report.json`. Preserve it
outside the ephemeral OSMO workflow. Start with the model-free adapter and compare any
future reasoning model against the same recorded state/evidence conditions.

## AirStack drone task adapter (OSMO)

`scripts/airstack_drone_dispatch.py` is RRM's output seam for the existing public
AirStack task actions: `takeoff`, `navigate`, and `land`. It uses the same
`task_msgs` action servers exposed to Foxglove; it does not issue PX4, MAVROS, service,
publisher, or trajectory commands directly. A proposal defaults to dry-run:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
PYTHONPATH=/tmp/rrm-canonical-deps:${PYTHONPATH} \
  python3 scripts/airstack_drone_dispatch.py \
  --proposal-json /path/to/takeoff.json
```

An actual simulated task goal requires the explicit `--execute` flag and a
user-approved proposal. This low-level adapter is not a complete C06/C08 system by
itself; the separate authorized mission runner composes dynamic feasibility and
single-use admission around it. Do not invoke the adapter directly for autonomous,
unattended, or physical-robot operation.

For a future explicitly approved simulator goal, `--verify-observation` makes the
runner require fresh, read-only `/odometry_conversion/odometry` before dispatch and
then corroborate the task result with post-result odometry plus MAVROS state. Pair it
with `--outcome-json` to retain the immutable record:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
PYTHONPATH=/tmp/rrm-canonical-deps:${PYTHONPATH} \
  python3 scripts/airstack_drone_dispatch.py \
  --proposal-json /path/to/takeoff.json --execute --verify-observation \
  --outcome-json /root/AirStack/.rrm-artifacts/evidence/takeoff-outcome.json
```

The process exits nonzero unless the independent evidence verifies the action result.
Takeoff verification requires the requested absolute `map` altitude within the
configured 0.3 m tolerance, no more than 0.3 m horizontal displacement by default,
and a connected, armed vehicle state. Landing verification requires near-ground
odometry (default <=0.3 m), a connected, disarmed state, and a successful task result.
Navigation verifies fresh causal endpoint odometry against its proposal tolerance.
If an action server is unavailable, the runner records `goal_sent=false` and
`physical_outcome=NOT_DISPATCHED`. These are observation checks, not C06/C08 safety
authority.

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

## Deploying to the A10G (Legacy / Brev)

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
