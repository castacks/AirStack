> **Current continuation (2026-09-18):** Read [HANDOFF.md](HANDOFF.md) first. The [localhost command console](docs/scrum-8/command-console.md) now shows immutable reconciliation/admission/STOP/landing evidence in goal history, captures read-only live Isaac image plus canonical vehicle state, and queues each new live-observation request through a credential-free PSC bridge boundary. Returned PSC bundles must pass input/image hashes, scene binding, C04/C05 and adapter validation before an exact-proposal approval can be recorded. Approval remains non-dispatching in this increment. A real PSC worker needs an approved non-interactive PSC key/agent; the browser never accepts credentials. See [evidence](docs/scrum-8/evidence/office-inference-46288765.md) and the [architecture status](docs/scrum-8/README.md). The prototype material below is historical and does not override the current handoff.

The remaining route to full model-to-drone operation is: provision the approved PSC
bridge; dispatch only a separately revalidated and explicitly approved plan through
public AirStack task actions; then prove renewed scene/result/vehicle freshness in an
end-to-end simulator suite. The detailed safety conditions are in `HANDOFF.md`.

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

## RRM Command Console & PSC Bridge (OSMO)

The RRM GUI (Command Console) is used for visual intake, reviewing, and approving execution commands. It uses an asynchronous bridge to communicate with PSC. **All commands below must be run in your OSMO terminal.**

**Prerequisites:** Before you can start the console, two things must happen:
1. **Isaac Sim and AirStack must be running:** The console needs to connect to the active simulation to pull live camera images and robot odometry.
2. **You must fetch a historical PSC bundle:** The console requires an initial, previously completed inference job from PSC to bootstrap its internal state and UI. (This is what the "fetch" script does).

**1. Fetch a historical PSC bundle:**
*(This downloads a completed inference job from PSC. Look for the `Verified bundle:` path in the output).*
```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_office_fetch_import.sh <psc-job-id>
```

The PSC job ID is an operator-provided value, not a folder in a recreated
VS Code/OSMO workspace. Run the command in an MFA-capable terminal to download and
verify its bundle, then use the printed `Verified bundle:` path below. The transferred
handoff records `46288765` as the prior accepted Office inference; use it only when
you intentionally want that historical result, not as a default for a new scene.

**2. Start the console:**
```bash
# Provide the verified bundle directory path that was output by the fetch script above:
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_command_console.sh <verified-bundle-directory> [port]
```
*(The console will be accessible at `http://127.0.0.1:8787` by default)*

**Manual PSC Job Submission:**
If the console's automated background submission fails (due to MFA/Duo prompts blocking the non-interactive SSH key), use this helper script in your terminal to manually push a request created in the GUI:
```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
RRM_PSC_USER=<your-psc-username> bash scripts/rrm_psc_bridge_manual.sh /root/AirStack/.rrm-artifacts/command-requests/<REQUEST_ID>
```
*(Reload the GUI history after the Slurm job finishes to view results).*

For PSC password/Duo MFA, the manual bridge submits once and exits after printing the
job ID. Monitor it with `squeue -j <job-id>`; when it has completed, fetch and import
that exact result without submitting another job:
```bash
RRM_PSC_USER=<your-psc-username> bash scripts/rrm_psc_fetch_result.sh \
  /root/AirStack/.rrm-artifacts/command-requests/<REQUEST_ID> <PSC_JOB_ID>
```
The GUI records the submitted job ID in its history when the console is running, then
shows the terminal result after the fetch/import command completes. A result is
reviewable only when it is `CANDIDATE_ACCEPTED`; malformed/mismatched evidence is
`INFERENCE_FAILED`, and a model plan that the single-navigation Office adapter cannot
represent is `CANDIDATE_REJECTED`. Both terminal non-accepted states retain evidence,
do not dispatch, and must be retried with a new saved request rather than resubmitting
the same immutable request.

The manual bridge defaults to one `h100-80` GPU. To trial the lower-queue L40S 48 GB
option without cancelling a queued H100 job, explicitly set `RRM_PSC_GRES=gpu:l40s-48:1`.
Treat that as a separate validation run: import and review its result only if it
completes successfully.

### Live action → replan workflow (foundation)

`rrm/live_replan.py` is the provider-neutral, shadow-only coordinator for the future
continuous workflow. It records a fresh camera image plus separately verified live
scene state before each provider request; binds the provider result to that exact
observation; exposes only the first action of a multi-action plan for review; and then
requires a reviewed, independently verified outcome before it will accept another
observation/replan. It has no ROS, model-runtime, PSC, or dispatch dependency.

The next integration supplies a warm OSMO Cosmos worker (or approved VLM API) as the
provider. PSC batch jobs remain an offline evaluation path and must not be used as the
per-action control loop. The current coordinator is not connected to the GUI or flight
dispatcher yet, so it cannot move the drone.

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
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)"
```

This requests two GPUs, 24 CPU cores, and 96 GiB memory total. It is a shadow-only
model service until the provider adapter and GUI cycle view are connected.

## Loading an Isaac Sim Scene (AirStack)

When running AirStack (either locally or via OSMO), you can specify which Isaac Sim scene to load using the `--scene` flag or the equivalent environment variable `ISAAC_SIM_SCENE`.

**Locally (via airstack CLI):**
```bash
./airstack.sh up --sim isaac --scene office
```

**Remotely (via OSMO workflow):**
When submitting the `airstack-dev.yaml` workflow to OSMO, use the `--set-env` argument to override the default scene:
```bash
osmo workflow submit osmo/workflows/airstack-dev.yaml \
  --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)" \
  --set-env "ISAAC_SIM_SCENE=office"
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
user-approved proposal. The adapter is not yet the complete C06 supervision or C08
stop implementation; do not use it for unattended or physical-robot operation.

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
