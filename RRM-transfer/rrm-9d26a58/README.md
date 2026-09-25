> **Current continuation (2026-09-21):** Read [HANDOFF.md](HANDOFF.md) first. The
> [localhost command console](docs/scrum-8/command-console.md) now has a real,
> explicitly confirmed simulator execution path. It translates scene-independent
> movement text into typed goals only for task action servers discovered in the active
> AirStack configuration, executes them serially, verifies outcomes from fresh state,
> and exposes STOP/HOLD. It does not send PX4/MAVROS or trajectory commands directly.
> The older Cosmos/PSC proposal flows remain available as research/evidence paths; they
> are not required for direct takeoff, land, exploration, waypoint, or relative-motion
> commands.

# RRM-1 — Robotics Reasoning Model

> **Isaac callback update (2026-09-25):** After verifying the live 23-joint profile,
> an isolated probe deactivated the hand asset and delivered 240/240 physics callbacks
> to the disabled gateway. All returned `IDLE`, liveness stayed healthy, the asset was
> inactive on every callback, and zero actions were attempted. Maximum callback gap
> was 3.116 ms against 100 ms; the suite passes 234/234. This validates callback wiring
> only—not a dynamic articulation, load, stop/hold, safe state, or motion. See
> [HANDOFF.md](HANDOFF.md).

> **Live idle-heartbeat update (2026-09-25):** A second headless Isaac process ran
> 1,000 disabled gateway ticks against the live 23-joint Kuka-Allegro articulation.
> All returned `IDLE`, liveness remained healthy, motion stayed disabled, and a hard
> guard observed zero action calls; maximum externally measured tick time was 18.12 us
> against a 0.1 s limit. The suite passes 232/232. This is direct no-action timing,
> not physics-callback, scheduler-under-load, stop/hold, safe-state, or motion
> qualification. See [HANDOFF.md](HANDOFF.md).

> **Gateway liveness update (2026-09-25):** The isolated hand gateway now records
> simulator-thread heartbeat and stop-to-hold timing and latches stale, late-stop, or
> clock-regression faults closed across restart. CPU-only stress covered 1000 idle
> ticks and 100 concurrent reads with zero articulation calls. The RRM suite passes
> 231/231. This is not live Isaac stop qualification and enables no motion. See
> [HANDOFF.md](HANDOFF.md).

> **Authority update (2026-09-25):** The hand boundary now requires authenticated,
> short-lived, purpose/scope/epoch/generation-bound grants instead of a boolean
> authorization shortcut. Grant consumption is fsynced before reset, reconciliation,
> or dispatch intent and remains consumed after restart. This is a local HMAC contract,
> not production identity infrastructure or live execution approval. The RRM suite
> That checkpoint passed 227/227; no simulator or robot was touched. See
> [HANDOFF.md](HANDOFF.md).

> **Restart reconciliation update (2026-09-25):** Hash-linked boundary and gateway
> journals now restore stop generations, consumed IDs, and unresolved dispatches.
> Recovery requires a gateway hold plus fresh measured safe samples, then a separate
> boundary reconciliation and reset; it never implicitly enables motion. This is
> CPU/fake-articulation validation only. That checkpoint passed 226/226; no hand or
> aerial motion was sent. See [HANDOFF.md](HANDOFF.md).

> **Live profile smoke (2026-09-25):** A separate headless Isaac process found
> zero mismatches across the Kuka-Allegro 23-joint names and position/velocity
> limits. The gateway stayed disabled, returned `IDLE`, and made zero action
> calls. This does not qualify controller gains, safe state, stop, or motion.
> That checkpoint passed 223/223 tests. See [HANDOFF.md](HANDOFF.md).

> **Gateway update (2026-09-25):** An isolated, disabled-by-default Isaac hand
> gateway skeleton now exists in `simulation/hand_isaac_adapter.py`. It has
> fake-articulation tests only. It has not been bound to a live Isaac scene,
> qualified for physical stop, or used to send motion. See [HANDOFF.md](HANDOFF.md).

> **Hand boundary update (2026-09-25):** An injected-adapter, transport-free
> C06/C08/C09 prototype and negative tests are present in
> `rrm/hand_execution_boundary.py`. It has no Isaac/ROS implementation and grants
> no live execution authority. See [HANDOFF.md](HANDOFF.md) for the remaining
> gateway, reconciliation, stop, and supervised-test gates.
> That increment passed 213/213 tests; the current gateway increment passes
> 219/219. No hand or aerial motion was sent.

> **Current safety status (2026-09-24):** The latest bounded aerial-console
> takeoff failed and climbed to 4.076 m during verified recovery landing from a
> 1 m command. The vehicle ended grounded/disarmed, but the cause is unresolved:
> **do not repeat aerial flight**. Kuka-Allegro hand work is isolated from that
> stack; its controller prerequisite gate now permits preparation of one bounded
> contact trial, but **no hand execution is enabled**. Read [HANDOFF.md](HANDOFF.md)
> before continuing. The current
> worktree is uncommitted. The separate Kuka-Allegro tabletop probe now has a fresh,
> image/state-paired no-action run and a 12-fact simulator C02 teacher export; its
> manual image-only review scored 9/12 recall. The separate strict controller probe
> qualifies limits, reset, contact observation, safe-state, and independent stop, but
> not contact stability, grasp, C06/C08/C09 completion, or dispatch. The
> dependency-light RRM suite passes 206/206 tests.

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

## RRM Command Console (OSMO)

The GUI accepts a movement command, records an immutable attempt, discovers the real
public task executors in the running robot stack, compiles a typed action sequence, and
runs it after browser confirmation. Supported scene-independent command forms are:

- `take off`, `land`;
- `explore`, `survey`, `roam`, or `map the ...`, optionally with numeric or common
  language durations such as `for 20 seconds`, `for 2 minutes`, `briefly`, or
  `for a couple of seconds`;
- one or more robot-local map points, such as `fly to x=2 y=-1 z=1.5` or
  `fly through (1,2,1.5), (4,-2,2)`;
- current-heading-relative movement such as `move forward 2 meters`, `move left 1m`,
  or `move up 1m`;
- `come back`, `go back`, or `return home/to the start`, which inserts navigation to
  the fresh command-start map pose before any requested landing.

Grounded navigation/exploration automatically inserts takeoff. Exploration invokes
AirStack's `ExplorationTask`, whose active global planner uses the live VDB map and
continually delegates/replans through `NavigateTask`; coordinate and relative routes
use `NavigateTask` and its active DROAN local planner. Commands are rejected when the
required action server, canonical state, or exploration map feed is absent. A request
to move to an arbitrary visually described object still requires a real
`SemanticSearchTask`/equivalent executor. Likewise, a return leg requires a currently
served `NavigateTask`. ROS graph names created only by action clients are excluded, so
RRM does not invent either capability when its server is absent.

The camera is optional for these movement commands. It remains available for visual
evidence and the older model-grounding workflow. Commands therefore work after any
catalog scene switch or manual stage edit; execution relies on current robot/map state,
not the Office entity catalog.

The deterministic compiler records the origin of every numeric parameter in
`command-plan.json`: operator value, semantic interpretation, current environment,
versioned vehicle envelope, or policy default. Exploration receives a conservative
XY polygon around the start pose, tightened by fresh VDB extents when those extents
enclose the vehicle. Semantic estimates that fall outside a supported minimum are
adjusted visibly—for example, “a couple of seconds” is interpreted as 2 seconds and
recorded as a 5-second task minimum, rather than silently becoming 60 seconds.
Material ambiguity produces one targeted clarification and no plan.

**Prerequisite:** Isaac Sim and AirStack must be running so the console can discover
task servers and canonical robot state.

**Start the live console:**
```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_command_console.sh
```

The page exposes **Plan and run** plus **Stop / hold**. The single Plan and run click
starts the autonomous mission; its compiled task plan, task feedback, and published
global-plan/replan updates are shown as scrollable evidence without an approval pause.
Plans that include takeoff also predeclare a public `LandTask` contingency. A terminal
takeoff result that fails physical verification while fresh evidence still shows an
armed airborne vehicle halts the requested sequence, executes that recovery landing,
and reports whether grounded/disarmed recovery was independently verified. The same
predeclared landing is available if a later inter-action reconciliation halts with
fresh evidence of meaningful armed flight; absent that evidence, RRM does not issue a
blind landing.
RRM refuses dispatch when the robot container predates the current Isaac process,
preventing retained TF/controller state from crossing a simulation-clock reset. The
takeoff server itself starts from current physical odometry and aborts at 0.3 m of
unexpected horizontal displacement.
Plan records and independently observed task outcomes are stored below
`/root/AirStack/.rrm-artifacts/command-requests/<request-id>/`. Scene switching is
blocked while a command mission is active.

Between verified actions, the mission runner reacquires canonical airborne, armed,
connected, and odometry state. It records a `CONTINUE`, `SKIP_SATISFIED`, or `HALT`
decision before the next action. It never retries an unknown or failed action. Task
feedback includes elapsed time, pose, speed, and takeoff displacement when available;
terminal evidence includes numeric metrics and symptom classifications. These diagnose
what was observed but do not claim an unproven controller root cause.

An `airborne=true` report at or below 0.3 m map altitude is treated as contradictory
post-abort state, not permission to skip takeoff and begin navigation. RRM blocks all
new motion except an explicit landing/reconciliation command until the state is
cleared. This threshold matches the command adapter's existing recovery boundary and
assumes the supported AirStack configuration's map-zero ground convention.

### Command-mission history and evidence

Each saved goal produces an immutable `<request-id>` directory under
`/root/AirStack/.rrm-artifacts/command-requests/`. These gitignored files are the
source of truth for past runs:

| File | Evidence retained |
|------|-------------------|
| `input.json` | The saved goal at `task.objective`, task identity, and bound context. |
| `request.json` | Request/goal IDs, creation time, lifecycle state, and input/media hashes. |
| `command-plan.json` | Fresh discovered state, exact typed actions, parameter provenance/assumptions, inter-action policy, and any takeoff recovery action. |
| `command-mission.log` | Timestamped preflight state, task feedback, pose/speed/displacement telemetry, replan decisions, and results. |
| `command-mission-evidence/*-outcome.json` | Independently observed outcome for each dispatched action, including its verdict and reason. |
| `command-mission-evidence/replan-*.json` | Fresh inter-action observation and the resulting continue/skip/halt decision. |
| `command-mission-evidence/mission-outcome.json` | The terminal mission result: `VERIFIED`, `HALTED`, `RECOVERED_HALT`, or `RECOVERY_FAILED`, with action, replan, and recovery results. |

`tasks.sqlite3` at the command-request root is a convenience index for saved goals,
runs, lifecycle state, and checksum-bound evidence events; it is not a replacement for
the per-request files. The GUI's **Saved goals** panel shows the goal and attempt
count. Read the files above to audit exact actions and success/failure evidence.

Summarize all retained takeoff attempts without sending a command:

```bash
python3 scripts/rrm_takeoff_history.py
```

### Start a new two-GPU Office workflow

Use the two-GPU workflow for a fresh Isaac workspace plus warm Cosmos worker. The
catalog shortname is `office`, but direct environment configuration must use the
resolved Pegasus key `Office` (capital `O`). **Run this from your local machine or
an authenticated OSMO control terminal, not from the Remote-SSH shell inside an
already-running OSMO workspace.** That workspace intentionally does not carry your
personal OSMO client login or `~/.ssh/id_ed25519.pub`.

#### Scene selection in the two-GPU workflow

This uses the same `ISAAC_SIM_SCENE` / `ISAAC_SIM_STAGE_SCALE` environment contract
as AirStack's normal Isaac launch, but values are supplied at **workflow submission**.
The checked-in two-GPU YAML deliberately leaves both values unset, so a submission
without them loads Pegasus's `Default Environment`. You need no scene flags when that
is what you want. Each OSMO workflow is a fresh workspace, so include the two flags
again for every new Office workflow; they do not persist across submissions.

The checked-in RRM semantic examples and Office manifest require `Office` with scale
`1.0`. `Office` is the Pegasus key; do not pass the catalog shortname `office` as the
environment value. Selecting another catalog scene through the console changes only
the current workspace's Isaac and robot services; it does not affect the next OSMO
submission and disables the Office-specific semantic path until its scene binding
matches again.

```bash
cd /root/AirStack
osmo workflow submit osmo/workflows/airstack-live-replan.yaml \
  --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)" \
  --set-env "ISAAC_SIM_SCENE=Office" \
  --set-env "ISAAC_SIM_STAGE_SCALE=1.0"
```

#### Temporary OSMO boot checklist (until the version-pinned images land)

The deployed OSMO images are currently mutable. A new workflow can therefore
boot with two independent regressions even when the source checkout is correct:

- `robot-desktop` can start a few milliseconds before Isaac Sim, leaving stale
  TF/controller state; and
- the inner Isaac image can install NumPy 2.x, which causes Isaac's render
  writers to fail. The visible symptom is an empty Isaac camera feed, zero raw
  Ouster publishers, and consequently an empty Foxglove VDB map.

Run this **only on a fresh, grounded, disarmed workflow with no active mission**.
Do not restart Isaac or a robot as a recovery step while the vehicle is airborne
or a command is executing.

1. First wait for the normal control-plane gates:

   ```bash
   airstack ready
   ```

   If this reports that the robot predates Isaac, restart only the robot and
   rerun the gate:

   ```bash
   docker restart airstack-robot-desktop-1
   airstack ready
   ```

2. Confirm that the simulator image did not install the broken NumPy version:

   ```bash
   docker exec isaac-sim-livestream \
     /isaac-sim/python.sh -c 'import numpy; print(numpy.__version__)'
   ```

   If it prints `2.x`, apply this temporary repair inside the current inner
   Isaac container. It lasts only for this workflow; every fresh workflow must
   be checked again until the rebuilt image is deployed.

   ```bash
   docker exec isaac-sim-livestream bash -lc \
     '/isaac-sim/python.sh -m pip install --no-cache-dir --force-reinstall numpy==1.26.4'
   docker restart isaac-sim-livestream

   # Wait until this prints "Ready for takeoff!", then press Ctrl-C.
   docker logs -f isaac-sim-livestream

   # Start the robot after the newly restarted Isaac instance.
   docker restart airstack-robot-desktop-1
   airstack ready
   ```

3. Verify sensor data before opening the command console. `airstack ready`
   checks clock/control readiness but does not verify camera, lidar, or map
   content:

   ```bash
   docker exec airstack-robot-desktop-1 bash -lc '
     sws
     ros2 topic info /robot_1/sensors/ouster/point_cloud_raw
     ros2 topic info /robot_1/sensors/front_stereo/left/image_rect
   '
   ```

   Each topic must report at least one publisher. Within a few seconds, the
   VDB marker on `/robot_1/vdb_mapping/vdb_map_visualization` should contain
   points and the Foxglove map should populate. If either sensor has zero
   publishers, do not start a mission; repeat step 2 for this fresh workflow.

A rebuilt, version-pinned workspace image plus the pinned Isaac image is the
durable solution; this checklist is intentionally temporary.

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

### Legacy model action → replan workflow

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

For the separate targetless takeoff path, use
`examples/office_visual_eval/takeoff_context.json` with a reviewed copy of
`takeoff_authorization.example.json`. The checked-in authorization is an example, not
an operator approval. Execution still requires all explicit flags and live feasibility
admission described below.

The public AirStack task-action adapter is unavailable unless the operator adds both
`--execute --simulator-only` and configures an executable
`--feasibility-provider`. The provider receives the exact C03 query on stdin, may
query embodiment-specific simulator physics and planning, and must return a typed,
short-lived result on stdout. The bundled Office profile is
`scripts/airstack_drone_feasibility_provider.py`. It combines fresh read-only vehicle,
controller, planner and action-graph state with a map-frame Ouster point-cloud corridor
check; it stores checksum-bound inline evidence and fails closed on stale channels,
unknown coverage, obstacles, incompatible vehicle state, or a missing stop path. The
legacy `office-airframe-v1` / `office-bounded-nav-v1` profile remains limited to an
already-airborne vehicle and one straight `NAVIGATE` waypoint. The opt-in
`office-airframe-v2` / `office-bounded-flight-v2` profile adds separately checked
targetless `TAKEOFF` and multi-waypoint `NAVIGATE` paths. A marker may bind an exact
route with `map_route`, a nonempty list of map-frame XYZ objects. The adapter validates
a supplied route; it does not invent or globally search for one. These execution
flags are deliberately not shown as a routine startup command: use them only after
the worker image deployment, fresh simulator readiness/reconciliation, independent
observer coverage, and a supervised review.

The configured profiles are intentionally exact: embodiment `aerial-eval`, legacy
capability/limits `office-airframe-v1` / `office-bounded-nav-v1`, or expanded
capability/limits `office-airframe-v2` / `office-bounded-flight-v2`. The corridor start
must agree with the checksum-bound camera observation within 0.25 m, every observed
route waypoint must exactly match the compiled command, and odometry must be
`map -> base_link`. Any changed profile, stale channel, detached corridor, or
insufficient point/range coverage blocks admission. This remains the stricter
Office/catalog semantic-target path; the GUI's direct command path instead delegates
planning to the active public AirStack task executor and records its discovered state.

The warm OSMO Cosmos worker remains an optional proposal provider for the legacy visual
semantic cycle. PSC batch jobs remain an offline evaluation path and are not the direct
GUI command loop.

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

This requests two GPUs, 24 CPU cores, and 96 GiB memory total. The Cosmos worker is a
shadow-only model service and never receives dispatch authority. The GUI's separate
deterministic command path can dispatch only through discovered public AirStack task
servers after local confirmation; it does not grant the worker a control path.

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

## Kuka-Allegro isolated prerequisite gate

`simulation/hand_controller_probe.py` and `scripts/evaluate_hand_qualification.py`
form a simulator-only, fail-closed prerequisite check. The accepted probe S keeps all
23 joint peaks inside unchanged USD limits, converges a bounded arm target, confirms
repeatable resets and safe-state/stop windows, and proves a named fingertip contact can
be observed. Its calibration overlap is intentionally not treated as stable contact or
a grasp. The qualification result authorizes only preparation of one bounded contact
trial; it provides no C06/C08/C09 completion or execution dispatch.

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
