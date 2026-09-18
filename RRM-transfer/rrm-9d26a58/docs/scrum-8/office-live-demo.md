# Office learned-plan demonstration

Verified 2026-09-17 23:27 UTC: job `46288765` has been downloaded, hash-checked
and imported successfully. Fresh reimport matches saved decision and proposal;
status READY, action ID NAVIGATE_TO, target blue_marker, waypoint (3.2, 0, 1.5).
No dispatch. See [evidence](evidence/office-inference-46288765.md).
Full connection and visual command-entry UI remain deferred.

The retrieval instructions below are retained for reproducibility; this job's
retrieval/import milestone is complete.

Current next step: job `46288765` completed with ACCEPTED and execution_dispatch=false
according to the user's console/raw-output transcript. It selects NAVIGATE_TO
blue_marker with goal near($self, blue_marker). Actual bundle verification remains
pending authentication. Run in the **OSMO host terminal**:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_office_fetch_import.sh 46288765
```

This downloads into a new `.rrm-artifacts/psc-office-46288765.*` directory, verifies
SHA256SUMS and runs the existing offline importer using isolated Pydantic dependencies
in the running robot container. It prints the unexecuted proposal path. Source evidence
stays on PSC; the OSMO copy is ephemeral. No flight or model rerun occurs. The helper
requires `/tmp/rrm-canonical-deps` prepared in this session and fails early if absent.

Follow-up: retry job `46288321` failed before inference because its source path
fell back to `/src/rrm`. Updated transfer scripts pass source-root as a positional
batch argument and the batch logs it. For the already uploaded older snapshot,
explicitly export its `RRM_SOURCE_ROOT` and use `sbatch --export=ALL` to retry.
Accounting confirmed the requested 30-minute limit; elapsed time was one second.

Latest update, 2026-09-17: job `46280177` finished its inference bundle but its
candidate was REJECTED for using marker descriptions as entity IDs. The prompt
has been clarified. Full simulator connection is deferred at the user's request;
a visual command-entry interface for RRM is a later step. No retry result is claimed.

Run this in the **OSMO host terminal** to transfer the updated source:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_office_retry_transfer.sh 46280177
```

Authenticate to PSC in that terminal. Then run the printed `sbatch` command in the
**PSC login terminal**. It reuses `46280177/input.png`, creates a new source snapshot
and result directory, and requests 30 minutes because the prior model load took
18m08s. It does not connect to ROS or execute a flight. The original rejected bundle
remains evidence; do not edit it into an accepted candidate. New results must be
checked against the matching updated source/prompt before any later import.

The following section records the earlier integration state and plan:

Status, 2026-09-17: Office camera/PX4/GCS live. PSC job `46273277` has been
submitted for actual Office Cosmos inference; learned flight remains pending its
hash-checked result and live revalidation. An earlier accepted warehouse-label
candidate is not an Office decision or a flight result.

## What this first demonstration tests

The learned core receives a semantic task, explicit scene-fixture facts and the
frozen drone image. It chooses a semantic entity/action; the adapter alone maps
that entity to a waypoint. The new `compile_plan` path preserves the model's C05
and action IDs, without calling the deterministic objective parser. This is
ground-truth-assisted cognition plus embodiment execution, not proof of visual
localization, full world-building, online learning or cross-body generalization.

The first binding is a **draft** approach point `(3.2, 0, 1.5)` in robot-local map,
with 0.3 m endpoint tolerance. Before flight verify that current stage/spawn and
map alignment still match, the approach is clear, and the vehicle is ready.
Frozen replay timestamps are not live freshness. Do not relabel them as fresh
simulator observations. Scene markers are visual labels, not collision obstacles.

## Persistent model job

The Office inference was submitted as PSC job `46273277`. It runs independently of
OSMO. Monitor and inspect it from a Bridges-2 shell:

```bash
squeue -j 46273277
JOB=/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/office/46273277
cat "$JOB/console.log"
```

For a future repeat, run in the OSMO host terminal from `~/AirStack`:

```bash
bash RRM-transfer/rrm-9d26a58/scripts/rrm_office_transfer.sh \
  "$PWD/.rrm-artifacts/office-v1/rrm-office-frozen-v1.png"
```

Enter PSC credentials in the terminal only. This creates a separate timestamped
source snapshot on Ocean, preserving existing `/src/rrm`. It also copies the image,
available capture sidecar and scene launcher. Run the exact `sbatch` line it prints
in the PSC login terminal. No interactive GPU session is needed. Once submission
succeeds, the job survives laptop disconnect; an in-progress rsync may not.

The batch uses the existing Cosmos-Reason2-8B pinned snapshot and Transformers
environment, offline, on one H100. Results live under
`$PROJECT/physical-ai/runs/rrm/office/<JOB_ID>/`; `console.log`, `result.json`, copied
input image/context, source hashes and `SHA256SUMS` are retained. The batch is
inference-only because PSC has no ROS connection to this OSMO robot.

Retrieve that directory with the known working DTN rsync transport, then run inside
the RRM Python environment:

```bash
PYTHONPATH=. python scripts/rrm_import_office.py \
  --bundle /path/to/retrieved-job --output-dir /path/to/new-import
```

The importer verifies image/context hashes, exact prompt, reparsed raw response,
stored candidate equality and local scene binding before emitting the **actual**
learned proposal. It does not execute or establish freshness/authority. Checksums
detect changed artifacts; they are not cryptographic proof of model authorship.

After live revalidation, takeoff and navigation use public AirStack task actions;
do not bypass them with raw setpoints. For the navigation goal, the dispatcher is:

```bash
PYTHONPATH=. python scripts/airstack_drone_dispatch.py \
  --proposal-json /path/to/new-import/proposal.json --execute \
  --verify-observation --action-timeout-s 90 \
  --outcome-json /path/to/new-run/navigation-outcome.json
```

This command requires an already armed vehicle after verified takeoff. Navigation
success requires task success plus fresh causal odometry within the endpoint
tolerance. A timeout requests cancellation and records UNCONFIRMED; cancellation
acknowledgement is not proof of physical stop. No automatic retry/resume is allowed
after unknown acceptance/completion. The live mission supervisor, current-scene
revalidation and independent stop evidence are still integration work, not claimed
complete by this runbook.

The 2026-09-18 live attempt stopped at this gate: `droan_gl_node`, the owner of the
navigate action, exited `-11` during startup. The action name remained transiently
visible in graph discovery, but the live server handshake failed and no navigation
goal was sent. Fix and verify that node before repeating the mission. A server
unavailability record must say `goal_sent=false` and `physical_outcome=NOT_DISPATCHED`.

The failure was traced to stale Xvfb display-99 lock/socket files after a robot
container restart. The Compose startup now removes those exact files only when no
Xvfb process exists. A grounded diagnostic then initialized Mesa OpenGL 4.5, loaded
the Droan GL shaders, and exposed a live navigate server. Recreate the robot container
from the patched Compose definition and repeat all preflight gates before the next
flight; the one-off diagnostic node was stopped.

## Viewers

Office launch uses `rrm_office_visual_eval.py`, `ISAAC_SIM_SCENE=Office`, scale 1
and the `isaac-sim-livestream` profile. Its scene-specific startup pins
`activeGpu=0`, `multiGpu/enabled=false`, `physics/cudaDevice=0`. Robot and GCS
containers need not be restarted just to change the simulator variant.

On the Mac, keep the patched forwarding command running:

```bash
cd /Users/oabolade/Downloads/AirStack
PATH="$HOME/.airstack/osmo-patched/bin:$PATH" ./airstack.sh osmo webrtc
```

Connect the AirLab Isaac client to `127.0.0.1`. In another Mac terminal:

```bash
cd /Users/oabolade/Downloads/AirStack
PATH="$HOME/.airstack/osmo-patched/bin:$PATH" ./airstack.sh osmo foxglove
```

Connect Foxglove to `ws://127.0.0.1:8766`, import the local AirStack default layout.
`/gcs/robot_markers` shows the robot and executing trajectory. A working bridge is
not proof the Mac client is connected. Closing forwarding disconnects the viewer;
it does not itself stop the remote simulator. OSMO files remain ephemeral until
exported; local Git commits alone do not survive destruction of that storage.

## Evaluation scope

The [capstone implementation guide](https://001-physical-ai-book.vercel.app/docs/capstone/implementation-guide/implementation-guide)
provides integration, testing, performance and safety categories, not a ready-made
RRM test set or acceptance thresholds. Proposed RRM measurements are:

| Layer | Measurement |
| --- | --- |
| Visual world-building | Fact precision/recall, unsupported claims, evidence freshness |
| Reasoning | Correct target/action, valid schema, appropriate clarification/refusal |
| Execution | Observed endpoint error, task success, timeouts, model/action latency |
| Robustness | Ambiguous target, missing target, stale/conflicting facts, changed capabilities |
| Body independence | Same semantic task/core across different validated adapters |

One blue-marker flight establishes integration only. Freeze scene/task/seed/model
revision and run matched repeats/negative cases before claiming benchmark results.
