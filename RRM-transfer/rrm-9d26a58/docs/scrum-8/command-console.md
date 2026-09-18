# Local command console and Isaac camera

The console combines task entry, the previous verified Cosmos plan, a frozen model
reference image and a refreshable read-only Isaac camera view. Foxglove can stay
open alongside it for continuous telemetry and trajectories. It now also exposes a
narrow, explicitly gated Office-demo execution boundary for that one imported plan.

Update 2026-09-18: SQLite-backed goal and attempt history is available on the same
port. Refresh the browser after the console update. The previous accepted inference
is indexed as a historical candidate with execution state NOT_DISPATCHED.

## Saved goals and history

**Save new goal & request** creates an immutable goal and its first unsent request.
In **Saved goals & history**, select **Use goal** to fill the form, then **Save another
request** to create a separate attempt for that goal. Selection alone creates no
request or command. Editing the text explicitly switches to saving a new goal;
earlier goals, attempts and evidence remain unchanged. Expand a goal to see its
attempt statuses and download inputs or its historical result.

The database is `/root/AirStack/.rrm-artifacts/command-requests/tasks.sqlite3`.
The `goals` table stores text, constraint revision, embodiment reference and creation
time; `runs` stores task linkage, status, artifact directory and execution state.
Images, request contexts and model results stay in files. No API marks tasks running
or completed: currently supported statuses are SAVED_NOT_SUBMITTED and
CANDIDATE_ACCEPTED, both NOT_DISPATCHED. Inference and execution integration are future
work, so an accepted historical candidate is not presented as a successful flight.

History survives server restarts. Startup idempotently indexes existing request
folders, including legacy manifests, and checks their content hashes. Manifests
carry goal IDs for recovery if file saving succeeds before database indexing.
OSMO storage remains ephemeral: preserve the database **and** its referenced artifact
directories before ending the workflow. The original accepted inference stays on PSC.

## Open the running console

In the VS Code/Cursor Remote-SSH session's **Ports** panel, forward port **8787**.
Open the forwarded address, normally `http://localhost:8787`, in your Mac browser.
The server binds only to `127.0.0.1` on the OSMO host. If the IDE assigns another
local port, use the address it shows. No new PSC authentication is needed to use
the local console.

If restarting, run on the OSMO host:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_command_console.sh \
  /root/AirStack/.rrm-artifacts/psc-office-46288765.FCRcPE/bundle
```

The launcher reuses this session's Pydantic 2 dependencies from the robot container
and copies the existing read-only capture utility to a separate temporary directory.
Host/container Python are both 3.12 in this workspace. The running robot container,
its ROS workspace, dependencies and downloaded bundle are required. Ctrl+C stops
the foreground console; it does not stop the simulator.

## What the controls do

- **Refresh camera** subscribes briefly to the current front-camera image, with an
  eight-second observation timeout. The snapshot shows capture time and source
  simulation timestamp; an unchanged timestamp is flagged. It is not a video stream.
- **Save inference request** preserves a new `input.json`, original frozen
  `input.png` and `request.json` under `.rrm-artifacts/command-requests/<request-id>/`.
  Files are also downloadable through the page. Each request has new task/state
  identifiers, while its frozen observation times and declared capabilities stay
  unchanged. The existing inference CLI can load the saved input.
- **Last accepted plan** always labels the previous verified PSC result. It does
  not become a result for a newly entered instruction.
- **Open Foxglove** opens the Foxglove web app. Keep the existing Mac forwarding
  active and connect to `ws://127.0.0.1:8766` as in the Office runbook.

Saving does not submit a PSC job, run Cosmos, import a new result, or dispatch a
drone task. The current PSC Office batch still uses its fixed example context;
do not assume it will automatically consume these new request files. Camera
refreshes do not replace the frozen model input or establish fresh semantic facts.
The current camera view is mostly floor/wall; stage/map alignment is not validated.

## Exact-proposal approval and stop

The **Exact proposal execution** panel displays the imported proposal, public task
action, exact goal payload and a canonical SHA-256 digest. **Approve & dispatch**
requires a second confirmation and sends that exact digest to the loopback server.
The server writes `admission.json` before launching the existing
`airstack_drone_dispatch.py --execute --verify-observation` adapter. A changed digest,
duplicate active execution, prior decision or latched stop fails closed. Approval is
for the displayed historical PSC proposal only; it does not approve newly saved
requests, and no approval happens automatically.

**Reject** records `NOT_DISPATCHED` and makes that console instance terminal for the
proposal. **STOP** first closes admission and increments the stop generation, then
interrupts an active dispatcher. Once its ROS goal has been accepted, the dispatcher
handles that interrupt by requesting public action cancellation. The page intentionally
reports `SAFE_UNCONFIRMED`: cancellation delivery, acknowledgement or process exit is
not independent proof that the drone stopped. Reconcile vehicle state and restart the
console before any later admission.

Execution evidence is stored under
`.rrm-artifacts/command-requests/execution/<dispatch-id>/`, including the exact
proposal, admission record, dispatcher log, outcome when available and stop records.
These artifacts are ephemeral and gitignored.

The separate **Restart simulation (development)** button preserves earlier development
functionality. It restarts simulator/robot services and drops WebRTC temporarily. It is
not C08, not an emergency stop, and not evidence of a safe physical state.

This is a narrow single-process demonstration boundary, not the complete distributed
C06/C08 design in `interfaces.md`: it does not provide authenticated multi-user
authority, durable cross-host deduplication, restart reconciliation, or independent
motion-stopped/safe-confirmed sensing. Those omissions must remain explicit in results.

## Verification, 2026-09-17

71 CPU unit tests pass, including new request identity/replay preservation and HTTP
submission/file-access tests. HTML JavaScript passes Node syntax checking. Live HTTP
camera refresh returned two 480×300 images with advancing simulation timestamps:
3319429925804 → 3320329925784 ns. Image hashes were verified and the second image
visually inspected. Full graphical browser rendering was not tested automatically.
No model run, simulator restart or robot dispatch occurred.

## Database verification, 2026-09-18

75 CPU tests pass, including restart persistence, two attempts for one goal,
immutable goal text, legacy-folder indexing without rewriting evidence, corrupted
artifact rejection and history/download endpoints. JavaScript syntax and diff
checks pass. After restart, the live database integrity check returned `ok`; it
contained one historical goal/run from the actual verified bundle. Its original
four checksums still pass. No production test requests or robot tasks were created.
