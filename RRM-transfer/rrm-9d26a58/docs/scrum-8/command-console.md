# Local command console and Isaac camera

The current console combines Isaac scene selection, a refreshable read-only camera
view, mission-goal entry, private-worker RRM proposals, and saved-goal history.
Foxglove can stay open alongside it for continuous telemetry and trajectories. The
current page has no approval, stop, landing, or dispatch controls.

Update 2026-09-20: live-only mode now supports the current workflow visible in the
GUI: choose an AirStack catalog scene, capture a fresh observation, save a goal, and
select **Ask RRM for a proposed action**. The console calls the private warm worker
first for catalog-bounded entity verification and then for a checksum-bound semantic
proposal. The action is displayed for inspection only. The GUI does not run dynamic
feasibility, consume admission, or dispatch it.

Only the checked-in Office scene has a matching RRM manifest and entity catalog.
Selecting another catalog scene restarts Isaac and the robot stack but leaves the OSMO
workflow and Cosmos worker running; live proposals remain inhibited until a matching
manifest/catalog exists. Always capture a new frame after a scene change.

Update 2026-09-18: SQLite-backed goal and attempt history is available on the same
port. Refresh the browser after the console update. The previous accepted inference
is indexed as a historical candidate with execution state NOT_DISPATCHED.

## Saved goals and history

**Save goal** creates an immutable goal and its first mission record.
In **Saved goals & history**, select **Use goal** to fill the form, then **Save another
mission record** to create a separate attempt for that goal. Selection alone creates
no request or command. Editing the text explicitly switches to saving a new goal;
earlier goals, attempts and evidence remain unchanged. Each goal shows its recorded
attempt count. The underlying run/evidence records remain in the loopback API and
artifact directory; the current page does not expose evidence download or execution
controls.

The database is `/root/AirStack/.rrm-artifacts/command-requests/tasks.sqlite3`.
The `goals` table stores text, constraint revision, embodiment reference and creation
time; `runs` stores task linkage, status, artifact directory and execution state.
Images, request contexts and model results stay in files. The lifecycle is explicit:
`SAVED_NOT_SUBMITTED`, `INFERENCE_QUEUED`, `INFERENCE_RUNNING`, accepted/rejected or
failed candidate, then `REVIEW_REQUIRED` or recorded approval. The retained API can
serve allow-listed evidence such as grounded reconciliation, admission, dispatch
outcome, STOP delivery/outcome, landing, PSC submission/receipt/result, and exact
candidate approval. The current page intentionally does not render those historical
execution controls.

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
bash scripts/rrm_command_console.sh
```

That starts **live-only** mode from the checked-in Office context and scene manifest;
it needs no PSC job ID or imported bundle. It can capture/save live requests and use
the private worker, but has no historical proposal and no flight-dispatch surface.
To inspect the historical reference instead, pass its verified bundle directory as
the first argument. Historical reference mode remains separate from the automatic
mission work.

The launcher reuses this session's Pydantic 2 dependencies from the robot container
and copies the existing read-only capture utility to a separate temporary directory.
Host/container Python are both 3.12 in this workspace. The running robot container,
its ROS workspace, dependencies and downloaded bundle are required. Ctrl+C stops
the foreground console; it does not stop the simulator.

## What the controls do

- **Launch selected scene** accepts only a shortname from AirStack's checked-in scene
  catalog. It restarts the inner Isaac and robot services, persists the selected scene,
  clears prior camera readiness and never restarts/submits the OSMO workflow or warm
  worker. Non-Office scenes currently disable live proposals.

- **Refresh camera** subscribes briefly and read-only to the current front-camera image,
  canonical MAVROS state and canonical odometry. Refresh twice: the second capture must
  prove that the Isaac image timestamp advanced. It rejects a non-`camera_left` image,
  disconnected MAVROS, non-`map → base_link` odometry, malformed values, stale capture,
  or image/metadata checksum mismatch.
- **Save goal** preserves a new `input.json`, live `input.png`,
  `observation.json` and `request.json` under
  `.rrm-artifacts/command-requests/<request-id>/`.
  Each request has new task/state identifiers, while its frozen observation times and
  declared capabilities stay unchanged. The existing inference CLI can load the saved
  input.
- **Ask RRM for a proposed action** requires that exact saved mission record, a fresh
  checksum-bound observation, the matching Office scene, and the configured private
  worker. It obtains separate live entity evidence, then records and displays only an
  accepted first semantic action. It cannot create an ActionClient or authorize
  motion.
- **Open Foxglove** opens the Foxglove web app. Keep the existing Mac forwarding
  active and connect to `ws://127.0.0.1:8766` as in the Office runbook.

Historical PSC import/submission and exact-proposal supervisor endpoints remain in the
server for evidence compatibility and tests, but the current mission page does not
render their retired manual controls. The browser accepts no PSC password, SSH private
key or Hugging Face token. PSC is not the current per-action control loop.

The static scene catalog is labelled separately from the live observation; a camera
image alone does not establish that a marker is currently visible or that a new plan is
safe to execute. The current live-goal interface has no approval or execution control.
In the retained historical supervisor, candidate approval binds the exact proposal
SHA-256 and still does not automatically dispatch it.

The separate explicit mission runner now supplies that composition boundary outside
the GUI. With an operator authorization, `--execute --simulator-only`, and
`--feasibility-provider scripts/airstack_drone_feasibility_provider.py`, it performs
dynamic corridor/controller/resource checks, consumes one short-lived admission,
dispatches through the existing public task action, independently verifies the effect,
and then re-observes/replans. The bundled profile requires an already-airborne,
controlled, nearly stationary vehicle and one straight `NAVIGATE` waypoint.

## Retained historical exact-proposal supervisor

The current page does not expose this supervisor. Its retained server boundary binds an
imported proposal, public task action, exact goal payload and canonical SHA-256 digest.
The explicit historical approval route writes `admission.json` before launching the existing
`airstack_drone_dispatch.py --execute --verify-observation` adapter. A changed digest,
duplicate active execution, prior decision or latched stop fails closed. Approval is
for the bound historical PSC proposal only; it does not approve newly saved
requests, and no approval happens automatically.

In that boundary, reject records `NOT_DISPATCHED` and makes the instance terminal for
the proposal. STOP/HOLD first closes admission and increments the stop generation,
then interrupts an active dispatcher. Once its ROS goal has been accepted, the
dispatcher handles that interrupt by requesting public action cancellation and writing
a bounded cancellation record. Cancellation delivery, acknowledgement, or process
exit alone remains `STOPPED_UNCONFIRMED`. When the action server acknowledges the
cancel and three new odometry samples show speed at or below 0.10 m/s, the console may
instead report `STOPPED_VERIFIED`. This proves the measured motion stopped; it does
not make a broader collision-free or hardware-safe claim.

LAND NOW is a separate retained operator safety override; it does not wait for RRM inference
or normal proposal approval. It writes a typed LAND proposal and override record, blocks
new RRM commands, and uses the same public ActionClient-only dispatcher. If another
command is active, landing is launched only after that adapter records cancellation
acknowledgement. Missing acknowledgement leaves `LAND_BLOCKED_UNCONFIRMED` instead of
running two actions concurrently. STOP / HOLD remains able to cancel an active or
pending landing. Landing completion still requires fresh near-ground odometry and a
connected, disarmed vehicle before the existing verifier returns VERIFIED.

After any stop or prior execution survives a console restart, the retained safe-state
reconciliation route runs a read-only vehicle observer. Normal approval reopens only when the
drone is connected, disarmed, within 0.30 m of ground, moving no faster than 0.10 m/s,
and at least three fresh `map` to `base_link` odometry samples agree. The complete
observation is written before the new attempt opens; old admission and outcome files
are retained. An airborne, armed, disconnected, moving, stale, or wrong-frame reading
fails closed.

There is deliberately no generic Pause or Resume. STOP / HOLD cancels the command;
continuation should begin with fresh observations and a newly reviewed RRM plan rather
than resuming a potentially stale trajectory.

Execution evidence is stored under
`.rrm-artifacts/command-requests/execution/<dispatch-id>/`, including the exact
proposal, admission record, dispatcher log, outcome when available and stop records.
These artifacts are ephemeral and gitignored.

The current **Launch selected scene** control restarts simulator/robot services and
drops WebRTC temporarily. It is not C08, not an emergency stop, and not evidence of a
safe physical state.

This is a narrow single-process demonstration boundary, not the complete distributed
C06/C08 design in `interfaces.md`: it does not provide authenticated multi-user
authority or durable cross-host deduplication. Its grounded reconciliation and
motion-stopped checks are narrow single-host evidence gates, not an independent safety
controller or a general `safe_confirmed` guarantee. Those limits must remain explicit
in results.

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

## Live proposal and feasibility validation, 2026-09-20

The GUI live-goal proposal path, scene catalog gate, full injected mission
composition, dynamic feasibility contract, and AirStack read-only provider are covered
by the complete **148-test** suite. A live observer probe found fresh required channels
and Navigate/Land endpoints but correctly blocked the current grounded, disarmed,
no-control, planner-stuck state; measured minimum corridor clearance was approximately
0.391 m against a 0.4 m threshold. No action goal or vehicle command was sent.
