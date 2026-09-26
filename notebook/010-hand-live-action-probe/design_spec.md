# Design Spec: Live Action-Applying Stop/Hold Probe

> Notebook entry: `notebook/010-hand-live-action-probe/` · Date started: 2026-09-26 00:08 UTC
>
> Last updated: 2026-09-26 · Branch: `ore_proj` · Starting commit: `5e8cbdd4`
>
> **Status: `WIP`**

## 1. Problem Context

Gate 4 has two remaining gaps for the hand gateway:
1. **Independent watchdog deployment** tested under live Isaac Sim load.
2. **Live action-applying stop/hold** tested under live Isaac Sim load.

All previous gateway increments were strictly CPU-only mock tests. We now need a live `SimulationApp` probe that instantiates the gateway with `motion_enabled=True`, dispatches a single valid bounded motion (calibration), and verifies that both the asynchronous `request_stop()` fence and the daemon watchdog thread can independently interrupt live physics stepping and force a safe-state position hold.

## 2. Proposed Implementation

### 2.1 Fail-closed live action probe — `DONE`

Create `simulation/hand_live_action_probe.py` to be executed inside the
`isaac-sim-livestream` container using `/isaac-sim/python.sh`.

The probe will run two scenarios sequentially on a fresh reset:
1. **Live Stop Fence Test:**
   - Load `KukaAllegro` and enable the gateway.
   - Dispatch `ARM_JOINT_CALIBRATION`.
   - Step physics so the action begins.
   - Fire `adapter.request_stop()`.
   - Step physics and verify the gateway transitions to `HOLD_APPLIED` and limits motion.
2. **Live Watchdog Thread Test:**
   - Reset the scene.
   - Dispatch another `ARM_JOINT_CALIBRATION`.
   - Start the watchdog thread with a short interval.
   - Pause physics stepping long enough to make the callback heartbeat stale.
   - Verify the daemon watchdog trips the fence independently.
   - Resume stepping and verify the next tick enforces the hold.

The report records exact artifact hashes, profile mismatches, tick outcomes,
stop-to-hold timing, measured safe-state evidence, ordered ledger events, and
fail-closed validation errors. It refuses to overwrite any report or ledger. The
current articulation-only fixture explicitly fails `qualified_scene_equivalent`; it
must not be used to close Gate 4 until the qualified tabletop fixture, controller gains,
and measured object velocities are recreated and verified.

### 2.2 Qualified tabletop fixture parity — `DONE`

Extract an import-safe shared fixture contract and an Isaac-only builder used by both
live probes. The contract must bind the exact asset URL, physics step, seed, scene
entities, joint defaults, controller gains, safe-state thresholds, and 23-joint profile
from the qualified probe. The runtime builder must create both dynamic blocks and feed
their measured linear velocities into the adapter. Scene equivalence is true only when
all contract and live readback checks pass.

Implemented with pre-dispatch checks for the top-level asset digest, articulation/profile,
entity paths, controller gain readback, and measured dynamic-block velocity channels.
Both live probes use bounded initial and post-stop settling loops and require observed
progress toward the commanded target.

### 2.3 Target-environment evidence — `DESIGN/TODO`

Run only after separate explicit hand-command approval. Retain the versioned JSON
report and ledgers, then set this section and the overall status to `DONE` only if the
report has `gate_4_live_probe_passed: true`.

### Affected packages
| Package | Change |
|---------|--------|
| `simulation/hand_live_action_probe.py` | New live probe. |
| `simulation/hand_live_qualified_fixture.py` | Shared qualified-scene contract and builder. |
| `tests/test_hand_live_qualified_fixture.py` | CPU-only fixture contract coverage. |
| `docs/scrum-8/hand-embodiment-decision.md` | Gate 4 evidence status. |

## 3. Test Plan

### (a) CPU-only evidence-validator tests

Import the probe without Isaac and exercise every success/failure axis. Pass requires
all validator and fixture-contract tests plus the dependency-light RRM suite to succeed.

### (b) Static checks

Compile both live scripts and require `git diff --check` to report no whitespace errors.

### (c) Approved live Gate 4 run

Run the action-applying probe in the target Isaac container only with explicit approval.
Pass requires exact profile parity, `TARGET_APPLIED`, bounded `HOLD_APPLIED`, five safe
samples, watchdog `TICK_STALE` fencing followed by hold, all required ledger events, and
verified equivalence to the qualified scene.

## 4. Current Result

Implementation and CPU/static validation pass. The shared fixture has not been executed
inside Isaac, and no approved live Gate 4 report has been produced or retained, so Gate
4 remains open.
