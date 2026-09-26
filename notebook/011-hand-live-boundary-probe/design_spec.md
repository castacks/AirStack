# Design Spec: Live Action Boundary Probe

> Notebook entry: `notebook/011-hand-live-boundary-probe/` · Date started: 2026-09-26 00:17 UTC
>
> Last updated: 2026-09-26 · Branch: `ore_proj` · Starting commit: `5e8cbdd4`
>
> **Status: `WIP`**

## 1. Problem Context

Gate 5 requires C06 single-use admission and C09 append-before-dispatch evidence tested at the final adapter boundary with exactly one bounded simulator action.

The CPU gateway capabilities are implemented, while live Gate 4 evidence remains open.
Gate 5 must test the `HandExecutionBoundary` operating end-to-end with the
`IsaacHandAdapter` in the live Isaac environment.

## 2. Proposed Implementation

### 2.1 Boundary lifecycle probe — `DONE`

Create `simulation/hand_live_boundary_probe.py` using the live Isaac container.

The probe will:
1. Initialize the `SimulationApp` and `KukaAllegro` similarly to previous live probes.
2. Ensure joints are placed into valid start positions (midpoints).
3. Instantiate the `IsaacHandAdapter`.
4. Instantiate the `HandAuthorityVerifier` and `HandExecutionBoundary`.
5. Establish five measured safe samples and perform an authenticated boundary reset.
6. Build the complete immutable `DispatchContext`, exact dispatch authorization scope,
   and single-use signed calibration grant.
7. Dispatch once, apply the target, stop through the boundary, apply a hold, establish
   five fresh safe samples, and call `confirm_stopped()`.
8. Validate the exact ordered boundary and adapter ledger evidence.

The current articulation-only fixture explicitly fails `qualified_scene_equivalent`;
it must not be used to close Gate 5 until the qualified tabletop fixture, controller
gains, and measured object velocities are recreated and verified.

### 2.2 Qualified tabletop fixture parity — `DONE`

Use the shared qualified fixture from notebook 010. Gate 5 must consume the same live
scene-equivalence attestation and measured block-velocity callback as Gate 4; it cannot
substitute a boundary-only mock or constant safe-state input.

The boundary probe now refuses dispatch unless the shared fixture attestation passes,
uses measured block velocities, establishes a bounded initial safe window, records
observed target progress, and waits within a bounded budget for post-hold safe evidence.

### 2.3 Target-environment evidence — `DESIGN/TODO`

Run only after separate explicit hand-command approval. Retain the versioned JSON
report and ledgers, then set this section and the overall status to `DONE` only if the
report has `gate_5_live_probe_passed: true`.

### Affected packages
| Package | Change |
|---------|--------|
| `simulation/hand_live_boundary_probe.py` | New end-to-end Gate 5 probe. |
| `simulation/hand_live_qualified_fixture.py` | Shared qualified-scene contract and builder. |
| `tests/test_hand_live_qualified_fixture.py` | CPU-only fixture contract coverage. |
| `docs/scrum-8/hand-embodiment-decision.md` | Gate 5 evidence status. |

## 3. Test Plan

### (a) CPU-only evidence-validator tests

Import the probe without Isaac and test every required boundary lifecycle and ledger
condition. Pass requires the focused tests and dependency-light RRM suite to succeed.

### (b) Static contract checks

Compile the script and verify its construction against the current boundary APIs.
Require `git diff --check` to report no whitespace errors.

### (c) Approved live Gate 5 run

Run the action-applying probe in the target Isaac container only with explicit approval.
Pass requires authorized reset and dispatch, target and hold tick outcomes, fresh safe
evidence, stopped confirmation, complete ordered C06/C08/C09 ledger sequences, and
verified equivalence to the qualified scene.

## 4. Current Findings

The original script did not match the current boundary contracts and had no retained
live report. It has been repaired to use the actual authorization, reset, dispatch,
stop, safe-state, journal, and shared qualified-fixture APIs. CPU/static validation
passes. No approved live Gate 5 report has been produced or retained, so Gate 5 remains
open.
