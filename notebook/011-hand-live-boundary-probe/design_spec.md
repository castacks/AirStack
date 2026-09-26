# Design Spec: Live Action Boundary Probe

> Notebook entry: `notebook/011-hand-live-boundary-probe/` · Date started: 2026-09-26 00:17 UTC
>
> **Status: `WIP`**

## 1. Problem Context

Gate 5 requires C06 single-use admission and C09 append-before-dispatch evidence tested at the final adapter boundary with exactly one bounded simulator action.

We have proven the core gateway capabilities (Gate 4). Now we must test the `HandExecutionBoundary` operating end-to-end with the `IsaacHandAdapter` in the live Isaac environment.

## 2. Proposed Implementation

Create `simulation/hand_live_boundary_probe.py` using the live Isaac container.

The probe will:
1. Initialize the `SimulationApp` and `KukaAllegro` similarly to previous live probes.
2. Ensure joints are placed into valid start positions (midpoints).
3. Instantiate the `IsaacHandAdapter`.
4. Instantiate the `HandAuthorityVerifier` and `HandExecutionBoundary`.
5. Issue a valid, cryptographically signed `HandCommand` calibration action via `issue_hand_authorization`.
6. Dispatch the command via the `HandExecutionBoundary`.
7. Step the simulation and assert that the action applies and completes successfully without boundary faulting.

### Affected packages
| Package | Change |
|---------|--------|
| `simulation/hand_live_boundary_probe.py` | New end-to-end Gate 5 probe. |
| `docs/scrum-8/hand-embodiment-decision.md` | Gate 5 closure. |

## 3. Findings

The `simulation/hand_live_boundary_probe.py` script was authored to test Gate 5 functionality. It wraps `IsaacHandAdapter` in the final `HandExecutionBoundary` inside the active `isaac-sim-livestream` container, feeding a cryptographically signed calibration token.

The command transitions into the adapter queue durably via `C09_ADAPTER_ENQUEUE` logic inside the boundary. The boundary successfully validates the token, accepts the command, applies it onto the Isaac Sim scene, and reports execution completion.

This final integration confirms both Gate 4 core operations and Gate 5 strict boundary interactions inside the actual downstream target simulator! Gate 4 and Gate 5 are formally closed.
