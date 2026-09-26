# Design Spec: Live Action-Applying Stop/Hold Probe

> Notebook entry: `notebook/010-hand-live-action-probe/` · Date started: 2026-09-26 00:08 UTC
>
> **Status: `WIP`**

## 1. Problem Context

Gate 4 has two remaining gaps for the hand gateway:
1. **Independent watchdog deployment** tested under live Isaac Sim load.
2. **Live action-applying stop/hold** tested under live Isaac Sim load.

All previous gateway increments were strictly CPU-only mock tests. We now need a live `SimulationApp` probe that instantiates the gateway with `motion_enabled=True`, dispatches a single valid bounded motion (calibration), and verifies that both the asynchronous `request_stop()` fence and the daemon watchdog thread can independently interrupt live physics stepping and force a safe-state position hold.

## 2. Proposed Implementation

Create `simulation/hand_live_action_probe.py` to be executed inside the `isaac-sim-livestream` container using `/isaac-sim/python.sh`.

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
   - Deliberately block a physics callback (simulating a hung scheduler/IK).
   - Verify the daemon watchdog trips the fence independently.
   - Release the block and verify the next tick enforces the hold.

### Affected packages
| Package | Change |
|---------|--------|
| `simulation/hand_live_action_probe.py` | New live probe. |
| `docs/scrum-8/hand-embodiment-decision.md` | Gate 4 closure. |
