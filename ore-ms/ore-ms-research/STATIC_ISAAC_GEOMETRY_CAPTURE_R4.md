# Static Isaac geometry capture R4

27 November 2026 UTC. **FAIL.** The single isolated Isaac run stopped during camera setup, before the 5 m control or any render acquisition. No geometry, timing, TF, or adapter receipt was produced. The supervisor rejected the incomplete sequence despite container exit `0`. [Machine-readable gates and evidence hashes](static-isaac-geometry-r4/verification.json)

## Preflight and execution

The starting checkout matched `0225c0dcc72f63e8897b43d37c1d7468333807d2` on `ore-ms-orb-development`, with Pegasus pinned at `8c7a664095396bc9c4f4609531445d4cf6390fe4`. All 46 R2/R3 CPU tests and seven frozen receipt tests passed. The cached Isaac Sim 5.1.0 image used manifest `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697` and image config `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`. The authored stage and fixture matched the R1 pins. The staged R4 exporter, support code, and entry script were hashed before start and rechecked afterward. [Successful one-frame R3 qualification](STATIC_ISAAC_SHUTDOWN_LIVE_R3.md) · [Frozen receipt contract](STATIC_ISAAC_GEOMETRY_RECEIPT_R1.md)

The stopped diagnostic container passed structured mount inspection. It had exactly the two scratch binds, no network, private IPC, and privileged mode disabled. The host issued one start under a 360-second deadline. Docker recorded approximately 30.54 seconds elapsed. Operational AirStack container IDs, image IDs, start times, restart counts and status matched before and after. The isolated container was removed after its exit state was saved.

## First failure and independent evidence review

The new exporter retained handles to the authored USD camera transform operations before calling Isaac's camera initialization. It attempted to reapply the authored pose to preserve the frozen verifier's precise optical-frame convention. At `capture_r4.py:44`, the call to `op.Set(value)` raised `RuntimeError: Accessed schema on invalid prim`. The operation handle was invalid at that call. The evidence does not identify which setup operation invalidated it. The traceback is on line 4 of the lifecycle journal. [Exact executed exporter](static-isaac-geometry-r4/capture_r4.py)

| Gate | Observed result |
| --- | --- |
| Isaac startup | Completed |
| Stage and camera setup | Failed while reapplying a saved transform operation |
| 5 m and 15 m controls | Unexecuted |
| 10 m issue and target frames, exact two-second clock association, and timestamp-specific TF | Unexecuted |
| Frozen receipt, M08 issue binding and target-time exclusion | Unexecuted on real data. The existing synthetic receipt tests passed. |
| Exit and completion | Kit logged shutdown. Docker recorded exit `0`. The supervisor returned `1` and wrote `FAILED.json`. No completion marker was issued. |

Independent inspection found only the lifecycle journal in the capture directory. There were no native depth or ID arrays, measured surveys, clock records, timestamped transforms, or normalized receipt fields to compare. The recorded shutdown start and host exit provide teardown evidence for the failed run. No acquisition seal or geometry qualification was recorded.

## Evidence and next dependency

The evidence directory is `/root/ore-ms-artifacts/static-isaac-geometry-r4.8fSXMhEw`. It retains the exact stage, fixture, exporter and launch commands, `run/capture/phases.jsonl`, `run/child.log`, `run/container-exit.json`, `run/FAILED.json`, and the independent review. The 41-file manifest verified with digest `b8d4e764bbef631b4cf0b5011aa1fb8d5d39eea7477ac205e3c7cbaf1f4a6216`. The R1–R3 manifests and frozen M05–M08 package and plan checksums passed before and after. The exact R4 source is retained for audit. Its acquisition and receipt paths beyond camera setup remain unverified.

The next dependency is a camera setup that preserves the authored pose without writing through pre-initialization transform handles. Qualify that setup against the pinned USD before another geometry capture is authorized. The exact clock and transform gates remain unchanged. No retry, physical capture, M12 work, or M13 integration was performed.
