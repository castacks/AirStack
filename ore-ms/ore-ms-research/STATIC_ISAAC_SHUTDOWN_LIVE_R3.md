# Static Isaac shutdown contract live validation R3

27 November 2026 UTC. **PASS.** The single isolated run saved a native paired frame before shutdown. The pre-close hashes matched the retained artifacts. Kit logged shutdown and Docker attested to a clean exit. The supervisor wrote `COMPLETED.json`, with `native_shutdown_return: NOT_OBSERVED` and `shutdown_evidence: kit_fast_shutdown_log`. The independent review confirmed both classifications. [Machine-readable results](static-isaac-shutdown-live-r3/verification.json)

## Execution and isolation

The checkout matched `399fa221861e8fed9d6699c74022c6f698e1cca4` on `ore-ms-orb-development`. All 46 CPU tests passed. All six staged inputs matched the commit byte for byte, including the R1 USD fixture pinned by `a6664a3bd426d28a6c958b046901e3b3fd9656909700ad3dfb9bb43e198fd6f7`. The cached Isaac Sim 5.1.0 image used manifest `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697` and config `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`.

The stopped diagnostic container passed structured inspection with exactly the two approved scratch binds, no network, private IPC, and disabled privileged mode. The host issued one start. Docker recorded 102.33 seconds elapsed within the 150-second supervisor timeout. The operational before and after snapshots matched byte for byte for the simulator, GCS, and robot containers, including IDs, image IDs, start times, restart counts, and status. The isolated container was removed after its exited state was captured. [Runbook and qualified launcher](STATIC_ISAAC_ANNOTATOR_READINESS_R3.md#one-frame-procedure-for-separate-authorization)

## Acquisition and shutdown evidence

| Gate | Verified observation |
| --- | --- |
| Bounded native acquisition | Attempt 1 reported both payloads missing. Attempt 2 delivered the pair. The worker used two of eight allowed attempts. |
| Frame association and simulation time | Both native writer packets carried Replicator `ReferenceTime` `[6993, 30]`. Isaac resolved the common reference to `0.30000000447034836 s`. The readiness journal and frame metadata agree. |
| Raw artifact validity | Both arrays are 33 × 25 and 3,300 bytes, with little-endian float32 depth and little-endian uint32 IDs. Independent decoding verified 64 pixels with finite positive depth and the target instance ID. Camera intrinsics and pose metadata passed structural checks. |
| Persistence before shutdown | Lifecycle lines 11–14 record writing and validation completing. Line 15 seals the two arrays, frame metadata and readiness journal with SHA-256. Line 16 records the durable shutdown request. File timestamps fall within the acquisition interval, and the four retained hashes match the seal. |
| Native fast shutdown | The launch log records `fastShutdown=True` and line 486 reports `Simulation App Shutting Down`. The journal ends after the request, with no Python return record. |
| Host exit and completion | Two host Docker inspections agree on the diagnostic container identity and exited state. Exit code was `0`, with zero restarts, no OOM kill and no Docker error. The supervisor returned `0`. The completion marker contains all four evidence states and the verified exit-receipt hash. |

The independent review decoded the native buffers and recomputed their pre-close and completion hashes without using recorder validation code. The two raw arrays are byte-identical to those from the previous stationary R3 retry. This run has its own native reference, frame metadata, and acquisition timestamps in a fresh output directory. The previous failed shutdown verdict remains unchanged. No geometry accuracy, ROS TF, or two-second timing claim follows from this diagnostic. [Prior R3 retry](STATIC_ISAAC_ANNOTATOR_RETRY_R3.md)

## Retained evidence and next step

The raw evidence directory is `/root/ore-ms-artifacts/static-isaac-shutdown-live-r3.I0Y8EvuU`. Key files include `run/capture/frame.json`, `depth.f32le` and `instances.u32le` under `run/capture/`, both capture journals, `run/child.log`, `run/container-exit.json`, `run/COMPLETED.json`, and the independent review script and results. All 39 files passed `SHA256SUMS.txt`, with manifest digest `e093f2c73754e6f78b0529558c3039e85f8af377640280e4be5009344c7fdf0f`. R1, R2, previous R3 evidence and frozen M05–M08 package and plan checksums verified before and after.

The next separately authorized qualification should restore the 5 m and 15 m depth controls, then collect the 10 m center and off-axis samples at two verified simulation-clock ticks exactly two seconds apart. Require exact-time transforms and the existing independent receipt checks, with the now-qualified shutdown contract. No geometry run, M12 physical work or M13 integration was started. [Existing static geometry receipt protocol](STATIC_ISAAC_GEOMETRY_RECEIPT_R1.md)
