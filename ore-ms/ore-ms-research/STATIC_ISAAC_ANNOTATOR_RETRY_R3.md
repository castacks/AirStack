# Static Isaac annotator diagnostic retry R3

27 November 2026 UTC. **FAIL overall. Native paired acquisition passed.** The isolated worker saved a native depth and instance-ID pair with a common render reference and resolved simulation timestamp. It recorded shutdown starting, but left no durable shutdown-completion record. The container exited `0`. The fail-closed supervisor returned `1`, wrote `FAILED.json`, and withheld `COMPLETED.json`. [Machine-readable gates and artifact hashes](static-isaac-annotator-retry-r3/verification.json)

## Preflight and isolated execution

The run used commit `04a74f8e5ce06d12edfc632cb429c81954259537` on `ore-ms-orb-development`. All 37 offline tests passed. The R3 worker, readiness code and supervisor matched the previously qualified code. All six mounted source and stage files matched the starting Git commit. The R1 USD stage matched its published SHA-256 pin. The cached Isaac Sim 5.1.0 image resolved to manifest `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697` and config `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`. [Structured mount qualification](STATIC_ISAAC_MOUNT_VALIDATION_R3.md)

The structured mount check passed on the stopped container. It recorded exactly the approved read-only input and writable capture binds. The isolated container used no network and private IPC. One start was issued. The worker used two of eight allowed render attempts. Docker recorded about 120 seconds of runtime, within the supervisor's 150-second timeout.

## Native pair and shutdown

| Gate | Evidence |
| --- | --- |
| Attempt 1 | Both writer payloads were missing. The readiness journal records `PAYLOAD_NOT_READY`. |
| Attempt 2 | Both writer packets reported Replicator `ReferenceTime` `[7489, 30]`. Isaac resolved the common native reference to `0.30000000447034836 s` of simulation time. The readiness and frame metadata records agree. |
| Persisted arrays and metadata | The worker wrote one 33 × 25 little-endian float32 axial-depth array and one 33 × 25 little-endian uint32 instance-ID array, 3,300 bytes each, plus camera metadata. Independent decoding identified 64 pixels with finite positive depth and the target instance ID. This was an artifact validity check. No distance controls were run. |
| Save before shutdown | Lifecycle lines 11–12 record the start and completion of artifact writes. Line 13 records shutdown starting. |
| Native shutdown | Worker log line 489 reports app shutdown. No later lifecycle record confirms that `app.close()` returned. The log records `fastShutdown=True` at startup. The exact reason the completion record is absent remains undetermined. |
| Supervisor completion | The supervisor rejected the incomplete lifecycle journal after child exit `0`. It did not issue completion hashes or a completion marker. |

## Evidence and next dependency

Raw evidence is retained in `/root/ore-ms-artifacts/static-isaac-annotator-retry-r3.eroSaJ0U`. It includes `run/capture/depth.f32le`, `run/capture/instances.u32le`, `run/capture/frame.json`, the readiness and lifecycle journals, `run/child.log`, `run/FAILED.json`, and `independent-artifact-review.json`. All 39 retained file hashes passed `SHA256SUMS.txt`. The manifest digest is `77acd5bb06a15a7b0ff210c14b4af68cfdf58eb0232cf4c082b09802c6f61819`.

Operational container IDs, image IDs, start times, restart counts and status matched before and after for all three containers. The isolated container was removed after recording its exited state. The R1, R2 and prior R3 evidence manifests and frozen M05–M08 checksums verified. No further simulator run, geometry controls, ROS TF qualification, two-second capture, or M12/M13 work was undertaken.

The remaining dependency is a shutdown path that demonstrably returns control to the recorder. Review the pinned SDK's shutdown behavior offline and qualify a supported change while keeping the durable completion requirement. Native frame readiness and the existing association gate can be retained for any separately authorized verification.
