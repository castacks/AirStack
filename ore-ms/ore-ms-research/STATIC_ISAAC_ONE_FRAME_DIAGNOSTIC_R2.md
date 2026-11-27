# Static Isaac one-frame diagnostic R2

27 November 2026 UTC. **FAIL.** The single isolated Isaac Sim run initialized the camera, returned from all three render-step calls and failed at the first native frame readout. The instance-ID entry was `None`. No valid depth/ID pair or native timestamp was retained. The container returned `0` during teardown. The supervisor returned `1` and withheld `COMPLETED.json`. [Machine-readable verification](static-isaac-one-frame-r2/verification.json)

## Preflight and execution

The worker and supervisor came from commit `9ced696873b06b67acf28ef2124f0d5e022085ed` on `ore-ms-orb-development`. All ten offline recorder tests passed before launch. The locally cached Isaac Sim 5.1.0 image used manifest `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697` and config `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`. The [R1 USD stage](static-isaac-capture-r1/input/stage.usda) matched the R1 pin `a6664a3bd426d28a6c958b046901e3b3fd9656909700ad3dfb9bb43e198fd6f7`. It configures one stationary camera and one fixed planar target. The worker requested 33 × 25 pixels. [Qualified recorder and runbook](STATIC_ISAAC_RECORDER_DIAGNOSTIC_R2.md)

The host supervisor invoked the retained `launch-once.sh` with a 150 s timeout. The wrapper used `docker create`, checked the stopped container’s image and isolation, and issued a single `docker start --attach`. The container used GPU 0, private IPC, no network, and two scratch bind mounts, with input read-only and output writable. Docker recorded approximately 33.9 s elapsed. The attempt ended before the timeout. The retained commands are in `execute-once.sh` and `launch-once.sh`, with Docker records in `container-created.json` and `container-after.json`.

## Failure diagnosis

| Phase | Direct evidence |
| --- | --- |
| Isaac startup and stage load | Journal lines 1–4 record both completed. The worker log records Isaac startup complete at line 471 and identifies the isolated stage at line 473. |
| Camera creation and attachment | Journal lines 5–6 record the combined initialization and annotator attachment calls returning. |
| Render stepping | Journal lines 7–14 record playback and all three step calls returning. That establishes progress through the calls; delivery of an annotated frame remained unverified. |
| Native readout | Journal line 16 preserves the first exception. At worker line 52, accessing `instance["data"]` raised `TypeError: 'NoneType' object is not subscriptable`. The instance-ID value was absent at that read. Depth validation and native timestamp extraction had yet to run. |
| Artifact serialization | The journal never reached `write_artifacts`. The capture directory contains only `phases.jsonl`, with no depth buffer, instance-ID buffer or camera-frame metadata. |
| Worker shutdown | Journal line 17 records shutdown starting. Worker log line 474 reports app shutdown. There is no `shutdown/completed` event. The Docker record and supervisor both record child exit 0. The supervisor rejected the incomplete journal. |

The earliest supported failure is **native instance-ID readout**. The record does not establish whether the annotator was still warming up or failed to deliver data. The uncompleted shutdown checkpoint is a second observed failure. The exit sequence is consistent with Python failing to regain control from native teardown, but the underlying teardown mechanism was unverified.

**Smallest corrective action.** Prepare an offline worker change that checks and journals native render frame index and time plus the presence, type and shape of both annotators’ output during a bounded wait for a single ready snapshot. Fail explicitly on timeout, preserve the shutdown gate, and test the change with mocks before seeking separate execution authorization.

## Isolation and evidence

Before and after snapshots matched byte for byte for `isaac-sim-livestream`, `airstack-gcs-1` and `airstack-robot-desktop-1`. Each retained its container and image ID, start timestamp, running state and restart count of zero. The diagnostic container was removed after its final state was recorded. The copied inputs, R1 raw evidence and frozen M05–M08 package and plan hashes passed their post-run checks.

The raw evidence directory is `/root/ore-ms-artifacts/static-isaac-one-frame-r2.WzWvL9Pg`. It contains `run/capture/phases.jsonl`, `run/child.log`, `run/FAILED.json`, the pinned stage and worker inputs, the exact launch scripts, both operational snapshots and `SHA256SUMS.txt`. All 28 retained file hashes were independently rechecked. The manifest SHA-256 is `9224f5259b98c05d1fb8542cd45cda6be5d2e098a2684a7bb14ed48bbe501b48`. No ROS clock or TF qualification, full geometry capture or second Isaac attempt was performed.
