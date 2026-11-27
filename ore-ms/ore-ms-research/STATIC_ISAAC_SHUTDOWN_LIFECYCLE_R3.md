# Static Isaac shutdown lifecycle R3

27 November 2026 UTC. **PASS offline.** The R3 recorder now distinguishes a recorded Python return from a clean, externally verified Kit fast exit. The revised contract needs one separately authorized live validation. No simulator was launched for this work.

## Retained R3 shutdown

The retry at `b0f5d8122` had a valid native paired frame. Lifecycle lines 11–12 show artifact writing completed. Line 13 is the durable `shutdown/started` record from immediately before the call to `app.close()`. Child log line 489 reports Kit shutdown, and Docker records a clean exit about 3.48 seconds after the request. The process ended within the existing deadline. The worker had no later return or failure record. The supervisor rejected it with exit 1 and withheld `COMPLETED.json`. The historical result remains **FAIL overall, acquisition PASS**. [R3 retry report and evidence paths](STATIC_ISAAC_ANNOTATOR_RETRY_R3.md) · [Retained retry gate results](static-isaac-annotator-retry-r3/verification.json)

The launch log shows `--/app/fastShutdown=True`. NVIDIA documents the enabled default in Isaac Sim 5.1 as allowing process exit directly. The tagged Python implementation logs entry into shutdown, disables Kit logging and calls `shutdown_and_release_framework()`. The retained pinned-image excerpt follows the same sequence. AirStack's Pegasus examples stop their timeline and use `SimulationApp.close()` as the last statement of their application loop. They supply no post-close witness. Python resumption is therefore unsuitable as a universal requirement for the pinned configuration. [Isaac Sim 5.1 launcher settings](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.simulation_app/docs/index.html#isaacsim.simulation_app.SimulationApp) · [Tagged shutdown implementation](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py#L763) · [Pegasus shutdown example](../../simulation/isaac-sim/extensions/PegasusSimulator/examples/0_template_app.py#L99)

Native fast exit is consistent with the observed sequence and the documented configuration. The logs lack a native stack trace or exit-origin record, so they cannot establish the exact terminating instruction. Docker provides evidence against a timeout or an OOM kill in this run. The intact, synced request and earlier artifact records provide no indication of a torn journal write. A return checkpoint would be attempted only if Python regained control. No shutdown exception or signal was retained, and the shutdown source suppresses selected cleanup exceptions and disables logging before framework release. The evidence cannot completely exclude a masked internal event.

## Completion contract

| Evidence state | Required observations |
| --- | --- |
| `ACQUISITION_COMPLETE` | The worker validates the persisted arrays, metadata and readiness agreement using the existing frame checks. It durably records their SHA-256 hashes before requesting shutdown. |
| `SHUTDOWN_REQUESTED` | The lifecycle contains the synced shutdown-start record after acquisition validation. The only allowed subsequent lifecycle record is a successful native return. |
| `PROCESS_TERMINATED` | The external supervisor observes a clean attached command exit within its timeout. The launcher records the matching container's Docker state in host-only `run/container-exit.json` before cleanup. Isaac mode requires an exited, nonrunning container with status 0, valid start and finish times, zero restarts, no OOM kill and no Docker error. |
| `EVIDENCE_VERIFIED` | The supervisor checks the final lifecycle sequence, the host exit receipt, the persisted frame and readiness records, and agreement with the worker's pre-shutdown hashes. It hashes those artifacts and the child log before atomically publishing `COMPLETED.json`. |

A recorded successful Python return satisfies the native shutdown witness and is reported as `OBSERVED`. With no return checkpoint, the supervisor requires both enabled fast shutdown and the pinned Kit shutdown log line. That path reports the native return as `NOT_OBSERVED` and includes the host exit receipt and hash. A clean exit without the required sequence and native witness is insufficient. This contract verifies observed clean termination and evidence integrity. It cannot attest to every internal native cleanup hook. [R3 worker](static-isaac-annotator-r3/capture_one_r3.py) · [R3 supervisor](static-isaac-annotator-r3/supervise_r3.py)

The updated launcher records Docker state before the supervisor can certify completion. The mount gate, 150-second timeout, eight-render-attempt bound, native reference comparison, simulation-time lookup and isolated container configuration remain in force. The original R2 supervisor still requires its original complete shutdown journal. [Updated runbook](STATIC_ISAAC_ANNOTATOR_READINESS_R3.md#one-frame-procedure-for-separate-authorization)

## Offline verification and live gate

All 36 R3 and 10 R2 CPU tests passed. The archived native frame passed the unchanged acquisition checks offline. The archived run lacks the new checkpoint and exit receipt and remains ineligible for completion under the revised contract. The CPU suites exercised normal return, witnessed fast termination, an uncorroborated exit, nonzero exit, missing and altered artifacts, missing shutdown request, truncated journal, shutdown timeout, supervisor interruption, early termination, invalid exit evidence and an exception in shutdown. Mock launcher coverage verifies that validated mounts precede start and that the host exit receipt is retained. The R1, R2 and both R3 evidence manifests and frozen M05–M08 package and plan checksums were independently rechecked. No Docker or Isaac execution was needed for the tests. [Deterministic regression suite](static-isaac-annotator-r3/test_readiness.py)

```bash
research=ore-ms/ore-ms-research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$research/static-isaac-recorder-r2" \
  python3 -m unittest discover -v -s "$research/static-isaac-annotator-r3"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -v -s "$research/static-isaac-recorder-r2"
python3 ore-ms/maintenance/check_docs.py
```

A separately authorized one-frame run must verify the operator-visible outcome of the new gate on the pinned image. It must preserve a valid native pair, the pre-close and post-exit hashes, the durable request, and the externally attested exit outcome. The recorded native-return field must reflect the observed branch. Geometry controls remain outside this validation.
