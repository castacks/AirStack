# PAUSE acceptance escalation development evidence

2026-11-17, `ore_proj`, starting at `e089e2b93` with the source changes pinned in the
[compact report](../pause-escalation-20261117.json). The actual C++ monitor ran on
isolated DDS195 against a fake mode server. No simulator endpoint or flight was used.

All 28 candidate contract checks pass. The package passes nine of nine CTest checks,
including existing stale-source/service-fault and operator-preemption contracts;
aggregate 20 tests, zero failures/errors, two skips. An invalid negative escalation
budget was rejected at startup. Baseline source at the starting revision publishes
fault status but lacks the new escalation endpoints; its separate trace and exact
harness bytes are retained alongside the final candidate trace/harness.

| Episode | Test acceptance budget | First observed escalation | Resolution duration | Miss history retained |
| --- | --- | --- | --- | --- |
| Service absent, then rejects/timeouts, then accepts | 0.6 s | 0.649959 s | 1.50058 s | Yes |
| Superseded operator remains pending despite PAUSE reply | 0.6 s | 0.649963 s | 1.10005 s | Yes |
| New explicit PAUSE with available accepting controller | 0.6 s | None | 0.0386104 s | No |

These are monitor-reported steady durations from one observed development run,
not hard timing guarantees. Production default budget is 3.0 s. The JSON snapshot
identifies the process and local episode; separate Bool streams are not an atomic
transaction. False escalation can mean still within budget and never proves safety.
A late positive reply resolves the current communication/reconciliation episode,
not physical stopping. Every snapshot reports physical stop UNVERIFIED and scope
monitor_local_requests_only. Restart does not prove remote cancellation or authorize
old generations. No automatic LAND/disarm/restart or recovery consumer was wired.

The live installed monitor binary retained its original SHA256; the candidate
used a separate build/install. Integer-mode clients elsewhere remain independent
writers: controller authority fencing is still open. No historical flight was
requalified by this source-only increment.

`events.json` and `summary.json` are final candidate outputs. The `baseline-*`
files record absence of the new telemetry, not a failed physical stop. Harnesses
are source snapshots for isolated integration tests. The offline audit starts no
ROS node and requires only Python's standard library:

```bash
python docs/uav-results/pause-escalation-20261117/audit.py
```
