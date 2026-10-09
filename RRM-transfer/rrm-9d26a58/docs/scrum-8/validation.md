# Historical implementation and validation ledger

Dated checkpoints from 2026-09-13–26, not a current suite count or readiness report.
Initial source baseline: RRM `9d26a58eb8516b6754c5d12b041cdc789950e047`,
AirStack `ore_proj` / `a6dad8caf54722e5eba3481367e914ce213e6135`.
Later rows describe their then-working source, not a single published revision.

Current conclusions belong in [measured status](end-to-end-status.md).
The [handoff](../../HANDOFF.md#historical-chronology) retains detailed chronological
records; dedicated [GCS](evidence/gcs-takeoff-land-2026-09-14.md),
[inference](evidence/office-inference-46288765.md) and
[flight](evidence/office-flight-46288765-20260918.md) records retain distinct capture
evidence. Test counts below are checkpoint-local, not cumulative live success rates.

## Contract and cognition increments

| Date (UTC) | Increment and recorded result | Claim boundary |
| --- | --- | --- |
| Sep 13 | Initial C02/C03/C06/C08 primitives: 14/14 new tests, legacy oracle 5/5; allocation 11/11 requirements linked | Local admission bookkeeping only; no authenticated transport or physical stop. S01–S10 integrated SIL not run. CPU-only transfer workspace. |
| Sep 17 | C01/C04/C05 task/intent/plan records: 24 total tests; C02 explicit evidence: 29 total | Proposal-only; stale, missing, contradictory or negated UNKNOWN stays UNKNOWN. |
| Sep 17 | Public takeoff/navigate/land adapter: 33 total tests | Dry-run default; explicit execution gate is not a completed C06/C08 supervisor. |
| Sep 17 | Authorized two-goal transport trial: takeoff 2 m at 1 m/s, then land 1 m/s; both server results successful | First staging invocation failed importing rclpy before any goal. Corrected path used public actions only. Post-flight observation supported grounding but did not prove per-goal causality. |
| Sep 17 | Eight-second post-flight observation: final snapshot 8 complete, map pose (-0.502, -0.328, 0.020) m, connected/disarmed; two transient incomplete samples | Outcome-verifier code then added fresh pre/post requirements and VERIFIED/MISMATCH/UNCONFIRMED; no new live goal tested that increment. |
| Sep 17 | Semantic navigation bridge: 40 total tests; simulator teacher pipeline: 44 | Proposal-only. Fresh target evidence and capabilities required; teacher is scoring evidence, not perception. |
| Sep 17 | Cosmos Reason2-8B on PSC H100: first candidate rejected for missing grounded_entities; prompt repair accepted C04/C05; 51 total tests | One text-plus-teacher cognition result, no dispatch or demonstrated visual understanding. Revision a9fae2cf89dc64db96b12860417f0eb403013bb9. |
| Sep 17 | Visual C02/media-provenance and PSC evaluation seam: 56 total tests | Catalog-bound INFERRED claims; actual visual performance still pending paired capture/labels at this checkpoint. |
| Sep 20 | Office single-waypoint feasibility probe: 148 tests; takeoff/multi-waypoint extension: 156 | Live probe blocked: grounded/disarmed/no-control, planner-stuck, ~0.391 m clearance against 0.4 m. No admitted navigation flight. |
| Sep 21 | Direct GUI public-task translation: 169 tests and read-only server/state/map checks | Deterministic aerial grammar, not learned semantic execution. SemanticSearch excluded because only a client existed. |
| Sep 26 | Neutral goal routing, C01 translation and non-executing hand GUI preview: 271/271 tests | Supported red-block preview retained hash-linked records; under-evidenced blue held; unsupported text/unknown target refused. Numeric feasibility, dispatch and simulator action remained false. No general semantic replanning. |

## Recovery and hand prerequisites

| Date (UTC) | Observation | Limitation / retained reference |
| --- | --- | --- |
| Sep 24 | Aerial 1 m takeoff failed after 0.805 m lateral displacement in 1.24 s; recovery pre-sample 3.205 m; final z=0.000725 m, connected/disarmed, RECOVERED_HALT | No retry. Transport/safety regression, not research success. Artifact ID fbb4f424eddb433fa8616cf383246141. |
| Sep 24 | 23-joint hand no-action/tabletop fixtures: three matching resets; manual image-only score 9/9 precision, 9/12 recall; synthetic shadow score 5/9 recall | Neither score is learned visual performance. No grasp/contact/safe-state inference from the teacher's exists/kind/localized facts. |
| Sep 24 | Controller prerequisite: arm error 0.00330 rad, 23 velocity peaks within limits, three matching post-command resets, calibration contact, safe window, independent stop 0.917 s; 206 total tests | Contact peaked at 489 N: channel observation, not stable grasp. Five-gate evaluator permitted contact-trial preparation only. C06/C08/C09 and semantic execution remained incomplete. See hand decision. |
| Sep 25 | Durable restart reconciliation: 16/16 focused, 226 total tests | Mandatory hold, five fresh safe samples, append-before-clear, deduplication; fake articulation only. Reconciliation stays inhibited and needs separate reset. |
| Sep 25 | Signed HMAC-SHA256 scoped authority: 17/17 focused, 227 total | Local shared-key boundary; production identity/key provisioning, rotation and protected/asymmetric verification remain needed. |
| Sep 25 | Gateway liveness: fake hold 0.020 s against 0.050 s deadline; 0.051 s breach; 1000 idle ticks/100 reads, zero actions; 21/21 focused, 231 total | Fake-clock timing, not live physical stop latency. Watchdog faults persist across restart and do not directly call articulation. |
| Sep 25 | Live direct idle heartbeat: 23 profiles matched; 1000 IDLE ticks, zero apply_action; duration median/p95/max 2.14/2.53/18.12 us; 232 total | No physics callback, post-reset stepping, stop or hold. Artifact hand-live-heartbeat-20260925-a/report.json. |
| Sep 25 | Inactive-asset physics callback: 240/240 at declared 120 Hz; zero actions; wall gaps median/p95/max 0.196/0.329/3.116 ms; duration 6.66/14.71/47.68 us; 234 total | Asset inactive throughout; no loaded scheduler/controller/stop qualification. Zero-gravity-only diagnostic failed with 0.279244 rad drift. Accepted artifact hand-live-callback-20260925-d/report.json. |
| Sep 25 | Completed-tick deadline: overlong idle/action tick and clock regression durably latched; 237 total tests | Detection after simulator call returns. No native-call interruption or live stop qualification. |
| Sep 25 | Asynchronous watchdog fence: returned under 50 ms while fake apply_action blocked; one fault persisted after release and hold queued; 15/15 focused, 238 total | Logical in-process fence; cannot interrupt native call, durability waits for return. |
| Sep 25 | Asynchronous stop fence: request_stop returned under 50 ms during blocked fake tick; exact-generation hold deduplication and restart inhibition; 238 total | Logical stop is not physical stop or measured live stop-to-hold. |
| Sep 25 | Independent daemon watchdog: detected blocked fake call without action lock; motion authority false before release, one durable fault, hold after release; 19/19 focused | In-process thread, not out-of-process monitor. Live action-applying stop under Isaac load remains unqualified. |

The Sep 26 assessment also retained earlier learned-navigation timeout, a recovery
altitude excursion of 4.076 m, subsequent Office tracking/landing fixes with 0.005 m
takeoff horizontal displacement, and a failed Warehouse regression with delayed
airborne state after mission exit. At that checkpoint Office baseline flight was
qualified and Warehouse execution paused; later diagnostics must be read in current
status, not inferred from this historical label. Supervisor recovery admission had
deterministic coverage but no new authorized live regression.

## Reproduction and evidence limits

Original CPU verification used robot image `v0.20.8_robot-x86-64_dev`, explicit
`runc`, 2 CPU / 2 GiB limits, source mounted at `/workspace/rrm` and container-local
Pydantic 2.13.5. It downloaded no model/GPU package. With matching source/dependencies:

```sh
cd /workspace/rrm
python3 scripts/oracle_loop.py --suite --trace-dir /evidence/traces
python3 simulation/isaac_backend.py
python3 -m unittest discover -s tests -v
```

The relation-inference command is a geometry self-test, not Isaac integration.
The five-case oracle's T6 starts with a human present, not entry during motion.
Original export bundles recorded source/evidence hashes, container identity,
dependency versions and timestamps; the input ZIP/checksums were retained.
Referenced hand reports reside beneath AirStack's `.rrm-artifacts/`; their mention
does not establish availability in a fresh checkout. Missing raw bundles prevent
recomputing a live aggregate. No Jira completion or external publication is claimed.
