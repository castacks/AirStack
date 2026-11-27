# Static Isaac R4 two-step live clock qualification

27 November 2026 UTC. **FAIL. Neither of the two required clock epochs qualified.** The first render advanced the native graph clock and delivered two ROS messages with the exact expected timestamp. The independently sampled physics-step index remained at 3, so the probe rejected the clock association and stopped before step two. [Machine-readable gates](static-isaac-geometry-r4/clock-epochs-live-verification.json)

## Preflight and execution

One isolated clock-only attempt ran at commit `c4b5fb5f20fb422179181061b858508283caeb9a` on `ore-ms-orb-development`. All 84 CPU tests passed, including 10 R2, 36 R3, 31 R4 and seven frozen receipt checks. The launch checked the local Isaac image digest and image configuration, the fixture and executable source hashes, and the two structured scratch mounts. The container used a private IPC namespace, no network attachment and only the designated GPU. The host attach limit was 150 seconds with a 15-second kill grace. The isolated execution used the ROS 2 Jazzy bridge, Fast DDS, domain 91 and localhost discovery on `/clock`. The probe allowed two bounded Replicator calls, each with a nominal 0.1-second delta and a three-second subscriber wait after render return. [Clock-only probe](static-isaac-geometry-r4/clock_probe_r4.py) · [Offline semantics qualification](STATIC_ISAAC_CLOCK_SEMANTICS_R4.md)

## Independent review

| Required observation | Step 1 | Step 2 |
| --- | --- | --- |
| Independently observed physics-step advancement | **FAIL.** Index 3 before render, 3 after render, 3 after subscriber wait. | Not reached |
| Enabled graph, execution evidence and native clock progression | **PASS.** Tick, Time and Publish compute counts each rose from 0 to 6,997. Native graph time rose from 0 to `0.30000000447034836 s` and held through the wait. | Not reached |
| ROS publisher discovery and QoS | **PASS.** One Clock publisher was discovered with compatible reliable, volatile QoS. | Not reached |
| Exact native-to-wire encoding and callback grouping | **PASS.** The publisher input equaled the post-render graph time. Truncating seconds × 10⁹ independently reproduced `300000004 ns`. Both raw deliveries matched it. The bounded queue check settled on the third executor poll. | Not reached |
| Accepted clock epoch | **FAIL.** The probe recorded `CLOCK_ASSOCIATION_UNVERIFIED` in `subscriber_wait` and accepted no epoch. | Not reached |

The independent review recomputed the wire stamp from the captured publisher input and compared it against both raw ROS messages. The duplicates represent two delivered callbacks with one timestamp. The graph counts establish node evaluation, and the records leave the number of actual physics steps and individual publishes unresolved. The unchanged index establishes that this accessor did not witness advancement across the tested render. The evidence leaves open whether physics failed to advance or the sampled accessor failed to reflect advancement on this path.

The host recorded isolated container exit code 0 with no Docker error or OOM. The lifecycle journal recorded the shutdown request and the Kit log recorded `Simulation App Shutting Down`. A Python return from native shutdown was not observed. The clean exit and Kit shutdown evidence establish the isolated process termination, while the recorded acquisition failure prevented clock qualification. Neither `clock-check.json` nor `COMPLETED.json` was produced. All three operational container snapshots matched before and after, including IDs, start times and restart counts. All checked prior evidence and frozen research hashes matched. The geometry recorder was not run, and its exact `2,000,000,000 ns` requirement remains unchanged. The single live attempt ended at step one.

## Evidence and next check

Raw evidence is retained locally at `/root/ore-ms-artifacts/static-isaac-clock-epochs-live-r4.YB2kBYRM/`. From that directory, `sha256sum -c SHA256SUMS.txt` verifies all 42 manifest entries. The committed gate JSON pins the manifest digest. The raw callback stream is `run/capture/clock-raw.jsonl`. The before/render/after snapshots and executor events are in `run/capture/clock-diagnostic.jsonl`. The worker journal, Kit log and Docker exit record are `run/capture/phases.jsonl`, `run/child.log` and `run/container-exit.json`. The independent reviewer and its result are retained alongside the snapshots and test logs. [Evidence paths and hashes](static-isaac-geometry-r4/clock-epochs-live-verification.json#L1)

The smallest next task is an offline trace of how the pinned Replicator call drives physics callbacks and updates `World.current_time_step_index`. Identify which independent counter can witness the actual physics boundary, and keep both the existing exact native-to-wire gate and the requirement for two strictly advancing epochs. Geometry integration remains gated on clock-only qualification.
