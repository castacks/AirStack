# M00-C PX4 startup root-cause assessment

The **confirmed immediate mechanism** is a missed dependency window followed by the intended fail-closed latch: the guarded robot interface starts before the PX4 process/connection exists, its startup deadline is reached without verified initialization, and later connected/safe reports do not clear that failed initialization. The architecture/launch sequencing gap is strongly supported for this episode. The dominant underlying cause of simulator pre-PX4 delay (asset/scene setup, compute scheduling, or another subphase) is **not established**. No active counterfactual test ran.

This conclusion is based on original records read during M00-C, not the M00-B narrative: `RRM/evidence/M00-C/startup-source-records.json` retains the full original robot-interface and MAVROS ROS log snapshots, filtered timestamped Docker startup lines, and current same-boot process start ticks. Previously cited lines match the original files. The MAVROS log grew after M00-B; its earlier file hash is not expected to match the larger current file. Original log excerpts themselves match exactly. The interface log remains the same 106-byte file/hash.

## Verified startup sequence

`M00C_STARTUP_TIMELINE.csv` labels every observed clock/domain, evidence locator and missing observation. It is an event inventory with separate clocks, not a falsely merged total ordering. Docker receipt timestamps in this paragraph are UTC:

| Event | Observed record |
|---|---|
| Isaac/robot containers |18:41:25.413772267 /18:41:26.092998749, Docker State.StartedAt |
| Isaac application `app ready` |18:42:04.530384978, before vehicle graph/PX4 |
| Simulation App Startup Complete |18:42:09.695796205 |
| Robot interface/MAVROS launch reports |18:43:10.818346916 /18:43:10.818344703 |
| DDS routers / FCU URL |Router running18:43:10.829–.833; URL configured18:43:10.863, companion UDP14541→14581 |
| Guard terminal failure |Producer epoch1795805000.934741647 (18:43:20.934741647 UTC label); Docker receipt18:43:20.937541747 |
| Vehicle PX4 graph created |18:43:23.924567918 |
| PX4 startup / simulator TCP connection |18:43:32.436192713 /18:43:32.707706671, actual connection TCP4561 |
| Pegasus first received heartbeat |18:43:38.243360708; distinct from companion MAVROS connection |
| Onboard companion UDP initialized |18:43:38.989656007, local14581 remote14541 |
| PX4 startup script returns |18:43:39.112998572 |
| MAVROS first recorded connected heartbeat |Producer epoch1795805020.210903826; Docker receipt18:43:40.212123920 |
| Commander `Ready for takeoff!` |18:43:42.813218281; does not establish AirStack guard readiness |

Independent Decimal arithmetic yields **19.276162179 seconds** between the two original producer log timestamps. This verifies M00-B's rounded 19.28s; its float result differs only below the useful measurement precision. The value is a log interval, **not** heartbeat transport latency, timer uncertainty or physical response time. No historical clock-step/synchronization calibration was retained; displayed nanoseconds do not imply nanosecond accuracy.

Kernel `/proc` evidence independently places PX4 process start **21.62 seconds after interface process start**, using shared boot identity and100 ticks/s. It is not converted to UTC from today's uptime. Tick quantization is10 ms per timestamp (conservative delta uncertainty less than20 ms); process existence cannot prove when it became ready. These observations support the dependency miss without requiring that Docker start times equal process start times.

## Exact deadline origin and state mechanism

Source evidence: `RRM/evidence/M00-C/source-locations.json` and SHA256 inventory identify `robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp`:

- `using Clock=std::chrono::steady_clock`; `const Clock::time_point started_=Clock::now()` is a non-static member initializer, executed during **guard object construction before its constructor body**. It is neither node/container start nor simulation clock, FCU connection or first safe-state arrival.
- `actuation_startup_timeout_s` was observed 10.0 by M00-B GetParameters and fresh M00-C readback. Source validates it in(0,60]. A200 ms wall timer compares elapsed steady time against it. Timer scheduling/query contention can delay the error; the log is not the exact deadline edge.
- The exact numeric steady origin and its realtime mapping were **not instrumented**. Subtracting10 s from the error log would fabricate an observed construction timestamp. The source semantic origin is established; its precise runtime timestamp remains missing.
- On terminal deadline the source sets attempted=true, leaves initialized=false and emits STARTUP_DEADLINE. Later ticks do not retry initialization. `ready()` requires initialized plus a good readback≤0.5 s old, and `admit()` immediately rejects uninitialized state. This explains false readiness despite the present child double 1.0 and later connected/disarmed/ON_GROUND telemetry. Guard wall-clock timing does not become simulation time just because use_sim_time=true.

Current M00-C passive observation:61/61 readiness samples false, status STARTUP_DEADLINE; child double 1.0. Full sample records and GetParameters are in `RRM/evidence/M00-C/passive-readiness.json`. No SetParameters, restart, mode, goal or arming request occurred. Source/binary mapping is consistent with the M00-B loaded plugin diagnostic strings and hashes, but no reproducible-build or in-memory executable attestation was performed. The source explanation is conditional on that documented provenance limit.

## Cause levels and competing explanations

The [hypothesis matrix](M00C_HYPOTHESIS_MATRIX.csv) assesses ten hypotheses with supporting/contradictory evidence, missing measurements, method, confidence and action links. Current configuration mismatches remain reproduction blockers, but a wrong current scaling/profile is not supported as the cause of this STARTUP_DEADLINE. The observed terminal reason is not CONFIG_REJECTED or STARTUP_NOT_VERIFIED. No guard clock-domain defect or undesired automatic recovery defect is demonstrated.

`pegasus_app.py` source defers actual vehicle/backends to first Play computation, and the backend starts PX4 there. An early `app ready` therefore does not establish FCU readiness. The repository's OSMO reconciliation compares only robot versus simulator **container** StartedAt values: here robot starts 0.679226482s later, so that predicate would not select it, even though its interface starts before PX4. This is a confirmed insufficiency of that predicate as a PX4-readiness proxy **if this source is the deployed orchestrator**. Actual workflow/outer-entrypoint deployment identity is unavailable, so it is not proof that a particular OSMO service executed this source or caused the episode.

MAVLink TCP connected and later UDP/heartbeat phases are observed; simulator poll timeouts also exist. They can contribute after PX4 launch but cannot alone explain why the guard expired before PX4 existed. Historical DDS callback receipts, parameter transaction attempts and resource traces were not retained, so their contributions cannot be excluded or quantified. No conclusion is drawn from graphics warnings alone.

## Requirement and closure limits

The10 s timeout,0.5 s freshness and200 ms timer are **observed implementation parameters**, not newly accepted F00 requirements. The supplied E4 procedure requires readiness true for ≥30 s before a run; M00-C does not apply that as a substitute guard-start deadline. Original F00/SRS startup timing allocation and approvals remain unavailable (M00-B frozen records FR-07/10/14/19 and M00-B01). Their absence prevents approving a new timeout/reference solely to avoid failure.

Proposed remedies and verification are in [the corrective plan](M00C_CORRECTIVE_ACTION_PLAN.md) and `RRM/evidence/M00-C/M00C_CORRECTIVE_ACTIONS.csv`. No remedy was implemented or tested on the live system. Operational recovery is not root-cause/qualification closure. G0 remains **BLOCKED BY EXTERNAL INPUT**, with engineering blockers still open.
