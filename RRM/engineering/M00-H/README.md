# M00-H offline guard, adapters and recorder

Offline implementation/compilation/synthetic testing only. No Docker, simulator, ROS nodes, PX4/MAVROS, devices, active network probes or deployment. All writes including test files and build outputs remain under RRM. Historical G and upstream sources are imported/read by reference, never modified.

Run from repository root:

```
python3 -B RRM/engineering/M00-H/check_build_dependencies.py
python3 -B RRM/engineering/M00-H/build_guard.py
python3 -B RRM/engineering/M00-H/run_offline.py
python3 -B RRM/engineering/M00-H/h_preflight.py
python3 -B RRM/engineering/M00-H/validate_milestone.py
```

build_guard and h_preflight intentionally exit2 on BLOCKED. Missing host compiler/CMake/ROS/yaml dependencies prevent guard compile/link; Python standard-library modules compile with compile() and execute synthetic tests. No stub/syntax-only C++ build is presented as production verification. BUILD_CONFIG.json and BUILD_VERIFICATION.md give exact assumptions and gaps. The conditional procedure can compile the unchanged G original-header host when actual local dependencies exist, but never executes its binary or installs anything.

observation_adapters.py implements startup lifecycle, MAVROS connectivity transitions, graph discovery/readiness/loss and recorder emission/health interfaces with immutable producer stamps. ros_binding_contract.json names actual upstream message/topic interfaces and keeps production ROS binding NOT VERIFIED. No adapter commands, arms, changes guard state or starts a process. Graph presence is diagnostic observation, not actual guard/safety readiness. Late connection classification requires the same declared boot/clock; unrelated clocks are UNMEASURABLE. Caller-declared provenance is not source authentication.

reliable_recorder.py implements exclusive binary JSONL segments, complete partial writes, bounded interruption retry, 65536-byte maximum record, per-record fsync and initial directory sync. It latches failure and never overwrites/truncates an existing segment. Recovery creates a separate hash-linked segment with prior loss explicitly retained. Schema2 intentionally differs from G schema1; old bytes are not silently converted. scan_bytes reports structure, not physical durability or trusted origin. See RECORDER_RELIABILITY.md for failure/acknowledgment limits and external supervisor requirements.

guard_reference.py is a source-pinned synthetic model, never the real guard or a production substitute. It corrects G's omitted failed-transaction attempt latch and adds explicit query/unsafe/freshness/readback scenarios. C++ host source remains unchanged; no proof of native scheduling/ABI/service timing/production integration is claimed. GUARD_SEMANTIC_COMPATIBILITY.csv traces actual original source anchors to tests.

run_offline uses only synthetic fixtures, local temporary files and one short-lived offline Python child that exits abruptly to test journal termination detection. It never starts a network or robot service. Dependency register distinguishes host absence, verified source bytes, historical installed/image evidence and unverified future target identities. H preflight is a successor that extends sealed G preflight read-only; G files remain unchanged. Build/test/static/runtime/overall statuses stay separate and execution remains BLOCKED.

Original20 CHKs,29 mappings and historical n=1 timing/schedule retained. G0 BLOCKED, RA01 PENDING, WP01 NOT AUTHORIZED; Tier B target undesignated, execution NOT AUTHORIZED, isolation gate NOT PASSED. Stop after M00-H local commit for PD review; no push.
