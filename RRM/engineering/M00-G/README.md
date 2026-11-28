# M00-G AirStack-native offline simulation prototype

**OFFLINE PROTOTYPE — NO RUNTIME EXECUTION AUTHORIZED.** Compose is deliberately non-runnable: required image identities are unresolved, all profiles are disabled by default and every entrypoint is an explicit unresolved path. Do not resolve these fields as a launch workaround. No Docker invocation, pull, install, build, ROS/PX4/Isaac/MAVROS launch, device read or network probe is part of this package's offline workflow.

Implemented: standalone `config/compose.json` (JSON is a YAML subset), fixed environment template, MAVROS-only launch XML, confined DDS XML intent, dependencies, original-header guard-host C++/ament source, observational Python recorder/schema/auditor, strict config/provenance validator, execution preflight, synthetic startup reference/fixtures/tests and provenance/dependency registers. No root Compose includes, service extends, root stack modifications or generated module/fleet overlays. The AirStack CLI automatically adds root/generated configs; it is intentionally not called here. Future integration must explicitly preserve this closed service graph. Actual upstream CLI, standalone simulator script, MAVROS wrapper and guard paths are recorded in PROVENANCE.json.

Offline commands from repository root (stdlib only):

```
python3 -B RRM/engineering/M00-G/test_offline.py
python3 -B RRM/engineering/M00-G/config_validator.py
python3 -B RRM/engineering/M00-G/preflight.py
python3 -B RRM/engineering/M00-G/validate_milestone.py
```

The validator/preflight intentionally exit2 for the supplied BLOCKED configuration; this is the expected result, not a passing execution gate. Unknown/malformed/unsafe config returns FAIL; fully identified synthetic static config can PASS; runtime properties are NOT VERIFIED; actual overall readiness remains BLOCKED. No CLI accepts an execute flag or resolves overrides from the host environment.

Runnable offline: Python unit tests, supplied-event recorder library, event audit, configuration validator and preflight. Blocked runtime: all four Compose services, guard compilation/install, image initialization, scene/dependency closure, GPU grants, P0/P1 orchestration, adapters and independent durable collection. `startup_reference.py` models only synthetic deadline/latch ordering; it never replaces the real guard. Compose `service_started` is weaker than actual readiness. The wrapper's unchanged class still performs its normal parameter write if ever separately authorized; recorder itself issues no commands.

Configuration intent excludes privileged mode, physical-device mounts/rules, host PID/network/IPC, added caps, host ports, automatic restart, root user and uncontrolled environment/FCU routes. PID mode is omitted to retain the private default; pull_policy never forbids registry fetching; any explicit PID override is rejected. Nonprivileged GPU compatibility, runtime network/device denial and middleware service permissions remain unverified. UID10001, read-only root and noexec tmpfs are proposed restrictions, not demonstrated image compatibility. No GPU device grant is configured.

All configuration, event fixtures and upstream references are hashed in PROVENANCE.json and evidence manifests. Images/digests and scene identities are explicit unresolved values; no digest was invented. The synthetic positive configuration in the test harness uses synthetic.invalid references and a text identity marker, never a real image or USD. Reproduce tests using the committed Python stdlib files and unchanged upstream sources; a future runnable package additionally requires every DEP01–DEP11 item, reviewed manifest revision and separate PD scope. Replacing entrypoints/images/assets changes pinned inputs and requires a new reviewed manifest; no runtime automation is supplied.

See STARTUP_SEQUENCING.md, GUARD_COMPATIBILITY.md, RECORDER_SPEC.md, UNRESOLVED_DEPENDENCIES.csv and COMPLETION_REPORT.md. Original20 prerequisites/29 mappings, F00 history and current G0/RA01/WP01/Tier B boundaries remain unchanged. Stop after M00-G local commit for PD review; no push.
