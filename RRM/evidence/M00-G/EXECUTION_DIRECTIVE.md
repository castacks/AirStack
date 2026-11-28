# RRM-C001-F00-M00-G
## AirStack-Native Simulation Configuration Implementation and Offline Validation

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `dc17fa3918c5e8af12ecabade3cee3efdc223f85`
**Program Director:** ChatGPT
**Execution engineer:** Codex
**Previous milestone:** M00-F
**Authorization:** Offline implementation, static analysis, and offline testing only

**G0:** BLOCKED
**RA01:** Pending
**WP01:** Unauthorized
**Tier B target:** Undesignated
**Tier B execution:** NOT AUTHORIZED
**Runtime isolation gate:** NOT PASSED

---

## 1. Program Director authorization

I approve implementation of an RRM-owned, AirStack-native software-only simulation configuration based on the reduced `desktop` topology recommended in M00-F.

The intended components are:

- Isaac Sim / Pegasus.
- PX4 SITL.
- MAVROS.
- Original startup guard host.
- Independent diagnostic recorder.

The implementation must remain entirely under `RRM/`.

Do not modify the existing AirStack launch system, reference stacks, historical evidence, or controlled F00 package.

The purpose of M00-G is to produce a concrete engineering package that can be inspected, tested offline, and subsequently evaluated for a separately authorized simulation trial.

**Do not execute the simulation during M00-G.**

## 2. Mandatory startup checks

Before modifying files:

1. Confirm branch `ore_proj`.
2. Confirm HEAD equals `dc17fa3918c5e8af12ecabade3cee3efdc223f85`.
3. Confirm the working tree is clean.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/M00F_HANDOFF.md`.
7. Review all M00-F architecture, isolation, validation, and PD-decision artifacts.
8. Review M00-E1B through M00-E1E technical findings.
9. Review M00-D startup timing and guard behavior.
10. Review the original M00-E1A prerequisite baseline.
11. Review all applicable F00 requirement mappings.
12. Inspect the AirStack CLI, Compose, simulation, PX4, MAVROS, and ROS 2 integration files.
13. Consult the official AirStack documentation at https://docs.theairlab.org/0.20/docs/development/beginner/key_concepts/.
14. Consult `RRM-transfer/rrm-9d26a58/README.md` as read-only historical reference.

If the expected branch, HEAD, or clean working-tree state is not present, stop and report.

Do not reset, rebase, switch branches, or modify unrelated work.

## 3. M00G-01 — Implement the RRM simulation package

Create a dedicated implementation under:

`RRM/engineering/M00-G/`

Include:

- RRM-specific Compose configuration or overlay.
- Component definitions.
- Environment-variable template.
- Explicit service dependencies.
- Startup sequencing specification.
- Configuration provenance manifest.
- Read-only references to required upstream AirStack assets.

The implementation must reflect actual repository interfaces rather than invented command names or image references.

Where required images, commands, or assets cannot be verified, use explicit unresolved placeholders and make the configuration non-runnable until those fields are resolved.

Do not silently substitute different simulator, PX4, or MAVROS versions.

Do not pull images, install dependencies, or invoke Docker.

## 4. M00G-02 — Enforce isolation by configuration

Implement explicit static protections against:

- Privileged containers.
- Physical-device mounts.
- USB/serial passthrough.
- Host networking.
- Host PID/IPC sharing.
- Excessive Linux capabilities.
- Unrestricted host port publication.
- Uncontrolled MAVLink endpoints.
- External DDS discovery.
- Hardware command-service routing.
- Unsafe environment overrides.

Prefer internal-only container networking where technically compatible.

Do not claim that configuration checks alone prove runtime isolation.

Any unresolved isolation condition must fail the pre-execution gate.

## 5. M00G-03 — Preserve startup guard semantics

The startup guard implementation must preserve the relevant original F00 behavior.

Explicitly identify:

- Startup deadline source.
- Deadline evaluation.
- Failure latch behavior.
- MAVROS connection observation.
- Application-complete observation.
- Vehicle graph readiness.
- Recovery-state transitions.
- Guard-host configuration.

Do not modify the controlled original implementation.

If an adapter or wrapper is required, implement it under `RRM/` and document every behavioral difference.

Do not silently reset failure latches or reinterpret startup completion.

## 6. M00G-04 — Independent diagnostic recorder

Implement a recorder specification and, where feasible offline, a software component that can capture:

- Monotonic timestamps.
- Wall-clock timestamps.
- Startup event sequence.
- Deadline transitions.
- MAVROS connection state.
- Application-complete event.
- Vehicle graph readiness.
- Recovery events.
- Recorder initialization and termination.
- Recorder failure or data loss.

The recorder must be observational.

It must not issue vehicle commands, change guard state, or control simulator startup.

Define a stable event schema and provenance fields.

Include integrity checks for event ordering, missing events, duplicate events, and recorder health.

## 7. M00G-05 — Offline test harness

Implement tests that run without Docker, ROS 2, PX4, Isaac Sim, MAVROS, or external network access.

At minimum, test:

1. Valid configuration acceptance.
2. Privileged-container rejection.
3. Physical-device-mount rejection.
4. Host-network rejection.
5. Unsafe MAVLink endpoint rejection.
6. Unsafe DDS routing rejection.
7. Missing image/asset identity rejection.
8. Missing required configuration rejection.
9. Startup deadline failure.
10. Late MAVROS connection.
11. Application-complete before graph readiness.
12. Failure-latch persistence.
13. Recovery event ordering.
14. Recorder initialization failure.
15. Recorder data loss.
16. Duplicate event detection.
17. Nonmonotonic timestamp detection.
18. Missing terminal event detection.
19. Configuration checksum mismatch.
20. Unexpected upstream configuration change.

Include synthetic positive and negative fixtures.

Do not fabricate real runtime measurements.

## 8. M00G-06 — Reproducibility and provenance

Produce a machine-readable manifest identifying:

- Repository revision.
- Relevant upstream source paths.
- Source checksums.
- Proposed component versions.
- Image identities, where verified.
- Required simulator assets.
- Configuration inputs.
- Environment variables.
- Test harness version.
- Known unresolved dependencies.

Do not invent image digests.

If exact versions or assets remain unknown, record them as unresolved.

Ensure that the implementation can be reproduced from the declared inputs once those dependencies are supplied.

## 9. M00G-07 — Execution-readiness preflight

Implement an offline preflight tool that evaluates the package against the M00-F isolation acceptance gate.

It must report:

- PASS.
- FAIL.
- BLOCKED.
- NOT VERIFIED.

It must distinguish:

**Static configuration compliance** from **runtime containment verification**.

The preflight must fail closed when:

- Image identity is unresolved.
- Required assets are missing.
- Host device isolation is unverified.
- Network confinement is unverified.
- GPU privilege requirements are unknown.
- Recorder reliability is unverified.
- Required configuration provenance is missing.

The tool must not start services or interact with hardware.

A passing static test suite must not result in an overall execution-ready status.

## 10. M00G-08 — Qualification boundaries

Preserve the original F00 evidence baseline.

Maintain the 29/29 requirement mappings and all 20 technical prerequisite dispositions.

Distinguish:

- Historical F00 evidence.
- New RRM engineering diagnostics.
- Formal qualification evidence.

The M00-G implementation is an engineering prototype.

It does not establish F00 qualification equivalence, close G0, approve RA01, or authorize WP01.

Document material differences between the proposed simulation and the original controlled configuration.

## 11. Required deliverables

Create under `RRM/engineering/M00-G/`:

1. RRM-specific simulation configuration and overlays.
2. Environment-variable template.
3. Component dependency manifest.
4. Startup sequencing specification.
5. Startup guard compatibility assessment.
6. Independent recorder implementation or explicit blocked interface.
7. Event schema.
8. Offline configuration validator.
9. Offline execution-readiness preflight.
10. Synthetic fixtures and offline tests.
11. Configuration provenance manifest.
12. Implementation README.
13. Unresolved dependency register.
14. M00-G engineering completion report.

Also create:

- `RRM/evidence/M00-G/M00G_EVIDENCE_INDEX.csv`
- `RRM/evidence/M00-G/M00G_VALIDATION_RESULTS.json`
- `RRM/evidence/M00-G/M00G_PREREQUISITE_CONTINUITY.csv`
- `RRM/evidence/M00-G/SHA256SUMS`
- `RRM/handoffs/M00G_HANDOFF.md`

Update:

`RRM/PROJECT_STATUS.md`

Use clear filenames and maintain consistent traceability.

## 12. Validation requirements

Run the full offline test suite.

Verify:

- No prohibited container settings.
- No physical-device access configured.
- No unauthorized external endpoints.
- No active services started.
- Recorder behavior tested with synthetic events.
- Guard compatibility assumptions documented.
- All unresolved dependencies surfaced.
- Provenance checks pass.
- Historical evidence unchanged.
- Protected files unchanged.
- All 20 prerequisites preserved.
- All 29 mappings preserved.
- G0 blocked.
- RA01 pending.
- WP01 unauthorized.
- Tier B undesignated and unauthorized.
- Runtime isolation gate not passed.

Record test counts, failures, warnings, and skipped tests.

Do not report unresolved runtime conditions as successful validation.

## 13. Git restrictions

Remain on `ore_proj`.

Modify only `RRM/`.

Do not modify:

- `RRM-transfer/`.
- Existing AirStack files outside `RRM/`.
- Historical milestone evidence.
- Controlled F00 package.

Stage only:

`git add -- RRM/`

Inspect the staged diff.

Commit locally with:

`RRM-F00 M00-G: implement offline AirStack simulation package`

Do not push.

## 14. Completion and stop conditions

M00-G is complete when:

1. A concrete RRM simulation package exists.
2. Offline isolation validators are implemented.
3. Startup guard compatibility is assessed.
4. An independent diagnostic recorder is implemented or its interface is explicitly blocked.
5. Synthetic startup and recorder tests pass.
6. Reproducibility inputs are recorded.
7. Execution-readiness preflight reports unresolved runtime blockers accurately.
8. Historical evidence and governance boundaries are preserved.
9. Changes are committed locally.

The final handoff must answer:

- What was actually implemented?
- Which components are runnable versus blocked?
- Which offline tests passed?
- Which isolation properties are statically established?
- Which runtime properties remain unverified?
- What exact dependencies prevent the first controlled simulation?
- What is the smallest safe next technical milestone?

**Stop after M00-G for PD review.**

No Docker startup, simulator launch, PX4 execution, MAVROS execution, network probing, physical-device access, actuation, Tier B/C trial, or WP01 execution is authorized.

**Begin M00-G now.**
