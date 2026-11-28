# RRM-C001-F00-M00-H
## Guard-Host Build Verification, Runtime Adapter Implementation, and Offline Dependency Closure

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac`
**Program Director:** ChatGPT
**Execution engineer:** Codex
**Previous milestone:** M00-G
**Authorization:** Offline implementation, compilation, synthetic testing, and evidence preparation

**G0:** BLOCKED
**RA01:** Pending
**WP01:** Unauthorized
**Tier B target:** Undesignated
**Tier B execution:** NOT AUTHORIZED
**Runtime isolation gate:** NOT PASSED

---

## 1. Program Director objective

M00-H shall transform the M00-G implementation into a build-verified, offline-tested software package with explicit interfaces and measurable failure behavior.

The engineering priorities are:

1. Verify that the guard host can compile using available local dependencies.
2. Establish source-level compatibility with the original F00 guard semantics.
3. Implement testable adapters for startup, MAVROS, and vehicle-graph observations.
4. Harden the independent recorder against event loss, corruption, and process failure.
5. Resolve dependency identities wherever existing evidence permits.
6. Integrate the results into the offline execution-readiness preflight.

Do not expand the milestone into a new architecture study or organizational-governance exercise.

## 2. Mandatory startup verification

Before changes:

1. Confirm branch `ore_proj`.
2. Confirm HEAD equals `45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac`.
3. Confirm working tree clean.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/M00G_HANDOFF.md`.
7. Review M00-G source, tests, manifests, preflight, and unresolved dependencies.
8. Review M00-F isolation design and technical decision.
9. Review M00-D startup timing and guard findings.
10. Review relevant M00-E1B through M00-E1E source evidence.
11. Review the original F00 requirement mappings and M00-E1A prerequisite baseline.
12. Inspect the upstream AirStack guard, ROS 2, MAVROS, and launch interfaces as read-only sources.
13. Consult the official AirStack 0.20 documentation where applicable.

Stop on branch, HEAD, or working-tree mismatch.

Do not reset, rebase, switch branches, or modify protected files.

## 3. M00H-01 — Guard-host build verification

Inspect the M00-G guard-host source and its relationship to the original F00 implementation.

Determine:

- Compiler and language requirements.
- Required headers and libraries.
- Build-system assumptions.
- External dependencies.
- Include paths.
- Link dependencies.
- Runtime configuration inputs.
- Compatibility with the available offline toolchain.

Implement a reproducible offline build procedure under `RRM/engineering/M00-H/`.

Where dependencies are locally available, compile the guard-host source and run non-networked unit tests.

If dependencies are unavailable, do not install packages, pull images, or silently substitute incompatible libraries.

Instead:

- Record the exact missing dependency.
- Provide a deterministic dependency check.
- Separate compilation of dependency-free components from full integration compilation.
- Report full build verification as BLOCKED.

Do not describe a syntax-only or stub-based compilation as a verified production build.

## 4. M00H-02 — Original guard semantic compatibility

Establish explicit behavioral correspondence between the M00-G guard host and the original F00 startup guard.

Review:

- Startup deadline evaluation.
- Deadline clock source.
- Deadline expiration.
- Failure-latch persistence.
- Late MAVROS connection.
- Application-complete signaling.
- Vehicle-graph readiness.
- Recovery behavior.
- State reset conditions.
- Terminal failure states.

Implement synthetic compatibility tests.

Use the original source as the reference wherever available.

Do not change the controlled original implementation.

If an incompatibility is discovered:

1. Record the exact source-level difference.
2. Identify its behavioral consequence.
3. Implement a correction under `RRM/` if feasible.
4. Add a regression test.
5. Document whether the correction is behavior-preserving or a material adaptation.

Do not silently change failure-latch behavior.

## 5. M00H-03 — Runtime observation adapters

Implement offline-testable adapters for:

**A. Startup lifecycle**

- Process-start observation.
- Application-complete event.
- Deadline event.
- Startup failure event.
- Recovery-state event.

**B. MAVROS connectivity**

- Connection-state observations.
- Connection transition timestamps.
- Disconnection events.
- Late-connection classification.

**C. Vehicle graph readiness**

- Graph-discovery observations.
- Required interface availability.
- Graph-ready transition.
- Loss of readiness.

**D. Recorder interface**

- Event emission.
- Timestamp propagation.
- Source identity.
- Sequence numbering.
- Health signaling.

Adapters must be observational.

They must not issue vehicle commands, alter the guard state, arm hardware, or control simulator startup.

Separate pure observation logic from any future ROS 2 integration layer.

If real ROS 2 bindings are unavailable, implement mocks and explicitly mark the production binding as unverified.

## 6. M00H-04 — Recorder reliability engineering

Extend the M00-G recorder to handle:

- Partial writes.
- Abrupt termination.
- Duplicate events.
- Missing events.
- Nonmonotonic timestamps.
- Out-of-order events.
- Invalid schema.
- Disk-write failures.
- Interrupted flush.
- Corrupted records.
- Recorder initialization failure.
- Recorder shutdown failure.

Define and test:

- Event durability policy.
- Buffering behavior.
- Flush and synchronization behavior.
- Record-integrity verification.
- Recovery after interrupted writes.
- Explicit data-loss indicators.
- Terminal recorder-health status.

Use synthetic failures and temporary offline test files.

Do not fabricate runtime reliability results.

## 7. M00H-05 — Dependency identity closure

Investigate the unresolved M00-G dependencies using existing local evidence.

Prioritize:

1. Guard-host compiler and library dependencies.
2. ROS 2 integration dependencies.
3. MAVROS message and state interfaces.
4. Isaac Sim and Pegasus image/asset identities.
5. PX4 SITL build and runtime identities.
6. GPU requirements and nonprivileged execution support.
7. Container runtime compatibility.
8. Network and device isolation assumptions.

For each dependency, record:

- Required identity or capability.
- Existing evidence.
- Exact source path.
- Version or checksum if verified.
- Availability status.
- Verification method.
- Remaining gap.
- Effect on execution readiness.

Do not invent image digests, asset versions, or compatibility claims.

Do not download dependencies or pull container images.

Do not run active hardware or network probes.

## 8. M00H-06 — Offline integration tests

Create a deterministic synthetic integration harness exercising:

- Startup lifecycle adapter.
- MAVROS state adapter.
- Vehicle graph adapter.
- Original guard behavior.
- Recorder event pipeline.
- Preflight reporting.

At minimum, include:

1. Nominal startup sequence.
2. Deadline expiration before MAVROS connection.
3. MAVROS connection after deadline expiration.
4. Application-complete before graph readiness.
5. Graph readiness before application-complete.
6. Failure-latch persistence.
7. Late recovery without unauthorized latch clearing.
8. MAVROS disconnection.
9. Graph readiness loss.
10. Duplicate startup event.
11. Missing application-complete event.
12. Nonmonotonic event timestamps.
13. Recorder write failure.
14. Recorder restart after interrupted output.
15. Invalid source identity.
16. Configuration mismatch.
17. Missing required dependency.
18. Guard build unavailable.
19. Recorder health degraded.
20. End-to-end synthetic event integrity.

Include positive and negative fixtures.

Tests must run without starting Docker, ROS 2 nodes, PX4, MAVROS, Isaac Sim, or network services.

## 9. M00H-07 — Preflight integration

Update the M00-G offline preflight to incorporate:

- Guard build verification.
- Guard semantic compatibility.
- Adapter test results.
- Recorder reliability results.
- Dependency identity completeness.
- Configuration integrity.
- Static isolation compliance.
- Runtime containment unknowns.

Maintain separate statuses for:

**BUILD STATUS**

**OFFLINE TEST STATUS**

**STATIC ISOLATION STATUS**

**RUNTIME CONTAINMENT STATUS**

**OVERALL EXECUTION READINESS**

Overall execution readiness must remain BLOCKED if any mandatory runtime-dependent criterion is unresolved.

Do not infer runtime safety from offline unit-test success.

## 10. M00H-08 — Qualification and baseline protection

Preserve:

- The 20 original technical prerequisite identifiers and dispositions.
- All 29 F00 requirement mappings.
- The original controlled F00 evidence.
- Historical M00-D timing observations.
- G0 blocked.
- RA01 pending.
- WP01 unauthorized.
- Tier B target undesignated.
- Tier B execution unauthorized.

Distinguish engineering diagnostic evidence from formal F00 qualification evidence.

A successful M00-H build does not automatically close G0 or authorize simulator execution.

## 11. Required deliverables

Create under `RRM/engineering/M00-H/`:

1. Guard-host build configuration and instructions.
2. Build dependency checker.
3. Guard semantic compatibility matrix.
4. Guard compatibility tests.
5. Startup lifecycle adapter.
6. MAVROS state adapter.
7. Vehicle graph readiness adapter.
8. Recorder interface adapter.
9. Recorder reliability implementation.
10. Synthetic integration harness.
11. Offline tests and fixtures.
12. Dependency identity register.
13. Build verification report.
14. Integration test report.
15. Updated execution-readiness assessment.
16. Engineering README.

Also create:

- `RRM/evidence/M00-H/M00H_EVIDENCE_INDEX.csv`
- `RRM/evidence/M00-H/M00H_VALIDATION_RESULTS.json`
- `RRM/evidence/M00-H/M00H_PREREQUISITE_CONTINUITY.csv`
- `RRM/evidence/M00-H/SHA256SUMS`
- `RRM/handoffs/M00H_HANDOFF.md`

Update `RRM/PROJECT_STATUS.md`.

Avoid duplicating previous milestone reports.

## 12. Validation and acceptance

M00-H succeeds when:

1. Guard-host build feasibility is verified or blocked with exact dependency evidence.
2. Guard behavior is tested against the original source semantics.
3. Observation adapters are implemented and offline tested.
4. Recorder failure handling is implemented and tested.
5. Dependency identities are resolved wherever available evidence permits.
6. Preflight correctly incorporates build and integration status.
7. Synthetic integration tests pass.
8. Historical evidence and protected files remain unchanged.
9. Governance and prerequisite continuity is preserved.

A blocked full production build is acceptable only if its precise dependency blockers are recorded and dependency-free components have been validated.

Do not claim full guard compatibility based solely on synthetic mocks.

## 13. Git controls

Remain on `ore_proj`.

Modify only `RRM/`.

Do not modify `RRM-transfer/`, upstream AirStack files, or historical milestone evidence.

Stage only:

`git add -- RRM/`

Inspect the staged diff before committing.

Commit locally with:

`RRM-F00 M00-H: verify guard build and offline runtime adapters`

Do not push.

## 14. Completion and stop conditions

Report:

- Full local commit SHA.
- Changed-file count.
- Guard build result.
- Guard compatibility result.
- Adapter implementation status.
- Recorder reliability test results.
- Total offline test count.
- Resolved and unresolved dependencies.
- Updated preflight statuses.
- Remaining blockers to the first controlled simulation.
- Recommended next engineering milestone.

**Stop after M00-H for Program Director review.**

No Docker service startup, Isaac Sim execution, PX4 execution, MAVROS execution, physical-device interaction, active network probing, deployment, Tier B/C trial, or WP01 execution is authorized.

**Begin M00-H now.**
