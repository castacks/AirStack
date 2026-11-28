# RRM-C001-F00-M00-I
## Native Guard Build Environment Resolution and Production Interface Verification

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `f5b14e08cbf0ef54bfe3c986ea377fbe50adbf61`
**Program Director:** ChatGPT
**Execution engineer:** Codex
**Previous milestone:** M00-H
**Authorization:** Passive inspection, offline implementation, local compilation with existing dependencies, and offline testing only

**G0:** BLOCKED
**RA01:** Pending
**WP01:** Unauthorized
**Tier B target:** Undesignated
**Tier B execution:** NOT AUTHORIZED
**Runtime isolation gate:** NOT PASSED

---

## 1. Program Director objective

Establish the shortest reproducible path from the existing M00-H guard-host source to a genuine native build against the intended AirStack/ROS 2 interfaces.

The milestone has four priorities:

1. Identify the authoritative upstream build environment.
2. Resolve the exact native dependency and message-interface contracts.
3. Correct RRM-owned source or build integration where evidence supports a correction.
4. Execute native build verification if a suitable offline toolchain already exists.

Avoid repeating synthetic testing that has already passed unless changes require regression coverage.

Do not introduce new simulation architecture or expand organizational-governance work.

## 2. Mandatory startup verification

Before modifying files:

1. Verify branch `ore_proj`.
2. Verify HEAD equals `f5b14e08cbf0ef54bfe3c986ea377fbe50adbf61`.
3. Verify the working tree is clean.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/M00H_HANDOFF.md`.
7. Read the M00-H native build evidence and dependency register.
8. Review the M00-H guard source, adapters, recorder, and integration tests.
9. Review M00-G's original simulation implementation and preflight.
10. Review M00-F's architecture and isolation findings.
11. Review M00-D's original startup-guard behavior and timing evidence.
12. Inspect applicable upstream AirStack build definitions, package manifests, ROS 2 message definitions, and launch files as read-only sources.
13. Consult the official AirStack documentation where relevant.

If branch, HEAD, or working-tree state differs from expectations, stop and report.

Do not reset, rebase, switch branches, or alter unrelated files.

## 3. M00I-01 — Authoritative build-environment identification

Identify the intended native guard build environment using existing evidence.

Inspect:

- Dockerfiles and development-container definitions.
- ROS 2 distribution declarations.
- Package manifests.
- CMake files.
- Colcon workspace configuration.
- Message-generation dependencies.
- Compiler requirements.
- YAML library requirements.
- Environment setup scripts.
- Existing CI build definitions.
- Relevant image references and build arguments.

Determine whether the guard is expected to build:

**A.** Directly on the host.

**B.** Inside an AirStack development container.

**C.** Inside a dedicated ROS 2 workspace.

**D.** Through another documented build mechanism.

Do not assume the current Codex host is the intended runtime or build platform.

Identify the preferred supported path and its evidence.

## 4. M00I-02 — Exact dependency resolution

For every dependency blocking M00-H:

- Identify the exact package or library.
- Identify the expected version or compatible range, where documented.
- Identify the upstream source establishing the requirement.
- Determine whether it is available locally.
- Determine whether it is generated during the normal AirStack build.
- Identify the required include or link interface.
- Record whether it is a build-time or runtime dependency.
- Record its availability and provenance.

Distinguish:

**AVAILABLE AND VERIFIED**

**DECLARED BUT NOT LOCALLY AVAILABLE**

**VERSION UNRESOLVED**

**GENERATED ARTIFACT NOT AVAILABLE**

**UNSUPPORTED OR INCOMPATIBLE**

Do not invent package versions or image digests.

Do not install packages, fetch dependencies, pull images, or start containers.

## 5. M00I-03 — Production ROS 2 interface verification

Inspect the actual upstream interfaces used by the original startup guard and proposed observation adapters.

Verify:

- Topic names.
- Message types.
- Service names.
- Service request/response types.
- QoS assumptions.
- Lifecycle state representations.
- MAVROS connection-state representation.
- Application-complete signaling.
- Vehicle graph readiness detection.
- Deadline and clock behavior.
- Failure-latch semantics.
- Recorder event mapping.

For each interface, record:

- Upstream source path.
- Source revision or checksum.
- Expected type.
- Publisher or provider.
- Subscriber or consumer.
- Relevant configuration.
- Compatibility status.
- Required adaptation.

Do not infer a production interface from a synthetic mock.

Where an interface is absent or cannot be established, mark it unresolved.

## 6. M00I-04 — RRM-owned compatibility corrections

Where the verified upstream interfaces expose mismatches, implement minimal corrections under `RRM/`.

Permitted work includes:

- Header and include corrections.
- CMake dependency declarations.
- Message type alignment.
- Adapter interface corrections.
- Configuration parsing corrections.
- Recorder event translation corrections.
- Deterministic build-script corrections.
- Regression tests for corrected behavior.

Preserve the original F00 guard's safety-relevant semantics.

Do not modify the original guard implementation or other upstream AirStack source.

If a correction changes behavior rather than merely restoring compatibility, document it as a material adaptation and do not claim F00 equivalence.

## 7. M00I-05 — Native build verification

Attempt a native build only if all required tools and dependencies are already locally available.

Permitted:

- Compiler invocation.
- CMake configure/build using existing local dependencies.
- Offline unit tests.
- Local source inspection.
- Dependency-free compilation.
- Build-artifact inspection.

Prohibited:

- Package installation.
- Network downloads.
- Docker or Compose startup.
- Container image pulls.
- ROS 2 node execution.
- PX4 or MAVROS execution.
- Isaac Sim startup.
- Active network probing.
- Physical hardware interaction.

Record distinct outcomes:

**SOURCE CHECK**

**CONFIGURATION**

**COMPILATION**

**LINKING**

**OFFLINE UNIT TESTING**

**PRODUCTION RUNTIME INTEGRATION**

Do not report production build success if only syntax checks or stub-based compilation succeeded.

If a genuine native build remains impossible, stop build attempts after recording the exact missing dependencies and the authoritative supported build path.

Do not manufacture replacement headers to simulate production dependencies.

## 8. M00I-06 — Minimal regression verification

Run existing offline tests affected by source changes.

Add tests only where necessary to verify:

- Corrected message contracts.
- Corrected build configuration.
- Corrected adapter mappings.
- Guard semantic preservation.
- Recorder event compatibility.
- Failure-latch behavior.
- Dependency-checker accuracy.

Do not expand the synthetic test suite solely to increase test counts.

Preserve the existing recorder failure-injection coverage.

## 9. M00I-07 — Build handoff and readiness decision

Prepare a concise native-build handoff that establishes:

1. The intended build environment.
2. The exact missing dependencies.
3. The verified production interface contracts.
4. The source corrections completed.
5. The native build stages actually verified.
6. The build stages still blocked.
7. The minimum external or local resource needed to continue.
8. Whether the next milestone should occur in a suitable build environment rather than the current Codex host.

Recommend one outcome:

**I1 — NATIVE BUILD VERIFIED**

Compilation and linking succeeded against genuine dependencies.

**I2 — BUILD PATH VERIFIED, EXECUTION ENVIRONMENT UNAVAILABLE**

The supported build path is established, but required tools or dependencies are not locally available.

**I3 — SOURCE OR INTERFACE INCOMPATIBILITY**

An unresolved source-level mismatch prevents build readiness.

**I4 — INSUFFICIENT EVIDENCE**

The intended build environment or required interfaces cannot be established.

Do not classify I2 as I1.

## 10. Required deliverables

Create under `RRM/engineering/M00-I/`:

1. `M00I_BUILD_ENVIRONMENT_ASSESSMENT.md`
2. `M00I_NATIVE_DEPENDENCY_MATRIX.csv`
3. `M00I_PRODUCTION_INTERFACE_MATRIX.csv`
4. `M00I_SOURCE_COMPATIBILITY_REPORT.md`
5. `M00I_BUILD_PROCEDURE.md`
6. `M00I_NATIVE_BUILD_RESULTS.json`
7. `M00I_REGRESSION_RESULTS.json`
8. `M00I_BUILD_HANDOFF.md`
9. `M00I_PD_TECHNICAL_DECISION.md`

Include corrected RRM-owned build files, source, and tests where necessary.

Also create:

- `RRM/evidence/M00-I/M00I_EVIDENCE_INDEX.csv`
- `RRM/evidence/M00-I/M00I_VALIDATION_RESULTS.json`
- `RRM/evidence/M00-I/M00I_PREREQUISITE_CONTINUITY.csv`
- `RRM/evidence/M00-I/SHA256SUMS`
- `RRM/handoffs/M00I_HANDOFF.md`

Update `RRM/PROJECT_STATUS.md`.

Avoid duplicating prior milestone reports.

## 11. Validation requirements

Validate that:

- Every dependency claim has an identifiable source.
- Every production interface claim is supported by actual upstream evidence.
- No missing dependency is falsely marked available.
- No stub-based compilation is represented as a native production build.
- Original guard behavior remains protected.
- All RRM-owned corrections are covered by relevant regression tests.
- Historical evidence remains unchanged.
- Protected files remain unchanged.
- All 20 technical prerequisites remain traceable.
- All 29 F00 requirement mappings remain intact.
- G0 remains blocked.
- RA01 remains pending.
- WP01 remains unauthorized.
- Tier B remains undesignated and unauthorized.
- Runtime isolation remains unverified.

Record failures, warnings, skips, and blockers accurately.

## 12. Git restrictions

Remain on `ore_proj`.

Modify only `RRM/`.

Do not modify `RRM-transfer/`, existing AirStack files outside `RRM/`, or historical evidence.

Stage only:

`git add -- RRM/`

Inspect the staged diff.

Commit locally with:

`RRM-F00 M00-I: resolve native guard build and production interfaces`

Do not push.

## 13. Completion and stop conditions

M00-I is complete when:

1. The authoritative build path is identified or its absence documented.
2. Exact native dependencies are mapped.
3. Production ROS 2 interface contracts are assessed.
4. Necessary RRM-owned corrections are implemented.
5. Native build verification is performed wherever the existing environment permits.
6. Regression tests pass or blockers are explicitly reported.
7. Build and runtime readiness remain correctly distinguished.
8. Governance and evidence continuity are preserved.
9. Changes are committed locally.

The final handoff must answer:

- Which build environment is authoritative?
- Which native dependencies are missing?
- Which production interfaces were verified?
- What source corrections were required?
- Did genuine native compilation and linking succeed?
- Which I1/I2/I3/I4 outcome applies?
- What is the smallest concrete next engineering action?

**Stop after M00-I for PD review.**

No Docker or Compose startup, Isaac Sim execution, PX4 execution, MAVROS execution, active network probing, physical-device interaction, deployment, Tier B/C trial, or WP01 execution is authorized.

**Begin M00-I now.**
