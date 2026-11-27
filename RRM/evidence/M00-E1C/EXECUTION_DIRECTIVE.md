# RRM-C001-F00-ENG-001-M00-E1C
## Independent Startup-Only Rig Architecture, Isolation Design, and Owner-Evidence Acquisition

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `4326dd079a2d80518983a5b793b9bab7f751a5db`
**Execution engineer:** Codex
**Technical reviewer:** ChatGPT
**Approval authority:** RRM Program Director (PD)
**Current gate:** G0 — BLOCKED BY EXTERNAL INPUT, with engineering blockers
**RA01:** Pending PD approval
**WP01:** Unauthorized
**Tier B target designation:** UNVERIFIED
**Tier B execution:** NOT AUTHORIZED

---

## 1. Mission

Develop an evidence-backed architecture for an independent, startup-only simulator test rig that could support future controlled investigation of the PX4/MAVROS startup-delay problem.

The architecture must prioritize physical-system isolation, deterministic configuration, observable startup dependencies, and reproducible evidence.

Obtain or formally request authoritative environment-owner information necessary to evaluate the design.

Compare the proposed independent rig with the existing workspace and produce a PD decision package.

**M00-E1C is a design-readiness milestone, not an implementation or test-execution milestone.**

## 2. Mandatory starting verification

Before engineering work:

1. Confirm branch `ore_proj`.
2. Confirm HEAD is `4326dd079a2d80518983a5b793b9bab7f751a5db`.
3. Verify the working-tree state.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/M00E1B_HANDOFF.md`.
7. Read the M00-E1B environment inventory, configuration traceability, isolation boundary, exposure register, instrumentation assessment, and prerequisite delta.
8. Read the M00-E1A PD decision sheet and prerequisite register.
9. Review the M00-D dependency graph, timing analysis, and controlled reproduction protocol.
10. Review relevant controlled F00 requirements and configuration records.
11. Consult `RRM-transfer/rrm-9d26a58/README.md` as read-only integration history.

Verify historical evidence integrity before proceeding.

Do not assume that prior reported findings establish current runtime conditions without supporting evidence.

## 3. E1C-01 — Environment ownership and authoritative evidence

Identify the responsible owner or owning team for each relevant subsystem, where records permit:

- OSMO workflows and execution infrastructure.
- Isaac Sim runtime and simulation assets.
- PX4 configuration and startup integration.
- MAVROS and MAVLink interfaces.
- ROS 2/DDS networking.
- Host/container security configuration.
- Vehicle hardware and physical interface management.
- F00 configuration control.

Prepare a structured evidence request covering:

- Approved workflow identifiers.
- Container image digests.
- Version manifests.
- Simulator scene and vehicle model identifiers.
- Network topology.
- Device-mount configuration.
- Privileged execution permissions.
- Hardware connection policies.
- Startup scripts and dependency ordering.
- Available instrumentation.
- Environment ownership and operational approval.

Distinguish verified owner identities from proposed contacts.

Do not invent ownership assignments or claim that requests were delivered unless they were actually sent through an authorized channel.

Where records cannot be obtained, preserve the corresponding external dependency.

## 4. E1C-02 — Independent rig architecture

Design a proposed isolated startup-only environment containing only the components needed to reproduce and measure the startup sequence.

Evaluate the minimum necessary roles of:

- OSMO or an equivalent orchestration mechanism.
- Isaac Sim or the relevant simulation dependency.
- PX4 software-in-the-loop execution.
- MAVLink transport.
- MAVROS.
- ROS 2/DDS.
- Readiness and deadline monitoring.
- Logging and timing instrumentation.

Specify the required startup dependencies and termination behavior.

The proposed rig shall not include physical flight-control hardware, physical actuator connections, or operational mission execution.

Where possible, design the rig to exclude rather than merely disable hazardous interfaces.

Provide a component architecture and startup dependency graph.

Explicitly distinguish architecture assumptions from verified platform capabilities.

## 5. E1C-03 — Isolation-by-design

Develop a defense-in-depth isolation model.

Address:

1. Physical hardware exclusion.
2. Serial and USB device access.
3. Container privileges.
4. Host device mounts.
5. MAVLink network endpoints.
6. ROS 2/DDS discovery boundaries.
7. Command-service bridges.
8. External network access.
9. Mission-control interfaces.
10. Actuation and arming pathways.
11. Operator access.
12. Emergency termination and recovery.

For each control, specify:

- Hazard or exposure.
- Proposed mitigation.
- Enforcement mechanism.
- Verification method.
- Evidence required.
- Failure mode.
- Residual risk.
- Responsible owner.

Prefer structural isolation controls over procedural assurances.

Do not claim that the proposed architecture is already isolated or safe to operate.

## 6. E1C-04 — Existing workspace comparison

Compare the independent rig against the current workspace's existing simulator/robot containers.

Evaluate:

- Physical-device exposure.
- DDS command-service connectivity.
- MAVLink endpoint configuration.
- Container privilege boundaries.
- Network isolation.
- Startup dependency reproducibility.
- Configuration traceability.
- Instrumentation coverage.
- Operational interference risk.
- Deployment complexity.
- Maintenance and evidence burden.

Distinguish observed conditions from inferred risks.

Provide an evidence-backed recommendation for the preferred candidate.

**A recommendation does not constitute PD target designation.**

## 7. E1C-05 — Startup instrumentation architecture

Design instrumentation sufficient to characterize:

- Workflow launch and application initialization.
- Simulator startup.
- Vehicle-graph availability.
- PX4 process initialization.
- MAVLink heartbeat availability.
- MAVROS connection.
- ROS 2/DDS discovery.
- Readiness-state transitions.
- Deadline evaluation.
- Host resource contention.

Specify event definitions, timestamp sources, clock synchronization, collection methods, and expected artifacts.

Preserve the historical observations:

- Deadline error → MAVROS connection: **19.276162179 seconds**.
- Application complete → vehicle graph: **74.228771713 seconds**.

These observations originate from one startup episode and shall not be treated as acceptance thresholds.

Design the evidence collection to support multiple independent trials and comparison of run-to-run variability.

Do not execute trials.

## 8. E1C-06 — Rig configuration and reproducibility

Specify a proposed configuration manifest containing:

- Repository revision.
- Container image identifiers and digests.
- Simulator version and scene.
- PX4 version and startup parameters.
- MAVROS/MAVLink configuration.
- ROS 2/DDS configuration.
- Network topology.
- Device access policy.
- Resource limits.
- Instrumentation versions.
- Startup and shutdown procedures.

Identify which values can be populated from existing evidence and which require owner confirmation.

Do not invent missing image digests, workflow identifiers, or controlled configuration values.

Provide a method for preserving immutable configuration records for future experiments.

## 9. E1C-07 — Design verification and safety acceptance

Develop a proposed acceptance checklist for eventual rig implementation.

The checklist shall require evidence that:

- No physical flight controller is connected.
- No physical actuator interfaces are exposed.
- Device mounts and privileges are appropriately restricted.
- MAVLink endpoints are confined to the intended environment.
- DDS discovery and command services cannot reach operational systems.
- Network boundaries are enforced.
- Startup and shutdown behavior are controlled.
- Instrumentation captures required events.
- Configuration is reproducible.
- Abort and recovery procedures are defined.
- Trial evidence can be retained and checksummed.

Separate checks that can be performed offline from checks requiring an authorized active operation.

No future rig may be declared safe solely because its design passes an offline review.

## 10. E1C-08 — PD architecture decision package

Prepare a formal decision package containing:

**A. Problem statement**

Why the current environment has not demonstrated sufficient isolation for Tier B.

**B. Candidate architectures**

Existing workspace versus independent startup-only rig.

**C. Recommended architecture**

Technical rationale, limitations, and dependencies.

**D. Proposed isolation controls**

Enforcement methods and verification evidence.

**E. Configuration requirements**

Known values, missing values, and owner dependencies.

**F. Implementation scope**

Specific work that would be required to construct the proposed rig.

**G. Safety and operational risks**

Including unresolved hardware exposure and middleware connectivity concerns.

**H. Requested PD decision**

One of:

- APPROVE DESIGN FOR FURTHER ENGINEERING
- RETURN FOR REVISION
- REJECT PROPOSED DESIGN

Leave the decision unselected until an actual PD decision is recorded.

Include this explicit statement:

**DESIGN PROPOSAL ONLY — NO DEPLOYMENT OR TIER B EXECUTION AUTHORIZED**

Approval of a design for further engineering shall not imply approval to construct, deploy, activate, or test it.

## 11. E1C-09 — Prerequisite and G0 continuity

Preserve all 20 original M00-E1A prerequisite identifiers.

For each of the 17 unresolved prerequisites, identify whether the proposed rig could:

- Resolve the prerequisite through design.
- Require implementation evidence.
- Require active verification.
- Require authoritative external input.
- Remain outside the scope of the rig.

Do not mark a prerequisite satisfied merely because a mitigation has been proposed.

Maintain G0 as blocked unless independent closure evidence meets the relevant criteria.

Keep RA01 pending unless the PD has approved it.

Do not authorize WP01 or alter the approved roadmap.

Update `RRM/PROJECT_STATUS.md` with M00-E1B completion, M00-E1C progress, and the next required decision.

## 12. Required deliverables

Create the following files under `RRM/`:

1. `verification/reports/M00-E1C/M00E1C_ENVIRONMENT_OWNER_MATRIX.csv`
2. `verification/reports/M00-E1C/M00E1C_OWNER_EVIDENCE_REQUEST.md`
3. `verification/reports/M00-E1C/M00E1C_RIG_ARCHITECTURE.md`
4. `verification/reports/M00-E1C/M00E1C_ISOLATION_CONTROL_MATRIX.csv`
5. `verification/reports/M00-E1C/M00E1C_EXISTING_VS_INDEPENDENT.csv`
6. `verification/reports/M00-E1C/M00E1C_INSTRUMENTATION_ARCHITECTURE.md`
7. `verification/reports/M00-E1C/M00E1C_CONFIGURATION_MANIFEST_SPEC.md`
8. `verification/reports/M00-E1C/M00E1C_DESIGN_ACCEPTANCE_CHECKLIST.csv`
9. `verification/reports/M00-E1C/M00E1C_PD_ARCHITECTURE_DECISION.md`
10. `verification/reports/M00-E1C/M00E1C_G0_CONTINUITY.md`
11. `evidence/M00-E1C/M00E1C_PREREQUISITE_IMPACT.csv`
12. `evidence/M00-E1C/M00E1C_EVIDENCE_INDEX.csv`
13. `evidence/M00-E1C/SHA256SUMS`
14. `handoffs/M00E1C_HANDOFF.md`
15. Updated `PROJECT_STATUS.md`

Additional offline analysis scripts and validation tests may be created under `RRM/` where justified.

Do not create executable deployment scripts that could inadvertently launch services as part of routine validation.

## 13. Validation requirements

Validate that:

- The architecture addresses the startup dependencies under investigation.
- Physical-system isolation is explicit.
- All identified device and command pathways have dispositions.
- Existing-workspace risks are traceable to evidence.
- Configuration assumptions are distinguished from verified facts.
- Owner evidence requests are specific and actionable.
- Instrumentation covers the competing startup-delay hypotheses.
- Proposed acceptance criteria are not misrepresented as approved F00 requirements.
- All 20 prerequisites remain traceable.
- G0 blockers and roadmap governance are preserved.
- Historical evidence remains unchanged.
- No deployment or active testing occurred.
- Protected repository files remain unchanged.

Report tests, skipped checks, warnings, and limitations.

## 14. Git governance

Remain on `ore_proj`.

All Codex-authored modifications must remain within `RRM/`.

Do not modify:

- `RRM-transfer/`.
- Existing AirStack files outside `RRM/`.
- Historical milestone evidence.
- The controlled F00 package.

Stage only authorized `RRM/` changes.

Inspect the staged diff before committing.

Commit locally using:

`RRM-F00 M00-E1C: design isolated startup rig and prepare PD decision`

Report the full SHA, changed-file list, validation results, evidence dependencies, and architecture recommendation.

**Do not push. Only the Program Director pushes to GitHub.**

## 15. Completion and stopping condition

M00-E1C is complete when:

- The independent rig architecture is documented.
- Isolation controls and verification requirements are defined.
- Existing-workspace comparison is complete.
- Owner evidence requirements are explicit.
- Instrumentation and configuration specifications are prepared.
- The PD architecture decision package is ready.
- G0 and prerequisite continuity is maintained.
- Validation is complete.
- Changes are committed locally.

The handoff must answer:

1. Is the independent rig the preferred technical candidate?
2. Which isolation risks would its architecture eliminate?
3. Which risks remain dependent on implementation or active verification?
4. What authoritative owner evidence is still missing?
5. What is the minimum implementation scope?
6. Which decisions must the PD make before engineering proceeds?

**Stop after M00-E1C. Do not construct, deploy, start, or restart the proposed rig or existing simulator services. Do not issue actuator commands, execute Tier B/C trials, or begin WP01.**

**Begin M00-E1C now.**
