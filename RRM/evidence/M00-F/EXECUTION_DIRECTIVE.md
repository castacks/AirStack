# RRM-C001-F00-M00-F
## AirStack-Native Software-Only Simulation Feasibility and Isolation Design

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `6c63054b2d1ac0dee0c5840ed3d4c31083874177`
**Program Director:** ChatGPT
**Execution engineer:** Codex
**Previous milestone:** GOV-002
**Reference:** https://docs.theairlab.org/0.20/docs/development/beginner/key_concepts/

**Authorization class:** Passive inspection, offline analysis, configuration design, and offline validation only.

---

## 1. Program Director decision

I am terminating the sequence of routine organizational-governance discovery milestones.

The objective is to return to meaningful engineering work using AirStack's documented containerized development and simulation architecture.

The previous search identified Sebastian Scherer as a potential professional routing contact, not a verified infrastructure or safety approval authority.

Do not contact Sebastian or other organizational representatives as part of this milestone.

Do not assume organizational authorization is universally required for software-only simulation development.

However, do not assume containerization alone guarantees isolation from physical hardware or external networks.

The next engineering gate is to establish technical feasibility and demonstrate, through static and offline evidence, that a proposed RRM simulation configuration can be isolated from physical systems.

This milestone does not authorize execution of that configuration.

## 2. Mandatory startup verification

Before modifications:

1. Confirm branch `ore_proj`.
2. Confirm HEAD equals `6c63054b2d1ac0dee0c5840ed3d4c31083874177`.
3. Confirm the working tree is clean.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/GOV002_HANDOFF.md`.
7. Review GOV-002's PD release decision and hold disposition.
8. Review M00-E1B through M00-E1E technical findings.
9. Review the M00-D startup investigation and timing evidence.
10. Review the M00-E1A technical prerequisite baseline.
11. Review the F00 requirements and configuration records.
12. Consult `RRM-transfer/rrm-9d26a58/README.md` as read-only historical reference.
13. Inspect the official AirStack 0.20 documentation and applicable repository configurations.

Record the exact documentation revision or retrieval date where available.

If the expected branch, HEAD, or clean-tree conditions are not met, stop and report.

Do not reset, rebase, switch branches, or modify unrelated work.

## 3. M00F-01 — AirStack architecture mapping

Identify the documented relationships among:

- AirStack CLI.
- Docker Compose.
- Robot/autonomy container.
- Isaac Sim.
- Ground control station.
- ROS 2.
- PX4/MAVROS.
- Simulation profiles.
- Custom stack configurations.

Inspect the repository to establish how these concepts are implemented in the checked-out revision.

Distinguish:

**DOCUMENTED:** Explicitly supported by official documentation.

**OBSERVED:** Directly established by repository source or configuration.

**INFERRED:** Reasonable engineering interpretation requiring verification.

**UNKNOWN:** Not established by available evidence.

Do not treat documentation for version 0.20 as proof that every feature exists unchanged in the checked-out repository revision.

## 4. M00F-02 — Simulation profile comparison

Evaluate the documented `desktop` and `desktop_split` development profiles, where present in the applicable documentation and repository.

Determine:

- Which components each profile starts.
- Whether PX4 is simulated or externally connected.
- Whether MAVROS requires a live vehicle endpoint.
- Which ROS 2 interfaces are exposed.
- How simulation time is provided.
- Whether the GCS is necessary for startup investigation.
- Which network connections are created.
- Whether any physical-device access is configured.
- Which startup dependencies may affect the historical F00 failure.

Do not assume either profile is safe merely because it is labeled for desktop development.

Produce a comparison matrix with evidence references.

## 5. M00F-03 — Physical-system isolation analysis

Perform static inspection of relevant Docker, Compose, AirStack, ROS 2, MAVROS, PX4, and simulator configurations.

Evaluate:

### A. Device access

- `/dev` mounts.
- USB and serial-device passthrough.
- Privileged containers.
- Host device access.
- Hardware interface configuration.

### B. Networking

- Host networking.
- Container network modes.
- Exposed ports.
- MAVLink UDP/TCP endpoints.
- DDS discovery and transport.
- Command-service bridges.
- External routing dependencies.

### C. Process and execution boundaries

- Host PID or IPC sharing.
- Container capabilities.
- Security options.
- Startup scripts.
- Environment-variable overrides.
- Automatic service discovery.
- Default launch behavior.

### D. Physical command pathways

- Arming or actuation interfaces.
- Vehicle command services.
- MAVLink forwarding.
- Physical controller discovery.
- Hardware-dependent startup hooks.

Identify both explicit hazards and areas where isolation cannot be established from static evidence.

The design must fail closed when physical-system isolation is uncertain.

## 6. M00F-04 — Minimum RRM simulation configuration

Design an RRM-specific, software-only configuration using the documented AirStack extension model.

Prefer a minimal startup-investigation configuration rather than the complete autonomy stack.

The proposed design should include only components required to investigate:

1. Startup sequencing.
2. MAVROS connection timing.
3. Application-complete timing.
4. Vehicle graph readiness.
5. Startup deadline behavior.
6. Recorder and diagnostic readiness.
7. Recovery behavior after startup failure.

Preserve the ability to compare results against the historical M00-D observations:

- 19.276162179 seconds from deadline error to MAVROS connection.
- 74.228771713 seconds from application completion to vehicle graph readiness.

These values are observations from one historical episode, not acceptance thresholds.

Do not modify existing AirStack launch or Compose files.

If implementation artifacts are useful, create **non-executable design examples under `RRM/` only**, clearly labeled as unvalidated and not authorized for deployment.

## 7. M00F-05 — Isolation acceptance gate

Define an explicit technical gate that must be satisfied before any future simulator execution can be considered.

At minimum, require evidence that:

- No physical-device mounts are present.
- No privileged container access is required.
- No hardware serial or USB endpoints are exposed.
- No MAVLink route reaches a physical vehicle.
- DDS traffic is restricted to the intended simulation environment.
- Command services cannot reach physical actuators.
- Host networking and externally reachable ports are understood.
- Startup scripts cannot silently select physical endpoints.
- Simulator dependencies and images are identified.
- Diagnostic instrumentation cannot alter safety-critical behavior.
- An independent shutdown/containment method is defined.
- The execution host and resource permissions are known.

Each criterion must have:

- Verification method.
- Required evidence.
- Pass/fail condition.
- Responsible reviewer role.
- Current status.

Static inspection may establish configuration intent, but it cannot by itself prove runtime containment.

Classify runtime-dependent criteria as **NOT YET VERIFIED**.

Do not mark the complete isolation gate passed during this milestone.

## 8. M00F-06 — Offline configuration validation

Where practical, create offline validators under `RRM/` that inspect the proposed configuration for prohibited settings.

Examples include:

- Privileged mode.
- Host networking.
- Device mounts.
- Host PID/IPC sharing.
- Excessive capabilities.
- Unrestricted port publication.
- Hardware endpoint references.
- Unsafe environment overrides.

Use synthetic positive and negative fixtures.

Validators must operate without starting Docker, Compose, ROS 2, PX4, Isaac Sim, MAVROS, or any networked service.

Do not install software or fetch container images.

A passing static validator is not proof of runtime isolation.

## 9. M00F-07 — F00 equivalence and claim limits

Separate three evidence categories:

**Category A — Historical F00 evidence**

Previously collected evidence for the controlled F00 baseline.

**Category B — New RRM simulation diagnostics**

Evidence from a proposed RRM-specific software-only environment.

**Category C — Formal qualification evidence**

Evidence accepted against the controlled F00 requirements under an approved qualification process.

Do not automatically promote Category B into Category C.

Document all material configuration differences.

Preserve the existing 29/29 requirement mappings.

Do not invent new acceptance thresholds or claim that a new simulation reproduces the original F00 configuration.

## 10. M00F-08 — Technical decision package

Prepare a concise decision package for PD review.

It must answer:

1. Is an AirStack-native software-only startup experiment technically feasible?
2. Which documented development profile is the best starting point?
3. What is the minimum proposed component set?
4. Which physical-system exposure pathways were identified?
5. Which pathways can be excluded through configuration?
6. Which isolation claims still require runtime verification?
7. What instrumentation is required?
8. What evidence would justify a separately authorized, bounded simulation test?
9. Does the proposed environment depend on shared or restricted infrastructure?
10. Is any specific external authorization genuinely necessary?

Recommend one of:

**F1 — Proceed to a separately authorized offline implementation milestone.**

**F2 — Continue targeted static investigation of specific unresolved hazards.**

**F3 — Hold pending access, platform, or safety evidence.**

Do not interpret F1 as permission to launch a simulator.

## 11. Required deliverables

Create under `RRM/`:

1. `engineering/M00-F/M00F_AIRSTACK_ARCHITECTURE_MAP.md`
2. `engineering/M00-F/M00F_SIMULATION_PROFILE_COMPARISON.csv`
3. `engineering/M00-F/M00F_PHYSICAL_ISOLATION_ANALYSIS.md`
4. `engineering/M00-F/M00F_MINIMUM_SIMULATION_DESIGN.md`
5. `engineering/M00-F/M00F_ISOLATION_ACCEPTANCE_GATE.csv`
6. `engineering/M00-F/M00F_F00_EQUIVALENCE_LIMITS.md`
7. `engineering/M00-F/M00F_PD_TECHNICAL_DECISION.md`
8. `evidence/M00-F/M00F_SOURCE_REGISTER.csv`
9. `evidence/M00-F/M00F_VALIDATION_RESULTS.json`
10. `evidence/M00-F/M00F_PREREQUISITE_CONTINUITY.csv`
11. `evidence/M00-F/SHA256SUMS`
12. `handoffs/M00F_HANDOFF.md`
13. Updated `PROJECT_STATUS.md`

Include offline validators and synthetic fixtures under `RRM/engineering/M00-F/` where useful.

Avoid duplicating historical milestone documentation.

## 12. Validation and repository controls

Validate:

- Every architectural claim has a source.
- Documented features are distinguished from repository-observed features.
- All physical-system pathways are considered.
- No configuration is falsely certified safe.
- No active operations occurred.
- No historical evidence was modified.
- No protected files were changed.
- All 20 prerequisites remain traceable.
- The original 29 F00 requirement mappings remain intact.
- G0 remains blocked.
- RA01 remains pending.
- WP01 remains unauthorized.
- Tier B remains undesignated and unauthorized.

Stage only:

`git add -- RRM/`

Inspect the staged diff.

Commit locally with:

`RRM-F00 M00-F: assess AirStack-native simulation isolation`

Do not push.

## 13. Completion and stop conditions

Report:

- Full local commit SHA.
- Changed-file count.
- Offline validation results.
- Profile recommendation.
- Minimum component configuration.
- Isolation hazards and unresolved questions.
- F1/F2/F3 recommendation.
- Whether external organizational authorization is actually necessary.
- Proposed next technical milestone.

**Stop after M00-F for PD review.**

No simulator startup, Docker service start/restart, PX4 execution, MAVROS execution, physical-device access, active network probing, actuation, deployment, Tier B/C testing, or WP01 execution is authorized.

**Begin M00-F now.**
