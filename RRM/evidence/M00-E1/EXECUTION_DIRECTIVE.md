# RRM-C001-F00-ENG-001-M00-E1
## Tier B Controlled Reproduction — Authorization Readiness

**Project:** RRM-C001 Autonomous UAS
**Repository:** `castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `5b11d4d348db399e0e3ae638e318da9d7079ace6`
**Execution engineer:** Codex
**Technical reviewer:** ChatGPT
**Approval authority:** RRM Program Director (PD)
**Current gate:** G0 — BLOCKED BY EXTERNAL INPUT, with engineering blockers
**WP01:** Unauthorized
**Tier B execution:** Not authorized

## 1. Mission

Prepare a complete, auditable authorization package for isolated Tier B simulator reproduction of the PX4 startup-delay failure.

Establish the experiment configuration, test matrix, instrumentation, safety controls, reproducibility requirements, evidence handling, and authorization boundaries.

**M00-E1 authorizes preparation, offline validation, and passive inspection only. It does not authorize starting or restarting simulator services.**

## 2. Mandatory source review

Read and reconcile:

- `RRM/ROADMAP_MANDATE.md`
- `RRM/PROJECT_STATUS.md`
- `RRM/handoffs/M00D_HANDOFF.md`
- `RRM/verification/procedures/M00D_CONTROLLED_REPRODUCTION.md`
- M00-D dependency graph, latency analysis, hypothesis matrix, and instrumentation plan.
- M00-C startup timeline and root-cause analysis.
- Existing G0 blocker and discrepancy registers.
- Relevant F00 requirements and configuration records.
- `RRM-transfer/rrm-9d26a58/README.md` as read-only historical integration context.

Verify the current branch, starting commit, working-tree state, and source evidence.

Do not silently replace or reinterpret the M00-D protocol.

## 3. E1-01 — Exact configuration identification

Identify the intended Tier B execution environment, including:

- AirStack repository revision.
- OSMO workflow and container identifiers.
- Isaac Sim version and scene.
- PX4 version, firmware configuration, and startup mechanism.
- MAVLink/MAVROS configuration.
- ROS 2 and DDS configuration.
- Vehicle model and simulator interfaces.
- Host resource constraints and available instrumentation.
- Relevant network ports and connection dependencies.

For each item, distinguish **verified**, **proposed**, **unknown**, and **requires approval**.

Do not assume that the current environment is equivalent to the controlled F00 baseline.

## 4. E1-02 — Experimental design

Develop a trial matrix capable of distinguishing:

1. Simulator initialization latency.
2. PX4 startup latency.
3. Service startup-order effects.
4. MAVLink transport establishment delay.
5. MAVROS connection latency.
6. DDS discovery latency.
7. Host resource contention.
8. Deadline and readiness-state-machine behavior.

Define an initial baseline condition and proposed controlled variations.

Specify trial counts and statistical methods with technical justification. Do not invent acceptance thresholds or imply that an exploratory sample size constitutes qualification evidence.

Separate exploratory diagnostics from formal requirement verification.

## 5. E1-03 — Timing and instrumentation

Prepare an instrumentation specification covering:

- Timestamp sources and clock domains.
- Workflow and container lifecycle events.
- Application initialization completion.
- Vehicle graph availability.
- PX4 process start and readiness.
- First observed MAVLink heartbeat.
- MAVROS connection.
- Startup deadline evaluation.
- Readiness-latch transitions.
- Host CPU, memory, GPU, and relevant network observations.

Preserve the established M00-D measurements:

- Deadline error to MAVROS connection: **19.276162179 seconds**.
- Application completion to vehicle graph: **74.228771713 seconds**.

These values are historical observations from one startup episode, not acceptance thresholds.

## 6. E1-04 — Safety and isolation review

Document how the proposed experiment will ensure:

- Simulation-only execution.
- No connection to physical actuators.
- No physical vehicle arming.
- No physical motion commands.
- No active mission interference.
- No unintended changes to other workflows.
- Controlled handling of simulator restarts.
- Preservation of existing logs and configurations.
- Explicit abort conditions.
- Safe recovery after an interrupted trial.

Identify any safety control that cannot be verified before execution.

Do not claim the environment is isolated without supporting evidence.

## 7. E1-05 — Evidence and reproducibility

Define a unique trial identifier and evidence directory convention.

Specify required artifacts for every attempted trial, including failed, aborted, and incomplete trials.

Require:

- Exact configuration and source identifiers.
- Execution timestamps.
- Raw logs.
- Timing-event extraction.
- Resource observations where available.
- Trial outcome and anomalies.
- SHA-256 evidence manifest.
- Reproduction commands or procedures.
- Operator and authorization references.

Never discard an unsuccessful trial or substitute a later successful result.

## 8. E1-06 — Authorization decision package

Prepare a PD decision document containing:

1. Purpose and technical justification.
2. Exact requested Tier B scope.
3. Proposed number and type of trials.
4. Identified environment.
5. Required service operations.
6. Safety and isolation evidence.
7. Risks and mitigations.
8. Preconditions.
9. Abort and recovery criteria.
10. Expected engineering evidence.
11. Unresolved uncertainties.
12. Explicit approval or rejection fields.

The decision document shall clearly state:

**PREPARED — NOT AUTHORIZED FOR EXECUTION**

No signature, approval, or authorization may be inferred from completion of M00-E1.

## 9. E1-07 — G0 and roadmap continuity

Update `RRM/PROJECT_STATUS.md` to reflect M00-D completion and M00-E1 progress.

Maintain the G0 blocker register, including missing frozen originals, configuration-equivalence gaps, qualification findings, release dependencies, and technical decisions.

Preserve RA01 as pending unless an explicit PD decision exists.

Do not authorize WP01 or close any G0 blocker without supporting evidence.

## 10. Required deliverables

Create the following files under `RRM/`:

1. `verification/reports/M00-E1/M00E1_CONFIGURATION_REGISTER.csv`
2. `verification/reports/M00-E1/M00E1_TRIAL_MATRIX.csv`
3. `verification/reports/M00-E1/M00E1_INSTRUMENTATION_SPEC.md`
4. `verification/reports/M00-E1/M00E1_SAFETY_ISOLATION_REVIEW.md`
5. `verification/procedures/M00E1_TIER_B_EXECUTION_PROCEDURE.md`
6. `verification/reports/M00-E1/M00E1_AUTHORIZATION_REQUEST.md`
7. `verification/reports/M00-E1/M00E1_G0_CONTINUITY.md`
8. `evidence/M00-E1/M00E1_READINESS_CHECKLIST.csv`
9. `evidence/M00-E1/SHA256SUMS`
10. `handoffs/M00E1_HANDOFF.md`
11. Updated `PROJECT_STATUS.md`

Include offline validation scripts and tests as appropriate.

## 11. Validation requirements

Codex shall verify:

- All proposed trials have defined objectives and required observations.
- Every active operation is identified as requiring separate authorization.
- Configuration uncertainties are explicitly recorded.
- Instrumentation covers the key competing hypotheses.
- Safety and abort procedures are documented.
- Evidence retention includes unsuccessful attempts.
- No controlled acceptance threshold has been invented.
- Existing milestone evidence remains unchanged.
- Roadmap and project status are consistent.
- Protected repository files remain unchanged.
- No Tier B or Tier C operation occurred.

Report all validation results, including limitations.

## 12. Git and repository controls

Work exclusively on `ore_proj`.

All new work must remain within `RRM/`.

Do not modify:

- `RRM-transfer/`.
- Existing AirStack files outside `RRM/`.
- Historical M00 through M00-D evidence.
- The controlled F00 source package.

Stage only `RRM/` changes.

Commit locally using:

`RRM-F00 M00-E1: prepare Tier B reproduction authorization`

Report the full commit SHA, changed-file list, validation results, and authorization-readiness assessment.

**Do not push. The Program Director alone pushes to GitHub.**

## 13. Stop condition

M00-E1 is complete when the authorization package is prepared, internally validated, committed locally, and submitted for technical review.

The handoff shall distinguish:

- Verified prerequisites.
- Unverified prerequisites.
- Safety blockers.
- Required PD decisions.
- Proposed Tier B scope.
- Conditions for executing M00-E2.

Stop after M00-E1.

**Do not execute Tier B trials, restart services, or begin WP01.**

**Begin M00-E1 now.**
