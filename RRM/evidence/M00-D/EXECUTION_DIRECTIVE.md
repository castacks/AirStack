# RRM-C001-F00-ENG-001-M00-D
## Controlled Startup Reproduction and Dependency-Latency Characterization

**Project:** RRM-C001 Autonomous UAS
**Repository:** `castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `b4ac9467b073b6ae7927a47b939f46e9407cb6c0`
**Execution engineer:** Codex
**Technical reviewer:** ChatGPT
**Approval authority:** RRM Program Director
**Current gate:** G0 — BLOCKED BY EXTERNAL INPUT, with engineering blockers
**WP01:** Not authorized

### 1. Mission

Determine the underlying causes of the PX4 startup dependency delay identified during M00-C.

Characterize startup sequencing, dependency availability, and timing variability using existing evidence and reproducible diagnostic methods.

Prepare a controlled reproduction procedure capable of distinguishing simulator initialization latency, PX4 startup latency, communication establishment delays, and readiness-state-machine behavior.

**This directive authorizes diagnostic preparation and passive observation only. It does not authorize active simulator restarts or qualification testing.**

### 2. Mandatory starting review

Before beginning, inspect:

- `RRM/ROADMAP_MANDATE.md`
- `RRM/PROJECT_STATUS.md`
- `RRM/handoffs/M00C_HANDOFF.md`
- `RRM/verification/reports/M00-C/M00C_ROOT_CAUSE_ANALYSIS.md`
- `RRM/verification/reports/M00-C/M00C_G0_REASSESSMENT.md`
- M00-C timing evidence, scripts, and tests.
- Relevant M00 and M00-B records.
- Applicable controlled F00 startup and readiness requirements.
- `RRM-transfer/rrm-9d26a58/README.md`, as read-only integration history.

Verify the starting branch, HEAD, working-tree state, and evidence integrity.

Record any discrepancies before proceeding.

### 3. Workstream D01 — Startup dependency graph

Construct a dependency graph covering the relevant startup components:

- OSMO workflow and container initialization.
- Isaac Sim initialization and scene loading.
- ROS 2 and DDS discovery.
- PX4 startup.
- MAVLink and MAVROS connectivity.
- Readiness-state transitions.
- Startup deadline and fail-closed latch.

For each dependency, identify its expected prerequisite, observed readiness signal, source of timing evidence, and failure propagation.

Distinguish documented dependencies from inferred dependencies.

### 4. Workstream D02 — Timing and latency characterization

Use retained logs to establish the available timing measurements.

The M00-C handoff reports **19.276162179 seconds between the deadline error and MAVROS connection**. Preserve this measurement and independently reproduce it from the underlying source.

Do not mislabel this interval as the total PX4 startup duration.

Characterize:

1. Workflow initialization latency.
2. Simulator readiness latency.
3. PX4 process startup latency.
4. MAVLink/MAVROS connection latency.
5. ROS 2 discovery latency.
6. Deadline evaluation timing.
7. Readiness-latch transitions.

For missing measurements, identify the instrumentation needed rather than inventing values.

### 5. Workstream D03 — Reproduction experiment design

Prepare a controlled experiment protocol specifying:

- Exact environment and configuration identifiers.
- Required startup conditions.
- Preconditions for safe execution.
- Clock synchronization and timestamp handling.
- Instrumentation and logging.
- Repeat count and statistical treatment.
- Expected observations under competing hypotheses.
- Abort conditions.
- Evidence retention requirements.
- Regression and safety checks.

Separate the protocol into:

**Tier A — Passive diagnostics:** Read-only inspection of existing evidence and already-running services.

**Tier B — Controlled simulator reproduction:** Starts or restarts simulator and related software services. Requires separate Program Director authorization.

**Tier C — Hardware-in-the-loop or physical testing:** Outside M00-D scope; requires a separate approved test directive.

Do not execute Tier B or Tier C under this assignment.

### 6. Workstream D04 — Diagnostic instrumentation

Implement non-invasive log parsers, timeline reconstruction utilities, and analysis tests within `RRM/`.

Where appropriate, prepare instrumentation patches or adapters in `RRM/` without applying changes to protected AirStack components.

Ensure diagnostic scripts cannot inadvertently arm vehicles, command actuators, modify safety settings, or restart services.

### 7. Workstream D05 — Root-cause discrimination

Build a hypothesis matrix distinguishing:

- Simulator initialization bottleneck.
- PX4 process initialization bottleneck.
- Service startup-order defect.
- MAVLink transport establishment delay.
- MAVROS discovery or connection delay.
- Startup deadline configuration mismatch.
- Readiness-state-machine defect.
- Infrastructure resource contention.

For each hypothesis, specify the observation that would support or refute it.

Assign confidence only where supported by evidence.

### 8. Workstream D06 — G0 and roadmap reconciliation

Update the blocker register and `PROJECT_STATUS.md` using actual findings.

Preserve the status of missing frozen originals, configuration-equivalence findings, recorder and boundary qualification findings, release dependencies, and approval requirements.

Review roadmap amendment RA01 and record its current approval status. Do not silently approve it.

Do not close G0 or authorize WP01.

### 9. Required deliverables

Create the following under `RRM/`:

1. `verification/reports/M00-D/M00D_DEPENDENCY_GRAPH.md`
2. `verification/reports/M00-D/M00D_LATENCY_CHARACTERIZATION.md`
3. `verification/reports/M00-D/M00D_HYPOTHESIS_MATRIX.csv`
4. `verification/procedures/M00D_CONTROLLED_REPRODUCTION.md`
5. `verification/reports/M00-D/M00D_INSTRUMENTATION_PLAN.md`
6. `verification/reports/M00-D/M00D_G0_REASSESSMENT.md`
7. `evidence/M00-D/M00D_BLOCKER_DISPOSITION.csv`
8. `evidence/M00-D/SHA256SUMS`
9. `handoffs/M00D_HANDOFF.md`
10. Updated `PROJECT_STATUS.md`

Add diagnostic scripts, tests, and supporting evidence as needed.

### 10. Validation

Implement and run appropriate checks for:

- Reproducible timeline calculations.
- Correct clock-domain handling.
- Dependency-graph consistency.
- Hypothesis traceability.
- Evidence-file integrity.
- Blocker continuity from M00-C.
- Roadmap/status consistency.
- Protected-file preservation.
- No unauthorized active testing.

Record every test result, including failures and limitations.

### 11. Repository and commit controls

Work exclusively on `ore_proj`.

Modify only authorized files under `RRM/`. Preserve prior milestone evidence and all existing work in `RRM-transfer/` and elsewhere in AirStack.

Stage only `RRM/` changes.

Commit locally using:

`RRM-F00 M00-D: characterize startup dependencies and prepare reproduction`

Report the full commit SHA, changed-file list, validation results, root-cause findings, and authorization requests.

**Do not push. Only the Program Director pushes to GitHub.**

### 12. Stop condition

Stop after completing the passive investigation and preparing the controlled reproduction procedure.

The handoff must clearly identify:

- What is established by existing evidence.
- What remains a hypothesis.
- Which measurements are missing.
- Whether a Tier B simulator reproduction is technically justified.
- The exact authorization and safety preconditions required for that reproduction.
- Which G0 blockers remain open.
- The recommended next engineering milestone.

**Begin M00-D now.**
