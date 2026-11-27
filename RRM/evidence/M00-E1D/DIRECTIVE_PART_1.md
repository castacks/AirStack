# RRM-C001-F00-ENG-001-M00-E1D
## Diagnostic Guard Host Architecture Review, Measurement Integrity, and Configuration-Change Decision

**Project:** RRM-C001 Autonomous UAS
**Repository:** `https://github.com/castacks/AirStack`
**Branch:** `ore_proj`
**Starting commit:** `9d60ee46b3348fbaa1811a5b234f80446b62d5bd`
**Execution engineer:** Codex
**Technical reviewer:** ChatGPT
**Approval authority:** RRM Program Director (PD)
**Current gate:** G0 — BLOCKED BY EXTERNAL INPUT, with engineering blockers
**RA01:** Pending PD approval
**WP01:** Unauthorized
**Tier B target:** Undesignated
**Independent rig:** Proposed candidate only
**Diagnostic guard host:** Material configuration change — approval pending
**Tier B execution:** NOT AUTHORIZED

---

## 1. Mission

Perform a focused engineering review of the proposed diagnostic guard host and its role in the independent startup-only rig.

Determine whether the guard host is necessary to achieve safe isolation and reliable startup diagnostics, whether it introduces unacceptable measurement interference, and whether an alternative architecture could satisfy the same objectives with fewer configuration changes.

Prepare a controlled configuration-change decision package for PD review.

The milestone shall answer:

1. What specific functions must the diagnostic guard host perform?
2. Which functions are safety-critical versus diagnostic?
3. Can those functions be implemented without an additional host?
4. Could the proposed host alter startup latency or dependency behavior?
5. What configuration changes would be required?
6. What evidence is necessary before implementation authorization?
7. Which architecture should be recommended for PD consideration?

**Do not implement or activate any architecture during M00-E1D.**

## 2. Mandatory starting verification

Before beginning engineering work:

1. Confirm the current branch is `ore_proj`.
2. Confirm HEAD is `9d60ee46b3348fbaa1811a5b234f80446b62d5bd`.
3. Verify the working tree is clean.
4. Read `RRM/ROADMAP_MANDATE.md`.
5. Read `RRM/PROJECT_STATUS.md`.
6. Read `RRM/handoffs/M00E1C_HANDOFF.md`.
7. Read `RRM/verification/reports/M00-E1C/M00E1C_RIG_ARCHITECTURE.md`.
8. Read the M00-E1C isolation-control matrix, instrumentation architecture, configuration-manifest specification, existing-versus-independent comparison, and PD architecture decision document.
9. Review the M00-E1B environment inventory and exposure register.
10. Review the M00-E1A prerequisite register and safety-readiness assessment.
11. Review the M00-D dependency graph, hypothesis matrix, and latency characterization.
12. Review relevant controlled F00 configuration and requirement records.
13. Consult `RRM-transfer/rrm-9d26a58/README.md` as read-only integration history.

If branch, HEAD, or repository state differs from expectations, stop and report the discrepancy. Do not reset, rebase, or switch branches.

Preserve all historical milestone evidence.

## 3. E1D-01 — Diagnostic guard host functional decomposition

Analyze the proposed guard host architecture.

Identify every intended function, including where applicable:

- Network boundary enforcement.
- DDS discovery isolation.
- MAVLink endpoint restriction.
- Command-service filtering.
- Hardware-interface exclusion.
- Startup event observation.
- Timing and telemetry collection.
- Evidence recording.
- Abort and termination coordination.
- Operator monitoring.

For each function, record:

- Functional objective.
- Safety relevance.
- Required interfaces.
- Input and output data.
- Trust boundary.
- Failure behavior.
- Timing sensitivity.
- Alternative implementation.
- Supporting evidence.
- Unresolved assumptions.

Separate safety-enforcement functions from diagnostic-observation functions.

Do not assume that a diagnostic host is an adequate safety boundary merely because it monitors traffic.

## 4. E1D-02 — Alternative architecture evaluation

Evaluate at least three candidate architectures:

**Architecture A — Independent rig with diagnostic guard host**

The M00-E1C proposed architecture.

**Architecture B — Independent rig with host/container-level isolation and passive instrumentation**

An alternative that relies on verified platform isolation controls rather than an additional diagnostic host.

**Architecture C — Minimal startup-only rig with separated safety and diagnostic components**

An architecture that minimizes interference with the startup path while preserving independent safety controls.

For each candidate, evaluate:

- Physical-system isolation.
- Serial/USB exposure.
- MAVLink and DDS confinement.
- Command-service bridge exposure.
- Configuration complexity.
- Platform feasibility.
- Instrumentation coverage.
- Startup timing interference.
- Failure containment.
- Reproducibility.
- Operational complexity.
- Evidence requirements.
- Required approvals.

Identify which features are verified, proposed, or dependent on owner evidence.

Do not claim that any candidate is implementable without evidence supporting platform compatibility.

## 5. E1D-03 — Measurement integrity and observer effects

Analyze whether each proposed architecture could influence the startup behavior under investigation.

Evaluate potential effects on:

- Simulator initialization.
- Vehicle-graph availability.
- PX4 process startup.
- MAVLink transport establishment.
- MAVROS connection.
- ROS 2/DDS discovery.
- Deadline evaluation.
- Readiness-latch behavior.
- CPU, memory, GPU, and network contention.

Identify whether instrumentation is:

- Passive and outside the critical startup path.
- Inline with a startup dependency.
- Resource-sharing with the measured workload.
- Potentially timing-altering.
- Unverified.

Develop a measurement-integrity risk register.

Where an architecture introduces new latency or dependency behavior, identify how that effect could eventually be characterized through separately authorized testing.

Preserve the M00-D historical observations:

- Deadline error → MAVROS connection: **19.276162179 seconds**.
- Application complete → vehicle graph: **74.228771713 seconds**.

These values originate from one startup episode. Do not treat them as acceptance thresholds, statistical distributions, or evidence that the guard host would improve startup behavior.

## 6. E1D-04 — Safety architecture and failure analysis

Conduct a preliminary architecture-level hazard and failure-mode assessment.

Include:

- Physical actuator access.
- Flight-controller connectivity.
- Privileged container execution.
- Host device mounts.
- MAVLink endpoint exposure.
- DDS command-service bridges.
- Network boundary failures.
- Guard-host failure.
- Instrumentation failure.
- Service restart side effects.
- Loss of evidence collection.
- Unexpected startup dependencies.

For each hazard or failure mode, specify:

- Initiating condition.
- Potential consequence.
- Proposed prevention.
- Detection mechanism.
- Required independent control.
- Residual uncertainty.
- Verification method.
- Authorization impact.

Do not assign quantitative risk probabilities without supporting evidence.

A diagnostic or monitoring component must not be represented as a substitute for enforceable isolation.

## 7. E1D-05 — Configuration-change impact assessment

Evaluate the proposed guard host as a controlled configuration change.

Identify:

1. Original F00 configuration assumptions.
2. Proposed independent-rig configuration.
3. New guard-host components.
4. Added network interfaces.
5. Added software dependencies.
6. Changed startup ordering.
7. Changed instrumentation paths.
8. Changed failure behavior.
9. Potential impact on F00 representativeness.
10. Required approval and verification evidence.

Distinguish between:

- A diagnostic environment used for engineering investigation.
- A configuration-equivalent F00 environment.
- A formally qualified F00 baseline.

Do not treat evidence obtained from a modified diagnostic rig as qualification evidence for the controlled F00 baseline without an approved equivalence argument.

## 8. E1D-06 — Platform feasibility and owner evidence

Review the M00-E1C owner-evidence request.

Determine which unanswered questions are decision-critical for each architecture.

Prioritize evidence concerning:

- Permitted workflow types.
- Container networking and namespace controls.
- Device-mount restrictions.
- Privileged execution policies.
- DDS domain and discovery isolation.
- MAVLink routing.
- Image and workflow configuration control.
- Instrumentation deployment options.
- Host resource allocation.
- Environment ownership and approval authority.

Prepare a minimal owner-evidence request that can resolve the highest-impact uncertainties.

If the relevant owner is known and an authorized communication mechanism is available, prepare the request for PD-approved delivery.

Do not send external requests without the required authorization.

Do not invent responses or owner identities.

## 9. E1D-07 — Architecture trade study

Develop a weighted or qualitative trade study comparing the candidate architectures.

Evaluation criteria must include:

- Safety isolation.
- Measurement integrity.
- Configuration traceability.
- Platform feasibility.
- Reproducibility.
- Implementation complexity.
- Operational maintainability.
- Evidence quality.

Clearly distinguish engineering judgments from verified facts.

If weights are used, label them as proposed engineering weights, not approved program priorities.

Perform sensitivity analysis where reasonable.

Do not automatically prefer the architecture with the most components or the most instrumentation.

Recommend one of:

- Retain guard-host architecture for further review.
- Prefer a simpler independent-rig architecture.
- Prefer separated safety and diagnostic components.
- Defer selection pending owner evidence.

Explain the rationale and limitations.

## 10. E1D-08 — PD configuration-change decision package

Prepare a formal PD decision document.

It must contain:

**A. Decision requested**

Whether to accept the proposed diagnostic guard host as part of the independent startup-only rig design.

**B. Technical rationale**

The intended benefits and evidence supporting them.

**C. Alternatives**

A comparison of the three architecture candidates.

**D. Configuration impact**

The changes relative to the existing and controlled F00 environments.

**E. Safety implications**

Isolation controls, failure
