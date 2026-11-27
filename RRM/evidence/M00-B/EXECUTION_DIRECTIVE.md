# RRM-C001-F00-ENG-001-M00-B — Codex Execution Directive

**Project:** RRM-C001 Autonomous UAS
**Branch:** `ore_proj`
**Repository:** `castacks/AirStack`
**Engineering workspace:** `AirStack/RRM/`
**Starting commit:** `e772605ccfb6c3bfe515e8e07961cdddc0831968`
**Current gate:** G0 — HOLD
**Execution:** Codex, with ChatGPT systems engineering review

## 1. Mission

Investigate and resolve the outstanding G0 blockers identified during M00.

The task shall establish which blockers can be closed through existing evidence, which require additional engineering work, and which depend on unavailable authoritative records or Program Director decisions.

Do not advance to WP01 or declare G0 passed without sufficient evidence.

## 2. Mandatory starting review

Read the existing M00 evidence, including:

- `RRM/verification/reports/M00_G0_READINESS.md`
- `RRM/handoffs/M00_COMMITTED_FILES.txt`
- M00 requirements, KPI, and IV&V matrices.
- M00 discrepancy and dependency registers.
- Relevant controlled F00 package records.

Use the actual M00 findings and their identifiers. Do not replace the existing assessment with a new, disconnected report.

## 3. Investigation workstreams

### B01 — Missing frozen baseline records

Identify each missing upstream frozen record.

For each record, document its expected source, required revision or checksum if known, the verification activities it blocks, and whether an existing authoritative copy can be located.

Do not reconstruct a missing controlled record and present it as authentic.

### B02 — Configuration equivalence

Compare the controlled F00 configuration with the available AirStack environment.

Assess relevant software versions, source revisions, simulator configuration, vehicle models, firmware, sensor models, interfaces, and container images.

Classify each difference as verified equivalent, non-equivalent, unresolved, or not applicable.

Do not modify the frozen configuration to force equivalence.

### B03 — Open qualification findings

Trace every unresolved qualification finding to its original evidence.

Document the affected requirement, observed condition, technical significance, corrective action, and closure evidence required.

Distinguish historical findings from currently reproduced failures.

### B04 — `actuation_ready=false`

Investigate the reported readiness condition using non-destructive diagnostics.

Determine:

1. Where the readiness state originates.
2. Which prerequisites contribute to the state.
3. Whether the condition is expected in the current configuration.
4. Whether it represents a genuine execution blocker.
5. What evidence would establish readiness.

Do not bypass readiness gates, force the flag to true, or command vehicle motion.

### B05 — Verification framework reconciliation

Review the 29 mapped requirements and associated KPI and IV&V records.

Identify any mappings affected by unresolved baseline records, configuration differences, or qualification findings.

Preserve the distinction between requirements verification and characterization-only metrics.

## 4. Repository and safety controls

All new tracked deliverables shall remain under `RRM/`.

`RRM-transfer/` and all existing AirStack files outside `RRM/` remain protected and read-only.

Codex shall not:

- Modify existing AirStack integration code.
- Alter the controlled F00 baseline.
- Change approved acceptance criteria.
- Restart active simulator or robot services without authorization.
- Bypass operational safety gates.
- Perform physical flight operations.
- Push commits to GitHub.

Existing RRM-transfer documentation may be consulted to understand prior Isaac Sim, Foxglove, WebRTC, and OSMO integration fixes.

## 5. Required deliverables

Create a dedicated `RRM/evidence/M00-B/` evidence directory and corresponding reports.

Deliver:

1. `M00B_BLOCKER_REGISTER.csv`
2. `M00B_FROZEN_RECORDS_ASSESSMENT.md`
3. `M00B_CONFIGURATION_EQUIVALENCE.csv`
4. `M00B_QUALIFICATION_FINDINGS.md`
5. `M00B_ACTUATION_READINESS.md`
6. `M00B_REQUIREMENTS_IMPACT.csv`
7. `M00B_G0_REASSESSMENT.md`
8. `M00B_HANDOFF.md`
9. Evidence integrity manifest with SHA-256 hashes.

Each blocker shall have an identifier, source reference, evidence location, current disposition, required action, responsible decision authority, and closure condition.

## 6. Gate decision rules

Report one of the following:

- **G0 READY FOR REVIEW:** All mandatory G0 conditions have supporting evidence and no unresolved blocking discrepancies remain.
- **G0 HOLD:** One or more mandatory conditions remain unsatisfied or insufficiently evidenced.
- **G0 BLOCKED BY EXTERNAL INPUT:** Required authoritative records or approvals are unavailable.

Do not interpret the completion of diagnostic scripts as proof that a blocked requirement has passed.

## 7. Commit and handoff

Before committing, confirm the branch and record the starting HEAD SHA and working-tree status.

Stage only `RRM/` changes. Run all applicable checks, inspect the staged file list, and verify that protected files are unchanged.

Create a local commit using:

`RRM-F00 M00-B: investigate G0 baseline and readiness blockers`

Return the commit SHA, changed-file list, test results, blocker disposition table, and G0 recommendation.

**Do not push. The Program Director will push to `ore_proj`.**

Stop after M00-B and return the evidence for ChatGPT technical review.

## 8. Authorization

Codex is authorized to perform read-only baseline investigation, non-destructive diagnostics, evidence reconciliation, report generation, and local commits within `RRM/`.

WP01 implementation, baseline modifications, hardware operations, and flight testing are not authorized under this directive.

**Begin M00-B.**
