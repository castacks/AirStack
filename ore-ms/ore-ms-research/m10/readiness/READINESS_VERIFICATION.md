# Readiness gates

These gates verify the review deliverables and evidence boundaries. Downstream fusion skill, task calibration, task-benchmark preflight and implementation acceptance remain future gates.

| Gate | Status | Observed evidence |
|---|---|---|
| RG01 Historical integrity | PASS | Starting commit pinned. Existing source and evidence had zero scoped changes outside the live roadmap. All nine archived milestone manifests verified. Ten numeric input files match their archived members. |
| RG02 Information provenance | PASS | Interface classifies detector codes, nuisance constraints, conditional estimates, hidden labels and unavailable future observations. Proposed allowlist excludes scope labels that disclose hidden interventions. |
| RG03 Identifiability | PASS | Reproduced equal-transmission residual 0, ranks 1 of 2 and 2 of 3, same-burden future gap 0.16372, and M06 dependence control. Independent constraints specified. |
| RG04 Definitions | PASS | Transmission, SNR, saturation, geometric support, radiometric contrast, predictive and estimation uncertainty, and task performance have distinct definitions and evidence limits. |
| RG05 Failure coverage | PASS | Frozen matrix covers matched cases, source/parameter mismatch, low information, missingness, censoring, geometry, extrapolation and blind shift. Reproduced wrong-air/dynamics coverage and OOD misses. |
| RG06 Fallback | PASS | Each class is routed in the specification to a task-qualified alternative, fixed comparator, abstention or unresolved-control restriction. No fallback uses truth or a transmission-only assertion. |
| RG07 Experimental validity | PASS as specification | Frozen visible-target localization task, all six comparison arms, grouped paired evaluation, task risk/coverage and calibration, ablations and negative controls. Benchmark execution remains pending. |
| RG08 Conditions | PASS | AC10/17/19/21 have owner, gate, evidence and progression impact. AC05/06 and AC20 independently addressed. All live conditions preserved. |
| RG09 Reproducibility | PASS | A clean arithmetic replay reproduced all 39 control rows, with matching archive checksums and referenced-member bytes. |
| RG10 Scope | PASS | Diff contains review prose, read-only evidence tooling and one roadmap status entry. No adaptive fusion code, training, policy evaluation or downstream milestone changes. |

[Archived evidence log](evidence/archive_integrity.log) · [Scope and catalog log](evidence/scope.log) · [Numeric audit](evidence/numeric_audit.csv) · [Reproduction](REPRODUCTION.md)
