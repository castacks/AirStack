# Research decision: close the diagnostic branch and return to the hypothesis

2026-11-05 America/New_York. Decision after EXP152, based on existing results through `0cff9bb86`. This is a synthesis and work decision, not a new empirical experiment.

**Decision: close patch-gap confidence development, park the thermal-bias diagnostic chain, and permit only the single test below as a route back to an uncertainty-selection claim.** No additional bin sweeps, ambiguity variants, calibration guesses, selector fitting or baseline reruns follow automatically. Findings remain available and qualified.

The original question is whether RGB/thermal uncertainty predicts relative geometric reliability beyond strong simple baselines, and whether exploiting it improves VO. Our demonstrated contributions currently support a characterization/feasibility report. They do not establish a general adaptive method, cross-modal covariance calibration or VO improvement.

## Evidence that changes a decision

| Question | Existing evidence | Strength and limitation | Work decision |
|---|---|---|---|
| Can we run meaningful RGB/thermal baseline comparisons? | EXP141–143 acquired complete development recordings and executed/scored DROID and ORB streams. | Substantial engineering result; exposed provided-reference data, differing backend/compute scope. | Keep infrastructure; stop treating another successful execution as method novelty. |
| Is tracking continuity equivalent to accuracy? | EXP143 thermal ORB has higher coverage in day/night/rain, with higher translation RPE in all three. | Repeatable descriptive finding on three exposed recordings. | Keep availability and accuracy as separate endpoints. |
| Does finalization or the constructor bug explain the night travel shortfall? | EXP144 finalization changes little; EXP150 repairs a demonstrated memory-dependence defect while travel/reference remains about 0.61. | Concrete bug fix and useful eliminated explanations; physical cause remains unidentified. | Keep the repair as an experimental artifact; park further symptom-led diagnosis. |
| Does the depth pattern extend beyond one night sample? | EXP151 thermal native/projected depth ratios 0.866 day / 0.827 night, consistent subsets about 0.873 in both. | Conditional sparse projection with shared calibration/time/visibility assumptions; no causal explanation. | Report the observation without fitting a scale correction. |
| Does patch ambiguity justify a confidence policy? | EXP152 joint primary controls have no supported image; 10-point sensitivity is weak, mixed and retains about 1/3 of points. | Insufficient robust conditional evidence; neither proof of no information nor calibrated confidence. | **Close this candidate branch.** Reopen only with materially new evidence, not another subdivision of the same points. |
| Have existing uncertainty selectors demonstrated useful recording-level transfer? | EXP131 combined ridge gives no primary benefit; EXP132 rules fail to beat the better fixed camera within any transfer recording. Recording-specific hindsight choices account for 98.431% of pooled rotation oracle headroom. | Qualified common-target proxies, small exposed cohorts and recipe dependence; hindsight comparator is diagnostic, not deployable. | Stop presenting pooled gains as evidence for within-recording adaptation. |
| Has the main research hypothesis been established? | No validated separate-recording predictive advantage plus downstream VO benefit. | The broad hypothesis remains open; the present implementation route is unsupported. | Do not build or claim adaptive fusion from this evidence. |

Sources: [EXP141](bounded-full-record-findings-2026-11-04.md), [EXP143](orb-full-record-findings-2026-11-05.md), [EXP144](orb-finalization-findings-2026-11-05.md), [EXP150](stereo-baseline-initialization-findings-2026-11-05.md), [EXP151](cross-recording-stereo-depth-findings-2026-11-06.md), [EXP152](stereo-ambiguity-controls-findings-2026-11-05.md), [EXP131](rotation-signal-audit-findings-2026-11-04.md), [EXP132](signal-transfer-findings-2026-11-04.md). The original hypothesis and Fall/Spring decision gates remain in [the research handoff](../../MS_RESEARCH_HANDOFF.md).

Audit counts measure implementation consistency. Progress is judged by resolved hypotheses, useful decisions, independent predictive evidence and demonstrated system benefit. The 152 experiment labels do not denote 152 independent scientific findings.

## Work closed or parked now

- Patch relative-gap calibration, threshold searches and confidence-policy development: **CLOSED_INSUFFICIENT_EVIDENCE**.
- Thermal depth/travel shortfall micro-diagnostics: **PARKED**. Reopen for a concrete fault that blocks the decisive test, or new authoritative reference/calibration evidence. An unexplained residual alone does not trigger another study.
- Tuning on EXP130–132 or EXP147–152 windows: **CLOSED_FOR_VALIDATION**. These remain development evidence.
- Additional ORB/DROID benchmarking or larger test/audit counts as a substitute for the uncertainty question: **OUT_OF_SCOPE**.
- End-to-end selector/fusion implementation: **DEFERRED_UNTIL_SIGNAL_GATE**.

Existing baselines, source snapshots, qualified references, bug fixes and recovery packets remain assets. The Fall characterization/feasibility report can now be written from the evidence above, including the negative and inconclusive results.

## One remaining test, with a stopping rule

[The bounded decisive-test contract](decisive-uncertainty-test-2026-11-05.md) freezes one existing angular-uncertainty rule, fixed-camera and metadata controls, whole-recording separation, meaningful oracle headroom, a practical improvement floor and translation/coverage safeguards. It includes no new predictor search or training. Any execution would be a qualified project-heldout reference-proxy test, not a claim of physical ground truth, network-training holdout or adaptive VO benefit.

**Current disposition: STOP_NOT_EXECUTABLE_CURRENT_INPUTS.** The scoped inventory finds no verified active evaluation recordings. All staged research recordings considered here are development-exposed; the two reported MS2 reserves remain candidates rather than assigned/verified evaluation data. The declared MAC/MACT checkpoint files and native uncertainty runtime are absent at their configured local locations. Existing ORB/DROID outputs are not substitutes for MAC/MACT uncertainty outputs.

No reserve payload was opened, assigned or scored. No new scientific outcome was computed for this decision. The prospective test remains unexecuted; preparation is not a positive or negative test result. Missing assets might be recoverable, and the candidates might become usable, but this inventory does not establish either fact. The evidence/status packet records the exact paths and checks.

This is an execution stop, not another open-ended readiness project. One bounded asset/input recovery session may be attempted only against the frozen test, with a maximum 4 working hours and no new adapter, estimator, feature or target research. If compatible inputs, an unchanged runtime and an exposed-data smoke cannot be restored within that limit, stop the execution attempt and deliver the characterization/negative-feasibility report. Do not consume reserves to debug it. No extra diagnostic series is authorized by this decision.

## Outcome rules

1. Compatible, genuinely separated inputs absent: stop execution and report the missing evidence. Do not relabel development data or infer that the scientific hypothesis failed.
2. Oracle headroom below the frozen practical floor: stop adaptation on that setting. Report insufficient opportunity rather than search for a favorable slice.
3. Headroom exists but the unchanged rule fails the benefit/guardrail criteria: close the present scalar-uncertainty route. Do not rescue it by selecting another rule after test labels.
4. Criteria pass: report a limited benchmark signal and decide separately whether a downstream method experiment is justified. No automatic deployment, fusion or novelty claim.

The next deliverable is this synthesis and a defensible research decision, not another local proxy experiment. The executable scientific test requires its stated evidence; until then, the default research activity is writing the completed characterization result.
