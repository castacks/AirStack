# Contribution decision after the original-paper shortlist

13 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408`. Synthesis of completed results; no new
model, inference, fitting, subset selection or scientific experiment.

## Decision

**Develop a focused technical report on matching uncertainty to the estimator's
residuals and decisions. Do not claim a validated new method from this shortlist,
or launch another variant by default.** Lead with the source-conditioning
analysis, the most explicit mechanism we derived. Use thermal target evaluation
and feature selection as separately scoped empirical boundary studies, not a
single causal chain. A draft report is linked below.

This is an immediate deliverable supported by existing evidence. It is not a
claim that an empirical report alone satisfies thesis requirements or publication
novelty. If the intended deliverable requires a new algorithm, that contribution
remains unresolved. Many original-paper gaps remain open; none of the failed
local configurations establishes that the broader direction is exhausted.

The strongest positive findings are worth retaining:

- On the 80 matched day RGB intervals, changing to the declared source-conditioned
  marginal model lowers translation mean by4.80% and rotation mean by31.05%.
- Released U²Flow handles the declared ordinary controlled thermal warp/appearance
  cases; there is no demonstrated matching failure in that assay to repair.
- Standard information selection improves rain-RGB translation mean/RMSE by
  13.65%/13.84% against spatial selection, with lower rotation and full coverage.

These are different populations, targets and estimators. They cannot be pooled
or presented as evidence that one common physical failure caused all outcomes.

## What the three shortlisted paths actually established

| Original opening / local path | Positive evidence | Limit and disposition |
|---|---|---|
| MAC-VO V1/V2: temporal/state-dependent uncertainty; shortlist A | Known-model joint GLS has useful synthetic risk reduction; the residual derivation distinguishes source conditioning from cross-time dependence. Day conditioned marginals improve matched supplied-reference errors. | The source-conditioned independent-increment joint recipe fails all3recording gates; covariance inflation gives identical point estimates. Close that recipe. No empirical increment correlation, physical covariance correctness or general multi-frame gain established. |
| MACThermal T2/T3 and MAC-VO V4: supervision/transfer; shortlist B | Existing U²Flow baseline succeeds on the fixed known-warp thermal assay and improves day/rain natural provided-geometry disagreement over zero flow. Existing alpha ranks proxy disagreement meaningfully. | Natural references do not identify true temporal matching errors. Training a new confidence head from those disagreements, or escalating nuisance severity to manufacture failure, is not justified by these results. Thermal representation/supervision remains a valid separate research direction. |
| MAC-VO V3: observation quality and complementary constraints; shortlist C | Standard information selection passes the complete bar on rain RGB; night RGB improves over uniform. | It passes1/6streams; night RGB does not beat spatial, day rotation worsens, thermal/coverage results are mixed. It selects only already accepted matches and uses a fixed-covariance research solver. Close that exact configuration. Standard Max-logDet is prior work. |

Primary evidence: [temporal comparison](temporal-window-comparison-findings-2026-11-13.md),
[thermal controlled assay](u2flow-thermal-warp-findings-2026-11-13.md),
[natural geometry diagnostic](natural-flow-geometry-findings-2026-11-13.md), and
[feature selection](feature-selection-findings-2026-11-13.md).

## Why lead with residual conditioning

The [derivation](temporal-residual-derivation-2026-11-13.md) identifies a precise
modeling distinction rather than adding another score. A source ray held fixed
in the optimized state is different from an uncertain source measurement whose
error affects predictions. Root UV rows in this parameterization vanish for
all source depths; root disparity remains a single depth observation. Independent
flow increments still produce dependent accumulated target positions. Conditioning
changes marginal weights as well as temporal blocks, so those changes need separate
controls. The executed comparison contains those controls.

This is **not** a demonstrated bug in MAC-VO or MACThermal, and conditional-source
weighting is not universally the physically correct model. An uncertain-source
model includes the prediction's source-ray Jacobian and its cross terms. The
derivation states both cases; the real run evaluates only the declared exact-
source, independent-increment approximation. Its day benefit is a result about
that choice, not proof that the sensor source ray has no uncertainty.

Generic conditioning, correlated-noise GLS and covariance propagation are established
mathematics. The inspected related work does not establish our mechanism as new.
The report should therefore claim a transparent derivation and controlled empirical
comparison for this implementation, not new covariance theory or a broadly superior
estimator. This is the clearest *explanatory thread*, not a validated method-paper
contribution. The other shortlist studies belong in distinct result sections.

## What could be claimed now

A defensible working statement is:

> We characterize three interfaces between predicted uncertainty and visual
> estimation: residual conditioning, correspondence-target validity and
> fixed-budget selection. Using controlled comparisons, we distinguish positive
> baseline behavior from configuration-specific transfer limits, while retaining
> accuracy, availability and reference-quality boundaries.

The contribution is a proposed reproducible characterization and modeling analysis.
Its publication increment remains unverified. Audits establish arithmetic/source
integrity; they do not establish novelty, physical calibration, independent ground
truth, broad transfer or statistical significance.

Do not use any of the following as the contribution: “thermal does not help,”
“covariance weighting fails,” “consistency is always wrong,” “no thermal labels
exist,” “NMS ignores spatial distribution,” or “a new Max-logDet selector.”
The saved positive results and prior-work/source checks contradict those shortcuts.
Do not restore the earlier incremental-thermal-utility-only framing as the sole
research objective.

## Other original-paper openings remain available

| Opening from the paper map | What this continuation did not test |
|---|---|
| T1 / I1: continuity and recovery | Full thermal-inertial state recovery, reinitialization, persistent VIO/SLAM and calibration drift. |
| I2 / I4: posterior approximation and reliability | Whether a stationary local Gaussian remains reliable under weak geometry, bias, incomplete convergence or sequential use. |
| T3 / I3: sensor transfer | A new cross-sensor adaptation mechanism or reduced-data IMU adaptation. |
| T4: multispectral representations | Training a shared or modality-specific representation and evaluating cross-modal correspondences with a suitable reference. |
| T5: active perception | Camera/motion allocation or closed-loop perception-aware planning. |
| V1 / V2: broader temporal structure | Learned real increment correlations, an uncertain-root likelihood, marginalization or a different persistent estimator. |

Their present implementation cost and novelty need separate assessment. They are
not automatically inferior because they were outside this offline shortlist, and
this table is not a promise that no later work covers them. The
[original-paper map](original-paper-gap-map-2026-11-13.md) separates author
limitations, assumptions and our hypotheses.

## Concrete next deliverable and stopping rule

The [report draft](uncertainty-interfaces-report-draft-2026-11-13.md) is ready for
review. Its primary question is: **what does each uncertainty signal describe,
and what evidence permits using it in a different residual or decision?** It
contains the completed evidence and clear claim limits. Improve that argument
and its literature positioning before authorizing another numerical ladder.

For a future new-method path, first specify one additional mechanism, its
version-specific prior-art difference, a deployable signal and a comparison that
can distinguish its explanation from alternatives. A direction does not need
to be globally “failing” to be worth studying. This requirement is about a precise
claim and informative test, not a veto on self-supervision or unexplored gaps.
No such candidate is selected by this synthesis.

A bounded next source task, if pursuing the conditioning thread as a method
contribution, is to compare exact-source and uncertain-source treatments in the
specific applicable learned BA/VO implementations. The existing derivation is
not proof that they omit this distinction. That review should produce a supported
overlap/contribution decision; it must not automatically trigger another root,
window, covariance or loss variant. The present configuration results stay fixed.

Advisor discussion can use the report and the evidence ledger. No message was
sent, no advisor approval/budget is assumed, and no new recording or held-out
assignment was made. This document supersedes the narrow next-step recommendation
in the earlier `contribution-decision-2026-11-13.md`; historical evidence remains.

## Verification

`results/shortlist-contribution-decision/evidence-ledger.json` records16 source
hashes, exact summary locations, claim wording and unsupported extrapolations.
`verification.json` checks existing audit statuses, cardinalities and arithmetic.
This is an evidence synthesis, not an additional empirical run. Original attempt
logs, numerical freezes and full artifacts remain in their verified Box packets.
