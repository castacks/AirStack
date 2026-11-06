# One bounded uncertainty-value test

2026-11-05 America/New_York. Linked [research decision](research-decision-2026-11-05.md). Status: **UNEXECUTED_NOT_READY**. This freezes the scientific question and decision rules; it is not an execution assignment or proof of readiness.

**Question:** Does the existing angular-normalized uncertainty rule select better RGB/thermal geometric estimates within genuinely separate recordings, beyond fixed-camera and metadata-condition choices, with practically useful accuracy and no translation/coverage penalty?

## Fixed scope

- Experts: unchanged MAC-VO RGB and project MACT thermal, with previously declared checkpoint SHA256 values and thermal recipe. The models are conventional experts, not a new algorithm.
- Candidate: existing EXP131 `angular_uncertainty` rule. Compare median grid covariance trace divided by `(fx²+fy²)/2`; choose RGB for a strictly smaller normalized value, otherwise thermal. Preserve the EXP131 feature-validity rule and thermal fallback for invalid signals on jointly eligible outputs. Sole-available output fallback is common to availability accounting; neither available means abstain. No fitting or parameter search.
- Baselines: always RGB, always thermal, and fixed metadata prior (day RGB / night thermal). Raw unnormalized uncertainty is a secondary diagnostic only; its result cannot replace the primary candidate. The hindsight best fixed camera per recording and per-interval oracle are diagnostic ceilings/comparators, not deployable policies.
- Decision boundary: both experts' outputs already completed. This tests output selection, without sparse scheduling, compute savings or live pre-input claims.
- Target: the same qualified nominal common-interval/common-frame rotation and translation loss contract used by EXP130–132, with source-specific conversion and timing assumptions explicitly bound. No new target invented to clear readiness. If source inputs cannot support that unchanged contract, stop. MS2 candidate metadata alone is not proof that it does.
- Data: at least two entire, previously unexamined project evaluation recordings, different from all development/training records. All chronological outputs, missing intervals and failures stay in the denominator. Report camera timing/reference qualifications and unknown network-training overlap. These are project-heldout claims only.
- Current conditional candidates remain `_2021-08-13-16-31-10` and `_2021-08-13-22-16-02`; they are **not assigned**. No substitution by already exposed records. No rain or broad-condition claim follows from two day/night records.

## Before any evaluation payload

Verify recording exposure/source provenance, compatible source frame/time target, original preprocessing/checkpoint/runtime identity, an exposed-data smoke with both experts' uncertainty/motion receipts, and a source-specific scoring command that cannot read labels during prediction. Bind actual artifact hashes and exact commands in a separate execution lock. If any required evidence is absent, no reserve payload access. Unknown checkpoint-training membership is disclosed, rather than used as a blanket veto on a qualified project-heldout benchmark.

This narrow completed-output test does not require building ORB/DROID again, a full selected-trajectory publisher, a new calibration model or a 12-method SOTA matrix. Those broader EXP133 deliverables do not expand this test. Conversely, an unchanged common-target contract cannot be assumed merely to avoid a real input incompatibility.

The readiness recovery allowance is one session, at most 4 working hours, using existing assets/code only. Missing compatible data/runtime after that means STOP_NOT_EXECUTABLE_CURRENT_INPUTS. No new implementation/diagnostic project is opened as a substitute. No scientific significance is attached to an administrative stop.

## Prediction lock and fixed scoring

Freeze all candidate/baseline choices, native eligible masks/receipts, exact source IDs/intervals and method/config/runtime hashes before reading new evaluation reference values. Both expert predictions must already be fixed before calculating either oracle or candidate losses. No test-label-driven retraining, model selection, temporal smoothing, scale/time fitting, binning or sample replacement.

Primary loss is mean rotation discrepancy in degrees on exact jointly eligible, reference-qualified common intervals, separately per recording. Translation discrepancy in metres and source/output coverage are safeguards. Match all compared methods on the same interval support; report full availability separately. Equal recording weighting is descriptive; each recording must pass individually. No pooled-frame significance or generalization claim from correlated intervals.

## Gates, evaluated once

**Input sufficiency:** each evaluation recording must have at least 1,000 joint reference-qualified intervals and at least 95% coverage of expected source intervals. Keep all missing/excluded counts. Failing support is STOP_INSUFFICIENT_EVALUATION, without changing sampling/thresholds.

**Opportunity gate:** let `B` be the lower mean rotation loss of the two fixed cameras in each recording, and `O` the mean per-interval rotation oracle loss. Require `(B−O)/B >= 0.10` in **both** evaluation recordings. A zero-error fixed baseline has no positive improvement opportunity and fails this gate. Below the floor: STOP_NO_PRACTICAL_HEADROOM. This is a predeclared engineering usefulness threshold, not a statistical confidence bound.

**Predictive-value gate:** on **each** recording, the unchanged candidate must reduce mean rotation loss by at least 5% against every fixed-camera and metadata baseline, while also beating the stronger hindsight per-recording fixed-camera comparator by at least 5%. Require no more than 5% translation-loss increase against each baseline on the same support, with no coverage reduction. If a safeguard baseline has zero translation error, require no increase above zero. No aggregate win can rescue a failed recording. Headroom without this benefit: STOP_NO_SIGNAL_FOR_CURRENT_RULE.

The conservative per-baseline safeguards make the claim narrow and falsifiable; they are not optimized from these future records. A failure can be metric tradeoff, weak ranking, input insufficiency or insufficient opportunity; report the exact category instead of a generic hypothesis rejection.

**Pass:** PASS_LIMITED_BENCHMARK_SIGNAL. This establishes a qualified result for the fixed rule on those recordings. It does not establish physical covariance calibration, a new learned model, downstream VO improvement, online scheduling, adaptive fusion or SOTA. Any subsequent study requires a separate scientific question, not post-test retuning of this experiment.

No further patch-gap variants or tuning of repeatedly scored windows is part of this contract. If the scientific gates fail, close this scalar uncertainty-selection route on the tested setting and write the negative/feasibility result.
