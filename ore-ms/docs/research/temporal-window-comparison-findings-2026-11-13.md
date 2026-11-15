# Conditioned temporal weighting: completed fixed comparison

13 November2026, America/New_York. Branch ore-ms-orb-development, baseline
a79658f63 plus source-hashed working tree. Four-arm numerical execution and
scoring rules frozen before qualification/real attempts. No training or native
inference. No real outcome-driven changes or repeated attempts.

**CLOSE_THIS_CONDITIONED_TEMPORAL_CONFIGURATION_NO_RETUNING.** Joint temporal
weighting misses every recording's declared advancement gate. Preserve positive
source-conditioning effects and negative cross-time/availability effects
separately; this does not close temporal uncertainty modeling generally.

## What was isolated

All60 existing five-frame windows, RGB-only and RGB+thermal, variable source
depth:480 attempts. Four arms retain observation IDs, camera means, geometry and
solver budgets. `original` calls the recovered original variable-depth solver.
`conditioned_marginal` removes root UV and propagated seed uncertainty while
keeping target-time covariance diagonal across observations. `conditioned_joint`
also retains cross-time blocks implied by independent increments.
`conditioned_scale4` multiplies conditioned marginal covariance uniformly by4.
Its normalized gradient stop is corrected by2, preserving stopping equivalence.
No covariance flooring, new features, robust kernel, clock fit or track rescue.
Stereo noise and cross-track/camera independence remain modeling assumptions.

20iterations maximum,1e-6normal-diagonal damping,12Armijo trials, coefficient1e-4,
scaled-gradient stop1e-6, matching the previous solver. All predictions globally
locked before parsing the previously exposed supplied RGB pose references.
849 nonreference input pins and300 opaque reference hashes recorded before runs.
Root UV removal is deterministic, not a performance-selected support change.

## Qualification and audit

Six first-window RGB/joint qualifications pass: original-weighting transformed
layout pose parity maximum8.89e-16; joint-cost directional finite differences
pass frozen absolute+relative bounds; independent full normal solve agrees with
depth-eliminated solve within3.36e-13. Exact source, protocol and receipts retained.

All480 attempts execute once:409 gradient stops,50 fixed-budget completions and
21 line-search failures. No retries. Failures remain unavailable for all four
intervals of their windows; all attempts/journals/partial states retained.
459 windows deliver1836 intervals over all arms. A budget completion is not a
stationary optimum. Original RGB/joint arms retain all80slots/recording.

Independent post-capture, pre-scoring-frozen auditor imports no producer/solver.
It reconstructs all459 final costs from saved geometry with inverse covariance
quadratic forms, using a separate recursive cross-block construction; maximum
cost difference9.43e-10. It checks all120 original-arm controls against historical
states (max8.42e-9, within frozen1e-8),1836 relative motions/errors, matched
statistics and comparison gates. Uniform4x controls give identical poses in this
run. Audit passes. No independent optimizer or frontend rerun is claimed.
The auditor is not blind. Its receipt's phase description was corrected from
“during capture” to “after capture, before scoring”; original receipt retained,
auditor source unchanged. This metadata correction changes no scientific result.

## Own-support results

These rows have different supports where windows failed. They are not direct
matched-effect estimates. Read the next table for pairwise comparisons.

| Recording | Arm | Delivered intervals | Translation mean / RMSE,m | Rotation mean / RMSE,deg |
|---|---|---:|---:|---:|
| day | rgb_original | 80/80 | 0.042382 / 0.048562 | 0.049162 / 0.053487 |
| day | rgb_conditioned_marginal | 80/80 | 0.040348 / 0.046689 | 0.033898 / 0.037927 |
| day | rgb_conditioned_joint | 80/80 | 0.040095 / 0.046401 | 0.038220 / 0.042137 |
| day | rgb_conditioned_scale4 | 80/80 | 0.040348 / 0.046689 | 0.033898 / 0.037927 |
| day | joint_original | 80/80 | 0.050748 / 0.055536 | 0.051793 / 0.055490 |
| day | joint_conditioned_marginal | 80/80 | 0.043417 / 0.048549 | 0.034815 / 0.038751 |
| day | joint_conditioned_joint | 76/80 | 0.042366 / 0.048025 | 0.037972 / 0.041835 |
| day | joint_conditioned_scale4 | 80/80 | 0.043417 / 0.048549 | 0.034815 / 0.038751 |
| night | rgb_original | 80/80 | 0.018235 / 0.029574 | 0.033155 / 0.053485 |
| night | rgb_conditioned_marginal | 80/80 | 0.018283 / 0.030033 | 0.032475 / 0.052816 |
| night | rgb_conditioned_joint | 80/80 | 0.017798 / 0.029170 | 0.032884 / 0.053360 |
| night | rgb_conditioned_scale4 | 80/80 | 0.018283 / 0.030033 | 0.032475 / 0.052816 |
| night | joint_original | 80/80 | 0.021610 / 0.033734 | 0.041751 / 0.077026 |
| night | joint_conditioned_marginal | 80/80 | 0.020983 / 0.034497 | 0.045750 / 0.089940 |
| night | joint_conditioned_joint | 80/80 | 0.022982 / 0.045006 | 0.045371 / 0.093203 |
| night | joint_conditioned_scale4 | 80/80 | 0.020983 / 0.034497 | 0.045750 / 0.089940 |
| rain | rgb_original | 80/80 | 0.241213 / 0.437051 | 0.034364 / 0.040788 |
| rain | rgb_conditioned_marginal | 68/80 | 0.222035 / 0.391820 | 0.029207 / 0.035857 |
| rain | rgb_conditioned_joint | 64/80 | 0.215695 / 0.384632 | 0.031015 / 0.037503 |
| rain | rgb_conditioned_scale4 | 68/80 | 0.222035 / 0.391820 | 0.029207 / 0.035857 |
| rain | joint_original | 80/80 | 0.242550 / 0.439273 | 0.034224 / 0.040646 |
| rain | joint_conditioned_marginal | 68/80 | 0.224263 / 0.395394 | 0.029098 / 0.035726 |
| rain | joint_conditioned_joint | 64/80 | 0.208442 / 0.387866 | 0.028046 / 0.031111 |
| rain | joint_conditioned_scale4 | 68/80 | 0.224263 / 0.395394 | 0.029098 / 0.035726 |

## Candidate versus matched controls

Candidate is `joint_conditioned_joint`. Negative means lower error. Advancement
requires at least5% lower translation mean AND RMSE against EACH of conditioned
joint-camera marginals, conditioned RGB-only joint weights and original RGB,
with no rotation mean/RMSE increase and no coverage loss, separately per record.
Controls remain the same methods; the term “joint” in their names distinguishes
camera mode from temporal covariance structure as shown in the tables.

| Recording | Control | Matched slots | Candidate translation change mean / RMSE | Candidate rotation change mean / RMSE |
|---|---|---:|---:|---:|
| day | joint_conditioned_marginal | 76 | -2.94% / -1.52% | +7.69% / +6.59% |
| day | rgb_conditioned_joint | 76 | +4.32% / +1.98% | -1.81% / -1.99% |
| day | rgb_original | 76 | -1.42% / -2.60% | -24.10% / -23.03% |
| night | joint_conditioned_marginal | 80 | +9.53% / +30.46% | -0.83% / +3.63% |
| night | rgb_conditioned_joint | 80 | +29.12% / +54.29% | +37.97% / +74.67% |
| night | rgb_original | 80 | +26.03% / +52.18% | +36.84% / +74.26% |
| rain | joint_conditioned_marginal | 60 | -8.11% / -4.16% | +3.81% / +1.90% |
| rain | rgb_conditioned_joint | 60 | +1.96% / +1.91% | -0.04% / -0.04% |
| rain | rgb_original | 64 | -21.00% / -18.46% | -8.82% / -10.79% |

All primary comparison gates fail. Day candidate loses4slots and worsens rotation
against conditioned marginals. Night retains80slots but worsens translation
mean/RMSE9.53%/30.46% against conditioned marginals. Rain loses slots relative
to conditioned marginals (64vs68); their common set has60slots, with translation
mean lower8.11% but RMSE only4.16% lower and rotation higher. Matched RGB-only
conditioned temporal weighting has lower translation error on each recording.

There are useful positive cases: on all80 day slots, source-conditioned marginal
weighting improves RGB translation and rotation over original RGB; joint-camera
conditioned marginals also improve over original joint-camera weights. These
are source-conditioning observations, not evidence that adding temporal blocks
or thermal improves the matched RGB estimator. Rain's favorable own-support
means must not hide missing windows. Uniform covariance inflation cannot explain
changes in point estimates because its control poses are identical here.

## Disposition

Close this exact independent-increment, source-conditioned, fixed-budget window
configuration without damping, threshold, window, head or loss sweeps. Synthetic
oracle utility did not transfer into consistent benefit for this approximate
real-track model. This result does not identify the physical cause, measure
actual increment correlations, reject learned temporal uncertainty, or exhaust
thermal representation/supervision directions from the original-paper map.

No default or learned model changed. Exposed short development windows, nominal
same-ID camera grouping, provided reference quality and unknown checkpoint
training membership retain their earlier limitations. No full-trajectory,
fresh-record, physical synchronization, runtime or calibrated-posterior claim.
No further experiment is automatically selected by this result.

Protocol: `experiments/temporal-window-comparison-freeze-2026-11-13.json`.
Source: `tools/temporal_window_comparison.py`,
`score_temporal_window_comparison.py`, `audit_temporal_window_comparison.py`.
Artifacts: `results/temporal-window-comparison/run-001`, including qualification,
all attempts, prediction lock, scores, independent audit and receipt correction.
Input recovery remains the verified persistent-window Box packet.
