# Fixed-budget selection: a positive rain-RGB case, mixed transfer

13 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus pinned working changes.

## Result and decision

The established information-selection baseline adds value on **rain RGB** in this
restricted replay. Against the matched spatial control, translation mean/RMSE
fall 13.65%/13.84%, rotation mean falls 26.96%, with 19/19 coverage. Against
uniform selection, translation mean/RMSE fall 55.40%/46.25%. Preserve this positive
case: reliable accepted matches can have different value to a fixed pose budget.

The complete continuation bar passes in 1/6 camera-recording streams. Night RGB
beats uniform but not spatial selection; day rotation worsens; thermal results
are mixed and two candidate streams lose one converged pair. Therefore close this
**32-point accepted-pool/fixed-covariance configuration** without budget, ridge,
seed or solver retuning. This does not close the original-paper selection gap
or refute the published method. It also does not establish a new algorithm:
Max-logDet selection is existing prior work.

## Fixed implementation and qualification

The [preceding design](feature-selection-next-decision-2026-11-13.md) connects
this to original MAC-VO V3, distinct from previous camera-expert selection and
persistent-window weighting. Native MAC-VO already applies NMS, geometry and
reliability filtering before random budget selection. This replay only chooses
subsets from already accepted matches; dense rejected candidates are absent.

All 114 first-19 camera-local pair slots from six development streams are kept:
15,987 accepted factors, k=min(32,N). Four fixed seeds each for uniform and 4x4
spatial controls, one minimum-trace reliability arm, one standard greedy
information arm, plus all-pool context: **1,254 attempted fits**. No inference,
training, new recordings, feature-mean changes, learned weights or robust kernels.
One 23-factor pool is capped unchanged; all eleven fits there are bitwise equal.

The common optical 3D residual is R X1+t-X0 with constant covariance C0+C1
at identity. Both saved native covariances are converted by the same cyclic
basis change; each point uses its own frame intrinsics and saved metric depth.
This is an explicitly fixed-covariance registration approximation, not the native
state-dependent covariance objective or complete frontend/VO reproduction.
All arms use identity initialization, median full-pool source-depth translation
scale, no solver ridge, analytic left-rotation Jacobians and the same Armijo
line search. The selection ridge never enters the fit.

Before real selections, **564 checks pass**: synthetic known transforms,
Jacobian finite differences, selection cardinality/repeats, unit/common-scale
invariance, degenerate-geometry rejection, capped-pool parity, and finite positive
means/SPD covariance/basis consistency for all 114 packets. The fixed solver has
100 maximum updates,24 line-search halvings, rank ratio1e-12, relative-gradient
threshold1e-9 and Armijo coefficient1e-4. These settings were not changed after
real outcomes. All subsets and fitted poses lock before reference values are read.

## Convergence and all fixed comparisons

**1,235/1,254 fits converge by the gradient rule;19 line searches fail.** There
are no iteration caps (maximum observed accepted updates11). Every failure and
its final state/cost/gradient remains retained and unavailable for primary
accuracy, as frozen. The information arm converges18/19 on day RGB and night
thermal,19/19 elsewhere. Its failed pair IDs are day RGB0 and night thermal15.
The latter narrowly misses the declared gradient rule; it was not relabeled or
rerun. Losing coverage remains part of the result.

Each row below uses the same matched pairs for information and **all four** seeds
of the specified control. Compute per-seed means/RMSE on that population, then
average the four seed statistics. Negative changes mean lower error. The full
bar requires at least5% improvement in both translation mean/RMSE, no worse
rotation mean/RMSE, and no successful-pair coverage loss against any control
seed. Each stream must beat both controls; the study requires all six streams.

|Stream|Control|Matched pairs|Translation mean change|Translation RMSE change|Rotation mean change|Coverage not lower|Full bar|
|---|---|---:|---:|---:|---:|---|---|
|day-rgb|uniform|16/19|-0.24%|-5.18%|+101.34%|no|fail|
|day-rgb|spatial|16/19|-2.27%|-5.92%|+71.66%|no|fail|
|day-thermal|uniform|18/19|-2.87%|-2.04%|+15.37%|yes|fail|
|day-thermal|spatial|17/19|-0.90%|+1.95%|+18.45%|yes|fail|
|night-rgb|uniform|19/19|-20.32%|-11.81%|-28.89%|yes|PASS|
|night-rgb|spatial|19/19|+2.62%|+10.68%|-22.96%|yes|fail|
|night-thermal|uniform|17/19|-24.46%|-28.21%|-4.40%|no|fail|
|night-thermal|spatial|15/19|-17.59%|-20.26%|-8.17%|no|fail|
|rain-rgb|uniform|19/19|-55.40%|-46.25%|-48.99%|yes|PASS|
|rain-rgb|spatial|19/19|-13.65%|-13.84%|-26.96%|yes|PASS|
|rain-thermal|uniform|19/19|+7.63%|+3.48%|+25.10%|yes|fail|
|rain-thermal|spatial|18/19|+4.45%|+2.84%|+13.96%|yes|fail|

Rain RGB information means are0.975898m translation and0.077386deg rotation;
spatial-control means1.130166m/0.105957deg, uniform2.188126m/0.151721deg,
all with19 matched pairs. Its all-pool context also has19/19 pairs and
translation mean1.688403m. This single-stream result supports the value of the
standard baseline under these assumptions; it is not broad transfer or a causal
explanation for the prior thermal/uncertainty findings.

![Matched comparisons in all six streams](../../results/feature-selection/run-001/selection-comparison.png)

## Reliability and all-pool context retained

|Stream|Arm|Converged pairs|Translation mean m|Rotation mean deg|
|---|---|---:|---:|---:|
|day-rgb|reliability|18/19|0.059805|0.034216|
|day-rgb|information|18/19|0.033503|0.091640|
|day-rgb|all_pool|19/19|0.031503|0.047990|
|day-thermal|reliability|19/19|0.266478|0.114602|
|day-thermal|information|19/19|0.258654|0.133777|
|day-thermal|all_pool|19/19|0.258813|0.104652|
|night-rgb|reliability|19/19|0.082308|0.042480|
|night-rgb|information|19/19|0.008299|0.008610|
|night-rgb|all_pool|19/19|0.007964|0.010143|
|night-thermal|reliability|19/19|0.006325|0.025868|
|night-thermal|information|18/19|0.005043|0.023548|
|night-thermal|all_pool|18/19|0.006116|0.022904|
|rain-rgb|reliability|19/19|1.819791|0.322219|
|rain-rgb|information|19/19|0.975898|0.077386|
|rain-rgb|all_pool|19/19|1.688403|0.104864|
|rain-thermal|reliability|19/19|1.929909|1.720555|
|rain-thermal|information|19/19|1.758628|1.442828|
|rain-thermal|all_pool|18/19|1.658261|1.167769|

These are own-converged-support summaries; where coverage differs, do not use
them as matched-pair superiority evidence. Full per-seed summaries, matched IDs,
translation/rotation RMSE and every failed slot remain in `scores.json`.
All-pool has a different observation budget and uses the research solver; it is
not the native production trajectory. Selection timings and modeled information
eigenvalues are saved for all fits, not treated as proof of frontend speed or
actual pose accuracy. No covariance-quality threshold was chosen from results.

## Independent verification and limits

The independent auditor reconstructs all point pairs from raw graphs, converts
covariances by axis indexing, and verifies114 information subsets using the
rank-three matrix determinant lemma rather than the primary6x6 score. It also
checks every stochastic/reliability set, all1,254 cost/gradient/rigidity receipts,
11 exact capped-pool fits, reference motions, failure eligibility, matched IDs,
per-seed summaries and six-stream gates. **481,008 checks pass**, including19
failed-fit receipts. Maximum absolute difference across all arrays/scalars is
2.67e-5 (large information eigenvalues included), within the recorded audit's
relative/absolute tolerances. No scorer/auditor retry or parameter change.

RGB supplied reference matrices follow the inherited camera-to-first convention.
Thermal comparisons use nominal same-ID RGB motion conjugated through the fixed
rig. The audit independently constructs absolute thermal camera poses. Physical
cross-camera timing, reference accuracy and mounting remain unverified; no offset,
pose direction, scale or extrinsic was fitted. These exposed prefixes are neither
fresh tests nor independent recordings per pair. No true-flow, full-trajectory,
statistical significance, published-system superiority or novelty claim.

Protocol: `experiments/feature-selection-freeze-2026-11-13.json`, SHA256
`770b5117de8aed4d9df448ba54093d3e75ef07ba07ce38903e332fba4d272e45`,83
source/input/reference pins. Tools: `feature_selection_comparison.py`,
`qualify_feature_selection.py`, `audit_feature_selection.py` under `tools/`.
Artifacts: `results/feature-selection/`, including original attempt logs,
qualification,114 prepared packets, subset/fit journal, global prediction lock,
scores, independent audit, PNG and standalone PDF figure.

## Contribution boundary

[Good Feature Selection](https://arxiv.org/pdf/1905.07807) already provides
Max-logDet selection and bias/variance reasoning. This is a baseline transfer
experiment with a stereo3D fixed-covariance residual, not its exact ORB/combined-
Jacobian reproduction. The positive rain-RGB case belongs in the evidence record;
the broader gate fails. Do not automatically invent a bias-aware selector, change
the point budget or reopen temporal weighting to rescue this result.

The next research deliverable should synthesize the completed shortlist results
and make a finite contribution decision, preserving multiple original-paper
openings. No specific learned correction is justified by this run alone.
