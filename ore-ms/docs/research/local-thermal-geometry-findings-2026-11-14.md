# Local thermal factors: geometry intervention and pose tradeoffs

14 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1.
Completed fixed comparison on existing nominal100group3recording development
cohort. New selection/conditional pose solves; no new frontend/training. Standard
information selection is prior art, not a new algorithm claim.

## Result

Thermal factors can improve translation even when replacing an equal number of
RGB factors, but this fixed joint solver worsens rotation. Conditioning thermal
selection on existing RGB information did not beat the simple uniform thermal
control. Predicted geometric information and observed pose benefit differ.
Keep the positive translation result with its rotation cost, rather than declaring
thermal useless or promoting a general VO improvement.

## Predeclared controls

64accepted observations per new solve. RGB64greedy information, RGB32greedy plus
thermal32conditional-information, RGB32greedy plus thermal32uniform, unconstrained
joint64information and uniform32each. Uniform seed0 key(condition,pair); no seed,
budget,weight or solver search. Original target UV/disparity covariances retained;
source3Dpoints fixed. Shared optical-body tangent; translation scaled for selection
by median RGB source depth. Selection ridge1e-6trace(H_RGB_all)/6. Max-logDet is
established GoodFeature methodology. No sensor semantic label or cross-camera
point correspondence is inferred from this selection.

Historical nominal_joint_motion.py restored byte-identically from original Box
packet.12fixed GaussNewton iterations, original damping/rank checks, original
covariance-weighted UV/disparity objective; not convergence, full native MAC-VO,
source-uncertainty marginalization or calibrated posterior. Allnewpredictions
locked before reference scoring. Explicit same-ID nominal simultaneity retained;
actual camera times unmodified, physical synchronization unverified. Supplied
pose exports are qualified reference products, not independent physical truth.
Allrecords previously exposed; not an untouched transfer trial.

## Primary matched comparisons

Both mixed arms use the same matched population as RGB64:99day,99night,88rain.
Translation RMSE metres and rotation RMSE degrees. These are per-interval errors,
not accumulated trajectory ATE.

| Condition | Matched/99 | RGB64 T / R | RGB32+thermal32 uniform T / R | RGB32+thermal32 conditional T / R |
|---|---:|---:|---:|---:|
|day|99/99|0.052418 / 0.088381|0.050566 / 0.105334|0.060320 / 0.103183|
|night|99/99|0.036902 / 0.053760|0.036813 / 0.059381|0.037382 / 0.077040|
|rain|88/99|0.296590 / 0.061179|0.288409 / 0.072510|0.296051 / 0.073163|

Uniform thermal translation RMSE decreases3.53%day,0.24%night,2.76%rain versus
RGB64. Translation mean improves1.31%day/1.70%rain but worsens0.47%night.
Rotation RMSE increases19.18%day,10.45%night,18.52%rain. Thus favorable translation
conditions exist, without an overall pose improvement. Conditional selection
worsens translation mean in every recording and translation RMSE day/night;
rainRMSE improvement0.18%is small and retains rotation cost.

The64factor RGB control is itself a changed estimator compared with native RGB.
Do not present mixed-vsRGB64 gains as full-system gains over native RGB. Historical
same-solver allRGB,alljoint and nativeRGB contexts are in scores.json on explicit
all-method common support. More/fewer backend factors do not measure frontend,
latency,power or end-to-end compute savings: both native camera pools already exist.

## Local geometry versus uncertainty interpretation

Every accepted packet carries source3Dcoordinates,targetUV/disparity observations
and3x3targetcovariance. Per-factor Jacobians describe how local points constrain
six-dimensional motion. Conditional selection keeps the32RGB seed, then chooses
thermal points by their marginal logDet gain. This directly tests spatial/depth
contribution of accepted two-frame matches. Packet IDs do not establish tracks
lasting three or more frames; no multi-frame persistence claim is made.

Unrestricted information selection chooses0thermal points on87/94common day
intervals,66/91night and87/87rain. Remaining subsets sometimes contain only1–2
thermal points. The unchanged solver validates each camera packet with a minimum
of3, so those joint subsets are rejected.5day/8night/1rain additional joint64
failures are preserved, not repaired by altering selector or validation. This
is a fixed interface incompatibility, not evidence of physical geometric failure.
Allmethod-common support94/91/87 is supplementary; it must not shrink the primary
paired comparison above.11rain slots are unavailable under the paired fixed-budget
contract(11outside the fixed paired-pool contract); all297expected slots retained.

Post-scoring formal information diagnostic: conditional selection predicts lower
translation/rotation variance than uniform thermal, yet has worse translation
errors. Both mixed variants have larger median predicted variance than RGB64.
For conditional mix, translation trace ratios versusRGB64 are1.39/1.26/1.33 and
rotation1.62/1.50/1.85 day/night/rain. This is conditional fixed-source model
information, not calibrated physical uncertainty. No new prediction was produced
from these posthoc diagnostics. They do not identify a causal covariance error.

## Verification and limits

Six fixed first-packet projection Jacobian checks against central finite differences
pass with original1e-5scaled error threshold. Every source packet hash checked
against original binding; standard information blocks use shared tangent and units.
Independent determinant-lemma reconstruction verifies all32selected thermal gains
for every available packet pair. Standalone auditor imports no producer and
checks64unique factors and camera allocation, historical solver source hash, and
reconstructs pairwise translation/rotation metrics. Gradient checks do not prove
solver convergence;12-step cap remains explicit.

This is a small, already exposed, fixed-source geometric control. Learned native
uncertainty need not describe depth/source error, bias or scene-dependent noise;
no physical cause is established. A different geometric prior/model could produce
different results. Feature/solver standard operations are established prior art.
This study is not a representation-learning result, a test of thermal entropy,
or proof that conditional sensor relationships cannot be learned.

## Decision

Keep mixed translation improvement as a scoped positive result. Do not promote
conditional-logDet thermal selection: it fails to beat simpler uniform thermal
selection. Do not automatically tune budget/regularizer or replace rotation with
RGB as a rescue and rebrand it a new contribution; earlier component-fusion studies
already tested that idea. This particular local-information criterion is now
executed, not pending.

The remaining substantive question is which local observations predict beneficial
translation corrections rather than merely large modeled information. A new
learning study would need factor-level evidence/targets and appropriate published
baselines, separate from this standard geometric control. Broad thermal/RGB
representation learning remains open; accuracy remains the endpoint.

Sources: GoodFeatureSelection https://arxiv.org/abs/1905.07807 (fresh abstract
check only; prior local method reading retained), GoodFeatureMatching
https://arxiv.org/abs/2001.00714 (fresh search abstract only), and existing
MAC-VO uncertainty/selector review. No new author-code execution. Raw artifacts:
results/local-thermal-geometry/run-001; prerequisites: verified
2026-11-14-thermal-incremental-comparison Box packet.
