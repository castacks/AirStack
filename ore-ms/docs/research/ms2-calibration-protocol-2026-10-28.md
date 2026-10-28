# MS² calibration target and retrospective evaluation protocol

Declared 2026-10-28 America/New_York, starting from `9a267c0bf`, branch `ore_ms`.
Status: specification only; no parameters fitted, experiment executed, or new
capture inspected. This protocol follows [EXP069](../../experiments/EXP-069-ms2-disparity-coverage.md).

## Question and scope

Can per-camera constant scale or additive signed bias plus scale improve
supplied-reference disparity intervals across the four already examined
recordings? Separately, does their approximate metric risk improve camera
selection while retaining the original point predictions?

These are two evaluation targets. Good disparity coverage need not improve
metric selection through nonlinear inverse disparity. Neither establishes
physical sensor calibration, a Gaussian residual law, independent geometric
truth, or the historical checkpoint training contract. The available-code
mismatch in EXP068 remains a separate finding; this protocol does not repair
or reinterpret the inference interface.

Use exactly EXP066's four populations: `exp063_rain`, `exp064_day`,
`exp064_night`, `exp064_rainy`; IDs `000000`–`000019` per recording, 80 frames,
106562 both-valid pairs. Each camera retains its native horizontal disparity
`d > 0`, variance `v > 0`, and own supplied endpoint's implied disparity
`d_ref = fx * B / Z_ref`. Residual `delta = d - d_ref` is positive when the
prediction exceeds that reference. Camera coefficients have native pixel units;
do not pool RGB and thermal values or compare their pixel likelihoods.

All four recordings have been inspected and influenced this design. Every
excluded-recording result is retrospective development evidence. Keep the final
two unexamined official-test captures reserved. Do not acquire new imagery,
train a network, refit checkpoints, alter correspondence/support, change depth
signs, clip disparities, or correct point predictions in this stage.

## Fixed model families

For camera `c`, model the residual descriptively with mean `beta_c` and
variance `s_c² * v`. Fit each camera separately. No image-, range-, weather-,
position-, or recording-specific coefficients, feature search, ridge sweep,
post-fit truncation, or outcome-dependent model replacement is permitted.

| Family | Residual mean | Residual variance | Free coefficients per camera |
|---|---|---|---|
| Original | 0 | v | none |
| Scale only | 0 | a² v | a² |
| Additive bias plus scale | beta | s² v | beta, s² |

Use the weighted Gaussian negative log likelihood as a fitting objective, not
as an assertion of a Gaussian data distribution. In each training fold, give
each training recording weight `1/3`, each of its 20 frames weight `1/20`, and
each of that frame's `n` both-valid camera observations weight `1/n`.
Consequently every observation has weight `w = 1/(3*20*n)` and weights sum to
one for each camera. Do not let a dense frame dominate the fit.

The closed-form fits are:

```text
Scale only:              a² = sum(w * delta² / v)
Bias plus scale:         beta = sum(w * delta / v) / sum(w / v)
                        s² = sum(w * (delta - beta)² / v)
Objective:              0.5 * sum(w * [log(2*pi*s²*v)
                                     + (delta-beta)²/(s²*v)])
```

For scale only substitute `beta=0, s²=a²`. With positive finite variance and
positive fitted scale, these minimize the stated objective. The bias is the
inverse-variance weighted residual mean, not the unweighted pixel mean.
Weights and observations are fixed independently of fitted values.

Require finite inputs, matching sample counts/order, positive native variance
and consistent camera pixel coordinates/resolution across the training
recordings. Missing inputs or zero/nonfinite fitted scale invalidate that
camera/fold explicitly; do not add an epsilon, drop the recording, or silently
reuse another fold. Empty original frames must be reported and stop the
declared fit rather than changing its weights. Existing support excludes some
observations; this remains conditional evaluation on that support.

An additive pixel bias is deliberately different from fitting a mean to
`delta/sqrt(v)`. Under that alternative, adding squared standardized mean back
to centered second moment reproduces the scale-only second moment and would
give the same original-point risk. Do not present those algebraically identical
risks as distinct camera selectors.

## Recording exclusion and preservation

Run four folds, each training on three recordings and evaluating only on the
fourth. Within each fold freeze both cameras' coefficients before evaluating
the excluded recording. No random pixel/frame splits or evaluation-dependent
refits. Publish every fold and both model families, including failures.
No final all-recording fit is part of this stage.

Before fitting, save the protocol text hash, exact parent commit, executable
source hashes, input-file SHA256 map, recording IDs, frame IDs, camera counts,
weights and numerical runtime. Restore EXP066's derived arrays only with its
validated provenance; EXP069 is the unchanged interval baseline. Verify native
frontend/reference/support/calibration ancestry from EXP063/064 and recheck
hashes after evaluation. Refuse differing existing destinations when restoring.
The current checkout does not contain these excluded scientific payloads;
restoration and verification are prerequisites to a subsequent execution.

Save per-fold training coefficients, objective components and training-only
sufficient sums separately from evaluation outputs. Independently reconstruct
coefficients with weighted normal equations or an independent optimizer,
recompute intervals and selector risks, and check training/evaluation IDs are
disjoint. Perturb only excluded labels in a synthetic control and require
unchanged fitted coefficients and deployable scores. Real evaluation labels
must never influence scores or parameters.

## Interval evaluation

The bias model describes reference disparity with center `d - beta` and
interval `[d-beta-k*s*sqrt(v), d-beta+k*s*sqrt(v)]`. Scale only uses zero bias;
the original uses zero bias and unit scale. Retain inclusive boundaries at
`k = 1, 1.96, 2, 3`, with the same hypothetical standard-normal benchmarks as
EXP069. For standardized residual `(delta-beta)/(s*sqrt(v))`, retain signed
mean, centered spread, second moment, quantiles, negative/positive tails,
coverage, and descriptive native-pixel NLL. Verify coverage plus both tails
equals one.

For each frame/camera, retain EXP069's 11 groups: all points, six supplied
midpoint-reference range bands with lower edges `[0,5,10,20,40,80]` metres,
and four native-variance bins with linear quartiles and right insertion for
ties. Construct bins once from original `v` and reuse them for all families.
Ranges are label-assisted diagnostics. Do not fit within these groups.

At 1.96 sigma, report each frame's signed coverage deviation and absolute
deviation from the benchmark, then median paired changes versus the original
and versus scale only. Retain all other widths, tail imbalance and NLL;
improving this marginal statistic alone is insufficient evidence of conditional
calibration. Report group counts and empty groups, nonempty-frame counts and
within-recording min/median/max. Compute paired changes before aggregation.
No pooling of cameras or recordings, pixel-independence significance tests,
or favorable-group selection.

Count nonpositive interval centers and lower/upper endpoints for every family
and width. Do not clip or invert a nonpositive lower bound into a finite
positive depth upper bound. No depth-coverage claim is part of this protocol.

## Secondary camera selection with fixed point estimates

Keep the original predicted 3D points, both-valid mask, nominal shared-point
support, midpoint reference target and both endpoint sensitivity targets.
The original finite-error losses remain unchanged. Interval recentering above
does not change the point estimates or reference errors being scored.

For native camera ray `q`, `fb = fx*B`, original predicted disparity `d`,
and axial-depth-only first-order propagation, define:

```text
J²             = ||q||² * (fb / d²)²
Original score = J² * v
Scale score    = J² * a² * v
Bias score     = J² * (s²*v + beta²)
```

The last expression includes squared bias because the original point estimate
has not been corrected. Using only `J²*s²*v` would discard that error term.
These are approximate point squared-error scores; they are not exact nonlinear
inverse-disparity risks, full pose covariances, or expected mean-distance risks.
Transforming the depth perturbation by the supplied rotation preserves its
Euclidean norm. Do not use reference depth in `J²` or labels in any score.

Choose the lower score; for exact ties average both camera losses (a declared
50/50 randomized tie policy) and retain tie counts. Require finite positive
scores; preserve and report nonfinite cases as policy failures without choosing
an undocumented fallback or excluding points. Never silently floor/clip them.

Retain original propagated selection, always RGB, always thermal, equal-random
selection, and the EXP056 image-wide per-camera median angular-variance control
as fixed comparators. Reuse its original geometric propagation and tie rule;
verify baseline parity before comparing new models. Any historical tie-rule
difference must be exposed and tabulated, not overwrite the baseline.

Report per-frame mean 3D distance (primary selection loss) and mean squared
3D distance, both endpoint sensitivities and explicit ties/wins/losses. Show
median paired risk ratios against every comparator and scale only. A zero
denominator gets an explicit zero/positive/undefined category; no epsilon.
Retain all frames and original candidate availability counts, including
single-camera-only and neither-valid cases outside the conditional risk.
This does not evaluate a complete operational fallback policy.

## Decision and execution boundary

Publish all four excluded-recording outcomes with calibration and selection
in separate tables. No model adoption or claim of new-capture transfer follows
from this retrospective stage, even if every reported comparison improves.
Any later prospective test requires a separately frozen complete method,
training set, coefficients, availability/tie policy and success criterion
before either reserved capture is accessed. There is no data-dependent
go/no-go threshold or permission to spend those captures in this declaration.

Independent geometric references would materially strengthen physical-error
claims; a usable CATS sample remains unconfirmed. Existing saved arrays suffice
for this retrospective comparison; no new source upload is needed. This
document specifies the next experiment and reports no new empirical result.
