# Thermal image timing: pose effects are mixed and survive fixed-query controls

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.
One fixed comparison complete:36three-pair native frontend batches and72converged
pose fits. No model training, timing/profile adjustment or numerical rescue.

## Result and scientific interpretation

The existing estimator retains all planned outputs under this modest synthetic
sensor profile. Memory/readout effects improve some pose estimates and worsen
others. Lower median predicted matching covariance does not give a monotonic
ordering of center-reference pose error in these controlled cases. The result
persists when the source query IDs are held fixed.

This is bounded evidence that image-matching uncertainty and timestamp-specific
pose accuracy are different evaluation targets under temporal image formation.
It is not evidence that the published uncertainty is generally wrong or that a
new confidence head is needed. There is no exact observed-pixel flow truth for
these integrated images, and no calibrated physical-noise ensemble. Thus we
cannot use pose discrepancies to diagnose bad flow covariance or probabilistic
pose-covariance coverage.

## Fixed inputs, target and estimators

Use all9qualified texture/motion cases from the
[image adapter](sensor-image-adapter-findings-2026-11-14.md): three existing
thermal texture proxies on a4m plane, stationary/constant-body-twist/accelerating
motion, and sharp/memory/rolling/combined image controls. Relative pose truth is
left-camera pose at center readout0.2s to0.3s, identical across arms. The virtual
8ms response/27.8microsecond row profile remains unchanged; it is not measured
MS2 calibration. No new scene, severity, texture, timestep or label was selected.

Released MACT checkpoint strict-loads without migration and its state remains
unchanged. Dense means/covariance outputs, native selection/covariance bodies and
the existing identity-initialized optical frozen-weight IRLS pose harness are
used with the same settings. This is not a full historical native VO trajectory,
IMU fusion run or TRGS-SLAM/TRNeRF reproduction. Each frontend batch contains
source stereo, target stereo and temporal-left pairs:108pair predictions total.

Two predeclared query modes give72planned fits:

- Primary regenerated queries: native selection independently responds to each
  image arm, followed by common synthetic-boundary eligibility.
- Common-ID control: use sharp-seeded source queries jointly admissible/finite
  across all four dense fields. Query identities stay fixed; their endpoint
  means and covariance values remain image-arm-specific.

The common renderer pixel mask intersects all arms/stereo/time views, is eroded
by31×31for covariance-patch support and intersected with the native32pixel crop.
Both source query and predicted target floor coordinates must be in that safe
mask; at least10accepted points are required. This oracle synthetic-boundary
eligibility is a research control, not a deployable native selection method.
Network images are not masked and remain exactly the qualified inputs.

All common-ID sets in this execution retain the sharp accepted set. Their counts
are equal across four arms, while primary counts can change. Overall support is
48–152points for primary fits and62–152for common-ID fits. Every72attempt is
retained; there is no point replenishment or missing-pose substitution.

The freeze inherits some historical provenance fields from its source parent.
Old trajectory/frame-count metadata are not live controls: the explicitly locked
sensor-adapter inventory, this script and the36/72population define this run.

## Primary pose errors: regenerated queries

Translation error, millimetres:

| Moving case | Sharp | Memory | Rolling | Combined |
|---|---:|---:|---:|---:|
| day twist | 4.238 | 2.422 | 3.423 | 2.208 |
| night twist | 0.626 | 1.248 | 0.679 | 1.537 |
| rain twist | 1.137 | 2.504 | 0.954 | 2.174 |
| day accelerating | 2.489 | 2.006 | 1.743 | 1.687 |
| night accelerating | 1.189 | 1.415 | 2.244 | 1.509 |
| rain accelerating | 2.338 | 4.382 | 1.564 | 1.822 |

Rotation error, degrees:

| Moving case | Sharp | Memory | Rolling | Combined |
|---|---:|---:|---:|---:|
| day twist | 0.06608 | 0.03808 | 0.05360 | 0.03327 |
| night twist | 0.01193 | 0.00904 | 0.00606 | 0.01293 |
| rain twist | 0.01050 | 0.03306 | 0.01337 | 0.02518 |
| day accelerating | 0.03714 | 0.04261 | 0.02765 | 0.03645 |
| night accelerating | 0.02016 | 0.02333 | 0.03559 | 0.02419 |
| rain accelerating | 0.03792 | 0.06465 | 0.02153 | 0.01357 |

Preserve both sides of the result. Day-twist combined rendering reduces
translation4.238→2.208mm and rotation0.06608→0.03327deg. Rain-accelerating memory
increases translation2.338→4.382mm and rotation0.03792→0.06465deg. These are
small absolute errors in artificial scenes, not a physical failure rate or a
claim that thermal artifacts generally help/harm odometry.

## Same-query control

Translation error, millimetres:

| Moving case | Sharp | Memory | Rolling | Combined |
|---|---:|---:|---:|---:|
| day twist | 4.238 | 1.741 | 3.288 | 1.394 |
| night twist | 0.626 | 1.411 | 0.585 | 1.724 |
| rain twist | 1.137 | 1.902 | 1.236 | 1.674 |
| day accelerating | 2.489 | 1.735 | 2.269 | 2.038 |
| night accelerating | 1.189 | 2.436 | 1.601 | 2.334 |
| rain accelerating | 2.338 | 3.575 | 2.001 | 1.655 |

Rotation error, degrees:

| Moving case | Sharp | Memory | Rolling | Combined |
|---|---:|---:|---:|---:|
| day twist | 0.06608 | 0.02692 | 0.05253 | 0.02133 |
| night twist | 0.01193 | 0.01269 | 0.00673 | 0.01730 |
| rain twist | 0.01050 | 0.01846 | 0.01418 | 0.00922 |
| day accelerating | 0.03714 | 0.03699 | 0.03513 | 0.04000 |
| night accelerating | 0.02016 | 0.03816 | 0.02779 | 0.03784 |
| rain accelerating | 0.03792 | 0.05501 | 0.02883 | 0.01780 |

The rain-accelerating memory regression remains2.338→3.575mm with identical
source IDs; the day-twist combined improvement remains4.238→1.394mm. Thus query
reselection alone does not explain the responses. The fixed-ID test still
changes predicted endpoints, depth and covariance together, and does not isolate
which of those causes each improvement/regression or supply an explained-error
percentage.

![All moving-case pose errors and both query controls](../../results/sensor-estimator-comparison/scoring-001/pose-errors.png)

## Stationary nulls and uncertainty readouts

Stationary inputs, dense outputs, query identities and fitted pose/information
outputs repeat bitwise across all four arms and both query modes. Nonzero
zero-motion estimation bias is retained, not subtracted or interpreted as a
renderer failure:

| Texture | Translation error, mm | Rotation error, deg |
|---|---:|---:|
| day | 1.735 | 0.03230 |
| night | 3.249 | 0.04288 |
| rain | 3.181 | 0.04735 |

The matching readout is median temporal x/y covariance trace over one fixed safe
pixel mask per case, before query selection. Among the6moving cases:

| Compared with sharp | Cases with lower matching covariance | Higher translation error (primary / fixed IDs) | Higher rotation error (primary / fixed IDs) |
|---|---:|---:|---:|
| Memory |6/6|4/6 /4/6|4/6 /4/6|
| Rolling |3/6|2/6 /2/6|2/6 /2/6|
| Combined |6/6|3/6 /3/6|3/6 /3/6|

For example, rain-accelerating memory covariance decreases0.00881→0.00799in
native output units while both pose errors rise. This shows divergence from
that aggregate readout, not proof that the learned per-match uncertainty target
was violated. Predicted2D covariance, the pose solve's inverse information and
true pose error are separate quantities.

Separate translation/rotation information standard-deviation proxies are saved
for every fit in the CSV. They do not uniformly decrease with the matching
readout; moving fixed-query translation proxies are roughly13–22mm. Those
curvature-based values are neither newly certified physical standard deviations
nor a sampling-coverage result. No covariance inflation/recalibration is selected.

## Verification and limitations

10pre-inference mechanics fixtures pass mask/target bounds, analytic geometry
and a known pose fit, joint-query membership, empty/minimum support and identity
scoring. Before inference the complete producer/auditor/source/image/solver
protocol is frozen. All36dense batches and72fits lock before any true-pose values
are read by the scorer. The capture function reads images and the boundary mask,
not pose labels. No code or tolerance amendment is needed during this execution.

Independent audit passes1,841checks: batch hashes/order, image locks, support
through an independent erosion implementation, native proposals, packet means
and covariance, fixed-ID membership, each successful fit's stationarity and
information inverse, center-reference pose errors/readouts, stationary bitwise
nulls and all attempt denominators. All72fits converge.

The9cases reuse three exposed textures; repeated arms are not independent
recording trials. One virtual timing profile, static planar radiance proxies,
identity raw/prepared-grid mapping and a pairwise IRLS harness limit transfer
claims. No significance, natural-camera noise, full-native-VO robustness or
field-wide novelty claim follows. The image adapter's documented integration
bounds/finite-history approximation also remain part of this result's limits.

## Connection to the papers and stopping decision

This addresses the evaluation/supervision extension in MACThermal and MAC-VO,
using known temporal support and a fixed pose timestamp. Existing
[TRNeRF](https://umautobots.github.io/trnerf) and
[TRGS-SLAM](https://arxiv.org/html/2603.20443v1) already model thermal image
formation; generic sensor-aware rendering/restoration is prior art.

The result motivates keeping the measurement model, supervision target and
uncertainty meaning explicit. It does not justify training covariance against
sharp-flow labels assigned to integrated pixels, multiplying weights until
pose error improves, or declaring a published covariance model defective.
A new method would need a specific mechanism and comparisons beyond these
established sensor-aware models. No such method is validated here.

Close this exact fixed-profile comparison without a motion/readout/time-constant,
query budget, solver, covariance or seed search. Preserve the positive cases
and complete availability. Other original-paper gaps remain open; this result
is one bounded empirical contribution, not their closure.

## Artifacts

Design/estimator freezes:`experiments/sensor-estimator-{design-freeze,freeze}-2026-11-14.json`.
Sources:`tools/sensor_estimator_comparison.py`,
`tools/audit_sensor_estimator_comparison.py`, `tools/summarize_sensor_estimator.py`.
Raw36dense batches,72packets/fit receipts, locks, metrics/CSV/figure and audit:
`results/sensor-estimator-comparison/`. No notebook files in repository/backup.
