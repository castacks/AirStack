# Existing learned frontend under the same monocular pose backend

Completed2026-11-16; protocol frozen15November2026. Full3495input images and3465
arm-pairs executed under the [prospective protocol](smokeseer-learned-protocol-2026-11-15.md).
No training, threshold/window selection, model rescue or new method. These are
exposed synthetic development results, not physical smoke or metric trajectory
evidence. This is not full MAC-VO/MACThermal reproduction.

## Decision: the SIFT gap does not survive this existing frontend control

Existing learned matching raises smokyRGBprimary success49.44→97.92%and thermal
72.12→98.53%over the same complete population. In nine identical-camera scenes,
the thermal advantage over smokyRGBshrinks from28.87to0.67percentage points.
Learned thermal resolves206of217prior shared rotation failures, including145of
the148whose supplied cleanRGBcounterparts succeeded. Only2/1046matched pairs
now fail rotation on both learned camera arms;7/1046fail the joint endpoint.

This largely closes the SIFT-based smoke-gap route as justification for a new
method on this dataset. The earlier substantial SIFT degradation remains valid
under that frontend; it is not a general failure beyond existing learned matching.
No additional weight/head/grid/threshold rescue is warranted by these results.
Physical smoke and independently qualified degradation/recovery remain separate
questions; this does not close thermal perception research or establish novelty.

The configuration is not universally better: cleanRGBprimary drops90.04→89.52%
overall, and cleanFicus drops80.73→40.37%. Preserve this regression. SmokyRGB's
higher success than supplied cleanRGBis a ranking reversal in this rendered
frontend/query/input configuration, not evidence that physical smoke improves
odometry. Its cause is not isolated; clean-camera binding remains a loader-based
assumption and query distributions differ between frontend families. No image or
model changes were tried to explain away that counterexample.

## Full population results

Primary: recovered pose and rotation error<=5degrees. Joint additionally requires
directed translation error<=15degrees. All unsupported pairs count as failures.

| Input | SIFT primary | Learned primary | Learned support | SIFT joint | Learned joint |
|---|---:|---:|---:|---:|---:|
| Smoky RGB | 571/1155 (49.44%) | 1131/1155 (97.92%) | 1149/1155 (99.48%) | 45.02% | 96.45% |
| Clean RGB | 1040/1155 (90.04%) | 1034/1155 (89.52%) | 1128/1155 (97.66%) | 87.62% | 88.83% |
| Thermal | 833/1155 (72.12%) | 1138/1155 (98.53%) | 1149/1155 (99.48%) | 67.97% | 98.27% |

| Input | Conditional median R / t error | Scene macro primary | Scene macro joint |
|---|---:|---:|---:|
| Smoky RGB | 0.469° / 1.454° | 97.82% | 96.32% |
| Clean RGB | 0.449° / 1.282° | 89.00% | 88.30% |
| Thermal | 0.479° / 1.160° | 98.54% | 98.30% |

Medians include only supported poses; all-pair denominators remain primary.
Adjacent pairs are not independent recordings; no significance claim follows.

| Scene | Pairs | SmokyRGB SIFT / learned primary | CleanRGB SIFT / learned primary | Thermal SIFT / learned primary |
|---|---:|---:|---:|---:|
| caterpillar* | 109 | 94.50% / 100.00% | 100.00% / 100.00% | 57.80% / 100.00% |
| chair | 109 | 33.03% / 95.41% | 100.00% / 99.08% | 59.63% / 100.00% |
| ficus_rgb | 109 | 21.10% / 99.08% | 80.73% / 40.37% | 59.63% / 98.17% |
| forest | 124 | 55.65% / 98.39% | 94.35% / 92.74% | 81.45% / 92.74% |
| hotdog | 109 | 12.84% / 86.24% | 59.63% / 74.31% | 8.26% / 97.25% |
| house | 149 | 69.80% / 100.00% | 100.00% / 100.00% | 99.33% / 100.00% |
| lego_smoke | 109 | 75.23% / 100.00% | 100.00% / 100.00% | 99.08% / 100.00% |
| mars | 119 | 63.87% / 100.00% | 100.00% / 100.00% | 99.16% / 100.00% |
| musclecar | 109 | 19.27% / 99.08% | 64.22% / 83.49% | 46.79% / 97.25% |
| room | 109 | 39.45% / 100.00% | 96.33% / 100.00% | 96.33% / 100.00% |

*Caterpillar modality camera paths differ; only within-arm SIFT/learned changes
are camera-matched there. House identity rotation passes every pair; elsewhere
identity passes none. Pooled identity success12.90%. Joint/direction and scene
reporting remain essential; do not retune the primary threshold.

## Nine identical-camera scenes

| Input | Pairs | Learned rotation success | Learned joint success |
|---|---:|---:|---:|
| Smoky RGB | 1046 | 1022 (97.71%) | 1005 (96.08%) |
| Clean RGB | 1046 | 925 (88.43%) | 917 (87.67%) |
| Thermal | 1046 | 1029 (98.37%) | 1026 (98.09%) |

## Previously identified shared failures

This is a descriptive stratum of the exposed development set, not selected
inference windows or a new held-out evaluation. Every declared pair was run.

| Endpoint | Prior SIFT both-fail pairs | Learned thermal successes within them | Learned smokyRGB successes within them | New learned both-fail pairs across all1046 | Learned cleanRGB succeeds within new shared failures |
|---|---:|---:|---:|---:|---:|
| Rotation | 217 | 206 | 198 | 2 | 1 |
| Joint | 269 | 254 | 238 | 7 | 1 |

Of the148prior cleanRGB-success/SIFT-shared-failure pairs, learned thermal
succeeds on145. Do not promote this development stratum to training targets
or certify a failure as uniquely smoke-induced from these labels.

## Scope of the comparison

The input grayscale bytes, camera model, E/recoverPose backend and endpoint
thresholds are unchanged. The complete correspondence frontend differs: SIFT
detected keypoints/mutual descriptors versus4096uniform queries, learned flow
and1pixel forward/backward cycles. Equal query caps do not equal compute or
identical selected points. No covariance weighting/ranking occurs; a difference
cannot be assigned uniquely to descriptors, covariance or sample placement.
Grayscale replication and omission of native thermal contrast enhancement bound
claims about the released networks; neither is a strongest-native-system result.
Training overlap, thermal radiometry, real-smoke transfer and render metres remain
unverified. Do not pool with supplied-reference real day/night VO results.

## Qualification and preservation

Full800×800FP32, batch2forward/reverse, unchanged12decoder iterations; one CPU
thread, deterministic algorithms and TF32disabled. Thermal checkpoint strict;
RGBuses only the previously audited seven-unused-key/fixed-buffer migration.
Checkpoint, model/config source, full3495gray input hashes, protocol and runner
were frozen before real pose predictions. Batch qualification passes6999checks;
analytic sampler check passes1004fractional/edge queries plus2cycle boundaries.
The initial sampler test failed because its expected array had shape(2,)rather
explicitly repeated(4096,2). Original source/log are preserved; the expected
fixture shape was corrected before freeze, with no inference/threshold changes.
No pose score was used to choose qualification inputs or change settings.

Journal records actual model-grouped order; canonical predictions retain exact
declared order and are locked before scoring. The original frozen SIFT scorer
is reused with only its result-directory adapter. Full sparse queries, flow,
covariance, cycle filters, accepted points, E/R/t and unsupported reasons survive.
Full dense fields are reproducible from inputs/checkpoints, not duplicated for
every pair. Packet hashes validate retained evidence, not full-field archival.

Independent audit passes97228checks and4guards across3465sparse packets, input/source pins,
filters, angular scores, supported-pose masks, canonical coverage, all scene
denominators and macro rates. The existing scorer and independent angle functions
are reused; no frozen SIFT source was modified. Execution took2288.67seconds (38.14minutes), including runner overhead. It is
recorded for this engineering run only, with no matched-cost or real-time claim.
The separate baseline/stratum comparison passes50independent set checks.

Raw results: `results/smokeseer-learned/`. Local packet:
`/root/ore-ms-artifacts/smokeseer-learned-2026-11-15/`. Original public ZIP and SIFT
packets remain separate and unchanged. No Git publication, Box upload or external
outreach occurred.
