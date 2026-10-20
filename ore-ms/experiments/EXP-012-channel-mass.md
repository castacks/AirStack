# EXP-012: Channel-specific mass-preserving native replay

Date: October 19, 2026 (America/New_York). Branch `ore_ms`; base commit
`3ac07f047` plus EXP-010/011/012 working changes. **Completed.**

## Result and decision

In late thermal, u-only mass-preserving redistribution reduces accumulated
translation disagreement from 2.8966 m to 1.4812 m, with rotation increasing
from 1.7129 to 2.0122 degrees. V-only reduces translation less, to 2.4742 m,
while rotation rises to 4.0743 degrees. Both-channel redistribution retains
the strongest translation change (0.7979 m) and a large rotation penalty
(4.2469 degrees). These conditional native-replay effects are not additive.

The channels are image coordinates: u horizontal, v vertical, not camera NED
axes. Both constrain all pose components. This does not identify a bad image
region or the physical cause of drift. U-only has a more favorable late-window
translation/rotation tradeoff than both-channel redistribution in this replay;
no covariance calibration or general method superiority is established.
Keep defaults and validate the frozen u-only rule on a separate clip before
further tuning or adopting a replacement.

## Question and frozen protocol

EXP-011 retained most of the late thermal translation gain when redistributing
both pixel channels without increasing their total precision, but retained
the rotation penalty too. Separate the u and v contributions before trying
spatial masks, preprocessing changes or fusion.

Four arms use original active covariance including its floors:

- `learned`: original covariance.
- `fixed_u_mass`: u variance becomes its pairwise harmonic mean; v unchanged.
- `fixed_v_mass`: v variance becomes its pairwise harmonic mean; u unchanged.
- `fixed_uv_mass`: both variances become their respective pairwise harmonic
  means, reproducing the EXP-011 control.

For each selected channel use `variance = N / sum_i(1 / learned_variance_i)`.
Each channel's sum of inverse variance is preserved before Huber, within
relative 1e-6; unselected pixel channels remain bitwise unchanged. No reference
data enters the weight rule. Disparity, cross terms, observations, selected
matches, source depths/points, K, baseline and prior initialization are fixed.
No depth cap, new inference, fitted scale or trajectory alignment is applied.

All three windows (early, original, late) and both modalities use the unchanged
analytic native LM/PINV, Huber/FastTriggs delta 0.1, trust radius 1e3, LM
minimum 1e-6 and StopOnPlateau settings (10 steps, patience 2, decreasing 1e-5).
The existing `macvo:replay-cpu` image runs Torch 2.7.1+cpu, PyPose 0.9.5,
NumPy 2.2.6 and SciPy 1.17.1, seed 0, one CPU thread, eager math.

All pairs retain their original frozen priors/world points. Independently
solved increments are composed for diagnosis; a new arm does not alter later
pair inputs. These are not new end-to-end VO runs. Recorded odometry uses
the same calibration, rectification and timestamp interpolation as EXP-011.
Evaluate translation and rotation both locally and with first-pose anchoring;
no reference or evaluation error chooses weights.

`prepare` checks the full EXP-011 frozen file set and pins the prior outputs,
new implementation, protocol and settings before new real solves. Learned and
both-channel controls must reproduce EXP-011 local motions within
1 mm / 0.01 degree. Source/frozen file hashes are checked before and after,
and every pair checks mass, unchanged channels and nonpixel fingerprints.

## Accumulated and local disagreement

Each cell below reports **translation RMS in meters / rotation RMS in degrees**
against the same recorded-SLAM proxy. First-pose anchoring, no fitted scale or
best-fit trajectory alignment. These six segments are dependent windows of
one clip, not six independent datasets.

Accumulated trajectory disagreement:

| Window | Modality | Learned | U-only, mass preserved | V-only, mass preserved | Both, mass preserved |
|---|---|---:|---:|---:|---:|
| early | rgb | 0.0904 / 0.3833 | 0.0892 / 0.3715 | 0.0904 / 0.3800 | 0.0892 / 0.3689 |
| early | thermal | 0.7027 / 4.1763 | 0.5321 / 3.1717 | 0.3779 / 4.9683 | 0.4119 / 4.7692 |
| original | rgb | 0.1277 / 0.3108 | 0.1270 / 0.2056 | 0.1283 / 0.3010 | 0.1274 / 0.1914 |
| original | thermal | 1.5171 / 5.6879 | 1.3619 / 5.1544 | 1.4686 / 6.2595 | 1.3410 / 5.8532 |
| late | rgb | 0.0995 / 0.3276 | 0.1023 / 0.2634 | 0.0980 / 0.3177 | 0.1002 / 0.2553 |
| late | thermal | 2.8966 / 1.7129 | 1.4812 / 2.0122 | 2.4742 / 4.0743 | 0.7979 / 4.2469 |

Consecutive local-motion disagreement:

| Window | Modality | Learned | U-only, mass preserved | V-only, mass preserved | Both, mass preserved |
|---|---|---:|---:|---:|---:|
| early | rgb | 0.0439 / 0.2308 | 0.0440 / 0.2309 | 0.0439 / 0.2312 | 0.0440 / 0.2315 |
| early | thermal | 0.1169 / 1.2684 | 0.1346 / 1.2510 | 0.1701 / 1.2811 | 0.1710 / 1.2488 |
| original | rgb | 0.0309 / 0.1038 | 0.0307 / 0.1024 | 0.0310 / 0.1050 | 0.0308 / 0.1036 |
| original | thermal | 0.1415 / 0.7869 | 0.1300 / 0.7604 | 0.1387 / 0.7880 | 0.1294 / 0.7682 |
| late | rgb | 0.0256 / 0.0861 | 0.0257 / 0.0834 | 0.0256 / 0.0856 | 0.0257 / 0.0832 |
| late | thermal | 0.3447 / 0.3574 | 0.2531 / 0.5134 | 0.3117 / 0.6314 | 0.1462 / 0.6100 |

Thermal accumulated rotation improves under u-only in early/original windows,
but worsens in late. V-only worsens accumulated rotation in all three thermal
windows. Even u-only is not a uniform local-motion improvement: early local
translation worsens 0.1169→0.1346 m despite accumulated translation improving
0.7027→0.5321 m. Late local rotation worsens 0.3574→0.5134 degrees under u-only.
The both-channel translation gain exceeds either single-channel gain in late,
so simple additive attribution of translation/rotation effects is not valid.
Small absolute RGB translation changes do not imply a general modality ranking.

## Validation and artifacts

Ten new test cases and twenty-one focused regression cases pass (31 total):
targeted-channel uniformity, preserved mass/dtype/cross terms/disparity,
bitwise unchanged other channel, exact predecessor controls, synthetic native
motion recovery and invalid variance rejection. These validate implementation;
real-data findings follow only after replay controls pass.

All 456 native pair solves (24 arm/window/modality trajectories) complete.
Learned and both-channel controls reproduce EXP-011 local motions to maximum
2.23e-16 m / 3.74e-16 degrees. Learned also matches the original trajectories
within the declared 1 mm / 0.01 degree local tolerances. Every selected channel
is uniform; every unselected channel is bitwise unchanged. Nonpixel hashes,
including disparity, agree across arms. Poses, losses and quaternion checks pass.
Maximum precision-mass relative difference is 5.69e-8 (limit 1e-6). All 63 frozen
file hashes match before and after replay, including the entire predecessor
input/implementation set and its control outputs. Original outputs are unchanged.

`results/channel-mass/run-001/` contains frozen `plan.json`, `analysis.json`,
24 motion NPZ files, six per-case analyses, `summary.csv`,
`channel-mass-comparison.png/pdf`, and logs/runtime provenance. The existing CPU
image ID is `sha256:bf5d89c63af6c867c49aeee4db2bb4ef1486c1a65ac026521de35fb8495a1cc2`.
No runtime rebuild, new reference extraction, model inference or data download
was needed. Commands are in the setup README; the original EXP-011 source,
plan, inputs and outputs remain unchanged.

## Limits

Same-clip dependent windows, covariance-selected features, provisional thermal
signed geometry, unknown original thermal preprocessing, and recorded-SLAM
proxy rather than independent ground truth remain limitations. Precision is
preserved before Huber, not necessarily after robust correction. Channel
redistribution can interact nonlinearly with LM damping and stopping. An arm's
translation/rotation effect is conditional on these fixed inputs and does not
establish calibrated confidence, a bad image region or a replacement default.


## Next validation gate

Freeze the u-only harmonic-mean rule unchanged and compare learned, u-only and
both-channel controls on a separate clip using the same translation/rotation
and replay checks. Do not choose additional mixing strengths or region masks
from these evaluated windows. Resolve signed thermal geometry and reference
quality before making accuracy or cross-modal comparability claims. If another
clip is not accessible, treat that as the next data requirement rather than
calling additional same-clip tuning independent validation. This is a proposed
follow-up; no separate-clip result is claimed here.
