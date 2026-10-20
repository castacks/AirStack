# EXP-011: Native replay with per-channel pixel precision mass preserved

Date: October 19, 2026 (America/New_York). Branch `ore_ms`; base commit
`3ac07f047` plus EXP-010/EXP-011 working changes. **Completed.**

## Result and decision

The late thermal translation improvement persists when each pair's original
u/v precision totals are preserved: recorded-SLAM disagreement RMS is
2.8966 m learned, 0.7898 m fixed_uv, and 0.7979 m fixed_uv_mass.
The associated rotation penalty also persists: 1.7129, 4.3745, and 4.2469 degrees.
Thus increasing total pixel precision is not necessary for this improvement
conditional on the saved inputs and native solver. Redistributing precision
across features can reproduce it at the original per-channel totals.

This does not isolate an incorrect image region, certify a better uncertainty
model, or justify changing defaults. Robustification and nonlinear solver
behavior interact with redistribution. Keep both translation and rotation
visible; no scalar correction, mask, preprocessing, calibration or VO default
has changed. These are independent fixed-prior pair replays, not fresh
end-to-end VO trajectories or independent dataset validation.

## Frozen protocol

Before new solves, `prepare` recorded the three arms, tolerances, prior
EXP-009 constants, Torch/thread settings and 48 input/implementation hashes.
All three windows (early, original, late) and both modalities are included.
No reference poses or error metrics choose weights.

- `learned`: original active covariance, including existing floors.
- `fixed_uv`: EXP-009's frozen per-modality u/v variances; original disparity.
- `fixed_uv_mass`: for each pair and channel independently, set every pixel
  variance to `N / sum_i(1 / learned_variance_i)` (the harmonic mean).
  This preserves the sum of inverse variance in each channel before Huber;
  original disparity variance and zero uv cross terms remain unchanged.

Disparity, source points/depths, K, baseline, observations, match-to-point
associations and the original static initializer are identical across arms.
Per-pair nonpixel fingerprints include disparity. No depth cap is applied.
All retained features, including depth tails, are kept. Constants are computed
in float64 and stored in the original covariance dtype; numerical conservation
is checked within relative tolerance 1e-6.

The native analytic disparity graph uses double-precision CPU LM/PINV,
Huber/FastTriggs delta 0.1, trust radius 1e3, LM minimum 1e-6 and unchanged
StopOnPlateau settings (10 steps, patience 2, decreasing 1e-5). One CPU thread
and seed 0. Each pair retains its baseline prior and world points; its solved
increment is composed from identity without changing the next pair's input.
Source inputs and all frozen hashes are verified before and after evaluation.

Both controls must match their EXP-009 saved local estimates within
1 mm / 0.01 degree. Learned must also replay the original source motion within
those tolerances. Reference trajectories are generated with the existing
recorded odometry interpolation, supplied extrinsics, rectification and
optical/NED conversion at actual modality acquisition times. Evaluation uses
first-pose anchoring only; no fitted SE(3) alignment or scale.

## Accumulated translation and rotation

Translation RMS disagreement with recorded SLAM, meters:

| Window | Modality | Learned | Fixed uv | Fixed uv, mass preserved |
|---|---|---:|---:|---:|
| early | rgb | 0.0904 | 0.0891 | 0.0892 |
| early | thermal | 0.7027 | 0.4515 | 0.4119 |
| original | rgb | 0.1277 | 0.1265 | 0.1274 |
| original | thermal | 1.5171 | 1.2878 | 1.3410 |
| late | rgb | 0.0995 | 0.1000 | 0.1002 |
| late | thermal | 2.8966 | 0.7898 | 0.7979 |

Rotation RMS disagreement with recorded SLAM, degrees:

| Window | Modality | Learned | Fixed uv | Fixed uv, mass preserved |
|---|---|---:|---:|---:|
| early | rgb | 0.3833 | 0.3671 | 0.3689 |
| early | thermal | 4.1763 | 4.3824 | 4.7692 |
| original | rgb | 0.3108 | 0.1876 | 0.1914 |
| original | thermal | 5.6879 | 5.4611 | 5.8532 |
| late | rgb | 0.3276 | 0.2558 | 0.2553 |
| late | thermal | 1.7129 | 4.3745 | 4.2469 |

The mass-preserving arm worsens thermal accumulated rotation in all three
windows relative to learned. RGB changes are small in absolute translation
on these windows. These do not establish a general modality ranking.

## Local motion and precision checks

Consecutive local translation/rotation RMS (each cell is **meters / degrees**):

| Window | Modality | Learned | Fixed uv | Fixed uv, mass preserved |
|---|---|---:|---:|---:|
| early | rgb | 0.0439 / 0.2308 | 0.0440 / 0.2315 | 0.0440 / 0.2315 |
| early | thermal | 0.1169 / 1.2684 | 0.1816 / 1.2505 | 0.1710 / 1.2488 |
| original | rgb | 0.0309 / 0.1038 | 0.0308 / 0.1034 | 0.0308 / 0.1036 |
| original | thermal | 0.1415 / 0.7869 | 0.1250 / 0.7452 | 0.1294 / 0.7682 |
| late | rgb | 0.0256 / 0.0861 | 0.0256 / 0.0834 | 0.0257 / 0.0832 |
| late | thermal | 0.3447 / 0.3574 | 0.1485 / 0.6108 | 0.1462 / 0.6100 |

Late thermal local translation improves 0.3447→0.1462 m with mass preserved,
while local rotation worsens 0.3574→0.6100 degrees. Early thermal accumulated
translation improves, but consecutive translation worsens 0.1169→0.1710 m;
a smaller accumulated RMS is not a uniform per-step gain.

The old fixed_uv arm's median late-thermal total precision ratios relative to
learned are 2.8432 (u) and 2.3935 (v). The new arm preserves both totals within
maximum relative difference 5.69e-8 across all 114 pairs, below the declared
1e-6 tolerance. It changes distribution while maintaining each pair's original
u:v:disparity aggregate precision relationship **before Huber**. It does not
preserve robust effective precision, individual feature weights, solver steps,
or the covariance-informed selection that originally produced the matches.

## Validation, runtime and restoration

All 342 solves complete. Both controls match EXP-009 local motions to maximum
2.23e-16 m and 3.74e-16 degrees; every learned replay matches the original source
within 1 mm / 0.01 degree. Pose and loss finite checks, quaternion checks,
nonpixel fingerprints and all 48 frozen file hashes pass. Source files remain
unchanged. All six original EXP-009 implementation hashes also match.

Eight new test cases and thirteen focused regression cases pass (21 total):
channelwise precision conservation, uniformity, dtype/nonpixel preservation,
unchanged uniform inputs, existing control rules, invalid variance/cross-term/
disparity rejection and native synthetic motion recovery, plus backend,
weight-ablation and EXP-010 diagnostics. No GPU inference was required.

Runtime: `macvo:replay-cpu`, image
`sha256:bf5d89c63af6c867c49aeee4db2bb4ef1486c1a65ac026521de35fb8495a1cc2`,
Torch 2.7.1+cpu, PyPose 0.9.5, NumPy 2.2.6, SciPy 1.17.1, Python 3.11.
The new `Docker/Dockerfile.replay` pins its base and principal math packages.
`TORCHDYNAMO_DISABLE=1` runs eager math, corresponding to the earlier missing-
compiler fallback. Native source is unchanged; numerical replay checks above
validate the runtime change. Full package freeze and image ID are archived.

Restored only five metadata files from `1_salon_mac_cov_day.zip` in Box folder
`428559630945` using authenticated byte ranges: two camera-info files and three
recorded odometry files. Total range payload 8,744,936 bytes. ZIP CRCs pass for
those five members; all three odometry SHA-256 values match EXP-009 provenance.
The complete 8.9 GB ZIP was not downloaded or rehashed. Selected file hashes
and byte ranges are retained in `metadata-restore-manifest.json`.

Artifacts: `results/precision-mass/run-001/` contains frozen `plan.json`,
`analysis.json`, all eighteen arm/window/modality motion NPZ files, per-case
analyses, `summary.csv`, `precision-mass-comparison.png/pdf`, package freeze,
image ID, test and replay logs, and execution/restoration provenance. Commands
are in the setup README. Existing result directories are refused; use a fresh
output root for reruns. Weights/data/credentials remain outside Git.

## Limits and next controlled measurement

These are three dependent windows of one clip, with selected features,
provisional thermal signed geometry, unknown original thermal preprocessing
and recorded SLAM rather than independent ground truth. The fixed-prior
construction does not test propagation into later matching, depths or priors.
Precision conservation is pre-Huber; the replay does not isolate robust
correction, stopping or scene geometry as the cause. No statistical significance,
calibrated uncertainty, adaptive fusion benefit or default replacement follows.

A bounded next test is channel-specific redistribution: preserve precision
mass while making only u uniform or only v uniform, retaining learned and
both-channel controls across the same six cases. This can separate channel
contributions before region masks. Declare this before evaluation and retain
both local/accumulated translation and rotation. An independent clip and
resolved geometry remain necessary before adopting a method.
