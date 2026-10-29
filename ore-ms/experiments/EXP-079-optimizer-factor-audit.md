# EXP-079: saved optimizer residual and information audit

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `2c5ad073d`.

The six saved EXP073/074/077 experts support offline reconstruction of the
reprojection/disparity residual and a conditional six-dimensional information
matrix at each first optimizer writeback. This is an arithmetic/provenance audit,
not a quality-selector evaluation. No pose labels, model weights, images, new
inference, solver replay, fitting or reserved captures are used.

## Inputs and causal boundary

Six EXP073–078 recovery archives and all 1162 member SHA256 digests were verified;
1102 ignored artifact files were restored, refusing differing existing files.
Historical tracked source files were verified in their archives but not restored
over newer Git versions. Current source is the recovered EXP078 commit chain.

All 434 camera intervals are retained. There are 337 distinct first motion
writebacks with 57594 retained factor blocks, and 97 RGB intervals with no motion
write in the known middle-night support collapse. All six experts' last motions
arrive at termination after the original final decision deadline. Consequently
331 reconstructed motions were available at those deadlines. Terminal results
are reported separately, never backdated into earlier causal features.

For every interval, saved factor source/target pixels and depths match the raw
selected/inbound/retained rows; counts agree with original pair health. Exact
integer timestamps, graph indices, unique first write, preserved journal hashes
and pre-refinement sequence boundaries are checked. Native global optimizer
poses before the normal writer are used, rather than refined trajectory poses.
The audited config is the synchronous global `TwoFrame_PGO`, analytic `disp`,
with mapping disabled and `MotionInterpolate`. Other configurations are rejected.

The saved landmark positions are reconstructed from source pixels/depth and the
recorded causal parent pose. The source creates these points in FP32; the
independent reconstruction uses FP64. Largest normalized difference is
5.70e-7, within the explicit vector-norm roundoff budget
`64*float32_eps*(1+norm(local_point)+norm(parent_translation))`.
Largest absolute difference is **1.8163 m**: the saved inputs contain inferred
camera points as far as **9161310 m**. These are model-derived geometry, not
physical distance measurements. All retained factors remain present; no depth
clipping, reference-based filtering or claim of millimetre identity is made.
`MotionInterpolate` only receives/modifies frame storage; it does not update the
saved point/observation fields. This source boundary and the landmark/raw checks
support using those fields to reconstruct the observed solve objective offline.

## Reconstructed quantity

In native NED camera coordinates, `x` is forward depth. The three residuals are
`fx*y/x+cx-u`, `fy*z/x+cy-v`, and `fx*baseline/x-disparity`, in pixels.
Stored target flow covariance fills the 2x2 UV block, including its cross term;
stored target disparity variance is the third diagonal. No depth-variance
replacement or inferred UV/disparity cross covariance is introduced.

At the recorded native pose, landmarks are fixed and the camera receives a
right perturbation with translation in metres and rotation in radians. The
camera-point differential is `[-I, skew(p)]`. After differentiating projection,
the audit computes `H = sum(J.T @ C^-1 @ J)` by Cholesky whitening. Covariances
must be finite symmetric positive definite; invalid blocks fail the audit rather
than being clipped or silently excluded. The full 6x6 information matrix,
whitened Jacobian spectrum, numerical rank, mixed-unit condition number, raw
residual summaries and whitened sum/mean squares are preserved per motion.

This is **unrobust, undamped conditional Gauss–Newton information**. It excludes
landmark uncertainty/correlation, learned-error calibration, marginalization,
Huber/FastTriggs corrections, LM damping and the optimizer's actual iteration
history. It is not the native robust optimizer Hessian or calibrated pose
covariance; neither its inverse nor a fabricated convergence flag is reported.
Whitened loss is not asserted to follow a chi-square distribution. Conditioning
mixes metre/radian coordinates, depends on their scale, and is not a universal
cross-camera score or proof of accuracy. The physical body frame remains outside
this camera-local audit. Computation from saved files is not measured online
latency, and journals only establish write-observation timing.

## Descriptive results

Medians below cover **all first writebacks**, including each terminal write.
Each residual column is a median of per-motion medians; whitened mean is the
median of per-motion whitened sum squares divided by three times factor count.
The two RGB middle-night motions are a very small, selected surviving population.

| Saved window | Camera | Motions | No write | Reprojection px | Absolute disparity px | Whitened mean/component | Mixed-unit condition |
|---|---|---:|---:|---:|---:|---:|---:|
| Early night | RGB |19|0|0.1203|1.4159|0.3858|10.39|
| Early night | Thermal |19|0|0.0345|0.1392|0.0277|12.32|
| Middle night | RGB |2|97|6.8328|38.3873|4.7413|5913.41|
| Middle night | Thermal |99|0|0.2871|0.2236|0.1068|35.82|
| Middle day | RGB |99|0|0.4260|0.1329|0.9784|14.45|
| Middle day | Thermal |99|0|0.2318|0.1823|0.3422|15.81|

All 337 matrices have numerical rank six under the standard SVD floating-point
threshold. Full numerical rank therefore does not identify whether an estimate
is accurate. The middle-night RGB survivor summaries have larger residuals and
worse conditioning, but this audit does not establish a ranking advantage,
threshold or recovery rule. Early/middle night reuse the same recording; nearby
intervals and the six experts are not independent scientific trials.

## Validation and reproduction

- 15 CPU tests pass: zero/nonzero residuals, finite-difference tangent Jacobian,
  correlated covariance, variance scaling, rank deficiency, rotated pose,
  invalid depth/pose/covariance and asymmetric covariance rejection.
- An independent validator imports neither producer nor estimator. It checks
  every slot and all 57594 factor blocks, reconstructs landmarks and residuals,
  and differentiates a separate forward perturbation numerically. Maximum
  relative Frobenius information-matrix difference is **3.52e-11** (rounded up).
- Seven semantic mutations are rejected: changed information, changed native
  pose, omitted interval, incorrect deadline availability, changed whitened loss,
  invented pose covariance and fabricated motion diagnostics.
- A runtime file-access guard blocks proxy/reference/calibration/model paths;
  recomputation yields identical rows and input hashes. 471 input/source hashes
  are checked before and after extraction. No original source artifact changes.

CPU execution used the existing AirStack robot image
`sha256:ae59352813af040d1a60000f2b13eed990b8364f8a416f5b87e385d4496aade7`,
Python3.12, NumPy1.26.4, SciPy1.17.1, `OPENBLAS_NUM_THREADS=1`, Docker's `runc`
runtime and `NVIDIA_VISIBLE_DEVICES=void`. No GPU or neural runtime was used.
The repository's jaxtyping pytest addopts were disabled for this standalone
NumPy-only fixture; no estimator tests are represented as rerun here.

From the MAC-VO root in that CPU container, with ore-ms mounted at `/workspace`:

```bash
python3 -m pytest -o addopts= Scripts/UnitTest/test_optimizer_factor_audit.py -q
python3 -m Scripts.Setup.audit_optimizer_factors \
  --root /workspace --output /workspace/results/optimizer-factor-audit/<fresh-run>
python3 -m Scripts.Setup.validate_optimizer_factors \
  --root /workspace --run /workspace/results/optimizer-factor-audit/<fresh-run>
```

Authoritative output: `results/optimizer-factor-audit/run-005/{audit.json,
validation.json,mutations.json,label-guard.json}`. Earlier run folders are
non-authoritative development attempts. Recovery target:
`backups/2026-10-29-exp-079`; dependencies are EXP073/074/077 saved experts.

## Decision

Saved optimizer factors can support read-only residual and conditional local
information diagnostics without another forward or solver run. They do not
recover missing convergence telemetry or calibrated pose covariance. Keep all
missing/deadline states and use these as diagnostics only. Before a gate is
proposed, separately predeclare a matched-support comparison against the existing
qualified pose proxy, preserve both cameras and availability/history controls,
and evaluate whether these summaries add information beyond the existing
signals. No score, threshold, default or scientific accuracy claim is adopted.
