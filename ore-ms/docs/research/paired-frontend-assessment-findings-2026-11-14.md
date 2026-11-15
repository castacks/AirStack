# Paired frontend assessment: reselection increases perturbation sensitivity

14 November2026, America/New_York. Branch `ore-ms-orb-development`,baseline
`a79658f6341194b0954852226f200313ccdf5408` plus source-pinned working changes.

## Result

Reselecting source queries increases centered pose spread in all three tested
thermal pairs under the fixed half-LSB image-jitter ensemble. Relative to retaining
the original queries, baseline-information-whitened pose-variance trace rises
6.341x day,2.959x night and2.041x rain. All384paired perturbation trials succeed;
no failed-trial censoring explains these differences. This is a concrete effect
of the query-selection policy in this harness, not proof of a new uncertainty
algorithm, physical miscalibration or better/worse true camera motion.

The absolute changes matter: day/night translation shifts remain millimetric;
rain shifts are much larger. The experiment identifies a selection-sensitivity
boundary worthy of retaining. It does not justify automatically fitting a
covariance scale, changing NMS or training a stability head.

## Exact comparison and scope clarification

The [preceding design](pose-posterior-reliability-next-2026-11-14.md) proposed
frozen-versus-regenerated frontend assessment. Implementation makes its control
precise: **fixed source-query IDs**, rather than frozen numeric frontend outputs.
Freezing all outputs would give zero spread and would not be a meaningful paired
input-noise comparison. Both arms receive exactly the same newly inferred dense
flow, depth and covariance for each perturbed image set.

- Fixed-query arm retains the unperturbed native source pixel IDs, resamples
  current dense measurements there, then repeats range/covariance-sanity checks.
- Regenerated arm reruns the native uncertainty/NMS/threshold source selector,
  then uses the same current dense measurements and downstream checks.

This isolates the total effect of reselection, including varying accepted point
counts. It does not separately isolate matching changes from an entirely frozen
matcher, or prove a fixed-budget identity effect. Initialization is identity in
both arms, reflecting the static-motion pair harness; no initialization-noise
experiment was performed. The selector RNG is reset to the same seed for every
trial, so independent random resampling is not an extra noise source.

We restored the SHA-verified76,827,690-byte archived MACThermal checkpoint
`eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a`
and12exact thermal stereo images: frames000000/000001 from each already exposed
day/night/rain prefix. Native256x640 images use the existing thermal producer
preprocessing in OpenCV4.10. No extra records, depth/pose labels or training.

The native FlowFormerCov architecture loads all428state tensors strictly with
no migration,19,157,555parameters. Isolated PyTorch2.7.1+cu128,torchvision0.22.1,
NumPy1.26.4,timm0.9.12,pypose0.7.3 andSciPy1.13.1 qualify on the Blackwell GPU.
Two native-size three-pair batches repeat bitwise; all dense outputs are finite
and covariance positive. The generic AirStack image had no Torch; its failed
runtime probe is not a scientific result. This is a new qualified runtime,
not a historical-bytewise benchmark reproduction.

Native NoDepth selector and MatchCovariance call bodies are AST-extracted with
detached CPU inputs and unchanged arithmetic/configuration. Profiling/type/grad
decorators and unrelated plugin imports are omitted. Native covariance floors,
patch/kernel conventions and finite-covariance checks are retained. Source and
target stereo and temporal matching use the correct pair inputs; each left image
is shared between its stereo and temporal computations.

The pose stage is an explicit optical3D **IRLS registration harness**, using
C0+R C1 R^T refreshed between steps and frozen within each step. It uses identity
initialization and gradient-score stopping, and computes inverse frozen-weight
normal information at its final state. It does not include covariance derivatives,
a log determinant or the native robust/disparity-BA optimizer. It is therefore
not the full historical MACThermal pipeline or MAC-I² visual-inertial system.
This approximation is explicit in the freeze and shared by both arms.

## Frozen input ensemble and controls

For each of the four prepared grayscale views, add independent per-pixel
PCG64uniform noise in[-0.5/255,+0.5/255], clip to[0,1], then replicate the same
gray tensor over three channels. Reuse the exact same noisy left tensor across
its stereo and temporal calls. The half-LSB magnitude is a quantization-scale
controlled perturbation chosen before outcomes, not measured sensor noise.
No gamma/severity/frame/seed sweep follows these results.

There are128draws per case plus unperturbed baseline and identical-input repeat:
390dense three-pair batches (1170neural pair evaluations),780two-arm pose fits.
All780converge by the fixed frozen-weight score rule; the weights remain unchanged.
All three identical-input dense repeats and both-arm pose/covariance repeats are
bitwise equal. Twenty-four pre-ensemble shared mechanics checks include known
transform recovery, Jacobians, query/packet/fit repeatability and tangent transport.
No numerical output or threshold was changed after the execution freeze.

## Complete case results

|Case|Successful paired trials|Original query retention|Reselection/fixed variance trace|Mean translation shift, fixed / reselected mm|Mean rotation shift, fixed / reselected deg|
|---|---:|---:|---:|---:|---:|
|day|128/128|79.13%|6.341x|0.665 / 2.293|0.002011 / 0.005337|
|night|128/128|65.53%|2.959x|1.313 / 2.117|0.002099 / 0.004124|
|rain|128/128|81.71%|2.041x|256.332 / 366.780|0.138779 / 0.218374|

All query sets change in128/128trials per case. Accepted point counts are
145fixed versus132–155reselected day,59versus51–72night and41versus36–49rain.
Both means and covariances are recomputed at the affected points, as a selector's
normal downstream effect. This is not a claim that only individual point identity,
independent of count/geometry, explains the added variance.

Pose shifts are left-SE(3)log differences from the corresponding unperturbed
estimator output. That output is **not ground-truth motion**. “Mean shift” is
algorithm drift under this generator, not an estimate of physical pose bias.
Empirical covariance is centered with ddof1. The reported trace normalizes both
arms with the same baseline information covariance, keeping translation/rotation
units consistent. Each row is one original image pair with128conditional draws,
not128independent recordings or new real-world motion trials.

![Paired sensitivity and absolute shifts](../../results/paired-frontend-assessment/run-001/paired-frontend-sensitivity.png)

## Relationship to the computed covariance

The frozen-weight information covariance is transported through the SE(3)Log
Jacobian to the baseline-relative tangent before comparison. Secondary results:

|Case|Baseline-relative95%containment,fixed / reselected|Empirical/readout largest generalized variance ratio,fixed / reselected|
|---|---:|---:|
|day|100.00% / 100.00%|0.0115 / 0.0661|
|night|100.00% / 100.00%|0.0246 / 0.0695|
|rain|93.75% / 80.47%|3.5814 / 7.3997|

Containment asks whether the unperturbed estimator output falls inside a
Gaussian95%ellipsoid based on each perturbed estimate/readout. It is a
**baseline-relative perturbation containment statistic**, not true-pose coverage.
The generalized variance ratios compare centered empirical spread with the mean
transported readout, including directional effects. All six eigenvalues and full
matrices are saved, not only the largest.

Day/night jitter spread is substantially smaller than these information readouts;
extra spread from reselection therefore does not automatically imply an
underestimated covariance. Rain has a direction with larger empirical spread
and lower baseline containment, but the input ensemble is not the checkpoint's
training-error distribution or a measured physical sensor model. These numbers
establish behavior under the declared generator; they do not prove physical
calibration failure, total predictive error or failure of published MAC-I².

## Independent audit and retained failure

The independent auditor checks shared draw hashes, all regenerated NMS/median/
RNG selections, range admission, inverse projection, native kernel/3D covariance
arithmetic, all780frozen-weight fit/readout receipts and every sensitivity
statistic. Independent analytic SE(3)logs and matrix-exponential Jacobian transport
check the producer's finite-difference transport. **13,883checks pass**, with no
invalid packet and all three null dense repeats verified.

The first audit is retained as failed: its NumPy float32 depth division produced
slightly different means from the Torch float32 inputs used by the solver. A
selected source depth differed by3.814697265625e-6m; the near-zero gradient check
was correspondingly affected. The amended auditor still independently checks
backprojection, then evaluates stationarity on the **exact serialized inputs**.
Experiment outputs, scorer and acceptance tolerances are unchanged. The original
auditor/source freeze,failed log and amendment receipt remain available.

The largest absolute difference across all audited arrays is768 in a very large
raw3D covariance entry, not a pose discrepancy. The precision receipt identifies
its native/independent values:3,249,314,304 versus3,249,313,536, a relative
difference2.36e-7. Relative/absolute checks
are applied per quantity; high absolute entries must not be confused with metric
errors. The independent pipeline audit verifies arithmetic, not a second model
implementation or physical sensor truth.

## Disposition and reproducibility

Complete the single frozen assessment without retuning. Reselection is now an
observed source of additional input-perturbation sensitivity in these three
cases; matching itself also changes in the fixed-query arm. A later method would
need to distinguish selection-count, geometry, shared matching and depth effects,
and establish practical true-pose benefit plus a prior-work distinction. The
classical rematching/initialization covariance literature remains relevant; the
variance-ratio observation is not automatically a new algorithm.

Protocols: `experiments/native-frontend-qualification-freeze-2026-11-14.json`
and `experiments/paired-frontend-assessment-freeze-2026-11-14.json` (SHA256
`e4d7c9b45fc20ad54d1c7a533c162225885e9eb867fc5008895848aa415a7c06`,40pins).
Sources: `tools/qualify_native_frontend.py`, `qualify_paired_frontend.py`,
`paired_frontend_assessment.py`, `audit_paired_frontend.py`. Qualified package pins
are in `environment/native-thermal-frontend-qualified-requirements.txt`.
Results,full dense fields,point packets,fit receipts,locks,score arrays,PNG/PDF
figure and original attempt logs live under `results/paired-frontend-assessment/`
and `results/paired-frontend-qualification/`.
