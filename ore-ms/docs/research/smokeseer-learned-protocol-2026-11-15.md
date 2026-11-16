# Existing learned correspondence control on synthetic smoke

15 November2026. This follows the [SIFT failure diagnosis](smokeseer-relative-pose-findings-2026-11-15.md#post-result-failure-diagnosis).
The question is whether an existing learned frontend reduces the observed SIFT
failure, before proposing a new method. This is exposed development data; there
is no held-out generalization or novelty claim. No training, fitted gate or sweep.

## Population and fixed inputs

Reuse every3495qualified image and3465arm-pairs from the unchanged
[SIFT protocol](smokeseer-relative-pose-protocol-2026-11-15.md): all ten train
scenes,1155pairs in each of smokyRGB,cleanRGB,thermal. Nine scenes have identical
RGB/thermal camera poses; report Caterpillar separately. Preserve all failures.
Never select the148clean-success/shared-failure pairs as a new benchmark.

Use exactly the existing frozen800×800uint8grayscale arrays, repeated to3channels
and divided by255in FP32. Alpha and16-bit handling remain identical to the prior
preflight. No resizing, crop, CLAHE, thermal percentile normalization or RGBcolour
rescue. This controls input differences, but is not native MACThermal preprocessing
or a strongest-colour-RGB claim. No stereo depth, new camera fitting or IMU.

Use the existing RGB checkpoint for bothRGBarms and MACT_Offroad for thermal:
`MACVO_FrontendCov.pth` SHA256
`bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a`;
`MACT_Offroad.pth` SHA256
`eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a`.
Native FlowFormerCov model/config,12decoder iterations, FP32, no pretrain download,
eval/inference mode. Reuse `qualify_native_frontend.load_model`; no model copy.
Thermal must load strictly. RGB uses only the existing exact legacy migration:
remove seven known unused keys and restore the deterministic9×9sampling buffer.
No missing active learned parameter or unknown key is allowed.

## Fixed correspondence and geometry

4096source queries: cartesian64×64grid at integer coordinates
`rint(linspace(8,791,64))` on each axis. Every arm uses the same grid. Source flow
is indexed at those integer pixels; target is source+forward flow. Keep targets
inside[0,799]on each axis. Bilinearly sample reverse flow at these target pixels;
retain only points with Euclidean forward/backward cycle error<=1pixel. No
covariance threshold, ranking or weighting; retain sampled covariance as evidence.
Flow/covariance is finite and covariance positive for every full output array.

This replaces the whole correspondence frontend. SIFT uses detected keypoints and
mutual descriptor tests; the learned control uses a uniform grid and flow cycle
check. Equal4096query caps do not mean identical queries, work or uncertainty.
Do not attribute a difference solely to neural descriptors or covariance learning.

Pass retained points and unchanged K to the exact frozen `smokeseer_sift.pose`
function: minimum8points, RNGseed0, E-RANSAC probability0.999, threshold1pixel,
1000iterations, single3×3E, recoverPose with the E mask, minimum8cheirality
inliers. No retry, alternative estimator or post-result adjustment. Use the same
primary/joint errors and unsupported denominators as the SIFT protocol, including
identity rotation, conditional medians, scene macro and nine-matched-camera counts.
No physical thermal calibration, render metres or trajectory accuracy claim.

## Qualification, freeze and execution

First full-size forward calls for the first declared pair of each arm passed:
finite positive covariance, bitwise repeats, unchanged model state; no poses scored.
Next qualify the exact two-direction batch: source/target and target/source in one
batch of2. Require correct direction order, finite positive arrays, bitwise repeat
and unchanged model state. Batch2controls memory; no precision/resolution rescue.

Before predictions, recheck every frozen gray file/hash/dtype/size and binding,
all model/source/config/backend pins, and qualify sparse sampling on a known
translation field. Freeze code, protocol, checkpoints, query grid and inputs.
CPU one thread, seed20261114, deterministic algorithms, TF32disabled,
CUBLAS_WORKSPACE_CONFIG=:4096:8. Runtime exact restored image
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`.
Record batch runtime/peak memory; no comparative efficiency claim.

Process allRGBpairs with the RGBmodel, then all thermal with the thermal model.
Journal actual order; reorder once into the complete declared pair order before
locking predictions. No reference pose enters inference, point selection or E.
Scoring starts after all predictions are locked. Retain all sparse flow/covariance,
cycle masks, accepted points, E/R/t, masks and unsupported reasons. Full dense
fields need not be duplicated; inputs/checkpoints/source remain reproducible.

An API, memory, binding, finite-output or whole-span support failure stops this
configuration without scoring a successful prefix. Preserve interrupted runs.
No shrinking population, lower precision, replacement checkpoint, tuned cycle
threshold or extra model follows automatically. After one completed run, compare
all scene/pooled results and the predeclared shared-failure subgroup descriptively;
that subgroup remains post-result development evidence. Independently audit
coverage, filters, geometry errors, counts and hashes. No scene trimming.

## Interpretation

A learned improvement would show that some SIFT failures are addressed by existing
correspondence machinery, not a new contribution. Failure would close this fixed
frontend/query/input configuration, not all learned matching or thermal VO.
This is a monocular frontend-plus-common-backend control, not full MAC-VO or
MACThermal reproduction. Existing model training overlap and real-smoke transfer
remain unverified. Prior source reading/architecture and checkpoint qualifications
are reused; no new paper mechanism is claimed.
