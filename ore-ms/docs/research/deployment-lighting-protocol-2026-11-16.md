# Frozen first real-lighting comparison

16 November 2026. Selected before image-motion predictions. This is a finite
relative-pose and image-characterization study on NGCLAB desk1 halfsphere bright,
changing (`ic`) and dim, not a full trajectory VO or author-system reproduction.
It advances the deployment question without requiring algorithm novelty.

## Population and arms

Use all three complete bags selected in the [data assessment](rgb-thermal-deployment-data-2026-11-16.md).
Decode every RGB/thermal image and mocap pose before prediction. At the first RGB
header time and every 0.5 seconds thereafter, declare a source/target interval of
0.5 seconds while the target is within the RGB time span. Each arm selects nearest
images to each anchor (20 ms maximum error, unique ordered image indices). No
estimated clock offset. References use raw mocap header times, linear position
and quaternion SLERP, with a maximum 50 ms bracket. Record unsupported intervals
before prediction. Predict every image-supported interval, including those lacking
reference. Reference gaps are unscorable, not algorithm failures. Report both
conservative all-declared success lower bounds and the paired common-reference
accuracy population, defined before predictions.
No motion or image-quality selection. Bright is a required control.

Arms: RGB SIFT, thermal SIFT, pretrained MAC-VO RGB temporal frontend and pretrained
MACThermal Offroad temporal frontend. This uses the released matching networks
with shared monocular geometry, not their full native stereo systems. No fusion
or independent depth/IMU enters this first comparison. No training/tuning follows.
Use the same 4096 query budget, raw 640×480 resolution and common pose solver.

RGB input is OpenCV BGR-to-gray uint8. Thermal mono16 maps each frame's minimum
and maximum linearly to 0–255, rounding to nearest and clipping; constant images
map to zero and remain in the study. This follows the author's min/max display
recipe in spirit, with rounding explicitly specified; no CLAHE or learned thermal
preprocessing is added. Both frontends see the same prepared per-modality pixels.
Report that thermal normalization is a processing choice, not an inherent sensor
advantage. Preserve raw thermal range as an additional observable.

SIFT detection/matching and essential/recoverPose solver are imported unchanged
from `tools/smokeseer_sift.py`: 4096 features, mutual 0.75 ratio, 1-pixel RANSAC,
probability .999, 1000 iterations, fixed seed 0, minimum eight cheirality inliers.
Learned uses the existing checkpoint loader, FP32/no TF32, deterministic inference
in both temporal directions, fixed 64×64 grid eight pixels from native borders,
bilinear backward-flow sampling, one-pixel cycle acceptance. No uncertainty
threshold/search. Undistort accepted raw pixel correspondences with released
per-camera calibration into that same camera's virtual K before the shared solver.

## Reference/calibration contract

The author's extractor at commit `6d4bf30ac28d69e9e28f8f634185e0829f39ab9b`
writes `/vrpn_client_node/RigidBody/pose` directly to camera `groundtruth.txt`;
its README identifies this as RGB optical-center pose. Follow that supplied
contract, without fitting camera-to-mocap transforms. This is not independent
physical metrology. The paper's timing section describes small residual offsets
and leaves raw timestamps unchanged; we do the same, with no clock fit.

Use the downloaded visible–LWIR calibration ZIP. Its thermal-to-RGB rotation is
not exactly orthogonal. Preserve original bytes and report singular values,
orthogonality error, determinant and distance to closest SO(3). Explicitly use
that closest proper rotation with supplied translation for reference composition;
no fitting to image motion or pose outcomes. Stop if the Frobenius correction
exceeds 0.01 or det is nonpositive. Run synthetic transform/projection controls
before predictions. Sensitivity to this published export is a qualification
limitation. Raw distorted pixels are not undistorted images; do not rectify twice.

## Endpoints and stopping decision

Primary: joint rotation ≤5° and directed translation-angle ≤15° on the predeclared
common-reference population: both sensors have qualified reference and motion
>1 mm at that anchor. Report its membership before prediction. No algorithm failure
is removed from that population. Do not call a rotation-only interval joint success. Also report solver
coverage, rotation success, conditional rotation/translation medians and p90,
identity-rotation control, consecutive failures and sampled recovery, elapsed time
and image-observable distributions. Missing/failed estimates are failures, never
successful identity fallbacks. All-declared lower-bound denominators include all intervals; paired accuracy
denominators include the frozen common-reference intervals. Reference gaps are
reported separately. Recovery is a sampled pair-success diagnostic, not a
stateful estimator recovery time; do not infer continuity across reference gaps.

Score each sensor against its own exposure times and calibrated camera motion;
paired comparisons share anchor indices. Small nonidentical exposure times and
viewpoints prevent a pixel-perfect causal sensor attribution. Report per condition
and frontend; do not pool unlike recordings into a universal recommendation.
Relate error/failure to recorded brightness, contrast, saturation and gradient
statistics descriptively; no selector or fitted threshold is validated here.
These recordings have different motions, so the bright-to-dim change is not an
identical-trajectory causal effect. Successive intervals are dependent; no naive
pairwise significance or population-wide generalization claim.

Freeze runner, protocol, checkpoints, model implementation, prepared inputs and
reference manifests after preflight and synthetic/native-size runtime controls.
Inference receives only input pixels and calibration; scoring starts after all
predictions are hashed. Keep incomplete attempts. Stop after this cohort and audit;
retain gains, regressions and null results. Smoke needs its own geometry/reference
qualification and must not be relabelled from this lighting result.
