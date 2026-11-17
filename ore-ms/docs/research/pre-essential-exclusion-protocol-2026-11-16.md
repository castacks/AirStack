# Fixed image-only exclusion before essential fitting

16 November2026 America/New_York. Follows the
[inlier-mixture diagnosis](essential-inlier-mixture-findings-2026-11-16.md).
Known exposed outcomes; engineering ablation, not blind validation or novelty.
No learned inference, feature matching, source-pose/reference adjustment or training.

## Fixed mask recipe and qualification before accuracy

Four variants, unchanged coordinates/order of remaining matches:
1. original: all cached accepted matches;
2. zero: RGB keep pixels nonzero in ANY of five own-record gray images, chosen
   at rounded linspace(0,image_count-1,5) ordinal positions; thermal all keep;
3. bottom: keep native pixel rows y<2*height/3, both cameras;
4. combined: intersection of zero and bottom keeps.

Use each recording/camera's complete sorted image manifest, not previous error
samples, to choose five frames; verify file SHA, dtype/shape/timestamp. Lock all
mask arrays, sampled IDs and qualification sheets before any refit/scoring.
Persistent zero is a candidate margin exclusion; truly dark valid scene pixels
can be removed. The bottom-third rectangle also removes real scene geometry and
may miss upper rig pixels. No physical invalidity or rig-segmentation certificate.
Do not alter width/row/zero threshold, sample positions, morphology or recipe from
visual or error outcomes. Save kept-pixel fractions and fixed image overlays.

Map each original virtual endpoint through own-record calibration to native
coordinates with frozen native/canonical helper. Nearest pixel is floor(x+0.5),
clipped only for boundary rounding after the existing0.05pxvalidity guard. A match
survives only if both source and target mask pixels keep it. No flow/pose/error
value decides inclusion. Save per-row retained counts and SHA256 of ascending
little-endian int64 original correspondence indices. Empty/insufficient subsets
remain explicit. Preflight all5516original rows/22064variant rows before refits.

## Runtime gate and unchanged fitting

Use recovered exact image
sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09,
CPU only, OpenCV5.0.0/NumPy2.2.6, one OpenCV thread, original pose helper.
No GPU/model calls. Replay all5516unmasked rows first. Every pose field present
(status/matches/E/essential mask/cheirality count/R/t/pose mask/error) must match
the original exactly, including arrays; timing/other frontend metadata excluded.
Unsupported inputs retain original input_unsupported and make no solver call.
Stop a failed parity gate without relaxing it or executing masked fits.

After parity passes, fit all three masked variants for both complete records,
frontends and cameras using existing pose(): same seed0, minimum8matches,
findEssentialMat RANSAC prob.999/1px/1000iterations, unchanged recoverPose and
minimum8cheirality inliers. Rejected old poses remain eligible for recomputation
when original inputs qualified; do not choose only previously successful rows.
Identical retained-index lists reuse that row's original or earlier-variant pose
result, with explicit reuse/solver-call receipts; no accuracy-dependent caching.
This filters already-generated matches, not equivalent to masking the network
input or redetecting features. Remaining points retain original sorted order.

Lock all22064geometry rows, then status-only choices for RGB/thermal/RGB-first/
thermal-first, before parsing reference score values. Keep camera/source/target/
times and directed camera translation origins. No post-mask alignment, selection
by error, output rescue, variant selection or mask/threshold sweep.

## Fixed endpoints, audit and stopping rule

Same all-declared/common-reference/both-camera≥2°/both-displacement≥1cm cohorts
and original index thirds. Per record/frontend/variant/policy report outputs,
R≤1°,R≤5°,jointR≤5°/directed-t≤15°, missing counts; failures in denominator.
Primary contrast: combined versus original learned fixed-camera and RGB-first
results on moving R1/R5 and translation joint, in EACH record. zero/bottom arms
are fixed factorial controls; preserve SIFT/opposite-camera results. Also report
newly correct/lost correct/new output/lost output on paired identical anchors.
No pooled accuracy, significance/generalization or end-to-end compute claim.
An observed gain is scoped development evidence; no automatic deployment.

Independent audit rebuilds mask arrays from image pixels, sample identity/hash,
retained-index counts/hashes/order, source/status/mask/pose consistency, locks,
original parity and full own-reference errors/choices/aggregate/transitions.
Numerical E/R/t consistency is checked for every recovered geometry; no need to
repeat all masked RANSAC fits independently. Synthetic controls cover both-endpoint
retention, nearest pixel/bottom boundary, image-only union and null/failed cases;
rehashed semantic corruptions must fail. Qualification/source/code/runtime pins
are saved. Reuse native/pose/priority helpers without editing frozen runners.

Prior local matching-mask and keypoint border-mask contracts and targeted official
OpenCV Feature2D mask documentation were checked. Pixel/ROI exclusion is established
engineering, not a new method; current post-match filtering is distinct from
feature-detection masking. Keep MAC-VO/MACThermal native stereo systems separate.

Save all attempts, parent pins, masks/plans/parity/refits/choices/scores/audits,
findings and a member-readback verified local incremental packet. Stop at results,
including regressions/nulls. No mask redesign, thresholds, training, author outreach,
Box upload, Git push, automatic commit or physical-truth/adoption claim follows.
