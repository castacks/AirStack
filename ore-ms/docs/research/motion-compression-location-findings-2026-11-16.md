# Compression precedes rotation-branch selection, with mixed match patterns

16 November 2026, America/New_York. Completed the
[frozen cache-location protocol](motion-compression-location-protocol-2026-11-16.md)
after the [rotation diagnostic](rotation-collapse-findings-2026-11-16.md).
All1,712moving rows remain:192FlashLight/236SmokeRoom anchors, both cameras
and both frontends. No new matches, essential matrices, RANSAC, pose optimization,
training, threshold tuning or replacement camera policy.

## Decisive result: changing the rotation branch cannot fix wrong returned poses

For each cached finite3×3essential matrix E, both rotation candidates were
inspected. Original recovered R agrees with one candidate in every supported
case. Among all146learned moving supported errors at5° (7FlashRGB,58Flashthermal,
40SmokeRGB,41Smokethermal), zero have an alternative rotation within5°.
The same zero-alternative finding holds at1° and for the SIFT supported errors.
Thus the wrong rotation is already encoded in the selected E; choosing its other
rotation branch cannot repair these existing outputs. This isolates one recovery
step without assigning the cause of the wrong E.

Official [OpenCV5 decomposition documentation](https://docs.opencv.org/5.0/main_modules/d_projection.html#decomposeessentialmat)
was checked: E provides two rotation candidates and two translation signs, with
first-camera→second-camera direction. The frozen local `smokeseer_sift.pose`
uses unchanged findEssentialMat/recoverPose calls. Only the rotation candidates
are scored here; translation-sign choices and new pose generation are not evaluated.

| Recording | Frontend/camera | Outputs / moving anchors | E available | Original R≤5° | Either E rotation≤5° | Wrong output with correct other rotation |
|---|---|---:|---:|---:|---:|---:|
| FlashLight | learned rgb | 192/192 | 192 | 185 | 185 | 0 |
| FlashLight | learned thermal | 168/192 | 188 | 110 | 111 | 0 |
| FlashLight | sift rgb | 105/192 | 192 | 33 | 45 | 0 |
| FlashLight | sift thermal | 134/192 | 148 | 110 | 124 | 0 |
| SmokeRoom | learned rgb | 221/236 | 236 | 181 | 181 | 0 |
| SmokeRoom | learned thermal | 231/236 | 236 | 190 | 191 | 0 |
| SmokeRoom | sift rgb | 162/236 | 236 | 85 | 106 | 0 |
| SmokeRoom | sift thermal | 134/236 | 161 | 100 | 115 | 0 |

An E can have a correct rotation while its proposal was rejected by the unchanged
support/cheirality policy. This accounts for the one additional learned thermal
candidate success in each recording and the larger SIFT candidate counts. They
are label-assisted diagnostics on rejected proposals, not achieved new outputs
or evidence that relaxing support gates is safe. No failed proposal was promoted.

## Accepted matches already show low motion in many RGB failures

All values below are medians of per-pair medians, not one pooled collection of
matches. Cells are the prior learned movingR≤5°correctness strata. n counts pairs
with a defined metric; masks can contain points even when the proposal has fewer
than8cheirality inliers and is rejected. Accepted and cheirality columns can
therefore have different populations, and neither n is a pose-availability count.

Reference epipolar residual is the angular distance of the target bearing to the
plane Eref·source, with Eref=[tref]×Rref from the own nominal camera-relative
reference. This includes translation direction rather than assuming pure rotation.

| Recording | Correctness cell | Camera | Accepted n | Raw ray change | Reference plane residual, accepted | Cheirality n | Reference plane residual, cheirality |
|---|---|---|---:|---:|---:|---:|---:|
| FlashLight | both_correct | rgb | 107 | 6.358° | 0.345° | 107 | 0.225° |
| FlashLight | both_correct | thermal | 107 | 7.209° | 0.357° | 107 | 0.306° |
| FlashLight | rgb_only_correct | rgb | 78 | 16.116° | 0.377° | 78 | 0.276° |
| FlashLight | rgb_only_correct | thermal | 75 | 0.082° | 7.503° | 64 | 4.727° |
| FlashLight | thermal_only_correct | rgb | 3 | 2.431° | 0.431° | 3 | 0.266° |
| FlashLight | thermal_only_correct | thermal | 3 | 5.289° | 0.657° | 3 | 0.433° |
| FlashLight | neither_correct | rgb | 4 | 6.075° | 0.849° | 4 | 0.292° |
| FlashLight | neither_correct | thermal | 4 | 5.088° | 4.685° | 4 | 4.791° |
| SmokeRoom | both_correct | rgb | 154 | 4.048° | 0.239° | 154 | 0.182° |
| SmokeRoom | both_correct | thermal | 154 | 4.478° | 0.170° | 154 | 0.140° |
| SmokeRoom | rgb_only_correct | rgb | 27 | 6.647° | 0.413° | 27 | 0.295° |
| SmokeRoom | rgb_only_correct | thermal | 27 | 5.027° | 1.005° | 25 | 0.291° |
| SmokeRoom | thermal_only_correct | rgb | 36 | 0.073° | 2.478° | 33 | 1.104° |
| SmokeRoom | thermal_only_correct | thermal | 36 | 8.259° | 0.203° | 36 | 0.143° |
| SmokeRoom | neither_correct | rgb | 19 | 0.142° | 2.080° | 17 | 0.634° |
| SmokeRoom | neither_correct | thermal | 19 | 5.239° | 0.378° | 19 | 0.115° |

SmokeRoom's thermal-only-correct RGB group has accepted ray-change IQR
0.062–0.100° (median0.073°). Its accepted reference-plane residual IQR is
1.884–3.535° (median2.478°), versus median0.239° in both-correct RGB cases.
The cached fitted-E plane residual is only0.033° in the former group: a small
residual against the model fitted from those matches does not certify motion
accuracy or agreement with the supplied motion.

This supports low-motion/inconsistent matching evidence upstream of recovery in
that group, conditional on the nominal reference/calibration. The full reference
is not independently certified physical match truth, so it does not prove which
points are incorrect or establish a causal network/scene failure.

On FlashLight, thermal in the RGB-only-correct group also has accepted ray-change
median0.082° and reference-plane residual7.503°, compared with0.357° in its
both-correct group. The opposite camera can exhibit the same broad failure
pattern; an always-thermal or always-RGB causal explanation is unsupported.

## Fixed examples retain important exceptions

All48sensor rows from the previous24locked image-sample anchors were traced;
no new visually selected sample was substituted. The following are descriptive
examples from that fixed set, not separately validated scene categories.

- SmokeRoom604RGB: accepted median ray change0.062°, supplied-reference amount
  5.641°, E rotation candidates0.159°/179.957°, original error5.779°.
  Its accepted matches already have very small median motion.
- SmokeRoom635thermal: accepted ray change0.056°, reference amount12.709°,
  reference-plane residual9.328°, E candidates0.122°/179.936°.
- SmokeRoom634RGB: accepted ray change9.614° and reference-plane residual0.411°,
  but E candidates0.153°/179.966° versus reference amount12.900°. Substantial
  match motion is present although neither E rotation is correct.
- SmokeRoom748thermal: accepted ray change8.351°, reference-plane residual0.378°,
  E candidates0.448°/179.982°, reference amount9.475°.
- FlashLight476RGB likewise has ray change5.253° with E candidates0.151°/179.975°
  and reference amount6.518°; the exception is not unique to SmokeRoom.

These exceptions prevent concluding that all compressed estimates originate in
near-zero accepted flow. Essential-model estimation, match mixtures/spatial
geometry, degeneracy or reference limitations can remain relevant. A scalar
median cannot determine which subset drives E. Cheirality-subset reference
residuals often improve, but those points were selected by the same fitted model;
this is not independent validation or evidence for adopting another mask.

Rotation-only warp residuals are saved alongside the plane residuals. Translation
and parallax can make a rotation-only warp disagree even for exact correspondences;
the translated-static-3D synthetic control has nonzero rotation-warp residual
and near-zero full-reference epipolar residual. Raw bearing motion is also not
a direct rotation estimator. No residual cutoff, ranking or quality gate is fitted.

## Verification, limits and decision

Independent audit passes129,109checks on1,712rows. It reconstructs unit rays
without the production bearing helper, decomposes E through an independent SVD,
checks complete candidate sets up to order and original-R membership, and verifies
all masks, own reference/frame binding, exact old-error parity, residual valid
counts/medians/p90 and pair-weighted group quartiles. All8tests pass: translated
points, corrupted matches, E branches, empty/degenerate masks, valid full replay
and rehashed residual/candidate-error/branch-count corruption controls.
Reference-free match/candidate features lock before parsing reference values.
Parent prediction/score/mask/reference files remain unchanged.

Input/reference/training/physical exposure qualifications from the parent studies
remain. A targeted official OpenCV API/source contract check was added; prior
paper reading was reused, with no new paper-body reading, author-system
reproduction or new-method claim. Prior uncertainty/closure/geometry negative
results are not overturned by this descriptive diagnosis.

Evidence: `results/motion-compression-location/`; per-pair rows and summaries under
`run-001/`, full prior sample trace, source receipts and reference-residual PNG/PDF.
Verified local incremental packet:
`/root/ore-ms-artifacts/motion-compression-location-2026-11-16/`.
No Box upload, Git push or automatic commit occurred.

Stop decision: wrong supported rotation is upstream of selecting a branch of the
cached E. Many failures have low-motion accepted matches, while fixed exceptions
retain substantial motion and still yield a wrong E. The next bounded question
is which cached essential-inlier spatial/motion subsets support the small-rotation
model in those pre-existing exceptions. Diagnose that mixture/geometry with fixed
controls before any solver/mask/threshold change; no causal component or correction
has been established by this study.
