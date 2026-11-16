# Reference contract and frame-invariant angular-motion diagnostic

16 November 2026. Exposed-data characterization using existing locked NGCLAB
predictions, without training, new inference, fitting an extrinsic, clock shift
or threshold rescue. Existing scalar-angle medians were inspected diagnostically
in the prior study; this is not a blind or independent validation dataset.

## Source audit

Inspect released calibration summaries and raw calibration payloads for the
missing mocap-rigid-body→camera transform. Verify ZIP CRCs and actual calibration
bag connection metadata. Camera→IMU and Kinect internal extrinsics do not establish
mocap→camera binding. A Kalibr internal body label does not make that body the
OptiTrack tracked rigid body. Camera–IMU shifts are not camera–mocap shifts.
Preserve failed PDF extraction and the successful alternative parser receipt.
Do not apply a plausible axis permutation, IMU transform or fitted clock to the
existing estimates. Unknown global world axes alone cancel in relative poses.

## A limited measurement that does not require the missing extrinsic

For fixed rigid-body→camera rotation X, relative camera rotation is conjugate to
relative rigid-body rotation: R_C = X^T R_B X. Thus trace and rotation magnitude
are identical. A common fixed world transform also cancels. Translation of the
camera relative to the body does not change this rotational identity. The paper
places the cameras and mocap markers on one rigid rack, supporting this limited
measurement. Reference timing and camera calibration still carry prior assumptions.

Measure |angle(estimated relative R) − angle(reference relative R_B)| in degrees.
This measures angular-motion amount, **not axis/direction, full orientation error,
translation, trajectory accuracy or match correctness**. In particular, rotations
about different axes can have identical magnitudes, so zero scalar error is not
correct orientation. Show that counterexample in synthetic qualification.

Use all previously declared half-second lighting intervals and the frozen common
reference-supported set for each recording (both cameras qualify). Recompute
reference angles from raw mocap at each sensor's own image-header endpoints using
the unchanged nominal timing and ≤50 ms bracket; no added offsets or refit.
Keep unsupported/missing reference separate, with all original rows retained.
For scorable common-reference cases, a missing/non-ok cached pose remains a
failure. Do not rescue finite rotations from rejected poses.

Report solver availability, conditional median and p90 absolute angle error,
and success ≤1° over all common-reference cases. Include the always-available
zero-angular-motion predictor. Report separate predeclared strata: reference
rotation ≥2° and <2°, retaining both, to expose small-motion identity effects.
Also report RGB/thermal paired wins and failures; no significance or generalization
claim from dependent intervals or three recordings. Keep frontend families and
conditions separate. Smoke has no newly qualified mocap and is not added to this
accuracy diagnostic.

## Execution and audit

Qualify conjugacy/global-frame invariance with random transforms, a wrong-axis
same-angle counterexample, quaternion sign invariance and rejected-output guards.
Freeze source, protocol and consumed cached prediction/mocap/association hashes
before full scoring. An independent auditor computes interval angles from direct
quaternion interpolation and relative-quaternion dot products, and recomputes all
scores/strata. Preserve the prior failed full-pose endpoint and all support results.
Stop after this source audit and partial-motion measurement. Full pose accuracy
remains open unless the missing frame chain is independently established.
