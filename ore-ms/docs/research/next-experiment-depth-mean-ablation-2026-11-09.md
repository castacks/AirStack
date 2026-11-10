# Next experiment: depth mean versus covariance in thermal motion error

9 November 2026, America/New_York. **Design only; not executed.** Decision follows the duplication/error-mechanism review. Runtime remains excluded.

## Question

Under the same native correspondences and solver, does improving the stereo/depth mean change thermal motion error more than replacing covariance weights? Recent common-frame thermal errors are mostly along motion direction. This does not prove depth is the cause; the intervention is intended to test that link.

Earlier EXP009 varied covariance while preserving depths; EXP066 diagnosed disparity/depth residuals; EXP070 fitted bias/scale risk scores without changing point means. ORB initialization/depth controls use a different backend. This experiment adds a mean-changing motion-level control; it is not another selector or a claim that stereo bias is a new idea.

## Four matched arms

| Arm | Stereo/depth mean | Factor covariance |
|---|---|---|
| Native replay | Original | Original |
| Weight control | Original | Fixed, reference-free per-channel control |
| Mean diagnostic | Reference-assisted depth/disparity | Original numeric observation covariance |
| Combined diagnostic | Reference-assisted depth/disparity | Same fixed covariance control |

Use saved native MAC/MACT factors and exact association IDs. Reconstruct source 3D points along their original camera rays at the assisted depth and, for the disparity graph, change the corresponding target disparity mean consistently. Keep UV observations, associations, intrinsics/extrinsics, source parent poses, optimizer/init/budget and numerical observation covariance identical within the declared arms. Do not replace a 3D mean while leaving an incompatible disparity mean silently unchanged. State the treatment of source landmarks/gauges explicitly.

Reuse the EXP009-style reference-free covariance control with its exact convention frozen for the selected native graph. Do not fit covariance to the outcome. Include RGB as a matched control so an apparent thermal effect is not interpreted without its reference sensitivity.

## Qualification before execution

Select a bounded packet of already-exposed pairs with source and target **own-camera** supplied depth products and exact pixel/point provenance. Record source IDs, timing/calibration, projected-depth validity and all missing/ambiguous cases before solving. All four arms use the same qualified factors; retain the full-native-support replay separately to expose losses from reference support filtering. Do not infer that projected LiDAR guarantees correspondence, visibility or physical truth.

First reproduce the unchanged native graph on the saved factors. Require a declared numerical parity tolerance, unchanged associations/inputs and explicit solver status; if parity fails, resolve the replay contract before interpreting another arm. Recover existing authorized backup inputs if needed. No new recording, training campaign or replacement model is implied by this design.

Reference-assisted depth is privileged diagnostic input. Its scores may localize sensitivity under the supplied benchmark convention, but cannot establish an executable method gain. No per-record pose scale, reference-fitted time offset or favorable subset is substituted for the depth intervention. Empty/invalid support remains explicit; no silent depth floor or clipping.

## Readout and resulting work

Freeze the exact source packet, covariance-control values, parity tolerance, qualification rules and scoring protocol before running the four arms. Report translation mean/RMSE, motion-axis error and rotation separately, matched denominators and all termination states. Compare each mean intervention with its matching weight arm, rather than only with a weakened baseline. Fixed-factor results do not establish accumulated/end-to-end VO gain.

If the assisted means reduce thermal motion error with retained rotation/support, inspect the signed stereo-mean residual and its range dependence to choose a deployable mean/frontend correction or adaptation target. Reuse the existing mean/reference diagnostics rather than repeat their surveys. If only weights help, return to the already demonstrated optimization/weight mechanism; if neither helps, investigate correspondence/solver/reference effects. Do not turn either result directly into a camera-confidence rule.

This is a concrete next experiment, with execution details to be pinned after input/replay qualification. It makes no new accuracy, physical-causality or generalization claim.
