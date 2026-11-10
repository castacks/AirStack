# Thermal stereo mean adaptation: trained and evaluated

10 November 2026, America/New_York. Branch `ore-ms-orb-development`; experiment freeze `f650b660f`.

**The controlled run is complete. This configuration does not provide an accuracy improvement, and it does not establish training descent.** It changes predicted stereo means, unlike the earlier covariance-only adaptations. A fixed final checkpoint was evaluated regardless of outcome; no learning-rate, parameter-scope or checkpoint search followed the rain results.

## Intervention and split

Reused existing source pilots: day `_2021-08-06-11-23-45` and night `_2021-08-13-21-18-04` for fitting, rain `_2021-08-06-16-19-00` for evaluation. Each contributes source frames 000000–000019. All were already development-exposed; checkpoint training membership is unknown. Rain was excluded from this fit and checkpoint selection, but is not a fresh validation record.

Restored exact thermal stereo images, original own-camera single-scan projected depth and numeric calibration from EXP048/041. Rain inputs reuse EXP054 and the recent recovery. Native uint16 thermal pixels receive the unchanged named preprocessing recipe. Training uses every eligible depth pixel, not only the native VO keypoints, avoiding the failed two-endpoint reference-coverage requirement.

Adapted only the horizontal output row of `memory_decoder.update_block.flow_head.conv2` weight/bias: **2,305 trainable scalars in two parameter tensors**. All other parameters/buffers and the vertical output rows remain bitwise unchanged. The real-model backward-only contract shows finite nonzero horizontal gradients and zero vertical gradients.

The frozen run uses full-resolution FP32, 12 decoder stages, seed 0, Adam at 1e-5 with no weight decay, gradient-norm clipping at 1, and exactly 40 updates interleaving day0/night0 through day19/night19. Loss is final-stage horizontal SmoothL1 with beta 1 pixel against `-fx*B/Z`. The label-only mask requires nonzero supplied depth and an inbound reference-implied right pixel; no predicted-positive filtering, depth clipping or hole filling. There are 342,569 eligible pixel uses across the 40 training frames. Pixels/adjacent frames are correlated, not independent trials.

The final mean-head artifact was locked before evaluation. During fit, filesystem guards reject excluded rain image/depth paths and prior-score reads. Training order and label counts are independently reconstructed. No rain evaluation or final-checkpoint choice enters the optimizer loop.

## Excluded-record stereo result

Twenty rain frames, 121,474 eligible depth pixels. Values below are equal-frame averages; all methods use the same label-derived support. Original and adapted predictions are finite on that support, with no nonnegative horizontal-flow samples.

| Metric | Original | Adapted | Change |
|---|---:|---:|---:|
| Signed horizontal flow MAE (px) |2.190303|2.216936|+1.22%|
| Depth MAE (m) |2.613372|2.645902|+1.24%|
| Per-frame depth RMSE average (m) |6.958122|7.070310|+1.61%|
| Depth AbsRel |0.179222|0.182738|+1.96%|

These supplied projected-depth comparisons retain visibility/calibration/timing limitations. They do not establish physical ground-truth accuracy.

## Native VO comparison

Ran the original 20-frame native MAC pipeline in four arms: original thermal, an unchanged original-model copy, adapted stereo means, and original RGB. Original temporal flow and disparity variance come from a frozen original-model copy in the adapted arm. The mean model receives identical native batches; only its stereo flow is used. Derived depth covariance changes with the new disparity through the unchanged native propagation formula; it is not claimed numerically fixed.

The unchanged-copy control reproduces all 20 saved pose/time arrays exactly and all 19 mapped interval slots, including unavailable status. A separate stub test changes the mean output while verifying temporal flow and disparity variance still come from the original branch.

Common support is 18 of 19 expected intervals. All thermal arms retain 18; RGB retains 19. Thermal pair 1 falls outside the inherited nominal common-time mapping horizon. No interval is removed based on the adapted error. Native camera motion is scored in the existing nominal common RGB frame and constant-twist interval convention against the supplied RGB reference; this is the same reference/model used in recent accuracy comparisons.

| Arm | Mean translation (m) | Translation RMSE (m) | Translation p95 (m) | Mean rotation (deg) |
|---|---:|---:|---:|---:|
| Original thermal |0.960001|1.048964|1.500730|0.075270|
| Unchanged copy |0.960001|1.048964|1.500730|0.075270|
| Adapted stereo mean |0.959302|1.049632|1.504289|0.074118|
| Original RGB |0.670000|0.867444|1.383230|0.031285|

Adaptation lowers mean translation by 0.073% and mean rotation by 1.53%, while worsening translation RMSE/p95 and rotation p95. These small mixed changes over 18 adjacent intervals do not establish a useful VO gain. RGB remains the stronger baseline on this common prefix. No full-record trajectory or deployment claim follows.

## Fitting did not establish descent

After the locked evaluation, recomputed the same training objective before/after adaptation to distinguish failed fitting from poor transfer. This is a posthoc diagnostic: it adds no optimizer update and chooses no checkpoint.

| Training population | Original SmoothL1 | Final SmoothL1 |
|---|---:|---:|
| Day |1.461836|1.463833|
| Night |1.973632|1.983137|
| Equal-frame combined |1.717734|1.723485|

The combined training objective worsens 0.335%. Therefore this is not evidence that a successfully fitted mean model failed to transfer; successful fitting itself was not demonstrated.

One additional bounded probe used only training day frame 0 at the original checkpoint. The native decoder reuses the mean head while detaching flow coordinates between recurrent iterations. Its stop-gradient graph gives an autograd directional derivative of −12.06 along the normalized negative gradient. Recomputed full forwards at fixed perturbations 1e-5 and 5e-6 instead give positive central-difference estimates 2.23 and 0.52; the original loss is restored exactly afterward. The differing numerical scales limit attribution: this is evidence that sampled recomputed forwards do not show the expected descent, not proof that detachment alone caused the 40-step outcome. No alternate learning rate, training objective or checkpoint was tried.

The next prerequisite for further adaptation is a **verified descent contract for this shared recurrent mean-head intervention**, including its objective and truncation semantics. More data or a larger adaptation should not be presumed to fix this run. That is an optimization question arising from actual fitting, not another covariance/selector variant. No additional fit was performed.

## Verification and artifacts

Six tests pass: sparse target units/sign/mask, masked horizontal loss, exact trainable scope/vertical-row preservation, excluded-data guard, changed-mean branch isolation and unchanged-copy output preservation. Real-model backward-only and 40-step state checks pass. All source model files and original checkpoints remain unchanged.

The independent audit imports neither producer nor scorer. It verifies 305 frozen source/input hashes, the two-tensor mean-head artifact and unchanged vertical rows, 40 training steps/label counts, 200 stereo metric quantities, 73 native mapped motions, all common-support VO statistics, 19 exact passthrough slots, and 40 post-fit training rows. State checks certify the declared parameter/output contract; they do not certify optimization success.

See the [frozen protocol](../../experiments/stereo-mean-adaptation-freeze-2026-11-10.json), [complete outcome](../../experiments/stereo-mean-adaptation-outcome-2026-11-10.json), and [record-use ledger](../../experiments/recording-exposure-ledger-2026-11-10-stereo-mean-training.json). Inputs, checkpoint, all predictions, native captures, logs and audit receipts are in `results/stereo-mean-adaptation/run-001` and the companion Box packet. No runtime benchmark, new dataset campaign or default-model promotion occurred.
