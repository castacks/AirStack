# Mean adaptation: a verified training descent contract

10 November 2026, America/New_York. Branch `ore-ms-orb-development`; protocol/source freeze `743ab87e4`.

**The training-only correction passes its numerical gradient and descent checks.** A final-output residual on a frozen decoder lowers the 40-frame training objective from **1.717747 to 1.690568** over three accepted updates (−1.58%). This is a training contract result, not a stereo/VO accuracy gain. Rain, the earlier candidate and its results remain unchanged.

## What was wrong with the previous gradient interpretation

The native mean convolution is reused at every recurrent decoder step. The source deliberately calls `flow_coords1.detach()` between steps. Autograd therefore differentiates a stop-gradient computation; it need not equal the derivative of rerunning all iterations after perturbing a weight used at every step. A finite nonzero gradient alone does not establish a descent direction for that recomputed forward.

Repeated the diagnostic on the first day and first night **training** frames, with CUDA matmul/CUDNN TF32 disabled, no optimizer step, and exact parameter/loss restoration. Step direction is the normalized negative autograd gradient. Positive finite-difference derivative means that direction increases the recomputed loss locally.

| Training frame | Autograd derivative | Recomputed derivative, step 1e-3 | Recomputed derivative, step 1e-4 |
|---|---:|---:|---:|
| Day 000000 |−12.06210|+1.89126|+1.76370|
| Night 000000 |−6.73856|−0.66954|−0.82850|

Day shows a sign mismatch at both scales; night agrees in sign but differs substantially in magnitude. The earlier small-step discrepancy is therefore not removed by these precision controls. This is consistent with the deliberately truncated recurrent graph, not evidence of a PyTorch or upstream-model bug. Two frames do not establish that this mechanism alone caused the prior 40-update outcome. No failed checkpoint was retuned.

## Narrow correction tested

Freeze the entire original decoder. Capture its final hidden feature tensor, native final flow-upsampling weights and original horizontal prediction. Add a **zero-initialized, one-channel 3×3 residual convolution** on those final hidden features, upsample it with the frozen native convex mask, and add it to the original final horizontal output:

`prediction = frozen_prediction + frozen_upsampling(residual_convolution(frozen_features))`.

The correction has 2,305 scalars, matching the previous active parameter count. It is applied only after the frozen decoder; it cannot alter earlier coordinates, hidden features or covariance outputs. Vertical flow stays original by construction. This is a different parameterization, not a silent change to the native decoder or an implementation-equivalent replacement for the old shared-head update.

With fixed features/masks and a fixed label mask, prediction is affine in residual weights. The unchanged sparse SmoothL1 objective is convex in those weights. The complete training gradient can thus be checked directly without differentiating through the recurrent trajectory.

Used only the same 20 day and 20 night frames. The existing named preprocessing, own-camera depth labels, eligibility mask and equal-frame weighting are unchanged; 342,569 eligible pixel uses are retained. The source dataset metadata still identifies rain so its files can be denied, but no rain images, labels or scores are read by this experiment. Original model state is hashed before/after all cache forwards and remains unchanged. Zero residual reproduces the frozen horizontal prediction exactly on every training frame.

## Numerical gradient check

Computed the 40-frame objective and its gradient in CPU FP64, using frozen native FP32 features/mask probabilities as constants. Central differences along the normalized negative gradient agree at both declared scales:

| Perturbation | Autograd derivative | Central difference |
|---|---:|---:|
|1e-4|−9.164673938|−9.164672674|
|1e-5|−9.164673938|−9.164673924|

Maximum relative discrepancy is about 1.4e-7, within the frozen tolerance. Zero-parameter loss is restored exactly after the check. This tests the explicitly defined final residual objective, not the original recurrent shared-weight objective.

## Three actual training-control updates

Used strict GPU FP32 on the full 40-frame objective. The fixed optimizer uses normalized negative gradients and an objective-checked Armijo rule: initial step 1e-3, coefficient 1e-4, halving on rejection, at most 12 trials, and minimum absolute decrease 1e-6. This is training-objective optimization, not evaluation-based learning-rate or checkpoint selection. Every actual update accepts its first trial; no retry configuration or excluded-record feedback is used.

| Update | Before | After |
|---|---:|---:|
|1|1.717747219|1.708617233|
|2|1.708617233|1.699557263|
|3|1.699557263|1.690568041|

A small residual artifact is saved as **training-control only**. No rain prediction, native VO run or default-model replacement follows in this pass. The previous mean-head checkpoint still has SHA256 `6051cbbd…a22580`.

Both parameterization and optimizer differ from the previous online Adam run. This control establishes a reliable optimization path; it does not isolate which change would improve the earlier fit or show that training descent transfers to VO accuracy.

## Independent verification

Five tests pass: spatially varying native convex upsampling against scalar neighbor sums, exact zero identity/frozen-input gradients, finite differences, actual backtracking descent, and rejection/restoration behavior. The existing excluded-data guard is reused unchanged.

The independent auditor imports neither producer nor scorer. It derives the convolution/upsampling adjoint analytically in NumPy, reconstructs the 40-frame initial gradient and both finite-difference scales, and independently computes final predictions/loss from the saved residual parameters. It verifies 225 frozen hashes, all 40 training caches, original label masks/counts, and all three recorded Armijo acceptance conditions. Independent final loss is 1.690568022; maximum FP32 prediction discrepancy is 1.93e-6 px. The earlier checkpoint remains unchanged.

The audit does not independently regenerate the native feature caches from images or recreate every intermediate optimizer gradient. Native state/zero-identity checks, source/cache hashes and scalar operator tests cover their declared contracts separately. No physical ground-truth, transfer, full-record trajectory or runtime claim is made.

See the [protocol](../../experiments/mean-descent-contract-freeze-2026-11-10.json) and [complete outcome](../../experiments/mean-descent-contract-outcome-2026-11-10.json). Caches, numerical checks, accepted-step journal, residual artifact and audit are in `results/mean-descent-contract/run-001` and the companion Box packet.

## Next step

The descent prerequisite now passes for this explicit final-output adapter. The next experiment can use a fixed training budget and the same record-separated stereo/native-VO controls, with the final adapter locked before any new rain scoring. Keep uncertainty and temporal-flow branches frozen. These remain exposed development recordings; a positive result would still need broader/fresh-record assessment. No further fit or accuracy evaluation occurred in this pass.
