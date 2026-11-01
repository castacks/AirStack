# EXP095 — opt-in covariance training semantics

2026-11-01 America/New_York. Implementation273de5b83;
[before-gate protocol](covariance-training-contract-protocol-2026-11-01.json)
2d12a6dc8. This implements a numerical prerequisite for own retraining.

`Train/MatchingNet/covariance_contract.py` supplies a new, explicitly opt-in
fixed-mean diagonal Gaussian covariance objective. All existing trainer, loss,
model, inference, adapter and checkpoint bytes remain unchanged. The historical
checkpoint training recipe and the cause of its failures remain unestablished.

## Contract

Raw covariance output means log standard deviation. Variance is exp(2raw), matching
the released inference transform for FP32/FP64. Per-component NLL, omitting the
constant, is half of squared residual times exp(-2raw) plus2raw. Average over
valid components and sum recurrent stages with gamma^(T-i-1); final-only mode is
explicit. This is a new objective, not an API-compatible replacement for old
`cov_loss`, whose numerical scale and validity reduction differ.

The predicted mean and label are detached. Only raw covariance outputs receive
loss gradients. A future head-only trainer must separately freeze all other
parameters and control their module modes/buffers; detaching the residual does
not stop raw-branch gradients through shared features. Training/eval return
structures in the existing decoder also differ, so the future runner must bind
full-resolution prediction/raw stages deliberately and test the binding.

Inputs are matching nonempty stage lists of[B,2,H,W] floating tensors. The mask
must be boolean[B,H,W] or[B,1,H,W]. A valid pixel also needs both target components
finite and target magnitude strictly below declared max_flow. Select valid pixels
before arithmetic. Nonfinite model output on a valid pixel raises; masked invalid
values do not enter the likelihood or its denominator. Empty valid batches raise
rather than silently yielding zero or stepping an optimizer.

Arithmetic promotes FP16/BF16 to FP32 and retains FP64. There is no epsilon,
variance clipping or hidden max_cov policy. Reject unrepresentable variance,
precision, likelihood and analytic raw-output gradients in the original output
dtype. Whole-model parameter-gradient finiteness still needs checking before an
actual optimizer step. Direct exp(-2raw) avoids the variance-squared intermediate
in reciprocal/division backprop, which can underflow even when the correct loss
and gradient are finite. Tests cover that case and half-output gradient overflow.

## Validation

| Check | Result |
|---|---|
| New objective tests |46pass|
| Existing adapter/loader/recorder/shared/state/readiness/exposure regressions |160pass|
| Total CPU suite |206pass|
| Pure-tensor CPU64 numerical gate |64cases,2195checks;max absolute discrepancy7.11e-15|
| Pure-tensor CUDA32 numerical gate |same64cases,2195checks;max absolute discrepancy9.18e-6|
| Maximum fraction of declared comparison tolerance |CPU2.71e-6;CUDA0.0211|
| Protected source verification |all588prior frozen source files unchanged|
| Released weights |both SHA256 identities unchanged|

Tests cover closed-form NLL/derivatives, optimum variance=residual², isolated
covariance gradients, masking invalid values before arithmetic, valid-component
normalization, strict target-magnitude boundary, empty batches, recurrent weighting
and final-only gradients, dtype promotion, finite-difference autograd gradcheck,
invalid options/shapes and representability failures. First45-test run passed;
a final output-gradient range test brought the new suite to46. No numerical gate
failed or tolerance changed. CPU64 tolerances rtol2e-10/atol2e-11;
CUDA32 rtol2e-5/atol2e-6. Independent Python math references and central differences
use seed0; they check code arithmetic, not calibrated uncertainty or a data outcome.

`Scripts/Setup/validate_covariance_training_contract.py` is a reproducible CLI:
run from MAC-VO-main with `--device cpu` or `--device cuda` and a fresh `--output`.
It writes an exclusive JSON containing per-case likelihoods, tolerances, errors,
source hashes and device/runtime. Artifacts are retained under
`results/covariance-training-contract/run-001/`, with exact source/protocol identity,
CPU/CUDA reports, protected-source verification and test log hashes. Runtime images
are the frozen event-tests6f8c1a10 and GPU8f7bf1c6 images. CPU Torch2.7.1+cpu;
GPU Torch2.7.1+cu128 on RTX PRO5000Blackwell.

## What remains before own data training

This module is deliberately separate from the legacy trainer. No network/image
forward, optimizer step, dataset label use, weight replacement or data fitting was
performed. Synthetic numerical cases are not a training or validation split.
The new objective is for two-channel flow residuals: sparse stereo depth or
pose-derived proxies must not silently become dense temporal-flow ground truth.

Next define the exact supervised task and source-backed labels; record full
initialization lineage and recording-level train/validation partitions; implement
an opt-in runner with exact trainable-parameter/module-mode checks, data recipe,
finite gradients, step/scheduler accounting and resumable provenance-rich saves.
Then declare a bounded development sanity run. Unknown base pretraining overlap
must stay explicit in any finetune claim. Keep conditional reserves untouched.
The separate prospective temporal-reference/exposure/test contract still has its
previous7missing requirements; this loss does not qualify those test recordings.

See [own-training prerequisites](own-retraining-prerequisites-2026-11-01.md).
Recovery: `backups/2026-11-01-covariance-training-contract` on the ore-ms Box root.
