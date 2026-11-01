# EXP095 — covariance-only log-standard-deviation objective

2026-11-01 America/New_York. Code273de5b83; before-numerical-gate protocol2d12a6dc8.
[Findings](../docs/research/covariance-training-contract-findings-2026-11-01.md).

New opt-in fixed-mean diagonal NLL agrees with variance=exp(2raw). Explicit mask,
finite target/magnitude filter, strict empty/invalid/numerical failure handling,
recurrent/final-only semantics and FP32-minimum arithmetic. Direct exp(-2raw)
avoids reciprocal-backward underflow. Legacy loss/trainer/inference untouched.

206tests pass (46new+160regressions); CPU64/CUDA32 numerical gates each pass
2195checks over64pure-tensor cases against independent math/central differences.
All588prior frozen source files and both checkpoint hashes unchanged. No image,
model forward, optimizer step, data fitting, reserve access or quality claim.
Next: source-backed task/labels/init/split and opt-in provenance/resume-safe runner.
