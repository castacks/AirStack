# Released U²Flow checkpoint: inference interface qualified

Run at 2026-11-13T21:40:31.572917-05:00. Branch ore-ms-orb-development, baseline a79658f63
plus frozen working source. CPU/FP32 compatibility test, not original published
CUDA/PyTorch environment or author benchmark reproduction.

**PASS_RELEASED_CHECKPOINT_CPU_FP32_INTERFACE_ONLY.** Source commit
`152bb38d90621c34902943763c00a77ecef8725f`;42 inference/source files hash-pinned.
The checkpoint is the author's Sintel release, chosen before inference for its
diverse-image training context, not by observed thermal scores. Author Drive
file ID `1IvkXlUfI4AzIp2_iteZBRRokhNQouAah`,20907253bytes, SHA256
`b3dd6d382fdcfb079716fd89a4033a16540a5f59dcd4e3550ff6bc4b7a68f752`.
Metadata and complete bytes retrieved through the connected Google Drive tools.
No full benchmark or thermal input payload was accessed.

## State and runtime qualification

Restricted `torch.load(weights_only=True)` and strict state load accept all106
state tensors,5218435parameters. Only uniform DataParallel `module.` prefixes
are removed; no parameter substitutions or author source edits. Model state
hashes are unchanged after all inference. Source/checkpoint hashes match before
and after. Two synthetic cases each repeat bitwise exactly,12 iterations each.
All48 upstream uncertainty-head outputs are finite before the author's final
nonfinite sanitization. All saved flow/log-uncertainty outputs are finite.

Isolated runtime: Python3.12.3,Torch2.7.1+cpu,Torchvision0.22.1+cpu,NumPy1.26.4,
SciPy1.13.1,Matplotlib3.9.4; single thread, deterministic algorithms, eval mode,
no gradients and mixed precision disabled. Complete package versions/install
logs retained. No production/AirStack dependency changes.

An existing AirStack container lacked torch. A first standard venv setup failed
because host ensurepip was unavailable; dependencies were instead installed in
an isolated target directory. Both are engineering setup outcomes, before the
frozen first checkpoint/model inference. That first scientific interface attempt
passes unchanged. Author deprecation warnings are retained; no source patch.

## Input, coordinate and uncertainty contract

Inputs are float32[0,1], replicated grayscale channels, matching the inspected
author loader range. Synthetic images125x157 are padded to128x160 using the
author padder; unpadding exactly recovers input bytes after normalization. This
qualification uses native-size padding, whereas published Sintel evaluation
also has dataset-specific resizing. No equivalence to that full protocol claimed.

The model returns source-to-target pixel displacement at padded image scale.
The exact author's `resize_flow` function is executed from its unchanged AST
without importing unrelated cv2/file-loader dependencies. An independent
constant-field control verifies separate horizontal/vertical factors:2px/-3px
on32x40 becomes6px/-6px on64x120. Padding alone adds no displacement scale.

Uncertainty output `alpha` is scalar. The inspected training loss uses
`sqrt(2)*exp(-alpha/2)*L1_discrepancy + alpha/2`; evaluation uses exp(alpha) as
an uncertainty ranking quantity. Preserve raw alpha. Do not treat it as a
calibrated2x2 covariance, copy it into the old MAC covariance tensors, or
silently clamp it to improve coverage. Anisotropic image resizing has no
established scalar-to-full-covariance contract here.

## Descriptive smoke results

| Synthetic case | Eligible pixels | EPE mean / RMSE,px | Median predicted x/y,px |
|---|---:|---:|---:|
| Identity |19625|0.011737 /0.013380|0.006791 /0.001531|
| Known displacement(4,-3) |18666|0.097953 /0.108262|4.087255 /-2.996994|

The target is generated from the same textured source with exact integer pixel
identity on a fixed geometric mask. It is not a model pseudo-label. EPE is
descriptive, not a threshold chosen to qualify accuracy. A separate NumPy-only
saved-array audit passes17 checks on masks, source-target identity, truth fields,
finite values and error reconstruction. It imports no Torch/model and reruns
no network; bitwise repeat qualification comes from the producer receipts.

## Next concrete experiment boundary

Checkpoint/interface access is resolved. Next restore the declared small raw
thermal development packet, validate its old hashes and source preparation,
then freeze the known-warp/appearance assay's transforms, masks and numerical
endpoints before first thermal inference. Keep clean versus appearance-changed
cases separate and assess actual EPE separately from predictive inconsistency.
No training or new architecture is warranted by these smoke results alone.
Original MACThermal comparison still requires its own restored assets/runtime;
this qualification has not established that comparator's readiness.

Protocol: `experiments/u2flow-inference-qualification-freeze-2026-11-13.json`.
Runner/auditor: `tools/qualify_u2flow_inference.py`,
`tools/audit_u2flow_interface_outputs.py`.
Artifacts: `results/u2flow-inference-qualification/run-001`.
[Source intake](thermal-learning-intake-2026-11-13.md) records prior-art boundaries.
