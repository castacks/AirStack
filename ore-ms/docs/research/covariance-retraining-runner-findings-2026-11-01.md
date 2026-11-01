# EXP096 — manifest-bound covariance-head retraining runner

2026-11-01 America/New_York. Initialcode4687a5649, runtime/control amendment40b7e57b6,
final checkpoint/resume implementationc1619bc4c. New opt-in infrastructure is
implemented and checked with synthetic software fixtures. No real-data fit is
started: the available project inputs have no identified source-backed dense
flow training manifest. Existing trainer/loss/inference remain unchanged.

## Data and initialization

`Train/MatchingNet/retraining_manifest.py` verifies pinned ledger, architecture,
initialization, input recipe, per-record label evidence and every prepared pair.
The ledger must classify each canonical ID as a development-exposed temporal
recording; aliases, conditional reserves, unknown/unused IDs, recording overlap,
duplicate sample content and shared prepared images across train/validation are
rejected. These checks enforce declared identities/evidence hashes; they do not
authenticate the truth or completeness of source attestations.

Each NPZ has exactly `image1,image2,flow,valid_mask`. Images are finite FP32
[3,H,W] RGB-order values in[0,1]; flow is FP32[2,H,W] first→second displacement
(u,v) in prepared-image pixels; mask is bool[1,H,W],Truevalid. H/W>=64 and divisible8.
No resizing,augmentation,padding or shuffle is implicit. Batch size1 and cyclic
manifest-order training are explicit. Finite target/magnitude/empty handling uses
EXP095; preflight rejects samples with no valid supervised pixels.

An [intentionally incomplete template](../../experiments/covariance-retraining-template-2026-11-01.json)
records the released RGB checkpoint SHA and
[exact FP32 architecture](../../experiments/covariance-retraining-architecture-2026-11-01.json).
Architecture pretrain=false prevents implicit downloads; strict checkpoint loading
uses the existing audited migration, with its report retained. Unknown base
training membership is mandatory in this prototype. A new finetune cannot claim
unknown pretraining overlap has disappeared. Fill source-backed recipe/recordings/
samples and every optimization option before use; this template is not executable.
New external data needs an explicit ledger entry and source evidence first.

Metadata checks found no flow-directory/flow-suffix label candidates in ore-ms;
/data and/datasets were absent and/mnt empty. Box root/depth2 filename metadata
identifies only9known TD3archives and backups, with no dense-flow source identified.
This is bounded filename/config evidence, not proof that labels exist nowhere.
No archive, new image, pose or conditional-reserve payload was opened.
Actual data readiness remains `NO_EXECUTABLE_REAL_DATA_TRAINING_MANIFEST`.

## Runner and checkpoint behavior

`Train/MatchingNet/retraining_runner.py` trains only24actual
`memory_decoder.cov_update.*` parameter tensors. Frozen parameters and buffers
stay in eval; only the decoder's own return-mode flag is set to return all12
full-resolution recurrent stages. Covariance update modules train; validation
uses eval modules while retaining recurrent output structure. Do not call the old
`inference` helper in this recurrent-return mode: it expects the separate eval
return tuple. Residual detachment and explicit parameter freezing are both needed.

Loss/gradient/gradient norm and updated-state checks reject nonfinite values.
Frozen-state snapshots are checked each step. Constant AdamW learning rate,
seed,budget,checkpoint interval,betas,epsilon,decay,clip,gamma,maxflow are explicit;
there is no hidden scheduler advance. Validation uses only its assigned records,
no optimizer step or best-checkpoint selection, with valid-component weighting.

Checkpoints contain full model/AdamW state,Python/NumPy/Torch/CUDA RNG,step,
trainable names,manifest and exact source/runtime identity. Resume requires matching
manifest/data/init/source,Torch/NumPy/Python/dependency/CUDA/cuDNN/device identity,
optimizer configuration and state counters. Frozen-state or nonfinite optimizer
tampering is rejected before output creation. Fresh output directories are required.
Interval/end checkpoints publish through exclusive same-directory hard links after
fsync; unrelated existing pending files are preserved. A failed run retains its
step log/failure report and already-published checkpoints. Resume files are
training-state containers, not direct replacements for legacy inference weights.

Example, from MAC-VO-main, once a real manifest has been completed:

```sh
python -m Train.MatchingNet.retraining_runner --root /root/AirStack/ore-ms --manifest /absolute/completed-manifest.json --output /absolute/fresh-run --device cuda
```

Use `--stop-after N` for an absolute declared pause step within the unchanged total
budget; use `--resume /absolute/step-0000000N.pt` with a fresh output directory.
The CLI always builds the real FlowFormer backend. The injected model factory is
a software-test seam. The process owns RNG/thread/deterministic settings.

## Validation and limits

CPU suite:232pass,1CUDA-only skip; new CPU cases26,previous regressions206.
CUDA synthetic fixture resume:1pass. Uninterrupted four-step versus two-step+
resume executions match model,optimizer,RNG and step-log bytes on CPU and CUDA;
CPU validation losses also match. Nonfinite gradients prevent optimizer stepping.
Manifest/alias/reserve/path/hash/split/NPZ and resume-state rejection tests pass.
Fixture labels/models are explicitly synthetic and temporary, with no scientific
training or quality interpretation.

The actual released RGB model also passes a frozen64x64 synthetic-input/zero-flow
**backward-only** smoke: strict loading,12recurrent stages,24trainable covariance
parameter tensors,finite gradients only in that block,frozen-mean equality and all
parameter/buffer bytes unchanged. Zero optimizer steps; this does not calibrate or
validate a data-trained checkpoint. Actual full-model CUDA optimizer/resume and
thermal-checkpoint training have not been demonstrated; CUDA resume above uses
the small software fixture.

First actual-backend smoke failed the bitwise mean comparison because its control
used a different requires-grad regime. The retained v2 amendment freezes identical
parameters on both control paths; unchanged inputs/targets and zero-tolerance
comparison then pass. No specific kernel cause is established. Final v3 repeats
that passing check after atomic-save ownership and CUDA-fixture test additions;
no threshold or data retuning. All original failures,protocols and source revisions
are retained. Protocolv1/v2/v3 link source/inputs before each smoke.

Artifacts: `results/covariance-retraining-runner/run-001/` holds metadata inventory,
readiness,protocol chain,architecture and real-backend evidence; the recovery packet
also retains test/smoke logs and local design. All591prior protected source files
and both released checkpoint hashes are unchanged. Prior prospective test readiness
still has7missing requirements. Next is source-backed training-label acquisition/
preparation and an exact development split/budget, then a declared bounded real-data
sanity fit; keep reference/test eligibility separate.

Recovery: `backups/2026-11-01-covariance-retraining-runner` on ore-ms Box root.
