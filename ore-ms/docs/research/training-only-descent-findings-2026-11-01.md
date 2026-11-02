# EXP099 — optimization descends on the training prefix

2026-11-01 America/New_York. Before-outcome protocol571eee374;
[fixed declaration](training-only-descent-protocol-2026-11-01.json).
The16-step candidate reduces the declared training objective on all16training
pairs. A fixed-batch Adam step and gradient-direction probes also descend locally.
The previous validation regression remains unchanged. This supports successful
local training optimization with an observed training/validation transfer gap;
it does not establish the cause of that gap or prove every model gradient.

## Fixed training-only checks

All16EXP097training pairs,initialization and the existing step8/step16 checkpoints
were specified before these diagnostic outcomes. The metric is the same source-
valid,component-weighted recurrent diagonal Gaussian NLL as EXP098,omitting its
constant;lower is better. Every frame retains native480×640pixels and the original
mask/magnitude rule. No new dataset,recording,hyperparameter search,checkpoint
selection,validation decoding or model export occurs in this diagnostic.

| Existing checkpoint | Training NLL CUDA32 | Independent NumPy64 |
|---|---:|---:|
| Initialization |−6.323683471036152|−6.323683285360707|
| Step8 |−6.476600695340512|−6.476600512700878|
| Step16 |−6.616049164724738|−6.616049031072675|

Step16minusinitial is **−0.2923656936885859**,better. All16individual training pairs
improve. The9,375,466valid two-channel components,all source identities/masks,
and192full-resolution flow-mean stage fields(16pairs×12stages)remain identical
across all three checkpoints. Thus the objective changes through covariance
outputs on a fixed mean and target support. Per-pair/per-stage decomposition,
variance,error and normalized-error statistics are retained without filtering.

## Fixed-batch local update and derivative

One transient copy starts at the released initialization and uses only the first
training pair. The pre-update loss and unclipped gradient norm match EXP098's
first step journal exactly. The same AdamW/clip/hyperparameters give:

| First training pair | CUDA32 NLL |
|---|---:|
| Before update |−3.897779941558838|
| After one Adam step |−4.021797180175781|
| Minus1e-4times clipped gradient from initialization |−3.9120047092437744|
| Plus1e-4times clipped gradient from initialization |−3.883655548095703|

The Adam update changes its batch objective by−0.12401723861694336. The symmetric
finite-difference estimate from independent NumPy64 likelihoods is141.7434846228205;
the analytic prediction using actual quantized FP32 parameter perturbations is
142.22326768733708. Relative discrepancy is0.003373449874399937(about0.337%).
Both signs agree;the small negative-gradient perturbation lowers the objective.
Epsilon1e-4was fixed before execution;no step-size/tolerance sweep or discrepancy
acceptance threshold was chosen. This is approximate directional evidence at
one batch and point,not an exhaustive deep-network derivative proof.

The initial state is restored exactly after both probes. The transient Adam model
and perturbed models are not exported. The original step8/16candidate,replay
checkpoint and every existing validation artifact keep their SHA256 identities.
No validation NPZ is decoded;compressed input hashing is separate from decoding.

## Independent integrity and interpretation

The independent auditor passes1,174scalar/support/mean comparisons:weighted
aggregates,per-stage NLL decomposition,NumPy64 versus CUDA32 likelihood at the
existing rtol2e-5/atol2e-6,exact initial-step journal binding,symmetric difference,
state restoration and unchanged source/data/checkpoint/result hashes. All597
production source bytes remain frozen. The prior241CPUtests remain applicable;
no production code changes or scientific/default replacements are made here.
The GPUruntime remains8f7bf1c6/Torch2.7.1+cu128 with deterministic FP32 settings.

EXP098's validation NLL remains−1.8312840658→−1.5511346159,worse by+0.28014945
on the other synthetic environment. It was not reevaluated or used to select
these probes. Together,the results demonstrate improved training-prefix NLL and
worse previously observed validation NLL. They are consistent with specialization
to the short training prefix. They do not distinguish data diversity,coverage,
optimization schedule,model parameterization or initialization history as causes.
There is no evidence here for a simple loss-sign reversal or a globally non-
descending first update. Base training membership remains unknown;all data are
synthetic RGB. Thermal calibration,physical VO accuracy and independent network-
held-out generalization remain outside this result.

Next design work can focus on a broader recording-level development experiment
rather than assuming that another loss-sign patch is needed. Define data coverage
and a training/validation monitoring plan before further outcomes;retain all
previous results and keep any eventual independent test separate. The existing
candidate remains unpromoted,and the7MS²reference/test requirements are unchanged.

Artifacts: `results/training-only-descent-diagnostic/run-001/` holds exact retained
protocol,input/source/result pins,all48checkpoint-pair scores,per-stage statistics,
fixed-batch probes and independent validation. Recovery also retains the executed
scripts and logs. Public protocol uses logical script names;the retained exact
protocol is bound by SHA256. No new trained checkpoint is included.

Recovery: `backups/2026-11-01-training-only-descent-diagnostic` on ore-ms Box root.
