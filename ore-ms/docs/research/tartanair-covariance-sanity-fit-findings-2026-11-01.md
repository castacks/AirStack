# EXP098 — first source-backed covariance-head sanity fit

2026-11-01 America/New_York. Exact execution declaration frozen before outcomes; public summary1951dcbfb;
[protocol](tartanair-covariance-sanity-fit-protocol-2026-11-01.json).
The real-data16-step finetune completes and exact checkpoint resume passes.
The declared validation objective **worsens**, so this run supplies no quality
improvement or basis to adopt the candidate. It is a bounded execution check.

## Fixed experiment

EXP097 provides16training pairs from abandonedfactory/Easy/P000 and4validation
pairs from amusement/Easy/P001,all native480×640provided syntheticRGB forward-flow
labels/source masks. See [data/source findings](supervised-flow-development-findings-2026-11-01.md).
Whole trajectories/environments are separated; the base checkpoint's pretraining
membership stays unknown. This is our finetuning-stage development split, not a
network-pretraining-held-out test or thermal calibration experiment.

The protocol uses EXP096's unchanged runner:only24cov_update parameter tensors,
frozen mean/backbone modes,batch1cyclic,seed0,16steps,AdamWlr1e-5,betas[.9,.999],
eps1e-8,decay0,clip1,gamma.85,max_flow400,constantLR,checkpoints8/16. These settings
were declared before EXP097pixels. GPUimage8f7bf1c6/Torch2.7.1+cu128/RTXPRO5000
Blackwell;FP32 deterministic algorithms and explicit CUBLAS workspace/oneCPUthread.
No setting,source prefix,loss,window or threshold was tuned after outcomes. The exact execution protocol is retained by SHA256, with its original pre-outcome local commit timestamp. The public summary uses logical script identifiers instead of machine-local lab paths; parameters and script hashes are unchanged.

One baseline evaluation on all4validationpairs,one full16-step fit,one final
full-prefix evaluation and a fixed8→16resume replay all pass on their first
attempts. The replay executes8additional verification updates,not an extra
candidate budget or checkpoint selection. Candidate16steps plus8replayupdates
were run. There is no alternate-candidate sweep or selected-best validation score.

## Outcome

The metric is the valid-component-weighted recurrent diagonal Gaussian NLL,
omitting its additive constant,with fixed stage weights gamma^(T-i-1). It is the
new covariance training objective, not a pose/flow-mean accuracy or final-only
inference calibration score. Lower is better; negative values are valid.

| Full declared validation prefix | Before | After16steps |
|---|---:|---:|
| CUDA32 weighted recurrent NLL |−1.831284065773379|−1.5511346159488193|
| Independent NumPy64 formula |−1.8312840941592554|−1.551134613642587|
| Valid two-channel components |2,191,698|2,191,698|

After-minus-before is **+0.2801494498245598**, worse. Every declared pair and
all support remain in the result. Independent source-valid NumPy64 NLL agrees
with CUDA32 under the predeclared rtol2e-5/atol2e-6 on all8pair evaluations.
No calibration/ranking/VO/generalization improvement follows from this run.
The small one-pass training prefix and4adjacent validation pairs cannot establish
an adequate training schedule or a broad result about own retraining.

## Integrity and exact resume

Independent audit passes1,967tensor/metadata resume comparisons. All428active
model state entries are finite:exactly24covariance parameter tensors change,
and404frozen parameter/buffer entries match canonical initialization byte for
byte. All48full-resolution validation flow-mean stage fields (4pairs×12stages)
are bitwise unchanged before/after. Valid masks,component counts and sample
identities are unchanged. Thus the change in this objective is attributable to
changed covariance outputs on the fixed support and mean,not a changed VO mean.

Full-run checkpoint8and16 have matching AdamW counters across all24parameters.
An independent process resumes checkpoint8to16 and matches full-run model,
optimizer moments/counters,RNG state and the last8step journals exactly after
loading onto the common audit device. Serialized container SHA values differ;
the proof concerns tensor/metadata values,not identical file containers.

Candidate checkpoint:
`results/tartanair-covariance-development/run-001/full/step-00000016.pt`
SHA256 `762c65bfc98da634a998a08352795be56d0aca70faa760f6109aad7922382f6f`.
Replay checkpoint:
`results/tartanair-covariance-development/run-001/resumed/step-00000016.pt`
SHA256 `195cb3b6e556b2fdbd8ae5137594c3d890aad832bfb02af411cff376c89b69ec`.
These are full training-state containers,including provenance/optimizer/RNG;
legacy inference loaders do not take the whole container as a weight dictionary.
The candidate is retained for reproducibility and is not promoted into defaults.

All597frozen source files,raw/prepared data/manifest/recipe/ledger/label evidence
pins and both released initialization SHA identities are unchanged. The input
stage's241CPUtests/1CUDA-onlyskip remain applicable; no production code changes
were made in this experiment. OriginalRGB and thermal checkpoint files remain
controls. Prior MS²reference/test readiness still has7missing requirements;
this syntheticRGBfit does not resolve them.

## Retained work and next diagnostic

`results/tartanair-covariance-development/run-001/` retains exact execution
protocol/commands,source/data/init pins,before/after per-pair/source-mask/stage
hashes,full and replay reports/journals,checkpoints8/16and independent validation.
The recovery packet includes all driver/evaluator/audit scripts and logs. All
required attempts succeeded;no failure retry or tolerance amendment occurred.

The next useful engineering check is a training-only objective/descent diagnostic
on the frozen data and candidate,keeping this validation outcome intact. That can
separate successful optimization of this fixed-mean objective from transfer to
the other synthetic environment. Larger training,alternative data/losses or any
thermal experiment need their own declared recipe and recording-level scope.
No causal explanation for the validation regression is established here.

Recovery: `backups/2026-11-01-tartanair-covariance-sanity-fit` on ore-ms Box root.
