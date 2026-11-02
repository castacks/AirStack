# EXP104 — trace-risk objective verified; matched training comparison designed

2026-11-02 (client date America/New_York), branch `ore_ms`.
Design/implementation freeze `882e8707c`, numerical fixture correction `e7c34d1ac`,
mathematical interpretation clarification `cea312766`. Source baseline `2066d225f`.
[Comparison protocol](trace-risk-objective-protocol-2026-11-02.json).

An opt-in final-stage trace-risk score is implemented and checked. It targets
expected squared endpoint error through the total predicted variance, matching
the trace score used for EXP103 pixel selection. No neural checkpoint was trained,
exported or promoted. The existing runner, objectives and inference defaults are
unchanged. This stage supplies the objective contract and comparison design;
a separate runner/resume/source/runtime execution freeze is required next.

## What the objective means

For fixed predicted flow error e=(u,v error), define s=||e||² and total predicted
variance V=exp(2r_u)+exp(2r_v). The per-pixel score is

    L = 0.5 * (s / V + log(V))

It is averaged over the existing valid pixels and applied only at the final
prediction stage. The mean and target are detached; gradients reach only final
raw covariance outputs. There is no pair mining, margin, weighting coefficient,
clipping, epsilon, scale fitting or empty-batch skip. Inputs retain the strict
existing diagonal-contract admission, including FP32 minimum, shape, target-mask,
flow-limit, finite variance/precision and output-gradient checks. Additional checks
reject an unrepresentable trace sum or trace-score/output gradient.

For a fixed input with finite positive conditional mean squared error m=E[s|x],

    d E[L] / d log(V) = 0.5 * (1 - m / V)

The expected score has its minimum at V=m. The raw-axis derivative for one pixel is

    d L / d r_i = (exp(2r_i) / V) * (1 - s / V)

The mean over N pixels introduces a factor 1/N. The score is also equal, up to the
constant 0.5log(2), to per-component isotropic two-dimensional Gaussian NLL using
covariance (V/2)I. It is not the original anisotropic diagonal likelihood, and it is
not a direct pairwise ranking loss. Strict numerical support is shared deliberately
with the diagonal comparator, even where a trace-only calculation could admit more
extreme values. Validation performs a detached diagonal pass; its extra cost must
be retained/reported in a future training comparison.

Importantly, an ideal unrestricted diagonal NLL predictor already estimates each
component's expected squared error; summing those estimates gives the same optimal
trace. This proposal changes gradient coupling and the restricted model's fitting
problem, not the population target into a newly correct one. Improved ranking is
an experimental hypothesis, not a mathematical consequence of changing the loss.

Trace alone leaves directional variance unidentified. Different positive axis
allocations with the same trace have identical scores; component calibration can
therefore worsen. If conditional error is exactly zero, the infimum is approached
as V tends to zero and no positive finite optimum exists. The implementation does
not hide that boundary with a floor or penalty.

## Frozen future comparison

| Arm | Stages optimized | Score |
|---|---|---|
| Control | All recurrent stages | Existing diagonal NLL, gamma0.85 |
| Stage control | Final stage only | Existing diagonal NLL |
| Trace candidate | Final stage only | Trace-risk score above |

All three start from the original supplied RGB checkpoint, not either finetuned
candidate. All train the same full covariance block (24 tensors, 3,075,202 scalars),
with all other parameters/buffers frozen. The same broad48 training manifest,
round-robin order, seed0,48updates, AdamW lr1e-5/betas(.9,.999)/eps1e-8,
weight_decay0, gradient clip1, checkpoints24/48 and maxflow400 apply to every arm.
No lambda search, hyperparameter tuning, early stopping or candidate selection.
The final-diagonal versus trace comparison isolates the score; recurrent versus
final-diagonal separately measures stage alignment. Different gradient clipping
frequencies must be reported because loss scales/coupling can change update size.

Primary descriptive endpoint: the added-two-record equal-record macro within-pair
MSE at50% retention, trace-ranked, comparing trace versus final-diagonal. Secondary
outcomes retain all per-record/group risk curves at10/25/50/75/100%, random/oracle,
all diagonal recurrent/final NLL, joint coverage50/80/90/95/99%, standardized
moments/bias, training score/gradients and frozen-state/resume evidence.

All36 existing office16/Japanesealley16/amusement4 pairs remain observed development
evidence. No data are newly labeled a held-out test. Ranking gain with a NLL or
coverage cost is reported as a tradeoff. A claim of dominance requires improvement
on the primary endpoint without worsening any declared per-record95% coverage or
final NLL; even that would not authorize deployment or establish generalization.
No aggregate alone can hide per-record regressions.

Before execution, implement the opt-in runner binding with objective/source and
resume identities, reject cross-objective checkpoints, prove exact resume, pin
commands/runtime/data, and commit a separate execution declaration. A fresh future
recording evaluation still needs its own selection/exposure contract. Base network
pretraining membership remains unknown. This stage does not launch training.

## Validation and retained corrections

| Evidence | Result |
|---|---|
| New analytic/numerical tests | 19 pass |
| Relevant existing loss/runner/final-layer regression tests | 81 pass,1CUDA-only skip |
| Combined CPU suite | **100 pass,1skip** |
| Independent source/data/math audit | **1,191 checks pass** |
| Existing production source files | 599 unchanged |
| Model optimization/export/promotion | None |

Tests cover an independent scalar formula and raw-axis derivative, float64
finite-difference gradcheck, the shared-trace conditional-mean optimum on errors5/10,
one synthetic gradient step decreasing loss, directional nonidentifiability,
fixed mean/target, masked NaNs and final-stage-only gradients, FP16/BF16/FP32 inputs,
invalid domain rejection, exact max-flow boundary, zero-error boundary and trace
sum overflow. These are synthetic tensor fixtures, not neural retraining.

The first suite had99pass/1skip/1failure: raw44 has finite FP32 trace, so the
sum-overflow fixture did not actually overflow. The corrected fixture assigns each
axis variance75% of FP32 maximum; both axes fit individually and their sum exceeds
the maximum. No objective, comparison endpoint or data changed. The corrected
suite passed on its first run, and the original failure log remains retained.
A later documentation clarification states the exact isotropic-likelihood identity;
independent AST comparison proves executable code unchanged after the passing run.

The independent audit verifies all old source/input/model/data/artifact pins,
new source pins, old-stage outputs, both test logs, the isotropic identity,
finite-difference log-trace derivatives and expected-risk minimum, frozen48pair
manifest/options, and absence of model checkpoint exports. CPU image:
`sha256:6f8c1a104929edf2f56a9ae26cc01596cd040baf51743f2d5950b71e392b3bf4`.
Results and exact scripts/logs are retained in the recovery packet; a new helper
and its tests are the only two source additions. No existing source was edited.

Scope remains synthetic provided RGB flow, unknown pretraining membership,
observed short development prefixes and no independent-test, thermal, physical VO
or generalization claim. Protected MS² reserves and its seven reference-readiness
requirements are unchanged.

Recovery packet: `backups/2026-11-02-trace-risk-objective`, incremental from EXP103
`2066d225fe9d355b2567ed2973fb6e155242d58d`; prior packets retain datasets/models.
