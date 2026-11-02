# EXP104 — opt-in trace-risk objective and matched comparison design

2026-11-02 (client America/New_York), branch ore_ms. Baseline2066d225f;
contract882e8707c, fixture fix e7c34d1ac, interpretation clarification cea312766.
[Full derivation, frozen comparison and validation](../docs/research/trace-risk-objective-findings-2026-11-02.md).

Added an opt-in final-stage score .5*(s/V+logV), s=squared endpoint error,
V=sum(exp(2raw)). Conditional optimum is expected squared error; also equivalent
to per-component isotropic Gaussian NLL up to a constant. Axis allocation remains
unidentified; no promise of better ranking. Existing diagonal NLL has the same
ideal population trace target. Means/targets fixed, strict original support,
FP32 minimum and finite-gradient/trace checks. No existing source/default changes.

Frozen future3arm design: recurrent diagonal control, final diagonal control,
final trace candidate. Same fullcovblock/originalinit/broad48pairs/48AdamW updates;
no tuning/earlystop. Primary observed-development ranking endpoint: added-two
macro withinpair50%-retention MSE. All perrecord NLL/coverage/risk outcomes retained.
Runner/resume/commands/runtime execution freeze required separately before launch.

100 CPU tests pass,1existing CUDA-only skip;19newtests. First overflowfixture was
insideFP32 range, corrected to peraxis.75*max; originalfailure retained, objective
unchanged. Later interpretation doc-only AST identity proven.1191 independent
source/data/math checks pass;599 oldsource unchanged. Zero model updates/exports/
promotion; no new data/reserve access. Unknownpretraining/syntheticRGB/observeddev
scope and prior7MS2 readiness gaps remain. Next: opt-in objective runner with
strict objective-bound checkpoint identity and exact resume, then execution freeze.
Recovery: backups/2026-11-02-trace-risk-objective.
