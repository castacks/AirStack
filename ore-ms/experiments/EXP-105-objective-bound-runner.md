# EXP105 — strict objective-bound runner and execution preparation

2026-11-02 client America/New_York, branch ore_ms. Baseline a8dde9404;
runner freeze6686a43da, zero-update native preflight freeze f1edbf928.
[Full findings and prepared commands](../docs/research/objective-bound-runner-findings-2026-11-02.md).

Added opt-in final_diagonal_nll/final_trace_risk_score wrapper; original engine,
loss/defaults unchanged. Exact objective ID/source participates in checkpoint
identity; all6cross-objective resumes rejected before output. Scoped bindings
restore and nesting rejects. Sidecar/CLI distinguish selected objective score
from common evaluation NLL; support2N, trace loss mean overN unchanged.

120CPU tests pass3CUDA-only skips;2targeted CUDA tests pass. Both modes on each
device exactly replay all model/optimizer/RNG/identity/manifest/journal/validation
fields; dropout fixture exercises stochastic resume. Native480x640 original-model
forward/backward for all3arms yields24finitegradients each,12identical meanfields,
unchanged state, zero optimizer steps. One training pair only, no quality claim.
7055independent hash/replay checks pass;601oldsource files unchanged. Audit folder
glob corrected for pytest truncation; failed audit preserved, scientific tests
passed first attempts and no implementation/objective/data/metric changes.

Prepared6serial commands:3×48scientific updates plus3×24resumeverification,
all unexecuted. Next commit host orchestration/execution and common36pair eval
freeze, then launch matched comparison. No real model checkpoint finetuning/export/
promotion. Unknownpretraining/exposedsyntheticRGB/7MS2 gaps/reserves unchanged.
Recovery: backups/2026-11-02-objective-bound-runner.
