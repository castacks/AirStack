# EXP105 — objective-bound runner verified on CPU, CUDA and the native network

2026-11-02 (client America/New_York), branch `ore_ms`.
Source baseline `a8dde9404`; runner/verification contract `6686a43da`;
real-network zero-update preflight declared in `f1edbf928` before its outcomes.

The two final-stage objectives now use an opt-in wrapper around the unchanged
training engine. Each resumes its own state exactly and rejects other-objective
state before creating output. CPU/CUDA synthetic replay and native-network
forward/backward checks pass. Six concrete scientific/replay commands are prepared;
no real-model optimizer update, candidate export, or promotion occurred.

[Runner protocol](objective-bound-runner-protocol-2026-11-02.json),
[native preflight protocol](objective-runner-preflight-protocol-2026-11-02.json),
[prepared comparison commands](objective-comparison-prepared-commands-2026-11-02.json).
The EXP104 comparison endpoints and controls remain unchanged.

## Behavior and checkpoint identity

`Train.MatchingNet.objective_retraining` requires an explicit objective mode:
`final_diagonal_nll` or `final_trace_risk_score`. Full-block parameter selection,
strict data/manifest checks, input order, AdamW, clipping, journal/checkpoint format,
atomic publication and runtime identity all remain those of `retraining_runner`.
The original recurrent trainer and final-layer policy remain independent.

The wrapper scopes the objective and source-identity bindings to one process-local
context, restores them on success or failure, and rejects nested/conflicting
policies. It adds both wrapper and trace-contract source hashes and an
`objective-policy/<objective-ID>` semantic key to the exact identity map used by
the existing resume checker. Different objective modes have different maps even
with identical files, data, trainable names and optimizer settings. A base recurrent
checkpoint also lacks that key. This binds declared semantics/source; it is not a
signature or proof against malicious checkpoint rewriting.

The unchanged manifest's diagonal contract remains the strict common data/numeric
admission contract. The checkpoint identity and `objective-contract.json` separately
state the actual optimized score. Final-stage diagonal uses the existing objective
with `final_only=True`; trace uses the EXP104 score with fixed mean/target.

The engine's `valid_components` remains2N scalar flow support. Trace loss itself is
averaged over N pixels; multiplying every validation support count by2 leaves its
weighted mean unchanged. The legacy engine's report field `validation_nll` denotes
the selected objective score and must not be compared across objectives. The
sidecar labels that meaning explicitly; the opt-in CLI prints it as
`validation_objective`. Future comparison uses a separate common evaluator for
all declared diagonal NLL, coverage and ranking metrics.

Bindings are not thread safe. Use the standalone CLI in its own process. Existing
outputs are never overwritten or given new sidecars. Sidecars on a newly created
failed-run directory identify the attempted objective for diagnosis.

## Verification

| Check | Outcome |
|---|---|
| Relevant CPU suite | **120 pass,3CUDA-only skips** |
| Targeted new CUDA replay tests | **2 pass,20deselected** |
| New cases | 20CPU cases +2CUDA cases |
| Same-objective exact replay | Both modes, CPU and CUDA |
| Cross-objective resume rejection | All6directed pairs among recurrent/final-diagonal/trace |
| Independent integrity/replay audit | **7,055 checks pass** |
| Prior source files | 601 unchanged;2additions |
| Real-model optimizer updates / exports / promotion | 0 / none / none |

For each objective/device, the synthetic software fixture runs4updates uninterrupted
and2+2 with resume. Entire checkpoint payloads match recursively, including all
model parameters/buffers, optimizer tensors and group settings, Python/NumPy/torch/
CUDA RNG state, manifest/source/runtime identity and trainable names. Journals
concatenate exactly, validation scores match, all frozen state remains unchanged,
and some trainable state changes. Dropout in the synthetic covariance fixture
exercises restored stochastic RNG. These fixtures are software evidence, not
RGB/thermal performance or real dataset training results.

Other tests verify final-stage-only gradients, exact dispatch to each score,
detached means/targets, semantic keys, sidecar/support semantics, invalid modes,
output preservation and binding restoration on deliberate failures/conflicts.
Both CPU and CUDA scientific test commands succeeded on their first attempts.

The independent audit reloads and recursively compares all four replay outputs,
checks journals/frozen state/sidecars, all six rejected-output absences, prepared
commands and source/data/model pins. It also checks previous EXP102/103/104 result
members against the locally verified recovery manifests. Its first attempt used a
glob longer than pytest's truncated temporary folder names and stopped at the
folder-count check. The corrected directory enumeration excludes pytest symlinks;
the second audit passes. Both audit scripts/logs are retained. No objective,
implementation, test result, metric, data, threshold or scientific outcome changed.

## Native-resolution real-network preflight

Added after the software resume gates passed and declared before real-network
outcomes. The fixed sample is the first broad-manifest training pair,
`abandonedfactory/Easy/P000`, SHA256
`b2759e33f42dbaf5ebb3506a627be1dd43989eef0636e0cc6a34236a9160ca81`.
The original RGB checkpoint and480×640 native inputs are unchanged. Each of three
arms performs one forward/backward pass, with no optimizer construction or step.

| Objective | Own score (not cross-comparable) | Gradient norm before clipping |
|---|---:|---:|
| Recurrent diagonal NLL | −3.897779942 | 142.225981 |
| Final-stage diagonal NLL | −0.782344162 | 36.830970 |
| Final-stage trace risk | −0.370703429 | 38.031798 |

Every arm has finite gradients on all24trainable tensors/3,075,202scalars and
absent gradients on frozen parameters. All12mean-flow fields are identical across
arms, all have586,972valid scalar components, and model-state digest before/after
is `c190d5c777dffe9543991750b231b9b644c757052e799c38762f66d0798c0bdc`,
matching EXP102 initialization. Gradient hashes/norms and complete state/pixel
bindings are retained. No gradient clipping or optimizer update occurs here; all
three norms exceed the proposed clip1, so clipping frequency and norms remain
required future training outcomes. One pair does not prove48-update stability or
quality, and synthetic exact resume does not replace the planned native replays.

CPU image `sha256:6f8c1a104929edf2f56a9ae26cc01596cd040baf51743f2d5950b71e392b3bf4`;
GPU image `sha256:8f7bf1c66f251755fe796a0ebd281b2131d870bc1f936c5585dcc28e08569a33`.

## Prepared execution and remaining step

The retained plan contains six serial argv commands with exact image ID, mounts,
module/options, fixed broad manifest, fresh output paths and log destinations:
three48-update arms followed by24→48replay for each from its step24checkpoint.
That is144scientific updates plus72verification updates, all still unexecuted.
All start from original initialization, not earlier finetuned candidates. The
future `results/objective-comparison/run-001` directory does not exist.

Before running, commit the concrete execution declaration and host orchestration
script with current source/data/runtime pins, revalidate fresh paths, and bind the
common36pair evaluation to fixed support/metrics. Retain all failures and never
change objectives, hyperparameters, windows or reporting based on outcomes.
No additional user permission is required for this authorized local work.

Scope remains exposed synthetic RGB development prefixes, unknown supplied-network
pretraining overlap, and no independent-test, thermal, physical VO or generalization
claim. Future evaluation evidence still needs its own exposure contract. The prior
seven MS² reference-readiness requirements and protected reserves remain unchanged.

Recovery: `backups/2026-11-02-objective-bound-runner`, incremental from EXP104
`a8dde9404375ea46d57116c1dcba31f18a83877e`. It includes synthetic fixture states,
not newly finetuned real-network weights; prior packets retain datasets/models.
