# EXP101 — smaller adaptation reduces drift,but initialization remains best

2026-11-01 America/New_York;executionUTC2026-11-02.
[Before-outcome policy/experiment declaration](final-layer-adaptation-protocol-2026-11-01.json)
dedcf163e. The final-convolution-only policy completes its fixed48updates and
exact24→48resume. Validation NLL regresses less than full-block adaptation,but
still loses to initialization on both predeclared metrics. No candidate is promoted.

## New opt-in policy

`Train/MatchingNet/final_layer_retraining.py` wraps the unchanged EXP096engine.
Only `memory_decoder.cov_update.cov_head.conv4.weight` and `.bias` train:2tensors,
1,154scalars versus the full covariance block's24tensors/3,075,202scalars. This
convolution is shared across12recurrent steps;the training loss still covers all
weighted recurrent outputs. It is not a final-stage-only training objective.

All other modules stay in eval;the decoder's own return-list flag stays true.
The existing engine snapshots every state entry outside the new prefix,so426
parameter/buffer entries must remain unchanged. Process-scoped configure/prefix/
source-identity bindings restore on success and failure. The new file's SHA is
included in checkpoint identity,trainable names are explicit,and a policy sidecar
identifies the method. A full-block checkpoint cannot resume under this policy.
Existing outputs get no new sidecar or overwrite. Existing trainer/loss/model/
inference files and default training behavior are untouched.

Nine new CPU tests cover trainable names/module modes,scope restoration on error,
nesting rejection,source binding,exact synthetic resume/frozen buffers,full-block
checkpoint rejection,existing output protection and invalid layer structure.
Full CPU suite:250pass/1CUDA-onlyskip(9new+241prior). The actual full-model CUDA
fit and exact resume below provide the new policy's real-backend execution check.

## Matched experiment and fixed metrics

Reuse EXP100's broad48pairs/3training trajectories,round-robin order,4exposed
amusement development-validation pairs and same released RGB initialization.
Same seed0,FP32 deterministic GPU8f7bf1c6 runtime,48AdamWupdates/checkpoints24/48,
lr1e-5,betas[.9,.999],epsilon1e-8,decay0,clip1,gamma.85,max_flow400,batch1cyclic.
The full-block comparator is the already-fixed EXP100broad48checkpoint,not a
newly selected or retuned model. Initialization/full-block results are re-evaluated
and reproduce exactly. No new data,parameter/step sweep or metric selection.

Primary is valid-component-weighted recurrent Gaussian NLL(without its constant),
secondary final-stage NLL. The shared training cohort is only the original16pairs,
not all48broad-training pairs. Lower is better for both metrics.

| Fixed common cohort/metric | Initialization | Full block48 | Final layer48 |
|---|---:|---:|---:|
| Original16train,recurrentNLL |−6.3236834710|−6.7065510671|−6.3355000126|
| Original16train,final-stageNLL |−1.2385678101|−1.2706064269|−1.2302119382|
| Same4validation,recurrentNLL |−1.8312840658|−1.7126711339|−1.8069245956|
| Same4validation,final-stageNLL |−0.4039463874|−0.3603153537|−0.3952175191|

Final-layer validation recurrent NLL is+0.0243594702worse than initialization,
and−0.0942534617better than the full-block arm. The final-stage score has the
same validation ordering. On the shared original training cohort,the smaller
policy slightly improves recurrent NLL but worsens final-stage NLL relative to
initialization. All four table rows are retained;no alternate metric is used to
turn the result into a baseline win. Stage weighting and deployment-objective
alignment remain open questions,not a cause established by this experiment.

## Covariance drift diagnostic

For exact same flow means and valid target support,teacher(initialization)to
student diagonal Gaussian KL per component is
`.5 * (exp(-2*(raw-raw0)) + 2*(raw-raw0) - 1)`.
This is a diagnostic;no anchoring/regularization term was added to training.
The KL and squared raw-log-standard-deviation shift are averaged over valid
components and normalized recurrent stage weights. Recurrent NLL above sums
stage weights,so its scale is different. A second variance-ratio/log formula
agrees pointwise at fixed rtol1e-8/atol1e-12. Lower drift is proximity to the
teacher,not a standalone calibration quality criterion;the teacher has zero drift.

| Validation covariance movement from initialization | Full block48 | Final layer48 |
|---|---:|---:|
| Stage-weight-averaged teacher→student KL |0.0246716295|0.0038967015|
| RMS raw log-standard-deviation shift |0.1489615186|0.0642073607|
| Final-stage teacher→student KL |0.0281011490|0.0047301530|

The conservative scope moves covariance outputs less on this fixed cohort while
losing less validation NLL. This comparison changes trainable scope and its
associated gradient/clipping dynamics;it does not isolate a universal mechanism
or establish an optimal parameter count.

## Integrity and exact resume

All3required commands(fit48,resume24→48,fixed3model evaluation)succeed on their
first attempt. There are48candidate updates and24additional verification replay
updates,not72candidate steps. Independent audit passes1,222scalar/support/mean/KL
checks. NumPy64 versus CUDA32 NLL agrees at unchanged rtol2e-5/atol2e-6 on all60
common-cohort evaluations for both metrics. Valid components remain9,375,466on
shared training and2,191,698on validation. All240full-resolution mean fields
(20pairs×12stages)have identical hashes across baseline/full-block/final-layer.

Only the2declared parameter tensors change;426other state entries match canonical
initialization. AdamW state contains exactly2parameter entries with correct24/48
counters and finite moments. Independent fresh-process24→48resume matches full
model,optimizer,RNG and last24step journals exactly after common-device loading.
Policy identity/source SHA and trainable names are retained. Every original597
source file,old dataset,candidate and validation artifact keeps its hash;only the
new opt-in policy/test source files are added. Scope/source bindings restore even
when the underlying engine raises.

Candidate training-state SHA256:
`4c6186a07b7a47a650737405462476168f58913b994626c522206bbdb96367db`.
It includes model/optimizer/RNG/provenance and is not a direct legacy-inference
weight dictionary. No model/default replacement or candidate adoption occurred.

## Scope and next work

All samples are provided syntheticRGBflow development data;original checkpoint
pretraining overlap remains unknown. The same single4pairvalidation recording
has been observed repeatedly and is development monitoring. It cannot support
independent network-training-held-out,thermal,physical VO or generalization claims.
The prior7MS²reference/test requirements remain unresolved.

Before further tuning,it is useful to evaluate the frozen baseline/full-block/
final-layer candidates across additional whole validation recordings under a
metadata-only selection and fixed metric plan. That would test whether the observed
ordering is confined to the current recording. Training-objective/last-stage
alignment can be studied as a separate declared change;this experiment does not
silently alter it or treat lower covariance drift as a replacement objective.

Artifacts: `results/final-layer-adaptation/run-001/` retains exact protocol/pins,
all60per-pair/per-stage NLL/KL/shift values,policy contracts,reports/journals and
checkpoints24/48plus replay48,independent validation and test/verification identity.
Public protocol uses logical script identifiers;the recovery packet retains exact
scripts/commands by SHA256. Existing broad data/full-block controls remain in
EXP100's packet. Recovery: `backups/2026-11-01-final-layer-adaptation` on Box.
