# EXP102 — frozen final-layer candidate wins the new-record macro;rankings vary

2026-11-01 America/New_York;executionUTC2026-11-02.
[Before-new-pixel declaration](frozen-multirecord-data-protocol-2026-11-01.json)
948c64091;[admitted before-score execution declaration](frozen-multirecord-execution-protocol-2026-11-01.json)
1bf520856. All3candidates were frozen before new images/labels were decoded.
No training update,model export or promotion occurs in this experiment.

The final-covariance-layer candidate improves over initialization on both added
recordings and has the lowest predeclared new-two-record macro NLL. The full-block
candidate has the lowest recurrent NLL on office,while initialization still wins
the legacy amusement recording. The old single-record ordering was not universal.
This is limited provided-reference syntheticRGB evidence,not a network-training-
held-out,thermal,physical VO or generalization proof.

## Before-payload selection and evaluation-only data

Fixed environments:office and japanesealley. Metadata-lexicographically-first
eligible Easy trajectories are office/P000 and japanesealley/P001. Exactly the
first16adjacent left-image/forward-flow/mask pairs per trajectory were selected
before pixels,without label/model-error/appearance filtering. The same4legacy
amusement/P001pairs remain for exact reproducibility. Whole trajectories are
separate from our3finetuning trajectories;only the declared prefixes are evaluated,
not entire recordings. Original base pretraining membership remains unknown.

Same immutable official AirLabHF65e00180 source/format/notice as EXP097. HTTPS
range acquisition fetched98rawmembers using91,504,295bytes,plus5,103,318bytes of
metadata reads. Exact206ranges,length,central-entry sizes/offsets,ZIPCRC and
extracted member SHA256 were checked. Full-archive LFS hashes are source assertions,
not recomputed whole-download hashes. No pose/depth/right-image payload extracted.

Prepared32newpairs at original480×640RGBunitFP32/provided(u,v)flow/source-mask0valid,
without transforms. Independent Pillow/NumPy preparation passes352checks across
128arrayfields. Source-aware exposure-ledger audit passes26records with all24prior
records/reserves preserved. Both new IDs are absent from our finetuning manifests.
The new manifest has an evaluation-only task and no training assignment;the
training manifest inspector rejects it as intended. All36pairs are retained.

## Frozen candidates and metrics

The candidates are the same released initialization,EXP100full-block broad48
checkpoint and EXP101final-convolution broad48checkpoint. Models are not chosen
or modified using the new outcomes. Same deterministic FP32 GPU8f7bf1c6 runtime,
source-valid masks,max_flow400,gamma.85,and12recurrent outputs as prior experiments.

Primary:equal-record mean of the two new recordings' valid-component-weighted
recurrent Gaussian NLL(without its additive constant). Secondary:the corresponding
final-stage NLL. Lower is better. New-only,legacy-only and all3record groups are
reported with both equal-record macro and component-pooled micro aggregation.
No aggregate/metric/record is selected after seeing outcomes.

### Per-record scores

| Fixed recording/metric | Initialization | Full block48 | Final layer48 |
|---|---:|---:|---:|
| Office16pairs,recurrentNLL |−7.8976327995|−8.1509837356|−8.0836814788|
| Office16pairs,final-stageNLL |−1.5378727373|−1.5278259016|−1.5497321600|
| Japanesealley16pairs,recurrentNLL |7.0942499234|7.1381671776|6.5378204774|
| Japanesealley16pairs,final-stageNLL |1.1604669527|1.2811996517|1.0560756814|
| Legacy amusement4pairs,recurrentNLL |−1.8312840658|−1.7126711339|−1.8069245956|
| Legacy amusement4pairs,final-stageNLL |−0.4039463874|−0.3603153537|−0.3952175191|

New-record component counts are9,560,066(office)and7,974,402(japanesealley);
legacy has2,191,698. Every model uses exactly the same support within each record.
The full-block recurrent gain on office coexists with worse final-stage NLL on
both new recordings. The final-layer candidate improves both NLL metrics over
initialization on both new records,but regresses on amusement. There is no single
uniform winner across all per-record/metric cells.

### Predeclared aggregates

| Group/aggregation/metric | Initialization | Full block48 | Final layer48 |
|---|---:|---:|---:|
| **New2record macro,recurrent(primary)** |**−0.4016914381**|**−0.5064082790**|**−0.7729305007**|
| New2record macro,final-stage |−0.1887028923|−0.1233131250|−0.2468282393|
| New2record micro,recurrent |−1.0795588454|−1.1977168546|−1.4340509088|
| New2record micro,final-stage |−0.3107100187|−0.2503249823|−0.3646514800|
| All3record macro,recurrent |−0.8782223140|−0.9084958973|−1.1175951990|
| All3record macro,final-stage |−0.2604507240|−0.2023138679|−0.2962913326|
| All3record micro,recurrent |−1.1630801269|−1.2549314326|−1.4754794618|
| All3record micro,final-stage |−0.3210691509|−0.2625455870|−0.3680475543|

The primary final-layer minus initialization difference is−0.3712390627;versus
full-block it is−0.2665222217. Final-stage new-record macro also improves over
initialization by−0.0581253470. Macro and micro differ because valid component
counts differ;the primary gives each new recording equal weight. Legacy-only
macro/micro exactly equal the four-pair legacy scores above. No percentage
improvement,pixel-independent confidence interval or model promotion is inferred.

## Independent integrity

The single fixed evaluation attempt succeeds. Independent audit passes2,445
scalar/support/mean/KL/aggregation/legacy-replay checks. NumPy64 NLL agrees with
CUDA32 at fixed rtol2e-5/atol2e-6 on all108model-pair evaluations for both metrics.
KL/raw-logstd drift statistics remain diagnostics,not new objectives or selection
criteria;the alternate KL formula and aggregation identities are checked.

All432full-resolution mean fields(36pairs×12stages)are identical across candidates.
The differences are covariance-output effects on fixed source means and masks;
no improvement in flow-mean accuracy or VO motion is measured. Every model's
active parameter/buffer digest is identical before/after inference and all
parameter gradients remain absent. Zero optimizer steps are executed.

All3legacy candidates reproduce every existing legacy per-pair score,stage hash,
mask and drift value exactly. All599production source files,old raw/prepareddata,
trained candidates and prior validation artifacts retain their hashes. The prior
250CPUtests remain applicable;there are no production code changes in this data/
evaluation experiment. Original initialization and candidate defaults stay intact.

## Interpretation and next diagnostic

This is additional evaluation evidence for the frozen final-layer adaptation,
with a reproducible improvement on both newly added prefixes. Its significance
is limited by2newtrajectories,adjacent prefixes,provided synthetic labels,and
unknown base-network training overlap. No values were fitted using these new
records,but they are now observed project development data. Do not relabel them
as an untouched future test,network-training-held-out data or independent physical
accuracy evidence. The previous7MS²reference/test requirements remain unresolved.

The next useful check is frozen-model uncertainty calibration/risk ranking on the
same declared cohorts,without tuning:likelihood gains alone do not show whether
confidence ranks bad correspondences or has useful coverage. Keep the current
NLL outcomes and frozen model endpoints as primary evidence;any extra diagnostic
must declare its own definitions and remain development analysis. There is still
no basis here to promote a thermal/VO candidate or alter the state boundary.

Artifacts: `results/frozen-multirecord-evaluation/run-001/` retains protocol/source/
modelpins,ZIP/range/acquisition records,label/evaluationmanifest evidence,source
ledger audit/admission rejection,all108per-pair/per-stage outcomes,per-record and
all macro/micro results,model state digests and independent validation. New raw
and prepared data are under `data/tartanair-frozen-evaluation/run-001/`. No trained
model is newly exported. Recovery includes all exact scripts/logs;public protocol
uses logical script identifiers bound to retained command/source SHA256.

Recovery: `backups/2026-11-01-frozen-multirecord-evaluation` on ore-ms Box root.
