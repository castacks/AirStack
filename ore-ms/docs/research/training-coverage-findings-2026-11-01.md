# EXP100 — broader data reduces regression at equal updates,baseline remains best

2026-11-01 America/New_York;executionUTC2026-11-02. Before-new-pixel
[data declaration](training-coverage-data-protocol-2026-11-01.json)18af8a0b7;
[admitted execution declaration](training-coverage-execution-protocol-2026-11-01.json)
28fc479d9;[host-orchestration correction](training-coverage-execution-protocol-v2-2026-11-01.json)
bdd96285d before any model command started.

Both48-update finetunes complete. The broader-data arm has lower validation NLL
than repeated use of the original16pairs,but neither beats initialization. The
secondary final-stage score has the same ordering. No candidate is promoted.

## Fixed comparison

Both arms start from the same released RGB checkpoint,with unknown pretraining
membership. Same FP32 deterministic GPU8f7bf1c6 runtime,seed0,batch1,constantAdamW
lr1e-5,betas[.9,.999],epsilon1e-8,decay0,clip1,gamma.85,max_flow400,48updates,
checkpoints24/48,and only24cov_update parameter tensors trainable.

Narrow:original16abandonedfactory/Easy/P000pairs in cyclic order,three passes.
Broad:16pairs each from abandonedfactory/Easy/P000,gascola/Easy/P001 and
hospital/Easy/P000,one round-robin stream(old_i,gascola_i,hospital_i). The new
trajectories are metadata-lexicographically-first eligible Easy prefixes,chosen
before pixel decoding. Every record contributes exactly its first16adjacentpairs;
no sample was chosen or dropped using labels/model errors. The same4amusement/
Easy/P001pairs remain exposed finetuning-development validation monitoring.

Updates/native batch shapes are matched. Data corpus,per-example exposure count
and ordering differ;this is not a factorial causal isolation of diversity alone.
Valid training-component exposures are28,126,398(narrow)and28,308,576(broad).
No independent network-training-held-out test or statistical record-level claim
is established:there is one validation trajectory with4adjacent pairs,one fixed
stream per arm,and unknown original checkpoint overlap.

## Data preparation and integrity

Added32provided syntheticRGBforward-flow pairs from the same official AirLabHF
immutable revision65e00180ea952475878a748543e11e9ec20beaac. Six ZIP metadata
indices pin members/sizes/offsets/CRC and asserted full-archive LFS hashes.
Range acquisition fetched98declaredrawmembers using97,644,763bytes;full archives
were not downloaded or rehashed. Source/format/notice basis is unchanged from
[EXP097](supervised-flow-development-findings-2026-11-01.md).

Native480×640 RGBunitFP32,provided(u,v)flow and source mask0valid are unchanged;
no crop/resize/augmentation. Independent Pillow/NumPy checks cover352pixel/flow/
mask assertions and128preparedarrayfields across all32newpairs. Full source-aware
ledger audit passes24records;the previous22records and conditional MS²reserves
remain unchanged. Both new manifests are admitted. All52prepared source samples
(48potentialtraining+4validation)are declared;the narrow arm uses the original20.

The first orchestration wrapper ran inside a CPU container without a Docker
client and stopped before launching a model command. Failed report/log/empty
stdout and original scripts are retained. Correction bdd96285d moves orchestration
to the host using stdlib hashing;settings,data,metrics,commands and budgets are
unchanged. All5actual model commands then succeed on their first invocation.
No scientific outcome retry or threshold/window/hyperparameter retuning occurs.

## Full common-cohort outcomes

Primary:valid-component-weighted recurrent diagonal Gaussian NLL,omitting its
constant. Secondary:final-stage diagonal NLL. Lower is better for both. The
common training cohort below is only the original16pairs,not all broad-arm data.
All20common samples are evaluated at initialization and each final checkpoint.

| Fixed common cohort/metric | Initialization | Narrow48 | Broad48 |
|---|---:|---:|---:|
| Original16training,recurrentNLL |−6.3236834710|−6.8874302996|−6.7065510671|
| Original16training,final-stageNLL |−1.2385678101|−1.2962055009|−1.2706064269|
| Same4validation,recurrentNLL |−1.8312840658|−1.2862593421|−1.7126711339|
| Same4validation,final-stageNLL |−0.4039463874|−0.2784590337|−0.3603153537|

On validation,the broader arm improves over the narrow arm by−0.4264117918
recurrent NLL and−0.0818563200final-stage NLL. Compared with initialization,
it still worsens by+0.1186129319and+0.0436310337respectively. The narrow arm
specializes more strongly to the shared original training cohort while losing
more validation NLL. This is consistent with a coverage/specialization tradeoff;
its precise cause and broader generality are not identified by this comparison.
No percentage improvement,selected metric,midpoint checkpoint or omitted pair
is used to make a positive adoption claim.

## Model and numerical audit

Independent audit passes484scalar/support/mean comparisons. Baseline recurrent
scores reproduce EXP098validation and EXP099training initialization exactly.
NumPy64 formula agrees with CUDA32 on all60common-cohort evaluations for both
predeclared metrics under unchanged rtol2e-5/atol2e-6. Training/validation support
is fixed at9,375,466and2,191,698components respectively. All240full-resolution
flow-mean fields(20pairs×12stages)have identical SHA256 across all3models.

Each arm changes exactly24covariance tensor entries;404frozen model parameter/
buffer entries match canonical initialization. Checkpoints24/48carry correct
AdamW counters and finite states. All48journalsteps follow declared stream order.
Runner-reported validation NLL exactly agrees with the external fixed-cohort
re-evaluation. All597prior source bytes,old data,old candidates and old validation
artifacts retain their hashes. There is no production code/default modification;
the prior241CPUtests remain applicable,with new data/numerical/model audits here.

Final training-state checkpoint SHA256:

- Narrow48: `fda54401f3fac0cd3df0c2ada8dae05e293d269bc7eb6a1fe1d2e916ee7ea00d`
- Broad48: `23712ed47fac43b4b3a364c7f2174003311f11233e25708a88e5c628d81e7c38`

They retain model,optimizer,RNG and data/source/runtime provenance. They are
isolated experimental containers,not direct legacy-inference weight replacements.
No thermal,pose,VO,physical-accuracy or independent generalization claim follows.
The previous7MS²reference/test requirements remain unresolved.

## Retained artifacts and next scope

`results/training-coverage-comparison/run-001/` retains both protocol stages and
orchestration amendment,source/index/range/acquisition records,new label evidence,
data/model audits,all60per-pair/per-stage evaluation rows,both manifests/streams,
fulljournals/reports and checkpoints24/48. New data are under
`data/tartanair-coverage-development/run-001/`;existing data remain at EXP097paths.
Public protocols use logical script identifiers;the recovery packet retains exact
execution scripts/commands bound by SHA256.

This experiment supports keeping broader coverage in development comparisons,
while initialization remains the current reference winner. Further adaptation
should be declared conservatively and monitored across more than one validation
recording before any quality/adoption claim. Data quantity,adaptation capacity,
regularization and objective alignment remain open factors;this run does not
select among them. Keep already-observed development monitoring distinct from
any eventual independent test.

Recovery: `backups/2026-11-01-training-coverage-comparison` on ore-ms Box root.
