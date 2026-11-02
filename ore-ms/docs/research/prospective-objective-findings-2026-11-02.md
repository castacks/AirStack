# EXP107 — Prospective frozen-candidate evaluation

The earlier 3.05% development ranking gain from final-trace versus final-diagonal training shrinks to **0.00479%** on two additional environment prefixes. Per-record directions oppose each other, the pooled micro comparison reverses sign, and final-trace likelihood is worse overall. This does not establish a stable trace-objective advantage. The recurrent arm retains the lowest primary macro ranking error. No model is promoted.

## Freeze and exposure

All four EXP106 models and runtime image `sha256:9c0ea2714e913f310b99bd37ddbcae7b616789f8fae856f325030d57b4f4ac26` remain fixed. Zero optimizer steps and zero new model/checkpoint exports. Selection begins at `3fb980635`; exact-member/support/metric freeze is `01f6146e8`; native execution freezes at `0e82ed18b`, after data audit and before model scores.

The [pinned AirLab TartanAir mirror](https://huggingface.co/datasets/theairlabcmu/tartanair/tree/65e00180ea952475878a748543e11e9ec20beaac) supplies the first 16 provided-flow pairs of `carwelding/Easy/P001` and `endofworld/Easy/P000`. Environment families were the first two lexical catalog entries absent from the 26-record ledger, excluding night/winter variants of already observed families.

The original fixed-P000 declaration stopped during metadata-only inspection because Carwelding P000 is absent. No member payloads or model scores were opened. The initial reader also assumed a combined flow archive; the mirror uses separate flow/mask archives. Availability amendment `d1442b573` retains the same environments/models/metrics and selects the lexically first trajectory with complete first17image/16flow/16mask ZIP entries. The corrected metadata pass selects Carwelding P001 and Endofworld P000; exact members are committed before acquisition. This is an explicit pre-payload selection revision, not an unchanged original declaration. Both metadata attempts are retained. No post-decode substitution occurs.

All 98 selected raw members pass ZIP CRC, size and SHA checks; only 92,913,670 bytes are transferred for acquisition (whole ZIP hashes are source assertions, not recomputed). Native 480×640 RGB/255, provided forward flow, mask-code-zero validity, finite-GT and norm(flow)<400 support remain unchanged. All 32 pairs are retained. Independent preparation checks pass for 128 array fields / 352 checks, with 34 distinct image hashes and zero overlap with the earlier training/evaluation images. Data audit: 1,238 checks.

The ledger appends two development-exposed recordings (28 total), preserving every prior record and reserved status. These prefixes were not used in our finetunes or prior candidate scoring. Original network pretraining membership remains unknown; this is prospective to this project comparison, not proof of network-training-heldout data.

## Frozen primary: two-record equal-record macro

Primary: squared endpoint error among the lowest-trace 50% of valid pixels within each pair; pool sums/counts within each recording, then average the two recording risks equally. All five retention curves, random/oracle controls, per-record/macro/micro likelihood, coverage and standardized moments are retained. Nominal joint coverage uses the same diagonal-Gaussian ellipse diagnostic as EXP103/106.

| Model | MSE50 (px²) | Joint coverage at nominal 95% | Final diagonal NLL | Recurrent diagonal NLL |
|---|---:|---:|---:|---:|
| Original checkpoint | 0.104354201 | 95.0892% | -0.659650 | -3.139407 |
| Recurrent diagonal | 0.101222899 | 93.9821% | -0.675409 | -3.362590 |
| Final diagonal | 0.101581155 | 94.4827% | -0.671477 | -3.156131 |
| Final trace | 0.101576287 | 94.5445% | -0.669154 | -3.176537 |

Final trace versus final diagonal: 0.101581155→0.101576287 px², an absolute change of −0.000004869 px² (−0.0047929%). Macro joint-95 coverage moves 94.4827→94.5445%; final diagonal NLL worsens −0.671477→−0.669154 (lower is better). Near-nominal aggregate coverage does not establish recording-wise or physical calibration.

## Per-record results

| Recording | Model | MSE50 (px²) | Joint 95% coverage | Final diagonal NLL |
|---|---|---:|---:|---:|
| carwelding | Original checkpoint | 0.156896899 | 94.3804% | -0.388812 |
| carwelding | Recurrent diagonal | 0.150526933 | 92.7386% | -0.397925 |
| carwelding | Final diagonal | 0.150835732 | 92.9421% | -0.399332 |
| carwelding | Final trace | 0.150632865 | 93.0971% | -0.407884 |
| endofworld | Original checkpoint | 0.051811502 | 95.7979% | -0.930489 |
| endofworld | Recurrent diagonal | 0.051918865 | 95.2255% | -0.952894 |
| endofworld | Final diagonal | 0.052326579 | 96.0232% | -0.943623 |
| endofworld | Final trace | 0.052519709 | 95.9918% | -0.930425 |

The trace arm lowers Carwelding ranking error and worsens Endofworld ranking error. Its final NLL also improves on Carwelding and worsens on Endofworld. The predeclared dominance gate therefore fails. The recurrent arm has the lowest ranking error among the adapted models on both records, but does not beat the original checkpoint on Endofworld.

## Pooled micro sensitivity

| Model | Pooled MSE50 (px²) | Pooled joint 95% coverage | Pooled final NLL |
|---|---:|---:|---:|
| Original checkpoint | 0.102373450 | 95.1159% | -0.669860 |
| Recurrent diagonal | 0.099364240 | 94.0289% | -0.685870 |
| Final diagonal | 0.099724361 | 94.5407% | -0.681737 |
| Final trace | 0.099726956 | 94.5990% | -0.679004 |

Pooled micro reverses the near-zero trace/diagonal ranking difference: final trace 0.099726956 versus final diagonal 0.099724361 px². This sensitivity is reported alongside the unchanged primary, not used to select an alternative winner.

## Independent audit and retained numerical failure

The final independent audit passes 7,492 checks, including all input/raw/member hashes, exact PIL/native-array preparation, support/reference identity, 749 protected source/data/artifact pins, fixed checkpoint digests, likelihood, coverage counts, standardized moments, stable-tie risk/oracle/random sums and per-record/macro aggregation. Three semantic mutations (primary macro, coverage95, retained population) are rejected. All 384 common recurrent mean fields are shared across four models; model-state digests equal EXP106 before/after inference, all gradients remain absent, and no optimizer is constructed.

Inference, metrics and pooled accounting succeed on their first attempts. The original final auditor fails its 2e-12 relative/absolute signed raw-sum comparison for standardized bias because a contiguous one-dimensional sum uses a different addition order from the producer’s strided two-dimensional sum. That failure log, original script and failed execution journal are retained. No results are overwritten or rescored.

Repair `15ad944f2` commits a numerical accounting change before the second audit: reference the signed sum with `math.fsum`, and bound producer summation error by `gamma_(N−1) × sum(abs(z)) + u × abs(fsum(z))`, with `u=eps64/2`, `gamma_k=ku/(1−ku)`. This is a derived forward-error envelope, not an outcome-chosen calibration tolerance. All other numerical, exact-support, model, coverage, ranking and aggregation checks remain unchanged. Maximum observed signed-sum discrepancy is 2.289e−9, corresponding to at most 7.966e−15 in a reported standardized mean; the maximum discrepancy uses only 0.0372% of its roundoff envelope. Frozen evaluation/capture/metrics/micro hashes remain exact across the repair. `completion.json` records final success while preserving the original failed execution journal.

## Decision and limits

Retain the recurrent arm as the ranking reference for further evaluation and retain every candidate as a research artifact; do not promote any model. Stage/objective changes have not established a useful, consistent ranking benefit over the recurrent control. A further objective change or additional fitting needs its own predeclared rationale and data/exposure contract.

These are only two temporally correlated synthetic RGB prefixes. Neither pixels nor adjacent pairs are independent recording replicates. No population significance, network-training-heldout, thermal, physical VO, universal calibration or deployment claim follows. The prior seven MS² reference-readiness requirements and protected reserves remain unchanged.

Artifacts: `results/prospective-objective-evaluation/run-001/`, `data/tartanair-prospective-evaluation/run-001/`, and both `results/prospective-objective-metadata/` attempts. Recovery target: `ore_ms:ore-ms/backups/2026-11-02-prospective-objective-evaluation`, incremental from EXP106 `98e869375`. The exact runtime image remains in the EXP106 backup; no new model binaries are duplicated.
