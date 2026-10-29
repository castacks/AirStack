# EXP-077: unchanged history rule on a separate day recording

2026-10-29 America/New_York. Branch `ore_ms`, base `fbdedfaa2`.

## Declared condition

Replicate the frozen EXP076 two-interval history rule on exactly the existing
EXP026 middle100-group window, prepared manifest start0, original source start584,
recording `20251027/0_salon_so_day`. This recording differs from the night
`night_mac_salon_warehousefig8` used to develop the rule. Its historical VO
outcomes were already examined in EXP026, so this is separate-recording
replication, not unseen evaluation, independent physical accuracy or verified
checkpoint-training exclusion. Do not choose alternate starts or tune the rule.
No reserved MS² recording or new scene is accessed.

Recover411 files from the EXP026 archive:405 prepared dataset files, five
reference-export files and the original preparation plan. Archive size1461483506
bytes and complete SHA256 match its original manifest; every restored member
matches its SHA256. No raw-image processing is repeated or substituted. Preserve
the original thermal MSA rectification and RGB images, exact timestamps and all
dropped-capture gaps. Original middle duration is10.199505091s RGB/10.200000048s
thermal, with3 gaps>150ms per camera. RGB-minus-thermal left acquisition offsets
range−23.000002ms to+28.414965ms, retaining changing camera order within groups.
The original chronological-group scheduler contract still holds.

## Frozen method

No changes to the scheduler, event recorder, original availability rule, two-pair
history confirmation, seed0, FP32, synchronous optimizer, checkpoints or estimator
defaults. Use the same pinned Blackwell runtime as EXP073–076. Hash and freeze
sources, inputs, checkpoints and exact commands before fresh standalone v2 RGB
and thermal controls and one shared100-group replay. Controls establish numerical
preservation in this runtime, not bitwise identity with the historical runtime.

Reuse the exact history function. An expert requires matching complete supported
first motion for target and preceding intervals at the original deadline;
length2 and RGB tie preference remain fixed. Pair0 cold-start abstains. The final
pre-drain abstention remains. Ignore newer-than-target intervals even if observed.
Retain missing/unsupported/stale states, raw covariance context and camera offsets.
No reference or proxy loss enters causal prediction.

The bounded adapter `history_day_replication.py` supplies the new dataset/reference
paths while leaving all frozen producer functions unchanged. Predictions are
saved before loading label inputs. Offline labels reuse EXP075's supplied
nominal body transforms, explicit near-SO3 roundoff policy, recorded SLAM reference,
reference endpoint bridges to each RGB interval, and separate meter/degree
residuals. These remain conditional SLAM-proxy comparisons, not independently
verified physical body/pose labels or deployable reference-assisted compensation.

Reuse the unchanged separate EXP076 scorer, comparing RGB-only, thermal-only,
fixed RGB preference and history. Report full available populations plus exact
matched intersections with history. No mixed-unit loss, coverage penalty,
confidence calibration, fitting, selector adoption or fused trajectory.

## Verification plan

`validate_history_day_replication.py` preserves the independent shared-event,
pose-label and history/population reconstruction logic of EXP074–076, bound to
this declared day manifest/reference and99 decisions. It imports neither the
history predictor nor scorer. Three stages independently check acquisition
release order/deadlines/live prefixes, nominal SE3 label reconstruction, and
history actions/full/matched populations. Existing sparse event audits compare
both shared expert outputs with fresh standalone controls. All original helper
hashes must remain unchanged after execution. CPU regressions and semantic
mutation checks accompany the actual run.

## Actual results

Both experts have99/99 originally supported pairs,99 first motion writes and no
repeated writes or observer errors. Each emits497 events. The original availability
rule has98 RGB decisions plus one final pre-drain abstention. The unchanged history
rule has97 RGB decisions and two abstentions: the only additional abstention is
pair0 cold start. Every remaining selected expert matches fixed RGB preference.
No recovery intervention is exercised in this entirely supported window.

| Policy | Available /99 | Full-population translation proxy RMSE (m) | Rotation proxy RMSE (deg) |
|---|---:|---:|---:|
| RGB-only |98|0.031459|0.12417|
| Thermal-only |98|0.077786|0.61578|
| Fixed RGB preference |98|0.031459|0.12417|
| Two-interval history |97|0.031505|0.12478|

Matched support removes the cold-start population difference:

| Same97 intervals | Translation proxy RMSE (m) | Rotation proxy RMSE (deg) |
|---|---:|---:|
| RGB-only / fixed RGB preference |0.031505|0.12478|
| History |0.031505|0.12478|
| Thermal-only |0.078053|0.61808|

History therefore replicates as a conservative availability/recovery control,
without a matched quality improvement over fixed RGB. It costs one interval of
coverage compared with either single expert in this condition. RGB has lower
aggregate conditional proxy loss here, whereas thermal was preferable in the
known night collapse condition. This does not establish a universally best
expert, a history advantage over the best fixed expert or physical accuracy.

On all98 original common-eligible intervals, translation proxy loss favors RGB63
times and thermal35 times. The reference-assisted per-interval oracle has RMSE
0.028456m versus fixed RGB0.031459m on those same98 intervals. This describes
remaining quality-selection headroom, not an achieved causal selector: the
oracle sees offline reference losses and cannot be used by the predictor.

## Verification outcome

| Check | Result |
|---|---|
| Exact selected restoration |411 files, original archive/member SHA256 PASS |
| Original source/model/input freeze |PASS before and after all runs |
| RGB standalone vs shared |2613 arrays across103 NPZ files bitwise equal |
| Thermal standalone vs shared |2613 arrays across103 NPZ files bitwise equal |
| Independent shared-clock audit |200 scheduled frames,99 deadlines,4158 decision fields |
| Independent nominal pose reconstruction |99 rows,10192 numeric components |
| Independent history/population audit |99 decisions,2376 action/loss fields |
| CPU regressions |76 passed |
| Semantic mutations |All6 rejected |
| Reference/label read guard |Read-text and read-bytes forbidden for label/reference paths; predictions byte-identical |

Semantic mutations cover cold-start promotion, future prior-pair access,
changed deadline, altered loss, inflated matched count and dropped abstention.
The outer prediction digest is updated in each challenge so failures exercise
semantic checks. These are software audits, not independent scientific trials.
Numerical preservation is fresh same-runtime preservation, not historical
EXP026 runtime parity. Pose labels retain all nominal-body/SLAM-proxy limits.

Authoritative artifacts are under `results/history-day-replication/`: restoration
proof, frozen capture plan and commands, standalone/shared logs and outputs,
both preservation proofs, CPU JUnit, label guard and mutations,
`run-001/` with the original shared events, `history-001/` with frozen predictions
and separate scores, and `proxy-001/` with offline labels and independent proof.
The adapter/core/scorer sources remain unchanged after execution.
GPU image is unchanged:
`sha256:700f33b2f919364dfd8aefe486568ff69a7832cc5942149f03eef5b561230239`.
Shared timeline SHA256:
`455e1868512e312ac6be9d59c622f5d62dfee0e07be3c26fdb0098e55e2e1803`.
History prediction SHA256:
`b5869bd7887169058922b62431de508cab9d341ac07e540931cf2703baba25b8`.

## Reproduction and decision

The frozen capture plan supplies all actual Docker commands, mounts, file hashes
and thread settings for the two standalone runs and one shared run. Use the
unchanged `run_td3_execution_v2` and `shared_arbiter_replay` with this middle dataset,
100 frames,start0 and a fresh output. Run the existing sparse event validator per
expert, then the new bounded auditor's `shared` stage. In the pinned runtime from
the MAC-VO root, the subsequent stages are:

```bash
python -m Scripts.Setup.history_day_replication predict --root /workspace \
  --output /workspace/results/history-day-replication/history-001
python -m Scripts.Setup.history_day_replication label --root /workspace \
  --output /workspace/results/history-day-replication/proxy-001
python -m Scripts.Setup.validate_history_day_replication proxy --root /workspace \
  --run /workspace/results/history-day-replication/proxy-001
python -m Scripts.Setup.score_arbiter_history --root /workspace \
  --run /workspace/results/history-day-replication/history-001 \
  --labels /workspace/results/history-day-replication/proxy-001
python -m Scripts.Setup.validate_history_day_replication history --root /workspace \
  --run /workspace/results/history-day-replication/history-001
```

All outputs must be fresh or immutable as specified by their producer. The
recovery snapshot is `backups/2026-10-29-exp-077`, dependent on EXP026 prepared
inputs/reference and EXP071–076 code/calibration/checkpoint recovery. No images,
weights, credentials or full source archives are duplicated in the new snapshot.

Keep the history rule fixed as a recovery baseline. The supported day replication
adds no quality benefit over RGB-only and no new switching behavior, while retaining
the declared cold-start cost. The next useful research question is quality when
both experts are supported: characterize available uncertainty/optimizer-health
inputs against the fixed proxy losses before choosing another selector. Do not
tune confirmation length to these outcomes or portray the proxy oracle as a
working quality gate. Reserved MS² captures remain untouched.
