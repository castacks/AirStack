# EXP-085: frozen solver replication on known RGB collapse

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `bb547d3d3`.

## Fixed question and declaration

Replicate EXP084 with unchanged research solver settings on the existing EXP032
middle100-frame window, manifest start0 (original paired start700), previously
examined in EXP033/074. Historical RGB support fails97/99 transitions; this
retrospective stress condition is not an unseen scene or untouched test set.
Test optimizer behavior, support coverage and qualified proxy errors separately.
An optimizer repair cannot be credited with a measurement on an unsupported pair.

Six sequential runs use original, passthrough and repair arms for RGB and thermal,
seed0, FP32, mappingfalse, synchronous original analytic global disparity graph.
Use the identical EXP084 image/checkpoints and frozen EXP083 whitened-Huber
settings, including delta0.1, ten iterations and seventeen candidate attempts.
Production optimizer, graph, original v3 runner, EXP084 adapter and run function
remain unchanged. New versioned entrypoints permit exactly100frames/start0 and
preserve the earlier fixed20-frame experiment. Fatal execution is recorded as
failure without retuning. No training, selector fitting, new scene or reserved
MS² access.

Recover only405 prepared files from the878919049-byte EXP032 archive with
SHA256 `24f4e5af25983bc10ae71d87c44a4e612022754af38bc143611785a52011713f`.
All selected member hashes must match. Before inference, freeze375 source files,
405 inputs, exact checkpoints, six commands, runtime and prior evaluation inputs.
All371 source hashes in EXP084's original inference protocol remain identical;
evaluation source is frozen at evaluation and the validator hashes itself in its proof.

Original/passthrough complete-array parity gates each repair run. All99 interval
slots per camera/arm are retained, including missing results; only first genuine
pre-refinement motion writes are scored. Stale repeated writes and filled poses
do not create new measurements. Terminal writes remain explicit. Full coverage
and matched original/repair scores must both be reported.

Use the unchanged EXP075 nominal-rig/reference-assisted SLAM proxy and separate
translation/rotation losses. Find each camera's first available validated middle
rig transform chronologically and check consistency across prior available
experts. Common intervals are original RGB transitions; own sensor intervals and
approximately8ms camera offsets remain explicit. No scale/alignment/threshold
fit. Reference independence and physical rig validity remain unresolved.

## Results

All six100-frame runs complete. Each camera original/passthrough control preserves
all2613 arrays across103NPZ files bitwise; every original/repair raw frontend and
uncertainty array matches (2577arrays/100NPZ per camera). Effective configurations
match. Both RGB arms retain54 observations and97/99 unsupported transitions;
both thermal arms retain17736 observations and support99/99. No tracking recovery
is observed. Filled/refined poses remain separate from genuine motion evidence.

All594 interval slots are retained:303 valid arm/camera scores and291 explicit
missing slots. Each RGB arm has two first writes (pair96 during processing and
pair98 during termination), plus one stale repeated write excluded from scoring.
Each thermal arm has99 first writes (98processing,1termination). Independent
audit checks all2112 events. The two supported RGB motions are the complete
matched RGB population, not a selected subset of successful scores.

| Camera / arm | Matched intervals | Translation mean m | Translation RMSE m | Rotation mean deg | Rotation RMSE deg |
|---|---:|---:|---:|---:|---:|
|RGB original|2/99|3.312922778|4.038666435|25.251417699|35.488721587|
|RGB repair|2/99|2.519732632|3.002267241|18.554269246|26.023199289|
|Thermal original|99/99|0.038378406|0.042871876|0.597856341|0.676590331|
|Thermal repair|99/99|0.037625980|0.041548897|0.598248977|0.676750109|

Passthrough scores equal original scores. Thermal translation RMSE falls3.086%;
rotation RMSE increases0.024%, effectively similar at this scale. RGB translation
and rotation RMSE fall25.66% and26.67% on only two supported intervals, with large
remaining proxy errors and unchanged97/99 unavailability. Comparing RGB's two
scores to thermal's99 would mix populations. This is not a trajectory/coverage
rescue, general improvement or independent physical accuracy result.

All101 repair solves reach `MAX_ITERATIONS`:1010 accepted steps in1013 attempts.
Three worsening candidates are rejected (one on RGB96,two on RGB98); independent
checks verify restoration and acceptance arithmetic. No fatal failure occurs,
but cap termination is unresolved, not convergence. Original termination remains
unqualified. The repair can change supported graph solutions; it cannot supply
observations to the camera-local support gate. In this pipeline the outlier filter
uses camera-local matches/depth/covariance before world-pose registration.

Process wall times original/passthrough/repair: RGB70.20/70.08/70.90s,
thermal43.01/35.82/30.01s. Per-solve passthrough/repair medians are26.16/24.75ms
RGB and34.44/18.13ms thermal. Wrapped peaks are11.756GiB RGB and5.720GiB
thermal; original peak memory was unmeasured. Single sequential runs, startup,
instrumentation and different stopping rules prevent a speed/real-time claim.
All own/common losses, sensor intervals, missing rows and PNG/SVG/CSV exports
remain available; plot gaps explicitly represent absent matched motion.

## Verification and limitations

All33 inherited solver/adapter tests pass in the unchanged pinned image. Protocol
checks verify all371 original frozen source hashes, exact checkpoints/image/repair
settings and all99 prior reference intervals. The independent audit recomputes
raw-array preservation, health/flags, first versus stale writes, sensor times,
reference interpolation/frame transport, losses, complete/matched populations,
and all1013 repair attempts/1010 accepted steps. Maximum relative finite-difference
gradient discrepancy is4.91e-5 at the unchanged1e-8 step; tolerances are unchanged.
Nine isolated mutations must reject altered loss, sensor interval, transform,
reference bridge, missing row, RMSE, invented convergence, physical-truth claim
and array parity. Original data is read through links; copied metadata is mutated.

The new audit wrapper initially omitted imports for `json` and `saved_context`.
Both failed logs are retained. Only imports were corrected; inference sources,
artifacts, settings, numerical criteria and evaluation were unchanged. The final
proof records the corrected validator SHA256. No inference rerun was needed.

These previously examined windows and the nominal-body/SLAM-reference proxy do
not establish held-out transfer, physical calibration or reference independence.
Combined with EXP084, this is evidence of a bounded numerical repair with mixed
proxy effects, not a validated general VO method. Keep production defaults
unchanged. A separate unchanged-parameter test on the already examined day window,
where both cameras track, would test another condition before any adoption.

## Reproduction

Restore the EXP032 middle prepared dataset, EXP075 proxy records and full night
reference, original two checkpoints and EXP084 source/runtime prerequisites.
From `MAC-VO-main/`, host orchestration requires only Python3 stdlib and Docker:

```bash
python3 -m Scripts.Setup.vo_solver_replication prepare --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/solver-collapse-replication/new-run
python3 -m Scripts.Setup.vo_solver_replication run --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/solver-collapse-replication/new-run
```

Prepare refuses an existing output directory and records exact commands. In the
pinned image with ore-ms mounted at `/workspace`, evaluate and audit separately:

```bash
python -m Scripts.Setup.evaluate_vo_solver_replication --root /workspace --output /workspace/results/solver-collapse-replication/new-run
python -m Scripts.Setup.validate_vo_solver_replication --root /workspace --output /workspace/results/solver-collapse-replication/new-run
```

Authoritative artifacts: `results/solver-collapse-replication/run-001/`.
Recovery target: `ore_ms:ore-ms/backups/2026-10-29-exp-085`.
The incremental source bundle requires commitbb547d3d3; data/reference dependencies
are EXP032/075 and EXP083/084 plus their parents. Model weights, prepared images
and Docker layers are excluded from this incremental backup; their identities
and recovery prerequisites are recorded. Credentials are excluded.
