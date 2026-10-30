# EXP-087: forward/backward flow closure as camera-quality evidence

2026-10-30 America/New_York. Branch `ore_ms`, starting commit `24efa5c32`.

The fixed closure diagnostic matches thermal-only on both night windows but
worsens daytime translation/rotation SLAM-proxy RMSE versus RGB-only by38.81%/
60.70%. This additional correspondence signal does not establish a better camera
selector. No gate is trained or promoted; original VO/defaults remain unchanged.

## Declared measurement and preserved controls

Execute the [previously frozen protocol](../docs/research/flow-cycle-quality-protocol-2026-10-30.md)
on original EXP084early20, EXP085middle100 and EXP086day100 windows, start0,
with original checkpoints and the exact pinned `macvo:solver-vo` image
`sha256:e1e72a2dc43630ffe47cf45c29278bd879d36b9bfb9fd6d621c1e88f88455b82`.
No new scene, reserved MS², training, selector fitting, solver change or trajectory
composition. These are known-window exploratory results, not unseen evaluation.

Freeze385source files,1335prepared-image/config/original-grid files, two checkpoint
identities and six exact commands before inference. Native FlowFormerCovFrontend
uses FP32, seed0, original batch2 stereo+temporal inference and two initial-depth
calls per process. Original integer-grid forward endpoints must reproduce cached
`grid_uv1` **bitwise** before each reverse call. Every434forward controls pass;
434reverse predictions and12warmup depth calls complete. No estimator/optimizer
is instantiated, and original motions, point selection and outputs stay fixed.

At each original audit-grid source u, use its unchanged forward endpoint v and
sample reverse flow f10(v). Closure is v+f10(v)−u. Native frontend retrieval provides
integer sampling only; a separate research bilinear sampler uses the same
integer-centered coordinates, inclusive image borders and four finite corners.
Out-of-bounds/invalid samples remain explicit; valid samples are never clamped.
Retain original forward-inbound masks. Save corner indices/values/weights, sampled
flow, every closure/mask and a SHA256 of the canonical dense reverse field.

Primary scalar is median sqrt((closure_x/fx)^2+(closure_y/fy)^2). The fixed variance
baseline is median(Sigma_uu/fx²+Sigma_vv/fy²) over original grid values with finite,
positive covariance diagonals. Neither scalar is calibrated pose risk/probability.
Of609336grid samples,529144have valid closure; all434camera/interval diagnostics
are defined. No mask/count, statistic or threshold is chosen after scoring.

GPU audit hooks reject pose/reference/score file reads. A separate guarded
prediction phase reads only captures and original native event availability,
saves all217choices before the evaluator opens proxy labels, and records zero
denied label reads. Original first-write eligibility is independent of error
values. RGB-only/thermal-only retain their own availability. Diagnostic policies
choose the lower scalar among motion-supported cameras with defined scores;
exact ties prefer RGB, no eligible candidate means abstain.

## Results on matched both-motion populations

Use unchanged nominal-body/reference-assisted SLAM-proxy losses. Physical rig
validity and independent ground truth remain unverified. All policies below have
the same exact19/2/99 intervals in their window, including terminal first writes.
Meters and degrees remain separate; no mixed-unit or cross-window aggregate.

| Window / N | Policy | RGB / thermal choices | Translation RMSE m | Rotation RMSE deg |
|---|---|---:|---:|---:|
|Early /19|RGB-only|19 /0|0.009971226|0.068001533|
|Early /19|Thermal-only|0 /19|0.009260253|0.031828841|
|Early /19|Variance|2 /17|0.009323917|0.031677203|
|Early /19|Closure|0 /19|0.009260253|0.031828841|
|Middle /2|RGB-only|2 /0|4.038666435|35.488721587|
|Middle /2|Thermal-only|0 /2|0.052767988|0.987251623|
|Middle /2|Variance|0 /2|0.052767988|0.987251623|
|Middle /2|Closure|0 /2|0.052767988|0.987251623|
|Day /99|RGB-only|99 /0|0.031673337|0.123630496|
|Day /99|Thermal-only|0 /99|0.077469182|0.612931850|
|Day /99|Variance|93 /6|0.044071545|0.188857991|
|Day /99|Closure|93 /6|0.043965437|0.198670299|

On all99middle union slots, variance and closure choose thermal throughout,
matching thermal-only0.042871876m/0.676590331deg with99/99coverage. RGB has only2
valid motions and97abstentions. This is not an advantage over thermal-only;
sparse RGB losses are not compared to99thermal losses as if populations matched.
All217interval slots and every full/pairwise-matched policy comparison are kept.

Removing any terminal motion from the both-supported population leaves18/1/98
pairs. Closure still matches thermal early/middle. Day closure becomes0.043919099m/
0.199627929deg versus RGB0.031458553m/0.124173799deg. Thus the daytime failure is
not solely the terminal-write convention. Processing-only counts do not establish
shared-clock/deadline availability.

## Every daytime thermal choice retained

Closure selects thermal on ordinals5,34,51,57,58,73. Five of six increase
translation error and all six increase rotation error versus RGB on the same
interval. The complete set, rather than a selected failure example, is:

| Pair | RGB translation m | Thermal translation m | RGB rotation deg | Thermal rotation deg |
|---|---:|---:|---:|---:|
|5|0.072570138|0.167381983|0.239643363|0.847885312|
|34|0.018816829|0.234090260|0.071671408|0.797931987|
|51|0.008528542|0.019386098|0.060132692|0.141303550|
|57|0.036695276|0.028775806|0.132061307|0.203928029|
|58|0.024246924|0.032243056|0.108514957|0.722538864|
|73|0.072949795|0.141198739|0.084119577|0.746730328|

Variance selects thermal on5,34,49,50,51,73: equal camera counts do not mean equal
choices. Closure's slightly lower translation RMSE than variance does not recover
the fixed-RGB baseline; rotation is worse. No causal mechanism or bad-reference
explanation is inferred merely from these associations.

## Verification, cost and limits

Nine tests pass: affine/random scalar-versus-vector bilinear interpolation,
fractional/border/out-of-bounds/nonfinite masks, inverse translations, normalization,
undefined diagnostics, actual label-read blocking, ties and missing/unsupported
policy cases. Independent scalar/NumPy audit reconstructs all609336cached-corner
samples,529144valid closures, all original source times/grids/covariances,
217prediction slots and868policy choices; it verifies complete/matched population
losses against the original validated reports. Eight semantic mutations reject
wrong choices, RMSE, missing rows, fabricated processing stage, label-use flag,
unsupported eligibility, diagnostic median and valid count.

Dense reverse fields are identified by hash and their sampled corner values;
the independent checker does not rerun the neural network or validate dense-field
values against physical truth. The sampler itself is independently tested using
known fields. The first CPU audit was stopped to cache NPZ arrays once per pair
instead of repeatedly decompressing them per point. Numerical criteria, sampling,
inference and scores were unchanged; the initial log is retained.

Forward+reverse capture process times earlyRGB/thermal21.75/13.63s,
middle86.63/44.78s, day87.44/44.88s. Reverse-pair synchronized neural medians (including auxiliary stereo) are
386.67/184.14ms early,394.25/187.37ms middle and396.11/187.29ms day. Peak CUDA
allocated memory about11.775GiB RGB/5.731GiB thermal. This measured additional
cost is material; offline sensor-causal inputs do not prove an online decision can
meet its deadline. No latency optimization or real-time claim is made.

## Decision and next boundary

Do not adopt or tune the closure preference on these known scores. Raw learned
variance, saved residual/information preferences and now this additional
correspondence-consistency scalar have not consistently improved on the best
fixed camera under the examined conditions. This does not establish that learned
camera selection is impossible or that closure contains no useful information.

Before fitting a richer gate, the next bounded work should audit a recording-level
evaluation plan and reference/common-frame provenance using existing sources.
Keep reserved captures untouched. Selection-model capacity, independent physical
validation and prospective recording access remain separate decisions; repeated
heuristic adjustment on these same windows would not create new evidence.

## Reproduction and recovery

Host orchestration from `MAC-VO-main/`:

```bash
python3 -m Scripts.Setup.flow_cycle_experiment prepare --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/flow-cycle-quality/new-run
python3 -m Scripts.Setup.flow_cycle_experiment run --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/flow-cycle-quality/new-run
```

Prepare refuses an existing output directory. In the pinned container with ore-ms
at `/workspace`, save predictions before separately evaluating and validating:

```bash
python -m Scripts.Setup.predict_flow_cycle --root /workspace --output /workspace/results/flow-cycle-quality/new-run
python -m Scripts.Setup.evaluate_flow_cycle --root /workspace --output /workspace/results/flow-cycle-quality/new-run
python -m Scripts.Setup.validate_flow_cycle --root /workspace --output /workspace/results/flow-cycle-quality/new-run
```

Authoritative artifacts: `results/flow-cycle-quality/run-001/`, including every
sample/corner, guard/capture record, prediction, score, matched comparison and
JSON/CSV/PNG/SVG export. Box target:`ore_ms:ore-ms/backups/2026-10-30-exp-087`.
Incremental bundle requires24efa5c32; large image/checkpoint/runtime dependencies
remain EXP031/032/026/084–086 and their parents. The backup includes original
score/event/grid metadata used in this evaluation; image/checkpoint/Docker layers
and credentials are excluded. Every archive member is SHA256-verified.
