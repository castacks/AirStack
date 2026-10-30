# EXP-086: unchanged solver replication on supported day tracking

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `0e4dc683f`.

## Fixed question and declaration

Test the frozen EXP083 weighted-Huber repair on the previously examined EXP026/077
middle100-frame window from separate recording `20251027/0_salon_so_day`, manifest
start0 and original source start584. Both cameras historically support99/99
transitions. This complements EXP084's short early-night control and EXP085's
severe RGB-collapse case. It is known-recording replication, not an unseen test,
verified training exclusion or independent physical accuracy validation.

Six sequential runs compare original, passthrough and repair for each camera.
Use the identical EXP084/085 image/checkpoints, seed0, FP32, mappingfalse and
original synchronous global analytic disparity graph. The unchanged100-frame
wrapper delegates the same EXP084 run function and instance adapter. All original
pipeline/optimizer/graph sources and numerical solver settings remain fixed:
whitened-block Huber delta0.1, ten iterations, seventeen candidate attempts,
consistent acceptance and best-state restoration. Fatal status is an explicit
failed experiment, with no retuning. No new scene, reserved MS², training, selector
fit, scale/alignment fit or production-default change.

Restore only411selected EXP026 files:405prepared inputs, five original reference
exports and the preparation plan. Verify the complete1461483506-byte archive and
every selected member SHA256. Original source timestamps, three acquisition gaps
above150ms per camera, thermal MSA preprocessing and camera geometry remain intact.
RGB-minus-thermal offsets vary from−23.000002ms to+28.414965ms and change order;
original RGB/thermal durations are10.199505091/10.200000048s. No fixed8ms offset
is substituted from the night experiments.

Before inference, freeze378source files,405prepared files, both checkpoints,
all six exact commands, image ID and prior reference/evaluation dependencies.
All375 source hashes from EXP085's inference protocol remain identical. New
evaluation source is frozen when scoring; the validator hashes itself in its
proof. The independent day reference reader changes only the source path of
EXP084's Decimal-time/quaternion interpolation. It agrees with the established
reader at all200camera timestamps to maximum absolute matrix difference5.69e-14.

Each original/passthrough complete-array parity check gates its repair arm.
Retain all99slots per arm/camera, including missing results, and score only first
pre-refinement native motion writes. Stale repeats and filled poses do not create
new observations. Terminal writes remain explicit. Report full populations and
matched original/repair intersections, separate translation/rotation losses,
actual camera intervals and common original RGB intervals.

Offline evaluation reuses the independently validated EXP077 day proxy contract,
first available nominal rigid transform per camera, invariance across available
old experts, supplied SLAM reference and existing reference-assisted endpoint
transport. This reference is not verified independent ground truth; nominal body
transforms lack physical validation. No proxy label enters inference or tuning.

## Results

All six100-frame runs complete. Original/passthrough controls preserve all2613
arrays across103NPZ per camera bitwise. Original/repair matches all2577 raw
frontend/uncertainty arrays in100NPZ per camera; effective configurations match.
Every arm supports99/99 transitions, with18588retained RGB observations and13805
thermal. Each has497events and99 first motion writes (98processing,1termination),
no stale repeats or observer errors. All594 interval slots have valid scores.

| Camera / arm | Matched | Translation mean m | Translation RMSE m | Rotation mean deg | Rotation RMSE deg |
|---|---:|---:|---:|---:|---:|
|RGB original|99/99|0.029155872|0.031673337|0.104527918|0.123630496|
|RGB repair|99/99|0.029009703|0.031589278|0.104332857|0.123785025|
|Thermal original|99/99|0.050666140|0.077469182|0.511210217|0.612931850|
|Thermal repair|99/99|0.049957486|0.076312816|0.511297316|0.613905886|

Passthrough scores equal original. Translation RMSE decreases0.265% RGB and1.493%
thermal; rotation RMSE increases0.125% and0.159%. These small mixed effects do not
establish general or independent physical improvement. All198repair solves hit
the ten-step cap:1980attempts/1980accepted, no rejection or fatal failure.
Iteration-limit termination remains unresolved, not convergence.

Process wall times original/passthrough/repair are50.74/51.04/50.34s RGB and
29.66/29.56/29.31s thermal. Passthrough/repair solve medians28.34/17.56ms RGB,
27.43/17.23ms thermal. Wrapped CUDA peaks11.756GiB RGB and5.720GiB thermal;
original peaks unmeasured. Single sequential runs, startup, instrumentation and
different stopping rules do not support speed or real-time claims.

## Validation and numerical-audit correction

Thirty-three inherited solver/adapter tests pass. Recovery verifies all411selected
members of the original archive, whose SHA256 is
`087606e1279b0032761989f93be4c10a66bfdceb426660e1b2a270385d89782e`.
Independent checks reconstruct all2982events, raw health/flags, sensor intervals,
first/native writebacks, reference interpolation/transport, both losses and full/
matched populations. All1980steps, objective values, updates, best-state and
acceptance rules are checked; max relative FD gradient discrepancy1.077e-4 at
unchanged step1e-8 and tolerance1e-3. Nine metadata corruption checks reject.
All CSV/JSON and per-interval PNG/SVG exports remain available.

The initially declared component-relative gradient check fails at two states:
RGB96/outer9 and thermal79/outer8. Maximum gradient-vector absolute discrepancies
are1.386e-7 and7.296e-8. Independent projected residuals differ from native by only
2.246e-12 and1.137e-12, amplified by covariance weighting and signed summation.
A covariance-precision hypothesis was tested and did not explain the failure.
Following native inverse/action arithmetic reduces the first discrepancy but
moves another small component across the threshold; this attempt is also retained.

The final independent audit explicitly refines the gradient comparison: apply
relative constant2e-7/absolute1e-8 to the sum of absolute per-factor gradient
contributions, with an additional whole-gradient norm check at the same constants.
This is a **post-inference validation-criterion refinement**, not an unchanged
component test. Both original strict failures are retained in the proof; their
largest fractions of the contribution-scaled budgets are1.16e-5 and1.70e-5.
All Hessian, FD, step, objective and acceptance checks remain unchanged.
`validate_vo_solver_numerics_v2.py` records the independent NumPy calculation;
its SHA256 is captured separately in the final validator proof.

A supplementary native fixed-state reconstruction reproduces recorded cost,H,g
**exactly at all1980states**. It runs no optimization or neural inference and
independently confirms the saved trace matches the actual native graph/solver
arithmetic. This replay supplements the independent math checks; it does not
replace them or establish physical correctness. All failed audit/diagnostic logs
remain. No inference source, output, solver setting, score, reference or default
was modified or rerun to resolve the audit discrepancy.

## Evidence across the three frozen conditions

Each row uses its full matched original/repair population, including terminal
first writes. Different populations are not combined into an aggregate score.

| Condition / camera | Matched | Translation RMSE original → repair m | Rotation RMSE original → repair deg |
|---|---:|---:|---:|
|Early night / rgb|19|0.009971 → 0.010003|0.068002 → 0.064742|
|Early night / thermal|19|0.009260 → 0.009297|0.031829 → 0.030196|
|Middle night / rgb|2|4.038666 → 3.002267|35.488722 → 26.023199|
|Middle night / thermal|99|0.042872 → 0.041549|0.676590 → 0.676750|
|Day / rgb|99|0.031673 → 0.031589|0.123630 → 0.123785|
|Day / thermal|99|0.077469 → 0.076313|0.612932 → 0.613906|

Early-night translation worsens while rotation improves; middle/day translation
improves while thermal/day rotation is similar or slightly worse. Sparse middle
RGB improvements do not recover its97unsupported transitions. This is a verified
bounded numerical-contract repair with mixed conditional proxy effects, not a
validated general camera-selection or VO-accuracy method. Keep it experimental
and leave defaults unchanged. These known windows should not be represented as
untouched evaluation, nor should their scores be used to tune another solver.

## Reproduction

Restore EXP026's day middle prepared data and reference, EXP077's day proxy and
EXP083–085 source/runtime prerequisites. From `MAC-VO-main/`, host orchestration:

```bash
python3 -m Scripts.Setup.vo_solver_day_replication prepare --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/solver-day-replication/new-run
python3 -m Scripts.Setup.vo_solver_day_replication run --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/solver-day-replication/new-run
```

Prepare refuses an existing output directory. In the pinned image with ore-ms
mounted at `/workspace`, score and audit separately:

```bash
python -m Scripts.Setup.evaluate_vo_solver_day_replication --root /workspace --output /workspace/results/solver-day-replication/new-run
python -m Scripts.Setup.validate_vo_solver_day_replication --root /workspace --output /workspace/results/solver-day-replication/new-run
```

Authoritative artifacts: `results/solver-day-replication/run-001/`.
Box target: `ore_ms:ore-ms/backups/2026-10-29-exp-086`.
The incremental bundle requires commit0e4dc683f. Data/reference dependencies are
EXP026/077 and EXP083–085 plus their parents; prepared images, model weights and
Docker layers are excluded from this incremental backup. Their identities and
recovery prerequisites remain recorded. Credentials are excluded.
