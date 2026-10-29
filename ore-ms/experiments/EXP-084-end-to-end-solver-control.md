# EXP-084: bounded end-to-end solver control

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `65cadf035`.

Six fresh GPU runs on the existing early-night 20-frame interval compare the
unchanged original pipeline, an instance-only passthrough adapter, and the frozen
EXP083 weighted-Huber repair, separately for RGB and thermal. The adapter
preserves every original array exactly. Repair slightly worsens translation and
improves rotation against the qualified SLAM proxy. Every repair solve reaches
its iteration cap. This supports further investigation, not a default change or
an independent physical accuracy claim.

## Frozen scope and execution

The pre-inference protocol freezes 371 source files, 85 prepared input files,
both original checkpoint hashes, evaluation dependencies, the image ID and all
six commands. Use the first20 rows, start0 of the previously examined EXP031
prepared early-night interval, FP32, seed0, mappingfalse and the original global
analytic disparity graph. No new scene, training, selection fit or reserved
MS² capture. Original production optimizer, caller, v3 runner and FlowFormer
sources remain unchanged. Historical GPU bitwise equivalence is not assumed.

`bounded_solver_adapter.py` attaches to one optimizer instance, and
`run_td3_solver_arm.py` temporarily wraps the original factory, delegating the
unchanged v3 execution path. Passthrough calls the original method exactly once.
Both instance and factory are restored on success or exception. Repair uses the
actual native graph and frozen EXP083 settings: dimensionless whitened-block
Huber delta0.1, ten iterations, seventeen candidate attempts per iteration,
consistent objective acceptance and best-state restoration. Fatal statuses
abort the execution explicitly; iteration-cap/small-step returns remain
unresolved and are never renamed convergence. No extra result polling.

The pinned Blackwell image `macvo:solver-vo` is
`sha256:e1e72a2dc43630ffe47cf45c29278bd879d36b9bfb9fd6d621c1e88f88455b82`.
It uses Torch2.7.1+cu128, torchvision0.22.1+cu128, PyPose0.9.5,
NumPy2.2.6, SciPy1.17.1 and OpenCV4.10.0.84. Finite CUDA execution and all
six complete runs were verified on RTX PRO5000 Blackwell. EXP031 recovery
verified all1087 archive members, restoring only90 selected files. The official
MAC-SLAM shared Box release was recovered with authenticated shared-link context;
its318804129-byte ZIP SHA256 is
`77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c`.
Only the two exact frontend checkpoints were extracted. Credentials and signed
download URLs are excluded from artifacts.

## Preservation and complete populations

For each camera, original/passthrough matches all533 arrays in23NPZ files
bitwise. Original/repair matches all497 frontend/uncertainty arrays in20NPZ
files bitwise. All six runs retain20frames and support19/19 transitions without
interpolation; RGB has3797 retained observations and thermal3614 in every arm.
Repair changes estimates downstream of the frozen frontend, not learned outputs.

Each run has97 events. All19 first pre-refinement motion writes are scored:
18 occur during pair processing and one during termination. There are114 scored
arm/camera intervals, including terminal results, and19 matched original/repair
intervals for each camera. Terminal output is not relabeled pre-deadline output.
Actual camera intervals, the approximately8ms sensor offset and own-interval
losses remain explicit. Sequential replay clocks do not establish simultaneous
availability or real-time operation. Final post-refinement maps remain execution
artifacts, not online estimates.

## Qualified proxy results

Use the unchanged EXP075 nominal body transform and reference-assisted transport
onto original RGB intervals. The supplied reference is SLAM output, not verified
independent ground truth. Physical extrinsic validity and reference independence
remain unresolved. No scale, alignment, thresholds or solver parameters are fit.
Mean/RMSE below use all19 matched common intervals, including the terminal write.

| Camera / arm | Translation mean m | Translation RMSE m | Rotation mean deg | Rotation RMSE deg |
|---|---:|---:|---:|---:|
|RGB original|0.008615368|0.009971226|0.061056378|0.068001533|
|RGB repair|0.008704116|0.010003348|0.057534804|0.064742104|
|Thermal original|0.008033690|0.009260253|0.024881844|0.031828841|
|Thermal repair|0.008068766|0.009296911|0.023883333|0.030195543|

Passthrough scores equal original scores. Translation RMSE increases about0.32%
RGB and0.40% thermal; rotation RMSE decreases about4.79% and5.13%. These small,
mixed differences on a reused short interval are not evidence of a general VO
improvement. Own-camera and common-interval scores, every row, all paired
per-interval differences, CSV and PNG/SVG plots are retained.

All38 repair solves report `MAX_ITERATIONS`, with380 accepted steps and no
rejections or fatal execution failures. Independent checks verify every candidate,
state restoration/retraction, objective, system, step and acceptance rule. This
establishes bounded numerical behavior, not convergence or covariance calibration.
The uninstrumented original arm has no reported native termination status.
Passthrough explicitly reports `ORIGINAL_RETURNED_UNQUALIFIED`.

Descriptive one-run adapter medians: passthrough/repair RGB28.44/18.23ms,
thermal29.63/21.54ms per solve. Process wall times are16.44/16.50/18.51s RGB
and12.99/12.19/12.92s thermal for original/passthrough/repair. Wrapped-arm peak
CUDA allocated memory is11.756GiB RGB and5.720GiB thermal; original memory is
unmeasured. Different iteration/stopping rules, instrumentation, process overhead
and single sequential runs prevent a speed or real-time claim.

## Validation and disclosed implementation corrections

Thirty-three focused tests pass: eighteen existing solver tests and fifteen
adapter/factory delegation, restoration, failure and finite-pose tests.
An independent NumPy checker recomputes raw-array preservation, all582 event
records, integer intervals, quaternion/reference interpolation, frame conversion,
reference-assisted transport, both losses and matched summaries. It independently
checks all380 repair attempts/accepted steps and finite-difference gradients;
maximum relative gradient discrepancy is8.17e-5 at fixed step1e-8.
Nine isolated semantic mutations are rejected: loss, interval, nominal transform,
reference bridge, missing row, matched RMSE, invented convergence, physical-truth
claim and array-parity count. Mutations operate on copied metadata and read-only
links to original run data; inference artifacts are untouched.

The first evaluation implementation incorrectly expected a transform in every
legacy deadline expert row; the last such expert is absent. Evaluation now uses
the same first validated rigid transform for all19 intervals and independently
checks invariance across available legacy rows. The failed evaluation protocol
and log are retained. No inference or fixed evaluation population was changed.

The first independent solver audit accumulated repeated SE3 rounding before
checking tiny reduction ratios. The final checker independently validates each
serialized state/retraction/cost, then computes acceptance ratios from reported
costs/reductions, avoiding division of independently rounded tiny differences.
H/g comparisons use the EXP083 absolute floor1e-8: a near-zero gradient component
differed by1.38e-9. Failed audit logs remain. No solver method, inference source,
step budget, objective, output or result threshold was changed after inference.

## Reproduction and recovery

After restoring prerequisites, build from `MAC-VO-main/`:

```bash
docker build -f Docker/Dockerfile.solver-vo -t macvo:solver-vo .
```

Run host orchestration from `MAC-VO-main/` with the repository's `ore-ms` root:

```bash
python3 -m Scripts.Setup.vo_solver_control prepare --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/end-to-end-solver-control/new-run
python3 -m Scripts.Setup.vo_solver_control run --root /absolute/path/ore-ms --output /absolute/path/ore-ms/results/end-to-end-solver-control/new-run
```

Prepare refuses an existing output directory and records exact GPU commands.
The explicit base commit in the protocol describes this declaration, not a dynamic
claim about future checkouts. From a container with ore-ms at `/workspace`, run:

```bash
python -m Scripts.Setup.evaluate_vo_solver_control --root /workspace --output /workspace/results/end-to-end-solver-control/new-run
python -m Scripts.Setup.validate_vo_solver_control --root /workspace --output /workspace/results/end-to-end-solver-control/new-run
```

Authoritative artifacts: `results/end-to-end-solver-control/run-001/`.
Box recovery target: `ore_ms:ore-ms/backups/2026-10-29-exp-084`.
The incremental source bundle requires commit65cadf035. Prior data/reference
artifacts require EXP031/075/079/080/082/083; exact checkpoint hashes remain
recorded in the protocol. Large images, checkpoints and Docker layers are not
included in this incremental backup. Its manifest verifies every archived member.

No default switch is supported. A separately declared unchanged-parameter
replication on an already examined harder interval is the next bounded check;
this experiment does not authorize tuning against its proxy scores.
