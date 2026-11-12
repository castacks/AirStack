# Raw thermal refinement: completed native comparison

12 November 2026, America/New_York. **CLOSE_THIS_CONFIGURATION_NO_RETUNING.**
The fixed raw-uint16 local refinement fails the predeclared native accuracy gate
on day, night and rain. It preserves thermal coverage but worsens translation
mean and RMSE on all three. No threshold, patch-size, frame or checkpoint search
follows. This closes this configuration, not thermal tracking in general.

## Frozen execution and controls

Exact compiler runtime `sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`
was restored from verified EXP106 base and EXP112 delta. All 2,782 checkpoint
packet members and archive/helper hashes pass; original RGB/thermal models are
unchanged. No image rebuild or dependency update occurred. All 26 native
hook/tracker tests pass, including the previously skipped class-loading check.

Parity freeze `95222529b` precedes six fresh 100-frame original/no-op captures.
All three pass bitwise: 2,613 exported arrays in 103 NPZ files per recording,
including poses, clocks, flags and uncertainty exports (7,839 arrays total).

Accuracy freeze `ef8359ee7` precedes refined/RGB captures; independent auditor
`f2c76642d` precedes scoring. Twelve native captures complete 1,200 frame
executions. All four arms on all three records are globally prediction-locked
before reference values are scored. This uses all 99 adjacent intervals of the
same exposed first100-group day/night/rain prefixes as the previous longer
study, rather than choosing a favorable extension or pooling recordings.

Refinement is the fixed [paper-informed mechanism](radiometric-tracking-preflight-2026-11-12.md)
and [sparse hook](radiometric-native-hook-2026-11-12.md). Original MAC flow
initializes raw-uint16 mean-normalized 15×15 patches, with ±2-pixel correction
bounds and 30 evaluations. No TartanMatch or NEC/PNEC enters. Rejections retain
original endpoints; covariance is the unchanged original control, not calibrated
uncertainty for refined observations. Native target-depth lookup and geometric
covariance propagation see the refined endpoint, so their consequences belong
to the total intervention effect. The frozen model/selector/initializer remains.

## Accuracy

Each row uses the identical original/no-op/refined/RGB intersection for that
recording. Mean / RMSE values are local motion errors, not trajectory ATE.

| Recording | Common intervals | Arm | Translation mean / RMSE, m | Rotation mean / RMSE, deg |
|---|---:|---|---:|---:|
| day | 99 | original | 0.112578 / 0.129597 | 0.104860 / 0.122611 |
| day | 99 | refined | 0.116139 / 0.134874 | 0.098924 / 0.116147 |
| day | 99 | rgb | 0.046390 / 0.052889 | 0.047218 / 0.051366 |
| night | 97 | original | 0.060616 / 0.092652 | 0.063685 / 0.087763 |
| night | 97 | refined | 0.061554 / 0.095373 | 0.066583 / 0.093642 |
| night | 97 | rgb | 0.022602 / 0.036333 | 0.035375 / 0.052741 |
| rain | 96 | original | 0.730025 / 0.837217 | 0.091694 / 0.117980 |
| rain | 96 | refined | 0.760323 / 0.870825 | 0.098158 / 0.125742 |
| rain | 96 | rgb | 0.264989 / 0.496752 | 0.036382 / 0.041918 |

Thermal own eligible coverage is unchanged at 99/99 day, 97/99 night and
96/99 rain; RGB retains 99/99 on each. Day rotation improves, while both night
and rain rotation worsen. Translation mean/RMSE fail on every recording. RGB
remains lower on all four mean/RMSE endpoints in each common population.

The gate requires lower thermal translation mean AND RMSE, no higher rotation
mean/RMSE, nonempty common support, and preservation of thermal eligible
interval IDs separately per recording. All three fail. No aggregate result,
tail statistic or accepted-patch count replaces that rule.

All 297 pair callbacks complete. Of 32,171 point attempts, 31,944 accept a local
fit and 227 retain their original endpoint after rejection. These counts do not
establish correctness of real correspondences.

## Audit and limits

The independent saved-output audit passes 1,332 motion-binding/metric checks.
It reconstructs native quaternion matrices, common-frame motion via independent
SciPy matrix log/exp, eligible interval sets, error statistics and all decisions.
It shares the qualified extrinsic lookup and producer reference deltas; it does
not independently certify physical calibration, clock semantics or reference
truth. The audit reruns no network or optimizer. Every frozen hash remains
unchanged. No scientific attempt was retried.

These are already exposed development recordings, supplied reference poses and
nominal frame/interval assumptions. No held-out generalization, runtime benefit,
calibrated covariance or directly measured real-flow accuracy is claimed. The
prototype is not the authors' Self-TIO or TP-TIO implementation. Its negative
result does not reject their complete systems.

## Decision

Close this fixed original-flow/raw-patch/original-covariance configuration without
retuning. Preserve the day rotation benefit together with the translation and
night/rain regressions. Do not automatically add a covariance variant, tracker
threshold or another diagnostic chain. Any future intervention needs a distinct
paper-supported hypothesis and recording-wise evidence. Original models and
production defaults remain unchanged.

Code: `run_radiometric_native.py`, `radiometric_accuracy.py`,
`audit_radiometric_accuracy.py` under `tools/`. Artifacts:
`results/radiometric-native/run-001/` contains all twelve captures, logs, raw
refinement receipts, three parity reports, runtime recovery, both freezes via
Git, global prediction lock, per-interval scores, summary and independent audit.
