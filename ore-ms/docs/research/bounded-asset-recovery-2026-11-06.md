# MAC/MACT recovery: both experts and uncertainty outputs operational

2026-11-06 America/New_York. Bounded recovery authorized after the [research stop/go decision](research-decision-2026-11-05.md); this is an engineering milestone, not another accuracy experiment. Native command/source/asset lock committed at **10f2f8868** before the smoke captures.

**Recovery and the development smoke passed.** Both original checkpoints and the exact historical compiler runtime are restored. Both experts processed the existing first 20night source groups, with all 19 transition pairs supported, positive temporal covariance summaries, finite native motion receipts and no observer errors. Missing local assets are no longer the execution blocker.

| Camera | Frames | Finite poses | Positive covariance receipts | Native motion receipts | Retained observations | Native wall seconds |
|---|---|---|---|---|---|---|
| rgb | 20 | 20 | 19 | 19 | 3799 | 20.57 |
| thermal | 20 | 20 | 19 | 19 | 1366 | 15.40 |


There were 40 native frame calls total, one attempt per camera. Original source nanosecond timestamps are preserved exactly, all pose quaternions are finite/unit, and the saved 19 uncertainty NPZs per camera reproduce the corresponding recorded grid-trace quantiles. These are camera-local outputs and execution receipts; no pose-reference value, accuracy score, selector benefit or physical common-time qualification was computed. The two reported reserve candidates remain unassigned and unopened.

## What was recovered

| Asset | Exact identity |
|---|---|
| RGB MAC-VO checkpoint | bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a |
| MACT thermal checkpoint | eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a |
| Historical native compiler runtime | sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09 |

Both checkpoints came from the existing EXP112 packet after its archive and all 2,782 regular members were verified. The EXP106 base image archive and EXP112 compiler delta/manifest/helper also match their recorded hashes. The verified base/delta merge loaded the exact expected image ID. No new Docker build, package update, model migration change, estimator patch or threshold adjustment was introduced by this recovery.

Actual restored environment: Torch 2.7.1+cu128, NumPy 2.2.6, OpenCV 5.0.0, NVIDIA RTX PRO 5000 Blackwell. CUDA matrix preflight passed. Historical driver-initialization diagnostic text remains in the logs alongside successful CUDA/native exit codes; it was not suppressed or treated as model failure. The exact recovered image, rather than the older EXP044 smoke image's package versions, defines this attempt.

The existing MS2 adapter prepared the unchanged first 20groups of `_2021-08-13-21-18-04` from the original raw RGB/uint16 thermal inputs, with all 80 image hashes exact. It used the current unchanged source-defined thermal recipe, seed 0, serial optimizer and opt-in uncertainty/events/root observation. The lock binds 542 source/config/asset files. Existing `Scripts.Setup.run_ms2_execution` ran without source edits. Native runtimes were about 21 seconds RGB and 15 seconds thermal; asset recovery took minutes rather than the four-hour cap.

## What changes and what does not

The earlier missing-local-files inventory is a historical observation and is now superseded for asset availability by this result. It was not evidence of permanent unavailability. The [single decisive-test contract](decisive-uncertainty-test-2026-11-05.md) remains unexecuted: camera-local smoke success does not supply its source-specific common-interval/common-frame proof or an assigned, verified fresh evaluation partition.

**Next gate:** check compatibility of the existing source-specific nominal target/conversion and frozen scoring contract on development inputs, then verify the fresh-recording partition and bind actual execution artifacts before any reserve payload. Keep the unchanged uncertainty candidate and numerical stopping rules. No additional patch-gap/bias diagnostics, predictor fitting, calibration guesses or new adapter research follows from this pass. If the unchanged target cannot be supported, stop at that concrete incompatibility.

Patch-gap confidence development remains closed and thermal-bias micro-diagnostics remain parked. No adaptive, heldout-accuracy, calibrated-confidence, physical, online or fusion benefit is claimed.

Artifacts: results/bounded-asset-recovery/run-001, including original input/source/asset pins, runtime/checkpoint recovery reports, GPU preflight, both full native captures, raw covariance/graph/pose/motion receipts and their checks, original commands and disposition. Recovery: ore_ms:ore-ms/backups/2026-11-06-bounded-asset-recovery. Existing EXP106 base-runtime and EXP112 compiler/model backup packets remain exact prerequisites rather than duplicate multi-gigabyte copies. No credentials or reserved payload are included.
