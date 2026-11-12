# MS² depth transfer: default ViViD checkpoint does not advance

12 November 2026. **CLOSE_THIS_CHECKPOINT_TRANSFER_CONFIGURATION_NO_RETUNING.**
The fixed reproduced monocular pipeline loses to freshly evaluated original
MACT on all three recording-wise depth-structure gates. All60 frame pairs
complete with unchanged candidate/common support; no model is promoted.

## What was fixed before inference

[Execution plan](ms2-depth-transfer-plan-2026-11-12.md), freeze `9f348f9d9`.
Original outdoor ThermalSfMLearner-MS depth checkpoint epoch139 and original
MACT checkpoint; first20 original thermal stereo groups each from existing
day/night/rain development prefixes. All180 raw left/right/depth files match
the prior packet, whose479 members and archive hash were verified. No fitted
stereo adapter weights, new data, optimizer or parameter search is involved.

The official MS² source `0bb0fc1a` demo applies its FLIR A65C default conversion
to native thermal codes. Both conversion function ASTs exactly match the ViViD
author source and SupDepth4Thermal utilities `f6725678`;18roundtrip checks pass.
This establishes nominal source convention compatibility, not physical
calibration or common operating conditions. The [working references](references/README.md)
record these source versions and reading limits.

The unchanged ViViD outdoor clip saturates large fractions of native codes
before resize: median99.7571%day,93.5474%night,99.6664%rain. These input-only
statistics were documented before inference. We did not expand the range or
drop affected frames. Raw input passes through the original author loader at
256×320; inverse depth is resampled to640×256 with fixed1e-6epsilons. The
aspect change and heavy clipping limit this exact default pipeline's transfer.
Neither is isolated as a cause by these results.

Fresh MACT uses original weights, source code, native depth formula and named
percentile/CLAHE/bilateral input replay. Both models run in exact runtime
`fd7be8a3` with strict FP32 and TF32 explicitly disabled. This is a fresh
controlled comparison, not reuse of historical scores or bitwise reuse of
historical depth maps. A posthoc first-frame snapshot check finds non-bitwise
differences from earlier native captures, whose runner did not explicitly set
these precision flags. That spot check is retained and does not change the
comparison, thresholds or gates; no inference was repeated. Physical or causal
explanations for those historical numerical differences were not tested.

## Matched-support depth-structure results

GT support: finite supplied thermal-left depth in(0.001,80)m, intersected with
finite positive predictions from both models. Both independently receive one
median scale on the identical common pixels per image. Scores are equal-frame
mean AbsRel and mean per-frame RMSE; no pixel pooling, stereo visibility mask,
confidence selection or missing-frame substitution is added. Candidate must
preserve all baseline-valid supported pixels.

| Recording | Frames | Common pixels, total | Scaled MACT AbsRel | Scaled mono AbsRel | Scaled MACT RMSE,m | Scaled mono RMSE,m | Gate |
|---|---:|---:|---:|---:|---:|---:|---|
| day | 20 | 154433 | 0.135344 | 0.516412 | 9.197944 | 12.754412 | FAIL |
| night | 20 | 187014 | 0.204707 | 0.805141 | 8.485951 | 15.110785 | FAIL |
| rain | 20 | 122871 | 0.190908 | 0.345083 | 6.443989 | 8.610129 | FAIL |

All20frames in each recording score. Candidate support does not regress; losses
are in both error endpoints, not omitted support. Neither pooling nor a
favorable recording can rescue the per-record failures.

For separate context, original unscaled MACT AbsRel/RMSE on the same support is
day0.163913/10.361459, night0.202921/9.313626, rain0.182824/6.428038. Those
metric values are not the primary comparator against reference-scaled monocular
predictions. No unscaled monocular accuracy or calibrated scale is asserted.

## Verification and decision

Six contract tests pass. All60 two-model prediction packets lock before scoring.
The independent audit reconstructs width interpolation with NumPy instead of
OpenCV resize, median scales, support counts, scalar errors, equal-frame metrics
and all recording/overall decisions:738checks pass at the frozen1e-9tolerance.
All729 source/input/model/plan pins remain unchanged. Both original checkpoints
and production defaults are unchanged; no pose optimizer or VO run occurred.

Close this fixed ViViD-checkpoint/original-input MS² transfer configuration
without a temperature-window, resolution or checkpoint sweep. This is not
evidence that thermal depth learning fails: it tests an already-known strongly
clipped default source pipeline on different exposed recordings. An alternative
requires a separately justified input/model contract, not a rescue selected
from these scores. No automatic metric-scale fit or VO integration follows.

These are supplied projected reference depths with nominal source geometry and
unknown upstream training overlap. There is no fresh-record generalization,
physical temperature-calibration or full-motion claim. Artifacts:
`results/ms2-depth-transfer/run-001/` includes input-gate/range evidence, all
prediction packets, global lock, per-frame scales/metrics, gates, independent
audit, and the separately labeled historical first-frame spot check.
