# Arbiter input audit: CATS conventions and temporal health fields

**Follow-up, 2026-10-29:** [EXP-071](../../experiments/EXP-071-causal-arbiter-events.md) implements the read-only event recorder identified here. It preserves fresh GPU control outputs bitwise and exposes pre-refinement motion commits. The missing common-frame/reference and CATS convention questions below are not resolved by instrumentation.

2026-10-28, America/New_York. Base `7a99bd8af`, branch `ore_ms`.
This audit uses the already inspected CATS scene and recovered EXP033 controls.
No new neural inference, gate training, pose correction or reserved-capture
access was performed. Its purpose is to identify usable inputs and missing
contracts for the [temporal arbiter proposal](temporal-selector-proposal-2026-10-28.md).

## What is ready

Existing saved temporal runs provide per-pair uncertainty, selected/inbound/
retained keypoint counts and spatial coverage. Those are concrete inputs for
an estimator-health gate. They do not provide independently calibrated pose
quality labels or the complete online motion/result timing record.

The CATS mask inspection supports using 255 as left rectification support in
the inspected scene. Disparity anchor/direction remains ambiguous; selecting a
sign from photometric scores would overstate the evidence. A trusted reader or
explicit convention is still needed before a signed-reference benchmark.

## CATS mask audit

All original 23 scene-member hashes were checked before and after processing.
Both mask values and both disparity signs were retained. The table gives full
image counts, not the subsampled patch diagnostics.

| Rectification | Mask 0 pixels | Mask 0 with finite disparity | Mask 255 pixels | Mask 255 with finite disparity |
|---|---:|---:|---:|---:|
| Color | 148288 | 17411 | 1080512 | 116855 |
| Thermal | 41254 | 13665 | 265946 | 104736 |
| Cross | 26424 | 13709 | 280776 | 104043 |

Every mask-0 pixel is exactly zero in the corresponding left rectified image.
There are no all-zero mask-255 pixels in color/thermal, and two in the cross
image. This is strong evidence of a left-image support mask in this scene;
it is not an occlusion/physical-visibility guarantee or a full reference mask.
The right image can be padded at the same unshifted coordinate even where the
left mask is 255. Target support must be checked at the actual correspondence,
not by applying the left mask at the same coordinate to both images.

A future left-anchored disparity evaluation must therefore retain finite
reference and left support separately, with explicit correspondence bounds and
occlusion handling. If the labels prove right-anchored, an appropriate right
support definition is needed instead. Neither finite-reference checks alone
nor treating every zero disparity as invalid is justified by this audit.

## Signed-disparity hypothesis check

Before results, freeze five hypotheses: stored labels anchored at the left or
right image with opposite `x = anchor_x + d` or `anchor_x - d`, and a zero-shift
left control. Keep the same row, 9×9 patches, zero-mean normalized correlation,
and at most 1024 evenly spaced row-major finite-label indices per camera.
No offset, sign, threshold or pixel selection is fitted. Bounds failures and
constant-patch scores remain unavailable rather than being discarded silently.
Cross-modality intensity NCC is not used to infer its convention.

The table reports mask-255 subset medians, with available/selected NCC counts.
Different hypotheses have different patch availability; the paired comparison
is separately preserved in the report.

| Interpretation | Color median NCC (available / selected) | Thermal median NCC (available / selected) |
|---|---|---|
| Left anchor, opposite x+d | 0.1730 (868/899) | 0.3917 (903/927) |
| Left anchor, opposite x−d | 0.3116 (885/899) | 0.5501 (858/927) |
| Right anchor, opposite x+d | 0.3249 (861/899) | 0.5792 (901/927) |
| Right anchor, opposite x−d | 0.1624 (878/899) | 0.4301 (853/927) |
| Left anchor, zero displacement | 0.2542 (886/899) | 0.6247 (901/927) |

Left-minus and right-plus score similarly; the zero-displacement thermal
control scores higher at the median than either. Texture, smooth thermal
regions, occlusion and projection quality affect these diagnostics. Thus this
check does **not** identify a trusted anchor/direction, an additive rectification
offset or bad reference pixels. Do not adopt the highest-scoring hypothesis,
take absolute disparity, or fit a correction to this scene and call it validated.

The [official submission interface](https://bigdatavision.org/CAT/submit.html)
specifies left/right input ordering for algorithms, but does not by itself
resolve the stored reference array's sign/anchor convention. No applicable
intrinsics/projection/extrinsic file has been recovered from the inspected ZIP
indices. A minimal useful additional source would be the release's reader or
a documented correspondence example, mask semantics and relevant calibration.
Another scene upload is not needed to establish the present limitation.

### First-forward input contract, not executed

Keep native image shapes and original signed/NaN labels. A declared simple
thermal input baseline can use the fixed mapping
`u8 = floor(255 * uint16_value / 65535 + 0.5)`, replicate the grayscale channel
three times, and convert to float32 in [0,1]. Use the same mapping for both
cameras with no reference-driven or per-image scale fit. Preserve padding
zeros and masks. This is an explicit experimental conversion, not a reproduction
of the thermal checkpoint's unknown training recipe or a claim that it will
perform well. The display percentile mapping from intake is not an inference
recipe. A forward comparison remains deferred until the target conventions
and complete input/runtime protocol are established.

## Temporal recovery and available fields

The EXP033 archive SHA256 and all 389 archived member hashes pass; 369 scientific
members were restored without overwriting differing files. This reuses the
three original reporting controls, not newly acquired sequences. There are
217 recorded pair arrays across 220 camera frames; the two long controls
overlap in time and are not independent recordings.

All controls use `AllKeyframe` and synchronous optimization. The source and
recorded arrays agree on the following results:

| Saved control | Pairs | Original unsupported | Stored flags after refinement | Median selected / inbound / retained | Median occupied 3×3 cells |
|---|---:|---:|---:|---|---:|
| Short early RGB | 19 | 0 | 0 | 200 / 200 / 200 | 9 |
| Long middle RGB | 99 | 97 | 94 | 0 / 0 / 0 | 0 |
| Long middle thermal | 99 | 0 | 0 | 200 / 186 / 186 | 8 |

`Selected` means the network/keypoint selector's output before inbound/outlier
filtering, not the number of dense image candidates. Retained counts summed
over pairs exactly reproduce saved totals: 3797 short RGB, 54 long RGB and
17736 long thermal. These are observation counts, not independent measurements
or calibrated reliability scores. Source tracking flags can mark unsupported
or skipped frames; these controls use all-keyframe selection.

| Proposed input | Actual availability | Causal-use boundary |
|---|---|---|
| Acquisition interval | Both endpoint timestamps recovered; recorder start matches source row exactly. | Use integer nanoseconds and actual expert intervals. |
| Selected/inbound/retained counts | Per-pair recorder arrays. | Available after current pair processing, not before estimating the pair. |
| Spatial coverage | Retained source coordinates allow a fixed 3×3 occupancy summary. | A diagnostic feature, not a calibrated error label. |
| Raw uncertainty summaries | Per-pair fixed-grid temporal covariance uu/vv/uv; trace quantiles and finite/positive counts extracted. | These are temporal flow covariance, not pose covariance or raw learned stereo variance. |
| Original support flag | Set during pair processing when matches are insufficient; copied before refinement in EXP033. | Per-pair value can be recorded causally; whole-window unsupported count cannot be an earlier-frame input. |
| Refined trajectory | Present after `system.terminate()` and map refinement. | Do not use as if it were the online unfilled motion estimate. |
| Optimizer residual/convergence history | Not persisted by these controls. | Missing, not zero; execution PASS is not convergence telemetry. |
| Pose covariance | Not present as a calibrated pose-level output. | Observation 3×3 covariance is not an SE(3) pose covariance. |
| Independent pose-loss labels / body transport | Not established here. | Existing GNSS magnitude checks and camera-local SLAM proxies do not supply verified body-pose error truth. |

For the long controls, the paired manifest rows match, but none of the 99 RGB
and thermal intervals has exactly equal endpoints. RGB timestamps are later
by 6.509–6.902 ms, with median 6.832 ms at both endpoints. This is an observed
acquisition offset, not a fitted latency correction or proof of its physical
cause. A real arbiter needs a declared asynchronous/timing policy and common
frame convention, rather than treating equal row numbers as identical motion
intervals. Exact equality is not inherently required of a valid asynchronous
system; the policy and its approximation error must be evaluated.

## Causality and telemetry contract for the next implementation

The source calls `start_optimize` for supported pairs, while `write_map` commits
the result at the next pair or termination. Even in synchronous mode, the time
an optimized result is computed differs from when the existing graph is updated.
At termination the map processor can fill motions using neighboring supported
frames and clears some boundary flags. This is why an after-refinement pose or
support flag must not silently become an online gate input.

The next bounded implementation should be a read-only event recorder before
refinement. Each event should include original input interval, result frame and
parent indices, result-availability stage/time, original support state,
selected/inbound/retained counts, spatial coverage, raw uncertainty summaries
and the actual unfilled camera-frame motion result if one exists. Preserve
missing/unsupported motion explicitly. Optimizer loss/iteration/termination
telemetry should be recorded directly if requested, never inferred from PASS.
Raw camera estimates remain raw camera estimates until verified body transforms
and an asynchronous comparison policy are available.

Before new inference, test that observing these events leaves outputs unchanged
and rejects future/end-of-clip fields from causal features. Audit simulated
sequences with unsupported and delayed results, and then verify one bounded
replay against the same original controls when the runtime is restored. This
does not require a gate fit or consuming reserved MS² captures.

The large RGB/thermal support contrast makes health inputs worth testing; it
does not prove the thermal pose is more accurate or that a learned arbiter is
better than a simple availability rule. Training a model to predict a support
flag supplied as its own input would be circular. Any accuracy claim needs a
distinct target, suitable reference and recording-level evaluation.

## Validation and artifacts

Independent validation reconstructs patches with explicit four-neighbor
bilinear weights and scalar NCC, checks all saved scores/mask groups and paired
comparisons, reconstructs all temporal counts/covariance quantiles/spatial
occupancies, verifies integer timestamps and source hashes, and checks report
summaries. Synthetic signed/zero shifts, fractional interpolation, patch bounds
and constant-patch cases pass. These checks validate diagnostic arithmetic and
preservation, not authoritative dataset semantics or predictive performance.

Scripts in `MAC-VO-main/Scripts/Setup`: `cats_convention_audit.py`,
`temporal_arbiter_field_audit.py`, `validate_arbiter_input_audits.py`.
Outputs: `results/cats-conventions/run-001/` and
`results/temporal-arbiter-fields/run-002/`. The temporal run002 only adds field
definitions and source timing provenance; its extracted report is unchanged
from run001. Sources are CATS intake and EXP033 private recovery. Recovery
target: `backups/2026-10-28-arbiter-input-audit`.
