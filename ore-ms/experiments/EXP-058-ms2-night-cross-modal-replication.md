# EXP-058: Third-recording frozen night camera-selection replication

Date: 2026-10-27 America/New_York. Branch `ore_ms`; starting commit `fc1434c2d`.

**Finding: the frozen propagated-uncertainty rule is worse than always RGB and equal-random selection in all 20 night frames, under both losses.** Median mean 3D-distance risk is 25.3% higher than always RGB and12.1% higher than equal random. Yet it improves over per-image camera-scale control by 2.2%. Thus a gain against the camera-scale comparator alone is insufficient evidence of useful cross-modal selection. This is a supplied-reference candidate-support benchmark, not independent physical truth or a causal night-condition effect.

## Frozen third recording

Before inspecting night winner results, select the existing EXP043/044 opening IDs 000000–000019 of `_2021-08-13-21-18-04`. This is a third distinct recording, previously studied within-camera and near-static in the earlier pose-reference audit. It is not verified checkpoint-training-held-out data or a new untouched statistical test set. Do not pool adjacent frames, pixels, repeated loss/anchor views or three recordings into independent trials.

Use unchanged EXP057 runner and EXP056 geometry/metric helpers. The same normalized angular radius1/fxRGB, unique reciprocal nearest labels, both sparse frontmost zbuffers, original single-scan reference, no primary depth threshold, exact tie handling, midpoint/RGB/thermal anchors, validity policy and all comparator rules apply. Original pixels, flow/covariance arrays, timestamps, checkpoint outputs, intrinsics and extrinsics are preserved. No new inference, calibration/latency fit, training, threshold tuning or fusion change.

Three fixed support overlays (IDs 000000/000009/000019) and ID000009 patches were inspected before night winner scoring. Road/curb, vegetation and building regions overlap plausibly at a coarse level. Dark RGB, illumination effects and textureless regions remain visible; no exact cross-modal identity, dense occlusion or dynamic-object mask is inferred. The inspection decision is saved with support outputs.

## Recovery and byte-preserving layout adapter

The original night payloads use different folders from day/rain. Recover only missing scientific files without overwriting differing content, after verifying all archive/member hashes:

| Snapshot | Archive SHA256 | Verified members | Selected/restored scientific files |
|---|---|---:|---:|
| EXP043 | `5658f69f6b2b186453003728a5a56e78b65cc5b523797c2ec204100f0e657fd9` |277|244|
| EXP044-complete | `1c62a44b23eb554485a07464d5ac4a9e70771b8cc99dbf613f785be4e47e81cd` |109|85|
| EXP045 | `95a74ab5bcd2613c11cac6730ad9ffe87c2c9717d7f2418a063e500691ccea7f` |32|23|

EXP043 supplies original night pixels/depth and source-ID/time manifest; EXP044-complete supplies saved RGB/thermal frontend arrays; EXP045 supplies the **night's own** decoded/raw calibration and full timestamp metadata. Calibration from another recording was not substituted.

`stage_ms2_night_cross_modal.py` creates 124 byte-identical files under a new input directory: metadata, left images, single-scan depth maps, left timestamps and40 saved stereo fields. It records every destination's original archive/member/hash and pinned original-manifest hashes in `layout-provenance.json`. The original source manifest is copied without changing its embedded original paths; those retain provenance, while the runner uses explicit staged locations. The composite integrity manifest is a layout map, not a newly supplied original-data archive. Source/copy bytes, all 124 original member hashes and three original-manifest hashes independently match; both historical scientific helpers remain unchanged.

This adapter solves a storage-layout issue, not any physical timing/reference question. No checkpoints, full transport archives, GPU runtime or model execution are needed for replay from saved fields. Original source directories and prior rain/day outputs remain unchanged.

## Support and availability

Night retains85,170 unique candidate pairs; all 85,170 have both predictions valid, with zero one/neither-valid cases. Per-frame counts range 4182–4354 (median 4251). Median per-image median 3D reference separation is 0.01282 m, with range 0.01246–0.01318 m. Recorded absolute left-camera time separation is 0.124 /4.497 /60.564 ms (min/median/max); it is not an exposure-delay measurement or an acceptance gate.

A small disagreement between two sparse reference products does not prove exact shared points or independent truth: both share acquisition/calibration/reference processing. Static rig transforms remain uncompensated for acquisition motion. These limits do not veto the scoped provided-pair benchmark; every selected frame remains in the results.

## Across the three frozen windows

Entries are the median over20 **per-image paired** learned-selector/comparator risk ratios, followed by the number below1. Below1 favors learned selection. Mean3D-distance risk is mean Euclidean reconstruction disagreement in metres; squared 3D risk is in m². The supplied candidate reference is the same for both modalities within each comparison, and all policies use identical both-valid support.

| Recording | Loss | Learned / camera scale (below1) | Learned / always RGB (below1) | Learned / equal random (below1) |
|---|---|---:|---:|---:|
| Rain | Mean 3D distance | 1.002 (9/20) | 0.190 (20/20) | 0.306 (20/20) |
| Rain | Squared 3D error | 0.988 (11/20) | 0.003 (20/20) | 0.006 (20/20) |
| Day | Mean 3D distance | 0.904 (19/20) | 1.157 (0/20) | 0.867 (20/20) |
| Day | Squared 3D error | 0.744 (20/20) | 1.318 (7/20) | 0.799 (20/20) |
| Night | Mean 3D distance | 0.978 (20/20) | 1.253 (0/20) | 1.121 (0/20) |
| Night | Squared 3D error | 0.979 (20/20) | 1.653 (0/20) | 1.285 (0/20) |

The night learned rule beats camera-scale and equal-angular geometry in all 20 frames by a small amount, but loses to always RGB and random in every frame. Always thermal also beats it in 17/20 mean-distance cases and19/20 squared cases. A selector can be worse than either fixed camera when its mistakes systematically favor the higher-error measurement; confidence ranking **within** a camera does not guarantee comparable scores **between** cameras.

Day remains a separate counterexample: it has a larger gain over camera scale but loses to always RGB in all 20 mean-distance cases. Rain has a larger gain over fixed-camera/random policies but no consistent advantage over camera-scale control. No one favorable comparator or recording establishes a universal cross-modal selector. The differences do not isolate illumination, weather, training membership or calibration as the cause.

![Three-recording frozen controls](../results/ms2-night-cross-modal-replication/three-recording-controls-final.png)

## All original night comparators

| Loss | Comparator | Learned/comparator min / median / max | Learned lower risk |
|---|---|---:|---:|
| Mean 3D distance | Always RGB | 1.1111 / 1.2528 / 1.3211 | 0/20 |
| Mean 3D distance | Always thermal | 0.9758 / 1.0161 / 1.0653 | 3/20 |
| Mean 3D distance | Equal random expectation | 1.0391 / 1.1210 / 1.1795 | 0/20 |
| Mean 3D distance | Per-image camera median angular variance | 0.9660 / 0.9779 / 0.9979 | 20/20 |
| Mean 3D distance | Equal angular geometry | 0.9657 / 0.9770 / 0.9980 | 20/20 |
| Mean 3D distance | Equal native-pixel geometry | 1.0740 / 1.1004 / 1.1282 | 0/20 |
| Mean 3D distance | Unpropagated learned angular variance | 0.9511 / 1.0215 / 1.0747 | 4/20 |
| Squared 3D error | Always RGB | 1.4081 / 1.6529 / 1.7634 | 0/20 |
| Squared 3D error | Always thermal | 0.9944 / 1.0541 / 1.1173 | 1/20 |
| Squared 3D error | Equal random expectation | 1.1656 / 1.2846 / 1.3528 | 0/20 |
| Squared 3D error | Per-image camera median angular variance | 0.9651 / 0.9789 / 0.9900 | 20/20 |
| Squared 3D error | Equal angular geometry | 0.9645 / 0.9764 / 0.9897 | 20/20 |
| Squared 3D error | Equal native-pixel geometry | 1.0972 / 1.1219 / 1.1665 | 0/20 |
| Squared 3D error | Unpropagated learned angular variance | 0.9839 / 1.0995 / 1.1620 | 1/20 |

On night, the learned rule chooses RGB for a median 36.4% of pairs, versus5.7% under camera-scale control. Its median lower-error choice fraction is 43.8%, compared with 56.5% for always RGB. These fractions are descriptive on selected support, not calibrated probabilities or independent accuracy trials. The fixed native-pixel geometry comparator beats learned selection in every night frame under both losses; keeping only equal-angular geometry would conceal this counterexample.

The rank-one metric score still assumes a fixed left ray/calibration and first-order inverse-disparity propagation. It is not a calibrated 3D covariance or a complete deployment policy. Positive tiny disparities remain untrimmed; invalid predictions would have been counted explicitly, rather than silently removed. No causal explanation of the failure has been established by this replication.

## Anchor and support sensitivity

Changing from RGB to thermal endpoint reference changes only 60/85,170 pointwise winner labels (~0.070%). Both endpoints preserve all 20 night failures against always RGB/random under both losses. Endpoint mean-distance learned/RGB ratios remain1.25284/1.25278, learned/random1.12105/1.12098, and learned/camera-scale0.97788/0.97788. This robustness is a benchmark sensitivity result, not proof of physical label correctness.

Even the strict2/256 m symmetric depth-consistency subset retains 3930 /4057.5 /4158 pairs per frame and preserves the failure: learned/always RGB median mean-distance ratio 1.216 and learned/random1.106, with zero frames below1; camera-scale ratio 0.976 remains below1 in all 20. The1/2/5% relative sensitivities likewise retain worse-than-RGB and worse-than-random mean-distance risk in all 20 frames. No threshold or subset is selected to rescue the result. All counts and full comparator/loss/anchor distributions remain in machine-readable reports.

## Each original night frame

| ID | Both-valid candidates | Median reference separation (m) | Thermal−RGB recorded left time (ms) | Mean-distance learned / camera scale | Learned / always RGB | Learned / equal random |
|---|---:|---:|---:|---:|---:|---:|
| 000000 | 4243 | 0.01289 | -4.583 | 0.995 | 1.220 | 1.109 |
| 000001 | 4279 | 0.01281 | -11.756 | 0.998 | 1.321 | 1.179 |
| 000002 | 4241 | 0.01272 | -6.001 | 0.987 | 1.250 | 1.124 |
| 000003 | 4207 | 0.01274 | -4.410 | 0.976 | 1.192 | 1.097 |
| 000004 | 4302 | 0.01283 | -2.974 | 0.983 | 1.262 | 1.126 |
| 000005 | 4223 | 0.01246 | -10.142 | 0.979 | 1.279 | 1.139 |
| 000006 | 4319 | 0.01288 | -4.381 | 0.985 | 1.288 | 1.138 |
| 000007 | 4218 | 0.01287 | 6.832 | 0.981 | 1.257 | 1.121 |
| 000008 | 4275 | 0.01282 | -1.522 | 0.966 | 1.139 | 1.068 |
| 000009 | 4245 | 0.01278 | -8.542 | 0.985 | 1.249 | 1.124 |
| 000010 | 4182 | 0.01289 | -2.752 | 0.967 | 1.111 | 1.039 |
| 000011 | 4262 | 0.01276 | 8.564 | 0.976 | 1.316 | 1.158 |
| 000012 | 4236 | 0.01273 | 0.124 | 0.971 | 1.164 | 1.071 |
| 000013 | 4249 | 0.01276 | -6.868 | 0.975 | 1.255 | 1.123 |
| 000014 | 4263 | 0.01286 | -1.034 | 0.977 | 1.217 | 1.105 |
| 000015 | 4354 | 0.01253 | -0.991 | 0.987 | 1.294 | 1.142 |
| 000016 | 4235 | 0.01284 | -2.548 | 0.977 | 1.239 | 1.111 |
| 000017 | 4287 | 0.01284 | -2.863 | 0.995 | 1.203 | 1.095 |
| 000018 | 4253 | 0.01293 | 60.564 | 0.967 | 1.258 | 1.106 |
| 000019 | 4297 | 0.01318 | 7.407 | 0.968 | 1.257 | 1.121 |

## Independent validation and replay

The unchanged EXP057 independent validator passes on original raw night data:170,340 reconstructed model predictions,120 sampled brute-force nearest/frontmost checks,80 depth masks,600 selector risks,800 subset risks (16,800 scalar risk fields),40 timestamp joins and230 frozen-hash checks including repeats. Layout/original-member provenance separately passes for 124 byte-identical source/copy pairs and three pinned original manifests. These checks measure implementation/preservation, not independent statistical observations or certified physical correspondence.

Input and helper hashes are recorded in all protocols. The generic runner's historical `source_commit` metadata refers to its original helper context; this experiment starts at `fc1434c2d` with unchanged helpers and its own adapter/source hashes. All prior rain/day scientific files remain unchanged. The three fixed overlays, paired patches and final PNG/PDF were inspected.

New code is limited to the byte-preserving staging adapter and three-recording plot. Existing `ms2_cross_modal_replication.py` and `validate_ms2_cross_modal_replication.py` execute unchanged. Runtime remains isolated Python3.12.3, NumPy2.2.6, SciPy1.15.3, Pillow11.3.0, Matplotlib3.10.7. From `ore-ms`, after restoring original EXP043/044-complete/045 scientific payloads:

```bash
python MAC-VO-main/Scripts/Setup/stage_ms2_night_cross_modal.py --root /root/AirStack/ore-ms --recovery-root /root/ore-ms-recovery --output /root/AirStack/ore-ms/results/ms2-night-cross-modal-replication/input
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_replication.py prepare --root /root/AirStack/ore-ms --dataset results/ms2-night-cross-modal-replication/input --recovery /root/AirStack/ore-ms/results/ms2-night-cross-modal-replication/input/layout-provenance.json --output /path/to/fresh-night/support
# Inspect fixed overlays and save inspection-decision.md before metric scoring.
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_replication.py score --root /root/AirStack/ore-ms --dataset results/ms2-night-cross-modal-replication/input --recovery /root/AirStack/ore-ms/results/ms2-night-cross-modal-replication/input/layout-provenance.json --support /path/to/inspected-night/support --output /path/to/fresh-night/metric
python MAC-VO-main/Scripts/Setup/validate_ms2_cross_modal_replication.py --root /root/AirStack/ore-ms --dataset results/ms2-night-cross-modal-replication/input --recovery /root/AirStack/ore-ms/results/ms2-night-cross-modal-replication/input/layout-provenance.json --support /path/to/inspected-night/support --metric /path/to/fresh-night/metric --output /path/to/fresh-validation.json
```

Staging and scientific runners refuse existing outputs. The plot expects the preserved EXP057 rain/day metric reports and current night metric report at their documented paths. Generated scientific outputs, layout provenance, scripts, fixed pre-scoring inspection and validation are preserved in the new recovery snapshot; duplicated staged input payloads, original archives, model weights and private configs are excluded. Recreate the input directory by staging from the verified prerequisites when replaying.

## Research decision and roadblocks

This third-recording replication is complete. There is no new compute/access blocker and no need for another source-owner calibration question to report the result. The concrete empirical roadblock is that the frozen metric uncertainty score can systematically prefer worse measurements across cameras, even though within-camera uncertainty ranking was useful.

The next necessary work is a focused **score-failure diagnosis**, separating camera-wide scale, local variance variation and nonlinear disparity-to-depth propagation against the fixed-camera/random baselines. Current results identify the failure, not its cause. Any fitted correction needs a declared recording-level development/evaluation protocol; using these already inspected scores to tune a rule and then calling the same windows held-out would not establish transfer.

Targeted papers on uncertainty calibration/model selection across heterogeneous sensors and nonlinear stereo-depth uncertainty are useful for that diagnosis. Occlusion/motion-aware sparse RGB–thermal correspondence remains relevant to stronger physical claims. Additional unchanged repetitions or immediate fusion are not justified as the sole response to this failure. CATS access and checkpoint training membership remain pending and separate; the latter is needed before network-training-held-out claims.
