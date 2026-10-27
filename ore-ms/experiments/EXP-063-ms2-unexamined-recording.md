# EXP-063: Frozen camera selectors on a previously unexamined recording

Date: 2026-10-27 America/New_York. Branch `ore_ms`; starting commit `db1d4934c`.

**Finding: the primary incremental-geometry gain does not transfer to this new recording.** The frozen combined model never beats uncertainty-only:17 frames worse and3 exact ties under both losses. Median per-image combined/uncertainty risk ratios are1.0385 for mean3D distance and1.4012 for squared error. Combined also never beats always RGB (15 worse,5 ties), although it improves on the original uncalibrated selector in many frames. No model is retuned or adopted.

## Selection and pre-acquisition freeze

The pinned official [rainy test list](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/MS2dataset/test_rainy_list.txt), together with day/night lists, defines the candidate pool. Exclude the three scientifically examined development recordings and select the first remaining sequence lexicographically: `_2021-08-06-16-45-28`. Freeze originalIDs000000–000019 before viewing new pixels or scores. No image-quality, timing or outcome replacement. The label is official split metadata, not verification of rainfall or a causal weather variable. Source access uses the user-supplied EXP040 download scripts; their hashes and split hashes are preserved.

Use the unchanged EXP062 four model families—constant, uncertainty-only, geometry-only and combined—with combined declared primary. Fit them **once on all three development recordings** (60 frames/133,441 both-valid development pairs), rather than reusing the favorable night-fold coefficients. Same six numerical inputs, fixedridge0.01, cost-sensitive squared surrogate, training-only equal-recording/frame standardization and decision threshold. Freeze the EXP060243-policy scalar multiplier and development-selected fixed camera using the same all-three development set. No hyperparameter or family selection uses this new recording.

Frozen artifact `frozen-models.json` was written at2026-10-27 19:42:15 America/New_York; new frontend execution began19:48:48. SHA256 `51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377` is enforced by the runner and independent validator. It pins the pre-acquisition declaration, original method/helper sources, split files, all128 development-input hashes, coefficients, scalers, training objectives and controls. Independent augmented least-squares reconstruction verifies the four fits/scalers and all243 scalar objectives. The development-selected fixed camera is thermal; scalar multiplier log10c=-2.45.

“Previously unexamined” concerns our scientific inspection of this capture. Checkpoint training membership is unknown; this is not a network-training-held-out claim. It is a distinct capture within the same collection/rig/reference pipeline, not statistically independent ground truth. These20 frames now count as examined; future changes must not reuse them as a new untouched test.

## Source acquisition and bounded payload

Only the selected recording’s sync/depth transport archives were acquired through64MiB HTTP206 ranges with exact Content-Range and byte-count validation. All413 ranges completed, with one recovered transient connection retry. Full compressed-stream scans and safe-member extraction reject absolute/traversal/link/special paths and bound selected files to32MiB. Complete tar indices are retained. Observed hashes are local integrity evidence, not publisher-issued checksums.

| Component | Bytes | Observed SHA256 |
|---|---:|---|
| sync_data | 16,322,830,705 | `1cd7376500afcd5021e5c9ac96e505edddf434fa9dc765447e5ac98d36835420` |
| proj_depth | 11,287,770,154 | `f69e79121102985bcf6150d7286732cf22a047959f219350de91b2639ca40ddb` |

Extract only80 native camera PNGs,40 single-scan depth PNGs,calibration,four complete timestamp streams and readmes (127 source members). Source image dtypes/sizes match the original contract: RGBuint8 384×1224×3; raw thermaluint16 256×640; depthuint16/256, zero invalid. Own calibration is decoded without fitting and is byte-equivalent in decoded JSON to the earlier rig calibration. No new rectification, clock/pose fit, timestamp gate or reference selection. Poses/navigation were not needed or acquired for this stereo-only experiment.

Each full camera timestamp stream has6,672 rows; the selected left-camera durations are1.933134216s RGB and1.871203872s thermal. Recorded RGB right-minus-left spans−0.2573 to−0.2296ms; thermal−8.9991 to9.4001ms; thermal-minus-RGB left spans−15.6497 to55.5136ms (median17.5925ms). These are recorded-time diagnostics, not exposure-delay measurements or inclusion gates. All20 supplied pairs remain.

All80 new camera file hashes are distinct from the240 recorded development camera hashes, including right images. This checks byte overlap, not physical scene independence. Full27.61GB transport archives stay local and are excluded from recovery; bounded original source pixels and all saved frontend fields are preserved.

## Runtime and support

Rebuilt the committed Blackwell/MS2 runtime, image `sha256:32b4393ac20e8dc63615a2b6c88bd6471a4685a490365b715f61f1ef03c94f8c`. Frontend Python3.11.13,Torch2.7.1+cu128,CUDA12.8,torchvision0.22.1,pypose0.7.3,NumPy2.2.6,OpenCV4.10.0.84,timm0.9.12,einops0.8.2 on RTX PRO5000 Blackwell. The rebuilt image has recorded transitive-package differences from earlier images. Before new inference, restored EXP048 first-frame RGB/thermal outputs reproduce bitwise, with exact model-input tensors, effective config and original checkpoint hashes; both repeated calls are bitwise. This bridges the tested first pairs, not a re-execution of every prior prediction. Startup driver diagnostics remain in logs; all numerical checks pass.

The unchanged native stereo runner produces40 saved flow/covariance fields and two extra first-pair repeat calls. All source IDs/timestamps/shapes,float32/finiteness/positive covariance and80 independently recreated producer-input tensors pass. Original RGB/thermal weights remain SHA-pinned; thermal preprocessing is the same explicit audited recipe, with equivalence to checkpoint training preprocessing still unknown. No stereo-network training or new VO.

Run unchanged EXP057 support/metric helpers and independent raw validator. Candidate radius,mutual/frontmost rules,midpoint and endpoint anchors,depth-consistency sensitivities and all controls stay fixed. All22,277 candidate pairs are both-valid; zero one/neither-valid. Counts/frame199–2036,median950.5. Median per-image reference endpoint separation0.2031m (range0.0492–0.4144m); that is disagreement between reference products, not certified same-point accuracy.

Before metric scoring inspect fixed overlays000000/000009/000019: outdoor bus parking/road area, buildings, trees, pedestrians and cloudy visible sky. Proceed with every frame under the nominal supplied-reference protocol; no physical correspondence certification or image-driven adjustment. Shared reference production,visibility,acquisition motion and first-order covariance limits persist. Numerical features use only predictions/coordinates, but candidate matching/support is reference-selected, so this is not a deployed correspondence/availability policy.

## Primary comparison and all midpoint controls

Entries are medians of20 paired per-image model/comparator risk ratios with strict-below1 counts. Below1 favors the model. Mean-distance risk is in metres, squared risk in m²; ratios dimensionless. Exact ties are retained. No pixel pooling, confidence interval, significance test or independence claim.

| Loss | Family | Uncertainty-only | Geometry-only | Always RGB | Always thermal | Random | Original learned | Camera-scale | Scalar-fit |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mean_l2 | constant | 2.0775 (0/20) | 1.9167 (0/20) | 1.9993 (0/20) | 1.0000 (0/20) | 1.3332 (0/20) | 1.4100 (0/20) | 1.3801 (0/20) | 2.0557 (0/20) |
| mean_l2 | uncertainty | — | 0.9074 (20/20) | 0.9696 (12/20) | 0.4814 (20/20) | 0.6327 (20/20) | 0.6821 (20/20) | 0.6605 (20/20) | 1.0000 (8/20) |
| mean_l2 | geometry | 1.1022 (0/20) | — | 1.0474 (1/20) | 0.5217 (20/20) | 0.6950 (20/20) | 0.7746 (19/20) | 0.7441 (19/20) | 1.0830 (0/20) |
| mean_l2 | combined | 1.0385 (0/20) | 0.9580 (19/20) | 1.0023 (0/20) | 0.5043 (20/20) | 0.6723 (20/20) | 0.7397 (19/20) | 0.7077 (20/20) | 1.0117 (2/20) |
| mse | constant | 3.6635 (0/20) | 3.0548 (1/20) | 3.0981 (2/20) | 1.0000 (0/20) | 1.5120 (2/20) | 1.5128 (0/20) | 1.3363 (0/20) | 3.4341 (0/20) |
| mse | uncertainty | — | 0.7119 (20/20) | 0.7143 (12/20) | 0.2732 (20/20) | 0.3360 (20/20) | 0.3757 (20/20) | 0.3444 (20/20) | 1.0000 (8/20) |
| mse | geometry | 1.4048 (0/20) | — | 1.0078 (1/20) | 0.3274 (19/20) | 0.4833 (19/20) | 0.6312 (14/20) | 0.4583 (14/20) | 1.1262 (1/20) |
| mse | combined | 1.4012 (0/20) | 0.9927 (19/20) | 1.0002 (0/20) | 0.3231 (18/20) | 0.4885 (18/20) | 0.6216 (14/20) | 0.4516 (14/20) | 1.0895 (3/20) |

The declared combined/uncertainty mean-distance ratio1.0385 corresponds to3.85% median paired harm; squared ratio1.4012 corresponds to40.1%. Three frames (000012,000014,000016) tie exactly under both losses; the other17 favor uncertainty-only. Some squared-loss relative harms are large (maximum45.424). No favorable anchor is selected to rescue this comparison.

Combined/always-RGB median ratios1.00235 mean-distance and1.00018 squared are small deficits, not large universal RGB advantages. Combined is worse in15 frames and exactly tied in5 under each loss. It does improve over geometry-only in19/20 frames under both losses, but adding uncertainty to geometry and adding geometry to uncertainty are different comparisons.

Uncertainty-only is nearly always RGB (median99.93% of candidates); combined median99.27%. Uncertainty-only beats always RGB in12/20 cases under each loss, whereas combined never does. Constant/development-selected camera is always thermal and transfers poorly here. Beating random/original/that weak constant is insufficient evidence for the primary geometry increment. Uncertainty-only is a predeclared control, not a post-result replacement winner; it needs its own prospective broader evaluation before adoption.

For absolute scale, medians of per-image mean-distance risks are6.097m always RGB,12.321m thermal,5.987m uncertainty-only and6.108m combined. Ratios of these displayed medians need not equal median paired ratios above. The supplied-reference errors are substantial and are not physical accuracy certification.

![Frozen new-recording evaluation](../results/ms2-unexamined-recording/figures-final/frozen-recording.png)

## Endpoint sensitivities

Each anchor reuses the same frozen choices and original support. No retraining or threshold change.

| Anchor | Loss | Combined/U-only | Combined/geometry | Combined/RGB | Combined/thermal | Combined/random |
|---|---|---:|---:|---:|---:|---:|
| rgb | mean_l2 | 1.0385 (0/20) | 0.9567 (18/20) | 1.0023 (0/20) | 0.5051 (20/20) | 0.6730 (20/20) |
| rgb | mse | 1.4025 (0/20) | 0.9928 (19/20) | 1.0002 (0/20) | 0.3187 (18/20) | 0.4833 (18/20) |
| thr | mean_l2 | 1.0385 (0/20) | 0.9569 (19/20) | 1.0024 (0/20) | 0.5036 (20/20) | 0.6717 (20/20) |
| thr | mse | 1.3998 (0/20) | 0.9927 (19/20) | 1.0002 (0/20) | 0.3230 (18/20) | 0.4884 (18/20) |

Both endpoint anchors preserve zero combined wins against uncertainty-only and always RGB under both losses. All per-image/family/anchor/loss risks,coefficients,features,margins,choices and availability counts remain in the reports/NPZs. Original metric reports also retain all frozen depth-consistency sensitivities for the historical comparators.

## Validation and preservation

- Freeze audit:128 original development-input hashes,independent augmented-LS fits/scalers on60frames/133,441 pairs,all243 scalar objectives and chosen policies match the pre-acquisition artifact. No new-recording labels enter fitting or normalization.
- Source/frontend audit:120 complete PNG files and3,571 chunk CRCs,127 archive members,80 timestamp joins,80 archived-producer tensor hashes,40 saved fields and two bitwise repeats pass; source camera overlap with development iszero.
- Independent raw geometry/metric audit:44,554 raw model reconstructions,120 nearest/frontmost samples,80 depth masks,600 selector risks/16,800 scalar fields,800 subset risks,40 timestamp joins and230 repeated frozen-hash checks pass. Physical reference certification remainsfalse.
- Frozen-evaluation audit:170 hash checks,133,662 feature values,89,108 margins,89,108 decisions,1,660 risk fields,1,056 summary fields,720 historical fields and100 availability fields pass. Minimum absolute margin1.785e-6. The validator imports no fitting implementation and independently reconstructs features/margins and scalar risks.
- Synthetic evaluator checks: an artificial unavailable frame remains explicitly present with null risks/empty margins and19 available frames; changed frozen artifact is rejected before preparation. Synthetic tests are not scientific samples. Original frozen artifact and scientific inputs stay unchanged.

These are repeated dependent implementation checks, not independent statistical observations. Saved-array replay uses CPU Python3.12.3,NumPy2.2.6,SciPy1.15.3,OpenCV4.10.0.84,Pillow andMatplotlib3.10.7; new frontend execution additionally requires restored checkpoints/GPU. The strict frozen-artifact hash is part of this bounded experiment.

Key files under `results/ms2-unexamined-recording`: pre-acquisition declaration and `frozen-models.json`; acquisition/extraction manifests and complete tar indices; `dataset/pilot`,source/calibration manifests,40`dataset/frontend-run-001` fields; `support`,`metric`,`frozen-evaluation`; independent validations and `figures-final`. Source acquisition/extraction/freeze audit/synthetic scripts are preserved verbatim under `source-scripts`; new reusable stage/evaluation/audit/plot scripts are tracked. Runtime parity outputs/logs and historical source manifests/producer needed by audits are preserved.

After restoring the frozen artifact,bounded inputs and original prerequisites, use fresh paths. Source preparation requires manifests without overwriting; saved source manifests can be reused for replay. The independent validator pins original source paths, so restore under the documented root.

```bash
python MAC-VO-main/Scripts/Setup/validate_ms2_frozen_source.py --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-unexamined-recording --output /path/to/fresh-source-audit.json
python MAC-VO-main/Scripts/Setup/ms2_frozen_recording.py evaluate --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-unexamined-recording --output /path/to/fresh-evaluation
python MAC-VO-main/Scripts/Setup/validate_ms2_frozen_recording.py --root /root/AirStack/ore-ms --base /root/AirStack/ore-ms/results/ms2-unexamined-recording --output /path/to/fresh-frozen-audit.json
```

The frozen audit intentionally validates the documented canonical `frozen-evaluation` path. Evaluating to another fresh path produces an additional result but does not redirect that audit automatically. The unchanged EXP057 raw validator commands are the same as EXP058,with this dataset/support/metric/integrity paths. The source/frontend validator requires original checkpoint files for SHA verification; saved-array frozen/geometry checks need no GPU.

## Research decision

The first genuinely unexamined-capture test of the declared method is complete and negative for a consistent geometry increment. Preserve the night-specific exploratory gain and this counterexample together. No model,scale,feature set,threshold or default is retuned on this result; no fusion or VO claim follows.

Further capacity on these observed windows would not establish transfer. A next prospective comparison should keep the simpler uncertainty-only/scalar/fixed-camera controls and declare additional recording/reference coverage before outcomes. This new recording is now examined. Remaining official test captures are available through the previously supplied pointers; check their access/coverage before requesting an upload. Independent geometric-reference data such as an accessible CATS sample would strengthen physical error claims, while checkpoint training membership remains needed only for network-training-held-out claims. No new data upload or author message was required here.

Box recovery target `ore-ms/backups/2026-10-27-exp-063`: bounded new source/frontend/support/evaluation files,freeze/validation/notes/figures,notebook and incremental Git bundle. Original development inputs depend on EXP057/058/060; source-selection pointers on EXP040; parity bridge on EXP048. Needed historical source manifests/thermal producer are included individually. Full transport/model archives,checkpoint files,private OAuth config and signed URLs are excluded.
