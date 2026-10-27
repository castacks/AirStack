# EXP-057: Frozen cross-modal reliability replication on MS² day

Date: 2026-10-26 America/New_York. Branch `ore_ms`; starting commit `6bed1f34d`.

**Finding: the rain camera-scale result does not transfer uniformly to the day window.** Pointwise propagated learned uncertainty beats the per-image camera-scale comparator in 19/20 day frames under mean 3D distance (median ratio 0.904, about 9.6% reduction), and20/20 under squared 3D error (ratio 0.744). Rain had near-parity ratios 1.002 and0.988. However, **alwaysRGB beats the learned selector in all 20 day frames under mean 3D distance**, with median learned/RGB ratio 1.157. The fixed selector is not universally superior to its baselines. Keep this counterexample alongside the day improvement over camera-scale control.

## Frozen replication and source recovery

Choose the already established EXP048 day opening sequence `_2021-08-06-11-23-45`, all 20 original IDs 000000–000019, before inspecting any day cross-modal winner outcomes. This is a distinct recording from rain, not an isolated weather effect, an untouched scientific sample or a verified network-training-held-out sequence. Earlier within-camera experiments already inspected these images; checkpoint membership remains unknown. Both windows are20 adjacent frames, not independent image trials. No later interval is selected to rescue any outcome.

Recover only missing `results/ms2-day-replication` members from Box `backups/2026-10-25-exp-048`. The777,347,065-byte archive SHA256 is `589b47fb91bd8bf01c905bcea581238f4f160633b6e231fecba494984010a51b`. All524 archive-member hashes pass;501 day members were selected and500 missing files restored. Existing matching content was preserved; no historical differing file was overwritten. Full 24.9 GB source archives, checkpoint weights and original GPU image are not required for saved-array replay.

Use unchanged EXP056 math helpers, original calibration, single-scan depth and saved native-resolution frontend outputs. The new runner adds dataset parameters without editing the prior rain scripts or outputs. Before day scoring, its rain control reproduces 320 support arrays bitwise,280 metric arrays numerically and7200 risk fields at rtol1e-12/atol1e-10. This checks the reimplementation against the already independently validated EXP056 calculations; it is not a second independent rain dataset.

## Unchanged support and comparison protocol

- Same normalized angular radius1/fxRGB in both directions; unique nearest depth labels and exact-tie exclusions.
- Same positive-Z/in-FOV and unique frontmost rounded-pixel sparse z-buffer tests, mutual nearest identity and one-to-one candidate pairs. No primary depth-agreement filter.
- Same depth sensitivities:2/256 m alone, then max(2/256 m,1/2/5% targetZ), with no outcome-chosen tolerance.
- Same fixed inspection IDs 000000/000009/000019 and uniformly spaced candidate displays. Building/road/bus/pole overlap looks plausible at the coarse level; no exact correspondence or dynamic-object certification follows. Decision to score is recorded before day winner results.
- Same nominal RGB-frame common midpoint and both endpoint reference anchors; same horizontal rank-one depth-uncertainty propagation, invalid-prediction policy and no positive-disparity clipping.
- Same mean squared 3D error and mean 3D distance; same alwayscamera, equalrandom/oracle, equalangular/equalpixel geometry, unpropagated learned angular variance and per-image/per-camera median angular-variance controls. No fitted scales, altered default or learned fusion.

Both calibrations decode identically to their existing saved versions. The primary analysis evaluates each camera at its own candidate left pixel and transforms the thermal reconstruction to the nominal RGB rig frame. This is a supplied-reference candidate benchmark; static rig transforms do not compensate for different acquisition times or object motion. The camera-scale baseline takes a median of learned normalized horizontal variance on reference-selected both-valid support, without fitting to error labels. It is not a transferred global camera calibration or a complete deployment policy.

## Available support

There are22,717 unique candidate pairs, all with both predictions valid; no one-valid or neither-valid cases. Per-frame counts are224 /1172.5 /1424 (min/median/max). The median of per-frame median reference-point separations is 0.1298 m, range 0.0851–0.1651 m. Recorded absolute left-camera separation is 26.019 /32.041 /36.503 ms (min/median/max), not measured exposure delay. Every frame remains in the benchmark; the inherited30 ms diagnostic is not imposed as a universal acceptance criterion.

Raw point identity, unobserved occluders and physical timing/calibration uncertainty remain unresolved. More geometric support than rain's worst frame does not establish more accurate physical reference labels. Pixel counts and repeated anchors/controls are not independent statistical samples.

## Recording-level comparison

Ratios are learned-selector/comparator risk on identical both-valid support, summarized as the median over20 per-image paired ratios. Below1 favors the learned selector. No pixels/recordings are pooled and no significance test is applied.

| Recording | Loss | Learned / camera scale | Below camera scale | Learned / always RGB | Below always RGB |
|---|---|---:|---:|---:|---:|
| Rain | Squared 3D error | 0.988 | 11/20 | 0.003 | 20/20 |
| Rain | Mean 3D distance | 1.002 | 9/20 | 0.190 | 20/20 |
| Day | Squared 3D error | 0.744 | 20/20 | 1.318 | 7/20 |
| Day | Mean 3D distance | 0.904 | 19/20 | 1.157 | 0/20 |

Day shows additional benefit from within-image variance variation relative to the tested per-image median-scale rule, while rain did not show a consistent advantage. This is evidence of variability between these two fixed windows, not a causal day/rain or weather explanation. Beating camera-scale control is insufficient to adopt the selector: on day, alwaysRGB and the unpropagated learned-angular comparator have lower mean-distance risk in every image. Under squared loss, alwaysRGB wins 13/20 day cases and the unpropagated comparator wins 20/20. Those outcomes must not be hidden by reporting only the geometry/camera-scale comparisons.

## All frozen day comparators

| Loss | Comparator | Learned/comparator ratio min / median / max | Learned lower risk |
|---|---|---:|---:|
| Squared 3D error | Always RGB | 0.4406 / 1.318 / 1.693 | 7/20 |
| Squared 3D error | Always thermal | 0.4408 / 0.5773 / 0.6683 | 20/20 |
| Squared 3D error | Equal random expectation | 0.4868 / 0.799 / 0.9287 | 20/20 |
| Squared 3D error | Equal angular geometry | 0.4198 / 0.552 / 0.6289 | 20/20 |
| Squared 3D error | Equal pixel geometry | 0.4323 / 0.5679 / 0.6497 | 20/20 |
| Squared 3D error | Per-image camera median variance | 0.5517 / 0.7439 / 0.9802 | 20/20 |
| Squared 3D error | Unpropagated learned angular variance | 1.191 / 1.556 / 1.729 | 0/20 |
| Mean 3D distance | Always RGB | 1.037 / 1.157 / 1.245 | 0/20 |
| Mean 3D distance | Always thermal | 0.6292 / 0.6917 / 0.7314 | 20/20 |
| Mean 3D distance | Equal random expectation | 0.7832 / 0.8673 / 0.9103 | 20/20 |
| Mean 3D distance | Equal angular geometry | 0.6115 / 0.6722 / 0.7068 | 20/20 |
| Mean 3D distance | Equal pixel geometry | 0.65 / 0.7182 / 0.7553 | 20/20 |
| Mean 3D distance | Per-image camera median variance | 0.8137 / 0.9045 / 1.04 | 19/20 |
| Mean 3D distance | Unpropagated learned angular variance | 1.102 / 1.189 / 1.286 | 0/20 |

The pointwise learned rule chooses RGB on a median 78.0% of candidate pairs, versus91.9% for camera-scale control. Its lower-error choice fraction is 63.8% at the per-image median; alwaysRGB's corresponding fraction is 71.3%. These per-point fractions and mean-risk comparisons answer different questions; neither makes uncertainty a calibrated probability. Finite tiny-disparity predictions remain unclipped, so squared-risk outcomes are particularly tail-sensitive.

![Frozen rain/day replication](../results/ms2-day-cross-modal-replication/rain-day-comparison.png)

## Reference/support sensitivities

The RGB and thermal endpoint anchors change380/22,717 pointwise winner labels (1.67%). Learned/camera-scale median ratios remain0.7438/0.7439 for squared loss and0.9051/0.9038 for mean-distance loss (RGB/thermal anchors), with 20/20 and19/20 improvements respectively. AlwaysRGB still wins all 20 mean-distance cases under each endpoint anchor. Robustness to reference-anchor choice does not independently validate shared point identity.

The strict2/256 m consistency subset leaves only 3 /13 /30 points per frame; its learned/camera-scale median mean-distance ratio is 1.000 (9/20 below1), and learned beats alwaysRGB in 2/20. This is a much smaller selected population and is not promoted to primary. The1/2/5% sensitivities retain day mean-distance learned/camera-scale medians about 0.901/0.905/0.906 (19/20 below1), and alwaysRGB wins 20/20 each. All original counts, reference thresholds, unavailable cases and comparator results remain machine-readable; no favorable subset is selected.

## Per-frame primary results

| Source ID | Both-valid pairs | Median reference separation (m) | Thermal−RGB recorded left time (ms) | Mean-distance learned / camera scale | Mean-distance learned / RGB |
|---|---:|---:|---:|---:|---:|
| 000000 | 1096 | 0.0851 | 26.277 | 0.949 | 1.166 |
| 000001 | 1172 | 0.1633 | -34.609 | 1.040 | 1.144 |
| 000002 | 1374 | 0.1055 | 29.119 | 0.941 | 1.102 |
| 000003 | 1254 | 0.1244 | -31.806 | 0.935 | 1.228 |
| 000004 | 1290 | 0.1054 | 27.525 | 0.913 | 1.193 |
| 000005 | 917 | 0.1320 | -33.272 | 0.895 | 1.037 |
| 000006 | 1424 | 0.1263 | 33.335 | 0.975 | 1.039 |
| 000007 | 1316 | 0.1246 | -34.880 | 0.899 | 1.217 |
| 000008 | 1072 | 0.1651 | 31.256 | 0.933 | 1.162 |
| 000009 | 1149 | 0.1556 | -36.503 | 0.877 | 1.213 |
| 000010 | 1173 | 0.1311 | 33.208 | 0.889 | 1.108 |
| 000011 | 1339 | 0.1247 | -33.585 | 0.858 | 1.155 |
| 000012 | 1377 | 0.1213 | 26.019 | 0.954 | 1.107 |
| 000013 | 1166 | 0.1476 | -34.952 | 0.883 | 1.091 |
| 000014 | 692 | 0.1470 | 28.606 | 0.822 | 1.129 |
| 000015 | 224 | 0.1379 | -32.275 | 0.814 | 1.222 |
| 000016 | 1360 | 0.1285 | 27.199 | 0.910 | 1.202 |
| 000017 | 1071 | 0.1457 | -33.727 | 0.894 | 1.159 |
| 000018 | 1347 | 0.1250 | 29.971 | 0.945 | 1.245 |
| 000019 | 904 | 0.1419 | -30.980 | 0.836 | 1.074 |

## Validation and reproducibility

Independent raw-input audit PASS:45,434 model predictions,120 sampled nearest-label/frontmost checks,80 reference-depth masks,600 selector risks and800 subset risks (16,800 scalar risk fields),40 timestamp joins and230 frozen-hash checks including repeats. It recomposes transforms with 4×4 inverses, reconstructs every raw prediction/variance with scalar arithmetic, independently takes camera-scale medians, checks all selection decisions and all risk fields/ratios including oracle/random, and preserves each availability case. These counts are implementation validation, not independent observations. Historical rain scripts/results and the restored day inputs remain unchanged.

Inputs and producer/validator hashes are recorded in the output protocols/reports. Runtime remains isolated Python3.12.3, NumPy2.2.6, SciPy1.15.3, Pillow11.3.0, Matplotlib3.10.7; no model/GPU execution. The three fixed overlays and final PNG/PDF were inspected.

New scripts under `MAC-VO-main/Scripts/Setup`:

- `ms2_cross_modal_replication.py` has `prepare` and `score` stages; it imports unchanged EXP056 geometry, reconstruction and risk helpers.
- `validate_ms2_cross_modal_replication.py` independently audits the selected dataset against its original Box manifest and raw payloads.
- `plot_ms2_cross_modal_replication.py` compares rain and day without pooling their samples.

From `ore-ms`, after restoring EXP048 bounded day inputs, EXP054 rain prerequisites and EXP055/056 context:

```bash
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_replication.py prepare --root /root/AirStack/ore-ms --dataset results/ms2-day-replication --recovery /root/ore-ms-recovery/2026-10-25-exp-048/manifest.json --output /path/to/fresh-day/support
# Inspect the fixed overlays; save an inspection-decision.md before scoring.
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_replication.py score --root /root/AirStack/ore-ms --dataset results/ms2-day-replication --recovery /root/ore-ms-recovery/2026-10-25-exp-048/manifest.json --support /path/to/fresh-day/support --output /path/to/fresh-day/metric
python MAC-VO-main/Scripts/Setup/validate_ms2_cross_modal_replication.py --root /root/AirStack/ore-ms --dataset results/ms2-day-replication --support /path/to/fresh-day/support --metric /path/to/fresh-day/metric --recovery /root/ore-ms-recovery/2026-10-25-exp-048/manifest.json --output /path/to/fresh-validation.json
```

For the frozen rain control, substitute `results/ms2-rain-replication` and the EXP054 recovery manifest. The comparison plot expects `rain-control/metric/report.json` and `day/metric/report.json` under one base, and refuses existing figure outputs. Every scientific runner refuses existing outputs. No historical default, image, calibration, time, threshold, checkpoint, inference setting, training split or fusion behavior changed. Results, protocols, pre-scoring inspection, rain-reproduction evidence and source are backed up separately from original large archives/weights.

## Decision and remaining roadblocks

The frozen replication is complete. There is no new compute or access blocker for this bounded evidence, and no additional source-owner calibration question is required. The empirical limitation is **cross-recording baseline dependence**: one camera-selection score does not consistently outperform both fixed-camera and camera-scale alternatives. Do not tune a new scale on these same two scored windows and call it generalization.

Any next method proposal needs a defined recording-level development/evaluation split, the fixed-camera and camera-scale controls, and a clear target claim (within-camera ranking, image-wide modality reliability, spatial modality selection or calibrated metric error). Existing same-recording later-day windows can examine temporal breadth, but cannot replace independent-recording transfer. More adjacent frames chosen for a favorable result or immediate fusion would not address this finding.

Exact shared physical correspondence and nonlinear depth-uncertainty meaning remain separate limits to stronger claims. Papers on cross-sensor uncertainty calibration/model selection and occlusion/motion-aware sparse RGB–thermal correspondence are useful targeted reading. They are not prerequisites for reporting these benchmark results. CATS access and checkpoint training membership remain pending; the latter is required before verified training-held-out claims.
