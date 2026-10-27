# EXP-060: Recording-excluded relative camera-score calibration

Date: 2026-10-27 America/New_York. Branch `ore_ms`; starting commit `fcc0d9d0f`.

**Finding: a fitted constant relative camera-score scale does not establish reliable transfer across the three tested recordings.** It improves day and partly improves night against the original selector, while worsening rain in every frame. Night still loses to always RGB in all20 frames under both losses. No fitted scale is adopted as a default.

## Frozen method and scope

The method and fixed grid were written before fitting in this session. The three rain/day/night opening20-ID windows and their outcomes were already inspected in EXP056–059; recording exclusion prevents fitting on the excluded recording but does not make these untouched validation or establish network-training-held-out status. This is a bounded exploratory transfer test on three recordings, not a generalization or weather-causality claim.

For each fold, fit one constant `c` using the other two recordings, then choose RGB when `c * U_RGB < U_thermal`. Exact score ties use an equal mixture. Both U values are the original predicted-depth, rank-one propagated scores. The candidate grid is `log10(c)=-6,...,6` in steps0.05 (241 finite constants), plus always-RGB and always-thermal limits. Primary fit loss is the equally weighted recording mean of per-image mean3D-distance risk divided by that image’s equal-random risk. Each recording has20 frames; pixels are not pooled. Exact fit-objective ties prefer a finite constant nearest1, then smaller c; limiting policies follow, RGB first. The training-selected fixed-camera comparator uses the same objective and training recordings.

The fitted constant and fixed-camera baseline are frozen before evaluating the excluded recording under midpoint/RGB/thermal anchors and mean-L2/squared losses. Squared loss and endpoint anchors do not enter fitting. All predictions, errors, both-valid supports and all20 IDs remain unchanged. Original camera-scale control and both fixed cameras are retained. Reference oracle is diagnostic, not deployable. No supplied reference depth is used in the fitted score at evaluation, but evaluation support remains reference-selected. No new inference, model training, clipping, timing/calibration adjustment, VO or fusion.

## Fold decisions

| Excluded recording | Development recordings | Selected log10(c) | c | Training-selected fixed camera |
|---|---|---:|---:|---|
| rain | day, night | -2.45 | 0.00354813 | always_rgb |
| day | rain, night | -2.45 | 0.00354813 | always_thr |
| night | rain, day | -0.8 | 0.158489 | always_thr |

## Excluded-recording results

Entries are medians of20 per-image paired fitted-selector/comparator risk ratios, with the number below1 in parentheses. Below1 favors the fitted selector. Cameras share the same conditional candidate reference and both-valid support. Repeated frames, anchors, losses and pixels are dependent observations.

### midpoint anchor

| Recording | Loss | Original | Camera scale | Always RGB | Always thermal | Random | Training-fixed | Oracle |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| rain | mean_l2 | 1.4155 (0/20) | 1.4131 (0/20) | 0.2880 (20/20) | 1.0784 (5/20) | 0.4640 (19/20) | 0.2880 (20/20) | 1.8242 (0/20) |
| rain | mse | 6.2454 (0/20) | 5.8636 (0/20) | 0.0222 (20/20) | 3.7431 (0/20) | 0.0442 (19/20) | 0.0222 (20/20) | 7.7288 (0/20) |
| day | mean_l2 | 0.8492 (20/20) | 0.7678 (20/20) | 0.9954 (10/20) | 0.5772 (20/20) | 0.7310 (20/20) | 0.5772 (20/20) | 1.1437 (0/20) |
| day | mse | 0.6474 (20/20) | 0.4846 (20/20) | 0.9753 (10/20) | 0.3754 (20/20) | 0.5360 (20/20) | 0.3754 (20/20) | 1.2896 (0/20) |
| night | mean_l2 | 0.9524 (20/20) | 0.9363 (20/20) | 1.1905 (0/20) | 0.9654 (18/20) | 1.0662 (1/20) | 0.9654 (18/20) | 1.6943 (0/20) |
| night | mse | 0.9191 (20/20) | 0.8999 (20/20) | 1.5030 (0/20) | 0.9683 (17/20) | 1.1646 (0/20) | 0.9683 (17/20) | 2.3217 (0/20) |

### rgb anchor

| Recording | Loss | Original | Camera scale | Always RGB | Always thermal | Random | Training-fixed | Oracle |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| rain | mean_l2 | 1.4176 (0/20) | 1.4147 (0/20) | 0.2878 (20/20) | 1.0798 (5/20) | 0.4634 (19/20) | 0.2878 (20/20) | 1.8339 (0/20) |
| rain | mse | 6.2585 (0/20) | 5.8738 (0/20) | 0.0222 (20/20) | 3.7504 (0/20) | 0.0442 (19/20) | 0.0222 (20/20) | 7.7423 (0/20) |
| day | mean_l2 | 0.8477 (20/20) | 0.7679 (20/20) | 0.9954 (10/20) | 0.5775 (20/20) | 0.7301 (20/20) | 0.5775 (20/20) | 1.1446 (0/20) |
| day | mse | 0.6459 (20/20) | 0.4846 (20/20) | 0.9752 (10/20) | 0.3763 (20/20) | 0.5371 (20/20) | 0.3763 (20/20) | 1.2883 (0/20) |
| night | mean_l2 | 0.9524 (20/20) | 0.9363 (20/20) | 1.1906 (0/20) | 0.9654 (18/20) | 1.0662 (1/20) | 0.9654 (18/20) | 1.6943 (0/20) |
| night | mse | 0.9191 (20/20) | 0.8999 (20/20) | 1.5031 (0/20) | 0.9683 (17/20) | 1.1646 (0/20) | 0.9683 (17/20) | 2.3216 (0/20) |

### thr anchor

| Recording | Loss | Original | Camera scale | Always RGB | Always thermal | Random | Training-fixed | Oracle |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| rain | mean_l2 | 1.4133 (0/20) | 1.4115 (0/20) | 0.2881 (20/20) | 1.0770 (5/20) | 0.4646 (19/20) | 0.2881 (20/20) | 1.8139 (0/20) |
| rain | mse | 6.2321 (0/20) | 5.8532 (0/20) | 0.0222 (20/20) | 3.7358 (0/20) | 0.0442 (19/20) | 0.0222 (20/20) | 7.7151 (0/20) |
| day | mean_l2 | 0.8507 (20/20) | 0.7678 (20/20) | 0.9954 (10/20) | 0.5789 (20/20) | 0.7311 (20/20) | 0.5789 (20/20) | 1.1465 (0/20) |
| day | mse | 0.6488 (20/20) | 0.4846 (20/20) | 0.9754 (10/20) | 0.3754 (20/20) | 0.5349 (20/20) | 0.3754 (20/20) | 1.2914 (0/20) |
| night | mean_l2 | 0.9524 (20/20) | 0.9363 (20/20) | 1.1905 (0/20) | 0.9655 (18/20) | 1.0662 (1/20) | 0.9655 (18/20) | 1.6943 (0/20) |
| night | mse | 0.9191 (20/20) | 0.8999 (20/20) | 1.5029 (0/20) | 0.9683 (17/20) | 1.1645 (0/20) | 0.9683 (17/20) | 2.3217 (0/20) |

## Interpretation and availability

On rain, the fitted selector loses to original and camera-scale controls in all20 frames under both losses, although it still beats always RGB in all20 and random in19/20. On day it beats original, camera scale and random in all20, but beats always RGB in only10/20 under either loss. On night it beats original and camera scale in all20, yet loses to always RGB in all20 and random in19/20 mean-distance and20/20 squared cases. Fitting a scale can improve an inadequate selector without making it a reliable modality policy. Endpoint sensitivities are retained above rather than used to choose a favorable fit.

The development-only fixed-camera choice itself transfers inconsistently: day/night folds choose always thermal, while the rain fold chooses always RGB. The fitted method beating that comparator does not prove it beats the more relevant fixed-camera counterexamples. No universal correction is supported. The one-parameter model may be inadequate; these data do not prove that every calibration method must fail.

Both-valid pairs remain25,554 rain,22,717 day,85,170 night. Rain additionally has200 thermal-only candidates; day/night have no one/neither-valid candidates on this support. This experiment retains those counts but does not fit an availability policy or evaluate the excluded candidates as if both predictions existed. Support geometry/reference provenance, uncompensated acquisition motion, rank-one first-order uncertainty and checkpoint training-membership limits remain inherited from EXP056–059. Reference disagreement is not certified physical error.

## Recovery and verification

Archive SHA256 and all273 members of EXP057/058/059 passed before restoring231 missing result files. Historical tracked files were inspected in archives but not copied over current source/notes. These are manifest integrity checks, not independent validation of physical labels. The original raw source/frontend/depth payloads are not restored or newly audited here.

The independent validator passes 7,923 recorded checks: 63 hashes, 2,187 grid objectives, 2,940 test risks, 2,160 historical risks, 504 summary fields, 69 split and choice checks. It imports none of the fitting code, reconstructs selections and per-policy risks from saved metric arrays, checks training objectives and argmins, fold exclusion, both limiting policies and exact ties, all per-image risks/summary fields and historical original/camera-scale risks. An additional exact-reduction argmin audit handles floating-point tie sensitivity. All63 original input files retain their hashes. These checks repeat dependent values and are implementation evidence, not independent scientific samples.

Outputs: `results/ms2-camera-scale-transfer/run-001/{protocol.json,report.json}`, `validation.json`, and `recovery-verification.json`. The protocol pins all63 input hashes, the runner hash and NumPy version. The validator pins its own hash and protocol/report hashes. CPU Python3.12.3 / NumPy2.2.6; no GPU or checkpoints needed.

```bash
python MAC-VO-main/Scripts/Setup/ms2_camera_scale_transfer.py --root /root/AirStack/ore-ms --output /path/to/fresh-run
python MAC-VO-main/Scripts/Setup/validate_ms2_camera_scale_transfer.py --root /root/AirStack/ore-ms --source /path/to/fresh-run --output /path/to/fresh-validation.json
```

## Research decision

This bounded constant-scale transfer experiment is complete. Keep current defaults and preserve all failures. More repeats on the same opening windows would not establish a calibrated camera-risk method. A subsequent method should declare a richer risk model, recording-level development protocol and evaluation target before fitting, with these results treated as development evidence. Within-camera ranking remains a separate supported observation.

Independent recordings with useful cross-camera error variation and stronger references would materially strengthen the study. Check existing Box/MS² coverage before acquiring data: day/rain archives and indices were preserved in earlier work but are not local in this session, and unused intervals can support a declared follow-up without an upload; they remain the same recordings. A genuinely unexamined recording or an accessible bounded CATS laser-reference sample would offer stronger evaluation evidence. CATS access and checkpoint training membership remain pending in the handoff; neither is assumed resolved. No owner message or new upload is required to report this experiment.

Recovery target: Box `ore-ms/backups/2026-10-27-exp-060`. Includes new code/notes/results, the pre-fit notebook, and an incremental Git bundle; EXP057/058/059 remain scientific-input prerequisites. Private credentials and model/full data archives excluded.
