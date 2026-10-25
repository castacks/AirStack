# EXP-051: Does low uncertainty select lower reference risk?

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `ed7ead9b4`.

This exploratory follow-up asks whether the existing covariance ranking can select a lower-error subset against MS²'s supplied depth references. Positive Spearman correlation alone does not guarantee useful selection under a squared-error loss. Exact exposure semantics are not a prerequisite for this provided-pair benchmark; EXP050's timing-scope clarification remains in force.

## Fixed analysis

Use all four previously inspected windows, all20 source IDs per window, both modalities, all three reference products and both own/common supports unchanged:960 image/reference/support cases. Single-scan depth and own support at50% retained coverage are primary (eight group summaries); merged/filtered labels, common support and other coverages are sensitivities. These are repeated views of160 image/modality cases, not960 independent observations. No new inference, acquisition, fitting, fusion or dataset selection.

At each pixel use normalized covariance trace as uncertainty and squared L2 normalized residual as loss, exactly from EXP046/048/049 saved arrays. Retain the lowest uncertainty at fixed100%,75%,50%,25% coverage. Equal-valued cutoff ties receive equal fractional inclusion: this computes expected risk at exact retained mass without using residuals or source order to break uncertainty ties. It is an evaluation convention, not an integer deployment policy.

The random-retention expectation is the full-support mean loss; divide selected mean loss by that value to get the risk ratio. An ideal reference-error ranking supplies the same-coverage lower bound; retaining highest uncertainty supplies a reverse descriptive control. Neither comparator is a deployable learned estimator. Preserve every image's curve, risk, oracle and reverse control; aggregate only median/min/max over20 images and counts below random expectation. No pooled pixel inference, p-values, outcome-chosen coverage or claim of independent adjacent frames.

## Interpretation limits

Squared loss emphasizes large residuals, so these results need not track median error or rank correlation. Selecting low uncertainty can change depth, spatial location and scene composition; this experiment does not isolate causal uncertainty information from those factors. EXP047/049's separate stratified associations remain relevant but do not turn this selection analysis into a controlled estimator. Common support intersects label products inside one camera, not identical physical RGB/thermal points. Camera supports/FOVs differ, so ratios are not a cross-modal winner test. Provided-label risk is not physical covariance calibration or independent geometric truth. Thermal producer/checkpoint compatibility remains documented separately; training-held-out status remains unverified.

## Replay

From `MAC-VO-main`, in `macvo:ms2-smoke` with AirStack at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_selective_risk --root /root/AirStack/ore-ms --output /path/to/fresh-run
python -m Scripts.Setup.validate_ms2_selective_risk --root /root/AirStack/ore-ms --run /path/to/fresh-run --output /path/to/fresh-validation.json
python -m Scripts.Setup.plot_ms2_selective_risk --run /path/to/fresh-run
```

Analysis requires original proxy reports and480 saved NPZs from EXP046/048/049; no raw imagery, model or GPU compute. Validation additionally checks restored historical result payloads against EXP043/044-complete/046/047/048/049/050 recovery manifests. Fresh output paths are mandatory. Runtime image SHA256 `8dc9fef8c6da18d5a05d7fc44f53b27c932ed67bfc42d2fccf60fd08cc0a29a8`; numerical work runs inside Docker. Startup driver diagnostics are retained separately from validation outcomes.

## Results at50% coverage

Each range is min / median / max of20 per-image ratios to random expected MSE. Below1 is better. Every primary image/modality case is below1 (160/160); the single-scan common-support sensitivity is also below1 in160/160 repeated views. These counts are descriptive, not independent statistical evidence.

|Window|Camera|Own-support risk ratio min / median / max|Common-support ratio min / median / max|
|---|---|---:|---:|
|night-opening|RGB|0.0447 / 0.0805 / 0.1510|0.0540 / 0.2301 / 0.6456|
|night-opening|THR|0.1041 / 0.1280 / 0.1503|0.2410 / 0.3072 / 0.3920|
|day-opening|RGB|0.1346 / 0.2446 / 0.9090|0.1238 / 0.3308 / 0.8757|
|day-opening|THR|0.3710 / 0.5485 / 0.7538|0.3525 / 0.5503 / 0.7517|
|day-middle|RGB|0.0005 / 0.5133 / 0.7863|0.0031 / 0.4626 / 0.7798|
|day-middle|THR|0.2605 / 0.4587 / 0.6889|0.1993 / 0.4128 / 0.7832|
|day-late|RGB|0.0983 / 0.5345 / 0.6852|0.1333 / 0.4820 / 0.9257|
|day-late|THR|0.0955 / 0.3198 / 0.6914|0.1443 / 0.4277 / 0.7998|

Median own-support MSE reductions are92%/87% RGB/thermal at night;76%/45% day opening;49%/54% day middle;47%/68% day late. This supports useful selective ranking relative to random retention on these labels. It does not justify numerical covariance calibration, a modality winner or fusion. Large reductions can be driven by removal of a high-loss tail: e.g. the day-middle RGB minimum ratio is0.00048, while its group median is0.5133. Preserve the per-image distributions rather than treating such a minimum as typical. No comparison across camera absolute risks is made.

![Fixed-coverage reference risk](../results/ms2-selective-risk/run-001/primary-risk-curves.png)

All3840 curve points, including oracle and reverse rankings and merged/filtered sensitivities, remain in the machine-readable report; the figure only shows single-scan median ratios. Lines connect evaluated coverages without claiming interpolation or optimal coverage.

## Validation and decision

All49,871 checks pass: independent cutoff masks reproduce all3840 points; per-image supports/source hashes, aggregate quantiles, oracle bounds and full-retention identities pass. Synthetic constant/tied uncertainties and permutation checks pass. All485 frozen inputs are unchanged and1,944 historical result hashes pass. Original NPZ hashes match their historical proxy reports. Numerical work used CPU inside the recorded Docker image; no model execution or defaults changed. An initial loop-based attempt was interrupted for performance before producing scientific outputs; the equivalent sorted-cutoff version completed and was independently checked.

Next useful control is whether covariance selection outperforms simple scene/depth alternatives at matched retained mass, before adopting a weighting rule. More frames alone do not resolve that comparison. No additional upload is needed for these existing-data controls; complementary CATS access remains pending.
