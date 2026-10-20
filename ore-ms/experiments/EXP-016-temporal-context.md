# EXP-016: Within-window uncertainty, context and temporal stability

Date: 2026-10-20T17:57:14.492678+00:00 (UTC). Branch `ore_ms`; base commit `3ac07f047` plus plan-frozen working source. **Completed; no consistent additional temporal signal established.**

## Result and decision

The large uncertainty differences between the three night windows in EXP-014/015 do not translate into a consistent uncertainty/error relationship over time within each window. Context-adjusted uncertainty correlations have mixed signs and some change sign under inbound filtering. This is insufficient support for a general temporal reliability score or adaptive fusion rule. Keep existing VO defaults and keep characterization separate from calibration claims.

There is limited positive evidence: early RGB reprojection has adjusted rho0.414 (0.436 with time), while late thermal epipolar has0.290 (0.249 with time). Neither pattern repeats across all windows/targets. Late RGB flow magnitude has raw rho0.981 with reprojection and0.921 with epipolar residual, versus uncertainty raw0.211/0.130 and context-adjusted0.120/0.100. Flow magnitude shares prediction errors with these residuals; this is a diagnostic comparator, not proof of an independent or superior reliability model.

| Camera/window | Target | Raw ρ | Flow ρ | Context-adjusted ρ | + time ρ | Difference ρ | First9 / last10 ρ |
|---|---|---:|---:|---:|---:|---:|---:|
| rgb/early | Reprojection | 0.060 | -0.188 | 0.414 | 0.436 | 0.187 | 0.133 / 0.673 |
| rgb/early | Epipolar | 0.451 | 0.270 | 0.093 | 0.093 | 0.313 | -0.017 / 0.588 |
| thermal/early | Reprojection | -0.279 | 0.658 | -0.195 | -0.133 | -0.498 | -0.483 / -0.261 |
| thermal/early | Epipolar | -0.384 | 0.586 | -0.343 | 0.068 | -0.556 | -0.500 / -0.297 |
| rgb/middle | Reprojection | 0.032 | 0.140 | 0.047 | 0.096 | 0.143 | -0.283 / 0.224 |
| rgb/middle | Epipolar | 0.286 | 0.853 | -0.076 | 0.054 | 0.127 | 0.483 / 0.030 |
| thermal/middle | Reprojection | 0.123 | 0.286 | -0.131 | -0.117 | 0.189 | 0.283 / 0.042 |
| thermal/middle | Epipolar | 0.226 | 0.356 | -0.126 | -0.111 | 0.348 | 0.367 / 0.321 |
| rgb/late | Reprojection | 0.211 | 0.981 | 0.120 | 0.110 | 0.040 | -0.083 / 0.248 |
| rgb/late | Epipolar | 0.130 | 0.921 | 0.100 | 0.138 | 0.020 | -0.217 / 0.285 |
| thermal/late | Reprojection | 0.114 | 0.186 | -0.333 | -0.321 | 0.096 | 0.583 / -0.236 |
| thermal/late | Epipolar | 0.282 | -0.030 | 0.290 | 0.248 | 0.150 | -0.083 / 0.406 |

Each row uses the 19 consecutive pair medians in one20-frame case, covering about1.90s. No windows are pooled for these correlations. Targets are forward reprojection norm and square-root Sampson distance, respectively. These are temporal associations among per-pair medians, distinct from EXP-015's within-pair associations across image points. In particular, thermal early's negative temporal correlation does not contradict a positive within-image spatial correlation.

## Protocol fixed before evaluation

Use all114 EXP-015 fixed-grid files, including failed late RGB; no retained-point substitution, new inference, new preprocessing, scale fitting, alignment or VO changes. Freeze all750 inherited/new input/source/literature hashes in `plan.json`. Primary population is all grid points. Inbound-only sensitivity is separate. Do not require target depth for epipolar residuals. Original depth/visibility/degeneracy rules remain those of EXP-015.

For each pair compute finite medians of raw covariance trace, predicted flow norm, inverse positive source forward depth, reprojection norm and Sampson residual, plus reference camera translation norm and rotation angle. Preserve every count. Primary partial Spearman uses ranks of uncertainty and error, both projected on intercept and ranked translation/rotation/flow/inverse-depth via least squares, followed by Pearson correlation of the residual ranks. This residualization is a descriptive nuisance regression, not a sensor scale correction or trained predictor. Secondary adjustment adds ranked pair index. The design ranks are5/6, leaving14/13 residual dimensions, respectively. All24 population/target summaries have19 finite rows and defined adjusted correlations. Statistical significance/independent-sample claims are not made.

Temporal sensitivity: rank correlation of raw adjacent first differences (18 per case; never bridge missing pairs), and frozen chronological first9/last10 splits with raw Spearman. Reranking residuals is not used. No sign, split point, residual target or context variable was selected after seeing the outcomes. The analysis code and test source are frozen in the plan.

## Repeatability and sensitivity

Five of12 all-grid raw associations change sign between first9 and last10 pairs. Early RGB epipolar goes from-0.017 to0.588; late thermal reprojection goes from0.583 to-0.236. These short segments have limited precision and temporal dependence, so the split diagnoses instability without establishing its cause.

Inbound-only context-adjusted reprojection correlations are RGB0.440/0.119/-0.117 and thermal-0.091/0.105/-0.366 (early/middle/late). Compare all-grid RGB0.414/0.047/0.120 and thermal-0.195/-0.131/-0.333. Late RGB and middle thermal change sign under visibility filtering. No common monotonic signal is established. Full inbound epipolar/time-adjusted results are retained in cases.csv/analysis.json, including signs that do not support the hypothesis.

## Papers and target definitions

The focused reading of [MAC-VO](https://arxiv.org/html/2409.09479v2) and the official [MACThermal thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf) is documented, with exact sections and source hash, in [literature notes](../docs/research/literature-2026-10-20.md). Their matching-uncertainty objectives motivate keeping image-space covariance, derived spatial covariance and geometric proxies distinct. Our context-control protocol is our own diagnostic, not a paper reproduction. The published preprocessing recipe supplies a concrete provenance audit target; it does not verify our supplied processed topic or checkpoint split. No MSO/MAC-I² novelty conclusion is asserted.

## Validation and artifacts

- 60 focused/regression tests pass, including nuisance-only, retained signal, rank-deficient design, constants, missing/insufficient rows, chronological splits and nonbridging first differences.
- All750 frozen hashes pass before/after.114 source pairs yield228 population rows,24 target summaries and12 arrays with medians, context, masks and rank residuals.
- A separate inverse-correlation-matrix computation matches all24 adjusted correlations within 9e-16. Independent source-median/count checks pass. All160056 source-grid observations have finite relevant scalar inputs; inbound counts remain explicit.
- PNG/PDF timeline and association figures were visually inspected. Full output: `results/temporal-context/run-001/`.
- CPU image: `sha256:bf5d89c63af6c867c49aeee4db2bb4ef1486c1a65ac026521de35fb8495a1cc2`; exact test command/image/date in staging/test-execution.json. Reproduction uses `python -m Scripts.Setup.td3_temporal_context prepare/run`; see setup README.

## Limits and next gate

Recorded SLAM is a proxy, dynamic TF is the same reference, calibration retains known thermal stereo inconsistency, and original preprocessing/checkpoint split provenance is unresolved. Predicted depth/flow are statistically coupled to both uncertainty and residuals, so conditioning can remove signal or introduce bias; it is not causal adjustment. First differences amplify noise; medians can hide spatial effects. These six cases are only three synchronized scene windows, each about1.9s, with no independent reference or out-of-recording validation. The checks do not establish forecasting, calibrated probabilities, cross-camera covariance comparability or absence of useful uncertainty information elsewhere.

Before training any relative-reliability or fusion model, audit the processed thermal converter against the thesis recipe and verify signed stereo geometry. Then use longer independent recordings and an independent reference to evaluate target-aligned error/calibration. Existing data support a coarse RGB degradation warning; stronger reliability claims remain open.

Snapshot: Box `backups/2026-10-20-exp-016`, with prior EXP-013/014/015 snapshots as prerequisites. Copyrighted full thesis PDF/text are locally ignored and excluded from redistribution; retrieve the official source using the metadata/hash.
