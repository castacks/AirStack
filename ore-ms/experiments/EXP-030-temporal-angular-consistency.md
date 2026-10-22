# EXP-030: Temporal uncertainty in nominal angular units

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `480f2d9bf` plus frozen diagnostic source. **Converting temporal matching covariance from pixels to nominal ray units yields positive spatial uncertainty/disagreement associations in six RGB and ten of twelve thermal windows. The late-night thermal association remains negative (median spatial Spearman −0.187), despite 94.0–97.0% independent tracker support. This is descriptive algorithm agreement, not calibrated physical error or a transferable fusion weight.**

## Question and fixed protocol

While recording-source replies are pending, test whether raw temporal matching uncertainty accompanies independent image-tracking disagreement without choosing either recorded odometry as truth. This differs from EXP-029's stereo variance: use saved temporal `grid_raw_flow_cov` before covariance floors, with channels uu,vv,uv in px², as recorded by `td3_uncertainty.py`. No new inference, camera correction, odometry reference or GPU operation.

Inherit all 18 EXP-029 cases: twelve MSA-thermal and six RGB day controls across three recordings. Use first 20 left images and 19 consecutive saved temporal pairs from each, including only the first 19 pairs of the 100-frame EXP-026 windows. This gives 342 pairs, 360 case-image entries and 435,024 source-grid observations. The native 16px grids with 32px border contain 1,008 thermal or 1,800 RGB points. These are previously selected windows, not unseen or random validation data. Night has no matched RGB control here.

Images are decoded directly to grayscale. Reuse EXP-027's independent forward/backward pyramidal Lucas–Kanade (LK) implementation unchanged: 21×21 window, maxLevel3, 30 iterations/0.01 epsilon, minEigThreshold1e-4, no model initialization. Primary support requires forward/reverse status, finite endpoint/backward coordinates, target inside the 32px border and forward/backward error ≤1px. Sensitivities use ≤0.5px, or primary with source 21×21 intensity standard deviation ≥5. Failures remain saved at every grid point. Model covariance does not select LK support. Unfiltered `all` summaries retain even failed LK outputs for audit; their disagreements are **not validated measurements** and are excluded from the findings below.

For endpoint p=(u,v), r=K⁻¹[u,v,1]ᵀ and n=r/‖r‖. Evaluate the Jacobian at the **model target endpoint**:

`J = (I − nnᵀ) K⁻¹[:, :2] / ‖r‖`

Construct the full 2×2 covariance C from uu,vv,uv. The scalar `trace(J C Jᵀ)` is a first-order nominal ray variance in rad². Model–LK angular discrepancy is exact `atan2(‖n_model × n_LK‖, n_model · n_LK)` in radians. Source-to-model/source-to-LK angular motion, source texture and native pixel quantities are retained as context. Finite positive-semidefinite covariance is checked without clipping or flooring. All 435,024 supplied covariance matrices pass. 14,535 observations have nominal σ>0.1rad (4,432 within primary LK support); they remain flagged and retained. The local propagation approximation may be poor for these broad distributions; they do not define a calibrated angular probability model.

No camera/body transform is needed for this camera-ray scalar. Physical interpretation still depends on the supplied pinhole K, especially the experimental thermal calibration. LK noise, source-coordinate uncertainty, rectification/model error and shared image ambiguities are omitted. An algorithm agreeing with another algorithm does not prove correspondence correctness.

## Results

Each table entry aggregates 19 pairs: σ is the square root of the median of the pair-median angular variances, disagreement is the median of pair-median angles, spatial ρ is the median of the 19 per-pair Spearman coefficients, and temporal ρ relates the 19 pair-median variances to pair-median discrepancies. Temporal association is contemporaneous, not predictive; pixels and adjacent pairs are dependent. No significance or pooled-pixel independence claim.

| Saved window | LK support % (pair min–max) | Nominal σ (mrad) | Disagreement (mrad) | Spatial ρ | Temporal ρ |
|---|---:|---:|---:|---:|---:|
| msa-replay-day-early-thermal | 51.7–71.6 | 0.970 | 1.574 | 0.432 | 0.109 |
| msa-replay-day-late-thermal | 61.8–71.2 | 3.478 | 1.407 | 0.572 | -0.160 |
| msa-replay-day-original-thermal | 57.5–70.9 | 1.524 | 1.949 | 0.325 | 0.214 |
| msa-replay-night-early-thermal | 85.2–93.0 | 2.328 | 0.977 | 0.515 | 0.233 |
| msa-replay-night-late-thermal | 94.0–97.0 | 1.373 | 0.237 | -0.187 | 0.337 |
| msa-replay-night-middle-thermal | 60.6–87.1 | 7.083 | 1.673 | 0.356 | 0.316 |
| salon-so-day-early-rgb | 48.6–52.3 | 1.394 | 1.112 | 0.430 | 0.165 |
| salon-so-day-early-thermal | 51.1–68.0 | 4.510 | 1.894 | 0.286 | 0.125 |
| salon-so-day-late-rgb | 53.2–59.4 | 0.885 | 0.856 | 0.415 | 0.496 |
| salon-so-day-late-thermal | 64.1–68.8 | 1.703 | 1.339 | 0.563 | -0.298 |
| salon-so-day-middle-rgb | 57.6–65.6 | 0.573 | 0.654 | 0.594 | 0.077 |
| salon-so-day-middle-thermal | 56.2–63.9 | 0.919 | 1.243 | 0.529 | 0.514 |
| long-salon-early-rgb | 89.8–91.6 | 0.058 | 0.077 | 0.361 | 0.698 |
| long-salon-early-thermal | 92.6–96.0 | 1.845 | 0.230 | -0.043 | 0.846 |
| long-salon-late-rgb | 55.4–68.3 | 0.507 | 0.590 | 0.519 | 0.554 |
| long-salon-late-thermal | 64.4–71.0 | 1.102 | 1.432 | 0.476 | 0.214 |
| long-salon-middle-rgb | 31.0–58.2 | 2.342 | 1.579 | 0.439 | 0.635 |
| long-salon-middle-thermal | 19.3–55.3 | 2.844 | 2.205 | 0.362 | 0.168 |


Thermal primary median spatial ρ ranges −0.187 to 0.572, versus RGB 0.361–0.594. With tighter forward/backward checking the ranges are −0.187–0.541 and 0.359–0.559; with texture support they are −0.149–0.575 and 0.354–0.604. The two negative thermal windows are late night and EXP-026 early (−0.043 primary), both previously observed low-image-motion windows. This identifies a limitation of a universal monotonic ranking here, not its cause. It does not establish uncertainty miscalibration because disagreement contains LK error and shared ambiguities.

Temporal ρ ranges −0.298–0.846 for thermal and 0.077–0.698 for RGB. Spatial and temporal associations answer different questions; neither ensures one modality's reported variance ranks its physical error against the other. Native trace and native endpoint-difference associations are retained alongside angular quantities in `analysis.json`/`summary.csv`.

Late-night thermal has nominal σ1.373mrad versus model–LK disagreement0.237mrad, with primary median spatial ρ−0.187 and temporal ρ0.337. Those two magnitudes are descriptive, not a variance/error calibration ratio: unknown LK noise, covariance modeling assumptions, selection and the nonlinear ray mapping prevent interpreting them as calibrated confidence or coverage. Different camera fields of view, scene regions, resolution and texture remain after pixel-to-angle conversion. No physically matched cross-modal reliability ordering is claimed.

![Nominal angular uncertainty and independent tracking](../results/temporal-angular-consistency/temporal-angular-consistency.png)

## Verification and replay

Seven focused tests pass (four new math tests and three prior LK controls): analytic center-pixel propagation/exact angle, finite-difference Jacobian with anisotropic cross covariance, image/intrinsics/covariance scaling invariance, and invalid covariance handling. Independent scalar checks cover 2,394 saved ray angles and 2,394 finite-difference covariance propagations, maximum absolute variance difference7.57e-12rad². All 95 inherited EXP-027 LK endpoint/backward/texture arrays and primary/tight masks reproduce exactly. All 729 newly frozen image/prediction/report/helper/test hashes and all 402 EXP-028 scientific hashes pass after analysis. The initial pytest invocation inherited unavailable jaxtyping flags; the standard geometry-runtime `-o addopts=` invocation passes, with both logs preserved. No scientific parameter or data change was needed.

Run with ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`, CPU `macvo:geometry` (image sha256:101a3943f3d10fbd85611e0401364e94e343f0936d7cae535a396138987cdbb1), one OpenCV thread/seed0:

```bash
python -m Scripts.Setup.td3_temporal_angular prepare --output /workspace/results/temporal-angular-consistency/new-run
python -m Scripts.Setup.td3_temporal_angular run --output /workspace/results/temporal-angular-consistency/new-run
python -m pytest -o addopts= Scripts/UnitTest/test_td3_temporal_angular.py Scripts/UnitTest/test_td3_low_motion.py -q
```

Restore EXP-023/025/026 prepared images, reports and temporal saved outputs first; the independent repeatability control additionally requires EXP-027 arrays. Saved-output replay needs neither checkpoint weights nor GPU inference. Frozen plan, all 342 full-grid arrays, quantile/association summaries, CSV, figure, independent validation scripts/logs and exact runtime inventory are under `results/temporal-angular-consistency/`. Box snapshot `ore-ms/backups/2026-10-21-exp-030` preserves these new artifacts and incremental source bundle, excluding source datasets, weights and credentials; prior recovery snapshots remain prerequisites.

## Decision

Angular propagation is a useful common nominal unit and passes geometry controls. It does not resolve camera provenance, physical reference accuracy, shared matching errors or selection differences. The mixed thermal associations persist under fixed support sensitivities. Keep the MSA experimental baseline and all VO/weighting defaults. Do not train an adaptive fusion rule against these tracker discrepancies. Recording-specific producer/quality/timing and physical camera/body provenance remain the external gates; a subsequent bounded check could compare uncertainty associations within fixed motion/texture strata rather than pool those nuisance conditions.
