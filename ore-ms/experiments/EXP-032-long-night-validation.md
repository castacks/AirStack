# EXP-032: Longer night windows, timestamp gaps and tracking-state audit

Date: 2026-10-22 (America/New_York). Branch `ore_ms`, base `68517e81f` plus frozen source. **All three thermal100-frame attempts complete without lost-track pairs. RGB has99/97/99 lost-track pairs out of99, despite the middle and late runners returning execution-pass status. Their stored interpolation masks undercount original lost-track frames because the existing postprocessor clears boundary flags. Thermal uncertainty/disagreement association is positive spatially on the longer moving windows, while temporal association and larger-gap behavior vary. This does not calibrate physical reliability or justify fusion weights.**

## Fixed extension and original timing

Reuse `20251027/night_mac_salon_warehousefig8` from EXP-031. Before new images or outputs, freeze one100-group block nearest each chronological third midpoint, excluding all60 prior paired indices; earliest start breaks a tie. Both modality timestamps must strictly increase and remain within recorded SLAM temporal support. Selection is independent of texture, model uncertainty, error and proxy displacement.

Unlike EXP-031's short continuous blocks, this protocol explicitly retains every positive acquisition gap in each selected100-group block. No image resampling or gap filling is performed. These are sampled sequences from an export with missing paired groups, not uninterrupted10Hz clips. Preserve actual timestamps and report pairs≤150ms separately from larger gaps. The150ms boundary is inherited from the earlier eligibility rule and was fixed before outcomes; it is not fitted to results.

| Window | Paired start | Duration RGB / thermal (s) | Gaps >150ms | Largest gap (s) | Body SLAM displacement (m) |
|---|---:|---:|---:|---:|---:|
| early | 200 | 14.898 / 14.901 | 30 | 1.300 | 30.713 |
| middle | 700 | 9.900 / 9.900 | 0 | 0.100 | 32.459 |
| late | 1200 | 11.002 / 11.000 | 1 | 1.200 | 32.005 |


Selected camera timestamps do not repeat EXP-031 acquisitions. Source archive/index, camera metadata, original checkpoints, experimental MSA calibration, intensity processing and VO settings are unchanged. Retrieve1,800 selected source images using ZIP byte ranges. Four extra existing raw files share selected timestamps; each is byte-identical to its canonical copy. All1,804 members independently pass central-directory CRC/SHA checks;600 original-producer thermal image sides reproduce supplied processed images exactly. Prepared inputs have3,041 frozen hashes. Full8.08GB archive hashing remains unverified; no additional user upload was required for this experiment.

## Operational outcome and a reporting pitfall

Six100-frame attempts use the same GPU image as EXP-031 (`f429af4afa23`), seed0, FP32, synchronous optimizer and original configs/checkpoints. All have99 saved temporal prediction pairs. Raw reports and outputs remain intact; no alternate windows, retries, threshold changes or fusion fitting.

| Window/modality | Raw runner status | Graph observations | Pairs below existing10-point gate /99 | Stored interpolation flags /100 | Interpretation |
|---|---|---:|---:|---:|---|
| early-rgb | FAILED | 0 | 99 | unavailable | no matches; no trajectory |
| early-thermal | PASS_EXECUTION_ONLY | 14890 | 0 | 0 | sustained execution |
| middle-rgb | PASS_EXECUTION_ONLY | 54 | 97 | 94 | filled output; sparse/no optimization |
| middle-thermal | PASS_EXECUTION_ONLY | 17736 | 0 | 0 | sustained execution |
| late-rgb | PASS_EXECUTION_ONLY | 11 | 99 | 95 | filled output; sparse/no optimization |
| late-thermal | PASS_EXECUTION_ONLY | 17199 | 0 | 0 | sustained execution |


The existing runner checks finite normalized poses, exact timestamps and nonzero total graph observations. It does not require sustained optimization. MACVO's existing `min_num_point=10` gate determines whether a pair starts optimization; it is not a newly fitted quality threshold. Early RGB has0 observations total and fails. Middle RGB's54 total observations reach this gate on only2 pairs; late RGB's11 total observations never do. Thus a raw execution-pass status is insufficient evidence that the RGB trajectory is supported by visual tracking.

`MotionInterpolate` takes `frames.data["need_interp"][1:].bool()` and clears its first/last two entries before interpolation. With an already-boolean tensor this is an alias of stored flags, so frames1,2,98,99 may be cleared in the stored state. An independent PyTorch2.7.1 control reproduces99 original flags becoming95 stored flags. Saved raw observation counts and all per-pair `VOLostTrack` logs independently recover97 original lost-track frames in middle RGB versus94 stored flags, and99 versus95 late. The exact original source and final masks are frozen/audited. The postprocessor's interpolation also uses frame indices; these filled paths must not be interpreted as timestamp-aware visual measurements.

This audit changes interpretation and records original loss counts; it does **not** modify the baseline, original reports, postprocessor or trajectories. EXP-031's four complete attempts were specifically rechecked: each has zero pre-refinement lost pairs, so its previous operational-completion finding is unaffected. Other historical post-refinement interpolation counts should not be assumed to count every original tracking loss without the same audit.

## Independent image consistency and support

Reuse EXP-030's nominal target-ray covariance propagation and exact model–LK ray-angle difference, with unchanged independent forward/backward LK parameters. All594 temporal pairs and833,976 source-grid points are retained, including failures. Primary FB≤1px, tighter≤0.5px and primary+source texture std≥5 are unchanged. Raw temporal covariance is before floors and includes the uv cross term. K/MSA applicability, LK error, shared ambiguity and source-coordinate uncertainty still limit physical interpretation.

| Window/modality | LK-supported pairs /99 | Grid support % (pair min–max) | Nominal σ (mrad) | Disagreement (mrad) | Median spatial ρ (available pairs) | Temporal ρ |
|---|---:|---:|---:|---:|---:|---:|
| early-rgb | 16 | 0.00–0.11 | 183.469 | 160.418 | unavailable (0) | -0.688 |
| early-thermal | 99 | 12.10–97.72 | 2.453 | 0.774 | 0.407 (99) | 0.769 |
| middle-rgb | 26 | 0.00–0.22 | 171.856 | 116.381 | 0.150 (6) | 0.056 |
| middle-thermal | 99 | 39.68–74.31 | 5.662 | 1.678 | 0.397 (99) | -0.198 |
| late-rgb | 3 | 0.00–0.11 | 152.660 | 271.256 | unavailable (0) | -1.000 |
| late-thermal | 99 | 9.23–84.33 | 5.966 | 1.572 | 0.537 (99) | 0.223 |


σ is sqrt(median pair-median angular variance); disagreement and spatial ρ are medians of available per-pair statistics. Temporal ρ associates contemporaneous pair medians, not future errors. Missing/rank-degenerate cases remain unavailable. RGB support is0–2 points per pair early,0–4 middle and0–2 late; even the available middle spatial coefficient is based on tiny per-pair populations. Late RGB temporalρ−1 uses only three supported pairs. These are not useful cross-modal geometric-error labels and do not establish confident RGB performance.

Thermal has substantially broader LK support on every pair. Its three spatial coefficients0.407/0.397/0.537 are positive on these longer moving windows, unlike some low-image-motion EXP-030/031 windows. This is descriptive context dependence across selected windows, not proof of a transferable confidence calibration. Per-case temporal associations0.769/−0.198/0.223 differ markedly; a positive spatial ranking does not guarantee useful frame-level ranking.

| Thermal window/stratum | Pairs | Accepted grid support % | Nominal σ (mrad) | Difference (mrad) | Median spatial ρ | Temporal ρ |
|---|---:|---:|---:|---:|---:|---:|
| early-thermal gap_le150ms | 69 | 90.483 | 1.819 | 0.507 | 0.379 | 0.358 |
| early-thermal gap_gt150ms | 30 | 46.683 | 15.788 | 2.816 | 0.464 | 0.743 |
| middle-thermal gap_le150ms | 99 | 68.020 | 5.662 | 1.678 | 0.397 | -0.198 |
| middle-thermal gap_gt150ms | 0 | unavailable | unavailable | unavailable | unavailable | unavailable |
| late-thermal gap_le150ms | 98 | 72.254 | 5.918 | 1.566 | 0.537 | 0.199 |
| late-thermal gap_gt150ms | 1 | 9.226 | 81.910 | 33.334 | 0.314 | unavailable |


Gap-table support is the accepted/grid count across all pairs in each stratum; quantiles/coefficients retain equal per-pair aggregation. Early thermal support declines from90.48% on69 ordinary pairs to46.68% on30 larger-gap pairs, while median discrepancy increases from0.507 to2.816mrad. Its temporal coefficient declines from0.769 overall to0.358 within ordinary pairs, illustrating how gap/context mixture can affect a window statistic. This does not identify gap duration as a causal explanation: time, image motion and scene content change together.

Late has one1.2s gap: its thermal primary support is9.23%, discrepancy33.334mrad and nominalσ81.910mrad. The remaining98 pairs have72.25% support and1.566mrad disagreement. One pair is an event description, not an estimated failure rate. Middle has no larger-gap pairs; its corresponding stratum stays empty, not zero-error.

All833,976 covariance matrices are finite positive semidefinite.28,367 thermal and399,155 RGB observations have nominalσ>0.1rad; they remain flagged and retained. A broad covariance also limits first-order small-angle propagation, so values must not be treated as calibrated angular probabilities. Raw full-grid quantities on failed LK locations are audit data, not valid disagreement targets.

![Measurements at actual acquisition times](../results/long-night-validation/pair-timeline.png)

![Native first-image values, unchanged display scale](../results/long-night-validation/source-windows.png)

Native RGB images remain dark/weakly textured, with median whole-image grayscale std about0.72 in each window. This describes the supplied imagery, not exposure/illumination hardware or the cause of failure. Thermal retains visible scene structure. No brightening or alternate preprocessing is introduced.

## SLAM proxy context, including filled outputs

Retain first-pose anchoring with no best-fit alignment or scale. These are discrepancies against supplied recorded SLAM and nominal body/camera transforms, **not independent ground-truth accuracy**. Camera/body origins, experimental MSA geometry and reference provenance remain unresolved. RGB rows below are explicitly interpolation-dominated output comparisons, not successful VO evaluations. Early RGB has no saved validated trajectory and receives no score.

| Window/modality | Output kind | Estimated / camera SLAM net displacement (m) | Anchored translation / rotation RMSE (m / deg) |
|---|---|---:|---:|
| early-thermal | no interpolation | 26.454 / 30.702 | 2.161 / 1.214 |
| middle-rgb | interpolation-dominated | 4.431 / 32.479 | 18.102 / 99.262 |
| middle-thermal | no interpolation | 31.717 / 32.477 | 0.483 / 1.137 |
| late-rgb | interpolation-dominated | 0.000 / 31.794 | 17.991 / 57.553 |
| late-thermal | no interpolation | 26.743 / 31.765 | 3.602 / 1.505 |


## Verification and replay

25 focused tests pass:7 new selection/angular controls,13 prior long-window/preprocessing/calibration controls and5 GPU adapter/uncertainty controls. A separate direct tensor-alias control reproduces the postprocessor boundary behavior. Independently enumerate midpoint/exclusion/reference eligibility, verify all1,500 pairing constraints,1,804 ZIP CRC/SHA members and600 exact producer replays. Check4,158 scalar angles and4,158 finite-difference covariance propagations (maximum absolute variance difference1.91e-11rad²), six complete forward/backward LK grids,16,632 quantiles,4,752 ranks and54 gap strata. Five finite timestamp-correct outputs are numerically checked (including two interpolation-dominated RGB outputs); one failure is preserved. Audit all six pre/post tracking states against raw counts/logs and saved masks.

All3,041 preparation and1,229 analysis hashes pass. Prior EXP-031751/267, EXP-030729, EXP-028402 and EXP-02997 hashes remain unchanged. Source acquisition initially failed because a staging script named `select.py` shadowed Python's standard module; it was renamed before any selection or acquisition, with the initial script/log preserved. No scientific rule changed. The inherited EXP-030 progress print says19 pairs even on variable-length input;99 arrays per case and594 pairs are independently verified. No data are omitted because of that cosmetic log label.

Runtime remains the exact EXP-031 GPU image `sha256:f429af4afa234fc1e37db6867ed9ee9c708af852ae755fe7d6e8150f70136300` on RTX PRO5000 Blackwell48GB, PyTorch2.7.1+cu128; CPU preprocessing/analysis is `macvo:geometry`101a3943f3d1/OpenCV4.10.0.84. Exact package inventories, image IDs, source/config/checkpoint hashes, thread flags and commands are preserved. This controls the runtime relative to EXP-031; earlier historical GPU snapshots had different transitive versions.

Artifacts under `results/long-night-validation/`: selection and exact gaps, archive/member/raw-duplicate records, prepared datasets, all six attempted outputs,594 angular/LK grids, execution/image context, full summary/gap CSV and JSON, source/summary/timeline figures, independent controls, tests/logs and replay scripts under `staging/`. Newly selected original images remain under the shared source `data/tartandrive3/20251027/night_mac_salon_warehousefig8/`, with prior hashes unchanged. Box snapshot `ore-ms/backups/2026-10-22-exp-032` preserves new selected source bytes/preparation/attempts/analysis and incremental source bundle; redundant compressed range cache, checkpoints, credentials and full source ZIP are excluded. Prior model/calibration/runtime recovery remains required.

With ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`:

```bash
python -m Scripts.Setup.td3_long_night prepare --output /workspace/results/long-night-validation/new-prepared
python -m Scripts.Setup.td3_long_night execute --plan /workspace/results/long-night-validation/new-prepared/plan.json --case early --modality thermal --output /workspace/results/long-night-validation/new-vo/early
```

Repeat explicitly for each fixed case/modality using the saved GPU flags and scripts. CPU preparation/analysis and GPU execution use their recorded images separately. Restored selected source/metadata, model weights and original producer/MSA calibration are prerequisites.

## Decision and further data

Thermal operational support persists across these longer windows. Spatial uncertainty/disagreement ranking is positive here, while temporal associations and gap-related support differ. RGB's finite/interpolated outputs must not be mistaken for recovery. Keep baseline settings and weights; no fusion model or physical calibration claim. A bounded next software task is to preserve pre-refinement tracking diagnostics before postprocessing and report execution, tracking support and filled output separately, with a controlled replay to prove pose behavior unchanged.

No more upload is needed to reproduce or audit this experiment. **For the next numerical cross-modal error comparison, a different night recording with visible RGB scene structure would help.** The current archive mainly supplies an RGB unavailability condition. Request available20251027 folder names first, then choose one appropriate clip; do not request the whole date. More images alone do not resolve pending recording-specific reference timing/quality, sensor independence or physical camera/body provenance.
