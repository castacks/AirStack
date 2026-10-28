# EXP-064: Prospective frozen uncertainty-only selection across three captures

Date:2026-10-27 America/New_York. Branch `ore_ms`; starting commit `48129f270`.

**Finding: the declared consistent-improvement criterion is not met.** Report all three preselected recordings below, including strict wins,ties,losses and both target losses. Models,normalizers and thresholds were unchanged from EXP063; no new-recording labels entered fitting.

## Prospective declaration and source selection

After EXP063’s geometry counterexample, declare its simpler uncertainty-only control primary for a new prospective suite. This is an explicit post-EXP063 family choice, not a retrospective primary claim for that earlier test. Choose first lexical unused capture independently within the pinned official day/night/rainy test lists. Exclude all four previously examined captures. Each new capture contributes originalIDs000000–000019 with no quality/timing/outcome substitutions.

| Split label | Frozen recording | Available frames | Candidates | Both valid | RGB-only | Thermal-only | Neither |
|---|---|---:|---:|---:|---:|---:|---:|
| day | _2021-08-13-15-46-56 | 20/20 | 48270 | 48026 | 0 | 244 | 0 |
| night | _2021-08-13-22-03-03 | 20/20 | 24387 | 24387 | 0 | 0 | 0 |
| rainy | _2021-08-06-16-59-13 | 20/20 | 12838 | 11872 | 0 | 966 | 0 |

The first three development captures remain the sole training set (60frames/133,441both-valid pairs). EXP063’s new capture and these three new captures were not added to training. Two official test captures remain unexamined:day `_2021-08-13-16-31-10`,night `_2021-08-13-22-16-02`. These suite captures now count as examined. Split labels do not establish weather/illumination causality; checkpoint network-training membership remains unknown. This is three distinct captures within a shared collection/rig/reference pipeline, not three independent physical-error datasets.

Before new-pixel inspection the suite,per-record model artifacts and success criterion were frozen; the declaration was separately saved and byte-verified in Box `backups/2026-10-27-exp-064-declaration` before new scores. Suite SHA256 `9781971e743186d0d65f977b3a60d51882e531d1ef33840d1784e0be80435888`. Parent numerical-model artifact SHA256 `51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377`. Per-record artifact hashes bind sequence metadata while all coefficients/scalers/objectives/input hashes are exactly equal to the parent.

## Model and criterion

Use unchanged EXP062/063 constant,uncertainty-only,geometry-only and combined cost-sensitive linear selectors. Primary is uncertainty-only; others are controls. Same fixedridge0.01,equal-recording/frame training weights,midpoint mean-L2 signed error target with error-gap weights,training-only standardization and zero-margin mixture. No new inference-time reference range/errors enter numerical features,though support/correspondences remain reference-selected. Margins are not probabilities,expected risks or calibrated full covariances.

For clarity, the frozen uncertainty-only unstandardized margin is approximately

```text
margin = 0.727174964 -0.046312168 log(U_RGB) -0.042768438 log(U_thermal)
RGB if margin>0; thermal if margin<0; exact0 equal mixture.
```

It is a learned joint-magnitude selector, not simply choosing the lower uncertainty or a single variance-ratio rescaling. Both variance inputs already include predicted-depth geometry. The raw lower-variance selector,per-image camera-scale control and frozen EXP060 scalar multiplier remain explicit comparisons. Neither model-score calibration nor a physical cause follows from a favorable selection ratio.

Declared consistent added benefit requires **each** recording’s midpoint mean-distance median U-only risk ratio to be strictly below1 against **both fixed cameras and the frozen scalar-fit**. A tie/failure on any comparator/recording fails that criterion. MSE and both endpoint anchors are retained sensitivities,not alternative success definitions. Report per-image paired ratios and exact better/equal/worse counts; no pixel pooling or a pooled60-frame significance test. Missing frames would remain explicitly unavailable; full coverage is reported separately.

## Primary results

Entries are median paired per-image U-only/comparator ratios, followed by better/equal/worse frame counts. Below1 favors U-only.

| Recording | Loss | Always RGB | Always thermal | Frozen scalar-fit | Original learned | Camera-scale | Per-record primary criterion |
|---|---|---:|---:|---:|---:|---|
| day | mean_l2 | 0.1920 (20/0/0) | 1.0355 (9/0/11) | 0.7979 (20/0/0) | 1.4244 (0/0/20) | 1.3858 (0/0/20) | NOT MET |
| day | mse | 0.0024 (20/0/0) | 4.5913 (0/0/20) | 0.3249 (19/0/1) | 5.9466 (0/0/20) | 5.6739 (0/0/20) | NOT MET |
| night | mean_l2 | 1.0709 (0/0/20) | 0.5828 (20/0/0) | 0.9822 (11/0/9) | 0.6494 (20/0/0) | 0.7195 (20/0/0) | NOT MET |
| night | mse | 1.0535 (4/0/16) | 0.4021 (20/0/0) | 0.8932 (13/0/7) | 0.4161 (20/0/0) | 0.4977 (20/0/0) | NOT MET |
| rainy | mean_l2 | 0.8692 (18/0/2) | 0.6253 (19/0/1) | 0.9149 (18/1/1) | 0.7300 (20/0/0) | 0.7255 (20/0/0) | PASS |
| rainy | mse | 0.3416 (19/0/1) | 0.4235 (19/0/1) | 0.6639 (18/1/1) | 0.5172 (20/0/0) | 0.5364 (20/0/0) | PASS |

Suite criterion on available frames: **NOT MET**. Complete20-frame coverage in every recording: **True**. This is a descriptive criterion on three short windows; it is not a hypothesis-test p-value or proof of generalization.

Day fails against thermal:median mean-distance ratio1.0355 (9 better,11 worse) and squared ratio4.5913 (0 better,20 worse). Night fails against RGB:mean-distance1.0709 (0 better,20 worse),squared1.0535 (4 better,16 worse). Rain passes the predeclared per-record median criterion:mean-distance U/RGB0.8692,U/thermal0.6253,U/scalar0.9149. Its strict counts are18/0/2,19/0/1 and18/1/1 respectively; the positive result has counterexamples and a tie. Both endpoint anchors retain these criterion directions.

Rain’s median mean-distance reductions are13.1% versusRGB,37.5% versusthermal and8.5% versusscalar-fit. This is a conditional capture-specific gain. Day’s favorable comparison with RGB cannot erase its failure versusthermal or the raw/camera-scale controls; night’s favorable comparison withthermal cannot erase its RGB failure. The suite does not establish consistent transfer.

## All four frozen families

Entries below retain the same paired-ratio/better-equal-worse convention. No favorable family is selected after outcomes. Constant equals the development-selected camera (thermal) here; oracle and all cross-family comparisons remain in JSON.

| Recording | Loss | Family | RGB | Thermal | Scalar-fit | Random | Original learned | Camera-scale |
|---|---|---|---:|---:|---:|---:|---:|---:|
| day | mean_l2 | constant | 0.2057 (20/0/0) | 1.0000 (0/20/0) | 0.7794 (19/0/1) | 0.3412 (20/0/0) | 1.3401 (0/0/20) | 1.2724 (0/0/20) |
| day | mean_l2 | uncertainty | 0.1920 (20/0/0) | 1.0355 (9/0/11) | 0.7979 (20/0/0) | 0.3204 (20/0/0) | 1.4244 (0/0/20) | 1.3858 (0/0/20) |
| day | mean_l2 | geometry | 0.2600 (20/0/0) | 1.0960 (1/0/19) | 0.8639 (15/0/5) | 0.4241 (20/0/0) | 1.4899 (0/0/20) | 1.4378 (0/0/20) |
| day | mean_l2 | combined | 0.2495 (20/0/0) | 1.1194 (3/0/17) | 0.8707 (17/0/3) | 0.4139 (20/0/0) | 1.4823 (0/0/20) | 1.4304 (0/0/20) |
| day | mse | constant | 0.0008 (20/0/0) | 1.0000 (0/20/0) | 0.0906 (20/0/0) | 0.0015 (20/0/0) | 1.3088 (1/0/19) | 1.2926 (6/0/14) |
| day | mse | uncertainty | 0.0024 (20/0/0) | 4.5913 (0/0/20) | 0.3249 (19/0/1) | 0.0048 (20/0/0) | 5.9466 (0/0/20) | 5.6739 (0/0/20) |
| day | mse | geometry | 0.0055 (20/0/0) | 5.9556 (0/0/20) | 0.6167 (17/0/3) | 0.0111 (20/0/0) | 8.9971 (0/0/20) | 7.8059 (0/0/20) |
| day | mse | combined | 0.0049 (20/0/0) | 6.1835 (0/0/20) | 0.5723 (17/0/3) | 0.0097 (20/0/0) | 7.6788 (0/0/20) | 6.9715 (0/0/20) |
| night | mean_l2 | constant | 1.8799 (0/0/20) | 1.0000 (0/20/0) | 1.6292 (0/0/20) | 1.3055 (0/0/20) | 1.1050 (0/0/20) | 1.1928 (0/0/20) |
| night | mean_l2 | uncertainty | 1.0709 (0/0/20) | 0.5828 (20/0/0) | 0.9822 (11/0/9) | 0.7529 (20/0/0) | 0.6494 (20/0/0) | 0.7195 (20/0/0) |
| night | mean_l2 | geometry | 1.0803 (4/0/16) | 0.5736 (20/0/0) | 0.9708 (13/0/7) | 0.7566 (20/0/0) | 0.6450 (20/0/0) | 0.7135 (19/0/1) |
| night | mean_l2 | combined | 1.0952 (3/0/17) | 0.5725 (20/0/0) | 0.9645 (12/0/8) | 0.7574 (20/0/0) | 0.6455 (20/0/0) | 0.7165 (19/0/1) |
| night | mse | constant | 2.8239 (0/0/20) | 1.0000 (0/20/0) | 1.9667 (0/0/20) | 1.4769 (0/0/20) | 1.0343 (0/0/20) | 1.1470 (0/0/20) |
| night | mse | uncertainty | 1.0535 (4/0/16) | 0.4021 (20/0/0) | 0.8932 (13/0/7) | 0.5974 (20/0/0) | 0.4161 (20/0/0) | 0.4977 (20/0/0) |
| night | mse | geometry | 1.0228 (1/0/19) | 0.3605 (20/0/0) | 0.8832 (15/0/5) | 0.5330 (20/0/0) | 0.3729 (20/0/0) | 0.4558 (20/0/0) |
| night | mse | combined | 1.0215 (1/0/19) | 0.3588 (20/0/0) | 0.8725 (15/0/5) | 0.5305 (20/0/0) | 0.3711 (20/0/0) | 0.4526 (20/0/0) |
| rainy | mean_l2 | constant | 1.3566 (4/0/16) | 1.0000 (0/20/0) | 1.5263 (1/0/19) | 1.1513 (4/0/16) | 1.1609 (3/0/17) | 1.1678 (4/0/16) |
| rainy | mean_l2 | uncertainty | 0.8692 (18/0/2) | 0.6253 (19/0/1) | 0.9149 (18/1/1) | 0.6846 (19/0/1) | 0.7300 (20/0/0) | 0.7255 (20/0/0) |
| rainy | mean_l2 | geometry | 1.0300 (8/0/12) | 0.7534 (18/0/2) | 1.1809 (2/0/18) | 0.8647 (19/0/1) | 0.8714 (15/0/5) | 0.8765 (16/0/4) |
| rainy | mean_l2 | combined | 0.9952 (10/1/9) | 0.7166 (19/0/1) | 1.0822 (4/0/16) | 0.8198 (19/0/1) | 0.8549 (18/0/2) | 0.8537 (17/0/3) |
| rainy | mse | constant | 0.8202 (13/0/7) | 1.0000 (0/20/0) | 1.2655 (4/0/16) | 0.9009 (13/0/7) | 1.2505 (6/0/14) | 1.2462 (6/0/14) |
| rainy | mse | uncertainty | 0.3416 (19/0/1) | 0.4235 (19/0/1) | 0.6639 (18/1/1) | 0.3673 (19/0/1) | 0.5172 (20/0/0) | 0.5364 (20/0/0) |
| rainy | mse | geometry | 1.0041 (9/0/11) | 0.8708 (12/0/8) | 1.2086 (3/0/17) | 0.7999 (15/0/5) | 0.8947 (11/0/9) | 0.9102 (12/0/8) |
| rainy | mse | combined | 0.9210 (12/1/7) | 0.8597 (13/0/7) | 1.0794 (7/0/13) | 0.7507 (16/0/4) | 0.8743 (12/0/8) | 0.8976 (13/0/7) |

![Prospective primary comparisons](../results/ms2-frozen-suite/figures/frozen-suite.png)

## Endpoint sensitivities

Use the identical frozen choices under each endpoint anchor; no re-fitting or re-pairing.

| Recording | Anchor | Loss | U-only/RGB | U-only/thermal | U-only/scalar-fit |
|---|---|---|---:|---:|---:|
| day | rgb | mean_l2 | 0.1920 (20/0/0) | 1.0352 (9/0/11) | 0.7979 (20/0/0) |
| day | rgb | mse | 0.0024 (20/0/0) | 4.5984 (0/0/20) | 0.3249 (19/0/1) |
| day | thr | mean_l2 | 0.1921 (20/0/0) | 1.0359 (9/0/11) | 0.7980 (20/0/0) |
| day | thr | mse | 0.0024 (20/0/0) | 4.5841 (0/0/20) | 0.3249 (19/0/1) |
| night | rgb | mean_l2 | 1.0712 (0/0/20) | 0.5805 (20/0/0) | 0.9827 (11/0/9) |
| night | rgb | mse | 1.0555 (4/0/16) | 0.4021 (20/0/0) | 0.8937 (13/0/7) |
| night | thr | mean_l2 | 1.0706 (0/0/20) | 0.5851 (20/0/0) | 0.9818 (11/0/9) |
| night | thr | mse | 1.0515 (4/0/16) | 0.4021 (20/0/0) | 0.8927 (13/0/7) |
| rainy | rgb | mean_l2 | 0.8695 (18/0/2) | 0.6325 (19/0/1) | 0.9153 (18/1/1) |
| rainy | rgb | mse | 0.3416 (19/0/1) | 0.4211 (19/0/1) | 0.6659 (18/1/1) |
| rainy | thr | mean_l2 | 0.8689 (18/0/2) | 0.6183 (19/0/1) | 0.9147 (18/1/1) |
| rainy | thr | mse | 0.3421 (19/0/1) | 0.4265 (19/0/1) | 0.6624 (18/1/1) |

Full per-frame/all-family/all-anchor/both-loss risks,margins,choices,availability and depth-consistency diagnostics are preserved rather than using favorable subsets or anchors to redefine primary success. The original supplied-reference depth controls do not certify physical point identities/calibration/timing.

Across the suite there are85,495nominal candidate pairs:84,285both-valid,1,210thermal-only (244day,966rain),zeroRGB-only/neither. All60frames have some both-valid support. The additional thermal-only candidates remain explicit counters; their errors are not silently treated as two-camera comparisons. This tests conditional risk,not a complete availability policy.

## Acquisition, runtime and fixed inspections

All six source archives were selected before pixels and acquired with four concurrent64MiB exact HTTP206 range requests. Byte counts/Content-Range/signatures,full observed archive SHA and safe-member extraction are retained,with complete tar indices. Observed digests are local integrity evidence,not publisher-issued checksums. Bounded sample per capture:80camera images,40single-scan depth maps,calibration,four full camera timestamps/readmes; full transport archives stay local and are excluded from backup.

| Recording | Component | Bytes | Observed SHA256 |
|---|---|---:|---|
| day | sync_data | 25,167,647,752 | `80acc2ee2692f4b6ae4adfd8b7777f45630058d6a798bdcfcea307d97ce6e2bf` |
| day | proj_depth | 14,713,613,110 | `4f78d32b6c5108fdc30729c18d912a21f6120f11656204d88c1e21809467a4cb` |
| night | sync_data | 17,857,739,933 | `e0637bbd5d96a6e7f37c15719eb7b952f02212ae996838594b1685267e160c05` |
| night | proj_depth | 13,101,778,084 | `6c9630161e7b1044306650671a4a25a10a4f8befa8a610b72f46a6dbc3fedff2` |
| rainy | sync_data | 11,015,429,671 | `e90f6bd240773e885b6bc19b16d974022309f0088bbe9e5188a92b0c2dc2c82d` |
| rainy | proj_depth | 5,258,709,546 | `5c8f91fa10d173000cdfa311ec41dcab24cdf7d4138d45aeb29190a4e75efc6e` |

Native source encoding/geometry,own calibration,source IDs and recorded times are checked without image/calibration/clock fitting or exposure gates. RGBuint8 384×1224×3; thermaluint16 256×640; single-scan depthuint16/256,zero invalid. All240 new camera images are checked against the320 prior camera hashes and across suite recordings; any within-record duplicates are counted rather than silently removed. Checksum nonoverlap does not prove independent scenes or training splits.

Reuse EXP063 image `macvo:ms2-smoke` and original SHA-pinned RGB/thermal checkpoints,explicit thermal preprocessing,seed/dtype/TF32/config. The prior first-pair RGB/thermal bitwise parity bridge remains in EXP063; new source audits require runtime dictionary/effective config exact equality to that validated run. Each new capture has40saved stereo fields,two first-pair repeat calls and80independent archived-producer input-tensor checks. Source modules and original numeric helpers remain byte-pinned. No original-network training or VO.

Each capture’s fixed overlays000000/000009/000019 were manually inspected before metric scoring,with decisions/hashes saved. No model,frame,support,calibration or threshold changes follow those views. Nominal candidate support uses unchanged EXP057 mutual/frontmost/angular-radius rules. Dynamic objects,acquisition motion,FOV and sparse reference production remain limitations. Numerical model inputs are prediction-based but the correspondence/population is not a deployed label-free pipeline.

The rain RGB overlays include a dark moving diagonal foreground band,consistent with a wiper/foreground occlusion;the fixed inspection recorded it before scores. No occlusion mask or frame replacement was introduced. It limits a physical same-point interpretation without establishing that labels are bad or that it causes the measured gains/losses. Tiny positive disparities and large finite depth errors remain untrimmed,so squared loss can be strongly tail-sensitive;mean-distance remains the declared primary.

## Validation and implementation provenance

- Parent models/scalers and128 development-input hashes unchanged; no newrecording labels used in fitting or normalization.
- Adapter replays EXP063 scalar report exactly and311,878 feature/margin/choice values bitwise. Independent audit adapters also pass on the old fixture. Altered suite/per-record artifact guards reject mutations. These fixtures are implementation evidence,not extra prospective observations.
- Per-capture source/producer/frontend,independent raw geometry and independently reconstructed frozen feature/margin/risk audits pass before aggregation. All frame availability and strict ties are explicit.
- Summary audit independently recomputes every per-record/all-family/anchor/loss ratio statistic,tie count,source overlap and declared criterion from per-image reports. Audit counts repeat dependent values,not independent statistical samples.

Adapters bind only the declared recording artifact guard and primary metadata while reusing EXP063 arithmetic. The new independent audits preserve its check loops with recording/hash/primary guards parameterized; runtime/config checks are added. Neither numerical model is refit. Source acquisition/extraction/pipeline scripts and pre-outcome declaration are preserved with the recovery.

Host analysis Python3.12.3/NumPy2.2.6/SciPy1.15.3/OpenCV4.10.0.84; frontend runtime/config versions unchanged from EXP063; plotMatplotlib3.10.7. New source/frontend audit needs original checkpoint files for SHA validation; saved-array geometry/frozen/summary replay needs no GPU. Use canonical restored paths and fresh output paths.

```bash
python MAC-VO-main/Scripts/Setup/ms2_frozen_suite.py evaluate --root /root/AirStack/ore-ms --suite /root/AirStack/ore-ms/results/ms2-frozen-suite --condition day --output /path/to/fresh-day-evaluation
python MAC-VO-main/Scripts/Setup/validate_ms2_frozen_suite.py evaluation --root /root/AirStack/ore-ms --suite /root/AirStack/ore-ms/results/ms2-frozen-suite --condition day --output /path/to/fresh-day-validation.json
python MAC-VO-main/Scripts/Setup/summarize_ms2_frozen_suite.py --root /root/AirStack/ore-ms --suite /root/AirStack/ore-ms/results/ms2-frozen-suite --output /path/to/fresh-summary
```

Audits/aggregation intentionally read the canonical `frozen-evaluation` directories; evaluating to fresh paths does not redirect them automatically. The unchanged original support/raw-geometry validator takes explicit per-capture dataset/support/metric/integrity paths.

## Research decision

Keep current defaults and preserve these prospective results together with EXP062/063 counterexamples. Any favorable control or individual capture does not validate general calibrated confidence,physical relative-error prediction,network-training-held-out performance or VO/fusion improvement. The new suite is now examined; do not relabel it untouched for a later method.

Before additional feature/scale fitting, consolidate which measurements are supported: provided-reference conditional ranking/selection versus physical calibration and transfer. The remaining two untouched captures are limited evaluation resources. Independent reference data such as an accessible CATS sample and checkpoint training provenance would strengthen specific physical/unseen-network claims; neither is assumed available. No user upload or author message was needed for this suite.

Recovery target `ore-ms/backups/2026-10-27-exp-064` includes all bounded inputs/frontend/support/metric/evaluation files,frozen declarations,validations,plots,notebook and incremental Git bundle. Parent experiment/helper/data prerequisites are EXP063 and original development EXP057/058/060. Full transport archives,original checkpoint files and private credentials are excluded.
