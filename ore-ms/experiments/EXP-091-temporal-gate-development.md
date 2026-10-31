# EXP091: fixed temporal quality-gate development comparison

The combined primary and all three prespecified controls failed the consistent-improvement development diagnostic. All 16 fits converged and fallback coverage was preserved. No candidate was adopted.

## Declaration and scope

The original pre-fit declaration is preserved byte-for-byte at `results/temporal-gate-development/run-001/declaration.md`; its SHA256 is recorded in `protocol.json`. This report expands that declaration for readability and adds results. Parent commit: `d9bfda5ce43b8d3aad785220c558ce0a2e2513cb`.

All 274 existing EXP089/EXP090 controller-prefix decisions are retained. The common 42-coordinate complete-feature/both-supported mask supplies 161 fitting rows: 96 from `td3/20251027/0_salon_so_day`, 48 from `td3/20251027/1_salon_mac_cov_day`, and 17 from `td3/20251027/night_mac_salon_warehousefig8`. The two night windows form one recording. Numerical X contains only current/two-lag RGB/thermal event-health features. Recording IDs route development folds and normalize training costs; they are not gate coordinates. Labels reside in a separate file.

All three recordings had informed development. Recording exclusion here is a retrospective diagnostic, not an untouched prospective test or a verified checkpoint-training holdout. Losses are nominal-body constant-twist interval projections against recorded SLAM, with existing unverified physical/frame/clock/deadline flags. No reserve, neural inference, physical/live validation or additional scientific payload was used.

## Fixed method

Combined 42 coordinates remains primary; uncertainty-only 12, health-only 30 and current-only 14 are controls. Every family uses the same eligibility mask. Current-only uses seven coordinates from each camera, not the first fourteen coordinates. Each family has three whole-recording-excluded fits plus one all-development artifact: sixteen fits total. All-development models are excluded from reported predictions.

Training-only unweighted population mean/std (ddof 0), true constant-column scale 1; binary target 1 iff RGB has strictly lower translation loss. Cost is the absolute squared translation-loss gap, normalized to sum 1 within each positive-cost training recording and then equally across those recordings. Zero-cost ties remain in coverage. Logistic objective has fixed ridge 0.01/2 on slopes and an unpenalized intercept. Margins are cost-sensitive scores, not calibrated probabilities.

Float64 L-BFGS-B starts at zero with maxiter 2000, gtol 1e-9, ftol 0, maxls 20, maxcor 10 and maxfun 100000. Native success, finite parameters/objective and analytic gradient infinity norm at most 1e-9 are required. One weighted class, no positive cost or solver failure produces a saved failed fit and fallback; there is no alternative fit or threshold. No parameter, feature or optimizer search occurred.

With complete eligible inputs, margin >= 0 chooses RGB; otherwise thermal. Incomplete/unsupported inputs use the original raw model-action fallback. There is no new abstention. Features/labels/source/protocol hashes were frozen before fitting; predictions were saved in a separate label-free phase before scoring. The primary diagnostic requires lower translation RMSE than each fixed camera and raw fallback on every recording on each exact matched population, with fallback coverage maintained. Empty comparisons cannot pass.

## Results

All sixteen fits passed the fixed acceptance criterion. Gradient infinity norms range from 1.44e-10 to 9.46e-10. No failed or alternate real-data fits occurred.

Translation proxy RMSE in metres; n is that method’s available population. Different n values are conditional summaries and must not be compared as matched results.

| Method | Day 0 RMSE (n) | Day 1 RMSE (n) | Night RMSE (n) |
|---|---:|---:|---:|
| rgb_only | 0.031459 (98) | 0.034459 (54) | 0.230324 (19) |
| thermal_only | 0.079808 (98) | 0.041846 (54) | 0.039657 (116) |
| raw_advice | 0.031459 (98) | 0.034459 (54) | 0.101162 (116) |
| history | 0.031505 (97) | 0.035326 (51) | 0.039978 (114) |
| combined | 0.055631 (98) | 0.034604 (54) | 0.101157 (116) |
| uncertainty | 0.048465 (98) | 0.034459 (54) | 0.101160 (116) |
| health | 0.047978 (98) | 0.034459 (54) | 0.101162 (116) |
| current | 0.071365 (98) | 0.034459 (54) | 0.101155 (116) |

All four gates, raw fallback and thermal-only cover 268/274 decisions; RGB-only covers 171 and history 262. Gate availability equals raw fallback on every individual decision, not only in aggregate. The full recording populations are 99, 57 and 118 decisions.

Exact matched comparisons for the combined primary:

| Recording | Comparator | Matched n | Comparator RMSE (m) | Combined RMSE (m) | Lower? |
|---|---|---:|---:|---:|---|
| Day 0 | rgb_only | 98 | 0.031459 | 0.055631 | no |
| Day 0 | thermal_only | 98 | 0.079808 | 0.055631 | yes |
| Day 0 | raw_advice | 98 | 0.031459 | 0.055631 | no |
| Day 1 | rgb_only | 54 | 0.034459 | 0.034604 | no |
| Day 1 | thermal_only | 54 | 0.041846 | 0.034604 | yes |
| Day 1 | raw_advice | 54 | 0.034459 | 0.034604 | no |
| Night | rgb_only | 19 | 0.230324 | 0.230310 | yes |
| Night | thermal_only | 116 | 0.039657 | 0.101157 | no |
| Night | raw_advice | 116 | 0.101162 | 0.101157 | yes |

The combined model worsens both day recordings versus RGB/raw. Its very small night improvement over raw does not beat thermal. Every control also fails the same diagnostic; no control becomes a replacement primary. All 28 pairwise comparisons for each of three recordings, exact sample IDs, translation/rotation summaries and descriptive equal-recording means are retained in `scores.json`. Rotation is reported separately and was not used as the fitting target.

The previously identified dominant night event, `EXP089/middle/96`, remains selected as RGB by all four gates. Its RGB translation proxy loss is 1.003081 m versus thermal 0.040678 m; the fixed history rule selects thermal there. This is descriptive evidence on already-exposed data. No row was removed, clipped or relabelled. The result does not establish physical failure cause or general superiority of the history rule.

## Validation and artifacts

200 meaningful tests passed before fitting, covering causal feature ordering, no feature/prediction file reads, missing inputs, per-record cost balancing, finite-difference gradients, true constant columns, zero-cost/one-class failures, training-only normalization, excluded-label/feature mutation and rejection of all-development model substitution. Existing recorder, history, prefix projection and motion admission/chain suites passed with the new gate tests.

The separate standard-library auditor imports no feature, fitting, inference or scoring implementation. It independently reconstructs 11,004 coordinates from exact observed prefixes, checks all 161 eligible bindings, all 16 training masks/normalizers/weights/objectives/stationarity conditions, all 274 predictions, coverage, label-to-action losses and 84 matched-comparison groups. Fifteen semantic corruptions are rejected after bypassing hash checks, including a genuinely excluded training ID, parameters, labels, margins, coverage and missing populations. JSON key reordering passes. Audit status: `PASS_INDEPENDENT_DEVELOPMENT_AUDIT`.

Artifacts: `results/temporal-gate-development/run-001/{declaration.md,protocol.json,features.json,labels.json,models/,models-lock.json,predictions.json,prediction-proof.json,scores.json,validation.json}`. Reproduction commands are in `MAC-VO-main/Scripts/Setup/README.md`. Notebook `008-temporal-gate-development` preserves test/execution/audit logs and runtime metadata. CPU runs used image `sha256:af124653ef9cd76660bb8a91f5ae609d6488d41fc78269fe03b3f484c5d35fd1`, NumPy 2.2.6, SciPy 1.17.1, one BLAS/OMP thread.

No production defaults changed. Next work should synthesize the fixed history-rule and learned-gate evidence and its proxy limitations before proposing another fit or test assignment. The two reported MS2 reserves remain unopened and unassigned; checkpoint training membership and recording-specific physical provenance remain unresolved.

Recovery: `ore_ms:ore-ms/backups/2026-10-31-exp-091`, incremental Git bundle based on `d9bfda5ce`, source metadata inputs, all new outputs, notebook and member-level SHA256 manifest. No checkpoints, images, credentials or runtime image binary are needed for this CPU comparison.
