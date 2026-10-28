# EXP-066: disparity residuals and inverse-depth amplification

Date: 2026-10-27 America/New_York. Starting commit db453e02d, branch ore_ms.

A small RGB disparity tail dominates the large depth error in EXP064 day. Uncertainty ranking changes with the chosen error domain, and positive whole-frame metric ranking does not guarantee positive ranking within fixed ranges. These are descriptive results on already examined supplied-reference samples; they neither explain physical causes nor repair cross-camera transfer.

## Availability and fixed protocol

Four complete local raw frontend populations exist: EXP063 rain `_2021-08-06-16-45-28`, EXP064 day `_2021-08-13-15-46-56`, night `_2021-08-13-22-03-03`, rainy `_2021-08-06-16-59-13`. Each has20 original IDs000000–000019 and both cameras' saved fields. Original day has only the first raw pair per camera locally; original rain/night have no complete local raw fields. Scope was fixed by availability before scoring. This experiment does not represent all seven captures or additional prospective evidence.

Keep all80 frames and106562 both-valid pairs (213124 camera observations), no clipping, fitting, new inference, downloads or changes to sample support/defaults. The two final official test recordings remain untouched. Primary reference for each camera is its own supplied endpoint, which differs intentionally from the common midpoint target of cross-camera selection experiments. Transform thermal endpoints back to their native camera. Confirm the endpoint lies on the native pixel ray; verify reconstructed XYZ and propagated variance against saved metric arrays.

Reference-implied disparity is `d_ref = fx*B/Z_ref`, predicted disparity `d = -flow_u`, and signed residual `delta = d-d_ref`. It is implied from supplied projected depth, not independently annotated correspondence truth. For positive depths/disparities, the exact algebra is

`Z_pred - Z_ref = -fx*B*delta/(d*d_ref)`.

The reference-depth first-order finite-error approximation is `-fx*B*delta/d_ref²`. Exact divided by linear signed residual equals `d_ref/d` for nonzero residual; the multiplicative identity also holds at zero residual. This is deterministic finite-error accounting. It is not a stochastic coverage test, and the reference linearization differs from the predicted-disparity Jacobian used by the original propagated covariance score.

Save per-point residuals, raw horizontal variance, propagated metric variance, native own-endpoint point error and unchanged midpoint range bands. Groups are all plus the six EXP061/065 Euclidean midpoint ranges [0,5,10,20,40,80,infinity); no minimum-count exclusion. Six predicted/reference disparity ratio bins use lower edges [0,.1,.25,.5,1,2], with empty bins explicit. For each group save means, squared risks, normalized squared disparity residual and three Spearman correlations (average ties; null for constant/n<2). Raw pixel uncertainties are never compared between cameras. Normalized squared residual means are descriptive and have no chi-square or calibration interpretation here.

## Results

All entries are medians of20 per-frame quantities. Correlations rank within each camera and frame; the final column is a median of paired per-frame ratios, not a ratio of reported marginal medians. No capture risks are pooled.

| Capture | Camera | Mean signed disparity (px) | Raw variance vs disparity error² ρ | Raw variance vs depth error² ρ | Propagated variance vs 3D error² ρ | Exact / reference-linear depth MSE |
|---|---|---:|---:|---:|---:|---:|
| exp063_rain | rgb | 0.257 | -0.042 | 0.134 | 0.677 | 0.4875 |
| exp063_rain | thr | 1.781 | 0.463 | 0.052 | 0.623 | 0.08736 |
| exp064_day | rgb | 0.005 | 0.152 | 0.106 | 0.663 | 892.1 |
| exp064_day | thr | 1.078 | 0.513 | 0.325 | 0.718 | 0.3848 |
| exp064_night | rgb | -0.144 | -0.100 | 0.358 | 0.693 | 0.2438 |
| exp064_night | thr | 1.953 | 0.317 | -0.563 | 0.287 | 0.1068 |
| exp064_rainy | rgb | -0.089 | 0.214 | 0.207 | 0.539 | 0.8739 |
| exp064_rainy | thr | 1.954 | 0.386 | -0.107 | 0.186 | 0.1868 |

For EXP064 day RGB, the fixed `d/d_ref < .1` bin contains a median0.6836% of valid points but99.0463% of axial-depth squared error and9.1270% of disparity squared error. It is nonempty in all20 frames (point fractions0.1883%–1.1374%). No points were removed. Median exact/reference-linear depth MSE is892.1, illustrating finite inverse-disparity amplification in this capture; other captures do not show this same headline tail. A median zero bin fraction elsewhere does not mean no rare points: EXP063 RGB has a maximum0.0598% and EXP064 rainy RGB0.3322%. Thermal has no points in this bin in these four windows.

Thermal mean signed disparity residual is positive and native mean depth residual negative in all80 frames. This is consistent with the signed residual structure in EXP065, but is still conditional on supplied references, camera convention and selected support. Neither a physical thermal calibration bias nor a sensor correction is established. The RGB averages also illustrate why signs of averaged disparity and averaged depth residuals need not invert: weights depend nonlinearly on both disparities.

In EXP064 night thermal, raw variance versus disparity squared error has medianρ=0.3168, while raw variance versus axial-depth squared error has medianρ=-0.5632 (negative in all20 frames). Propagated variance versus own-endpoint3D squared error is0.2871 unstratified. That positive aggregate does not survive every range: thermal20–40m,40–80m,>=80m propagated correlations have medians-0.5166,-0.4073,-0.8277 respectively; all three bands are nonempty in all20 frames. Sparse far-band counts are explicit in the full report and cannot be treated as independent or equally precise evidence. Range conditioning changes the target population; the report preserves all groups rather than selecting a favorable result.

The original propagated score includes predicted inverse-disparity geometry, which also appears in depth error. A positive propagated correlation alone cannot isolate learned uncertainty information, certify physical calibration, or establish cross-camera comparability. This diagnostic did not add a new geometry-control test; previous EXP056–064 controls and failed transfer outcomes remain relevant. Cross-camera midpoint risks and within-camera own-endpoint ranks answer different questions.

## Validation and artifacts

Independent audit PASS:4512464 scalar/hash/count checks;332 runner input hashes and160 native depth PNG hashes;80 frames/106562 shared valid pairs. The auditor reads native supplied depth PNGs, reconstructs disparities directly from raw frontend fields, checks saved derived arrays and endpoint transforms, recomputes rank correlations via independent average ranks plus Pearson dot products, verifies all group/bin risks and shares, and recomputes every summary median and availability count. Eight analytical fixtures cover zero/positive/negative residuals,1000× amplification, average ties, singleton/constant null ranks and empty groups. Figures inspected. These checks certify arithmetic/source preservation, not physical accuracy.

Tracked scripts in `MAC-VO-main/Scripts/Setup`: `ms2_disparity_diagnostics.py`, `validate_ms2_disparity_diagnostics.py`, `plot_ms2_disparity_diagnostics.py`. Results under `results/ms2-disparity-diagnostics/run-001/`: protocol/report,160 derived NPZs, independent/synthetic validation, runtime versions and PNG/PDF figures. Run at the MAC-VO root with `/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.ms2_disparity_diagnostics --root /root/AirStack/ore-ms --output <fresh-output>`, then the validator with `--run <output>`. Parent raw fields/source products are in verified EXP063/064 backups. Private EXP066 archive includes new outputs/scripts/notebook and commit bundle, excludes credentials/checkpoints/full transport archives.

## Decision

Do not discard tiny positive disparities after observing their effects or fit a label-assisted offset and present it as deployable. Keep the existing defaults and holdouts. A justified next step is a declared comparison of a distribution-aware disparity-to-depth risk representation against the existing first-order score, with camera-scale/geometry controls and explicit finite-risk assumptions. Before implementing, specify which distribution/tail assumptions can be supported: positive disparity alone does not guarantee finite inverse-depth moments. Independent geometric references and training provenance remain necessary for stronger physical claims. Another unchanged test capture or unbounded feature search is not justified by these results.
