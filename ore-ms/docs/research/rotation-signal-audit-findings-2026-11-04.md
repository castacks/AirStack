# EXP131: completed-output rotation signal audit

2026-11-04, branch `ore_ms`, parent `a00a87918`, pre-extraction/fitting freeze `6c7ddefc3`.

The predeclared combined ridge method selects thermal on all 323 intervals and adds no benefit over fixed thermal. The two uncertainty-only fixed rules reduce pooled rotation discrepancy against recorded SLAM, while increasing translation discrepancy. This is a retrospective development diagnostic; it establishes no general selector or physical accuracy benefit.

| Method | Rotation mean (deg) | Translation mean (m) | Rotation-winner accuracy | RGB / thermal choices |
|---|---:|---:|---:|---:|
| Fixed RGB | 0.250211 | 0.248600 | 33.44% | 323 / 0 |
| Fixed thermal | 0.232241 | 0.127102 | 66.56% | 0 / 323 |
| Training-recording best fixed | 0.232241 | 0.127102 | 66.56% | 0 / 323 |
| Minimum raw median trace | 0.217604 | 0.205861 | 70.28% | 132 / 191 |
| Minimum angular-proxy median trace | 0.220639 | 0.225521 | 69.04% | 160 / 163 |
| Maximum retention fraction | 0.232241 | 0.127102 | 66.56% | 0 / 323 |
| Maximum occupancy fraction | 0.234167 | 0.183355 | 65.33% | 104 / 219 |
| Ridge: uncertainty | 0.232241 | 0.127102 | 66.56% | 0 / 323 |
| Ridge: health | 0.233387 | 0.127105 | 62.85% | 12 / 311 |
| Ridge: combined (primary) | 0.232241 | 0.127102 | 66.56% | 0 / 323 |

The EXP130 retrospective rotation oracle is 0.165564 degrees. All methods use the same 323 joint rows, with zero missing/invalid-feature fallback rows; the original 380-row availability population remains unchanged. The table reports all ten predeclared methods, including diagnostics. No method is selected for adoption after scoring.

| Fixed uncertainty rule | Rotation reduction vs thermal | Oracle headroom recovered | Translation increase vs thermal |
|---|---:|---:|---:|
| Minimum raw median trace | 6.302% | 21.952% | 61.965% |
| Minimum angular-proxy median trace | 4.995% | 17.399% | 77.433% |

These rules trade translation for rotation; the two targets cannot be collapsed into an overall improvement claim. Raw pixel trace and focal-normalized trace are different hypotheses. The focal normalization approximates angular flow scale, and is not calibrated pose covariance. Better raw-trace results on this pool do not establish cross-camera uncertainty calibration.

| Evaluation recording | Rows | Fixed thermal rotation (deg) | Raw trace (deg) | Angular proxy (deg) | Combined ridge (deg) |
|---|---:|---:|---:|---:|---:|
| night_thermal_so_salon | 133 | 0.015110 | 0.015110 | 0.015110 | 0.015110 |
| night_thermal_so_salon3 | 190 | 0.384232 | 0.359350 | 0.364510 | 0.384232 |

Both fixed uncertainty rules select thermal throughout salon (133 rows), so their gains arise in salon3 (190 rows). On salon3/middle the angular-proxy rule improves rotation mean from 0.326229 to 0.281869 degrees; on salon3/late it worsens 0.442235 to 0.447151 degrees. The raw rule improves both salon3 windows, to 0.294280 and 0.424420 degrees respectively. There is no uncertainty-rule advantage on the other recording.

The combined regression and uncertainty-only regression select thermal for every evaluation interval in both recording-disjoint folds. The health-only regression makes twelve RGB selections on salon and worsens that recording’s rotation mean. This fixed model/regularization failed to recover the available rotation headroom; the experiment does not establish that no other model could do so. No lambda, feature or threshold was changed after seeing these results.

The retained/inbound fraction is exactly equal between cameras on every joint row, so its difference is identically zero in both recordings. Occupancy and inbound-count differences vary. Descriptive Spearman correlations between median uncertainty-ratio and RGB-minus-thermal rotation discrepancy are about 0.201 on salon and 0.099 on salon3. Other feature ranges/correlations and training-only coefficients/scales are retained in the output JSON. Correlated intervals and two recordings do not justify significance claims.

![Frozen signal results](../../results/rotation-signal-audit/run-001/signal-audit.png)

Each fold trains on the complete other recording and predicts its evaluation recording using only those training labels. Three uncertainty log ratios and three health differences are fixed in advance. Ridge uses lambda=1, unpenalized intercept and training-only feature standardization, with no search. The training-recording best-fixed baseline chooses thermal in both folds. Window IDs and reference values never enter model features. The full reference outcome file was already inspected in EXP130; recording-disjoint computation is not a blind held-out test or independent replication.

Signals come only from the target pair_end event, with exact native interval matching and controller receipt at or before the existing completed-output decision boundary. Both target motions are already available there. No reference-derived quantity enters the extracted features. This validates timing for choosing a completed output in the saved replay; a sparse scheduler deciding whether to run that target expert cannot use these same target signals. Both experts’ execution costs remain incurred, and there is no new scheduling or compute-saving result.

Validation: 5,744 checks and 10 semantic mutations pass. The audit independently reconstructs event provenance, focal factors and feature formulas; reproduces ridge using augmented least squares rather than the fitted normal-equation implementation; checks recording-disjoint training statistics/IDs and choices; and recomputes selected losses. It rejects late/wrong-interval events, verifies missing-feature thermal fallback, and poisons each evaluation recording’s labels to confirm that its fold’s model and choices remain unchanged. Twenty direct pins plus EXP130 ancestry remain unchanged. Predictions were locked before new selector summaries. CPU Docker emitted the existing driver-init warning and exited zero.

Protocol and source were committed before feature extraction. A syntax error was corrected before freeze; no feature extraction or fitting occurred in that failed preflight. No scored attempt was retried or overwritten. No reference acquisition, new inference, reserve payload, candidate/default change or messages occurred. EXP128 MSA exact-replay failure remains unchanged. Physical body mounting, clock equivalence, reference independence and original network training overlap remain unresolved.

Prediction SHA256: `237af19f1e63bd24486149100ba473cc6947e0223aed6083b3a6cfb6d624d2ba`. Evaluation SHA256: `28253a96cc08ae5ccb0bcc1eced7e1de0119195b4fca4198c48a29a893e441a3`. Artifacts: `results/rotation-signal-audit/run-001/` contains feature/prediction locks, 323 source-event feature receipts and choices, fold models, evaluation, audit and figures. [Protocol](rotation-signal-audit-protocol-2026-11-04.md).

Research implication: EXP130 shows a rotation oracle target; EXP131 finds limited observable signal for two fixed rules and no benefit from the primary learned transfer method. Translation deterioration prevents treating those fixed rules as a general pose selector. The next useful check is a separately frozen transfer of the unchanged rules on existing saved day/night captures, with input recipes reported separately and both pose metrics retained. Audit saved timing/feature/reference sufficiency before running it. Additional independent recordings and the existing physical reference/clock/mounting evidence remain necessary before generalization or deployment claims.
