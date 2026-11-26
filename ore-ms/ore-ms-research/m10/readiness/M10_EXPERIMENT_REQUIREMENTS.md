# Frozen proposed M10 downstream experiment

This specification is frozen for readiness review. Policy implementation, threshold estimation and evaluation each require separate authorization and preflight evidence. Environmental burden and transmission scores are explanatory variables only.

## Task and operating clock

Localize a known target class in a search region of a registered, controlled synthetic RGB/LWIR scene. Use 33×25 common pixel centers with finite known nominal calibration. At t=2 s, produce one supported visible-target pixel location or abstain. The optional atmospheric forecast is issued at t=0 s for horizon 2 s. The selected task frontend can use images acquired at t=2 s. Neither stage can read later inputs. Actor and background geometry, target material and air/illumination controls come from the supported synthetic primitives. Externally supplied candidate range may describe a search fixture without revealing presence, first hit or target location.

The scorer labels a feasible target localization when at least four first-hit target pixels and at least 25% of its in-frame projected support are visible. The point reference is the centroid of the visible target support. An issued location within 1 pixel Euclidean distance counts as correct when the opportunity is feasible. Score true absence, full occlusion, empty projection, insufficient partial support and matched-looking distractors as separately labeled negatives for a **supported visible-target localization**. For any infeasible case, a declared supported location is a false-supported claim. A non-issued prediction is an abstention, with no implication of physical absence. The thresholds are benchmark conventions for this grid. A scale change requires a newly frozen protocol.

## Comparison matrix

All arms use identical task scenarios, seeds, timestamps, nuisance admissibility and candidate operating conditions. Frontends, calibration, output axes and task label rules are fixed across arms. Apply the same acquisition and validity gates to every feasible arm. Missing modality tests run on the same paired opportunity group.

| Arm | Required interpretation |
|---|---|
| RGB-only | Same RGB frontend and task criteria throughout. |
| LWIR-only | Same thermal frontend and task criteria throughout. |
| Fixed multispectral fusion | Static equal channel contribution on eligible paired candidates, without per-issue environmental dependence. |
| Nonadaptive reliability baseline | One constant per-modality reliability calibration from independent development/calibration data, frozen across issues; use shared admission/abstention gates. |
| Adaptive reliability-aware fusion | Uses only admitted information, with documented dependencies, calibration and support flags. |
| Privileged reference | Chooses from the same eligible candidate predictions using ground-truth correctness, with candidate/coverage limits stated. Lives in the scorer and is nondeployable. |

At least 128 independent development, 128 calibration and 256 locked holdout realization IDs are planned. Each ID has paired controls. Group all views, timestamps and interventions by ID. Validate case/ID counts and split hashes before policy work. No reference model or cutoff can be tuned on the blind set. Pre-specified strata include reference, wavelength-selective attenuation, noise/rails, missing single/both modalities, apparent contrast without support, crossover, partial/full occlusion, known/unknown range or material, calibration/timing offsets, independently perturbed time constant and process amplitude, wrong air, and blind future-only shift with identical issue inputs. Include absent targets and matched distractors. Keep attenuation, contrast, geometry, availability, reliability-estimation error and adaptive decision error as separate recorded fields.

## Scoring and acceptance design

Primary task outcome is correct supported localization per feasible opportunity at a predeclared risk/coverage operating point. Freeze thresholds on calibration, report all opportunities and equal-coverage slices on holdout, and compare separately to each fixed and single-modality reference. For a gain claim require an improvement with a paired two-sided 95% interval excluding zero versus the strongest fixed comparator selected on development and no unsupported stratum claim. Report false-supported localization on absent, hidden and other infeasible cases, selective risk, issuance/availability and abstention rates, conditional location error, Brier/reliability for correct-candidate probabilities, and costs (end-to-end latency and measured compute on matched hardware). Screen the claimed operating point at no more than 5% false-supported localization, using an upper 95% bound separately in absent and hidden primary negative strata. If safety or calibration bounds cannot be estimated credibly, make no gain claim for the affected cell and retain it.

Predeclare the primary hypothesis and preserve null/negative outcomes. Use paired realization-level differences and a 95% grouped bootstrap (10,000 draws, seed 10091), resampling IDs with all paired cases/timepoints kept together. Report stratum pointwise intervals as descriptive plus sample counts, coverage denominator and uncertainty. Empty and tiny cells remain explicit. Require ablations of environmental forecast/burden features, forecast horizon, geometric/health gate dependence, conditional dependence versus independence treatment, and M09/novelty features. Score calibration and fallback-cost separately, including false alarms and missed mismatches. Report exact grouping and a modality-failure/outcome table. A lower burn-in error or better environmental forecast score cannot satisfy the primary outcome.

Before supervised policy comparison, require a checksummed corpus manifest and allowlist, independently verified centroid/visibility/negative labels on frozen fixtures, paired split and acquisition audit, working single and fixed frontends, and a separate task-calibration plan with stop rules. No part of this benchmark was run at readiness.
