# Measurement-level translation refinement: small day benefit without rotation cost

15 November 2026 UTC. Branch `ore-ms-orb-development`, baseline `53afa19d1`
plus frozen local tools. Application baseline ablation, not a new learned model,
SFVO reproduction or established algorithm novelty.

## Main result

Keeping native RGB rotation and re-solving translation from RGB/thermal
measurements gives a **scoped daytime improvement without an orientation cost**.
Position RMSE falls11.115217→11.037059m (**−0.703%**) versus native RGB.
RGB-only translation re-solving reaches11.074827m, so thermal's additional
improvement over the matched reoptimization control is **−0.341%**.
Allsixpredefined daytime100-group blocks beat both controls on position RMSE,
with their original native RGB orientations unchanged.

This is small but useful application evidence. The78mm path-RMSE difference
against native includes40mm from RGB reoptimization; the thermal-specific
increment is about38mm. Do not credit the complete native-to-fused gain to thermal.
No statistical significance or practical deployment threshold is established.

Rain also improves against native RGB, but **fails the thermal-specific test**:
thermal63.398303m versus RGB-only refinement63.241788m, so thermal adds0.247%
regression. Native rain RMSE is64.141241m; its apparent1.158%improvement cannot
be credited to thermal. Night has the same missing pair247 and no full path.

## Complete600-pose paths

| Condition | Method | Position RMSE m | Orientation RMSE deg |
|---|---|---:|---:|
| day | Native RGB | 11.115217 | 4.036908 |
| day | RGB translation re-solve | 11.074827 | 4.036908 |
| day | RGB + thermal translation re-solve | 11.037059 | 4.036908 |
| night | Native RGB | Incomplete | Incomplete |
| night | RGB translation re-solve | Incomplete | Incomplete |
| night | RGB + thermal translation re-solve | Incomplete | Incomplete |
| rain | Native RGB | 64.141241 | 4.346644 |
| rain | RGB translation re-solve | 63.241788 | 4.346644 |
| rain | RGB + thermal translation re-solve | 63.398303 | 4.346644 |

Day final position error also improves: native17.656844m, RGB-only17.579883m,
RGB+thermal17.519431m. Rain endpoint error is124.093053m,121.631504m and122.107880m
respectively; thermal again underperforms RGB-only refinement.
No fitted gauge/scale/time alignment is used beyond the initial identity gauge.
These are the same exposed, nominal provided-reference records.

## All fixed100-group blocks

Thermal translation re-solve compared against native RGB and RGB-only translation
re-solve. Negative favors thermal. Rotation RMSE equals native RGB in every
complete block, by construction and independent checks.

| Condition | Start | Position change vs native RGB | Position change vs RGB re-solve |
|---|---:|---:|---:|
| day | 0 | -1.674% | -0.540% |
| day | 100 | -0.239% | -0.221% |
| day | 200 | -0.338% | -0.338% |
| day | 300 | -0.052% | -0.052% |
| day | 400 | -0.211% | -0.211% |
| day | 500 | -0.761% | -0.355% |
| night | 0 | +0.090% | -0.002% |
| night | 100 | -1.246% | -0.644% |
| night | 200 | Incomplete | Incomplete |
| night | 300 | -1.721% | -0.929% |
| night | 400 | -0.854% | -0.202% |
| night | 500 | -1.103% | -0.079% |
| rain | 0 | -2.057% | +0.052% |
| rain | 100 | +0.440% | +0.575% |
| rain | 200 | +0.860% | +0.855% |
| rain | 300 | -0.502% | -0.378% |
| rain | 400 | -0.671% | +1.164% |
| rain | 500 | -2.036% | +0.511% |

The thermal arm beats both controls in6/6day,4/5complete night and1/6rain blocks,
11/17complete blocks overall. The one incomplete night block remains included.
Night's firstblock slightly improves against its constrained control but remains
worse than native. No successful subset was chosen for prediction or excluded
from reporting. Reusing these records/windows is not independent validation.

## Why this is a bounded extension rather than a new weighting direction

The [9November translation study](translation-fusion-findings-2026-11-09.md)
blended completed RGB/thermal translations and never reoptimized their underlying
observations. That study is already completed; retaining RGB rotation is not a
new conceptual contribution. The newly qualified native shared-factor study
provided a specific measured motivation: lower day position error accompanied
by worse rotation, persisting under pose feedback.

This assay changes the optimized state to onlythreeworld-translation coordinates,
keeps the native RGB quaternion and jointly re-solves the actual calibrated
RGB/thermal disparity residuals. Its matched RGB-only three-coordinate control
separates thermal influence from the additional reoptimization. The comparison
answers whether a thermal translation benefit remains when thermal is prevented
from changing rotation. It is a concrete native measurement-level ablation, not
another learned scalar, confidence head, capacity or weight search. Thermal
precision0.1 is unchanged from the previous fixed baseline.

The successful day case supports preserving this constrained application
candidate for separately frozen validation. It does not establish a condition
predictor, a general solution across rain/night, or a research novelty claim.
Do not select another weight or winning window from these scored results.

## Mechanism and controls

`FixedRotationGraph` exposes a3element translation parameter to the unchanged
native analytic LM/Huber/PINV/trust/scheduler. Native disparity residual,
covariance and Jacobian code are delegated unchanged; only the translation
columns of the Jacobian enter the solve. The original pose graph is deliberately
outside the wrapper's optimized parameter list, preventing quaternion updates.

Native RGB rotations come from the already locked, reference-free sequential RGB
backbone on the same source pair. Both candidate arms have their own float32
committed translation histories. Source points are registered at that arm's
parent pose. Warm initialization adds the current native RGB displacement to
that arm's parent translation. Current RGB measurements are required to compute
that backbone; no future frame or supplied pose reference is used for prediction.

The native RGB orientation branch remains an independent backbone rather than
being re-estimated from corrected translation. This is a defined asymmetric
architecture, not a claim that translation/rotation uncertainty is independent.
Missing thermal factors use a fresh RGB-only constrained solve at the fused
arm's state. Missing RGB holds internal state and publishes a null increment.
No interpolation, solver rescue, factor removal, timestamp adjustment or
reference-dependent pairing occurs.

3592new constrained solves finish without an exception,1796per arm. Thermal
factors are present in595day/558night/598rain fused solves;45use onlyRGB factors.
Both arms retain599/598/599supported increments. The already computed1796native
RGB backbone solves are also logically required by this architecture; they were
reused here. Reduced optimized dimension is not measured end-to-end compute saving.

## Qualification and audit

Nine fixed qualification pairs (0/1/2 eachcondition) verify exactly3optimized
coordinates, exact RGB residual/Jacobian prefix equality, finite-difference
Jacobians (maximum normalized error1.924e-7) and exact float32 native quaternion
preservation after optimization. A PyPose slicing warning is retained in logs;
all equality/derivative checks pass. No qualification/source/tolerance amendment
was needed for this experiment.

All predictions are hash-locked before new reference scoring. Independent audit
passes **10922checks**: input/source/baseline pins, parent/committed-state chains,
exact native quaternion equality for every available output, missing-state hold,
relative transforms recomputed from endpoints, direct committed-state full/block
metrics and native orientation-metric equality within1e-8deg. No new model
inference, training, reserve access or test-driven tuning.

This remains fixed-accepted-measurement replay under the verified static-motion,
all-keyframe, no-mapping configuration. It is not fresh dense-frontend inference,
live VO, certified exposure synchronization, physical ground truth or new-record
validation. Existing source-depth, calibration and supplied-reference limitations
remain. The gains do not identify an environmental cause or prove calibrated
cross-modal uncertainty.

## Primary-source overlap checked before execution

[SFVOv1](https://arxiv.org/html/2609.21754v1), targetedIII-C/D andIV-C, already
uses separate learned rotation/translation confidence and alternating bidirectional
bearing-based pose updates. Its confidence ablation retrains a shared-confidence
baseline and describes weighted least-squaresPnP, so do not treat its reported
contrast as isolating confidence from every solver difference. Our hard-fixed
RGB rotation and native disparity solve do not reproduce that architecture.
The paper promises code; bounded title/GitHub-title searches did not identify a
verified author release. This is not proof no public/private implementation exists.

[Buczko/Willert2016](https://www.cvlibs.net/projects/autonomous_vision_survey/literature/Buczko2016ITSC.pdf),
introduction and rotation-compensated-flow rationale inspected, already separates
motion components to improve correspondence assessment. Generic rotation/translation
decoupling and confidence roles must not be presented as new. No author code was
executed. Source payload hashes, reading scope and release-check limits are in
`results/component-constrained-native/source-review/sources.json`; reading copies
remain outside the repository. The living registry now contains61entries.

Artifacts: `results/component-constrained-native/{source-review,qualification-001,
run-001}`; tools `component_constrained_native.py` and
`audit_component_constrained_native.py`. Runtime remains Torch2.7.1+cpu,
PyPose0.6.8, one CPU thread. Native defaults are unchanged.
