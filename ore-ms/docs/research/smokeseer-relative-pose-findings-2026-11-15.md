# Fixed SIFT exposes a substantial synthetic smoke gap

15 November2026. Completed all ten declared scenes,3495image feature extractions
and3465arm-pair predictions once under the unchanged [protocol](smokeseer-relative-pose-protocol-2026-11-15.md).
No training, parameter/window search, fallback or new method. These are exposed
synthetic monocular relative-pose results, not real-smoke stereo-VO or an author-
system reproduction. Clean-camera equivalence follows the released loader; it
is not independently verified render provenance.

## Result and interpretation

All-ten-scene rotation success falls from90.04%on clean RGB to49.44%on smoky
RGB using the same fixed estimator. That is a40.61percentage-point degradation
on these supplied counterparts. The effect is substantial within this diagnostic;
it is not a contribution from our fusion/optimization changes.

For the nine scenes with identical RGB/thermal training camera poses, thermal
rotation success is73.61%versus44.74%for smoky RGB:28.87percentage points higher.
Thermal beats smoky RGB in eight of these nine scenes, but loses on Hotdog.
Caterpillar is preserved separately because its camera paths differ. Thermal
remains below clean RGB and does not establish a universally reliable solution.

Primary success requires recovered pose and rotation error<=5degrees; joint
success also requires directed translation error<=15degrees. Every unsupported
pair counts as failure. All reference translations are defined.

## All declared scenes

| Input | Pairs | Recovered pose | Rotation success | Joint success | Conditional median R / t error |
|---|---:|---:|---:|---:|---:|
| Smoky RGB | 1155 | 821/1155 (71.08%) | 571/1155 (49.44%) | 520/1155 (45.02%) | 2.113° / 5.947° |
| Clean RGB | 1155 | 1126/1155 (97.49%) | 1040/1155 (90.04%) | 1012/1155 (87.62%) | 0.641° / 1.586° |
| Thermal | 1155 | 979/1155 (84.76%) | 833/1155 (72.12%) | 785/1155 (67.97%) | 0.867° / 2.418° |

Conditional medians include only recovered poses; they do not replace the
all-pair success denominator. Scene macro primary rates are48.47%smoky RGB,
89.53%clean RGB and70.75%thermal. Do not treat adjacent pairs as independent
recordings or infer statistical significance from the pooled sample count.

| Scene | Pairs per arm | Smoky RGB primary | Clean RGB primary | Thermal primary | Identity rotation |
|---|---:|---:|---:|---:|---:|
| caterpillar* | 109 | 94.50% | 100.00% | 57.80% | 0.00% |
| chair | 109 | 33.03% | 100.00% | 59.63% | 0.00% |
| ficus_rgb | 109 | 21.10% | 80.73% | 59.63% | 0.00% |
| forest | 124 | 55.65% | 94.35% | 81.45% | 0.00% |
| hotdog | 109 | 12.84% | 59.63% | 8.26% | 0.00% |
| house | 149 | 69.80% | 100.00% | 99.33% | 100.00% |
| lego_smoke | 109 | 75.23% | 100.00% | 99.08% | 0.00% |
| mars | 119 | 63.87% | 100.00% | 99.16% | 0.00% |
| musclecar | 109 | 19.27% | 64.22% | 46.79% | 0.00% |
| room | 109 | 39.45% | 96.33% | 96.33% | 0.00% |

*Caterpillar thermal uses a different camera path; its thermal/RGB difference
is not a matched-camera effect. House is the only scene where identity rotation
passes the5degree endpoint (all149pairs); its rotation endpoint is therefore
weak by itself. Identity passes12.90%of all1155pairs. Preserve joint/direction
and support reporting rather than changing thresholds after seeing this result.

## Nine matched-camera scenes

| Input | Pairs | Rotation success | Joint success |
|---|---:|---:|---:|
| Smoky RGB | 1046 | 468/1046 (44.74%) | 417/1046 (39.87%) |
| Clean RGB | 1046 | 931/1046 (89.01%) | 903/1046 (86.33%) |
| Thermal | 1046 | 770/1046 (73.61%) | 730/1046 (69.79%) |

Hotdog is a retained counterexample: thermal9/109primary successes, smoky
RGB14/109and clean RGB65/109. This run does not isolate the reason. No contrast
rescue, scene removal or alternative matcher was tried. Clean RGB also remains
imperfect in Ficus and Musclecar; not every failure can be assigned to smoke.

## Execution, qualification and audit

One CPU thread, OpenCV5.0.0/NumPy2.2.6 in the pinned restored image
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`.
Execution took403.62seconds including recorded runner overhead; this is not a
matched runtime comparison or real-time claim. Original uint16Mars images were
normalized at full depth; all grayscale hashes match the preflight.

The first synthetic API qualification stopped on an inappropriate1e-6exact-pose
assertion for a1pixel-tolerance RANSAC estimate. The original source/log are
preserved. Before any real-pair predictions, the check was corrected to verify
exact pose/sign using analytic E, and estimated E using repeatability, support
and pixel residual. No frozen estimator parameters or scientific endpoint gates
changed. Final qualification passes10497checks; independent score mechanics
passes404checks. Runner and all inputs were frozen before real predictions.
Predictions were locked before reference scoring.

Independent audit passes56558checks and three mutation guards across all3465
rows: pins, unique coverage/order, pose masks, recovered rotations/directions,
angular scores, all scene/pooled denominators and macro rates. It validates
arithmetic/provenance, not physical calibration or novelty. Raw predictions,
correspondence coordinates, E/R/t, masks, unsupported reasons and the failed
qualification are under `results/smokeseer-sift/`.

`tools/smokeseer_sift.py` is necessary because the existing bearing assay consumes
frozen flow and has a different rotation-only/E-iteration contract; it was not
copied or repurposed. `tools/audit_smokeseer_sift.py` is independent. Original
preflight/protocol scripts and pins remain unchanged.

## Post-result failure diagnosis

A separate exploratory audit consumes only the locked outputs above: no new
image predictions, reruns, fitted rules, thresholds or scene exclusions. It keeps
all1046matched-camera pairs; Caterpillar retains its separate own-path results.
This is a post-result diagnosis, not a prospectively held-out selector experiment.

| Existing endpoint | Fixed thermal | Hindsight best fixed camera per scene | Pairwise success union | Both cameras fail |
|---|---:|---:|---:|---:|
| Rotation | 770/1046 (73.61%) | 775/1046 (74.09%) | 829/1046 (79.25%) | 217/1046 (20.75%) |
| Joint | 730/1046 (69.79%) | 732/1046 (69.98%) | 777/1046 (74.28%) | 269/1046 (25.72%) |

The union is a reference-informed ceiling for choosing between these two saved
outputs, not an executable selector or a bound on new matching/fusion methods.
Rotation has59RGB-only successes and361thermal-only successes; joint has47and360.
Per-scene hindsight choice leaves54rotation/45joint successes below the pairwise
union, or5.16/4.30percentage points. This is nonzero selection headroom on this
synthetic population, distinct from earlier near-zero natural availability audits;
do not pool populations or claim either result is universal.

More importantly,217pairs fail rotation on both cameras; clean RGB succeeds on
148of them (68.20%). Joint has269shared failures, with163clean-RGB successes
(60.59%). Clean failures remain69/106respectively; this is not evidence that all
shared failures are caused by smoke. Hindsight selection cannot fix shared failure.

Thermal's276rotation failures comprise47insufficient matches,107insufficient
cheirality support and122recovered-but-wrong rotations. Of59RGB-only successes,
32occur when thermal is unsupported and27when it returns an inaccurate pose.
For joint success the analogous split is19unsupported/28inaccurate. Thus output
availability is not equivalent to correctness; no new confidence predictor or
calibration follows from match-count associations.

| Scene | Shared rotation failures | Clean RGB succeeds within these | Longest consecutive shared failure |
|---|---:|---:|---:|
| chair | 27 | 27 | 8 |
| ficus_rgb | 36 | 25 | 7 |
| forest | 17 | 10 | 11 |
| hotdog | 86 | 52 | 23 |
| house | 0 | 0 | 0 |
| lego_smoke | 0 | 0 | 0 |
| mars | 1 | 1 | 1 |
| musclecar | 48 | 31 | 7 |
| room | 2 | 2 | 1 |

Hotdog's longest shared failure spans23adjacent rotation intervals, and38under
the joint endpoint; animation intervals are not seconds. These runs were counted
on the entire declared population, not selected for another benchmark. Do not
infer material, radiometric or smoke-density causes from these output labels.

**Subsequent gate, now completed:** the [learned control](smokeseer-learned-findings-2026-11-16.md)
reduces the217shared rotation failures to2. The following was the prospective
next question, not an outstanding task: determine whether the shared gap survives an existing learned
correspondence baseline before designing a method. The prior
[MACThermal thesis](https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments)
already addresses thermal correspondence through normalization and geometric
augmentation; simply proposing learned thermal matching is not new. Its restored
checkpoint passed earlier geometry-free temporal inference, so a frontend-only
control is a concrete feasibility candidate. That does not qualify its800×800
synthetic input, query/matching budget, domain transfer or pose accuracy.

A future comparison must freeze those interfaces and use the whole declared
population, both existing RGB/thermal checkpoints and the same monocular geometry
backend. Do not restrict execution to the148reference-identified failures, give
only the neural method privileged preprocessing, invent stereo depth, or call a
frontend control a full MACThermal reproduction. No new model was run in this diagnosis; no architecture or training was selected.

Analysis plan, helper, raw breakdown and256-check independent set audit are under
`results/smokeseer-failure-audit/`; preserved packet:
`/root/ore-ms-artifacts/smokeseer-failure-audit-2026-11-15/`. Parent predictions,
protocol and archive remain unchanged. This audit qualifies a next baseline
question, not an unsolved problem beyond strong learned baselines.

## Decision

The synthetic environmental test is complete. Smoke-related RGB degradation is
now measured under a fixed control, and a classical thermal baseline recovers
much of the lost success. That supports studying degraded visibility, while
remaining known thermal-robustness evidence rather than a new-method claim.
Do not resume nearby weight/threshold tuning or automatically add another model.
Before a new mechanism, identify which retained failures remain beyond strong
baselines and verify an evaluation population that supports the intended claim.
Real-smoke execution remains separately limited by the Gascola export contract.

Verified local packet: `/root/ore-ms-artifacts/smokeseer-sift-2026-11-15/`.
The frozen public input ZIP is in the prior synthetic-preflight packet. Decoded
grayscale caches are reproducible from it and are not duplicated in the evidence
packet. No Box upload, Git commit/push or author outreach occurred.
