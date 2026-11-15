# Gap map starting from the original papers

13 November 2026, America/New_York. Targeted primary-source review. The user
clarified that the task is to identify gaps in the original papers, not reduce
the question to gaps remaining in our current experiments. This supersedes the
previous recommendation to organize all next work around incremental thermal
utility. No method or experiment is selected by this map.

A = explicit author limitation/future work; M = assumption visible in the method;
E = boundary of inspected evaluation; I = our research interpretation.
An original-paper gap is not automatically an unfilled field-wide gap.

## MAC-VO: temporal estimation, uncertainty structure and selection

Source: [v2](https://arxiv.org/html/2409.09479v2), III-A–D, V, Appendix A.

| ID | Source evidence | Research question exposed |
|---|---|---|
| V1 — temporal estimation (A) | V identifies two-frame estimation and proposes multi-frame optimization, BA and loop closure. | How should uncertainty persist and change when observations are reused over time? |
| V2 — covariance structure (M/I) | III-A predicts diagonal image-flow covariance; Appendix A-C assumes independent pixel coordinates and depth before projection. | When does neglected input dependence matter? The resulting 3D covariance already has off-diagonal terms: adding those is not the gap. |
| V3 — feature selection (M/I) | III-B combines NMS, geometric filtering and a frame-relative uncertainty cutoff. | Does selecting individually reliable features preserve the geometry needed to constrain all motion directions? |
| V4 — supervision (M/I) | III-A trains covariance with ground-truth synthetic flow likelihood. | Which training signal supports reliable uncertainty where dense thermal labels are scarce? This is an extension question, not demonstrated failure of synthetic training. |

Later-work qualification: MAC-I² addresses visual–inertial fusion, one stated
extension. Current [official optimizer documentation](https://mac-vo.github.io/wiki/Modules/IOptimizer/)
also lists two-frame pose/landmark BA. Thus “add BA” is too broad; it does not
establish a new persistent multi-frame uncertainty treatment. No current code
body was re-audited here.

## MACThermal: continuity, supervision, sensor diversity and autonomy

Source: [thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf),
3.2–3.5 and6, printed pp23–28,61–63 / PDF36–41,74–76.

| ID | Source evidence | Research question exposed |
|---|---|---|
| T1 — continuity (A) | Chapter6 identifies dependence on continuous video/stable calibration; proposes reinitialization and thermal–inertial SLAM. | How can a thermal estimator recover state through outages, timing jitter or calibration changes? |
| T2 — training data (A/M/I) | 3.2 describes RGB–thermal translation alignment/domain problems and geometry-generated flow supervision. | How can supervision preserve geometry while covering real thermal appearance changes? |
| T3 — transfer (A) | Chapter6 identifies limited platforms/conditions and motivates broader thermal data. | What can transfer across sensors, platforms and thermal conditions without a new full training recipe? |
| T4 — multispectral estimation (A) | Chapter6 names modality-invariant matching and cross-modality fusion. | Can complementary modalities improve representation or estimation, beyond changing weights on frozen outputs? |
| T5 — active perception (A) | Chapter6 identifies forward-FOV blind spots and proposes resource-aware mapping and perception-aware planning. | Can motion or camera allocation improve perception rather than passively accepting its failures? |

These extend beyond covariance comparability. Thermal–inertial and multispectral
systems already exist; the unresolved contribution would require a specific
mechanism and comparison, not merely adding a sensor.

## MAC-I²: estimator scope, approximation and transfer cost

Source: [v1](https://arxiv.org/html/2609.07116v1), IV-A, V–VII,
Appendices B,E–G.

| ID | Source evidence | Research question exposed |
|---|---|---|
| I1 — persistent estimation (A/E) | VII proposes full VIO/SLAM and broader fusion; evaluation includes initialization/calibration and AppendixE pose improvements. | Does the approach remain consistent in continuous estimation with repeated states and measurements? “No pose benefit evaluated” would be false. |
| I2 — posterior approximation (M/I) | AppendixB uses a local stationary-point Gaussian approximation and Gauss–Newton information. | How reliable is that uncertainty under weak geometry, substantial residuals or imperfect convergence? This is an approximation question, not a demonstrated mathematical error. |
| I3 — sensor adaptation (M/I) | AppendixF states learned IMU behavior is sensor-specific; raw-IMU operation is available. | Can adaptation need less per-sensor data while retaining meaningful covariance? |
| I4 — evaluation granularity (E/I) | AppendixF includes visual/IMU covariance generalization and binning analysis. | What additional evidence is needed for conditional coverage, joint directional consistency or sequential reliability? Generalization itself is already evaluated. |

MAC-I² partially addresses MAC-VO's fusion extension; it does not close all
continuous-estimation, transfer or posterior-model questions. None is established
as field-wide novelty by this review.

## Related original method: DNLS

[DNLS v2](https://arxiv.org/html/2305.09527v2), section5 and AppendixB,
explicitly identifies unobservable absolute noise scale, sparse keypoint/RANSAC
training gradients, additional computation and failures involving dynamic
objects. These expose distinct identifiability, supervision-coverage, efficiency
and robustness questions. Pose-loss training itself is already its contribution.
Our small shared-weight experiment neither reproduces DNLS nor closes those
questions. No new official-code inspection or execution occurred in this pass.

## What later work already overlaps

This is a targeted check, not a systematic citation survey.

| Original opening | Known subsequent/related coverage | Status of the gap claim |
|---|---|---|
| MAC-VO visual–inertial extension | MAC-I² directly addresses initialization/calibration and evaluates pose improvement. | Partially addressed; generic covariance-based VI fusion is not a new gap. |
| Thermal/multispectral inertial estimation | MSO thesis Chapter5 and previously reviewed thermal-inertial papers. | Broad application already populated; exact comparative mechanism remains to be specified. |
| Anisotropic uncertainty in differentiable BA | [MomentBA v1](https://arxiv.org/abs/2609.13691) proposes covariance from matching-response moments in differentiable BA. Abstract inspected only. | Additional overlap warning; full methods/code not yet reviewed. It does not prove the original dependence question solved. |
| BA in the released MAC-VO ecosystem | Official docs list a two-frame reprojection BA implementation. | Code/version distinction required; paper future work cannot be repeated as a blanket absence claim. |
| Every gap above | No exhaustive subsequent-work search completed. | “Not settled by the original paper” is supported more strongly than “nobody has solved it.” |

MSO depth remains the earlier full Chapter5 review, not fresh thesis reading;
later manuscript/code remain unresolved. See the
[version-specific review](mso-method-overlap-review-2026-11-12.md).

## Compare with local work only after identifying the gaps

| Gap family | Local coverage | What is still a legitimate research direction |
|---|---|---|
| Temporal/joint geometry | Five-frame conditional-ray/stereo pilot completed; exact gate failed. | Different treatments of persistent state, dependencies, marginalization or continuity remain possible. A generic longer-window rerun adds little. |
| Covariance structure/consistency | Full spatial covariance, local pose information, calibration and pose-combination controls exist. | A precise missing dependence/approximation can be studied; “full covariance” alone is already covered. |
| Supervision/representation | One thermal matcher replacement, one depth-transfer setup, synthetic covariance fits and a six-parameter shared-weight recipe tested. | Thermal-specific/self-supervised or shared-representation mechanisms are not exhausted by those configurations. |
| Selection/observability | Geometry, conditioning and opportunity diagnostics exist. | A new selection objective must differ from existing point-count, trace or conditioning rules; no new solution is asserted here. |
| Continuity/recovery | Bounded synthetic handover/buffering tests exist. | Production thermal-inertial state recovery and calibration drift are broader than that engineering fixture. |
| Active perception | This pass did not audit planner implementations/results. | Treat as a separate research axis; do not call it untouched without that audit. |

The local failures describe configurations. They do not remove original-paper
gaps from consideration. Existing assets may make one direction easier to test,
but that is a feasibility preference, not evidence that it is the only question.

## Decision following this review

Keep multiple directions available: temporal uncertainty/continuous estimation;
thermal supervision and representation; structured uncertainty and selection;
and recovery/active perception. Compare their scientific specificity, later-work
overlap, data/implementation requirements and fit to the user's interests before
choosing one. Cross-camera reliability is one option within this map.

The useful next source pass is version-specific methods/code review for a short
list chosen from these families. This document does not impose the prior narrow
“incremental thermal utility only” gate, reopen a closed recipe or launch runs.

## Reading record and verification

Fresh targeted reading: MAC-VO method/discussion/AppendixA; MAC-I² method,
experiment/ablation/conclusion and AppendicesB,E–G; MACThermal3.2–3.5/6 plus
selected dataset timing context; DNLS discussion/AppendixB. Not full-paper,
full-thesis or official-system reproductions. Paper figures were not visually
reanalyzed. Thermal thesis PDF has85pages,17420576bytes and SHA256
`91d680477b21d2292e81877df8ed05dfef0dbe829784857baa1581add645d6a4`.
PDF/text remain outside the repository and backup packet. MomentBA is abstract-only.
