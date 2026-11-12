# Return to the RGB–thermal VO research question

12 November 2026. Branch `ore-ms-orb-development`. Completed paper/source and
experiment review; proposed experiment below is not executed or numerically
frozen. No new inference, fitting, reference scoring or reserve access.

## Decision and contribution target

Defer ACVNet. It could establish a stronger thermal depth control, but we have
not shown that better standalone depth produces useful RGB–thermal motion
constraints. Do not resume the closed preprocessing, matcher, covariance-weight
or bearing-rotation configurations as automatic follow-ups.

The target remains: do RGB and thermal measurement uncertainties identify useful
complementary constraints, and does using them improve a shared motion estimate
beyond RGB alone? Better within-camera ranking, lower thermal depth error and
better thermal-only VO are supporting evidence, not substitutes for this target.
Thermal need not win on average to help where RGB fails.

**Selected next hypothesis for feasibility:** learning measurement weights through
one shared RGB–thermal metric-motion objective can produce more useful relative
measurement influence than independently learned pixel-error uncertainty and
post-estimation pose combination. Freeze both correspondence/depth means to
isolate that hypothesis. This is an unverified hypothesis, not a novelty claim
or evidence that weights alone can repair systematic errors.

## What the papers actually supply

| Source | Author mechanism / inspected scope | Project implication and boundary |
|---|---|---|
| [MAC-VO v2](https://arxiv.org/html/2409.09479v2), III-A–D | Flow-error NLL, uncertainty-based selection and geometry-aware covariance in pose optimization; relevant method sections rechecked. | Foundation for retaining measurement geometry. Does not establish RGB/thermal cross-calibration. |
| [MACThermal thesis](https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments) | Earlier local thermal preprocessing/training reading retained; no new thesis reading here. | Thermal adaptation is already baseline context. Another thermal component change needs an explicit downstream hypothesis. |
| [MAC-I² v1](https://arxiv.org/html/2609.07116), IV-C–E | Covariance-weighted visual/inertial optimization; visual factors constrain completed relative poses. Relevant sections inspected, not full-paper/code reproduction. | Pose-level fusion is a meaningful baseline, already approximated/tested here. It is not evidence that camera confidences transfer across modalities. |
| [DNLS v2](https://arxiv.org/html/2305.09527v2), III, V, Appendix B | Learns positional covariance through relative-pose optimization, with supervised rotation or cycle/anchor losses. The noise scale is unobservable and pose benefit does not guarantee calibration. | Supports changing the learning target. Two independently trained DNLS models do not automatically provide comparable fusion covariances. Its monocular rotation objective is not our metric RGB–thermal estimator. |

DNLS official source was cloned and inspected at
`73d2e5c8dcd9afe2e6a82db3425d7d2704c54ae0`. README, active Theseus optimizer
construction and backward-mode handling were inspected. The release uses
Theseus and PNEC/opengv bindings; no installation or execution was attempted.
A future local implementation would be DNLS-inspired, not its reproduction.

## Comparison with completed work

| Completed family | Relevant result | What this hypothesis changes |
|---|---|---|
| EXP095–109 covariance-head fits | Pixel NLL/trace-risk training and expanded synthetic RGB ranking do not establish thermal or fusion benefit. | Train against shared motion quality, not detached per-pixel error alone. Merely fitting another confidence head is insufficient. |
| EXP009–012 and EXP079–086 | Covariance influence, channel/mass, geometry and solver behavior already investigated; rotation/translation tradeoffs persist. | Keep these as prerequisites. No renamed residual, scalar cue or weighted-solver repair proposed. |
| [Native information weighting](native-information-weights-findings-2026-11-09.md) | Recording-excluded scale-calibrated full pose combination increases aggregate mean rotation/translation errors 8.95%/3.75% against RGB. | Place the learning target before separate camera poses are completed; compare with pose combination rather than claiming it missing. |
| [Bearing transfer](bearing-rotation-transfer-findings-2026-11-12.md) and [longer NEC](unweighted-longer-findings-2026-11-12.md) | Fixed covariance and replacement-flow rotation routes fail promotion. | Do not reopen these configurations. A differentiable shared metric-motion objective would be a different mechanism, not a repeat rain assay. |
| [Raw thermal refinement](radiometric-native-findings-2026-11-12.md) and [MS² transfer](ms2-depth-transfer-findings-2026-11-12.md) | Exact component configurations fail their declared gates. | No automatic frontend swap or depth-model ladder. |

The [opportunity study](thermal-opportunities-findings-2026-11-09.md) found only
21 simultaneous thermal translation/rotation wins among 1,735 paired native
intervals. Translation-only selection has more opportunity but a rotation cost.
These supplied-reference development observations motivate testing complementary
measurement constraints; they neither prove such constraints exist nor bound
what a joint estimator could achieve. This review has not established that no
historical code ever pooled factors; novelty must not be inferred from filenames
or this targeted inventory.

## Concrete next experiment: shared-motion learning feasibility

Use the existing stereo geometry to estimate one metric body motion from both
cameras' own temporal correspondences and stereo measurements. Cross-modal pixel
matching is not required. Each modality keeps its own intrinsics, stereo baseline,
source landmarks, observation endpoints and camera-to-body transform. Do not
concatenate UV/disparity arrays under one camera matrix.

The intended objective is a sum of each camera's whitened measurement residuals,
all depending on the same body-motion state. Learn only an explicitly declared
measurement-weight/covariance branch through the fitted motion; freeze matching,
depth, source points, selection, support and all mean-network weights. Retain
an explicit global weight gauge and inspect relative RGB/thermal influence.
Fixing a gauge removes a mathematical ambiguity; it does not calibrate uncertainty.
Conditioning on frozen landmarks also does not model all landmark uncertainty.

**Immediate finite deliverable:** one isolated differentiable joint-factor
prototype and its forward/gradient qualification, before real-data training.
It must not replace the native backend. The inspected native
`TwoFramePGO/Graphs.py` marks covariance access with `no_grad`/`inference_mode`;
`Optimizer.py` performs an iterative native solve. That path has not been
qualified as a differentiable map from covariance to fitted pose. Removing a
single decorator is not a verified training solution.

Qualify a fixed suite of synthetic calibrated stereo rigs: noiseless recovery,
anisotropic error in either modality, identical duplicated observations,
modality-order permutation, rigid change of body coordinates, near-degenerate
geometry and invalid/nonfinite observations. Compare pose-loss gradients to
central finite differences on well-conditioned cases; retain rejected and
ill-conditioned cases. Verify both modalities receive finite gradients and all
frozen means/parameters stay unchanged. Freeze the exact case count, seed,
iterations, dtype, damping, tolerances and source hashes before executing this
suite. No hyperparameter or dependency-port sweep. Failure means no training
under that implementation, with the numerical cause reported.

**Temporal precondition:** use actual camera endpoints. Neither nearest frame
index nor reference-assisted bridging makes asynchronous intervals identical.
For the real-data stage, either establish the declared simultaneous-pair
approximation and its limits from the existing capture contract, or explicitly
model a shared trajectory at native timestamps using sensor-only information.
If neither is supported, stop this real-data configuration; synthetic mechanics
cannot qualify a physical fusion claim. Do not reopen an unbounded calibration
or data-acquisition investigation to rescue it.

## Required real-data comparison if feasibility passes

Freeze a single model, optimizer budget, loss, initialization, masks, fallback
and stopping rule before fitting. Keep complete recordings together. Existing
exposed day/night/rain recordings can support recording-excluded development
folds; none becomes a fresh test recording through that partition. Identify
actual available captures and their exposure before assigning a future test.
No reserved payload is opened by this decision.

Controls on the same solver, geometry and available observations:

1. RGB-only and thermal-only, plus unchanged native experts as context.
2. Shared solver with fixed weights and with original predicted covariances.
3. Independent-pose combination using the existing declared baseline.
4. Shared solver with pose-trained measurement weights, the sole candidate.

Report each recording's translation and rotation mean/RMSE, tails, complete
attempt/availability counts and both common-support and all-attempt outcomes.
Retain failed intervals; no reference-filled gaps. Training uses a fixed
translation/rotation loss with scales set from training only, never target-fold
references. Reference access is confined to training loss and offline scoring.
No image/snippet reference scale correction is allowed in metric VO outputs.

The provisional advancement rule is at least 5% lower translation mean and RMSE
than RGB-only and each shared-solver control on every evaluated recording,
no worse rotation mean/RMSE and no coverage loss. These are design thresholds,
not a completed execution freeze or a change to older experiments' rules.
Also compare against the pose-combination control. A favorable aggregate alone
does not promote a recording-dependent result. On failure, close the exact
configuration without a head/loss/weight sweep. On success, evaluate contiguous
trajectories and new recording-disjoint data before claiming broader VO benefit;
report alignment conventions and retain gaps rather than stitching references.

Pose-trained weights are task weights until separately assessed for calibration.
A useful camera fusion result would still not establish generalization to IMU,
lidar or another modality. Existing reference/timing/unknown checkpoint-overlap
limits remain attached to every claim. No new accuracy result follows from this
review.
