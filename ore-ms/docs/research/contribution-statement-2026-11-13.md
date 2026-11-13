# Proposed contribution: limits of RGB–thermal uncertainty transfer

13 November 2026, UTC. Evidence synthesis only. No new inference, scoring,
training, experiment selection or claim of established publication novelty.

## Working title and contribution statement

**From uncertainty ranking to RGB–thermal odometry: an empirical study of transfer limits.**

On the evaluated RGB–thermal pipelines and recordings, useful within-camera
uncertainty ranking does not establish reliable cross-camera geometric-error
selection. Improved native-disparity likelihood likewise does not establish
improved camera selection. The tested weighting, learning and shared-window
methods have not demonstrated consistent downstream pose gains against matched
RGB-only controls. We characterize these distinctions with fixed-target controls,
recording-exclusion comparisons and explicit accuracy/availability accounting.

This is a proposed empirical contribution built from completed evidence. It is
not a successful new fusion algorithm or a general impossibility result. Its
incremental value relative to the full applicable prior work remains unverified.

## Evidence supporting each part

| Claim | Completed evidence | Exact boundary |
|---|---|---|
| Within-camera ranking need not transfer to cross-camera selection | [MS² synthesis](ms2-transfer-findings-2026-10-27.md): EXP046–054 within-camera controls; EXP060 recording-excluded scale test; EXP064 frozen prospective camera-selection suite | EXP064 meets its per-capture criterion on rain but fails on day/night. These are supplied-reference 3D point losses, not pose errors. |
| Statistical fit and selection utility are different endpoints | Same synthesis: EXP069 coverage and EXP070 bias/scale comparison | Thermal disparity NLL improves for bias-plus-scale versus scale-only in all four retrospective folds. Both fitted selectors worsen risk versus the original on every capture; bias-plus-scale versus scale-only selection results are mixed. Do not conflate the comparison baselines or claim full calibration. |
| A pose-level uncertainty bridge needs its own validation | [Fixed-native weighting](native-information-weights-findings-2026-11-09.md) | With native means fixed, the primary calibration increases equal-record rotation/translation errors by8.95%/3.75% versus RGB on common support. This closes that control, not pose covariance generally. |
| Learning weights does not by itself demonstrate thermal contribution | [Matched weight learning](shared-weight-learning-findings-2026-11-12.md) | Six fixed40-update heads; all three gates fail against equally trained RGB-only and original RGB-cov. Exposed recording-exclusion development, not fresh-test generalization. |
| Persistent tracks and joint depth refinement do not by themselves resolve the observed regression | [Five-frame geometry](persistent-tracks-windowed-geometry-findings-2026-11-12.md) | Joint versus matched RGB-window translation means worsen19.74% day,18.51% night,0.55% rain; both variable arms retain all240 planned intervals. This specific covariance/solver/window recipe is closed. |
| Availability and accuracy are distinct | [Whole-record findings](decisive-uncertainty-findings-2026-11-06.md) | The declared evaluation ends STOP_INSUFFICIENT_EVALUATION; coverage/headroom requirements fail. Do not report a valid positive whole-record adaptive result. |

These studies use different populations, error targets and estimators. Their
results cannot be pooled as one controlled causal experiment. In particular,
EXP070 does not prove that miscalibration causes the later pose regressions.

## What distinguishes the proposed study from the inspected MSO chapter

The [MSO method review](mso-method-overlap-review-2026-11-12.md) establishes that
RGB/thermal covariance-weighted reprojection factors and adaptive factor graphs
already exist. The proposed study asks whether uncertainty's claimed reliability
meaning survives changes in camera, error target and recording, and separates
that question from downstream estimator benefit. Statistical cross-modal
calibration and quantitative relative-error ranking were not demonstrated in the
inspected MSO Chapter5. That bounded source finding is not a universal novelty
claim. Its later workshop manuscript and exact implementation remain unverified.

MACThermal already uses thermal uncertainty for geometric weighting; MAC-I²
already links feature and pose uncertainty in visual–inertial fusion. Their
[relevance and limits](published-covariance-claims-2026-11-12.md) prevent presenting
covariance propagation or pose-graph weighting itself as the contribution.

## Claims the current evidence does not support

- Thermal is generally worse at night, or RGB is always the best modality.
- Calibrated RGB–thermal covariance has been achieved, or covariance quality
  alone explains the measured pose regressions.
- A validated method that predicts the incremental benefit of adding thermal.
- A complete MSO reproduction, a new state-of-the-art result, or verified novelty.
- Independent physical ground truth, verified cross-camera exposure timing,
  unseen checkpoint training membership, or independent fresh-record performance.

Supplied reference products, nominal pairing, temporal dependence and previously
exposed development recordings remain material limitations. Numerical/source
integrity audits do not remove them. Preserve positive rain and thermal-win
subsets alongside the negative aggregate/transfer findings.

## Remaining research decision

The evidence supports a bounded characterization report now. A method paper
would additionally need a distinct mechanism and evidence that its deployable
signal predicts incremental thermal benefit against matched RGB controls.
Neither has been established by renaming existing calibration/ranking studies.

Before selecting more experiments, finish the version-specific prior-art
comparison using an accessible later MSO manuscript/implementation and assess
whether this empirical characterization is sufficiently incremental. If those
sources remain unavailable, record that limitation rather than filling it with
a novelty assertion. Closed recipes stay closed. This note consumes no reserved
recordings and does not assume older supposedly untouched captures remain
untouched; the November6 whole-record attempt already exposed its two candidates.
