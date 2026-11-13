# MSO method overlap and contribution boundary

12 November 2026, America/New_York. Focused primary-source review; no new
experiment, training, baseline reproduction or universal novelty claim.

## Finding

MSO directly overlaps RGB–thermal covariance-weighted state estimation. Its
full thesis chapter specifies image-space feature uncertainty, RGB/thermal
reprojection weighting, and adaptive factor removal in sliding-window
visual–thermal–inertial bundle adjustment. This is stronger evidence of overlap
than the project summaries previously inspected. The inspected chapter does
not demonstrate statistically calibrated cross-modal uncertainty or prediction
of the relative geometric error of RGB versus thermal. Those are candidate
study boundaries, not verified novel contributions.

## Sources and reading depth

- Shibo Zhao, *Resilient Spatial Perception for Autonomous Robots*, CMU-RI-TR-26-26,
  April 2026. [Official publication](https://publications.ri.cmu.edu/resilient-spatial-perception-for-autonomous-robots)
  links the [defense folder](https://drive.google.com/drive/folders/1Tm0qfmQLDUa7r_A2cI26WzzoGaZSBlkb).
  Read complete Chapter 5, printed pp68–84 / PDF pages69–85, including methods,
  equations5.2–5.10, evaluation, tables5.1–5.5 and limitations; also relevant
  Chapter7 sections7.3.1 and7.4.2. Not a full reading of every thesis chapter.
  File `shiboz_phd_ri_2026.pdf`, Drive ID `1wNZVMuBtNxPvYy2CtGa4e7UJEC2ZMz1B`,
  184 PDF pages; SHA256 `bd872e8c7ed112c6be8ee5b98d8a80ad7d0853aca09b5f8f8f76e09d91324d0e`.
  PDF metadata incorrectly names another author; the title page and institutional
  record identify Shibo Zhao. Temporary reading copy stays outside the repository.
- [Author homepage](https://shibowing.github.io/) lists MSO as under review, with
  video, and no linked method/code release in that entry. Chapter5 describes its
  source as under review2025 and promises future open source. These are dated
  source statements, not confirmation of current publication status.
- [IROS2026 DFR workshop](https://jonasfrey96.github.io/data_in_field_robotics/)
  lists accepted submission17, *MSO: Uncertainty-Aware Multi-Spectral Inertial
  Odometry for Extreme Environments*, with a changed author list. The venue is
  non-archival and permits both short papers and previously published posters;
  the listing alone does not establish this submission's type or method changes.
  No manuscript was obtained; OpenReview API queries returned403. Its contents
  and relationship to Chapter5 remain unresolved.
- The [Super Odometry code link](https://superodometry.com/) resolves to
  [superxslam/SuperOdom](https://github.com/superxslam/SuperOdom/tree/f10e65cd50007767b22e4c401689665e20d827d6).
  Inspected pinned `ros2` README and complete117-entry tree. README describes a
  slim LiDAR/IMU implementation; this is not a verified MSO RGB–thermal release.
  Other branches' implementation bodies were not reviewed. Bounded repository
  searches and public `shibowing`/`castacks` listings found no identifiable MSO
  release. This is not evidence that no public/private implementation exists.

## Comparison with the original research question

“Present” refers to Chapter5, “not demonstrated” to its inspected methods/results;
“unresolved” marks missing implementation or later-version evidence.

| Question | MSO evidence | What this means for ore-ms |
|---|---|---|
| Per-feature covariance | Present: radiometric patch tracking with normalized LSSD residuals, inverse Gauss–Newton Hessian and rotated image covariance, eqs5.3–5.9. | A common uncertainty representation across RGB/thermal is established prior work. A learned covariance source differs in mechanism, but changing its source alone does not establish novelty. |
| Coordinate and scale comparability | Same image-space covariance role in RGB/thermal reprojection factors, eq5.10. Statistical agreement of scales across cameras is not demonstrated. | Shared residual dimension does not prove calibration. Pixel covariance also depends on projection and resolution; a common geometric error target needs an explicit transform and units. |
| Covariance to geometry/pose | Image covariance weights tightly coupled sliding-window BA. Section5.6 explicitly limits uncertainty to image space, excluding depth, landmarks and poses. | Weighting a pose estimator is established; explicit calibrated landmark/pose covariance is a different claim. MACThermal/MAC-I² already cover related propagation, so this limitation alone is not a novelty proof. |
| Statistical calibration | No coverage, standardized error, NLL or calibration reliability evaluation demonstrated in Chapter5. Photometric normalization/calibration is not statistical covariance calibration. | Potential comparison axis. Existing ore-ms EXP069/070 already examine coverage/NLL and recording exclusion; do not propose them as untouched. |
| Relative RGB/thermal error prediction | Figures5.6/5.7 show feature counts, reliability-based selection and changing modality contributions. No quantitative ranking test against a common per-observation/per-interval geometric error target demonstrated. | The distinction between confidence-driven adaptation and verified relative-error prediction is relevant to the original question. Incremental fusion benefit is a further target, not guaranteed by ranking alone. |
| Transfer | Multiple datasets, robots and degraded environments evaluated. No held-out recording calibration/ranking protocol demonstrated. | Broad deployment evidence is present; transferable statistical comparability is a separate test. ore-ms EXP060/070 already contain retrospective recording-exclusion controls. |
| Fusion ablation and availability | Joint system and RGB/thermal-only evaluations exist. Table5.3 reports separate RGB and thermal runs; SubT Table5.2 compares MSO/ROVTIO. No matched fixed-input uniform-versus-covariance joint ablation demonstrated. Failures and completed trajectories are discussed. | Generic nighttime fusion and modality ablations overlap. Existing ore-ms EXP009 and nominal thermal fixed/cov controls already isolate weighting; latest learned/windowed pilots have matched RGB controls and negative results. |
| Exact implementation | Code-release promise and a related LiDAR/IMU repository; verified MSO code unavailable in this review. | Noise scaling, Hessian regularization, feature admission and factor-removal thresholds remain unresolved. A chapter-inspired baseline must not be called an author-system reproduction. |

Equation5.7 gives inverse Hessian uncertainty, but the chapter does not provide
a complete statistical noise-scale/regularization specification. Its SE(2)-to-2D
covariance presentation also leaves extraction details to implementation. These
are reproducibility questions, not proven mathematical errors. Older CV/defense
summaries mention broader uncertainty hierarchies; Chapter5's explicit limitation
bounds the present finding, without excluding richer unpublished/later versions.

## What the reported results establish

MSO's experiments support adaptation and robustness in degraded scenes. They do
not imply universal accuracy gains: Table5.2 beats ROVTIO on4/6 sequences, while
ROVTIO is better on runs10/11. Table5.3's two modality rows must not be read as
one measured joint-fusion result. The mixed-condition400m demonstration reports
about0.825m RMSE against Super Odometry reference; reference quality differs from
independent instrumented ground truth. The wildfire run reports4.08m endpoint
error over172.87m; its caption says2.6% while section5.5.3 gives2.36%, consistent
with direct division. These are author results, not local replications.

## Existing ore-ms evidence and remaining claim

[Prior-art correction](research-relevance-and-prior-art-2026-11-12.md) records
EXP009 and nominal thermal weighting results. [MS² transfer findings](ms2-transfer-findings-2026-10-27.md)
record error-target dependence (EXP061), scale transfer (EXP060), coverage
(EXP069), and bias/scale/NLL transfer (EXP070). Better thermal NLL in EXP070 did
not yield a better camera selector: its fitted selectors worsened every capture.
The [learned-weight](shared-weight-learning-findings-2026-11-12.md) and
[persistent-window](persistent-tracks-windowed-geometry-findings-2026-11-12.md)
recipes also failed all three declared gates. These close those exact recipes,
not the general possibility of thermal benefit.

A defensible candidate contribution is an evidence-backed account of **when
RGB/thermal uncertainty is comparable for a specified geometric error target,
when it predicts relative reliability, and when that prediction improves fusion
on unseen recordings**. Existing results already show several separations between
these claims. A new scalar, another dataset, covariance weighting itself, or the
absence of one MSO ablation is insufficient evidence of novelty.

The next decision should use the completed inventory to identify one additional
claim beyond those existing results, and establish its baseline against the
applicable MSO version. Exact MSO reproduction requires a verified implementation
or enough author-supplied details; the later workshop manuscript may change this
boundary. No further fitting, threshold search or replay is selected by this
review. No author was contacted. The full-method review of the thesis is complete;
the later-manuscript/code part remains explicitly unverified.
