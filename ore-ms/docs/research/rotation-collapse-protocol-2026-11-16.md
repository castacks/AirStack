# Cached rotation-collapse diagnostic protocol

16 November 2026, America/New_York; base4408b7fb8 plus frozen
[failure-stratum analysis](paired-failure-strata-findings-2026-11-16.md).
Hypothesis motivated by exposed inspected failures, not a blind validation study.
Does a supported near-identity rotation explain a substantial portion of wrong
moving proposals? This describes predictions, not a deployable camera selector.

Use every541FlashLight and838SmokeRoom anchor, both SIFT and learned and both
cameras:5516rows. Inherit common-reference and both-camera≥2° moving cohorts;
all statuses/references remain explicit. Preserve frozen R≤1°/R≤5° correctness
and the prior learned moving four-cell strata. No new images, inference, pose
solver, clock/axis/bias fit, score threshold rescue or window selection.

Reuse reference_contract_angles.degrees for estimated rotation amount. Its
hard-coded old NGCLAB runner lacks these restored supplied-reference errors;
a small adapter is necessary. Validate all status-ok R matrices: finite3x3,
orthogonality/determinant within1e-5 ofSO(3). Unsupported/rejected proposals retain
null estimated angles, even if their raw rejected row contains an intermediate R.
Lock estimated-angle rows before parsing supplied-reference score values.

Define near-identity prospectively as estimated angle≤0.1°. This single descriptive
assay cut is not tuned or used to reject/select a camera. Report continuous
estimated/reference-angle and full-geodesic-error quartiles as well. For own
reference amount≥2°, report estimated/reference amount ratio; otherwise ratio null.
Near-identity is not automatically failure: static estimates and reference motions
between2° and5° can satisfy coarse correctness. Preserve those cases.

Join own nominal supplied-reference rotation amount and full orientation error.
Identity error equals reference angle, including unavailable-prediction controls.
For each record/frontend/camera report all-declared, common-reference and existing
paired-moving cohorts: anchors, outputs, reference-eligible/compared counts, near-
identity outputs, R1/R5 successes, wrong supported outputs and near-identity subsets
of those errors, identity R1/R5 counts and continuous quartiles. Missing outputs
are distinct from wrong supported outputs. Prior learned R5 correctness cells are
additional fixed descriptive groups, retaining both-supported markers.

For every compared output verify |estimated amount−reference amount|≤full error
and full error≤estimated amount+reference amount (1e-5° arithmetic tolerance).
Magnitude agreement does not certify orientation: retain wrong-axis synthetic
controls. Report signed full-error minus identity-error quartiles; do not choose
an outcome-dependent tolerance or call a scalar ratio calibrated risk. Fixed
camera policies remain unchanged; no label-assisted hybrid output is scored.

Independent audit must use a separate trace/skew atan2 angle formula, reconstruct
all source binding/status/null/threshold/identity/count/quartile/stratum fields
and reject rehashed semantic corruptions. Synthetic controls cover identity,
small true motion, missing estimates, exact boundary, and equal-angle wrong axes.
No physical pose/reference truth is established by arithmetic checks.

Stop after diagnostic findings and a verified local incremental packet. A positive
near-identity finding does not establish the cause (correspondence, geometry,
calibration or reference), a new detector, or usefulness of preferring larger
rotation. No fitted threshold, gyro selector, training, Box upload, author outreach,
Git push or automatic commit follows from this diagnostic. Keep all failed
attempts and both records/frontends; do not pool accuracy across records.
