# Essential-inlier motion and spatial mixture protocol

16 November2026 America/New_York; follows the
[compression-location diagnostic](motion-compression-location-findings-2026-11-16.md).
Known exposed development data; no new inference, solver/refit or camera policy.

Use all192FlashLight and236SmokeRoom moving anchors, both cameras/frontends,
1712rows. Preflight parent locks/audits, original counts/masks/coordinates, own
camera calibration and previous24locked visual sample IDs. Inherit old correctness
cells; no new error threshold, window, image sample or causal scene label.

For each row use five fixed masks: all accepted matches; original essential-mask;
its accepted-set complement; original cheirality-mask; essential minus cheirality.
Require cheirality⊆essential and exact old counts. Preserve empty subsets explicitly.
Do not estimate new E, select a new subset or recover another pose.

Unit-bearing angle change is computed using the existing helpers. Define a
near-stationary correspondence as ray change≤0.1°. This is a new fixed descriptive
per-match assay, distinct from the prior≤0.1°estimated-pose assay. No alternative
cut is tried and no camera/gate is chosen. For every subset report count, low-motion
count/fraction, and25/50/75%angular-change quantiles. Also retain the two-by-two
accepted-motion versus essential-retention counts and low/high retention fractions;
zero denominators yield null, never zero success.

Project stored virtual points back to native images using each recording's own
published calibration. Reuse Comparison.camera and existing spatial metric helper
(spatial's8×8cells,≥3points per occupied cell, hull/entropy/top4 fractions, boundary
roundoff convention). Generic inverse projection composes the same published
pinhole/MEI model; do not borrow SmokeRoom file association for FlashLight.
For every mask and both endpoints report all spatial fields. Retain accepted vs
essential vs cheirality support separately; wide support is not true matching.

Lock all source-only feature rows before joining prior reference correctness cells
or candidate/reference values. Compare all frozen per-record/frontend/camera
cohorts and original learnedR≤5°cells. Summary is quartiles of per-pair scalars
with n/missing, not pooled match weighting. Full coordinate/grid/motion arrays and
masks remain traceable to original row indices; save per-pair metrics/count grids
without duplicating raw parent arrays. Do not infer physical equal FOV/area from
native-grid fractions. Prior EXP078coverage and EXP112geometry nulls remain;
this is diagnosis, not a renewed scalar-quality selection test.

Visuals use the exact prior24sample anchors (all48camera source images) and their
own locked gray file SHA/time. Show accepted source locations in gray, essential
locations colored by the fixed near-stationary assay, with cheirality outlines.
No point subsampling, image substitution, segmentation or density/temperature
labels. Source-only overlays cannot establish object identity or correspondence
truth. Boundaries/projection roundtrips must pass before overlays are rendered.

Independent audit separately reconstructs masks/contingencies, unit-ray angles,
parameterized native projection,8×8counts/hull/entropy, all feature and aggregate
fields, old sample IDs and rendered source hashes. Synthetic controls cover
stationary/moving mixtures, empty groups, denominator nulls, native roundtrip and
full-record calibration association. Rehashed semantic corruptions must fail.

Save all attempts, source/features/locks/summaries, overlays, audit and verified
local incremental packet. Stop after quantifying which fixed subsets support E;
correlation or visual concentration does not prove an algorithmic failure cause.
No fitted rejection threshold, output replacement, training, new pose inference,
Box upload, author outreach, Git push or automatic commit.
