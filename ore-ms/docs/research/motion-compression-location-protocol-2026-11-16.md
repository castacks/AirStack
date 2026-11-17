# Locate motion compression in cached correspondences and essential matrices

16 November 2026 America/New_York. Follows the
[rotation diagnostic](rotation-collapse-findings-2026-11-16.md).
Known exposed outcomes; finite diagnostic, no new matcher/pose solver or policy.

Use all192FlashLight and236SmokeRoom frozen both-camera≥2° moving anchors,
both SIFT/learned and RGB/thermal:1712rows, including every failed proposal.
Preflight all5516original prediction rows/unique keys and source locks; bind own
virtual K from the actual comparison config and calibration, matching arrays,
essential/pose masks and exact own target_from_source reference transforms.
Reference-free feature stage uses predicted data only; labels/reference motion
values join after feature/candidate lock. No new inference, RANSAC, fitting,
threshold search, clock/axis adjustment or output replacement.

Reuse existing Comparison.camera, bearing_rotation.bearings and angle helper.
Earlier rotation/geometry runners hard-code different inputs/contracts; a small
adapter is needed to inspect cached E candidates and paired match masks here.
For every moving row, report all accepted, cached essential-mask and pose-mask
subsets, including empty subsets with null statistics (not zero residual).
Masks must exactly match stored correspondence counts and cheirality counts.

Reference-free extraction: unit bearings from stored virtual points, median and
p90 raw angular bearing displacement; estimated-R angular warp residuals and
cached-E target epipolar-plane residuals for each subset. Decompose finite3×3 E
using OpenCV's two rotation candidates; lock their matrices/amounts and original-R
candidate agreement. Invalid/multiple/missing E remain explicit. Do not regenerate
matches, E, cheirality masks, translation or recovered outputs.

Reference join: calculate full rotation errors of both E candidates against the
own supplied reference; report whether either meets the original1°/5° bounds.
Only these label-assisted candidate counts diagnose disambiguation; they are not
an achieved oracle policy. A wrong recovered rotation with both E candidates
wrong cannot be repaired by choosing the other branch of that fixed E.

Compute reference-R warp residual and reference E=[t]×R plane residual using
own camera origin/directed relative translation. Interpret reference E only on
existing paired-displacement≥1cm eligibility; retain other intervals as null.
For E and Eref, use arcsin(|b_target·E b_source|/||E b_source||), degrees;
norm≤1e-12 is numerically degenerate and excluded with an explicit valid count.
R-warp residual is angle(b_target,R b_source). Pure rotation warp is not a
prediction of translated-image flow; cancellation/parallax can make small ray
motion compatible with a larger true rotation. No geometric cause or match truth
is certified by nominal supplied-reference epipolar consistency.

Per record/frontend/camera report complete moving counts, output/candidate
availability, exact old-error parity, candidate1°/5° counts, original wrong outputs
with an alternative correct branch, and median/quartiles of per-pair median
residuals. Never pool residuals over matches (avoid dense-pair domination).
Also summarize each original learned movingR5 correctness cell, with missing
outputs separate. Continuous statistics have n/missing counts. No new quality
cutoff, feature ranking, selection policy or re-estimated pose is evaluated.

Independent audit uses separately implemented ray normalization, trace/skew angle,
SVD essential decomposition, own skew product and aggregate quantiles. Verify full
populations, matrix candidate sets up to order, source/mask/frame binding and all
per-pair/subset/aggregate fields. Synthetic controls include translated static3D
points consistent with E despite nonzero pure-rotation residual, swapped target
matches, E rotations, and empty/degenerate masks. Rehashed semantic corruptions
must fail. Save all failed attempts; no post-result protocol/source rescue.

Stop after locating what these fixed caches can establish. Distinguish matching/
essential-estimation effects from recovery-branch effects; they need not identify
one causal component. Preserve nominal reference/physical/training limits, prior
uncertainty/closure/geometry nulls and both recording controls. Save verified local
incremental evidence with parent pins. No new inference, trained gate, author
outreach, Box upload, Git push or automatic commit.
