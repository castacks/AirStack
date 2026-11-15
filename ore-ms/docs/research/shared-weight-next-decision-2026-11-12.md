# Next decision after the nominal joint-factor benchmark

12 November 2026, America/New_York. Branch `ore-ms-orb-development`, Git baseline
`4f28219d9`; recovered prior result packet identifies `d3f2eafee`. That commit
is not available from origin; its artifact manifest, rather than Git ancestry,
authenticates the recovered files. Status: REVIEW_COMPLETE_PROTOCOL_DRAFT.
No new method accuracy experiment or weight training has run.

## Evidence established in this session

The [recovered benchmark](nominal-joint-motion-findings-2026-11-12.md) is closed.
All 974 manifest file hashes match. Its eight mechanics tests pass in an isolated
NumPy 1.26.4 / SciPy 1.13.1 runtime. A separate saved-output audit rerun passes
13,374 checks and 1,780 objective reconstructions, preserving all three failed
advancement decisions. These are baseline replay checks, not new accuracy results.
All six saved camera streams contain finite source/target UV, depth, disparity
variance and endpoint 3D covariance fields. Field availability does not qualify
covariance positive definiteness, provenance, independence or a new adapter.

## Candidate triage

MAC-VO v2, III-D equation 4, combines both endpoint spatial covariances.
The recovered joint UV/disparity solver conditions on fixed source landmarks.
This is a modeling difference, not evidence of the cause of its failure.
A two-endpoint Gaussian registration implementation already exists in
`tools/pose_covariance_replay.py`, with covariance-rotation derivatives and
log determinant. It was used in the November 9 pose-covariance work. Do not
present adding source uncertainty alone as a new research contribution or
repeat that existing prototype under a new name.

DNLS v2, method and Appendix B, learns feature uncertainty through pose loss.
Its reported monocular rotation mechanism is inspiration, not evidence that a
metric RGB–thermal extension works. Its sparse gradients and generalization
limitations apply to interpreting any local follow-up. This session rechecked
those paper sections; the existing pinned official-code inspection is historical,
not a fresh code execution.

## Selected hypothesis and bounded next deliverable

Test whether **measurement-specific, task-trained weights transfer across these
exposed recordings and add value beyond identically trained RGB-only weights**.
This remains untested by the fixed/original-covariance joint benchmark. The
mechanism changes the training target, not frontend means or source support.
A learned joint system beating native RGB alone would be insufficient: an
improved RGB backend could explain that gain, as the recovered rain result shows.

The immediate implementation deliverable is a qualified differentiable version
of the existing conditional joint objective, plus an identically parameterized
RGB-only control. Keep the conditional-landmark and nominal-time assumptions
explicit. Do not simultaneously add a source-covariance model, matcher, robust
kernel, clock correction, depth adaptation or pose-combination method.

Before any fitting, write a numbered notebook design and freeze source/input
hashes, exact head features/parameter count, gauge, loss units, solve budget,
stationarity tolerance, derivative acceptance tolerances and failure handling.
The following experiment design fixes the scientific comparisons; these missing
numerical details mean this note is not an executable scientific freeze.

1. Reproduce all original-covariance RGB and joint predictions at zero adapter
   output, including the one declared rain RGB fallback. Preserve original
   fixed-budget receipts. A changed optimizer is a separate control, never
   silently treated as exact baseline parity.
2. Qualify real-packet weight-to-pose derivatives on a fixed first common
   interval per recording and both camera branches. Use independently solved
   finite differences with stationarity checks; retain every probe. If the
   original fixed-budget solver is not stationary enough, explicitly distinguish
   differentiation of the unrolled algorithm from differentiation of an optimum.
   The earlier synthetic implicit-gradient result does not qualify this new path.
3. Only after those checks pass, use three recording-excluded development folds:
   train day+night, evaluate rain; train day+rain, evaluate night; train night+rain,
   evaluate day. Use the same fixed learning budget for RGB-only and joint heads.
   Fit feature normalization only on each training fold. No target-pose labels or
   target loss may influence fitting, early stopping, checkpoint choice or features.
4. Retain frozen native RGB, original RGB-cov, original joint-cov and joint-fixed
   controls. Report learned RGB-only and learned joint separately. Lock predictions
   for all folds before scoring. Retain all 297 slots and the unchanged support
   and fallback policy. Report per-record translation and rotation mean, RMSE,
   p95, coverage and all-method common support; never promote a pooled-only win.
5. Advancement requires learned joint to reduce translation mean and RMSE by at
   least 5% against BOTH learned RGB-only and original RGB-cov on every recording,
   with no rotation mean/RMSE or coverage regression. Native RGB and original
   joint-cov remain mandatory context. Failure closes this exact learned recipe,
   without feature, loss-weight, threshold, recording or checkpoint searches.

This design permits a later finite learning experiment; it does not authorize
silently treating a failed qualification as a pass. No fresh data or protected
reserve access is needed. All folds remain development-exposed; nominal supplied
pairing, supplied pose references and unknown training overlap preclude claims
of verified physical accuracy, fresh-test generalization or calibrated covariance.
No new experiment result or novelty claim is established by this review.

## Primary reading

- MAC-VO v2, III-D and equation 4: https://arxiv.org/html/2409.09479v2
- DNLS v2, method and Appendix B: https://arxiv.org/html/2305.09527v2

The working reference registry records the reading event. Existing publication
metadata and BibTeX entries are unchanged.
