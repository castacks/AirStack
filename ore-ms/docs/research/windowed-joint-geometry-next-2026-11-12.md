Latest execution: [persistent-tracks-windowed-geometry-findings-2026-11-12](persistent-tracks-windowed-geometry-findings-2026-11-12.md). The explicitly authorized persistent-track replay resolves the old packet admission stop, then the fixed five-frame geometry pilot fails all three accuracy gates. Proposal below is historical; no repeat/retune follows.

# Proposed next hypothesis: shared multi-frame geometry

12 November 2026, America/New_York. Status: PROPOSAL_ONLY_NOT_EXECUTED.
The user asked what follows the closed shared-weight recipe.

## Recommendation

Test a short-window shared RGB–thermal motion estimator that jointly optimizes
camera motion and landmark inverse depths, constrained by the original stereo
measurements. Use original frontend means and fixed original covariances; no
weight-head or new frontend training. Hypothesis: repeated observations can make
thermal constraints useful when uncertain source geometry is allowed to adjust.
This is an unverified alternative mechanism, not an explanation established by
the failed weight experiment.

The latest solver conditions on fixed source landmarks and estimates each pair
independently. Another scalar head does not change those assumptions. Earlier
two-endpoint Gaussian registration already models source uncertainty, and DROID
already provides per-camera bundle-adjustment baselines. Neither is evidence of
a tested shared-rig, multi-frame joint-landmark implementation in the inspected
inventory. A targeted search found no such completed control; this is not an
exhaustive proof of code absence or a novelty claim.

Dai et al., Multi-Spectral Visual Odometry without Explicit Stereo Matching,
arXiv1908.08814v1, sectionsIII–IV, describes temporal within-modality image
alignment, inverse-depth landmarks and sliding-window optimization with a shared
visible/LWIR rig. Its mechanism motivates temporal shared geometry; the proposal
here would use available feature/stereo residuals rather than claim to reproduce
that direct photometric system. Abstract and methodsIII–IV inspected; experiments
not independently reproduced. Author-system implementation availability is not
verified; no official code was executed. Source: https://arxiv.org/pdf/1908.08814

## Bounded pilot design

- Keep the existing exposed first100 groups of day/night/rain. Use20 fixed,
  nonoverlapping five-group windows per recording. Score80 within-window adjacent
  intervals per recording; retain the19 between-window intervals explicitly as
  outside this pilot, without reporting whole-prefix coverage or trajectory ATE.
- Compare matched RGB-only window optimization, shared RGB+thermal window
  optimization, and their frozen-depth versions. Retain native RGB and original
  RGB-cov on exactly the same slots. The RGB-only window solver isolates benefit
  of the extra sensor from benefit of a new backend or longer temporal support.
- Fix each window's first body pose to remove its pose gauge. Keep metric stereo
  residuals and camera calibration; depths must not freely absorb residuals.
  Use per-camera tracks with explicit identities, not concatenated independent
  pair packets masquerading as a multi-view track. First qualify whether saved
  native landmark IDs and observations provide these tracks. If not, declare
  precisely the missing observations; do not silently invent tracks or run a
  matcher/data search to rescue the pilot.
- Freeze residuals, tracks/masks, covariances, initialization, numerical budget,
  damping, derivative convention, validity checks and source/input hashes before
  solving. Check pose/depth gauge, Jacobians and identical zero-change controls.
- Require at least5% lower translation mean and RMSE versus matched RGB-only
  window optimization AND original RGB-cov for each recording, no rotation
  mean/RMSE or within-window coverage regression. No pooled-only promotion or
  parameter/window search after failure. No learning follows automatically.

The immediate deliverable is one fixed-control pilot, not a new full SLAM stack.
Carry the supplied same-ID nominal-time assumption and all actual timestamps
explicitly; this is not verified physical simultaneity. All data remain exposed
and reference/checkpoint qualifications remain. No protected reserve, new capture,
IMU fusion, cross-modal pixel matcher or thermal photometric model is proposed.
A passing pilot would justify separate replication, not a paper-level claim.
