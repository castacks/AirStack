# Fixed protocol: temporal flow closure as camera-quality evidence

**Execution update, 2026-10-30 America/New_York:** [EXP087](../../experiments/EXP-087-flow-cycle-quality.md) completes the unchanged protocol. All forward controls pass; closure matches thermal-only at night but worsens both daytime losses versus RGB-only. No scalar adopted. The status and plan below are the original pre-inference declaration.

2026-10-30 UTC. Branch `ore_ms`, base `a177ba60c`.
Status: declared, unexecuted. No new signal results or quality gate are claimed.

## Question and scope

Does forward/backward correspondence inconsistency provide camera-quality
information beyond raw learned variance and tracking availability on the three
already examined windows? The [opportunity audit](solver-decision-and-selection-headroom-2026-10-30.md)
shows conditional day translation headroom, limited early translation headroom and
no middle translation headroom beyond thermal-only. Its labels motivated this
exploratory choice; these windows cannot become unseen evaluation afterward.

Use all existing EXP08420-frame early, EXP085100-frame middle and EXP086100-frame
day inputs, start0, original checkpoints, exact prepared images and pinned GPU
runtime. Original v3 motion/uncertainty outputs remain frozen. Do not use the
repair arm, change original tracking/poses or train/fine-tune a network or gate.
No new scene or reserved MS² capture. Bound additional reverse matching to all217
pairs for both cameras (434reverse predictions), with forward preservation controls.

## Measurement contract

The saved NPZ audit grid contains `grid_uv0`, `grid_uv1`, `grid_inbound`, `K`,
`height_width` and `grid_raw_flow_cov`. Use this frozen grid rather than selecting
points from errors or new covariance predictions. The original forward endpoint
is v=u+f01(u). Run the same native matching network on image1,image0 and bilinearly
sample reverse flow f10(v), using the established native coordinate/sampling
convention. Record the raw pixel closure c=v+f10(v)−u for every original grid point.

The primary camera diagnostic is median sqrt((cx/fx)^2+(cy/fy)^2) across original
forward-inbound points with finite, in-bounds reverse sampling. This is a normalized
image-correspondence diagnostic, not a calibrated angular/pose uncertainty or a
probability. Keep every invalid/out-of-bounds point and denominator count. A
camera with zero valid closure samples has an unavailable diagnostic; do not set
its score to zero. Record native pixel norm, quartiles and valid fraction as
context; do not select the best statistic after scoring. No clipping, thresholds,
new outlier filter or fitted camera multiplier.

Before signal analysis, fresh forward predictions at the frozen grid must match
stored forward endpoints in the same runtime. A failed control stops analysis
for investigation; do not silently substitute approximate/new forward outputs.
Reverse inputs are images in opposite order, not reversed or fabricated VO poses.
Source association always records the original integer sensor endpoints.

## Fixed offline comparisons

Eligibility requires a genuine original first pre-refinement supported motion.
Compare RGB-only, thermal-only, the earlier fixed lower angular-variance trace
preference, and lower median normalized flow closure. Variance baseline is median
(Sigma_uu/fx²+Sigma_vv/fy²) over its finite positive-diagonal original grid values.
Each diagnostic is unavailable if it has no valid samples. No learned combination.

With both eligible cameras and defined diagnostics, lower scalar wins, exact ties
prefer RGB. With one eligible camera whose diagnostic is defined, use it; otherwise
abstain. Report complete coverage for every policy and paired error on exact
matched intersections; do not compare differently supported RMSEs directly. Report
primary both-motion-supported and union populations separately. Frozen camera
baselines keep their own availability, rather than acquiring feature-based masks.

Retain all217interval slots, unsupported/undefined scores, exact choices and both
translation/rotation losses. Repeat the predeclared processing-only sensitivity
that excludes any terminal motion, without treating it as an online deadline test.
Use unchanged nominal-body/reference-assisted SLAM proxies for offline scoring;
reference, winner labels and errors never enter signal construction or selection.
No alignment/scale/threshold fitting or stitched/fused trajectory.

## Causality and limits

Both images belong to the current/past interval, so closure needs no future frame.
The extra neural pass has an unmeasured latency cost. Offline signal availability
does not establish availability before a real arbiter deadline; any online claim
requires a separately instrumented shared chronological replay. Frame conversion
and reference bridges remain evaluation-only. A cycle-consistent wrong match is
possible, so closure is a hypothesis, not a guarantee of correct correspondence.

Freeze exact model/source/input/output hashes and all commands before reverse
inference. Verify scalar versus bilinear sampling, endpoint/finite masks, original
forward preservation, independent closure arithmetic, no-label access, all-slot
populations, tie/missing behavior and decision/loss mutations. Retain all windows
and both outcomes, including negative results. Do not promote a scalar or tune its
thresholds from these known-window results. A learned gate or prospective recording
is a later, separately declared experiment.
