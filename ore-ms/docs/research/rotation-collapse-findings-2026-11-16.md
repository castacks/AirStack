# Moving failures often contain small nonzero rotation estimates

16 November 2026, America/New_York. Completed the
[fixed rotation-collapse protocol](rotation-collapse-protocol-2026-11-16.md)
after [failure-stratum characterization](paired-failure-strata-findings-2026-11-16.md).
All5,516original rows, both frontends/cameras and both records remain.
Zero new inference, parameter fitting, selector changes or threshold searches.
These are known nominal supplied-reference development outcomes.

## Fixed near-identity result

The prospectively defined assay is estimated angle≤0.1°. It explains a minority
of learned supported errors. The cut remains unchanged despite many estimates
falling just above it. Missing proposals remain separate from incorrect poses.
Each error fraction below is near-identity wrong outputs / all wrong supported
outputs; output/anchor denominators separately retain every moving input failure.

| Moving learned population | Camera | Outputs / all anchors | At R≤1°: wrong near-identity / all wrong | At R≤5°: wrong near-identity / all wrong |
|---|---|---:|---:|---:|
| FlashLight | rgb | 192/192 | 3/16 | 3/7 |
| FlashLight | thermal | 168/192 | 6/111 | 4/58 |
| SmokeRoom | rgb | 221/236 | 14/70 | 7/40 |
| SmokeRoom | thermal | 231/236 | 9/115 | 3/41 |

SmokeRoom RGB near-identity covers20.0% of strict errors and17.5% of coarse
errors; thermal covers7.8%/7.3%. The fixed test does not explain most errors.
FlashLight RGB has only7coarse errors,3near-identity; thermal has58coarse errors,
4near-identity. A larger percentage of a small RGB failure set does not make
RGB worse than thermal or justify universal thermal priority.

Near-identity also occurs in coarse-correct moving estimates:7SmokeRoom RGB,
6SmokeRoom thermal and2FlashLight thermal outputs fall within5°. The existing
moving cohort begins at2° of reference motion; a near-zero prediction can still
pass a5° criterion. On the strict1° moving endpoint, all near-identity outputs
fail, as expected from the SO(3) triangle bound. This assay is descriptive and
is not used to reject, replace or route an estimate.

## Continuous amounts expose a broader motion-underestimation pattern

These are the same four learned coarse correctness cells frozen in the prior
study. Only returned poses contribute angle/error statistics. Reference-angle
quartiles use that same available/compared subset, so missing outputs do not
change the comparison population silently. Ratios use own-camera reference
amount≥2°. Cells are conditioned on supplied-reference outcomes and cannot be
used as live input labels.

| Recording | R≤5° correctness cell | Examined camera | Compared / cell anchors | Median estimate | Median reference | Median estimate/reference ratio |
|---|---|---|---:|---:|---:|---:|
| FlashLight | thermal_only_correct | rgb | 3/3 | 0.151° | 5.382° | 0.0231 |
| FlashLight | rgb_only_correct | thermal | 54/78 | 0.875° | 16.932° | 0.0609 |
| FlashLight | neither_correct | rgb | 4/4 | 0.117° | 14.462° | 0.0092 |
| FlashLight | neither_correct | thermal | 4/4 | 0.544° | 14.204° | 0.0502 |
| SmokeRoom | thermal_only_correct | rgb | 25/36 | 0.187° | 8.124° | 0.0247 |
| SmokeRoom | rgb_only_correct | thermal | 22/27 | 0.696° | 8.611° | 0.0707 |
| SmokeRoom | neither_correct | rgb | 15/19 | 0.158° | 7.190° | 0.0230 |
| SmokeRoom | neither_correct | thermal | 19/19 | 0.605° | 7.994° | 0.0709 |

In SmokeRoom's thermal-only-correct cell, available wrong RGB outputs have an
estimated-angle IQR0.108–0.243° against own reference IQR6.198–8.412°; median
amount ratio0.0247. In its RGB-only-correct cell, available wrong thermal outputs
have estimated IQR0.347–1.058° versus reference IQR7.319–10.123°, ratio0.0707.
These estimates are small nonzero motions, often above the fixed0.1° assay cut.
No broader cutoff is fitted to capture them.

SmokeRoom neither-correct RGB/thermal median error-minus-identity-error is
−0.018°/−0.392° on their respective available subsets. RGB estimates therefore
often provide little angular improvement over the zero-rotation control in
this particular labelled failure group. This is full geodesic error versus the
same camera's reference amount; it is not a separate calibrated truth source.
Wrong axes, correspondences, geometry, reference issues or other causes are not
distinguished by these scalar summaries.

In contrast, SmokeRoom both-correct RGB/thermal median ratios are0.9961/1.0060;
FlashLight both-correct ratios are0.9950/1.0041. Correct amount is not sufficient
for correct orientation: the equal-angle wrong-axis synthetic control remains
wrong by more than5°. Preferring the larger predicted angle is not tested or
established as a useful selector, particularly on static/small-motion inputs.

## Frontend and identity controls

Median amount ratio over each camera's returned moving poses:

| Recording | Frontend | RGB | Thermal |
|---|---|---:|---:|
| FlashLight | learned | 0.9987 | 0.9019 |
| FlashLight | sift | 0.0850 | 1.0803 |
| SmokeRoom | learned | 0.9875 | 0.9739 |
| SmokeRoom | sift | 0.0647 | 1.0418 |

SIFT RGB shows strong overall motion compression in these matched moving
cohorts; learned RGB's aggregate median is close to1while particular failure
strata still show severe underestimation. A whole-record median does not identify
those cases. SIFT thermal has no≤0.1°outputs on either moving cohort, yet still
has24FlashLight and34SmokeRoom coarse supported errors; near-identity is not a
necessary condition for failure.

Identity-only coarse successes on moving anchors are39/192RGB and38/192thermal
inFlashLight,90/236RGB and92/236thermal inSmokeRoom. The learned fixed cameras
respectively achieve185/110 and181/190coarse successes, retaining missing outputs
in each denominator. No moving identity control achieves a strict1°success.

Static mixture changes the whole common-reference comparison: SmokeRoom identity
achieves561RGB/560thermal strict successes across837anchors, versus412/348for
learned cameras. Identity supplies an output everywhere, while those matchers
return500/494outputs. This is why whole-record availability, static controls and
moving correctness remain separately reported; it is not a VO or motion-recovery
claim. Identity is not substituted into the existing policies.

## Verification, evidence and stop

Independent audit passes47,487checks on all5,516rows. It uses trace/skew atan2
angles rather than the production SciPy rotation magnitude, verifies source/status
and null semantics, frozen0.1°labels, own-reference joins, all triangle bounds,
identity comparisons, full/moving/common and learned coarse-stratum aggregates,
and independently interpolated quartiles. All8tests pass, including rehashed
angle, signed identity-gap and summary corruptions; identity/static, rejected
intermediate R, fixed boundary and equal-amount wrong-axis controls are retained.

The original scalar-angle helper is reused without changing its frozen NGCLAB
runner. That older unqualified-mocap scalar study is distinct from the current
nominal supplied-reference full orientation errors. Existing source/checkpoint/
runtime/reference provenance remains inherited; no physical clock/exposure or
independent pose truth certificate follows from this arithmetic diagnostic.
Prior paper/source reading was reused; no fresh paper-body review, author-system
reproduction or new-method claim.

Full rows, locks, summaries, logs and audit: `results/rotation-collapse/run-001/`.
The PNG/PDF in `results/rotation-collapse/figures/` decomposes moving outcomes into
correct, wrong near-identity, wrong other and missing proposals, alongside identity
controls. A per-member verified local incremental packet is under
`/root/ore-ms-artifacts/rotation-collapse-2026-11-16/`, with parent packet pins.
No Box upload, Git push or new automatic commit occurred.

Stop decision: the fixed≤0.1°hypothesis covers only a minority of learned errors;
continuous data support substantial motion underestimation in the labelled coarse
failure groups. Do not relax the assay or infer its cause. The next bounded
question is whether compression is already present in accepted correspondences
or introduced by essential-pose recovery. Examine the same fixed caches and
controls before proposing a correction or quality mechanism; no fitted camera
selection follows automatically from the observed ratios.
