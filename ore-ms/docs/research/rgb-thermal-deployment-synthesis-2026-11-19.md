# RGB–thermal odometry deployment characterization

19 November 2026. The evaluated thermal handoff substantially reduces
metric position error during sustained controlled RGB degradation. Its
early recovery accuracy varies across recordings. In real recordings,
thermal sometimes supplies useful motion estimates when RGB gives none,
while RGB can retain strong support through a visible light transition.
The results motivate evaluating thermal as a conditional fallback with
separate accuracy, availability, continuity, and cost endpoints.

## Controlled degradation and recovery

On the two IRS RTVI recordings, the tested single-filter handoff changes
the active camera while carrying forward the inertial state. The RGB
images are dimmed for ten seconds. The trigger uses incoming RGB pixels
and holds thermal after switching. Score common Vicon–IMU timestamps with
a unit-scale yaw and translation fit on the ten seconds before dimming.
The recordings are from the same rig and the trigger threshold was chosen
for the artificial intervention.
[Brightness-triggered handoff](irs-rtvi-causal-dimming-2026-11-18.md).

| Metric or control | Medium | Difficult |
| --- | ---: | ---: |
| Dimmed RGB-only position RMSE during dimming | 0.657 m | 1.015 m |
| Image-triggered thermal handoff RMSE during dimming | 0.105 m | 0.113 m |
| Image-triggered handoff RMSE in first 2 s after light returns | 0.183 m | 0.254 m |
| Withhold three flagged RGB updates at matched switch time | 0.181 m | 0.177 m |
| Also fix thermal extrinsics at handoff | 0.078 m | 0.175 m |

Withholding flagged updates improved Difficult early recovery while
expanding the corrected-output gap to about 196 ms in sensor time.
Three IMU-only predictions restored roughly 50 ms sensor-time cadence.
In paced replay, discarding stale corrected arrivals offline left
receipt gaps of 146.6 and 143.7 ms, only 2–3 ms shorter than corrected-only
receipt. This control preserved the corrected trajectory and its recovery
error. Sensor timestamp cadence and arrival cadence answer different
deployment questions.
[Withholding control](irs-rtvi-flagged-visible-update-ablation-2026-11-18.md),
[paced delivery](irs-rtvi-paced-imu-bridge-delivery-2026-11-18.md).

The fixed-extrinsics control improved Medium before recovery started.
On Difficult it reduced the change in position error after light
returned, while the error at the first recovery sample was already
0.153 m. The scheduled hold on Difficult reached 0.122 m early RMSE.
Across both recordings, no tested causal schedule demonstrated both
the 0.15 m early RMSE and 0.15 s output-gap criteria. A lower dimming
error can coexist with a recovery offset and with stale arrivals.
[Fixed-extrinsics test](irs-rtvi-fixed-thermal-extrinsics-2026-11-18.md),
[recovery error decomposition](irs-rtvi-early-recovery-error-origin-2026-11-19.md),
[comparison requirements](pose-accuracy-baseline-comparison-2026-11-18.md#open-claims-and-comparison-requirements).

## Real recordings and computation

| Evidence tier | Observed result | Scope |
| --- | --- | --- |
| SubT-MRS SmokeRoom, supplied-reference pairwise scoring | Learned thermal fallback filled 15 RGB gaps on 236 moving anchors. Success within 5° rose from 76.7% to 81.4%. Joint success rose from 72.3% to 73.7% on 483 translation anchors. | Projected matching cost rose 6.6% on moving anchors and 42.7% over the whole recording. These are pair proposals and cached cost projections. |
| SubT-MRS FlashLight, supplied-reference pairwise scoring | Learned RGB produced estimates on all 192 moving anchors, with 96.4% within 5°. RGB-first fallback added no moving outputs. With SIFT, thermal reached 57.3% within 5° versus 17.2% for RGB. | The result changes with the frontend. The supplied-reference scores address relative motion. |
| DiTer++ PARK, image-space field comparison | Across a shaded-to-daylight exit, RGB exceeded stretched thermal in median held-out epipolar support even after matching counts, 98.1% versus 92.5%. Median supported half-second chains were 64 versus 5. | RGB feature support held through the exit. Thermal preprocessing mattered, and the thermal calibration and tracked pose frame remain unresolved. |
| Longer day backend control, supplied-reference path scoring | Adding thermal translation refinement with RGB fallback improved position RMSE by 0.140% over the strongest RGB refinement, while orientation error rose about 1.94% and solver CPU rose 10.79%. | The solver cost excludes the frontend. Extra RGB optimization accounts for much of the earlier measured night gain. |

These are distinct populations and endpoints. Keep the original learned
RGB-first pipeline as the paired comparison anchor. Use fixed RGB and
thermal controls, retain coverage and bad runs, and report cost where it
was measured or projected.
[Field score summary](../../MS_RESEARCH_HANDOFF.md),
[paired baseline decision](baseline-recommendation-2026-11-17.md),
[PARK geometry](diter-plusplus-park-geometry-2026-11-19.md),
[PARK feature persistence](diter-plusplus-park-track-persistence-2026-11-19.md),
[long-day control](long-day-fallback-findings-2026-11-15.md),
[night optimization control](optimization-feedback-findings-2026-11-15.md).

## Stateful field comparison in SmokeRoom

The fixed RGB VINS-Fusion control gave 0.043 m held-out position RMSE
with full scoring coverage in the earlier clear segment. In the
continuous replay, RGB still emitted a corrected pose for every frame
through the obscuration and later context, while supplied-reference
RMSE rose to 1.61 m and 6.88 m. The first sustained 0.30 m exceedance
began at 263.8 s, 26.7 s before the previously identified pairwise RGB
gap cluster. The under-ten-track criterion first fired 24.75 s after
the exceedance, and no corrected-output gap exceeded 150 ms. Early
error grew mainly along the direction of travel. The trajectory and
image evidence leave the cause unresolved.
[Clear RGB control](smokeroom-vins-rgb-control-2026-11-19.md),
[fixed replay and onset diagnostics](smokeroom-vins-fixed-replay-2026-11-19.md).

The fixed thermal arm initialized after the frozen alignment interval,
so it yielded no qualified metric RMSE or matched-camera RMSE. Across
the obscuration window its estimated endpoint distance was 116.7 m
versus 2.67 m in the supplied trajectory. That difference is invariant
to yaw and translation alignment. The run closes this fixed thermal
configuration as a switching baseline. The supplied reference includes visual, LiDAR,
inertial, and map constraints, which limits its independence.
[Fixed-camera results](smokeroom-vins-fixed-replay-2026-11-19.md),
[reference qualification](smoke-reference-pose-findings-2026-11-16.md).

A scheduled follow-up kept the initialized RGB–IMU state and prior,
then compared continuous RGB, RGB track reset, and a thermal switch
with the same reset. At native thermal resolution and cadence, the
switch retained output continuity but increased obscuration RMSE to
4.881 m versus 1.614 m for continuous RGB and 1.033 m for RGB reset.
Later errors were 3.267, 6.884, and 0.512 m, respectively. The
scheduled RGB reset performed best in both windows on this exposed
recording. This demonstrates neither a causal reset policy nor a
benefit to preserve by reducing the thermal pixel or update budget.
[Scheduled SmokeRoom comparison](smokeroom-vins-scheduled-handoff-2026-11-19.md).

The same fixed-time RGB reset intervention on FlashLight passed the
pre-reset control with 0.139 m RMSE and retained full pose coverage.
On [90,110) s, the reset increased RMSE from 0.237 m to 2.707 m.
The two recordings thus establish a meaningful downside to discarding
active tracks. The direction of the effect depends on the replay
context and scheduled time.
[FlashLight stateful transfer](flashlight-vins-rgb-reset-transfer-2026-11-19.md).

## Relationship to prior work

MAC-VO develops geometry-aware covariance for two-frame stereo pose
optimization. Its scope helps distinguish our pairwise correspondence
and motion diagnostics from a stateful handoff result.
[MAC-VO, v2](https://arxiv.org/html/2409.09479v2).

Doer and Trommer's RRxIO paper evaluates radar fused with visible- or
thermal-inertial odometry. Our IRS replays use the pinned author-derived
camera–IMU filter to characterize a visible-to-thermal handoff. The
radar-assisted paper results are prior evidence for resilience through
poor observations. Radar is outside these local replay arms.
[RRxIO author publication](https://christopherdoer.github.io/publications/2021_09_IROS2021).

Hierarchical adaptive odometry already selects features, state directions,
and estimator inputs under degradation, with an inertial fallback.
Our contribution here is comparative evidence about the size and limits
of thermal benefit in these specific conditions. The measured recovery
residuals do not identify an observability failure or a new estimator
mechanism.
[Resilient Odometry via Hierarchical Adaptation](https://arxiv.org/html/2608.25427v1),
[local prior-art and residual comparison](super-odometry-handoff-relevance-2026-11-18.md).

## Research decision

The PARK frontend study and SmokeRoom saved-trace diagnostics are complete.
The current evidence supports a bounded
deployment characterization and motivates targeted thermal handoff under
substantial RGB degradation. The tested SmokeRoom handoff is closed
at this switch time and estimator configuration. The reset helped on SmokeRoom and hurt on FlashLight. A general
reset policy remains unsupported.

Independent validation of stateful metric
recovery through naturally occurring RGB loss and return remains open.
A new recording becomes necessary if that field-recovery claim is central.
An eligible test needs a same-rig transition with calibrated camera–IMU
timing and transforms, a traceable metric pose reference, and a bright
control that passes before the intervention is scored.

This synthesis uses checked-in research notes and the current PARK
and SmokeRoom artifacts. The historical IRS replay packet is absent on this machine.
The synthesis involved no new inference or trajectory replay.
