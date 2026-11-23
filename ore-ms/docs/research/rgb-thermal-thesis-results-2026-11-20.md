# Thesis results for RGB–thermal odometry under visual degradation

**Thesis question.** Under what conditions does thermal input improve
odometry accuracy or availability, and what input or compute cost accompanies
the improvement?

Use within-recording comparisons under each original protocol. Report metric
trajectory error, pairwise motion accuracy, feature support, sensor-time
output gaps, and loader timing as distinct outcomes.

| Condition and comparison | Accuracy or feature support | Output availability or continuity | Input or compute cost | Reference and scope |
| --- | --- | --- | --- | --- |
| [IRS Medium bright](irs-rtvi-bright-medium-metric-2026-11-18.md) and [Fast Dark](irs-rtvi-fast-dark-common-support-2026-11-18.md), fixed visible vs. fixed thermal | Bright position RMSE 0.089 → 0.248 m. Fast Dark 4.302 → 0.282 m. | Fast Dark published 1,735 poses per arm. Medium and Fast Dark are different runs on the same rig. | Native camera cadence in both arms. | Vicon–IMU position at common timestamps with early unit-scale alignment. The comparison across recordings includes motion differences. |
| [IRS Medium and Difficult](irs-rtvi-causal-dimming-2026-11-18.md), dimmed visible vs. image-triggered thermal switch and hold | During dimming, 0.657 → 0.105 m and 1.015 → 0.113 m. First 2 s after light returns, 0.462 → 0.183 m and 1.013 → 0.254 m. Both triggered early-recovery results exceed 0.15 m. | All arms score 200 common positions during dimming and 40 during early recovery. Triggered sensor-time gaps stay below 51 ms. | One active camera. Compute was outside this comparison. | Common Vicon–IMU timestamps and pre-dimming alignment. Programmed ten-second dimming. |
| [SmokeRoom](smokeroom-vins-scheduled-handoff-2026-11-19.md), continuous RGB vs. RGB reset vs. thermal switch with reset | Obscuration RMSE 1.614 / 1.033 / 4.881 m. Later RMSE 6.884 / 0.512 / 3.267 m. | The thermal switch retains full scoring coverage with a maximum corrected gap near 50 ms. | Native resolution and cadence. No reduced-budget comparison. | Supplied fused trajectory, common 20 Hz scoring grid, scheduled interventions on one exposed recording. |
| [SmokeRoom pairwise fallback](smoke-rgb-first-findings-2026-11-16.md), learned RGB vs. RGB-first with thermal fallback | Success within 5° rose from 76.7% to 81.4% on 236 moving anchors. | Thermal filled all 15 moving RGB proposal gaps. Eleven additions met 5°. | Projected matching cost +6.6% on the moving subset and +42.7% overall. | Supplied-reference relative rotation. Pairwise proposals and cached cost projection. |
| [FlashLight pairwise transfer](flashlight-transfer-findings-2026-11-16.md) | Learned RGB reached 96.4% within 5° on moving anchors. With SIFT, thermal reached 57.3% and RGB 17.2% on those anchors. | Learned RGB produced outputs on all 192 moving anchors. Thermal fallback added zero there. | Whole-recording learned matching overhead projected at 3.3%. | Supplied-reference relative rotation. The observed ranking depends on the frontend. |
| [IRS Fast Dark cadence](irs-thermal-input-budget-intake-2026-11-19.md), native vs. warm-start 10 Hz thermal | 0.282 → 0.171 m on the common retained-time cohort. | 611/612 matches per arm. Maximum corrected gap 66.68 → 116.68 ms. | 1,231 → 616 post-prefix frames. Loader wall time 4.292 → 3.745 s. | Vicon–IMU reference; full-rate common alignment. One serial timing per arm including I/O and setup. |
| [IRS Medium cadence](irs-thermal-cadence-medium-transfer-2026-11-19.md), native vs. warm-start 10 Hz thermal | 0.248 → 0.168 m on the common retained-time cohort. | 620/621 matches per arm. Maximum corrected gap 50.24 → 100.23 ms. | 1,241 → 621 post-prefix frames. Loader wall time 4.061 → 3.216 s. | Same rig and rate protocol on a second recording. One serial timing per arm including I/O and setup. |
| [IRS Fast Dark startup](irs-thermal-cadence-startup-2026-11-19.md), [five-second prefix](irs-thermal-cadence-five-second-startup-2026-11-20.md), and [spatial pooling](irs-thermal-spatial-pooling-fast-dark-2026-11-20.md) | Immediate 10 Hz, five-second warm-up and 2 × 2 pooling each failed their frozen qualification. | Retain failure and missing-output accounting in the individual reports. | Lower startup cadence, shorter full-rate prefix or fewer pixels per image, respectively. | Same-rig controls bound the scope of the successful warm-start result. |
| [IRS thermal residual withholding](irs-thermal-innovation-withholding-2026-11-22.md), full rate vs. fixed thinning vs. residual withholding | Dimming RMSE on Medium 0.102 / 0.105 / 0.106 m and Difficult 0.186 / 0.184 / 0.114 m. Difficult early recovery 0.261 / 0.342 / 0.240 m. | 180 and 168 common dimming epochs out of 200 per recording. Full coverage in early recovery. The two sparse arms approximately double output gaps to 100 ms. | Each sparse arm omits 20 of 200 thermal images. Wall-clock cost was outside the comparison. | Fixed-thermal and IMU replay with a frozen open-loop omit list. Vicon–IMU position on common supported timestamps. |
| [IRS single thermal update](irs-thermal-innovation-withholding-2026-11-22.md#single-update-effects-over-the-next-two-seconds), keep vs. omit | High-residual omission lowered 2 s position RMSE in five of six cases and beat its nearby low-residual omission in three of six pairs. | Exactly one missing output per replay. 478 of 480 common reference epochs across the twelve comparisons. | One frame omitted per arm. | Both replays receive identical subsequent inputs. Each starts with full-rate estimator history. The standalone residual missed the frozen two-recording criterion. |
| [DiTer++ PARK](diter-plusplus-park-geometry-2026-11-19.md), shaded passage to daylight | Median held-out epipolar support at equal match count was 98.1% RGB vs. 92.5% stretched thermal. | Median supported half-second feature chains were 64 RGB vs. 5 stretched thermal. | Feature comparison; compute outside scope. | Image-space field evidence. Pose-frame and thermal calibration provenance remain open. |
| [CAVERS headlight movement](cavers-headlight-transition-intake-2026-11-22.md), handheld cave recording | Median adjacent-frame RGB track retention 82.3% before, 7.4% during and 69.5% after the selected event. Every tested sensor-only start exceeded the 0.15 m pre-event translation bound. | All expected camera epochs emitted poses in the matched replay. Publication measures output continuity. | Native asynchronous streams. Compute was outside this comparison. | OptiTrack body pose and a post-scout development window. The failed pre-event controls leave trajectory recovery unqualified. |

## Claim supported by these results

In the tested rig and estimator, thermal reduces metric position error
under sustained darkness and controlled visual degradation. Thermal
availability can also fill some RGB proposal gaps in recorded smoke.
The stateful SmokeRoom comparison and the early IRS recovery windows
show why accuracy must be measured alongside cadence. Full-rate
initialization followed by 10 Hz thermal is a measured input-saving
configuration on two IRS recordings, with larger corrected gaps.

## Validation scope

Use the IRS runs as the metric comparison and the SubT-MRS runs as
supplied-reference field evidence. Keep PARK as a field frontend
comparison and an optional extension. Its calibration and reference
frame remain unresolved for metric replay. The
[natural-transition audit](natural-transition-reference-intake-2026-11-22.md)
records the PARK, M2DGR Door, Brno and JPL Mars Yard limitations.
The CAVERS replay qualified the input and output timing and exposed
initialization sensitivity before the event. Independent metric recovery
through a natural field transition remains open. A new candidate needs
an RGB+IMU lead-in control before any recovery comparison.
[Baseline and recording gates](natural-recovery-baseline-gate-2026-11-23.md).

For chapter figures, use separate panels for the IRS controlled
degradation and its early recovery, the SmokeRoom field comparison,
and the thermal-rate tradeoff. Keep the common cohorts and reference
type on each panel. Do not pool RMSE across recordings or treat a
single loader timing as a realtime delivery measurement.
