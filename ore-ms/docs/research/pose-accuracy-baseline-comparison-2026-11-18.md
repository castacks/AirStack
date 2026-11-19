# Pose accuracy baselines for camera fallback

18 November 2026. Retrospective comparison of published run summaries
in this repository. Each row uses the original study's own scoring
cohort and reference. Cross-record sums and statistical inference from
temporally correlated anchors are outside this analysis.

## Metric trajectory position

| IRS RTVI condition | Visual RMSE | Thermal RMSE | Lower observed RMSE |
| --- | ---: | ---: | --- |
| Easy, uninterrupted | 0.127 m | 0.140 m | Visual by 0.013 m |
| Dark, uninterrupted | 5.402 m | 0.213 m | Thermal by 5.189 m |
| Fast dark, uninterrupted | 2.127 m | 0.125 m | Thermal by 2.002 m |

The [uninterrupted controls](irs-rtvi-stateful-replay-2026-11-17.md)
use the authors' camera-specific presets and fit translation and yaw over
each run. Scale is fixed. The two fast-dark arms differ by one scored row.
These are within-record descriptive comparisons of complete configurations.

During the [paced fast-dark frame loss](irs-rtvi-paced-recovery-2026-11-17.md),
fallback gives 0.236 m versus 0.344 m for uninterrupted visual over the
same interval. That is 0.108 m lower and 0.086 m above the frozen 0.15 m
limit. The retrospective fixed-thermal control with the same pre-window
fit convention scores 0.047 m. Fallback is 0.189 m higher. Fixed thermal
uses its native preset and has thermal history before the intervention.
In the [continuous-frame dimming control](irs-rtvi-photometric-transition-2026-11-18.md),
switching scores 0.066 m versus 0.058 m for dimmed visual, a 0.008 m
increase. Existing results support fixed thermal as a required reference
arm for any fast-dark switching trial. Track counts and output cadence
alone cannot establish an accuracy gain.

## Relative rotation and translation direction

Re-express the locked [IRS joint threshold counts](irs-rtvi-nominal-camera-pose-2026-11-17.md).
The endpoint is rotation within 5° and translation direction within 15°,
including missing proposals as failures on the common 1 cm motion cohort.
For each row, choose the larger of the two fixed-camera counts and of
the two fallback counts for a retrospective comparison. These choices
use observed aggregate outcomes and cannot be used as a routing policy.

| Recording / interval | Best fixed | Best fallback | Added joint successes | Difference |
| --- | ---: | ---: | ---: | ---: |
| Easy / 0.5 s | 137/137 visual | 137/137 visual-first | 0 | 0.00 pp |
| Dark / 0.5 s | 132/229 thermal | 134/229 thermal-first | 2 | +0.87 pp |
| Fast dark / 0.5 s | 39/123 thermal | 40/123 thermal-first | 1 | +0.81 pp |
| Fast dark / 0.1 s | 105/121 thermal | 105/121 thermal-first | 0 | 0.00 pp |

The sole gains over the stronger fixed controls are two and one pairs.
The larger apparent gains over visual-only in darkness are mostly
available from fixed thermal. In [SubT-MRS FlashLight](flashlight-transfer-findings-2026-11-16.md),
learned RGB and RGB-first tie on 452/519 joint successes, so that recording
also shows zero gain over the stronger fixed learned arm.

## IRS RTVI continuation checkpoint

Saved 18 November 2026 on `ore-ms-orb-development` at repository base
`1c8b7ee55`. The baseline sections above summarize the historical
reports. The replay chain below uses newly captured pose rows.

| Completed control | Result to carry forward |
| --- | --- |
| [Medium bright](irs-rtvi-bright-medium-metric-2026-11-18.md) and [fast dark](irs-rtvi-fast-dark-common-support-2026-11-18.md), uninterrupted | With the first 10 s fit and later common timestamps scored, visual/thermal position RMSE is 0.089/0.248 m in Medium and 4.302/0.282 m in Fast Dark. |
| [Easy two-second dimming](irs-rtvi-photometric-transition-2026-11-18.md), [Medium ten-second dimming](irs-rtvi-medium-dimming-2026-11-18.md), and [Difficult transfer](irs-rtvi-difficult-dimming-transfer-2026-11-18.md) | Easy shows no gain. Medium and Difficult show a large dimming-window gain over dimmed visual. Scheduled thermal hold passes 0.15 m during and after dimming on Difficult. |
| [Medium hold](irs-rtvi-medium-thermal-hold-2026-11-18.md) and [two-second delayed return](irs-rtvi-medium-delayed-return-2026-11-18.md) | Immediate return and hold both miss 0.15 m in the first two seconds after restored light. Delayed return scores 0.152 m in its primary post-switch window; it does not justify a return-delay sweep. |
| [Image-triggered handoff](irs-rtvi-causal-dimming-2026-11-18.md) and [matched-switch withholding](irs-rtvi-flagged-visible-update-ablation-2026-11-18.md) | Image-triggered switching meets 0.15 m during dimming on both bags. Withholding the three flagged visible updates improves Difficult early recovery from 0.254 to 0.177 m, changes Medium from 0.183 to 0.181 m, and increases the sensor-time output gap to about 196 ms. |
| [IMU-only gap bridge](irs-rtvi-imu-only-output-bridge-2026-11-18.md) and [paced receipt audit](irs-rtvi-paced-imu-bridge-delivery-2026-11-18.md) | Three predictions restore roughly 50 ms sensor-time cadence and preserve every corrected pose. Pacing shows one stale corrected arrival per bag. Discarding stale arrivals offline leaves 146.6/143.7 ms receipt gaps on Medium/Difficult, only 2–3 ms better than corrected-only receipt. Early recovery stays at 0.181/0.177 m. |
| [Continuously running thermal](irs-rtvi-warm-thermal-shadow-2026-11-18.md) | Offline switching into a separately running thermal trajectory improves Medium early recovery to 0.120 m and worsens Difficult to 0.261 m. Displacement-vector error across the switch rises to 9.1/6.4 cm versus 0.6/0.8 cm for the single filter. No parallel execution was measured. |
| [Fixed thermal extrinsics at handoff](irs-rtvi-fixed-thermal-extrinsics-2026-11-18.md) | Early Medium RMSE falls from 0.181 to 0.078 m; Difficult stays above the bound at 0.175 m and later error increases. |
| [Early recovery error decomposition](irs-rtvi-early-recovery-error-origin-2026-11-19.md) | Fixed extrinsics improve Medium before recovery starts. On Difficult they reduce subsequent displacement error, while starting recovery at 0.153 m. |

### Open claims and comparison requirements

No tested causal schedule clears the early position and continuity
criteria on both recordings. Medium and Difficult are exposed recordings
from the same rig with artificial dimming. Retain a common Vicon–IMU
cohort, pre-intervention unit-scale yaw and translation, native visible
and native thermal controls, 0.15 m position RMSE, and a 0.15 s output
gap. Report timestamp ordering and unsupported reference rows. For
another experiment, state what changes in the estimator and which open
accuracy or delivery claim it tests. The fixed medium delay, veto, bridge
and warm-shadow controls are already measured.

The [Super Odometry review](super-odometry-handoff-relevance-2026-11-18.md)
records the papers and separates directional residuals from observability.
Early causal recovery squared error was over 99% horizontal on both
bags. The audit does not identify a particular bias or state mechanism.

Independent metric validation through a natural light transition remains
open. [Redcourse Box GNSS](redcourse-pose-reference-intake-2026-11-18.md)
covers the candidate interval and still lacks quality and frame/timing
qualification. [ViViD++](vivid-calibration-materials-2026-11-18.md) has a
recorded transition and failed the metric control. [OdomBeyondVision](odombeyondvision-transition-intake-2026-11-18.md)
has eight Vicon bags labeled bright.

### Recovery and verification

The new evidence is local under `/root/ore-ms-artifacts/irs-author-rescore/`.
Use the per-run protocols and saved loader diffs, binaries, poses, logs,
scorers, and hash manifests under `medium/`, `difficult/`, `causal/`,
`veto/`, `bridge/`, `paced-bridge/`, `warm-shadow/`, and `fixed-extrinsics/`. Keep failed
captures with their corresponding stage. The CRC-verified public bags and
author calibrations are in each recording's `release/` folder. The two
versioned paper PDFs and directional audit are in `superodometry/`.

The upstream loader pin is
`ae29afe0d7876f3838056b457292e4aecf9656be` with the build adaptations
recorded in the [fast-dark replay](irs-rtvi-fast-dark-common-support-2026-11-18.md).
The catkin workspace and upstream checkout are shared across the sequential
experiments. Restore from the stage-local artifacts when replaying an older
experiment. Evidence for the historical FlashLight results and older IRS
runs remains represented by their checked-in notes. The newly restored
runs and continuation edits have not been uploaded or published.
