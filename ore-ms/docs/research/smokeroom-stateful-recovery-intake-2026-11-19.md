# SmokeRoom stateful recovery intake

SmokeRoom is an exposed field-obscuration candidate for a future
stateful test against a supplied fused reference. A replay could test
whether a thermal handoff preserves output and reduces error through
an obscured interval, conditional on a matched clear-prefix control.
Independent metric validation requires different reference evidence.

The [existing fallback study](smoke-rgb-first-findings-2026-11-16.md)
locates all 15 moving RGB proposal gaps at nominal 290.5–306.5 s.
These are half-second pairwise decisions. They establish an exposed
candidate interval, and leave sustained estimator failure and return
unmeasured. Freeze the existing span and ten seconds of context on each
side before acquiring additional images. All times below are offsets
from the first exported RGB header, 1676767774054630249 ns.

Retrieve the public pose/IMU ZIP and verify member CRCs, finite rows,
and strict timestamps. Retrieve only the two timestamp lists from the
9.25 GB image archive using verified byte ranges and ZIP member CRCs.
The timestamp-list intake transfers 2,775,720 bytes including the ZIP
directory. The pose/IMU ZIP contains 83,667 pose rows and 83,768 IMU rows.
[Author reference folder](https://drive.google.com/drive/folders/1bSGDe1FFQYEjs20P8L15pTx7T0HCn2L0),
[previous raw-header binding](smoke-reference-pose-findings-2026-11-16.md).

| Time span, s | RGB / thermal / IMU / poses | Maximum within-span gap, RGB / thermal / pose | Path length from supplied poses |
| --- | ---: | ---: | ---: |
| 280.5–290.5 | 245 / 299 / 1,999 / 1,999 | 41.06 / 50.00 / 10.00 ms | 5.61 m |
| 290.5–306.5 | 393 / 479 / 3,200 / 3,200 | 40.92 / 50.00 / 5.01 ms | 7.45 m |
| 306.5–316.5 | 246 / 297 / 1,999 / 1,999 | 41.13 / 50.00 / 10.00 ms | 4.71 m |

Eight RGB samples were declared at 280.5, 285.5, 290.5, 295.5,
300.5, 306.5, 311.5, and 316.0 s before reading the images. The approach
is already hazy at 280.5 s. Walls and scene structure become harder to
see across the middle samples and more visible by 316.0 s. This confirms
an appearance degradation-to-improvement sequence. A clear initialization
control needs an earlier interval, and these sparse samples do not locate
an exact recovery boundary. The eight images, contact sheet, verified
range receipts, and coverage script are in
`/root/ore-ms-artifacts/smokeroom-recovery-intake/`.

The dataset paper describes trajectories built from map, visual, LiDAR
and inertial constraints. Its supplement illustrates RGB feature
degradation and thermal support in smoke. The inspected public
SmokeRoom bag and reference listings supply RGB, thermal, IMU,
calibration and integrated odometry. They expose no separate
SmokeRoom metric trajectory that can establish independence from the
tested observations. Shared inputs could influence the replay scores.
[Dataset paper](https://openaccess.thecvf.com/content/CVPR2024/papers/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.pdf),
[supplement, Figure 7](https://openaccess.thecvf.com/content/CVPR2024/supplemental/Zhao_SubT-MRS_Dataset_Pushing_CVPR_2024_supplemental.pdf),
[official release inventory](https://superodometry.com/datasets).

Before a stateful replay, verify estimator support for the released RGB
MEI and thermal pinhole models and establish nominal camera–IMU timing.
Require a clear RGB-only prefix control before interpreting the
obscuration span. Freeze any output-health trigger and recovery test
before scoring, and retain fixed RGB and thermal arms. Use the result
as supplied-reference SmokeRoom characterization. The independent
natural-transition claim remains open. This intake ran no estimator.

## Clear moving RGB control

Scout RGB every 20 s from 10 to 270 s. Early sampled dark frames have
almost no net displacement in their surrounding ten-second windows.
The later 230 s and 250 s frames show clear indoor structure. The 270 s
frame already shows haze. Select the ten-second window around 250 s, the
latest clear moving anchor on that grid, and verify it on a fixed
one-second grid from 245 through 255 s. All eleven samples show visible
walls, door frames, or floor structure without apparent smoke obscuration.
The scout is an appearance check on sampled frames.

In the half-open 245–255 s control, the exported timestamps contain 246
RGB, 297 thermal, 1,998 IMU and 1,998 supplied pose rows. The maximum
internal RGB and pose gaps are 41.27 and 10.00 ms. The pose export shows
5.04 m of path length and 4.91 m net displacement. This qualifies a
visually clear, moving candidate for the RGB-only control. The estimator
must still initialize and pass its frozen metric check before interpreting
the obscuration test. A stateful continuation must consume the intervening
inputs through the candidate failure interval and recovery context.

The timestamp and reference hashes, bounded image receipts, and both
contact sheets are saved with the intake. No estimator was run.
