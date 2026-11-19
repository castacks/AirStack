# Redcourse pose-reference intake

Read the ZIP directory and six NovAtel odometry members directly from
`ore_ms:ore-ms/night_macthermal_salon_redcourse_waypointskip1.zip`
using the existing bounded reader. The ZIP reader verified member CRCs.
The audit read 25,453,822 bytes and used no image or estimator outputs.

The source export contains 6,244 finite 13-field odometry rows with
`map → gps_frame` frame IDs. Six repeated timestamps account for 24
additional rows across the full source. Neither fixed target interval
contains repeated timestamps, and all endpoint interpolation brackets
are unique. Use the original source-rate positions and the published
[EXP-037 interval endpoints](../../experiments/EXP-037-redcourse-lit-interval.md).
Read endpoint positions by linear interpolation, with the same fixed
−0.1/0/+0.1 s sensitivity offsets used in [EXP-033](../../experiments/EXP-033-gnss-and-tracking-reporting.md).
No offset was fitted or accepted as a clock correction.

| Fixed interval | Source rows inside | Largest bracketing gap | Horizontal net displacement at zero offset | Fixed-offset range |
| --- | ---: | ---: | ---: | ---: |
| Full 21.90 s context | 219 | 124.53 ms | 49.638 m | 49.552–49.646 m |
| Original 1.90 s lit block | 19 | 124.53 ms | 1.852 m | 1.804–1.883 m |

The stream covers both intervals and indicates motion. These values measure
exported GNSS endpoint displacement. The offset range measures sensitivity
to the specified offsets. The pose rows contain position, quaternion and
twist. They lack explicit fix/solution status and pose covariance. The
scalar `errors.txt` has no identified GNSS quality semantics. The archive
lists a NovAtel IMU-to-vehicle transform, while the exported trajectory uses
`gps_frame`. The association between those origins, physical timestamp
alignment and receiver accuracy remain unverified. Redcourse therefore
does not qualify for camera-pose RMSE or rotation error scoring.

For already-scored comparisons, use [IRS RTVI](irs-rtvi-paced-recovery-2026-11-17.md)
for metric position under controlled frame loss, the
[IRS dimming control](irs-rtvi-photometric-transition-2026-11-18.md) for
continuous arrivals with artificial dimming, and
[SubT-MRS FlashLight](flashlight-transfer-findings-2026-11-16.md) for
real recorded illumination and supplied-reference pairwise rotation and
translation-direction scores. Each supports a different claim.

Local source bytes, directory inventory, member SHA-256 hashes, and
the endpoint calculator are under
`/root/ore-ms-artifacts/redcourse-reference-intake/`.
