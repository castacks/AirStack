# Track survival near previously measured position-error episodes

The historical estimator traces were unavailable locally. This
retrospective image-only scout uses the same point detector and optical
flow settings as the [SmokeRoom persistence study](smokeroom-track-persistence-2026-11-20.md).
It compares adjacent and three-step track counts at common ending frames.
The counts are from an independent proxy tracker.

VINS-Mono uses tracked-feature count among its failure criteria and its
released frontend maintains per-feature age. Liu et al. use feature age
to rank stereo VO samples in robust motion estimation. These prior uses
motivate comparing fresh and sustained support. This scout asks whether
sustained support changes before the previously measured position error.
[VINS-Mono paper](https://arxiv.org/abs/1708.03852),
[pinned feature tracker](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/feature_tracker/src/feature_tracker.cpp),
[stereo VO paper](https://doi.org/10.3390/s17102339).

Before fetching frames, select two-second windows at 250, 261.8 and
263.8 s on SmokeRoom and 85, 88 and 90 s on FlashLight, relative to
the first RGB export timestamp. The SmokeRoom boundary is the known
first sustained error exceedance. FlashLight uses the known scheduled
reset time. Sample 10 unique frames per window at 5 Hz within 25 ms of
each target. Retrieve exact ranges and CRC-verify all 60 selected images.
Compare seven matched endpoints per window. The persistence count carries
one set of corners through three steps, roughly 0.6 s, without reseeding.

| Recording and window start | Median adjacent tracks | Median persistent tracks | Minimum adjacent / persistent |
| --- | ---: | ---: | ---: |
| SmokeRoom clear 250 s | 237 | 199 | 213 / 182 |
| SmokeRoom pre-onset 261.8 s | 241 | 202 | 201 / 168 |
| SmokeRoom post-onset 263.8 s | 220 | 182 | 184 / 110 |
| FlashLight clear 85 s | 224 | 182 | 214 / 162 |
| FlashLight pre-reset 88 s | 267 | 225 | 247 / 178 |
| FlashLight post-reset 90 s | 222 | 119 | 121 / 46 |

The SmokeRoom pre-onset median rises slightly for both measures and
the persistent minimum falls modestly. This scout provides no clear
persistence warning advantage in the immediate pre-onset window.
FlashLight persistence fell in the post-reset interval, but the scout
used the same unmodified images for both original estimator arms.
It cannot attribute that drop to the reset or explain the arm-specific
metric error. The exposed, short image windows support a descriptive
check only. They leave native estimator feature identities, warning
thresholds and independent transfer untested.
[SmokeRoom error onset](smokeroom-vins-fixed-replay-2026-11-19.md),
[FlashLight reset](flashlight-vins-rgb-reset-transfer-2026-11-19.md).

Source, checked frames and 42 endpoint rows are in
`/root/ore-ms-artifacts/subt-recovery-access/` under `warning-*`
and `warning_scout.py`. No estimator replay or threshold fit was run.

## Spatial concentration follow-up

Using the same 60 saved frames and 42 endpoints, check whether similar
counts conceal more concentrated image support. Feature bucketing for
distributing support across the image is established in the
[stereo VO study](https://doi.org/10.3390/s17102339). Here, divide
normalized image coordinates into a 4 × 4 grid and use exponentiated
Shannon entropy of cell occupancy as the effective number of occupied
cells. Also report the square root of the smaller eigenvalue of the
normalized coordinate covariance, which summarizes spread along the
narrower direction. Neither metric tests triangulation or observability.

Compute both metrics for adjacent and persistent endpoint tracks.
In a descriptive equal-count check, sample 40 points without replacement
200 times per endpoint using seed 11. All endpoints have at least 40
points. Take the median across replicates, then across the seven
endpoints in each window. Windows and tracker settings are unchanged.

| Window | Adjacent effective cells, matched 40 | Persistent effective cells, matched 40 | Persistent minor-axis spread, matched 40 |
| --- | ---: | ---: | ---: |
| SmokeRoom clear | 7.65 | 6.45 | 0.182 |
| SmokeRoom pre-onset | 7.36 | 6.46 | 0.171 |
| SmokeRoom post-onset | 5.34 | 3.93 | 0.126 |
| FlashLight clear | 10.02 | 9.10 | 0.196 |
| FlashLight pre-reset | 10.62 | 9.62 | 0.195 |
| FlashLight post-reset | 9.01 | 8.21 | 0.211 |

SmokeRoom persistent coverage is nearly identical between the clear
and pre-onset windows at matched count. Its narrower-axis spread falls
slightly before onset and more afterward. The strongest spatial change
is therefore descriptive of the post-onset window and supplies no
clear advance warning on this sample. FlashLight measures again apply
to the shared input images and cannot explain the estimator-arm
difference. The unresampled analysis shows the same qualitative pattern.
The replay reproduced all 84 original track counts. Scripts and
per-endpoint spatial outputs are saved with the prior artifacts under
`warning_spatial*.py` and `warning-spatial*.csv`.
