# SmokeRoom fixed-camera replay through field obscuration

Continue the [qualified RGB-only control](smokeroom-vins-rgb-control-2026-11-19.md)
through the [exposed SmokeRoom interval](smokeroom-stateful-recovery-intake-2026-11-19.md).
Replay every native image from 225 to 316.5 s in two independent fixed-camera
VINS-Fusion runs with the same published IMU samples. Preserve the RGB MEI
model and the original noise, optimizer and nominal timing settings. For
thermal use the published pinhole intrinsics, normalized homogeneous
camera-to-IMU matrix, and prior per-frame mono16 min-max conversion.
The thermal transform keeps the published upper 3 × 4 block.
Both runs consume all 2,247 RGB or 2,731 thermal images and exit cleanly.
The 18,403 IMU samples include boundary padding. Verified input hashes
and a separate image decoder cover both native streams.

The fixed replay protocol fits yaw and translation at unit scale on the
245–250 s 20 Hz grid, with a 20 ms reference interpolation cap and
100 ms estimate cap. The five-second alignment requires at least 90
supported grid timestamps. Compare individual arms and the common
cohort wherever the corresponding early alignment is available.
Output gaps below use corrected output timestamps, with window endpoints
included. The recorded comparison uses a supplied fused reference.

| Window, s | RGB corrected / inputs | Thermal corrected / inputs | RGB / thermal max gap | RGB position RMSE |
| --- | ---: | ---: | ---: | ---: |
| 245–250, alignment | 123/123 | 0/148 | 41 ms / 5.00 s | 0.029 m |
| 250–255, clear check | 123/123 | 0/149 | 41 ms / 5.00 s | 0.043 m |
| 280.5–290.5, pre-cluster | 245/245 | 113/299 | 41 ms / 6.23 s | 1.067 m |
| 290.5–306.5, obscuration | 393/393 | 479/479 | 41 ms / 50 ms | 1.614 m |
| 306.5–316.5, later context | 246/246 | 297/297 | 41 ms / 50 ms | 6.884 m |

The extended RGB run reproduces all 727 earlier position rows exactly
and its held-out clear check passes. It exceeds 0.30 m in every later
window while maintaining output cadence. The pinned implementation
returns false unconditionally from its internal failure detector.
Consequently a missing-output trigger would see no RGB gap here.
The median internal track count drops from 114 in the clear check to
33 during obscuration. Higher apparent track counts and regular output
can coexist with poor metric motion. The late window marks the end of
the earlier pairwise proposal-gap cluster. It is not a verified time
at which RGB odometry recovers.

The thermal arm initializes at 286.729 s, 61.7 s after the declared
start. Its missed early alignment keeps all thermal and common-cohort
RMSEs unavailable. A diagnostic based on the same first and last
supported timestamps during obscuration gives a thermal endpoint
distance of 116.694 m versus 2.673 m for the supplied reference.
Neither translations nor rotations of the world frame could change
that distance. Keep this diagnostic separate from an RMSE or an
estimate of why initialization or scale failed.

This closes the fixed thermal configuration as a qualified switching
baseline. The sequence exposes a real field case where an RGB estimator
continues producing outputs as its position accuracy degrades.
The run tests no handoff and cannot establish the performance of an
already initialized shared-state thermal arm. Independent reference
validation remains open.

A separate audit reproduces the RGB scores, both output-status summaries,
and the rigid-invariant endpoint diagnostic. Native output, grids,
checksums, source changes and logs are under
`/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/fixed/`.

## Saved RGB warning timing

Apply the existing RGB alignment to the full 250–316.5 s interval on a
20 Hz grid. All 1,330 timestamps have support under the unchanged
interpolation caps. Before scoring, define a sustained error episode as
40 consecutive samples above 0.30 m. Compare the onset with the existing
under-ten-track count and the 150 ms output-gap criterion at native
corrected timestamps. No estimator or threshold search is involved.

| Observation | Time or count |
| --- | ---: |
| First position error above 0.30 m and first sustained onset | 263.80 s |
| Lowest track count in the two seconds before onset | 66 |
| Median track count in the two seconds after onset | 81 |
| First native frame with fewer than ten tracks | 288.55 s |
| Native frames with fewer than ten tracks | 7, all at 288.55–289.49 s |
| Largest consecutive corrected-output gap | 41.32 ms |

The low-track signal arrives 24.75 s after the first sustained onset.
Position error remains above the bound through 300.05 s, briefly drops
below it or has shorter excursions, and begins another sustained
episode at 301.15 s. The first onset also precedes the previously
identified 290.5 s pairwise gap cluster by 26.7 s. The timing shows
that this run had accumulated error before the cluster. It does not
identify when smoke began to affect the estimator or what caused the
drift. The track-count and output-gap criteria give no warning before
the first sustained episode. This exposed run gives no prospective
detector performance estimate.

An audit reproduces the episode boundaries, warning frames and output
gaps and checks all 820 overlapping samples against the earlier
scorer. The protocol, input hashes, extended grid and audit are under
`/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/rgb-warning/`.

## First drift onset

Use the existing alignments and interpolation caps over 250–275 s.
Decompose the change in aligned position residual between each pair
of endpoints into motion parallel and perpendicular to the horizontal
reference chord, with a separate vertical component. Positive
parallel change means the estimate advances farther along the chord.
The two short windows bracket the previously observed 263.8 s onset.

| Window, s | Position error at start → end | Estimated / reference net motion | Along / cross / vertical residual change | Relative rotation discrepancy |
| --- | ---: | ---: | ---: | ---: |
| 250–255 | 0.039 → 0.060 m | 2.753 / 2.743 m | +0.009 / +0.061 / −0.025 m | 0.271° |
| 255–260 | 0.060 → 0.192 m | 2.694 / 2.607 m | +0.085 / +0.094 / −0.043 m | 0.162° |
| 260–265 | 0.192 → 0.363 m | 2.889 / 2.713 m | +0.176 / +0.039 / −0.041 m | 0.245° |
| 265–270 | 0.363 → 0.711 m | 3.084 / 2.721 m | +0.363 / +0.034 / −0.009 m | 0.981° |
| 270–275 | 0.711 → 0.856 m | 2.198 / 2.052 m | +0.146 / +0.028 / +0.005 m | 2.157° |
| 261.8–263.8 | 0.240 → 0.300 m | 1.163 / 1.105 m | +0.057 / +0.022 / −0.012 m | 0.276° |
| 263.8–265.8 | 0.300 → 0.423 m | 1.195 / 1.070 m | +0.125 / +0.003 / −0.038 m | 0.257° |

The estimate accumulated error before crossing the threshold and
continued to overestimate forward travel immediately afterward.
Rotation discrepancy remained below 0.3° over both two-second
windows around the crossing. Thirteen RGB samples from 258 through
270 s show a doorway and corridor transition around onset. Structure
remains visible, and haze is more apparent in the later frames. The
motion comparison localizes the early discrepancy to translation
along the direction of travel. It does not establish global scale
error or isolate smoke, geometry, inertial bias, or reference error
as the cause. No fit, threshold, or estimator setting changed.

All 501 position-error magnitudes reproduce the previous grid.
Independent complex arithmetic verifies the displacement decomposition
for all seven endpoint pairs. The frozen protocol, tables, and contact
sheet are under
`/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/rgb-drift/`.
