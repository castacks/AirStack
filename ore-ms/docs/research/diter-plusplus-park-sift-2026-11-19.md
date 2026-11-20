# DiTer++ PARK image-space SIFT support across the passage exit

Reuse the three [timing preflight windows](diter-plusplus-park-intake-2026-11-19.md#message-timing-and-calibration-preflight).
Use every RGB header and its distinct nearest thermal header within
16.7 ms, with earlier timestamps breaking ties. Score consecutive
paired anchors within each window. Preflight and hash all 242 decoded
image inputs before SIFT. Retain native resolution, the repository's
SIFT settings and mutual 0.75 ratio matching, with single-threaded
OpenCV 4.10.0 and NumPy 2.2.6. Report match count and occupancy on a
4 × 4 image grid. Occupancy is the smaller endpoint count for each pair.
[Existing SIFT helper](../../tools/smokeseer_sift.py).

Full-range linear conversion of the native mono16 data produced zero
thermal matches in all 118 pairs. The median across frames of the 1st,
50th, and 99th gray percentiles was 87, 89, and 90 in all three windows.
After seeing this result, freeze one thermal preprocessing control
using the existing diagnostic rule. Stretch each raw thermal frame
between its 5th and 95th percentiles, clip, and cast to uint8. Reuse the
original RGB results and hold every message digest, pair, SIFT setting,
and metric fixed. This assesses conversion sensitivity on already
exposed frames. [Existing percentile rule](../../tools/diagnose_fixed_thermal_groups.py).

| Window relative to scout center | Pair anchors | Median RGB matches | Median stretched thermal matches | Median RGB / stretched thermal occupied cells |
| --- | ---: | ---: | ---: | ---: |
| +5 to +6 s, shaded approach | 30 | 605 | 64.5 | 14 / 9 |
| +8 to +10 s, passage exit | 59 | 648 | 103 | 16 / 12 |
| +12 to +13 s, daylight | 29 | 929 | 127 | 16 / 14 |

All 118 pairs in each of the RGB and stretched thermal arms have at
least eight mutual matches. RGB has more matches on every paired
anchor. The first thermal arm's zero counts reflect strong sensitivity
to the 16-to-8-bit conversion. The exposed sequence contains geometry
and scene-content changes along with illumination. Mutual matches
measure candidate correspondence support. This test leaves geometric
correctness, stateful tracking, and metric recovery unmeasured.

Independent tally scripts check every saved pair against the input
lock, recompute grid occupancy from match coordinates, and reproduce
all nine window/arm summaries. The contrast control's first preflight
stopped on a list-versus-tuple comparison. The corrected comparison
verifies the same timestamp pairs and message digests before inference.
The protocols, input locks, retained failure, match records, runners
and audits are under `/root/ore-ms-artifacts/diter-park/`, with results in
`frontend-sift/` and `frontend-sift-thermal-contrast/`.
