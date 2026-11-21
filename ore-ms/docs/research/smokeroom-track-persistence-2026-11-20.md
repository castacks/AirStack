# SmokeRoom RGB–thermal track persistence

The earlier [SmokeRoom intake](smokeroom-stateful-recovery-intake-2026-11-19.md)
identifies a clear moving segment, subsequent obscuration and improved
visibility later. This follow-up measures whether image feature identities
persist through short windows in each condition, complementing the
half-second pairwise and stateful comparisons.

Use fixed 20-image windows at 10 Hz starting at 245, 298 and 314 s relative
to the first RGB timestamp. These timestamps come from the exposed
clear, obscured and later spans in the intake. Bind each modality to
the closest unique acquired image (25 ms to target, 40 ms between sensors).
Thermal uses per-image min–max scaling and RGB uses grayscale conversion
at native resolution. Both use up to 300 Shi–Tomasi corners (quality
0.01, 8 px separation) and pyramidal Lucas–Kanade (21 × 21, three pyramid
levels above the base level, 30 iterations, epsilon 0.01). Keep only
forward and backward successes with a round-trip error of at most one
pixel and an in-bounds destination. Count adjacent tracks after
reseeded detection and persistent tracks carried from the first image
without reseeding.

Archive ranges must return exact HTTP 206 ranges and each selected
member passes ZIP CRC verification. The public SmokeRoom and FlashLight
reference ZIPs were also restored and passed ZIP CRC verification. The
restored FlashLight hash matches the
[previous qualification](flashlight-qualification-findings-2026-11-16.md).
[SmokeRoom images](https://drive.google.com/file/d/140UJgOCa-2itHUZQfwOK1zISD8XRGw51/view),
[author reference folder](https://drive.google.com/drive/folders/1bSGDe1FFQYEjs20P8L15pTx7T0HCn2L0).

| Window | RGB initial / surviving at image 20 | Thermal initial / surviving at image 20 | Median adjacent RGB / thermal |
| --- | ---: | ---: | ---: |
| Clear, 245 s | 300 / 151 | 16 / 14 | 265 / 15 |
| Obscured, 298 s | 157 / 107 | 83 / 25 | 150 / 100 |
| Later, 314 s | 235 / 122 | 10 / 0 | 218 / 238 |

Thermal adjacent support increased from clear to obscured as RGB
support fell. RGB still carried more of its original feature identities
through the obscured window. In the later window, thermal had high
adjacent counts after reseeding, while all ten original tracks were
lost by image 5. This distinction matters when interpreting fresh
feature availability as evidence of recovery. Camera views and
preprocessing differ, and the windows are exposed. The supplied pose
positions cover about 1.10, 0.87 and 1.11 m of sampled path, respectively.
This diagnostic does not score estimator accuracy or test a handoff.

All 120 selected images decoded with the declared types and dimensions.
A separate audit checked saved image hashes, uniqueness, timing and
monotonic survival counts, and recomputed the table from per-frame
rows. Maximum RGB–thermal offsets were 29.51, 25.20 and 30.97 ms.
Source, audit, reference archives, frame receipts and per-frame counts
are saved under `/root/ore-ms-artifacts/subt-recovery-access/`.
