# DiTer++ PARK temporal feature persistence

Use the existing RGB and stretched thermal SIFT matches and their
[held-out epipolar classifications](diter-plusplus-park-geometry-2026-11-19.md).
Chain matches by exact image-coordinate equality at the shared
frame. Collapse duplicate coordinate pairs and remove links with
multiple distinct partners at either endpoint before applying the
geometric support filter. This removes responsibility for choosing
among SIFT orientation duplicates from the comparison. Across all
118 edges per camera, 7,385 RGB and 1,026 thermal duplicate rows were
collapsed, and 235 RGB and 56 thermal ambiguous distinct links were
excluded. Preserve every eligible start for five and fifteen links
within the three fixed windows. Every edge in a geometrically
supported chain meets the existing held-out one-pixel condition.
Survival fractions use the initial supported links at each start.

| Window | Median five-link chains, RGB / thermal | Median fifteen-link chains, RGB / thermal | Median fifteen-link survival, RGB / thermal |
| --- | ---: | ---: | ---: |
| Shaded approach | 209 / 10 | 69 / 2 | 13.5% / 3.2% |
| Passage exit | 206 / 24 | 64 / 5 | 12.0% / 5.4% |
| Daylight | 274 / 23 | 79 / 5 | 10.0% / 5.2% |

Five links span about 160–174 ms. Fifteen links span about 493–506 ms.
During the exit, RGB has at least eight geometrically supported
fifteen-link chains at all 45 starts, versus 3 of 45 for thermal.
Neither camera reaches zero chains at any eligible start. With the
geometry condition omitted, median fifteen-link counts at the exit
are 80 RGB and 8 thermal. Thermal track support persists through the
exit, with lower absolute counts and a smaller fraction of initial
links surviving. The eight-chain count is descriptive. These are
chains of adjacent image matches on one exposed passage. Stateful
odometry, physically correct landmark identity, and pose recovery
remain untested.

An independent reverse join reproduces 71,879 retained chains,
728 start/arm records, and all 24 summaries. The first audit used the
wrong endpoint as its lookup key. Correcting the audit reproduces the
original forward-join outputs. Frozen inputs, per-edge diagnostics and
chains are under `/root/ore-ms-artifacts/diter-park/frontend-track-persistence/`.
