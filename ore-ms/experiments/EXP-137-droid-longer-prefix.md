# EXP137: longer exposed prefixes resolve bounded DROID initialization

2026-11-04. Parent EXP136. Source/native freeze `50c9819f6`.
[Full findings](../docs/research/droid-longer-prefix-findings-2026-11-04.md).

Fixed original0..100prefixes from three exposed day/night/rain records, both cameras.
All6 initialize and return101/101finite offline poses. Previously uninitialized
cases first initialize at sources21/56/60/58/29; daythermal remains18. Only allocated
buffer64→128 and prefix length change; motion/warmup settings and runtime stay fixed.

240overlap canonical PNGs and all120first20actual tensor/clock/K/frontend traces
match prior EXP136, zero categorical divergence. 82tests and13874independent checks/
30mutations pass;2467pins unchanged. All12916recovery members hash-checked, only1200
camera PNGs extracted, no actual reference scoring/reserves/training/default adoption.

Offline future-image refinement does not retroactively turn prior uninitialized
online states into successes. No pose accuracy/adaptive benefit/generalization or
whole-recording claim. Next explicit common-target-time qualification and recording-
level comparison. Recovery `backups/2026-11-04-droid-longer-prefix-exp137`.
