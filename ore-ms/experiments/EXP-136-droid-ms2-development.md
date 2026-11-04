# EXP136: DROID stereo metric contract and initialization limits

2026-11-04. Parent EXP135. Initial freeze `37fc4bad1`; capacity/lifecycle
amendment `7dd04bef8`. [Full findings](../docs/research/droid-ms2-development-findings-2026-11-04.md).

Pinned source/checkpoint and Blackwell runtime build. 72 tests, 23 CPU/CUDA
metric-geometry controls and amended independent1,818checks/30mutations pass.
Provided-baseline/.1 scales translations; no EuRoC1.10 or reference scale fit.
Original integer source clocks and camera-local output direction are explicit.

Initial full-resolution RGB capacity failures and uninitialized-backend failures
remain retained. Separate native-area-cap/intrinsics and lifecycle amendment
processes all120source frames: onlydaythermal reaches8keyframes and returns20
admitted offline poses; otherfive cases remain uninitialized/null. No threshold
tuning or filled-uninitialized success. This is engineering availability, not
RGB/thermal accuracy or adaptive superiority.

Next predeclared longer exposed recording/prefix and common-time qualification;
benchmark stays unready and reserves closed. Recovery
`backups/2026-11-04-droid-ms2-development-exp136`.
