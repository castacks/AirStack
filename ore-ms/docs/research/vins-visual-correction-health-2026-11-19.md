# Do visual–inertial state corrections warn of drift?

The [SmokeRoom output and track-count check](smokeroom-vins-fixed-replay-2026-11-19.md#saved-rgb-warning-timing)
missed the onset of sustained RGB position error. Check whether VINS-Fusion's
state change at a visual update supplies an earlier indication. Instrument
the pinned serial runner immediately after IMU propagation and after image
processing. At each camera timestamp record the Euclidean change in
position and velocity. The two states share their prior measurements and
the post-update state includes the window optimizer. These are state
correction magnitudes, rather than independently measured position error.

The logging-only replay reproduces all 2,237 SmokeRoom RGB poses and all
1,331 poses in each FlashLight arm byte for byte, including published
velocity. Telemetry starts at the second corrected pose and binds to
subsequent camera timestamps. The scorer uses trailing 250 ms medians
with at least four samples. Each score has a fixed alert rule of three
consecutive frames above the 99th percentile in the prior clear scoring
window. Compute the FlashLight bounds on continuous RGB and carry them
unchanged into the reset arm. At runtime these scores would need the
current corrected state and prior measurements. Here their timing is
measured in sensor time and contrasted with an offline supplied-reference
position error trace.

| Case | Sustained 0.30 m error onset | First position alert in inspected interval | First velocity alert in inspected interval |
| --- | ---: | ---: | ---: |
| SmokeRoom RGB from 255 s | 263.80 s | None through 265.8 s | 256.77 s, with another at 257.30 s |
| FlashLight continuous RGB, 90–100 s | None in interval | None | None |
| FlashLight RGB reset, 90–100 s | 90.80 s | 94.72 s | 96.67 s |

SmokeRoom has no alert from either score in the two seconds before or
after 263.8 s. The early velocity alerts precede the crossing by about
seven seconds and clear before the onset. The result gives no evidence
that they identify the impending degradation on a usable decision
horizon. On FlashLight the first position and velocity alerts lag the
reset drift onset by 3.92 and 5.87 s. Neither score alerts in the first
two seconds after reset. Small current corrections can coexist with
substantial accumulated state error.

The frozen rule does not qualify as a detector on these exposed runs.
A [signed velocity follow-up](vins-longitudinal-correction-warning-2026-11-20.md)
also missed the warning windows. Persistent state bias and external motion
constraints remain untested. A separate audit reproduces
4,887 trailing windows, all four thresholds, and every reported alert.
The source, native telemetry, logs and analysis are under
`/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/visual-correction/`.
