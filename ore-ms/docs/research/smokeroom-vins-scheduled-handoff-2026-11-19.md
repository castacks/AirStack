# SmokeRoom scheduled handoff from an initialized state

The [fixed RGB run](smokeroom-vins-fixed-replay-2026-11-19.md) drifted while
publishing normally, and its fixed thermal counterpart initialized too
late to qualify. Test whether full-rate, native-resolution thermal helps
after a single scheduled switch from an initialized RGB–IMU state.
Use 260.0 s relative to the first exported RGB header. The time is
selected on this exposed recording before its known 263.8 s sustained
error crossing. There is one switch and no return to RGB.

| Arm | Before the scheduled time | After the scheduled time |
| --- | --- | --- |
| Continuous RGB | Original RGB input | Continue RGB and existing tracks |
| RGB reset control | Same RGB input | Clear active visual tracks, then continue RGB |
| Thermal switch | Same RGB input | Clear active visual tracks, then use native-rate thermal |

Both intervention arms keep the body state, IMU preintegrations and
marginalized prior. The prior keeps its RGB extrinsic parameter in a
separate fixed slot. New factors use the active camera's own fixed
intrinsics and extrinsics. Each intervention drops 328 active feature
IDs and verifies exact prior residual parity at the switch. The reset
control uses the same remap and tracker reset with RGB calibration.
The audit reproduces all 850 pre-switch pose rows exactly across all
arms, including rotation and velocity, and verifies full continuous
RGB parity with the fixed replay. Each arm initializes once.

The first destination frame is 260.033670 s for RGB and 260.028703 s
for thermal. The respective switch gaps are 40.74 and 35.77 ms. Both
arms retain native timestamps, calibrated camera models, nominal time
settings, and the earlier EuRoC noise proxy. Thermal retains the fixed
mono16 min-max mapping from the previous run.

## Common-support metric results

Use the same unit-scale alignment on 245–250 s and score the common
20 Hz supported timestamps. This uses the published fused trajectory.

| Window, s | Common rows | Continuous RGB RMSE | RGB reset RMSE | Thermal switch RMSE |
| --- | ---: | ---: | ---: | ---: |
| 250–255, clear check | 100 | 0.043 m | 0.043 m | 0.043 m |
| 260–262, transition | 40 | 0.218 m | 0.219 m | 0.217 m |
| 262–280.5, pre-cluster | 370 | 0.716 m | 0.622 m | 1.811 m |
| 280.5–290.5, context | 200 | 1.067 m | 1.018 m | 2.955 m |
| 290.5–306.5, obscuration | 320 | 1.614 m | 1.033 m | 4.881 m |
| 306.5–316.5, later context | 200 | 6.884 m | 0.512 m | 3.267 m |

The thermal switch clears the 0.30 m transition bound, retains full
scoring coverage and keeps the maximum corrected-output gap near
50 ms. It fails the primary accuracy comparison against both RGB arms
during obscuration. Later thermal error is lower than continuous RGB,
while the RGB tracker reset achieves the lowest error in both windows.
The reset is a scheduled intervention on an exposed recording. This
experiment does not isolate which tracks or estimator effects explain
the gain, or establish a deployable reset rule. The published reference
also limits independent accuracy interpretation.

Close this full-rate scheduled thermal configuration. It supplies no
obscuration benefit to preserve in a reduced-rate or reduced-resolution
sweep. Other conditions, schedules, and estimators remain untested.
The frozen protocol, patch, switch logs, native frame ledgers and
independent audit are under
`/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/handoff/`.
