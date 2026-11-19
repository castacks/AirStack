# IRS RTVI handoff to a continuously running thermal estimator

The [paced bridge](irs-rtvi-paced-imu-bridge-delivery-2026-11-18.md) preserved
the offline pose scores but left both early recovery errors above 0.15 m.
Test one distinct accuracy hypothesis using the native thermal estimates
already running from bag start. Retain the corrected visible prefix and
three IMU-only outputs. At the exact existing first thermal update, use
the separately estimated thermal trajectory through the remaining windows.
This is an offline output composition with recordings already used for
development. There is no concurrent estimator or transport timing result.

Align native thermal coordinates to the visible prefix with one
unit-scale yaw and translation fitted to the ten seconds before the
first flagged visible frame. Thermal interpolation brackets must finish
before that first flag. Visible estimates supply the target for this fit,
with no Vicon–IMU input. The two trajectory fits have 200 Medium and
199 Difficult pose pairs and overlap RMSE 0.064 and 0.059 m. Both use
the intended thermal update as the first thermal pose.

For position scoring, retain the shared pre-dimming Vicon–IMU fit for
the two methods with a visible prefix. The fixed thermal control has
its own fit on the identical timestamps. The three scoring windows
retain full common reference coverage in both recordings.

| Recording and arm | Dimming RMSE | First 2 s after light returns | 2–10 s after |
| --- | ---: | ---: | ---: |
| Medium, single-filter handoff | 0.104 m | 0.181 m | 0.134 m |
| Medium, continuously running thermal | 0.102 m | 0.120 m | 0.215 m |
| Medium, fixed thermal control | 0.102 m | 0.119 m | 0.215 m |
| Difficult, single-filter handoff | 0.098 m | 0.177 m | 0.209 m |
| Difficult, continuously running thermal | 0.182 m | 0.261 m | 0.220 m |
| Difficult, fixed thermal control | 0.182 m | 0.261 m | 0.220 m |

Switching to the warm independent trajectory meets 0.15 m in the early
Medium window and increases error on Difficult. In Medium the later
window also worsens. The output after the switch closely follows fixed
thermal by construction. This test compares a complete alternative
estimator state, including its native thermal preset and earlier sensor
history, so the result does not isolate thermal feature warm-up.

Both variants switch about 45.5 ms after the last IMU-only prediction.
For the warm trajectory, displacement-vector error across this
boundary is 9.1 cm on Medium and 6.4 cm on Difficult. The single-filter
handoff gives 0.6 cm and 0.8 cm at the same stamps. The independent
estimator's output needs a continuity criterion before deployment. The
observed RMSEs and boundary errors do not support promoting this fixed
switch policy across both recordings.

Independent complex-plane alignment arithmetic verifies both
cross-estimator fits and all 18 window scores to within 1e-10 m.
The frozen protocol, alignment pairs, common pose ledgers and checksums
are under `/root/ore-ms-artifacts/irs-author-rescore/warm-shadow/`.
