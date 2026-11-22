# IRS Medium thermal admission using recent track counts

The five-frame history gave a small displacement benefit in the dimming
window at lower coverage. It did not improve both motion metrics over
accepting all updates, and the displacement benefit did not persist into
the first two seconds after visible light returned. This diagnostic does
not support using either count as an admission rule on its own.

## Method

The [policy literature note](thermal-policy-prior-art-and-lulea-intake-2026-11-21.md)
motivates comparing conditional and unconditional admission at fixed
coverage. The [track survival scout](track-survival-warning-scout-2026-11-20.md)
compared fresh and sustained image support. Here the score uses the actual
thermal estimator's accepted tracks and tests the incremental value of
recent counts. It uses no feature-identity persistence measurement.

Add logging just before track retirement and redetection in the native
15-feature RRxIO thermal replay. All 1,747 output poses reproduce the
previous native trace byte for byte. Every pose binds uniquely to a
track count within 512 ns. Use the existing Medium dimming and early
recovery windows. The thermal input is unmodified throughout. Candidate
scores are the accepted count on the present image or the mean of that
count and the four preceding images, with no future frames. Require both
scores and reject histories with a gap over 75 ms. All 240 opportunities
in the two windows are eligible.

Use consecutive published body poses and reference poses interpolated at
the same timestamps, with a 100 ms bracket limit. Score incremental rotation
and displacement against the released Vicon–IMU reference. Fit displacement
yaw in the preceding 10 seconds, at unit scale. Rank each window offline
by each score, breaking ties by time, and compare at equal retained
counts. This uses causal input scores and offline coverage selection.
It measures error among retained outputs of a continuously running
thermal estimator. The filter is unchanged by admission decisions, and
no switch, frame rejection, latency or trajectory recovery was tested.

## Results

| Window | Retained | Current count, displacement RMSE | Five-frame mean, displacement RMSE | Current count, rotation RMSE | Five-frame mean, rotation RMSE |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dimming | 100/200 | 5.99 mm | 5.44 mm | 0.0787° | 0.0779° |
| Dimming | 150/200 | 5.86 mm | 5.76 mm | 0.0758° | 0.0713° |
| Dimming | 180/200 | 5.92 mm | 5.93 mm | 0.0732° | 0.0728° |
| Dimming | 200/200 | 5.97 mm | 5.97 mm | 0.0713° | 0.0713° |
| Early recovery | 20/40 | 4.49 mm | 4.85 mm | 0.0653° | 0.0698° |
| Early recovery | 30/40 | 4.56 mm | 4.69 mm | 0.0687° | 0.0680° |
| Early recovery | 36/40 | 4.44 mm | 4.46 mm | 0.0668° | 0.0673° |
| Early recovery | 40/40 | 4.48 mm | 4.48 mm | 0.0669° | 0.0669° |

At half coverage during dimming, the mean reduces displacement RMSE
by 9.2% relative to the current count, a 0.55 mm difference per update.
Rotation RMSE remains higher than it is with full retention. During early
recovery, the mean has higher displacement RMSE than the current count
at all three partial coverages. Close this exact count-history rule on
Medium. A future comparison requires a different measured cue or a
separate qualified recording.

The scorer and protocol were hashed before scoring. An arithmetic
readback independently reconstructed all 32 RMSE entries and verified
native pose parity. Logs, code, per-opportunity scores and hashes are
under `/root/ore-ms-artifacts/irs-dual-preflight/admission-history/`.
