# FlashLight fixed-time RGB track-reset transfer

The earlier [SmokeRoom reset](smokeroom-vins-scheduled-handoff-2026-11-19.md)
improved trajectory error on one inspected run. Test the same intervention
on the March FlashLight export. The fixed 90 s reset time was chosen
before inspecting new images, poses or replay outcomes for this test.
This is an exposed second recording with a new stateful comparison.
The interval is not selected to match SmokeRoom degradation levels.

Use RGB and the published IMU over [65,120) s. Reuse the same VINS-Fusion
binary, fixed MEI model, camera-to-IMU transform, EuRoC noise proxy and
zero added clock shift. Retrieve the two sequence-linked calibration
files separately. Their bytes match the SmokeRoom downloads, despite
distinct release identifiers and filenames. This confirms the published
files used in the replay and leaves physical calibration applicability
unverified. The retrieved reference ZIP matches the prior hash.
The March exported timestamps preserve the previously documented limits.
[Prior FlashLight qualification](flashlight-qualification-findings-2026-11-16.md),
[released intrinsics](https://drive.google.com/file/d/13iTBn_po_GWxt3X8kNWVGZ2wwQ38Eou0/view),
[released extrinsics](https://drive.google.com/file/d/10YJQ3FMRw95F3_yhOsX2bbuMuvQbtnVV/view).

Both arms consume all 1,350 verified RGB images, initialize once,
and publish 1,331 corrected poses. IMU input includes 11,094
samples with padding. The driver adds a constant 170 s to both
input clocks to reuse the binary's 260 s switch point. It removes
that constant for scoring. Frame spacing and time differences are
preserved. All 595 pre-reset position, orientation, and velocity
rows match exactly. At the first image after 90 s, the reset clears
372 active feature IDs and verifies exact marginalized prior residual
parity. The body and inertial history remain initialized.

| Window, s | Common grid rows | Continuous RGB RMSE | RGB reset RMSE |
| --- | ---: | ---: | ---: |
| 80–85, alignment | 100 | 0.111 m | 0.111 m |
| 85–90, pre-reset control | 100 | 0.139 m | 0.139 m |
| 90–92, transition | 40 | 0.109 m | 0.366 m |
| 90–110, primary | 400 | 0.237 m | 2.707 m |
| 110–120, later | 200 | 0.411 m | 2.475 m |

The pre-reset control passes the 0.30 m / 90% gate. Both arms retain
full scoring coverage and corrected gaps under 42 ms. Reset raises
primary position RMSE by 2.470 m and gives no measured accuracy benefit
in either reported post-reset interval. The SmokeRoom gain therefore
does not transfer to this fixed-schedule FlashLight control. This
comparison tests one interruption to feature history, with no
health-triggered decision rule and a supplied fused reference.

The frozen protocol, input manifests, pose rows, patch pin, scores,
and independent audit of all ten RMSEs are under
`/root/ore-ms-artifacts/flashlight-reset-transfer/`.

## Comparison of post-reset motion and track return

Use fixed relative-time windows after the scheduled reset in FlashLight
and SmokeRoom, with each recording's existing reference alignment.
For descriptive timing, mark track-count return at the fourth
consecutive corrected frame at or above half the median count in the
preceding two seconds. Markers occur at 0.197 s in SmokeRoom and 0.316 s
in FlashLight. This rule was defined for the saved-trace comparison
and has no prospective trigger interpretation.

| Recording | Median tracks in first 2 s, continuous / reset | Position RMSE in first 2 s, continuous / reset | Position RMSE at 5–10 s, continuous / reset |
| --- | ---: | ---: | ---: |
| SmokeRoom | 106 / 102 | 0.218 / 0.219 m | 0.530 / 0.477 m |
| FlashLight | 118 / 119 | 0.109 / 0.366 m | 0.091 / 3.693 m |

In FlashLight, the reset trajectory moves a net 0.543 m in the first
two seconds and 5.506 m in the first ten. The supplied reference moves
0.028 m and 0.116 m, respectively. The continuous arm estimates
0.028 m and 0.099 m over those same intervals. The two estimated
orientations remain within 0.64° of each other at the ten-second
endpoint. Although the median track count equals 143 in both arms
at 5–10 s, the reset arm already has substantial translation error.

The SmokeRoom reference moves 1.089 m in the first two seconds.
Its two estimates differ by less than 0.009 m in position at the
two-second endpoint, and the reset advantage grows later. The two
experiments differ in post-reset motion, scene, and reset time. The
FlashLight result identifies spurious translation in a nearly stationary
interval. It does not identify an observability failure, a scale
mechanism, or a safe velocity threshold for deciding when to reset.

An independent count check confirms the native return markers, and
both early RMSEs reproduce the previous input grids. The frozen
analysis, hashes, per-window results and endpoint checks are in
`/root/ore-ms-artifacts/rgb-reset-comparison/`. No replay was run.

## Moving-interval reset eligibility

To keep the camera, calibration, source data, and estimator fixed,
screen three reset times in the existing FlashLight replay. Before
opening their reference positions, declare the first of 100, 105, or
110 s eligible if it has at least 0.5 m net displacement in the
preceding two seconds and at least 2.0 m in the next five. The supplied
pose is used to select an offline case, never as an estimator input.
Candidate displacements use the existing 20 ms interpolation cap.

| Candidate, s | Prior 2 s net displacement | Next 5 s net displacement | Eligible |
| --- | ---: | ---: | --- |
| 100 | 0.109 m | 3.884 m | No |
| 105 | 1.088 m | 0.911 m | No |
| 110 | 1.113 m | 1.920 m | No |

All endpoints are supported. No candidate satisfies both bounds, so
this fixed three-time screen ends before replay. The result does not
rule out a moving interval elsewhere in FlashLight. It leaves the
within-recording motion comparison unresolved. Independent interpolation
reproduces all six distances. Protocol, selection and input hashes are
under `/root/ore-ms-artifacts/flashlight-reset-transfer/moving-reset/`.
