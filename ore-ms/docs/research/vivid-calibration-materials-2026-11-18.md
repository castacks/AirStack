# ViViD++ released IMU settings and stronger-motion check

The previous [metric inertial audit](vivid-imu-metric-audit-2026-11-18.md)
needed same-rig stationary data and translation-rich reference motion.
The [author download page](https://visibilitydataset.github.io/4_download.html)
provides a calibration archive and a Drive mirror. The archive has
12 YAML files. The mirror's `handheld_targets` and `handheld_results`
folders show those handheld target and result files. The visible
`handheld_files` folder has no listed children. None of these locations
provides a raw stationary calibration recording.

## Released Microstrain settings

The prior inventory overlooked `handheld_targets/Microstrain.yaml`.
It names `/imu/data` at 100 Hz and supplies four IMU noise and random-walk
values. The release gives no underlying raw stationary recording,
estimation procedure, or explicit units for those values. These are
the authors' supplied Kalibr settings. Their measurement provenance
and applicability to the particular recording remain unverified.

| Field | EuRoC proxy in the previous OpenVINS run | Released Microstrain YAML |
|---|---:|---:|
| Accelerometer noise density | 2.0000e-3 | 2.50e-5 |
| Accelerometer random walk | 3.0000e-3 | 4.00e-5 |
| Gyroscope noise density | 1.6968e-4 | 5.00e-3 |
| Gyroscope random walk | 1.9393e-5 | 2.22e-3 |

The [bright OpenVINS control](vivid-openvins-bright-control-2026-11-18.md)
was repeated once with only those four values changed as provided.
The camera, geometry, clocks, initialization and executable stayed fixed.
Replay exited normally and gave 1,385 reference-associated poses,
**89.88% coverage** and **1,944.80 m position RMSE** compared with the
earlier 322.96 m result. It again misses the frozen 90% / 0.30 m gates.
The observed change does not resolve which noise assumption caused
either failure. No unit conversion or parameter search was applied.

## Translation-rich candidate

The author manifest lists ten indoor mocap CSVs. Seven additional small
references were downloaded, about 4.0 MB combined. Six non-robust
recordings passed the pose-only screen of at least 20 translation-exciting
anchors at the existing 0.5 m/s² threshold. The selected bright
`global_aggresive` recording has the largest reference acceleration RMS
among the bright candidates and 132 exciting anchors in that screen.
Its spelling in the bag differs from the `global_aggressive` reference
calibration entry. That entry supplies a +0.115 s shift.

Before acquisition, the KAIST range-read bag index confirmed the RGB
and external IMU streams and 1,686 IMU records. The Google Drive connector
listed the official mirror file but rejected its binary download at the
256 MiB connector limit. Resumed the partial direct transfer from the
authors' public mirror after matching the first 8 KiB and 64 KiB of
overlap. Verified the full KAIST footer and recorded the SHA-256.
Only this 1.16 GB bag was staged. The decoded external IMU topic has all
1,686 records, with the `imu_link` frame and increasing sensor timestamps.

Under the unchanged metric audit on the common reference/IMU support,
133 of 166 anchors meet the acceleration threshold at the primary
150 ms half-width. **IMU–mocap acceleration RMSE is 6.09 m/s²** against
the 0.50 m/s² limit. The 300 ms half-width gives 5.29 m/s².
Gyro integration over 100 ms gives 8.48° p95 disagreement on 130 turns
of at least 1°. The bag contains no qualifying stationary second.

This closes the low-motion explanation as sufficient for the earlier
metric failure. It preserves the unresolved clock, reference and sensor
hypotheses. More scene motion did not qualify this nominal binding.
The original raw stationary and dynamic calibration captures, or another
independently verified same-rig reference, are still needed before
turning these results into a metric camera-switch trial.

Manifests, extracted settings, replay configurations, complete ledgers,
and the audit are in
`/root/ore-ms-artifacts/vivid-plusplus-intake/calibration-materials/`.
The bag is staged at
`/osmo/run/workspace/vivid-plusplus/global_aggresive.bag`.
No Box storage was used.
