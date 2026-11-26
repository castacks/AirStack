# ORE-MS-ISAAC-REMEDIATE-001-R1

**REMEDIATION VERIFIED** for the published image and its recreation on the existing OSMO pod. The fresh-workflow and scientific pilot gates remain outside this result.

| Gate | Disposition | Evidence |
|---|---|---|
| R01 | PASS | M11 and earlier readiness evidence unchanged. Existing core dump checksum preserved. |
| R02 | PASS | Ported the reference NumPy pin and late version check with constraints on the two later installs. |
| R03 | PASS | PX4 venv and SITL present and used, MAVROS namespace source unchanged, live odometry received. |
| R04 | PASS | Build completed the final Isaac Python assertion. An independent check passed for the candidate and rejected NumPy 2.4.6 in the old image. |
| R05 | PASS | Both profiles resolved the published digest and preserved the default, robot and GCS references. |
| R06 | PASS | Unique tag published, Linux amd64 registry manifest verified and pulled by digest. |
| R07 | PASS | Pulled digest used for clean service recreation. Isaac, ROS, Pegasus and PX4 executed in the normal workflow environment. The separate offline Pegasus probe was blocked by external asset resolution. |
| R08 | PASS | RGB8, 32FC1 depth, matching CameraInfo and a 12,091-point raw LiDAR cloud received. |
| R09 | PASS | Steady clock monotonic in domain 1, MAVROS recovered and pod-side Foxglove delivered sensor, pose and map messages. |
| R10 | PASS | No in-container package install or NumPy downgrade was used for the recreated service. |
| R11 | PASS | Complete build log, failed probe and runtime excerpts, measurements, checksums and reproduction instructions preserved. |
| R12 | PASS | Task source and evidence submitted on the research branch. The delivery records the remotely verified commit SHA. |

[Implementation](IMPLEMENTATION_DIFF.md) · [Build](BUILD_PROVENANCE.md) · [Manifest](IMAGE_MANIFEST.md) · [Clean runtime](CLEAN_RUNTIME_VERIFICATION.md) · [Smoke test](SENSOR_SMOKE_TEST.md) · [Regression](REGRESSION_RESULTS.md) · [Remaining blockers](REMAINING_BLOCKERS.md)
