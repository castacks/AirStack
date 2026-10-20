# CMU Starling Max 2 code organization

Integration candidates and internal development are separate. This branch is a working branch, not a merge-ready patch. Do not merge its entire history into upstream: old commits contain private aircraft labels and internal experiment material. Prepare a clean integration branch containing only the integration paths below.

## Integration candidates

- robot/ros_ws/src/gamma_zero_controller/: ROS controller, launch, configuration and unit tests; JSON golden samples are under test/fixtures/.
- robot/ros_ws/src/gamma_zero_msgs/: command interface.
- PX4 interface, SVG lifecycle/Foxglove changes and focused regression tests.
- docs/robot/autonomy/: integration guide.

## Internal development only

- internal/starling_max_2_ctbr_cmu/checks/: local campaign fixtures requiring external policy assets and isolated ROS tooling.
- internal/starling_max_2_ctbr_cmu/analysis/: recorded-flight analysis.

- docs/internal/starling_max_2_ctbr_cmu/history/: historical evidence, excluded from upstream integration.
- Checkpoints, logs, bags, environments and build outputs stay outside source control. Existing immutable artifacts are not renamed or rewritten.

## Compatibility and validation

The robot identifier is starling_max_2_ctbr_cmu; class names are StarlingPolicy and CTBRConfig. Export a fresh inputs JSON for the unchanged actor bundle with the new input-format identifier before running the renamed runtime. Do not edit existing receipts or weaken checksum checks. An old raw training checkpoint still contains its historical robot identifier; resuming it requires an explicit migration, not an alias silently added to the training roster.

Live mode remains gated on physical transforms, thrust calibration, firmware handoff/failsafe verification and attended props-off checks. This reorganization does not qualify flight or launch training.

## Reorganization validation — 2026-10-20

Naming-only model changes preserve numerical physics. No training, hardware connection or flight was started. Existing actor bundles and archived receipts were not modified. Feature source paths/content were scanned for the private aircraft label. New files are intent-to-add so `git diff -M` includes them; changes remain uncommitted and unpushed.

- Fresh dependency build failed because robot_interface could not discover airstack_msgs. The changed gamma_zero_controller package then built against the existing validated ROS dependency overlay into separate build-starling/install-starling directories. C++ source did not change in this reorganization.
- 220 unit/regression tests passed (41 controller, 179 SVG).
- 64-step cross-CPU actor parity passed: maximum physical error 2.1852552890777588e-5, under the existing 1e-4 tolerance. Socket latency median 5.16 ms, p95 5.65 ms, maximum 5.90 ms in this sample; not a real-time guarantee.
- Renamed interface, actual-policy backend and SVG lifecycle fixtures passed with synthetic FCU state. New receipts are in the existing external runtime's organization-validation directory; old receipts remain intact.
- New input metadata was exported and parity-checked on S2, then transferred to Xiaoxin. The actor SHA binding is unchanged. No hardware qualification is implied.
