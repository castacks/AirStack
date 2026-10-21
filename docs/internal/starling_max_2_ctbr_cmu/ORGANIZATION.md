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

Naming-only model changes preserve numerical physics. No training, hardware connection or flight was started. Existing actor bundles and archived receipts were not modified. Feature source paths/content were scanned for the private aircraft label. The reorganization was committed locally; no branch was pushed.

- Fresh dependency build failed because robot_interface could not discover airstack_msgs. The changed gamma_zero_controller package then built against the existing validated ROS dependency overlay into separate build-starling/install-starling directories. C++ source did not change in this reorganization.
- 220 unit/regression tests passed (41 controller, 179 SVG).
- 64-step cross-CPU actor parity passed: maximum physical error 2.1852552890777588e-5, under the existing 1e-4 tolerance. Socket latency median 5.16 ms, p95 5.65 ms, maximum 5.90 ms in this sample; not a real-time guarantee.
- Renamed interface, actual-policy backend and SVG lifecycle fixtures passed with synthetic FCU state. New receipts are in the existing external runtime's organization-validation directory; old receipts remain intact.
- New input metadata was exported and parity-checked on S2, then transferred to Xiaoxin. The actor SHA binding is unchanged. No hardware qualification is implied.

## Obsolete-path cleanup — 2026-10-20

AirStack is the sole maintained ROS deployment path. The obsolete standalone S2 runner/bridge, configuration, fixture, session/worker copies and runbook were removed from the policy repository. This AirStack package and its current validation tools remain active. The analyzer in internal/starling_max_2_ctbr_cmu/analysis/analyze_flight.py supports both selected-command logs and legacy prepared/published pairs, so the old S2 analyzer is unnecessary. No AirStack runtime behavior changed in this cleanup.

## Review cleanup — 2026-10-20

Formatted package manifests and setup.py for review; XML element content and Python AST were checked unchanged. Replaced nonexistent normalized example locations with explicit operator-supplied paths, clarified actor/metadata socket wiring, corrected source-provenance wording and highlighted that the watchdog also affects conventional mode. No controller, interface, launch or configuration behavior changed.

## Simple trajectory validation — 2026-10-20

Implemented directly on the dronesoccer deployment worktree: hover, forward/backward, yaw and altitude routines, selected by launch argument. Existing figure-eight defaults remain unchanged. No live aircraft connection or flight was started.

- Controller rebuild and 238 controller/SVG regression tests passed, including yaw feedforward, fence admission and timed completion.
- Frozen 302M actor parity passed for 64 samples with zero observed action/physical error; socket median 0.605 ms, p95 0.752 ms, maximum 1.198 ms (sample measurements, not real-time guarantees).
- All four 32-second routines completed in native MuJoCo at 0 and 100 ms additional command delay. Position RMS across eight runs: 0.0602–0.0702 m; maximum position error: 0.1199 m. Yaw-routine heading RMS: 0.0187/0.0197 rad. These results do not validate physical braking or firmware behavior.
- Synthetic ROS lifecycle passed with the yaw launch override and actual policy worker: takeoff, engage, start, hold, handback, land, idle; recorder produced a 40-topic bag.
- Receipts and native results are outside source control in the deployment runtime simple-pattern-validation directory. Physical transforms, thrust calibration, firmware handoff/failsafe and attended checks remain open.
