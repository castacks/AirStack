# Current UAV development checkpoint

2026-11-15, `ore_proj`, source based on `07ff1dcbb` with the UAV checkpoint changes. Simulation only; one GPU; Cosmos deferred.

Four bounded cases pass independent physical/contact acceptance: takeoff/hover/land, moving operator PAUSE/cancel/land, automatic PAUSE from a dedicated monitor’s stale-source input, and an observation-conditioned marker viewpoint. Across capabilities 006–037, current dispositions are **4 PASS, 11 NOT DEMONSTRATED, 8 NOT IMPLEMENTED and 9 NOT TESTED**. PASS applies to the specific cases below, not broad autonomy or repeatability.

| Case | Evidence and limit |
| --- | --- |
| Takeoff/hover/land | Maximum physical z 2.08534 m, radius 0.14528 m, speed 0.45958 m/s. Full envelope, body landing support, terminal goals and fresh final disarm passed. Maximum truth gap 0.178 s. |
| Moving supervisory stop | Real monitor PAUSE at physical speed 0.310 m/s; maximum extra displacement 0.152 m; stable stop within 1.833 wall seconds including one-second dwell. Navigation canceled, normal landing/disarm completed. Full contact checks passed. |
| Automatic stale-source stop | Dedicated monitor input stamped ten seconds old while main estimator/controller kept genuine state. Physical speed at injection 0.314 m/s, maximum extra displacement 0.141 m, stop criterion met in 3.050 wall seconds including one-second dwell. Detector/ack transition, contact acceptance, cancellation and supervisor landing/disarm passed. |
| Observation-conditioned viewpoint | Planned 0.635 m from measured marker depth and acquisition-stamped camera TF; physical displacement 0.555 m. Recomputed raw frames show bbox area ×1.407 and optical surface depth 3.465→2.904 m with fixed calibration. Full navigation, contact and landing/disarm checks pass. Restricted fixture/oracle depth; no semantic-gain claim. |
| Relative localization | Preflight-only truth-assisted registration, then 1166 held-out motion pairs: RMS error 2.68 cm, maximum 5.34 cm. Does not qualify SLAM, global localization or GNSS denial. |
| Earlier survey | Five legs and settled captures completed; raw PhysX waypoint errors 3.4–6.7 cm. Camera area coverage and old contact qualification remain unknown. |
| Earlier fixed circle | Raw reference-to-PhysX RMS tracking error 17.2 cm with receipt/frame limits. Not a high-speed or precision-control qualification. |
| Restricted red fixture | Exact RGB/depth/calibration pairing; saved inspection frame matched at surface optical depth 2.73 m. Ideal simulated depth; no general recognition, object-center or world binding. |

All seven rigid bodies were monitored in the four passing cases. The predeclared contact policy permits body/floor support with at most 1 mm numerical penetration and camera/floor proximity only near ground with strictly positive separation and zero impulse. No unknown or undeclared interactions occurred in these cases. No geometry, mass, solver settings or backend admission thresholds were changed; the new source-validity and acknowledgement checks are stricter than the historical receive-only monitor. Old flights without separation remain unqualified; superseded assessments are retained.

A blocked attempt exposed client cleanup losing pending goal acceptance. Explicit goal IDs and pending futures now survive guard exceptions; late acceptance is canceled even before armed telemetry appears, and terminal results are reconciled. Three isolated actual-client contracts pass, including delayed acceptance canceled before mock arming. All four qualified episodes ended with no pending goals.

The watchdog now checks source timestamps, finite state and quaternion norm; a steady receipt deadline and wall timer also detect frozen-clock silence/replay. PAUSE retries are bounded and require a positive controller reply. Readiness remains faulted while intervention is pending; fresh state does not automatically resume motion. The earlier receipt-only/rejected-service limitations remain recorded as historical findings. The physical fault test used a dedicated instance of the installed monitor, a private stale-stamp relay and the real controller service. It did not corrupt PX4 state or qualify automatic LAND, all fault types, or exclusive controller authority.

The watchdog package passes all seven CTest checks; colcon aggregate has zero failures and two skips. The evaluator/perception/contact/stop suite passes 82 tests, including automatic-trigger evidence and timing-label cases. Three actual-client handoff regressions pass. Planner functional/GPU checks pass all 18; its broader historical lint/style failures remain open.

Nineteen public attempts remain visible: thirteen completed procedures, three incomplete airborne trials with recovery, and three blocks before flight. Different cases and source versions prevent pooling them into a success rate. No F6 root cause was established.

The simulator remains running for the user’s viewer. After the latest grounded restart, all seven AirStack readiness gates passed; postflight readiness also returned true. Kit reports streaming-server startup and its signaling TCP49100 is listening/reachable. UDP media and browser rendering were not directly verified; a socket check alone is not a rendered-viewer claim. PX4 is disarmed/on ground, the temporary qualification monitor has stopped, and the main monitor reports healthy/acknowledged. New flight evidence uses immutable complete-record snapshots while the viewer continues running.

Source and raw records remain local, with no external backup or push; source and compact result records are included in this checkpoint commit. GUI video was not recorded. No Gazebo or Cosmos was started.

See [execution details](uav-task-execution.md), [capability matrix](uav-task-campaign.md), [physics evidence](uav-physics-evidence.md), [discrepancies](uav-discrepancies.md), and [restricted perception](uav-marker-perception.md).

## Resume point

No active mission or pending goal remains. Isaac/PX4 are intentionally left running for the viewer, disarmed/on ground; recheck readiness, clock alignment and Kit signaling after every restart. Next work: controlled marker ambiguity/occlusion and depth-failure abstention, or exclusive mode authority/fault escalation contracts. Do not start another powered flight without a new declared case and fresh bounded recorders. Preserve old failures and narrow PASS scopes. GUI video, general semantic grounding, autonomous LAND escalation and broad repeatability remain open.

The minimal Pydantic-only core test workflow remains separate from the optional UAV pytest suites. Missing pytest/NumPy/OpenCV now yields explicit module skips rather than breaking core unittest discovery; four simulated missing-dependency checks pass. Optional suites execute inside the robot ROS container with those dependencies installed.
