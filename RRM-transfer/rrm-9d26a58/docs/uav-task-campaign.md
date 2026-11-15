# UAV capability execution matrix
2026-11-15; ore_proj, source based on07ff1dcbb with the current UAV checkpoint changes. Simulation only; one GPU; Cosmos deferred.

Scope006–037; historical001–005 retained separately. PASS applies only to the stated case/source. Earlier no-separation flights remain unqualified. Machine-readable matrix: [CSV](uav-campaign.csv).

Primary counts: NOT DEMONSTRATED: 11, NOT IMPLEMENTED: 8, NOT TESTED: 9, PASS: 4

Public attempts19:13completed procedures (including intentional cancellation),3incomplete airborne recoveries,3blocked beforeflight. Different cases/source versions; no pooled success-rate claim.

## 006 — Object detection and localization
Primary verdict: **NOT DEMONSTRATED**
Source: `rrm/uav_marker.py; scripts/uav_marker_capture.py; uav_marker_qualification.py`

Restricted red-fixture detector developed, 13 tests pass. Passive development captures 19/20 OBSERVED plus 1 ambiguity, then corrected 20/20. Concurrent marker inspection retained 2,000 OBSERVED exact RGB/depth/calibration pairs and one exact saved-image match with surface optical z 2.72554 m. Uses oracle simulated depth; no general detection, object-center localization or world binding. The observation-conditioned viewpoint also preserves six raw exact-pair frames, acquisition TF and recomputed detector evidence; no object-center/world pose binding is added.

Next procedure / physical concern: Develop bounded detector with labelled scene truth; test image to object ID and framed pose, occlusion and uncertainty.

## 007 — Autonomous inspection
Primary verdict: **NOT DEMONSTRATED**
Source: `tasks/navigate; front_stereo/left/image_rect`

Inspection-01 completed public takeoff, 0.75 m navigation, physical settling, fresh image capture, landing/disarm. PhysX camera-capture position about 0.094 m from raw map waypoint. This is viewpoint acquisition in an empty grid; semantic inspection and collision evidence absent. Survey-03 added five successful settled camera viewpoints, PhysX raw-frame errors 0.034–0.067 m. Marker-inspection-01 also completed a bounded viewpoint flight with exact-stamp red-fixture/range observation, then landed/disarmed; target viewpoint was predefined, not inferred from perception. A later observation-conditioned marker viewpoint completed with full contact evidence and40.7%bbox-area gain; semantic asset inspection remains unqualified.

Next procedure / physical concern: Develop stop-and-observe route; constrain camera range/incidence, motion blur and dwell; require timestamped image at reached viewpoint.

## 008 — Target tracking
Primary verdict: **NOT IMPLEMENTED**
Source: `current robot source/graph inventory`

Restricted static-fixture detection now exists. No moving-object association/pursuit or loss/occlusion tracking workflow has been qualified; repeated detections alone are not target tracking.

Next procedure / physical concern: Depends on detection, latency-aware state association and bounded pursuit; test occlusion/target loss before pursuit.

## 009 — Adaptive decision-making
Primary verdict: **NOT TESTED**
Source: `rrm/loop.py; random_walk_node.cpp`

Existing replanning and behavior transitions; no live adaptive qualification.

Next procedure / physical concern: Inject changed semantic fact in replay, then test monitor-triggered hold/replan with fixed backend.

## 010 — Fault detection and recovery
Primary verdict: **NOT DEMONSTRATED**
Source: `drone_safety_monitor.cpp; takeoff_landing_task.cpp`

Survey-01 stale telemetry caused harness cancellation and recovery landing/disarm; Survey-02 planner source-stale rejection also recovered. Real safety monitor independently passed isolated silence detection and mock PAUSE request. Integrated fault campaign remains incomplete. Actual installed watchdog DDS90 detected receipt silence but continuous10s-old source data stayed healthy; rejecting mockPAUSE service received no retry during3s outage. Automatic physical fault recovery remains unqualified. New source-aware monitor passed actual-binary isolated contract and a dedicated-monitor injected stale-source physical stop. Supervisor cancellation/LAND completed recovery; no automatic LAND policy or general fault campaign qualification.

Next procedure / physical concern: Grounded watchdog harness first; airborne fault requires independent stop/landing capability and truth.

## 011 — Energy-aware autonomy
Primary verdict: **NOT DEMONSTRATED**
Source: `interface/mavros/battery`

Battery messages observed; some fields NaN. No measured discharge-dependent mission response.

Next procedure / physical concern: Record finite charge/voltage first; test injected low-battery response in isolated simulator, never treat NaN as healthy.

## 012 — Autonomous landing and docking
Primary verdict: **NOT DEMONSTRATED**
Source: `takeoff_landing_planner/src/takeoff_landing_task.cpp`

Normal landing/disarm independently qualified in one separation-aware vertical episode, including body floor support and fresh ground/disarm. Docking NOT IMPLEMENTED; combined capability remains NOT DEMONSTRATED.

Next procedure / physical concern: Add independent PhysX descent/contact evidence; docking needs pad-relative estimate and contact/latch criteria.

## 013 — SLAM and localization
Primary verdict: **NOT DEMONSTRATED**
Source: `vdb_mapping_ros2; odometry_conversion`

Actual motion diagnostic after preflight-only truth-assisted yaw/translation registration:1166held-out pairs, positionRMSE0.02677m,max0.05345m, final0.01910m.0unpaired; source epoch mismatch up to0.06s. No loop closure, map correctness or unaided global localization qualification.

Next procedure / physical concern: Compare localization against simulator truth over motion; classify drift and map correctness separately.

## 014 — Multi-UAV autonomy
Primary verdict: **NOT TESTED**
Source: `config/fleets; example_multi_px4_pegasus_launch_script.py`

Current scene one UAV.

Next procedure / physical concern: Validate namespace, per-vehicle command authority and static separation before any coordinated motion.

## 015 — Trajectory and motion planning
Primary verdict: **NOT DEMONSTRATED**
Source: `droan_gl; trajectory_library`

Inspection-01 NavigateTask succeeded with physical arrival evidence. Survey-02 first navigation failed admission on 0.54 s commanded start source age. Both outcomes retained; robustness unqualified. Depth-1 latest-state subscriptions built and all 18 functional/GPU regressions passed; survey-03 completed five legs. Broad package lint suite remains failing.

Next procedure / physical concern: After vertical baseline, short open path with independent clearance; distinguish reference from actual airframe.

## 016 — Flight control and trajectory tracking
Primary verdict: **NOT DEMONSTRATED**
Source: `pid_controller; trajectory_controller`

Fixed-circle radius 0.5 m/requested 0.3 m/s completed with return/settle and landing/disarm. Raw tracking-point vs PhysX RMSE 0.17174 m, max 0.29498 m over 283 receipt pairs; peak physical speed 0.42245 m/s. Frame/time alignment and contact limits prevent precision/control-envelope qualification.

Next procedure / physical concern: Record tracking point plus physical pose, align epochs/frames, measure lag and RMSE; do not infer physical tracking from action success.

## 017 — Autonomous takeoff
Primary verdict: **PASS**
Source: `takeoff_landing_planner/src/takeoff_landing_task.cpp`

Separation-vertical-02: one bounded public takeoff/hover/land episode independently passed PhysX envelope/contact acceptance and final disarm. Maximum physical z2.08534m, radius0.14528m, speed0.45958m/s; no unknown/undeclared contacts. Predeclared separation-aware policy; old no-separation flights remain unqualified. New monitor source also completed takeoff/hover/landing portions of the automatic stale-source stop episode with full independent contact checks; not a matched repeatability run.

Next procedure / physical concern: Repeatability and broader takeoff conditions remain unqualified; preserve all prior blocked/incomplete attempts.

## 018 — GNSS-denied navigation
Primary verdict: **NOT TESTED**
Source: `mavros_interface; PX4 estimator`

Normal localization does not demonstrate GNSS denial.

Next procedure / physical concern: Establish alternative localization and measurable validity before degrading GNSS; landing remains available.

## 019 — Airborne detect-and-avoid
Primary verdict: **NOT IMPLEMENTED**
Source: `MAVROS adsb plugin; coordination_msgs`

Plugin presence alone does not implement avoidance; no suitable encounter workflow found.

Next procedure / physical concern: Need cooperative traffic truth, time-to-collision and separation authority before encounter trial.

## 020 — Geofencing
Primary verdict: **NOT DEMONSTRATED**
Source: `MAVROS geofence plugin; rrm/uav_acceptance.py`

Independent offline envelope assessor added and 11 tests pass. Probe enforces measured 2 m horizontal / 3 m altitude / 2 m/s envelope by landing on violation or stale telemetry. No runtime boundary-crossing trial or planner geofence qualification.

Next procedure / physical concern: Test command rejection and independent runtime boundary intervention separately, including braking distance.

## 021 — Contingency and emergency landing
Primary verdict: **NOT DEMONSTRATED**
Source: `takeoff_landing_task.cpp; mavros_interface.cpp`

Survey01/02 and supervised-stop01 aborted airborne and recovered with public LAND and final disarm. Actual-client mock tests confirm cancel-before-LAND and unresponsive-cancel/publicLAND fallback ordering. Fault-specific integrated emergency campaign and fallback-path physical qualification remain incomplete.

Next procedure / physical concern: Fault-specific detection, mode handover, descent and disarm must all be recorded; no airborne forced disarm.

## 022 — Communication-aware autonomy
Primary verdict: **NOT TESTED**
Source: `PX4 failsafe_flags; MAVROS state`

PX4 reports GCS lost despite Foxglove connected; Foxglove is not equivalent to PX4 GCS heartbeat.

Next procedure / physical concern: Distinguish DDS, MAVLink and UI loss. Test one link at a time with independent observation and recovery.

## 023 — Wind-aware flight
Primary verdict: **NOT TESTED**
Source: `Pegasus logic/dynamics/linear_drag.py`

Drag exists; no wind disturbance workflow qualified.

Next procedure / physical concern: Need explicit wind vector/force with ground truth, then evaluate drift/recovery and controller saturation.

## 024 — Semantic scene understanding
Primary verdict: **NOT IMPLEMENTED**
Source: `SemanticSearchTask.action; ZED config`

Semantic search action interface/client does not establish server or scene understanding.

Next procedure / physical concern: Build small authored-label perception pipeline with negative cases; no Cosmos in this session.

## 025 — Behavior and intent prediction
Primary verdict: **NOT IMPLEMENTED**
Source: `current robot source/graph inventory`

No benign-object intent predictor found in current UAV stack.

Next procedure / physical concern: Requires temporal tracked-object evidence, prediction horizon and independent held-out trajectories.

## 026 — Coverage path planning
Primary verdict: **NOT DEMONSTRATED**
Source: `fixed_trajectory_task.cpp (LawnMower); scripts/uav_vertical_probe.py (sequential survey)`

Existing LawnMower primitive found despite absent CoverageTask server (initial inventory corrected). New measured sequential survey completed all five legs/images in survey-03 after latest-state QoS change; prior survey-01/02 failures retained. Forward-camera area coverage and collision/contact evidence unknown.

Next procedure / physical concern: Measure actual ground camera footprint/occlusion coverage, not just route completion; retain two failed survey attempts. Existing fixed LawnMower primitive uses overloaded width/height parameters and is not yet tested here.

## 027 — Active perception
Primary verdict: **PASS**
Source: `camera topics; navigate action`

One restricted static red-fixture viewpoint selected from3 fresh exact RGB/depth/calibration observations and acquisition-stamped optical-to-map TF. Policy planned0.635471m; PhysX capture displacement0.554962m. Raw detector recomputation confirms median bbox-area ratio1.407407 and surface optical depth3.465420→2.904359m with unchanged calibration. Navigation/LAND/disarm and independent envelope/contact checks pass. Oracle depth and resolution proxy only; no semantic information gain or general active-perception qualification.

Next procedure / physical concern: Test controlled ambiguity/occlusion and depth failure with predeclared abstention; qualify broader framing/uncertainty before semantic target-directed flight.

## 028 — Payload delivery
Primary verdict: **NOT IMPLEMENTED**
Source: `Iris simulated vehicle; task action inventory`

No delivery actuator/task found.

Next procedure / physical concern: Need benign payload attachment/release model, mass change and landing/drop acceptance.

## 029 — Aerial manipulation
Primary verdict: **NOT IMPLEMENTED**
Source: `Iris simulated vehicle; task action inventory`

No usable aerial manipulator workflow found.

Next procedure / physical concern: Requires arm dynamics, contact, coupled center of mass and force limits; out of immediate baseline.

## 030 — Agile high-speed flight
Primary verdict: **NOT TESTED**
Source: `fixed_trajectory_task.cpp; FixedTrajectoryTask.action`

Trajectory workflow exists, high-speed envelope unqualified.

Next procedure / physical concern: Qualify low-speed tracking first; motor instantaneous model limits transfer claims; require acceleration/yaw/clearance limits.

## 031 — Heterogeneous multi-domain teaming
Primary verdict: **NOT TESTED**
Source: `common/ros_packages/coordination`

Peer profiles/gossip exist, no actual mixed embodiment trial.

Next procedure / physical concern: Validate frame transforms, role authority and timing before coupled tasks.

## 032 — Human-machine teaming
Primary verdict: **PASS**
Source: `Foxglove bridge; drone_safety_monitor command`

Separated-stop-01: one bounded real-monitor operator PAUSE/cancel/LAND episode passes independent PhysX and full separation-aware contact acceptance. Speed at pause0.30999m/s, max extra displacement0.151776m, stable stop1.83266wallseconds including1s dwell. Navigate canceled, landed/disarmed, no pending goals. Earlier failures retained.

Next procedure / physical concern: Broader supervisory interaction and repeated scenarios remain unqualified; scripted command path does not qualify UI usability.

## 033 — Learning-based autonomy
Primary verdict: **NOT TESTED**
Source: `rrm/cosmos_reason2.py; osmo/cosmos-worker`

Cosmos explicitly deferred. Deterministic task execution is not learned autonomy.

Next procedure / physical concern: Inventory only until model scope changes; preserve reusable physical task/evidence records.

## 034 — Prognostics and health management
Primary verdict: **NOT IMPLEMENTED**
Source: `diagnostics; mavros sys_status`

Health telemetry exists; no prognostic degradation model qualified.

Next procedure / physical concern: Separate current-fault diagnosis from remaining-life prediction; need calibrated degradation evidence.

## 035 — Cyber resilience
Primary verdict: **NOT TESTED**
Source: `osmo/cosmos-worker/dev_control.py; network config`

Worker private token controls exist, deferred; not a UAV cyber-resilience demonstration.

Next procedure / physical concern: Read-only configuration review and isolated auth tests; no network attack against active flight controls.

## 036 — Runtime assurance
Primary verdict: **PASS**
Source: `drone_safety_monitor.cpp; px4_actuation_guard.hpp`

One bounded dedicated-monitor source-stale episode passed independent acceptance: private odometry relay stamped10s old during actual motion; installed monitor detected fault, PAUSE transitioned unacknowledged→acknowledged, no manual PAUSE. Physical speed at injection0.31397m/s, max extra displacement0.140849m, stable stop3.04970wallseconds including1s dwell. Public navigation canceled, supervisor LAND/disarm, complete separation-aware contacts. Not exclusive mode arbitration, general runtime assurance or automatic landing.

Next procedure / physical concern: Define fault-specific persistence and escalation with exclusive controller authority; qualify service rejection and actual sensor/link loss physically only after isolated contracts.

## 037 — Online sensor calibration
Primary verdict: **NOT IMPLEMENTED**
Source: `camera_info; ZED calibration; MAVROS calibration plugins`

Camera calibration messages observed, not online calibration.

Next procedure / physical concern: Need identified parameters, excitation/observability and error convergence against independent truth.
