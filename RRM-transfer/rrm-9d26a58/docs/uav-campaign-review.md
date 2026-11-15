# UAV campaign review and continuation

2026-11-15. This is a bounded simulation checkpoint, not completion of every executable scenario. Source and compact results are saved in the UAV checkpoint commit. One GPU; Cosmos deferred.

## Audit coverage

Capabilities 006–037 each have a documented primary disposition in the [execution matrix](uav-task-campaign.md): 4 PASS, 11 NOT DEMONSTRATED, 8 NOT IMPLEMENTED, 9 NOT TESTED. All passing claims are scoped to the recorded procedure. Nineteen public attempts include thirteen completed procedures, three incomplete airborne trials with recovery and three blocks before flight. Do not combine cases/versions as a success-rate estimate.

The supplied directive reports historical 001 mission planning PASS, 002 replanning PASS for one valid run with repeatability pending, 003 navigation PASS, 004 avoidance NOT DEMONSTRATED, and 005 exploration FAIL from constraint violation. Original historical evidence was not available for this session, so those are preserved reported outcomes, not independently reverified results. No new credit is assigned to avoidance or exploration.

## Maturity

Existing interfaces and installed nodes establish software availability. Completed routes and observations establish bounded executed functionality. Independent pose/contact/final-state checks establish the four bounded verified cases. They do not establish repeatable, generally qualified autonomous behavior. Operator PAUSE is separate from automatic safety intervention; a canceled route is separate from completed navigation.

## Missing implementations

- 008 Target tracking: Restricted static-fixture detection now exists. No moving-object association/pursuit or loss/occlusion tracking workflow has been qualified; repeated detections alone are not target tracking.
- 019 Airborne detect-and-avoid: Plugin presence alone does not implement avoidance; no suitable encounter workflow found.
- 024 Semantic scene understanding: Semantic search action interface/client does not establish server or scene understanding.
- 025 Behavior and intent prediction: No benign-object intent predictor found in current UAV stack.
- 028 Payload delivery: No delivery actuator/task found.
- 029 Aerial manipulation: No usable aerial manipulator workflow found.
- 034 Prognostics and health management: Health telemetry exists; no prognostic degradation model qualified.
- 037 Online sensor calibration: Camera calibration messages observed, not online calibration.

## Prioritized continuation

1. Preserve the source/clock validity and acknowledgement contract now tested in isolation and one physical source-stale case. Extend fault persistence, parameter boundaries and recovery scenarios without weakening thresholds.
2. Bounded acknowledged PAUSE retries now pass isolated rejection/unavailability/late-reply tests. Define escalation and exclusive controller-mode authority before physical service-failure scenarios; preserve action reconciliation and prohibit forced airborne disarm.
3. Resolve camera descendant collider intent as a configuration decision. Preserve the currently measured mass/collision geometry and contact semantics; any change requires new physics qualification.
4. Establish an obstacle scene requiring actual route modification, with independent clearance. Empty-space completion and stationary zero collisions do not demonstrate avoidance.
5. Qualify camera/IMU/body frame registration and observation uncertainty before target-derived flight. Restricted color and ideal depth are development fixtures.
6. Calculate camera footprints, occlusion and ground-area coverage before claiming coverage planning; five viewpoints do not establish a coverage percentage.
7. Test repeatability under declared initial conditions and fixed sources. Record all outcomes; do not repeat unchanged failures until a pass appears.
8. Add battery-response, link-specific loss, GNSS-degradation and bounded disturbance contracts one at a time. A sensor topic or PX4 failsafe flag alone does not establish mission response.

Multi-vehicle and high-speed work remain later priorities because namespace/control authority, separation monitoring and the relevant physical envelopes are unqualified. No model or second simulator is required for the next isolated safety-contract work.

## Evidence and limitations

Physics parameters, mass, COM, frame/clock assumptions, contact policy and measured braking are consolidated in [physics evidence](uav-physics-evidence.md). [Discrepancies](uav-discrepancies.md) preserve failed and superseded results. F6 remains unconfirmed: planner safe-prefix/transform warnings were retained, with no proven defect classification or root cause. The old F6 analysis was not a prerequisite for the current bounded work.

The local evidence package includes raw JSONL telemetry, independent PhysX/contact captures, source hashes, health history, images, component logs, synthetic checks and a working-source patch/snapshot. Final assessment inputs were frozen and verified either after stream shutdown or as exact complete-record snapshots while the viewer stayed live. Earlier assessments made while streams were growing remain separately retained. There is no verified external backup or GUI video, and broad package lint/style failures remain open.

Isaac/PX4 remain running for the viewer, disarmed/on ground. Readiness and signaling were checked after the last restart and again after the bounded fault flight. Browser rendering is not directly verified. No Gazebo sequence was started. Further flight should follow the recorded source/readiness/contact procedure and a newly declared case.
