# Bounded UAV task execution

2026-11-15, `ore_proj`, base `07ff1dcbb` plus working changes. Simulation only, one GPU; Cosmos deferred. See [current status](uav-current-status.md), [capability dispositions](uav-task-campaign.md), [physics evidence](uav-physics-evidence.md) and [discrepancies](uav-discrepancies.md).

Nineteen public-flight attempts are retained: thirteen completed procedures, three incomplete airborne trials with recovery, and three blocks before flight. Cases and source versions differ; this is not a pooled success-rate estimate. Four episodes pass their declared physical/contact cases: bounded takeoff/hover/land, moving operator PAUSE/cancel/land, dedicated-monitor stale-source automatic PAUSE, and one observation-conditioned marker viewpoint. Earlier flights lacking separation remain unqualified.

## Tools

| Tool | Purpose and scope |
| --- | --- |
| `scripts/uav_vertical_probe.py` | Public takeoff, hover, optional inspection/survey/circle or supervised stop, landing and fresh disarm verification. Observation only unless `--execute-simulation` is supplied. |
| `rrm/uav_acceptance.py` | Independent offline envelope evaluator. Violations override action success; missing truth/contact/disarm evidence blocks PASS. |
| `scripts/assess_uav_capture.py` | Phase metrics from receipt-paired independent PhysX captures. |
| `scripts/assess_uav_contacts.py` | Whole-episode envelope, ground support and separation-aware contact assessment with recorder-health coverage. |
| `scripts/assess_uav_stop.py` | Real monitor receipt, physical braking distance/dwell, cancellation and contact assessment. |
| `scripts/assess_uav_localization.py` | Preflight-only yaw/translation registration and held-out relative pose diagnostics; cannot qualify SLAM. |
| `scripts/uav_contact_status.py` | Preserve recorder-health history from before scene launch. |
| `scripts/uav_marker_capture.py`, `rrm/uav_marker.py` | Passive restricted red-fixture observations with exact RGB/depth/calibration pairing. |
| `scripts/uav_timing_probe.py` | Read-only reference/odometry source and receipt timing. |
| `scripts/uav_watchdog_probe.py` | Actual monitor against a mock controller, isolated DDS88. |
| `scripts/test_uav_guard_contract.py` | Actual probe against mock tasks on DDS89: normal cancellation, unresponsive cancellation/public LAND, delayed acceptance before mock arming. |
| `scripts/uav_monitor_fault_probe.py` | Actual monitor fault limits on DDS90: fresh/stale-source/silent receipts and rejecting PAUSE service. |

## Execution procedure

Run ROS inside the robot container, source its overlay and explicitly set `ROS_DOMAIN_ID=1`. Plain Docker shells here do not automatically export the robot domain. Verify and preserve source hashes before copying scripts into the container. Each output directory must be new; tools refuse overwrite.

```bash
docker exec airstack-robot-desktop-1 bash -c '
  source /root/AirStack/robot/ros_ws/install/setup.bash
  export ROS_DOMAIN_ID=1
  python3 /tmp/uav_vertical_probe.py --output /bags/NEW-UNIQUE-RUN \
    --execute-simulation --inspect-offset-m 0.75
'
```

Options include `--survey-size-m 0.75`, `--fixed-circle-radius-m 0.5`, or `--inspect-offset-m 1.0 --supervised-stop`. Use one route mode per episode. The exclusive `--inspect-offset-m 1.0 --automatic-state-fault-stop --execute-simulation` case starts a private state relay and a dedicated installed watchdog before flight. It alters only relay header stamps, preserving genuine main-estimator/controller state, and requires healthy/acknowledged qualification preflight. The temporary monitor stops only after final ground/disarm. No manual PAUSE is sent at fault injection. The survey uses five separately settled navigation legs; a closed NavigateTask can otherwise finish immediately when its terminal point is already near the start. A canceled route is not a completed navigation goal.

Preflight requires fresh connected/disarmed/ON_GROUND state, healthy watchdog, verified actuation guard, finite map odometry and starting |map z| ≤ 0.25 m. The target is absolute map z = 2 m. Limits: horizontal radius 2 m, relative height -0.3 to 3 m, speed 2 m/s, hover tolerance 0.3 m, freshness 2 wall seconds. Low-rate state/safety flags use reliable DDS; odometry/images use sensor QoS. No arming or admission checks are bypassed.

The probe retains explicit goal IDs and pending acceptance futures. On a guard failure it reconciles acceptance, cancels accepted goals even before armed telemetry appears, and waits for terminal results before normal LAND. If cancellation cannot be confirmed, it uses the existing public RobotCommand LAND and continues terminal reconciliation and ground/disarm observation. It never forces airborne disarm. Unknown pending goals remain an explicit failure.

Start contact health recording before the simulator scene. Use a unique `UAV_CONTACT_DIR`, fixture manifest, and `ISAAC_SIM_TRUTH_DIR`; the truth directory needs `capture.json` with a unique capture ID and `enabled:true`. Verify subscription, all seven bodies, matching source hash, complete metadata, fresh history, recording truth and no errors/caps before dispatch. A final healthy status alone cannot establish continuous coverage.

After fresh final ground/disarm and terminal actions, preserve immutable captures. When the viewer must stay live, copy exact complete-record JSONL prefixes into a new snapshot, verify each source-prefix digest, retain byte/line bounds and assess those copies. Disable further truth capture after snapshot; bounded contact acquisition may continue to its cap. If stopping a scene for a new case, use its verified PID only after ground/disarm. Recheck readiness, clock alignment, PX4 and Kit signaling after every restart. A browser render is not proven by a TCP listener. Assess frozen files and retain input hashes. An assessment made while a stream was growing is provisional; keep it and create a separately named frozen result. Never overwrite earlier evidence.

The actuation startup guard can latch before simulator readiness. Restart only after independent ground/disarm confirmation. Failed arming containment can leave PX4 in disarmed LAND; inspect health and mode before any ordinary grounded AUTO.LOITER restoration. Do not relax backend lock/readback deadlines to make admission pass.

## Outcomes

| Attempts | Outcome |
| --- | --- |
| Observe-01 | INVALID logger: NaN battery fields rejected; explicit unknown markers added. No flight. |
| Vertical-01 / 02 | BLOCKED before flight: startup latch, then non-armable LAND mode. |
| Vertical-03 / vertical-truth-01 | Procedures completed; latter has independent motion, but no complete contact evidence. |
| Inspection-01 | Settled 0.75 m viewpoint and image completed; raw PhysX goal error about 0.094 m. |
| Survey-01 / 02 | Incomplete airborne trials: stale state receipt, then stale planner command source. Recovery landing/disarm observed. |
| Survey-03 | Five legs/images completed after latest-only reference subscriptions; raw endpoint errors 0.034–0.067 m. Area coverage unknown. |
| Circle-01 | Bounded circle completed; raw reference tracking RMSE 0.172 m. |
| Marker-inspection-01 | Predefined viewpoint completed; exact saved-image detector match at surface optical depth 2.72554 m. No general object/world binding. |
| Contact-vertical-01 | Procedure completed; health history started 12.64 seconds late, so incomplete qualification. |
| Contact-vertical-02 | Procedure completed; permissive PASS superseded by strict camera-contact policy failure. Old zero-impulse data lack separation, so impact remains unknown. |
| Supervised-stop-01 | Incomplete flight from stale health publication; recovery landing/disarm observed. |
| Supervised-stop-02 | Physical PAUSE/braking/cancel/land completed after health-cadence change; old contact data remain unqualified. |
| Separation-vertical-01 | BLOCKED before arm; pending acceptance race exposed and corrected without relaxing backend gates. |
| Automatic-fault-stop-01 | PASS for one dedicated-monitor source-stale automatic PAUSE, no manual command. Physical speed at injection0.314m/s, extra displacement0.141m, stable stop3.050wallseconds including dwell; detector/ack, supervisor cancellation/LAND, final disarm and full contacts passed. |
| Separation-vertical-02 | PASS for bounded vertical case with full independent separation/contact and final-state evidence. |
| Separated-stop-01 | PASS for bounded operator intervention with full contact evidence; PAUSE at 0.310 m/s, extra displacement 0.152 m, stable stop in 1.833 wall seconds including dwell. |

Combined evaluator/perception/contact/stop tests: 82 pass. Three actual-client handoff contracts pass after the driver extension. The watchdog package has all seven CTest checks passing (aggregate zero failures, two skips). Planner functional/GPU checks pass all 18, while the broad colcon lint/style run retains 98 failures and 13 skips; it is not a clean package suite.

No F6 root cause was established. Planner safe-prefix/transform warnings are retained rather than automatically labeled F6. No GUI video was recorded; onboard images and physical traces are separate evidence. Source and raw records remain local, with no external backup, commit or push. The host AirStack CLI dry-run remains broken (`_env_value` missing); invalid effective launches were not executed.

For automatic-stop evaluation supply `--automatic-state-fault` to `assess_uav_stop.py`. Evidence must include the ten-second input-stamp shift, actual detector log, a new false→true acknowledgement, fault flag, no manual PAUSE event, terminal navigation cancellation and qualified physical/contact evidence. Timing origin is injection, not the later controller acknowledgement. The final separately named assessment corrects inherited operator-PAUSE metric labels without changing the measured trace or verdict. Historical old-monitor probes use older input assumptions; the current registered package CTest is authoritative for the new watchdog contract.

The exclusive `--marker-viewpoint --execute-simulation` route uses live exact-pair observations and acquisition-stamped TF to choose one bounded horizontal viewpoint. Raw before/after frames and calibration are saved; the resolution proxy is assessed separately by `assess_uav_viewpoint.py`. One case passed raw/policy/goal binding, physical displacement and full-flight contact evidence with bbox area×1.407. This does not establish semantic inspection or general active perception.
