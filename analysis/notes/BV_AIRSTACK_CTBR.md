# BV AirStack CTBR integration — 2026-10-20

## Location and boundary

Deployment development is on yutong-xiaoxin, `/home/yutongw/Desktop/AirStack_worktrees/bv-ctbr`, branch `codex/bv-airstack-ctbr`, based on `yikuan/SVG_ground_control_dev` at b21d28c75d45b6b0d858d7f9485690435fdeb24f. The original `/home/yutongw/Desktop/AirStack` release checkout was preserved. S2 remains policy/training/export only. No hardware connection, arming, flight or publication was performed.

Runtime `/home/yutongw/Desktop/AirStack_runtime/bv-ctbr`; container `bv-ctbr-dev`, installed AirStack v0.18.0 robot-x86-64_dev image, network none, not privileged, no devices, ROS domain87. Separate `/runtime/build` and `/runtime/install`; host runtime stores policy, private Python environment, receipts and bags. Host CPU worker uses JAX0.7.2/Flax0.12.0; ROS is Jazzy/Python3.12 in Docker.

Ported source from loco_mjx db888431fb69a9edfd9569fd371cfd370b0a59f3. Candidate exported actor:301989888 transitions, SHA256 61486e97f5ed79af90933bb480c9fff7e1c23bfb360e60baeddaefa9db876f26; matching inputs_301989888.json copied together. Never replace this bundle with a raw training checkpoint.

## Implemented

Sibling gamma_zero_controller and gamma_zero_msgs packages; local Unix-socket inference, 50Hz policy loop, explicit engagement and acknowledged handback, stamped references, final-command fence caps, fault latching and logging. Managed PX4Interface accepts a single atomic SelectedCommand topic and ignores legacy callbacks; its command-age watchdog stops heartbeat on timeout. SVG lifecycle hooks are opt-in; takeoff/landing remain conventional. Landing waits for the generation corresponding to its own handback request. Foxglove includes policy mode/ownership/reference/latency/fault, and existing SVG MCAP recorder captures the new topics.

Default references are the original loco_mjx `flown` and `large` figure-eights, not SVG's default scenarios. Reference code preserves both abrupt benchmark timing and separately evaluated 3s ramp timing. Default is smoothed,10s settle,2laps,captured origin/yaw. Explicit reference_mode=svg enables the separate SVG goals/teleop path. Live launch refuses unverified transforms/thrust/firmware; live example leaves physical bounds/lever arm null.

## Validation receipts

Baseline:8 packages built;177 unchanged SVG tests passed. New controller suite:36 tests, including404 original-S2 reference samples across both patterns/ramp variants. Final SVG suite:179 passed, including2 new tests rejecting stale/old-generation landing acknowledgements. Both tested_trajectory and explicit svg reference modes passed the integrated lifecycle fixture. Foxglove smoke suite passed.

Compiled interface fixture: legacy command rejection, trajectory forwarding, CTBR FLU/FRD axes/thrust sign, stale heartbeat shutdown, replayed timestamp rejection, duplicate selected-publisher rejection. See interface-result.json.

Actual-policy ROS fixture: engagement, tested-reference start, policy hold, acknowledged conventional handback and no automatic recovery. Separate fault receipts cover stale state, estimator reset, stalled worker, stale SVG reference and lost interface acknowledgement: backend-result.json, reset-result.json, worker-failure-result.json, reference-failure-result.json, ack-failure-result.json. Worker-stall injection paused only this task's verified worker and resumed it in cleanup.

Full SVG lifecycle fixture uses synthetic FCU feedback: takeoff→engage→start→hold→handback→land→idle, with actual SVG/backend/PX4Interface/recorder processes. This tests software routing/state transitions, not PX4 firmware dynamics. MCAP metadata verifies nonzero reference, selected command, acknowledgement, trace, odometry, rates setpoint and trajectory setpoint streams. See svg-lifecycle-result.json and bag-audit.json. Isolated shadow namespace only.

Native MuJoCo3.7.0 on Xiaoxin used the saved evaluated BV model, actual ported policy/controller, a30ms thrust filter, two full laps plus2s final hold. Four runs completed:

| Pattern | Extra delay | Position RMS m | Maximum m |
|---|---:|---:|---:|
| flown |0ms|0.0658103|0.1069902|
| flown |100ms|0.0658517|0.1028333|
| large |0ms|0.0898987|0.1677641|
| large |100ms|0.0896346|0.1649898|

These are new native rollouts, not identical saved S2 traces or firmware-in-loop tests. See mujoco-results.json. Numerical reference parity tolerance1e-12; same exported policy across machines is not bit-identical.64-step/3reset comparison max physical deviations: thrust8.95e-6N, roll2.185e-5rad/s, pitch3.241e-6rad/s,yaw2.168e-6rad/s; all below1e-4 absolute cross-CPU check. Original1e-5 check failed; do not report exact inference parity. Xiaoxin socket latency over64requests:median4.90ms,p955.33ms,max6.33ms, below20ms period in that sample. See policy-parity.json.

## Resolved issues and acceptance limits

- Fresh container needed dependency closure and sourced overlay; unsourced early test collection failures are superseded by passing baseline/regression suites.
- Ownership checks must use resolved topic names after remapping.
- Fixture teardown must terminate child process groups; orphaned ros2-run children otherwise retain the worker socket.
- Custom message package requires explicit ament_cmake build-type export for rosbag discovery. First bag omitted selected_command; corrected bag includes it.
- Normal IDLE after landing does not require streaming acknowledgements.
- Plugin instance/global subscriptions must be destroyed before the class loader; fixed the discovered interface shutdown crash and reran lifecycle/shutdown.

Hardware remains unqualified: measured estimator/IMU transform, verified thrust map, firmware CTBR/Offboard-loss behavior and radio handback, attended props-off checks and bounded flights are outstanding. Fence/braking effectiveness on real hardware is not proven. No direct motor-control path was added. No default SVG trajectory should be substituted for the evaluated reference without a new matching simulation evaluation.

Final lifecycle and graceful shutdown passed without faults or plugin teardown errors. flight-analysis.json parsed the new selected-command log, reported zero faults and zero tracking-phase samples for the short lifecycle fixture (correct: it only sampled settling/hold). Full trajectory scores come from mujoco-results.json, not this fixture.
