# M00-C corrective-action plan — proposals only

No corrective source/configuration change or live recovery was performed. The [candidate matrix](../../../evidence/M00-C/M00C_CORRECTIVE_ACTIONS.csv) records rationale, affected requirements/blockers, safety, exact change scope, configuration impact, verification method, regression risks and approval authority for CA01–CA05.

| Candidate | Recommendation and verification |
|---|---|
| CA01 dependency-aware startup | Preferred design direction for review: wait for actual process/transport/epoch/safe-state prerequisites before constructing the guarded interface, with a separately justified bounded orchestration wait. Preserve the existing admission guard. Verify missing/stale/unsafe dependencies, delayed PX4, service failures and clock resets in an isolated approved startup case. “App ready” or container start alone is insufficient |
| CA02 one-time operational recovery | Possible future supervised robot-only recovery after independently confirmed safe state and no pending work. Requires explicit PD authorization; may restore a session but does not establish reproducibility, source equivalence or F00 qualification. Not performed in M00-C |
| CA03 startup instrumentation | Record steady guard construction, callback/service/timer edges, sim Play/PX4 spawn/connection milestones and resource observations. Prepare any future candidate under RRM; protected-source integration needs a concrete reviewed diff and approval. Verify bounded overhead and no added command/control path |
| CA04 timing requirement review | Obtain authoritative startup requirement/budget before any proposed timeout/reference change. A longer timeout is not justified by this single failure. Review fault detection, stale-state exposure and worst-case behaviour; material baseline changes require PD approval |
| CA05 controlled records/assets | Obtain FR-01–FR-20, full approval/integrity chains and exact reproduction assets. Review each source/image/binary delta; later independently verify REC-01 and BOUNDARY closure under approved successor cards |

## Proposed active discrimination case — NOT AUTHORIZED / NOT EXECUTED

Active startup testing is necessary to demonstrate that a changed launch order reliably fixes the mechanism and to quantify DDS/transport/scene/resource contributions. The current directive prohibits restarts and integration changes. This document therefore supplies a concrete procedure for later PD authorization; completion of M00-C does not authorize it.

1. Reviewer approves scope, exact isolated environment/images/source, safety/recording procedures, requirement trace and operator authority. Authenticate missing baseline inputs if a F00 reproduction claim is intended. Do not reuse an unknown/airborne/pending-goal runtime.
2. In a separately isolated, non-actuating mock/bench simulator startup harness, record explicit guard steady origin and callback/transaction/deadline events plus simulator/PX4 spawn and heartbeat times. Use declared clock maps; preserve missing observations and all attempts.
3. Compare concurrent startup with dependency-gated startup at unchanged guard timeout/profile/freshness thresholds. Freeze the scenario list before execution; collect delayed/missing PX4, unsafe/unknown state, missing service, reset epoch and unresponsive dependencies. No vehicle launch/arm/motion goal is part of this proposal; any test that requires simulation/service startup or synthetic ROS writes needs explicit approval first.
4. Confirm no control commands bypass the gate, no automatic repair after terminal failure, no active-mission restart, correct one-attempt semantics and unchanged LAND/DISARM availability in isolated mocks. A received READY flag alone is insufficient; reconstruct the full transaction and evidence coverage. Quantitative timing bounds come from authoritative approved requirements, not this report.
5. Retain every failure, bind exact binaries/scene/seed/test/configuration/attempt, and submit independent evidence for ChatGPT technical review and PD disposition before any integration or WP01 step.

No protected-file patch is proposed as ready to deploy. The required upstream/requirements review may change the correct remedy. CA01 has regression risks around stale readiness tokens, clock resets, circular dependencies and blocking operational recovery; those must be explicitly exercised. CA02 restarts may invalidate retained control state and are unsafe without confirmed quiescence. CA03 instrumentation overhead and sync storage can change timing; tests must measure and bound it against approved budgets. CA04 cannot relax timing to obtain a pass. CA05 cannot substitute a version tag or rebuilt binary for the qualified executable.

F00-01 and F00-02 require their own authorized corrective work and original evidence; startup recovery does not close recorder supervision or native boundary trigger/recovery. All candidates remain proposed, with verification pending.
