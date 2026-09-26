# RRM command console

> Scope note (2026-09-26): this console is the deterministic aerial command adapter,
> not the universal RRM language layer. Its direct execution path does not yet consume
> the embodiment-neutral `GoalRequest`, and its inter-action decisions are bounded
> reconciliation rather than general semantic replanning. See
> [goal-to-finish performance status](end-to-end-status.md).

The localhost console is the simulator command surface. It can switch to any Isaac
entry in AirStack's checked-in scene catalog, optionally capture the front camera,
save immutable movement commands, translate them to public AirStack task actions, run
one serial mission, show its state, and request STOP/HOLD.

Start it in the running AirStack workspace:

```bash
cd /root/AirStack/RRM-transfer/rrm-9d26a58
bash scripts/rrm_command_console.sh
```

Forward port `8787` through the Remote-SSH/IDE Ports panel and open its localhost URL.
The server binds only to `127.0.0.1`.

## Current command path

1. Enter a movement command and select **Save goal**. A camera capture is optional.
2. Select **Plan and run**. That deliberate click starts the autonomous mission; the
   generated plan is displayed as evidence and is not a second approval gate.
3. The server reads current `map -> base_link` odometry, MAVROS state, airborne state,
   VDB map availability, and the ROS action graph from robot domain 1.
   For Isaac, it also verifies that the robot container started in the current
   simulation-clock epoch; a robot graph retained across an Isaac restart is rejected.
4. It distinguishes actual action servers from client-only action names and compiles
   only against executors present in the active robot configuration.
5. It writes `command-plan.json` before launch, including every parameter's origin and
   every declared assumption, and serially invokes the existing public
   task actions. It never publishes a trajectory or sends PX4/MAVROS commands.
   The GUI shows the exact compiled actions plus new `/robot_1/global_plan` publications
   and task feedback in bounded, scrollable evidence panels while execution continues.
6. Each task result is checked against fresh causal odometry/state. Before every later
   action, the runner reacquires canonical state and records an immutable
   `CONTINUE`, `SKIP_SATISFIED`, or `HALT` replan decision. Failure,
   cancellation, timeout, missing evidence, or an unavailable server halts the sequence.
   A plan containing takeoff also contains a distinct recovery `LandTask`. If takeoff
   returns terminal success but physical verification mismatches while fresh evidence
   shows the vehicle connected, armed, and airborne, the requested sequence halts and
   the predeclared landing runs automatically. Its grounded/disarmed result is verified
   independently and reported as `RECOVERED_HALT` or `RECOVERY_FAILED`. A later
   inter-action halt uses the same predeclared landing only when its fresh observation
   proves meaningful armed flight; an unavailable observation never triggers a blind
   recovery command.

The supported text contract is intentionally deterministic:

- `take off`, `launch`, `ascend`;
- `land`, `touch down`;
- `explore`, `survey`, `roam`, `map the ...`, or `move around`, optionally
  `for N seconds/minutes`, common quantities such as `for a couple of seconds`, or
  conservative modifiers such as `briefly`;
- map points: `fly to x=2 y=-1 z=1.5`, `go to (2,-1,1.5)`, or a multi-waypoint
  route such as `fly through (1,2,1.5), (4,-2,2)`;
- current-heading-relative motion: `move forward 2 meters`, `move backward 1m`,
  `move left 1m`, `move right 1m`, `move up 1m`, or `move down 1m`;
- return intent: `come back`, `go back`, `return home`, or `return to the start`.

When a grounded vehicle receives navigation or exploration, RRM inserts a typed
takeoff first. Commands already satisfied by observed state and commands outside this
contract fail without dispatch. A return command also requires a real, currently
served `NavigateTask`; an action name created only by a client is not enough. It uses
the fresh start pose captured at
compile time and inserts a `NavigateTask` before landing; it is not treated as “land at
the exploration endpoint.” Deictic destinations such as “fly there” produce one
targeted clarification rather than guessed motion.

The supported simulator stack uses map-zero as ground. If its airborne topic remains
true at or below 0.3 m map altitude after an abort, RRM classifies the state as
contradictory. It will not skip takeoff and start an airborne task from that flag; only
an explicit landing/reconciliation command is admitted until state agrees again.

Every generated number is tagged as operator-explicit, semantic interpretation,
environment observation, vehicle envelope, or policy default. Numeric values outside
the supported envelope request clarification. Semantic quantities may be conservatively
bounded with the requested and applied values both retained. Thus “a couple of
seconds” means 2 seconds but is visibly raised to the 5-second minimum supported by
the current task envelope; it never silently falls back to 60 seconds.

The public takeoff server anchors every vertical takeoff trajectory at the latest
physical odometry rather than a retained controller tracking point. Production config
also bounds horizontal takeoff displacement to 0.3 m; exceeding it terminates the
takeoff action early and holds the current pose. RRM halts. It runs the predeclared
recovery landing only when fresh evidence shows an armed vehicle above the recovery
altitude; it never retries takeoff or assumes that cancellation means stable flight.

## Where planning and replanning happen

RRM is the command-to-task adapter, not a second flight controller.

- `ExplorationTask` activates AirStack's configured global planner. In `full_default`,
  random-walk planning checks the VDB occupancy map, generates collision-checked global
  paths, delegates each path to `NavigateTask`, and continues planning as the vehicle
  progresses.
- `NavigateTask` sends the supplied map-frame path through the active DROAN local
  planner and trajectory controller, including its reactive obstacle handling.
- `TakeoffTask` and `LandTask` use the existing takeoff/landing planner.

The console requires a fresh map-frame VDB point cloud before exploration. It records
the finite observed VDB extent and creates a conservative search polygon around the
start pose, tightening the policy radius only when the fresh extent encloses a credible
margin around the robot. The extent is evidence about observed occupied points, not a
claim that all enclosed space is collision-free; collision authority remains with the
AirStack planners. Coordinate
navigation supplies the requested waypoint path to `NavigateTask`; it does not claim
that RRM performed a separate global route search.

The AirStack GCS action relay is a JSON-to-typed-goal transport boundary. It is not a
natural-language planner. The console uses the same public goal schemas directly in
the robot domain so task feedback, cancellation, and results remain end-to-end.

## Scene and configuration behavior

Movement command saving does not depend on the Office manifest or entity catalog.
After any catalog scene switch—or after a manual Isaac stage edit—the next execution
uses newly discovered action servers and current robot/map state. Scene switching is
blocked while a mission is active.

"Any configuration" means capability-adaptive, not capability-inventing: a command is
accepted only if that configuration actually serves every required task action. The
selected stack may list `NavigateTask` or `SemanticSearchTask` because a client exists
even when no corresponding server is running. Discovery uses the ROS action-server
graph rather than the combined client/server action list, so return navigation and
commands such as "find the red chair" are rejected instead of being misrepresented as
executable. They become routable only when the required executor is actually served.

## STOP/HOLD and evidence

**Stop / hold** sends SIGINT to the mission adapter, which requests cancellation on the
active public action. Cancellation acknowledgement is not called a physical stop. When
available, three new odometry samples at or below `0.10 m/s` are required to record
`MOTION_STOPPED`; otherwise the result remains `UNCONFIRMED`. No later action in the
mission starts after a stop or failed verification.

Artifacts live under `.rrm-artifacts/command-requests/<request-id>/`:

- `command-plan.json` — discovered state/capabilities and exact typed action sequence;
- `command-mission.log` — preflight state, task feedback, pose/speed/displacement,
  replan, and result records;
- `command-mission-evidence/*-outcome.json` — per-action observed result;
- `command-mission-evidence/replan-*.json` — fresh inter-action state and decision;
- `command-mission-evidence/mission-outcome.json` — terminal mission record.

Takeoff outcomes contain numeric displacement/altitude metrics and symptom labels such
as `LATERAL_INSTABILITY_OBSERVED` and `ALTITUDE_TARGET_MISMATCH`. These labels separate
an observed execution symptom from a language-planning error; they do not assert
whether the root cause is PX4 tuning, simulation physics, estimation, or the task
planner. Aggregate retained attempts without flight authority with:

```bash
python3 scripts/rrm_takeoff_history.py
```

SQLite goal/run history remains at
`.rrm-artifacts/command-requests/tasks.sqlite3`. Older PSC/Cosmos and exact-proposal
supervisor endpoints remain for evidence compatibility, but their retired manual
controls are not rendered by the current page.

## Validation

As of 2026-09-23, the full RRM CPU suite passes 193 tests. Grounding, environment
extent, diagnostic, history aggregation, and inter-action decisions are covered by
hermetic tests. The modified
`takeoff_landing_planner` C++ package builds successfully, and live `airstack ready`
passes the new clock-epoch gate. Live read-only discovery in the
running `full_default` stack found fresh canonical state, a fresh `map` VDB point cloud,
and real takeoff/land/navigate/fixed-trajectory/exploration servers while correctly
excluding client-only semantic search. No action goal was sent during this validation.
