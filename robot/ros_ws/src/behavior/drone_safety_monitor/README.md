# Drone safety monitor

Watches source-stamped odometry and requests trajectory PAUSE on invalid/stale state. Operator commands are `pause`, `resume` and `rewind`. A positive controller reply confirms request acceptance; independent motion evidence is required to establish a physical stop.

## Health contract

Odometry stamps must use the node's ROS clock. Source age and age since accepted receipt must each stay within `state_estimate_timeout`. Stamps must be positive, no more than 0.1 ROS seconds in the future, and not move backwards within an epoch. Position, orientation, linear velocity and angular velocity must be finite; quaternion squared norm must lie in [0.98, 1.02]. Covariance and localization accuracy are not qualified by these checks.

Repeated equal stamps do not refresh the accepted-receipt deadline. A backwards ROS clock jump invalidates previous state and requires valid data in the new epoch. Before the ROS clock is initialized the monitor remains unhealthy. A separate steady-clock receipt deadline detects silence/replay even when simulation time stops.

Health publication and RPC deadlines use wall/steady time. A recovered state never automatically resumes motion. The existing `state_estimate_timed_out` flag remains true while either state is unhealthy **or a requested PAUSE lacks a positive acknowledgement**, so existing task consumers do not confuse fresh state with intervention readiness.

## Intervention contract

Startup requests PAUSE. A new state fault or operator PAUSE latches a pending intervention. One PAUSE RPC and one previously sent operator RPC may coexist: protective PAUSE does not wait for a held resume/rewind reply. Rejection, unavailable service or PAUSE response timeout leaves the intervention pending and triggers retries at the configured interval. Expired PAUSE client requests are removed; late replies cannot acknowledge a newer pending request. This does not cancel a request already executing remotely.

A superseded operator request remains tracked, including after its response deadline.
Readiness stays faulted (`operator_request_unresolved` when state is otherwise
healthy), and PAUSE acknowledgement stays false until that operator RPC completes
and a causally newer PAUSE receives a positive reply. An operator response timeout
itself requests PAUSE; resume/rewind is never automatically retried. If the response
never arrives, readiness remains unconfirmed; fresh odometry cannot clear it.
This is local request reconciliation, not controller-side generation fencing:
other controller clients can still change modes, response loss can leave uncertainty,
and restarting this process does not prove remote requests were canceled.

`resume` and `rewind` require healthy state and an acknowledged PAUSE with no pending mode request. They are sent once, never retried automatically. The controller's own mode interfaces remain separate command authorities; this monitor is not an exclusive arbiter for every controller writer. No forced disarm or autonomous LAND escalation is provided.

## Acceptance deadline and escalation telemetry

An unresolved protective intervention raises `pause_escalation_required` after
`pause_escalation_timeout_wall_s` steady seconds (default 3.0). This is an authored
budget for observing a positive controller reply and reconciling superseded local
operator requests. It is not a physical stopping deadline. The wall health timer
samples the deadline; publication latency also depends on callback scheduling.

Retries, repeated operator PAUSE messages and a late operator reply followed by a
renewed PAUSE all retain the original episode start. A positive PAUSE reply with
no superseded operator outstanding resolves the current escalation. The JSON status
retains `deadline_exceeded=true` and freezes `elapsed_wall_s` for that episode.
A later intervention increments `generation` and clears its deadline history.
Unhealthy state can remain unhealthy after request acceptance. A pending operator
request can also keep an intervention unresolved despite a positive PAUSE reply.

`pause_intervention_status` is one JSON snapshot with these fields:

| Fields | Meaning |
| --- | --- |
| `schema_version`, `monitor_instance_id`, `sequence` | Version 1; random process identity and increasing publication sequence |
| `generation`, `active` | Monitor-local intervention episode and unresolved-request status |
| `acceptance_budget_wall_s`, `elapsed_wall_s` | Declared steady-time budget and elapsed episode duration |
| `deadline_exceeded`, `escalation_required` | Retained miss history; currently unresolved and overdue |
| `pause_submission_attempts`, `pause_response_timeouts`, `pause_rejections` | Episode counters; unavailable service does not count as an RPC submission |
| `last_pause_result` | `not_attempted`, `service_unavailable`, `awaiting_response`, `accepted`, `rejected`, `response_timeout`, `response_exception` or `submission_exception` |
| `operator_outcome_unresolved`, `pause_acknowledged` | Local reconciliation and PAUSE reply status |
| `physical_stop`, `authority_scope` | Always `UNVERIFIED` and `monitor_local_requests_only` |

The JSON snapshot carries consistent fields; separate Bool topics are convenience
signals and are not an atomic multi-topic transaction. `escalation_required=false`
can mean still within budget, rather than acknowledged or safe. Consumers must
check publication freshness, process identity and the complete snapshot as needed.
Restart changes process identity and starts a fresh pending startup PAUSE; an old
process's generation cannot authorize a new process or cancel remote execution.

This telemetry changes no existing readiness, retry or operator admission policy.
No recovery consumer, automatic LAND, disarm or controller authority transfer is
wired to it. It does not implement the full C08 exchange envelope or authenticated
controller-side generation fencing. Independent motion evidence is still required.

## Parameters

All periods must be finite and positive. Response timeout must not exceed retry period; health publication period must not exceed one second.

| Parameter | Default | Clock / purpose |
| --- | --- | --- |
| `state_estimate_timeout` | 1.0 s | ROS time: source and accepted-receipt age |
| `receipt_timeout_wall_s` | 2.0 s | Steady time: last accepted advancing source stamp |
| `health_publish_period_s` | 0.25 s | Wall timer: health evaluation/publication |
| `pause_response_timeout_wall_s` | 0.75 s | Steady time: pending RPC deadline |
| `pause_retry_period_wall_s` | 1.0 s | Steady time: minimum interval between PAUSE attempts |
| `pause_escalation_timeout_wall_s` | 3.0 s | Steady time: unresolved acceptance/reconciliation escalation budget |

The wall deadline is deliberately separate from the ROS-time threshold. Changing simulation speed does not silently redefine either clock domain.

## Endpoints

Names below are relative to the node namespace. The canonical launch file remaps odometry and the mode service through the selected stack.

| Endpoint | Type | Purpose |
| --- | --- | --- |
| `state_estimate` | `nav_msgs/Odometry` subscription | Source-stamped state |
| `command` | `std_msgs/String` subscription | Operator pause/resume/rewind |
| `set_trajectory_mode` | `airstack_msgs/TrajectoryMode` client | PAUSE or explicit operator mode |
| `state_estimate_timed_out` | `std_msgs/Bool` publisher | State fault, unacknowledged intervention or superseded operator RPC |
| `pause_acknowledged` | `std_msgs/Bool` publisher | Latest requested PAUSE accepted; not physical stopped state |
| `state_health_reason` | `std_msgs/String` publisher | Health reason, including `pause_pending` / `operator_request_unresolved` |
| `pause_escalation_required` | `std_msgs/Bool` publisher | Current unresolved acceptance deadline exceeded |
| `pause_intervention_status` | `std_msgs/String` publisher | JSON snapshot of episode identity, deadline history and local request outcomes |

## Verification

The package registers `isolated_watchdog_contract`, `isolated_operator_preemption_contract`
and `isolated_escalation_contract` with CTest. Run the package tests inside the robot ROS overlay:

```bash
colcon test --packages-select drone_safety_monitor --event-handlers console_direct+
colcon test-result --test-result-base build/drone_safety_monitor --verbose
```

The test starts the actual installed C++ binary on isolated DDS domain 197, using a controlled ROS clock and mock controller. It covers source/clock faults, resumed receipt, service rejection, withheld/late replies, availability recovery, bounded retries, continuing heartbeat and operator interlocks. Output defaults to a new temporary directory; `WATCHDOG_TEST_OUTPUT` selects a new persistent directory. It never commands the simulator or an aircraft. Physical regression is separate and source-specific.

The preemption test uses the actual build target and a mock controller that holds
resume/rewind responses while PAUSE or a stale-state fault arrives. It requires
PAUSE receipt before releasing the operator response, faulted readiness while that
request is unresolved, and a new PAUSE after its late application. It also covers
operator-response timeout without a separate fault. Both tests run serially on
DDS197; neither establishes physical stopping or exclusive mode authority.

The escalation contract uses isolated DDS195 and a fake mode server. It checks
unavailable service, rejection, response timeout, later acceptance, duplicate PAUSE,
held superseded operator replies and renewal after completion. It validates current
escalation, retained history and process/sequence identity. It does not command a
simulator or qualify physical stopping or exclusive controller authority.
