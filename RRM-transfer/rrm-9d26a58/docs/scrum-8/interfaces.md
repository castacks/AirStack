# Logical interface contracts — revision 1

These contracts define semantics before choosing serialization, ROS topics, services, actions or process placement. `C01`–`C09` are architecture IDs, not new approved system requirements. Each refines the requirements allocated in [architecture](architecture.md).

## Shared envelope and rules

Every exchanged record has `schema_version`, `run_id`, `task_id` when applicable, globally unique `message_id`, producer identity/epoch, monotonic producer sequence, `caused_by` references, wall timestamp, and simulation timestamp with clock/episode ID when relevant. Monotonic durations are compared only within an identified clock domain. Simulation reset starts a new episode/epoch. Expiry uses the dispatch authority's monotonic clock, not a learned timestamp or paused simulation time.

IDs/revisions are immutable references to stored payloads. A payload change requires a new revision. Retransmission keeps the message ID and identical bytes; a repeated ID with different contents is rejected. Unknown schema major version, malformed/nonfinite fields, unresolved refs or missing required evidence cannot authorize motion. Error results carry typed reason, affected refs and recoverability; silence is never success. Consumers reject regressing revisions and detect sequence gaps. Message limits, queue bounds and deadlines are required deployment configuration, not unspecified infinite waits.

Geometry is typed with frame, units, timestamp, uncertainty and resource identity behind the embodiment boundary. Joints and Cartesian positions are different types; neither can be guessed from tuple length. Task reasoning sees entity IDs and semantic limits/feasibility. Authored operation definitions supply preconditions/effects and their revision; generated effects cannot redefine success.

| Contract | Producer → consumer | Required payload | Failure/default |
| --- | --- | --- | --- |
| C01 TaskRequest / Interaction | O ↔ R/W/S | Task ID/revision, objective, context refs, constraints, issuer/authority, permission ref, requested embodiment; interaction ID/type, unresolved issue, response, scope, expiry and status | Missing authority or material clarification/approval: pending, no dispatch |
| C02 StateSnapshot / Observation | E/perception → W → R/P/S/M/T | Snapshot ID/revision, observation clock/times, entity/resource IDs, fact truth/confidence/provenance, frame refs, task progress, interaction history, stale/missing channels | Unknown or stale required fact: refresh/assist/hold |
| C03 CapabilityDeclaration / Feasibility | E → P/S/R | Embodiment ID, declaration revision, operation semantic revisions, resources, availability, semantic constraints, typed numeric-limit profile ref, stop/observe support; query binds action/state/profile and returns feasible/infeasible/unknown with reasons | Undeclared action/resource, unavailable resource, unverified limits or grounding: no dispatch |
| C04 ReasoningResult | R → P/O/W | Task revision, state/capability refs, grounded goal predicates/entities, constraints, evidence refs, unresolved ambiguities, intent status, explanation, model/prompt revision when used | Unsupported intent or ambiguity: explicit result; no executable plan |
| C05 PlanProposal | P → S/M/W | Plan ID/version, task/intent/state/capability/semantics refs, uniquely identified actions, dependencies, resource requests, authored preconditions/effects, expected-effect observation window, deadlines and recovery budget | Cycles, bad arity/grounding, unknown semantics or unmet feasibility: reject proposal |
| C06 SafetyDecision | S → dispatch authority/E/T | Decision ID, exact action digest, attempt/dispatch ID, complete checked-context refs, allow/deny/needs-approval/unknown, reasons, authority identity, issue/expiry clock, checks/evidence, stop generation | Only current explicit allow is eligible; all other statuses hold |
| C07 ExecutionRequest / Status / Outcome | S ↔ E → M/W/T | Dispatch ID, plan/action refs, authorized digest and decision, resource reservation, grounded command ref; accepted/rejected/progress/terminal status, actual adjustments, observation refs and error | Acceptance timeout: reconcile same ID, never blind new attempt; completion without effects remains unverified |
| C08 Stop / Cancel / Override / Reset | O/M/S ↔ E | Intervention ID/authority, target task/resources/dispatch IDs, reason, priority, stop generation; received/cancel-accepted/stopped/safe-confirmed status with observed evidence | Latch admission closed; lost acknowledgment leaves safe state unconfirmed |
| C09 TraceEvent / RunManifest | All → T → evaluator | Envelope, event type, complete referenced payload or immutable artifact hash, source/config/model/scene versions, seeds, clock mapping, resource allocation, dependencies, decision and outcome links | Missing record is evidence-incomplete; pre-dispatch logging failure inhibits admission |

## C01: clarification and approval are distinct

Clarification resolves intent or entity binding. Approval grants a scoped permission according to the configured authority; a text response cannot implicitly grant broader control. Store requested and received revisions, actor, decision and expiry. A response for an old task/interaction does not approve a changed goal. Rejection, timeout and withdrawal are explicit statuses. Constraints and dialogue persist in W. Material uncertainty includes unresolved target identity, stale safety evidence and indeterminate capability; thresholds and reasons are visible run configuration. A low-confidence number alone does not decide that a risk is acceptable.

## C02: evidence and memory

Fact truth is `TRUE`, `FALSE` or `UNKNOWN`. Missing, stale, contradictory or insufficiently supported evidence is UNKNOWN; negating UNKNOWN remains UNKNOWN. Preconditions and successful effects require TRUE. Explicit negative evidence is required for FALSE unless a named source declares complete coverage of the relevant domain. For wildcard negation such as “not on any support,” absence in a partial scene is not negative proof. Confidence never substitutes for provenance or observation time.

Snapshots are immutable, task-relevant views including resource states, operator decisions and observation gaps. They describe belief; separate simulator ground truth is used for scoring. State update and dispatch races must be serialized or conservatively revalidated. The first implementation invalidates on any snapshot revision; a later dependency-filtered strategy needs explicit evidence that omitted changes are irrelevant.

## C03: declaration is not certification

Capabilities declare operation vocabulary, supported resource combinations, semantic limits and the versioned numeric model used by the adapter. Feasibility evaluates a specific grounded request under that profile and snapshot. Missing pose/frame, stale transform, unsupported action, resource conflict, unknown limit and unavailable observation/stop channel have separate reasons. Never ground an absent pose to zero. A policy that clamps a command must report requested and applied values and trigger renewed validation when the command changes. Declaring support for grasp does not prove a grasp can succeed.

## C05/C06/C07: admission and dispatch lifecycle

Action identity is `(run_id, task_id, plan_id, plan_version, action_id)`. Each physical attempt receives a new `dispatch_id`; retries of a network message retain it. The action digest covers operation/semantic revision, targets, parameters, requested resources, expected effects and grounded command reference. Authorization binds that digest, all identity fields and revisions of state, capabilities, permissions, approvals, constraints, authority epoch and stop generation.

At the final execution boundary, validate that context still matches, expiry has not elapsed, all gates explicitly allow, no stop is latched, resources remain reserved and the decision/dispatch has not been consumed. Serialize this check with admission and stop invalidation. A transport cannot use a detached earlier PASS. Numeric validation is mandatory after grounding and before every motion chunk; a new unsafe observation causes active interruption. A changed chunk requires an appropriately bound check.

Execution states: `PROPOSED → AUTHORIZED → SENT → ACCEPTED → RUNNING → SUCCEEDED | FAILED | INTERRUPTED`. REJECTED can occur before acceptance. Accepted does not mean started, and SUCCEEDED means the executor completed its command, not that task effects are established. M separately records `effects_verified`, `effects_unmet` or `effects_unknown`, with the later snapshot and observation window. An empty plan only succeeds when the goal is freshly established; otherwise it is infeasible or unresolved.

Dispatch deduplication must survive reconnection. After lost acknowledgment, query that dispatch ID; do not retry under a new ID while its outcome is unknown. After supervisor restart, start inhibited and reconcile outstanding execution and resource state before new admission. Distributed exactly-once execution is not assumed. A local lock alone cannot enforce the remote boundary; E must implement deduplication and generation checks.

## C08: interruption and recovery

Stop closes admission first and increments the stop generation, invalidating outstanding authorizations. It cancels queued/in-flight work through a bounded independent path. `received`, `cancel_accepted`, `motion_stopped` and `safe_confirmed` are distinct observations. A hand holding an object may need controlled hold rather than releasing it; the safe condition is declared and measured per embodiment. Missing safe-state evidence means `SAFE_UNCONFIRMED`, never SAFE.

The path must operate while the reasoner is busy or failed. Override first interrupts/reconciles active work, then creates a new authorized task/control context; it is not permission to bypass numeric or safety checks. Reset requires an authorized request scoped to the current stop generation and fresh adapter evidence of the safe condition. It never reuses an old allow decision. Communication loss triggers the adapter's declared protective behavior and escalates; queued work cannot automatically restart on reconnect.

Recovery records the divergence, invalidated plan, selected alternative/assistance/stop, budget consumed and final observed result. Replanning itself is not counted as a recovery. Repeating a rejected unchanged proposal is blocked pending new evidence, changed context or an alternative. Exhaustion yields explicit failure/assistance and safe-condition status.

## C09: reconstruction and timing

Store task requests and revisions, snapshots/fact provenance, capability profiles, feasibility queries, interpreted intent, plans and authored semantics, policy/model identity, safety/permission/approval decisions, admissions, adapter adjustments, status, observations, comparisons, replans, interventions and final status. Every dispatch links backward to one valid allow and forward to a terminal result or explicit unresolved outcome. Large sensor records may be referenced by immutable hashes if exported and resolvable.

Record append-before-dispatch intent durably. If evidence storage fails, inhibit new execution and use the independent stop path; a stop must not wait for successful disk writes. Bound queues and record dropped observations/gaps. Restart epochs and monotonic sequences expose missing records. Replay validates causality and computes metrics from recorded boundaries; it does not claim to reproduce a stochastic physical trajectory bit-for-bit.

## Implementation scope

The first contract foundation implements only explicit truth handling, semantic capability checks and an in-process authorization-consumption/stop-generation guard. It is not C01–C09 serialization, a planner, authenticated safety authority, physical stop implementation, telemetry replay service or a replacement for the existing loop. Integration must satisfy the full contracts above before use beyond mocks.
