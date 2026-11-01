# Opt-in state-evidence shadow boundary — predeclared design

Declared2026-11-01 America/New_York at source `dd8c0d7eb`, before implementation
and new coverage comparison. Design is informed by the previously inspected
RGB recovery and root evidence; this is development work, not a prospective test.

The boundary is a separate pure opt-in function. It consumes camera-scoped
controller journal prefixes, native target rows/pair, original deadline and
explicit per-camera root envelopes. Scope/clock identifiers are caller
attestations. Reject malformed global scope, camera, ordering, worker identity
or after-deadline input by raising ValueError; emit no advice for invalid context.
No clock conversion, late-event truncation, graph repair or root inference.

Compute the unchanged raw advisory availability internally from those events.
For each raw-available camera, bind its unique exact first-write child and invoke
the unchanged predecessor validator at that same deadline. Eligible means raw
availability AND `PASS_PREDECESSOR_EVIDENCE`. Every unavailable/rejected expert
remains in diagnostics. An unavailable camera is not promoted merely because a
root or other state evidence exists. A missing witness is missing documentation,
not proof of inaccurate/unsupported initialization.

| RGB eligible | Thermal eligible | Shadow action |
|---|---|---|
| true | true | RGB (existing fixed preference) |
| true | false | RGB |
| false | true | THERMAL |
| false | false | ABSTAIN |

Fallback chooses the sole eligible expert, including thermal when preferred RGB
fails evidence. No fallback may bypass evidence and no history warm-up is added.
The result is advisory shadow metadata, always `physical_motion_admitted=False`;
no motion, estimator feedback or production/default integration. This is state
evidence gating, not a calibrated quality score or physical verification.

Freeze implementation/tests/scripts and input metadata hashes before replaying
all four EXP092/EXP093 flag-off/on captures. Population:236original decisions /
472camera slots from two already-exposed day/night windows, with repeated control
and witness executions. At every original prefix compute raw, unchanged symmetric
two-interval history and this boundary. Preserve original advice/availability,
all rejected/missing/cold-start/terminal cases and per-camera evidence; compare
coverage/actions only. Do not read reference labels/errors, score trajectories,
fit/tune, infer a new scene or open reserves. Never backfill old roots. No wider
held-out, physical, causal-error or general-quality improvement claim.

Before execution, test meaningful raw/evidence intersections, actual root vs
nonroot support, receipt/identity binding, missing/late/conflicting state, exact
large timestamps and immutability. An independent auditor must reconstruct
choices and populations directly from timelines and prior independently audited
state bindings without importing the new boundary/replay. Mutation checks must
reject advice/eligibility/evidence/deadline/scope/population/physical-admission
corruptions. The underlying raw/history/validator/admission sources stay unchanged.
