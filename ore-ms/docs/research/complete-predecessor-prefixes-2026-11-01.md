# Complete predecessor evidence on EXP092 controller prefixes

2026-11-01 America/New_York; branch `ore_ms`; source commit `33a31e53c`.

The complete prefix replay retains all76camera slots from the two original
[EXP092 runs](../../experiments/EXP-092-root-initialization-capture.md).
Both arms have34ordinary supported-parent evidence passes and2final slots
without a received child motion. The flag-off control has2missing initialization
records; the witness arm has2documented-root passes. All original advice and
availability remain unchanged. No physical motion is admitted.

## Scope and method

This follows EXP092's proposed complete-prefix accounting. It reads only saved
metadata from the already-exposed EXP090/day-original20-frame inputs and
EXP092 flag-off/flag-on controller journals. Each arm has19original decisions
and38camera slots. These repeated runs share the same inputs; they are not
independent scientific recordings.

The replay locks144source, capture-output, script and declaration hashes before
execution, after verifying the previous capture validation/plan and original
580source,85input,2checkpoint and6script bindings. Protected sources include the
unchanged predecessor validator, shared controller, history/gate and motion
admission/composition modules. No images are decoded, models run, labels or
proxy scores read, parameters fitted, reserve opened or default changed.

For each original decision, each camera's journal contains only received events
before that decision, using its original controller deadline. Run and camera
scope prevent cross-run graph-index aliases. Native requested endpoints come
from the archived input rows. All first-write candidates ending at that target
are inventoried; only a unique exact-interval candidate is checked. Missing,
ambiguous or interval-mismatched candidates remain explicit untested slots.
No later optimizer drain, nearest-time pose or repaired parent is substituted.

The caller explicitly supplies the unique same-prefix initialization envelope
only when the chosen child's native parent index is zero. The validator itself
does not auto-discover initialization. Missing control roots remain missing.
Ordinary cases require unique prior supported parent health, matching earlier
optimization submission/first-write identity, ordered repeated writes and exact
latest observed parent-pose equality, all before the child's first write.
No new acceptance criterion or selector is introduced.

## Complete population

| Arm | Slots | Exact first-write candidates | Ordinary predecessor passes | Documented root passes | Missing root records | No child by deadline |
|---|---:|---:|---:|---:|---:|---:|
| Control, root logging off | 38 | 36 | 34 | 0 | 2 | 2 |
| Witness, root logging on | 38 | 36 | 34 | 2 | 0 | 2 |
| Total | 76 | 72 | 68 | 2 | 2 | 4 |

Both cameras contribute the same per-arm populations:19slots,18exact candidates,
17ordinary supported-parent passes,1root case and1no-child slot. Each arm has
36originally available camera slots. Both root cases are pair0; both no-child
cases are pair18 at the final frame boundary before optimizer drain.
There are no ambiguous or interval-mismatched candidates and no unsupported
ordinary parents in this bounded day capture. Absence of those cases here does
not establish general behavior; the historical middle-night unsupported
recovery remains documented separately.

Control root checks return `REJECT_PREDECESSOR_EVIDENCE` with
`missing_root_initialization`; this is missing documentation, not evidence that
the initialized state was unsupported or inaccurate. The witness roots return
`PASS_PREDECESSOR_EVIDENCE` with `documented_root_initialization`. Ordinary passes
return `supported_parent_with_observed_pose`. No-child slots are `NOT_CHECKED`
with `no_unique_exact_child`, not validator failures or invented null motions.

Thus evidence passes change34→36of38camera slots; original motion availability
stays36of38 and advice stays18RGB/1ABSTAIN per arm. This documents initialization
coverage, not improved motion coverage, accuracy or a deployed rejection policy.
The eleven missing roots in earlier historical journals remain unproven.
Every result has `physical_motion_admitted=False`.

## Independent validation

The auditor imports neither replay nor predecessor validator. It rescans each
original timeline/prefix, checks contiguous worker ordinals and original receipt
cutoffs, independently reconstructs current target availability and advisory
choices, native candidate identities, child submissions, parent health, parent
submission/write ordering, exact pose binding and explicit root documentation.
It then compares every output field, all76slot identities and all arm/camera
counts. Its2,404recursive type/value/schema comparisons pass.

All18semantic output corruptions are rejected: promoted missing root, erased or
misbound witness, wrong parent write/health, cross-run scope, changed advice or
availability, physical admission, omitted slot/arm/camera, altered count/deadline/
child, hidden no-child case, duplicate sample and boolean-to-integer substitution.
JSON key reordering passes. All144frozen input/source/script hashes still match
at audit completion. Status: `PASS_INDEPENDENT_COMPLETE_PREDECESSOR_PREFIXES`.
These checks establish recorded bindings and arithmetic, not independent physical
observations or provenance authentication. Existing function unit suites were
not repeated because no production implementation changed.

## Artifacts, recovery and next bounded step

Artifacts: `results/complete-predecessor-prefixes/run-001/` contains the preserved
declaration, hash-bound protocol, all76rows, grouped summaries and validation.
Recovery includes the standalone frozen replay/auditor, source capture metadata
and local protocol/run/audit logs. The CPU event-test image was used without
neural inference. Writes are exclusive; preserve existing outputs before replay.

The next useful replication is the unchanged witness/validator on the already
exposed middle-night recovery window, with exact inputs and fresh same-runtime
controls frozen before inference. It can test that documented initialization
and unsupported ordinary recovery remain distinct in a fresh journal. Preserve
all slots, failure outcomes and advice; do not adopt a quality/rejection policy
or consume a reserve. Recording-specific rig/reference, physical admission and
checkpoint-training provenance remain independent unresolved inputs.

Recovery: `ore_ms:ore-ms/backups/2026-11-01-complete-predecessor-prefixes`.
