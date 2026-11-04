# EXP124: leased shared-memory native handover

2026-11-03, ore_ms, parent EXP123/cb7f703e1. Freeze before native outcomes.

Research link: test whether the sparse/readiness handover mechanism retains its
native useful-delivery behavior with explicit finite encoded storage and worker
ownership. This is synthetic engineering validity, not natural-fault adaptation,
physical accuracy, training improvement, or deployment readiness.

Use a new opt-in controller/worker; preserve historical code. Primary store cap
4MiB, diagnostic2MiB; same EXP123 eviction/lifetime/outages. Allocate one shared
2MiB handover arena globally. One lease at a time. Worker receives only descriptor
metadata via bounded messages, maps exact shared bytes, validates integrity and
strict source order, drops mapping/views before acknowledgment. Controller reuses
only after matching token/camera/source/group acknowledgment. No next step until
current acknowledgment: one global command credit and serial backpressure. Bound
step message<=4096 bytes and all other messages<=65536 serialized bytes; fail
closed on excess, timeout, malformed descriptors or wrong acknowledgments.

Encoded store+allocated arena<=6MiB primary/4MiB diagnostic. Excludes staged input,
Python/control serialization/kernel buffers, normal file decode, tensors/GPU,
RSS/OS cache. No total process-memory or live sensor cadence claim.

Six dual captures: unperturbed4 fullread, perturbed full_read_cache4,
bounded_probe4 (original), fixture_on4 (frozen EXP122 request/actions),
leased_probe4 live readiness/catchup, diagnostic_probe2 live. Same100 exposed day
frames, RGB20–34/thermal50–64/both75–79 outages, immediate requested transport
feedback, weights/native solver/readiness4attempts20extras and2queries total.
Catch-up misses consume queries. No query on unselected expert; no past deadline
can be revised by historical invocation. Fixed fixture is mechanism validation,
not a fresh adaptive controller. All outcomes retained; no retries or tuning.

Primary contract: strict byte controls against EXP122 unperturbed and fixture_on;
all lease/message/store constraints;4MiB live selected eligibility>=original,
unavailable/uncommitted-opportunity<=original; all4 return episodes obtain an
on-time adjacent output; live calls<fullread.2MiB is descriptive, no required
availability result. Report all95 guarded slots, actual timing/calls and return
latencies. Lock capture files then predictions before any source pose reference;
no new reference-based accuracy claim required. Independent event/prefix/endpoint
and ownership audit. New data unnecessary for this bounded test; natural sustained
transitions and justified independent pose references needed for broader claims.
