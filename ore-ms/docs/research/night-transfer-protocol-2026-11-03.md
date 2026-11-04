# EXP125: night-condition transfer

2026-11-03, ore_ms. Parent680a2da7c / EXP124. Freeze before native outcomes.

Use the sole prepared >=100-frame night window in the exposed EXP119 inventory:
results/long-night-validation/prepared-001/datasets/middle,100frames. Different
recording from the day development window; verify all400 image hashes disjoint.
This night interval and its original native health were previously inspected.
This is condition transfer of the unchanged candidate, not a blind holdout,
independent training evaluation, or causal effect of illumination. No new data
or reserved captures. Preserve the final two unexamined MS2 captures.

Use exact EXP124 leased_handover/worker/transport,4MiB store+2MiB arena, one global
lease/command credit,4/20readiness caps,1query/group2/run, existing cache clocks,
solver/weights,100source frames and current/prior retention/expiry/eviction.
Same source outages RGB20–34,thermal50–64,both75–79. Transport failures are explicit
and requested, not silent/natural faults. No mechanism or budget changes.

Six fresh dual captures: unperturbed fullread; perturbed full_read_cache,
bounded_probe,leased_probe,fixed_rgb,fixed_thermal. All share dual root/guards,
encoded store/arena/startup overhead; fixed arms are not pure single-expert costs.
Keep all95guarded slots per arm, native failures, unavailable choices and abstains.
Strict unperturbed array-byte comparison to EXP119 full-read night. If capture
fails preserve attempt, no automatic retries or replacement windows.

Predeclare matched_rgb and matched_thermal readouts: constant camera choice at
each original candidate deadline using its exact requested native source history
and returned outputs. Copy its full actual call/time/resource cost to each.
No new inference and no outcome-selected schedule. This isolates readout choice
conditional on candidate acquisition/compute. It is not a deployable fixed-camera
scheduling comparison or an independent run; separate native fixed arms supply
that comparison under common overhead. Report both readouts, not just the better.

Execution gate: original unperturbed arrays byte-exact; all source/prefix/native
interval/lease/encoded-store/control-message budgets pass independent audit.
Research superiority gate (separate): candidate selected eligibility>=full and
>=original sparse, actual pair calls<full, selected eligibility strictly greater
than both native fixed-camera arms and both exact-cost constant readouts. These
are descriptive recording-level tests, not a significance test. Report each
criterion; do not tune the policy, budgets or camera preference on a failure.
All4returnepisode output-readiness latencies/censoring remain descriptive; a
native unsupported modality is not assumed recoverable because transport returns.

Capture/prediction locks before scoring; no pose-reference labels read. Report
actual synchronized native and controller time alongside the approximately10s
source span. Serial offline group deadlines and scoped encoded capacity are not
live acquisition cadence, total RSS/GPU memory, physical pose accuracy or general
uncertainty calibration. Broader natural transitions and independent justified
pose references remain useful additional evidence.
