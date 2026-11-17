# Invalid command timestamp containment

2026-11-17, `ore_proj`, source based on `08c465459` plus pinned changes. Actual
controller on isolated DDS190 with synthetic odometry; no simulator endpoints.

Negative seconds abort the precursor controller on all four construction paths:
legacy replacement, legacy append, fenced replacement and fenced lease preparation.
Some noncanonical-nanosecond baseline cases demonstrate acceptance; two final
baseline cases remain discovery-incomplete and carry no claimed disposition.
Earlier discovery-limited attempts are preserved locally. The final candidate
rejects all eight cases without process exit or buffer mutation, and accepts a
valid zero-stamped trajectory after every rejection. Exact submitted path CDRs and
source/harness hashes are retained. Format validation does not prove freshness.

A shared guard checks nonnegative seconds and nanoseconds below1e9 before trajectory
construction. It covers legacy and fenced paths without changing valid timestamps,
trajectory geometry or controller dynamics. Controller12/12CTest passes; aggregate31
tests, zero failures/errors, two skips, including message-package results.

Separate follow-up: an earlier isolated default-budget lease test produced a core
coincident with SIGINT cleanup. Its contract checks passed, but cleanup exit status
was not asserted. The core is preserved locally outside Git and root cause remains
uninvestigated. This was an isolated test executable; no live-controller crash or
binary replacement was identified. Review teardown exit reporting on resumption.

[Compact report](../controller-stamps-20261117.json). Standard-library audit:

```bash
python docs/uav-results/controller-stamps-20261117/audit.py
```
