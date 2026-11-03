# EXP117 pre-execution fixture amendment

The first draft lock was written before a failed test result was checked. Retain its plan, source snapshots and failure log under run-001. No schedules or outcomes were computed with that draft.

The two-request probe fixture put the next decision more than1second after the active sensor's last resolved state; both states were stale and the specified policy correctly requested a full refresh, not a periodic inactive probe. Keep the active RGB receipt fresh in the fixture to test the intended probe-burst branch. No cache logic,TTL,probe period,burst length,cohort,guard,warmup,scoring or primary criterion changes.

The execution version uses a new exclusive run-002 and separate fixture-v2 input lock, with the first lock's digest retained. Rerun the focused suite successfully before freezing it. This correction occurs before computing any policy outcomes.
