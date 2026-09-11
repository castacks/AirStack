# (f) Trial matrix — running record (campaign 2026-09-icra27-vic-v7-external)

Orchestrator: `agent_study/osmo/osmo_orchestrator.py`, state in `agent_study/osmo/state/`,
log `orch_pilot_real.log` (scratchpad). Cap: 4 pods. Queue order (round-robin):
E2:son:1 (rerun) · E1:son:1 · E1:opus:1 · E2:opus:2 · E2:son:3 · E1:son:2 · E1:opus:2 ·
E2:opus:3 · E2:son:4 · E1:son:3 · E1:opus:3 · E2:opus:4 · E2:son:5 · E1:son:4 · E1:opus:4 ·
E2:opus:5 · E1:son:5 · E1:opus:5.

| Trial | Pod | Resources | Start (EDT) | Disposition | Score | Judge calls | Agent h | USD |
|---|---|---|---|---|---|---|---|---|
| E2 sonnet #1 (pilot) | as-e2-sonnet-5-001-1 | 8/32Gi/150Gi | 06:28 | **infra defect** — orphaned agent judge overlapped scoring (see (e)); archived `.pilot-orphan-contaminated`; RERUN queued | (R3, not of record) | 14 | 4.0 (cap) | — |
| E2 opus #1 | as-e2-opus-5-001-2 | 4/16Gi/100Gi | 11:08 | scored | **R8** | 16 | 3.97 | 26.23 |
| E2 sonnet #2 | as-e2-sonnet-5-002-2 | 4/16Gi/100Gi | 11:18 | running (R6 passed in-session 12:28; orphan watchdog armed) | | | | |
| E2 sonnet #1 (rerun) | as-e2-sonnet-5-001-2 | 8/32Gi/200Gi | 15:14 | submitted | | | | |
| E1 sonnet #1 | as-e1-sonnet-5-001-1 | 8/32Gi/200Gi | 15:11 | submitted | | | | |
| E1 opus #1 | as-e1-opus-5-001-1 | 8/32Gi/200Gi | 15:12 | submitted | | | | |

Notes
- Pod resource requests differ across trials (cluster contention, §(c) finding 3);
  recorded per trial in `osmo_workflow.txt`. Flight budgets are judged on the
  odometry clock, R2 rates are wall-clock (`ros2 topic hz`, ≥5 Hz threshold).
- Runner fix `cdd72a8` (process-group kill + pre-scoring sweep) is in every pod
  launched from 15:11 EDT on.
