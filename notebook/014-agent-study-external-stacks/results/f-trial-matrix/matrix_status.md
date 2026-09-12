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
| E2 sonnet #2 | as-e2-sonnet-5-002-2 | 4/16Gi/100Gi | 11:18 | scored (agent exited voluntarily; scoring R7 fail on clearance, R6 pass; sweep found no orphans) | **R6** | 7 | 3.89 | 21.53 |
| E2 sonnet #1 (rerun) | as-e2-sonnet-5-001-3 | 4/16Gi/100Gi | 16:41 | running — **audit flag**: agent executes `r5_provenance.py` directly (nohup, STUDY_R7=1) bypassing the counted shim (rule 5); final scoring unaffected | | | | |
| E1 sonnet #1 (attempt 1) | as-e1-sonnet-5-001-1 | 8/32Gi/200Gi | 15:11 | **infra failure — OSMO node went NotReady at 19:01 EDT (exit 137/2137)** after 3 h 40 m; in-session R1–R5 passed; re-queued (rule 4) | — | 6 (not scored) | — | — |
| E2 sonnet #3 | as-e2-sonnet-5-003-2 | 4/16Gi/100Gi | 19:04 | scored (voluntary exit; audit clean) | **R8** | 11 | 1.86 | 17.98 |
| E2 opus #2 | as-e2-opus-5-002-3 | 4/16Gi/100Gi | 19:04 | scored (voluntary exit; audit: 2 direct executions of check internals — `r4_planner_check.sh route_planner_beta`, importlib of `r5_provenance.py` — no direct judge flights) | **R8** | 14 | 2.11 | 22.86 |
| E1 opus #1 | as-e1-opus-5-001-1 | 8/32Gi/200Gi | 15:12 (agent 15:53) | scored (agent exited voluntarily; provisioning 1078 s; audit: 0 direct harness execs) | **R8** | 14 | 2.19 | 22.57 |

Notes
- Pod resource requests differ across trials (cluster contention, §(c) finding 3);
  recorded per trial in `osmo_workflow.txt`. Flight budgets are judged on the
  odometry clock, R2 rates are wall-clock (`ros2 topic hz`, ≥5 Hz threshold).
- Runner fix `cdd72a8` (process-group kill + pre-scoring sweep) is in every pod
  launched from 15:11 EDT on.
