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
| E2 sonnet #1 (rerun) | as-e2-sonnet-5-001-3 | 4/16Gi/100Gi | 16:41 | scored after **resume-scoring** (runner crashed on a root-owned `system/core` dump before scoring; fixed `091f176`, scoring rerun in place 20:51–21:31 with the parked workspace; agent fields reconstructed: wall-clock cap hit, no usage event). In-session R1–R6 passed (R6 at 20:22); final-state scoring: R7 fail (clearance), R6 fail (`./takeoff` timed out 180 s after the R7 flight), R5/R4 fail (alpha not running, beta configured), R3 pass. **Audit flag: 22 direct executions of judge internals** (`a3_checks.py`, `r5_provenance.py` via nohup) besides 13 counted shim calls. | **R3** | 13 (+22 direct) | 4.0 (cap) | — |
| E1 sonnet #1 (attempt 1) | as-e1-sonnet-5-001-1 | 8/32Gi/200Gi | 15:11 | **infra failure — OSMO node went NotReady at 19:01 EDT (exit 137/2137)** after 3 h 40 m; in-session R1–R5 passed; re-queued (rule 4) | — | 6 (not scored) | — | — |
| E2 sonnet #3 | as-e2-sonnet-5-003-2 | 4/16Gi/100Gi | 19:04 | scored (voluntary exit; audit clean) | **R8** | 11 | 1.86 | 17.98 |
| E2 opus #2 | as-e2-opus-5-002-3 | 4/16Gi/100Gi | 19:04 | scored (voluntary exit; audit: 2 direct executions of check internals — `r4_planner_check.sh route_planner_beta`, importlib of `r5_provenance.py` — no direct judge flights) | **R8** | 14 | 2.11 | 22.86 |
| E1 opus #2 | as-e1-opus-5-002-1 | 8/32Gi/200Gi | 21:20 | scored (voluntary exit; audit: 2 direct check executions, see audit log) | **R8** | 13 | 3.28 | 21.18 |
| E1 sonnet #2 | as-e1-sonnet-5-002-1 | 8/32Gi/200Gi | 21:05 | scored (voluntary exit at 3.8 h; in-session R6 00:01, R7 ×3 fail then **R7 + R8 passed 01:07**, but the agent left planner alpha active in the final state; final-state scoring: R7/R6 fail — beta not running, R5 fail — alpha route fails minimum geometry, R4 pass; audit: 3 direct R4-check executions) | **R4** | 9 | 3.80 | 22.91 |
| E1 sonnet #3 | as-e1-sonnet-5-003-1 | 8/32Gi/200Gi | 00:48 | scored (voluntary exit at 1.45 h; audit: 1 direct check execution) | **R8** | 10 | 1.45 | 16.50 |
| E1 opus #3 | as-e1-opus-5-003-1 | 8/32Gi/200Gi | 00:49 | scored (voluntary exit at 1.9 h; audit: 1 direct check execution) | **R8** | 15 | 1.89 | 17.50 |
| E1 sonnet #1 (attempt 2) | as-e1-sonnet-5-001-2 | 8/32Gi/200Gi | 19:04 (agent 19:45) | scored (cap hit; in-session R1–R6 by 22:44, R7 ×3 fail; final: R7 fail clearance, R6 fail goal error 7.9 m, R5/R4 fail alpha not running; audit clean; provisioning 853 s) | **R3** | 11 | 4.0 (cap) | — |
| E2 opus #3 | as-e2-opus-5-003-1 | 4/16Gi/100Gi | 21:31 | scored (cap hit while re-verifying R7 after in-session R8 at 22:57; final scoring R7+R8 pass; audit clean; no usage event) | **R8** | 7 | 4.0 (cap) | — |
| E2 sonnet #4 | as-e2-sonnet-5-004-1 | 4/16Gi/100Gi | 00:36 | scored (voluntary exit at 1.45 h; **in-session R6/R7/R8 all passed 01:59–02:06**; final-state scoring 5 min later: R7 fail clearance, R6 fail track order, R5/R4 fail alpha not running, R3 pass; audit clean) | **R3** | 9 | 1.45 | 10.60 |
| E1 opus #1 | as-e1-opus-5-001-1 | 8/32Gi/200Gi | 15:12 (agent 15:53) | scored (agent exited voluntarily; provisioning 1078 s; audit: 0 direct harness execs) | **R8** | 14 | 2.19 | 22.57 |

Notes
- Pod resource requests differ across trials (cluster contention, §(c) finding 3);
  recorded per trial in `osmo_workflow.txt`. Flight budgets are judged on the
  odometry clock, R2 rates are wall-clock (`ros2 topic hz`, ≥5 Hz threshold).
- Runner fix `cdd72a8` (process-group kill + pre-scoring sweep) is in every pod
  launched from 15:11 EDT on.
