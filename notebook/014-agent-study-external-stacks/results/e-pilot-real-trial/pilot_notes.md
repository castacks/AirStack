# (e) Pilot real trial — E2 (Aerostack2) / claude-sonnet-5 / #1 — 2026-09-11

Pod `as-e2-sonnet-5-001-1` (8 CPU / 32 Gi / 150 Gi; RTX PRO 5000, driver 580.126.20),
runner `e20b682` (+ uncommitted pycache → "-dirty"), agent start 10:28:33 UTC.
Provisioning (2 AS2 images) 156 s. Auth probe OK.

## In-session judge trajectory (agent-side invocations, 14 of 20)

| UTC | rung | result |
|---|---|---|
| 12:41 | R1 R2 R3 R4 | pass pass pass pass (first judge call 2 h 13 m into the session) |
| 12:45 / 12:57 | R5 | fail / **pass** |
| 13:05 | R6 | **pass** |
| 13:12 … 14:10 | R7 ×5 | fail (obstacle route) |
| 14:28 | — | wall-clock cap (4 h) — agent killed mid-Bash-tool; no usage/cost event |
| 14:42:55, 14:42:56 | R7 ×2 (agent-side) | fail — **logged AFTER the kill**: an in-flight agent judge invocation survived `subprocess.run(timeout)` (only `claude` was killed) and overlapped the scoring pass |

## Final scoring (top-down)

R7 fail (14:41–14:42) → R6 **fail** (14:42–14:48) → R5 fail → R4 fail → R3 pass → **score R3**.

The R6/R5/R4 scoring bring-ups ran while the orphaned agent judge was still
driving the same system (its two fail events land inside the R6 scoring
window), so the R6-and-below verdicts are **contaminated** and this trial's
score is not of record until re-scored. The workspace was discarded with the
pod, but the agent's full solution is archived (`agent_solution.tar.gz`, 
per-repo patches, `solution_manifest.txt`) → re-score by re-provisioning E2
and replaying the tarball (planned: pod "rescore" mode). Disposition:
**infra defect (runner), re-score pending; not counted.**

Runner fix `cdd72a8`: the agent runs in its own process group and the whole
group is killed at the cap; a pre-scoring sweep kills any leftover
`judge.sh`/`r5_provenance.py`/check process (recorded in
`pre_scoring_sweep.txt`). Pods launched after 15:11 EDT carry the fix; the
two E2 pods already running (opus #1, sonnet #2) were protected manually:
opus #1's last agent judge finished before its kill (no orphan; scoring ran
clean), sonnet #2 has an ssh watchdog that kills non-scoring judge processes
once its agent is gone.

Auth watch (lead note 15:09 EDT: shared logins expire the token faster):
no auth-related events in this transcript; orchestrator now flags
`authentication_error`/`OAuth token`/`/login` in downloaded transcripts as
infra failures and re-queues.
