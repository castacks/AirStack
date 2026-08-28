# (c) Reference-solvability re-demonstration under frozen params — 2026-08-28

**Question:** does the reference solution pass R7 under the frozen safe
config (pin `891ca138`, pause-brake-only) with the candidate 240 s
per-checkpoint budget? This is the re-demonstration the lead's budget
decision was waiting on.

**Setup:** fresh A1-identical workspace (`run_trial.py`'s own prep
functions) at the pinned commit; reference solution planner b; judge
shim's `STUDY_WAYPOINT_TIMEOUT` edited 120→240 (the only deviation,
recorded in `PURPOSE.md`); 5 × `./judge --scoring R7`, fresh wall-time
seed each. Raw artifacts (odom CSVs, bring-up logs):
`agent_study/runs/ref_validation_r7_b240_001/`.

## Result: 0/5 PASS — and the budget was never the binding constraint

| Run | Seed | Flight | Outcome | Failure mode |
|-----|------|--------|---------|--------------|
| 1 | 880107 | 136 s | FAIL in-order | Absorbing hover at field entry: boxed at first pillar row, PAUSE brake, parked at (10.3, 1.7, 13.2) for 85+ s → stationary-conclude. 240 s budget untouched. |
| 2 | 880401 | 202 s | FAIL goal error 4.15 m > 2.5 m | Threaded the whole field in ~135 s, then absorbing hover 4 m short of goal at (45.1, −0.9, 15.6) — 5.6 m above plan altitude. |
| 3 | 880770 | 482 s | FAIL clearance 0.302 m < 1.0 m | Only completed route: in-order ✓, goal error 0.216 m ✓, killed by a single pillar shave. |
| 4 | 881324 | 421 s | FAIL in-order | Reached first goal fast (crested z=20.4, above 18 m pillar tops), then 280 s churning a ~4 m pocket at (42, 5) unable to progress to the reversal leg; final stall. All completed legs ≤ 25 s. |
| 5 | 881873 | 243 s | FAIL in-order | 8/9 checkpoints reached; mid-route absorbing hover at (28.5, 16.3, 14.0) from t≈180 s. |

Every completed leg across all runs finished in ≤ 25 s — nowhere near
even the original 120 s budget. **The 120-vs-240 s recalibration
question is moot**: the frozen config fails R7 by stalling (absorbing
PAUSE hover / local-minimum churn) or shaving pillars, never by running
out of leg time.

## Failure-mode reading

The three modes are the structural signature of a purely reactive local
planner in a dense field, and they sit exactly where the 008 tuning
campaign predicted:

- **Absorbing hover (runs 1, 2, 5):** boxed → PAUSE brake → drift above
  plan altitude (above-plan climb is free in the frozen asymmetric
  z-cost) → paused vehicle sweeps no camera FOV → no new free-space
  evidence → permanent hover. Every stall shows z 3–6 m above plan.
- **Local-minimum churn (run 4):** no global search/memory to route
  around structure occluding the next waypoint.
- **Clearance shave (run 3):** the frozen expansion/veto radii can
  thread the 5 m pillar spacing but not with a guaranteed 1.0 m margin.

The recovery mechanisms that would break the hovers (REWIND,
breadcrumb-retreat unstick) are exactly the ones rejected as
crash-prone in this entry's tuning campaign — blocked on the
controller-level safe-retreat primitive (virtual-time tracker races
committed-path discontinuities).

## Consequence for campaign v5

Verdict (c) is **❌ under the frozen config**: reference R7 solvability
is NOT demonstrated (0/5; the two mid-tuning passes came from configs
later rejected as unsafe). The lead decision changes shape from "pick a
budget" to one of:

1. **Recalibrate R7 difficulty** into the frozen stack's envelope
   (fewer/sparser pillars, 2 legs, and/or clearance gate 1.0→0.5 m —
   run 3 passes at 0.5 m). Cheapest; nothing scored yet, so a re-pin
   is allowed.
2. **Controller safe-retreat primitive**, then re-enable the
   already-implemented retreat unstick — fixes the named root cause,
   larger scope (trajectory controller timeline).
3. **Different local planner** (map-based search over LiDAR/VDB local
   costmap) — correct long-term fix for local minima, biggest lift.

`HARD_CAP_S` note for whichever path is taken: the judge caps flight
observation at 900 s regardless of budget; run 3 used 482 s, so the cap
is not currently binding, but a recalibrated rung should re-check it.
