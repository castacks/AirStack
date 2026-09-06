# Disaster benchmark dashboard

Last reconciled against `/media/share/coa-sei` and the live OSMO queue: **2026-09-05**.

> **Legend:** 🟩 **DONE** · 🟦 **READY** · 🟧 **RERUN** · 🟨 **VERIFY / IN PROGRESS** · ⬜ **NOT READY**
>
> 🟩 means a usable passed benchmark result exists. Pre-optimization results
> count; failed or stopped attempts never count as DONE. 🟦 means the scene is
> ready but the method has not run. 🟧 means the method failed or stopped and
> needs a rerun. ⬜ means the scene itself is not ready.

## At-a-glance matrix

| Disaster | Locale | Level | Scene ready | Frontier | Lawnmower | VLFM | CoNavGPT2 | RayFronts/RAVEN |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **Fire** | **Urban** | L1 | 🟩 | 🟩 | 🟧 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Urban** | L2 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Fire** | **Urban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Urban** | L1–L3 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Hurricane** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Urban** | L1–L3 | 🟨 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Tornado** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Urban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Urban** | L2 | 🟩 | 🟨 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Earthquake** | **Urban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Suburban** | L1 | 🟩 | 🟨 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Earthquake** | **Suburban** | L2 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Earthquake** | **Suburban** | L3 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |

## Fire

### Urban

| Level | What is done | What is left | Intended run folder |
|---:|---|---|---|
| L1 | Canonical frozen scene; Frontier, VLFM and CoNavGPT2 baselines | Lawnmower rerun + RayFronts | `urban_fire_l12_8robot_optimized_pod56/2026-09-05_12-49-49` |
| L2 | Canonical frozen scene is published and passes a fresh Nucleus cold-open | Four baselines + RayFronts | `urban_fire_l12_8robot_optimized_batch/<timestamp>` |
| L3 | Canonical frozen scene; all four shared-planner baselines | RayFronts | `urban_fire_l3_8robot_optimized_1gpu/2026-09-05_06-00-18`; `urban_fire_l3_remaining_8robot_optimized_pod57/2026-09-05_07-46-06` |

All four Urban Fire L3 shared-planner results are present under
`/media/share/coa-sei`. L3 CoNavGPT2 passed at RTF 0.226 and was uploaded
and verified at 13:12 UTC. L1 Frontier also passed after its spawn correction
at RTF 0.219 and was uploaded and verified. All three canonical scenes are now complete. The L1
and L2 Frontier attempts on pod 56 occurred before the canonical publication,
stopped during scene open and were not uploaded. L3 then opened correctly on
pod 57, but two attempts were rejected by an obsolete stereo-disparity gate
even though the optimized mission intentionally uses simulator RGB-D. Commit
`bbeb4237` makes the gate validate the actual Mighty depth-cloud publisher and
subscriber instead. One-GPU fallback batches started on pods 56 and 57 at
06:00 UTC on 2026-09-05. L3 Frontier passed, uploaded and verified. Its timed
window was 54.8 min (RTF 0.183), but that RTF is excluded from the performance
average because Kit had selected another host tenant's GPU rather than pod 57's
reserved card. The bag remains valid for benchmark outcome metrics. The
remaining jobs restarted at 07:45–07:46 UTC with Kit explicitly pinned to the
reserved cards (Isaac indices 1 on pod 56 and 2 on pod 57) and hardened MAVROS
pre-arm enabled. L3 Lawnmower and VLFM each passed on their second attempt and
were uploaded and verified; L3 CoNavGPT2 also passed. The original four L1
attempts reached takeoff but failed because robot 2's generated x=-517.5 m spawn
was outside the 1 km ground plate: its local pose diverged to z=-20.7 km. These
failures were not uploaded. The corrected retry moved that spawn inside the
plate; L1 Frontier then passed with all eight drones. L1 VLFM also passed with
all eight drones at team RTF 0.2115 and is uploaded and verified. L1 Lawnmower
exhausted two attempts (first the perception readiness gate, then takeoff with
robot 8 pre-arm timing out and robot 4's action relay timing out); neither
failed attempt was uploaded, so it requires a focused rerun. L1 CoNavGPT2
passed 17/17 steps and was uploaded and NAS-verified at 18:01 UTC. Its 600.06
simulated seconds took 3194.47 wall seconds (RTF 0.18784) over 27 VLM rounds.
The Fire runner was stopped at the iteration-5 boundary before an L2 stack was
started so pod 56 could switch to Urban Earthquake; its stopped L2 stub was not
uploaded. Two out-of-bounds L2 Fire spawns were already fixed for the later
focused rerun. The
corrected 2-GPU workflows `airstack-mission-8robot-2gpu-5`
and `-6` remain queued as backups.

### Suburban

The accepted pre-optimization 8-robot sweep is
`frozen_suburban_8robot/2026-08-31_11-11-42`. Frontier, lawnmower and VLFM
passed at L1–L3; CoNavGPT2 passed at L2–L3. The optimized Fire L1 CoNavGPT2
rerun also passed, so all shared-planner Fire/Suburban cells are complete.

Development history only:

| Run folder | Result |
|---|---|
| `frontier_wildfire_1robot/2026-08-25_23-34-47` | 1-robot frontier smoke run passed |
| `wildfire1km_1robot_a/2026-08-27_07-41-10` | VLFM passed; CoNavGPT readiness failed |
| `wildfire1km_5robot_a/2026-08-27_18-23-15` | 5-robot VLFM passed; one optional step failed |
| `wildfire1km_5robot_b/2026-08-28_15-43-45` | 5-robot frontier passed; lawnmower readiness failed |

## Hurricane

### Urban

🟥 No completed Urban Hurricane scenes or benchmark results are recorded.

### Suburban

| Level | Completed methods | Remaining methods | Result folder |
|---:|---|---|---|
| L1 | Frontier, lawnmower, VLFM, CoNavGPT2 | RayFronts/RAVEN | See folders below |
| L2 | Frontier, lawnmower, VLFM, CoNavGPT2 | RayFronts/RAVEN | `hurricane_suburban_l2_8robot_optimized_batch/2026-09-04_22-53-30` |
| L3 | Frontier, lawnmower, VLFM, CoNavGPT2 | RayFronts/RAVEN | `hurricane_suburban_l3_lawnmower_rerun/2026-09-05_02-38-56` plus the original batch |

L1 result folders:

| Run folder | Result |
|---|---|
| `hurricane_suburban_8robot/2026-09-02_20-13-16` | Frontier, lawnmower and VLFM passed; original CoNavGPT2 attempt stopped |
| `hurricane_suburban_l1_conavgpt2_optimized/2026-09-04_04-11-15` | CoNavGPT2 passed; 600 sim s, RTF 0.1522 |
| `hurricane_suburban_l1_conavgpt2_gt600/2026-09-04_17-23-22` | Best valid GT run: RTF 0.3194; 2,177 detector calls; 3,120 boxes; 7 person fires ≥0.65; 0/55 GT visits |

The short `hurricane_suburban_l1_conavgpt2_rtf_*` folders are optimization
experiments, not additional benchmark cells. `rtf_s`, `rtf_t`, and `rtf_u`
are startup-invalid.

## Tornado

### Urban

Urban Tornado L1–L3 generation is complete and frozen-scene export is in
progress with another agent. No benchmark result has been submitted yet.

### Suburban

The accepted pre-optimization 8-robot sweep is
`frozen_suburban_8robot/2026-08-31_11-11-42`. All four shared-planner methods
passed at L1–L2. Frontier, lawnmower and VLFM passed at L3; L3 CoNavGPT2 was not
run and is READY. A failed L3 lawnmower attempt is superseded by its passed
canonical iteration and does not change the DONE status.

## Earthquake

### Urban

Urban Earthquake L1, L2 and L3 are canonically published with GT. The L1-L2
shared-baseline batch started on pod 56 at 18:02 UTC under
`urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41`; L1 Frontier
passed 17/17 steps at team RTF 0.19510 and was uploaded and NAS-verified at
19:23 UTC. L1 Lawnmower also passed 17/17 steps after its automatic robot-8
startup recovery, at team RTF 0.20130, and was uploaded and NAS-verified at
20:43 UTC. L1 VLFM passed 17/17 steps at team RTF 0.18919 and was uploaded
and NAS-verified at 22:05 UTC. L1 CoNavGPT2 then passed 17/17 steps on its
second attempt at team RTF 0.25092 and was uploaded and NAS-verified at 23:51
UTC, completing all four L1 baselines. Its first attempt failed the
perception-readiness gate because
robot 1's `global_mapper_ros` DDS subscription was not discovered after a
flapping PX4 startup; the failed artifacts were retained locally and were not
uploaded. On the second attempt, robot 4's odometry stopped crossing the stale
main DDS-router path even though its local topic was healthy; restarting only
that router restored the stream and allowed the 600.03-s search to finish in
2391.34 wall seconds. L2 Frontier started immediately afterward. Its first
attempt failed at takeoff when robot 5 exhausted all three MAVROS pre-arm state
checks; the other seven drones took off, and the failed attempt is retained
locally but was not uploaded. The automatic second mission attempt started at
00:17 UTC after a clean stack teardown, cleared all eight takeoffs (including
robot 5 on its first pre-arm check), and entered the timed Frontier search at
00:45 UTC. The queued 2-GPU workflow
`airstack-mission-8robot-2gpu-7` remains a backup. L3's
four optimized baseline cells are now running on pod 57 from
`urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13`. The first
run folder (`17-13-17`) was stopped and not uploaded after robot 6 reproduced
a takeoff obstruction on both clean stack attempts. Robot 6 was moved from
`(212.5, 140)` to vetted clear point `(300, 200)` across all four environments;
the corrected Frontier run passed 17/17 steps at team RTF 0.19188 and was
uploaded and NAS-verified at 19:53 UTC. L3 Lawnmower then passed at team RTF
0.19888 and was uploaded and NAS-verified at 21:13 UTC. L3 VLFM passed at team
RTF 0.21211 and was uploaded and NAS-verified at 22:29 UTC. L3 CoNavGPT2
then passed at team RTF 0.21651 and was uploaded and NAS-verified at 23:43
UTC, completing all four L3 baselines. Pod 56 has the accepted L1 results
above.

### Suburban

Suburban Earthquake L1–L3 are complete and ready for benchmark runs. L1
first started on pod 57 at 23:45 UTC from
`earthquake_suburban_l1_8robot_optimized_pod57/2026-09-05_23-45-04`; L2 is queued next on pod 56
after Urban Earthquake L1–L2; L3 follows L1 on pod 57. Each level will run
Frontier, Lawnmower, VLFM and CoNavGPT2 with the optimized 8-robot profile.
L1 Frontier attempt 1 failed takeoff after robot 5 was auto-disarmed and all
three bounded takeoff requests were rejected; it was retained locally and was
not uploaded. The clean automatic attempt 2 cleared all eight takeoffs, but
the frozen scene held only about 0.0436 RTF and would exceed the bounded run
gate, so it was stopped cleanly, retained locally and not uploaded. The cause
was isolated to redundant cooking of the generated suburban ground-collider
sheet; frozen suburban missions now use the scene's existing Pegasus ground
plane and a 5-hour completion gate. A fresh optimized L1 run started at 01:29
UTC in `earthquake_suburban_l1_8robot_optimized_pod57/2026-09-06_01-29-21`.
No Suburban Earthquake benchmark result has been accepted yet.

## Failed and superseded attempts

| Run folder | Why it does not count |
|---|---|
| `conavgpt_wildfire_1robot/2026-08-25_03-31-00`, `05-27-48`, `22-26-47` | Robot readiness failures |
| `lawnmower_wildfire_1robot/2026-08-26_00-32-51` | Robot readiness failure |
| `wildfire1km_1robot_b/2026-08-27_07-42-17` | Frontier and lawnmower readiness failures |
| `hurricane_suburban_8robot/2026-09-02_16-35-56` | Readiness failure, then aborted |
| `hurricane_suburban_8robot/2026-09-02_19-02-31`, `19-34-47`, `19-48-19`, `19-56-32`, `20-04-16` | Aborted before a scored cell |
| `hurricane_suburban_l1_conavgpt2_gt600/2026-09-04_14-02-06` | Stale-container/network conflict; superseded by successful `17-23-22` run |

## Next work queue

1. 🟨 Complete the active Urban Earthquake batches on pods 56 and 57; upload
   each passed iteration immediately and publish no failures.
2. 🟦 Run Suburban Earthquake L1 then L3 on pod 57, and L2 on pod 56,
   immediately after those active batches finish.
3. 🟧 Rerun Urban Fire L1 Lawnmower and all four Urban Fire L2 methods.
4. 🟦 Run the queued 2-GPU RayFronts/RAVEN suburban missions.

## Active batch plan

The four shared-planner baselines are assigned to the existing 1-GPU pods,
including an explicit fallback attempt for the heavier Urban Fire cells.
RayFronts/RAVEN remains in the 2-GPU queue. Each active fallback is capped at
12 wall-hours. A cell that exhausts its automatic attempts is diagnosed and
requeued as a focused rerun; it remains outstanding until a passed result is
uploaded.

| Order | Pod | Cells | Runs | Expected batch wall time | State |
|---:|---|---|---:|---:|---|
| 1 | `airstack-mission-1gpu-56` | Fire/Suburban L1 CoNavGPT2 | 1 | ~45 min | PASSED — focused rerun completed before the L2 batch |
| 1 | `airstack-mission-1gpu-57` | Tornado/Suburban L3 CoNavGPT2 | 1 | ~45 min | PASSED — focused rerun completed before the L3 batch |
| 2 | `airstack-mission-1gpu-56` | Hurricane/Suburban L2 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | COMPLETE — 4/4 passed, uploaded and verified |
| 2 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | COMPLETE — 3/4 passed and uploaded; lawnmower failed twice and was not uploaded |
| 3 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 lawnmower | 1 | 61 min | PASSED — uploaded and verified at 03:40 UTC |
| 4 | `airstack-mission-1gpu-57` | Urban Fire L3 Frontier | 1 | — | STOPPED — two attempts hit the obsolete disparity gate; zero scored/uploaded runs |
| 3 | `airstack-mission-1gpu-56` | Urban Fire L1/L2 Frontier | 2 | — | STOPPED — launched before canonical publication; zero scored/uploaded runs |
| 5 | `airstack-mission-1gpu-56` | Urban Fire L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | 8 | 5 h 11 min | STOPPED FOR EARTHQUAKE CUTOVER — L1 Frontier, VLFM and CoNavGPT2 passed/uploaded (RTFs 0.219, 0.2115, 0.18784); Lawnmower failed twice and needs a focused rerun; Fire L2 remains deferred |
| 6 | `airstack-mission-1gpu-56` | Earthquake/Urban L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | L1 4/4; L2 0/4 | ≤12 h | RUNNING — all four L1 methods passed/uploaded/NAS-verified at team RTFs 0.19510, 0.20130, 0.18919 and 0.25092; L2 Frontier attempt 1 failed robot-5 pre-arm and was not uploaded; attempt 2/2 cleared all eight takeoffs and is in the timed search |
| 5 | `airstack-mission-1gpu-57` | Urban Fire L3 Frontier | 1 | 86.7 min total / 54.8 min timed | PASSED — uploaded and verified; RTF 0.183 excluded from performance average due wrong-host-GPU placement |
| 6 | `airstack-mission-1gpu-57` | Urban Fire L3 × lawnmower, VLFM, CoNavGPT2 | 3 | ≤12 h | COMPLETE — 3/3 passed, uploaded and verified |
| 7 | `airstack-mission-1gpu-57` | Earthquake/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | 4/4 | 5 h 12 min | COMPLETE — all four passed/uploaded/NAS-verified at team RTFs 0.19188, 0.19888, 0.21211 and 0.21651 |
| 8 | `airstack-mission-1gpu-57` | Earthquake/Suburban L1 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | RUNNING — the original Frontier attempts were not uploaded (attempt 1 robot-5 takeoff failure; attempt 2 stopped at ~0.0436 RTF). Redundant frozen-ground collider cooking is now disabled; a fresh optimized run started at 01:29 UTC in `earthquake_suburban_l1_8robot_optimized_pod57/2026-09-06_01-29-21` |
| 8 | `airstack-mission-1gpu-56` | Earthquake/Suburban L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | QUEUED NEXT — starts after Urban Earthquake L1–L2 completes |
| 9 | `airstack-mission-1gpu-57` | Earthquake/Suburban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | QUEUED — starts after Suburban Earthquake L1 completes |

RayFronts/RAVEN is split into three 2-GPU workflows. Each runs three scene
levels under a 12-hour mission cap, while the pod itself remains alive for 48
hours for inspection and corrective reruns.

| Workflow | Cells | State |
|---|---|---|
| `airstack-mission-8robot-2gpu-2` | Fire/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-3` | Hurricane/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-4` | Tornado/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-5` | Fire/Urban L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | QUEUED — 12 h mission cap, 48 h inspectable pod |
| `airstack-mission-8robot-2gpu-6` | Fire/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | QUEUED — 12 h mission cap, 48 h inspectable pod |
| `airstack-mission-8robot-2gpu-7` | Earthquake/Urban L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | CANCELED — replaced by the active split run on pod 56 |
| `airstack-mission-8robot-2gpu-8` | Earthquake/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | CANCELED — replaced by the active split run on pod 57 |

The broad Fire and Tornado batches started at `19-51-50` / `19-58-31` were
stopped during startup once the accepted pre-optimization sweep was found.
They were not uploaded and did not produce scored cells. Pod 57's earlier
`19-51-50` preflight also exposed a launcher/Pegasus helper version mismatch;
the matching helper was committed before the clean restart.

The first focused starts at `20-14-35` were also stopped before scoring after
the Kit command line showed unpinned, multi-GPU rendering. The clean starts
above disable multi-GPU and explicitly pin both Kit and offboard CUDA to each
pod's reserved card; their command lines were verified before preflight.

## Average 600-s scene performance

Wall clock is the timed 600-s search window; RTF is simulated seconds divided
by that wall clock. Values average across accepted successful runs in each
scene family, independent of baseline and optimization generation. Failed and
short tuning runs are excluded.

| Disaster | Urban avg wall / RTF | Suburban avg wall / RTF |
|---|---:|---:|
| Fire | — | 97.8 min / 0.102 (n=11) |
| Hurricane | — | 38.3 min / 0.265 (n=9) |
| Tornado | — | 77.4 min / 0.129 (n=11) |
| Earthquake | 48.6 min / 0.207 (n=8) | — |

## Actual results (detector-confirmed team progress and PPL)

A GT victim counts as detected when its world-frame XY location falls inside a **12 m circle around a planner `search_target`** during the 600-s search. A target circle exists only after a `person` detection clears the shared 0.65 confidence gate, is depth-projected, and forms a clustered target instance. One liberal circle can credit multiple GT people; drone proximity alone never counts. Time-integrated progress is normalized area under the cumulative detector-confirmed progress curve; marker chunks were sampled at about 20-s intervals (final persistent target state is always read, so final detection counts are exact). Paths are 1 Hz, world-frame XY odometry. Ideal lengths are OR-Tools oracle estimates for open Euclidean multi-depot routes through victim centres; fixed-sector methods preserve recorded robot ownership, while CoNavGPT2 permits joint assignment. Ground debris does not obstruct an aerial XY geodesic, and no return to launch is required. PPL uses the ideal route through detected GT victims: `progress × ideal_detected / max(actual, ideal_detected)`.

### Per completed run

| Scene | Method | Run folder | GT | Detected | Detector-confirmed progress | Time-integrated progress | Actual team path | Ideal all-target path | PPL |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| Earthquake / Urban L1 | CoNavGPT2 | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_004__earthquakeurbanl1v1_conavgpt2_team__conavgpt2_team` | 39 | 0 | 0.000 | 0.000 | 7.96 km | 3.19 km | 0.0000 |
| Earthquake / Urban L1 | Frontier | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_001__earthquakeurbanl1v1__frontier` | 39 | 0 | 0.000 | 0.000 | 3.98 km | 5.27 km | 0.0000 |
| Earthquake / Urban L1 | Lawnmower | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_002__earthquakeurbanl1v1_lawnmower__lawnmower` | 39 | 0 | 0.000 | 0.000 | 7.38 km | 4.81 km | 0.0000 |
| Earthquake / Urban L1 | VLFM | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_003__earthquakeurbanl1v1__vlfm` | 39 | 0 | 0.000 | 0.000 | 4.16 km | 5.35 km | 0.0000 |
| Earthquake / Urban L3 | CoNavGPT2 | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_004__earthquakeurbanl3v1_conavgpt2_team__conavgpt2_team` | 64 | 0 | 0.000 | 0.000 | 6.03 km | 5.15 km | 0.0000 |
| Earthquake / Urban L3 | Frontier | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_001__earthquakeurbanl3v1__frontier` | 64 | 0 | 0.000 | 0.000 | 5.15 km | 8.13 km | 0.0000 |
| Earthquake / Urban L3 | Lawnmower | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_002__earthquakeurbanl3v1_lawnmower__lawnmower` | 64 | 0 | 0.000 | 0.000 | 10.35 km | 8.13 km | 0.0000 |
| Earthquake / Urban L3 | VLFM | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_003__earthquakeurbanl3v1__vlfm` | 64 | 0 | 0.000 | 0.000 | 3.31 km | 7.93 km | 0.0000 |
| Fire / Suburban L1 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_001__firesuburbanl1v1__frontier` | 49 | 10 | 0.204 | 0.175 | 5.59 km | 1.73 km | 0.0033 |
| Fire / Suburban L1 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_002__firesuburbanl1v1_lawnmower__lawnmower` | 49 | 0 | 0.000 | 0.000 | 14.53 km | 1.78 km | 0.0000 |
| Fire / Suburban L1 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_003__firesuburbanl1v1__vlfm` | 49 | 11 | 0.224 | 0.204 | 4.79 km | 1.69 km | 0.0125 |
| Fire / Suburban L2 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_008__firesuburbanl2v1_conavgpt2_team__conavgpt2_team` | 79 | 0 | 0.000 | 0.000 | 5.68 km | 1.53 km | 0.0000 |
| Fire / Suburban L2 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_005__firesuburbanl2v1__frontier` | 79 | 15 | 0.190 | 0.129 | 14.11 km | 2.27 km | 0.0046 |
| Fire / Suburban L2 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_006__firesuburbanl2v1_lawnmower__lawnmower` | 79 | 0 | 0.000 | 0.000 | 13.37 km | 2.28 km | 0.0000 |
| Fire / Suburban L2 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_007__firesuburbanl2v1__vlfm` | 79 | 27 | 0.342 | 0.266 | 4.52 km | 2.26 km | 0.0257 |
| Fire / Suburban L3 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_012__firesuburbanl3v1_conavgpt2_team__conavgpt2_team` | 84 | 0 | 0.000 | 0.000 | 6.92 km | 1.89 km | 0.0000 |
| Fire / Suburban L3 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_009__firesuburbanl3v1__frontier` | 84 | 17 | 0.202 | 0.105 | 11.70 km | 2.95 km | 0.0067 |
| Fire / Suburban L3 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_010__firesuburbanl3v1_lawnmower__lawnmower` | 84 | 4 | 0.048 | 0.003 | 14.29 km | 3.00 km | 0.0007 |
| Fire / Suburban L3 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_011__firesuburbanl3v1__vlfm` | 84 | 0 | 0.000 | 0.000 | 4.22 km | 2.92 km | 0.0000 |
| Hurricane / Suburban L1 | CoNavGPT2 | `hurricane_suburban_l1_conavgpt2_gt600/2026-09-04_17-23-22/iter_001__hurricanesuburbanl1v1_conavgpt2_team__conavgpt2_team` | 55 | 0 | 0.000 | 0.000 | 6.47 km | 2.89 km | 0.0000 |
| Hurricane / Suburban L1 | Frontier | `hurricane_suburban_8robot/2026-09-02_20-13-16/iter_001__hurricanesuburbanl1v1__frontier` | 55 | 1 | 0.018 | 0.018 | 5.21 km | 4.88 km | 0.0001 |
| Hurricane / Suburban L1 | Lawnmower | `hurricane_suburban_8robot/2026-09-02_20-13-16/iter_002__hurricanesuburbanl1v1_lawnmower__lawnmower` | 55 | 0 | 0.000 | 0.000 | 8.69 km | 3.66 km | 0.0000 |
| Hurricane / Suburban L1 | VLFM | `hurricane_suburban_8robot/2026-09-02_20-13-16/iter_003__hurricanesuburbanl1v1__vlfm` | 55 | 0 | 0.000 | 0.000 | 0.06 km | 0.00 km | 0.0000 |
| Tornado / Suburban L1 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_016__tornadosuburbanl1v1_conavgpt2_team__conavgpt2_team` | 30 | 0 | 0.000 | 0.000 | 1.83 km | 0.85 km | 0.0000 |
| Tornado / Suburban L1 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_013__tornadosuburbanl1v1__frontier` | 30 | 0 | 0.000 | 0.000 | 10.71 km | 0.94 km | 0.0000 |
| Tornado / Suburban L1 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_014__tornadosuburbanl1v1_lawnmower__lawnmower` | 30 | 2 | 0.067 | 0.017 | 11.66 km | 0.89 km | 0.0002 |
| Tornado / Suburban L1 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_015__tornadosuburbanl1v1__vlfm` | 30 | 0 | 0.000 | 0.000 | 5.08 km | 0.91 km | 0.0000 |
| Tornado / Suburban L2 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_020__tornadosuburbanl2v1_conavgpt2_team__conavgpt2_team` | 40 | 0 | 0.000 | 0.000 | 4.57 km | 0.92 km | 0.0000 |
| Tornado / Suburban L2 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_017__tornadosuburbanl2v1__frontier` | 40 | 4 | 0.100 | 0.068 | 11.38 km | 1.03 km | 0.0016 |
| Tornado / Suburban L2 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_018__tornadosuburbanl2v1_lawnmower__lawnmower` | 40 | 0 | 0.000 | 0.000 | 10.56 km | 1.07 km | 0.0000 |
| Tornado / Suburban L2 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_019__tornadosuburbanl2v1__vlfm` | 40 | 0 | 0.000 | 0.000 | 4.78 km | 1.04 km | 0.0000 |
| Tornado / Suburban L3 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_021__tornadosuburbanl3v1__frontier` | 70 | 2 | 0.029 | 0.018 | 9.78 km | 1.79 km | 0.0006 |
| Tornado / Suburban L3 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_022__tornadosuburbanl3v1_lawnmower__lawnmower` | 70 | 2 | 0.029 | 0.012 | 13.29 km | 1.82 km | 0.0004 |
| Tornado / Suburban L3 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_023__tornadosuburbanl3v1__vlfm` | 70 | 0 | 0.000 | 0.000 | 4.80 km | 1.76 km | 0.0000 |

### Average by baseline

| Method | Completed / total runs | Avg progress | Avg time-integrated progress | Avg actual team path | Avg ideal all-target path | Avg PPL |
|---|---:|---:|---:|---:|---:|---:|
| CoNavGPT2 | 7/48 | 0.000 | 0.000 | 5.64 km | 2.34 km | 0.0000 |
| Frontier | 9/48 | 0.083 | 0.057 | 8.62 km | 3.22 km | 0.0019 |
| Lawnmower | 9/48 | 0.016 | 0.004 | 11.57 km | 3.05 km | 0.0002 |
| VLFM | 9/48 | 0.063 | 0.052 | 3.97 km | 2.65 km | 0.0042 |

### Target-circle radius sensitivity

These rows change only the GT-to-target-circle association radius; the detector gate and target circles are unchanged.

| Method | Runs | 12 m progress | 17 m progress | Gain vs base | 22 m progress | Gain vs base |
|---|---:|---:|---:|---:|---:|---:|
| CoNavGPT2 | 7 | 0.000 | 0.000 | +0.000 | 0.000 | +0.000 |
| Frontier | 9 | 0.083 | 0.106 | +0.023 | 0.130 | +0.047 |
| Lawnmower | 9 | 0.016 | 0.022 | +0.006 | 0.037 | +0.021 |
| VLFM | 9 | 0.063 | 0.073 | +0.010 | 0.082 | +0.019 |

#### Runs with zero detections at 12 m

| Scene | Method | 12 m detected / GT | 17 m detected / GT | 22 m detected / GT |
|---|---|---:|---:|---:|
| Earthquake / Urban L1 | CoNavGPT2 | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L1 | Frontier | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L1 | Lawnmower | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L1 | VLFM | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L3 | CoNavGPT2 | 0/64 | 0/64 | 0/64 |
| Earthquake / Urban L3 | Frontier | 0/64 | 0/64 | 0/64 |
| Earthquake / Urban L3 | Lawnmower | 0/64 | 0/64 | 0/64 |
| Earthquake / Urban L3 | VLFM | 0/64 | 0/64 | 0/64 |
| Fire / Suburban L1 | Lawnmower | 0/49 | 1/49 | 4/49 |
| Fire / Suburban L2 | CoNavGPT2 | 0/79 | 0/79 | 0/79 |
| Fire / Suburban L2 | Lawnmower | 0/79 | 0/79 | 1/79 |
| Fire / Suburban L3 | CoNavGPT2 | 0/84 | 0/84 | 0/84 |
| Fire / Suburban L3 | VLFM | 0/84 | 0/84 | 0/84 |
| Hurricane / Suburban L1 | CoNavGPT2 | 0/55 | 0/55 | 0/55 |
| Hurricane / Suburban L1 | Lawnmower | 0/55 | 0/55 | 0/55 |
| Hurricane / Suburban L1 | VLFM | 0/55 | 0/55 | 0/55 |
| Tornado / Suburban L1 | CoNavGPT2 | 0/30 | 0/30 | 0/30 |
| Tornado / Suburban L1 | Frontier | 0/30 | 0/30 | 1/30 |
| Tornado / Suburban L1 | VLFM | 0/30 | 2/30 | 2/30 |
| Tornado / Suburban L2 | CoNavGPT2 | 0/40 | 0/40 | 0/40 |
| Tornado / Suburban L2 | Lawnmower | 0/40 | 0/40 | 0/40 |
| Tornado / Suburban L2 | VLFM | 0/40 | 0/40 | 0/40 |
| Tornado / Suburban L3 | VLFM | 0/70 | 0/70 | 0/70 |

Breakdown opportunities count each GT victim once per completed run/method; the same frozen-scene victim is therefore one opportunity for each baseline that searched that scene.

### Detection breakdown by pose

| Pose | Detected / opportunities | Detection rate |
|---|---:|---:|
| crouched | 0/23 | 0.000 |
| lying | 10/490 | 0.020 |
| seated | 38/391 | 0.097 |
| unknown | 20/541 | 0.037 |
| upright | 27/476 | 0.057 |

### Detection breakdown by visibility

| Visibility | Detected / opportunities | Detection rate |
|---|---:|---:|
| full | 7/377 | 0.019 |
| partial | 4/333 | 0.012 |
| unknown | 84/1211 | 0.069 |

### Detection breakdown by occlusion

| Occlusion | Detected / opportunities | Detection rate |
|---|---:|---:|
| banded | 0/22 | 0.000 |
| feet_shins | 4/88 | 0.045 |
| flank | 0/20 | 0.000 |
| legs | 0/51 | 0.000 |
| midriff | 0/26 | 0.000 |
| none | 7/377 | 0.019 |
| submerged | 0/60 | 0.000 |
| torso | 0/37 | 0.000 |
| torso_head | 0/29 | 0.000 |
| unknown | 84/1211 | 0.069 |

### Detection breakdown by environment

| Environment | Detected / opportunities | Detection rate |
|---|---:|---:|
| Earthquake / unknown | 0/412 | 0.000 |
| Fire / at_home | 0/8 | 0.000 |
| Fire / cul_de_sac | 27/165 | 0.164 |
| Fire / gridlock | 4/216 | 0.019 |
| Fire / open_ground | 21/143 | 0.147 |
| Fire / parking_refuge | 32/224 | 0.143 |
| Fire / pools | 0/43 | 0.000 |
| Hurricane / roof | 1/160 | 0.006 |
| Hurricane / water | 0/60 | 0.000 |
| Tornado / pile | 1/141 | 0.007 |
| Tornado / skirt | 3/127 | 0.024 |
| Tornado / street | 1/78 | 0.013 |
| Tornado / trail | 0/4 | 0.000 |
| Tornado / yard | 5/140 | 0.036 |
