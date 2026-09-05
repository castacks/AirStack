# Disaster benchmark dashboard

Last reconciled against `/media/share/coa-sei`: **2026-09-04**.

> **Legend:** 🟩 **DONE** · 🟦 **READY** · 🟧 **RERUN** · 🟨 **VERIFY / IN PROGRESS** · ⬜ **NOT READY**
>
> 🟩 means a usable passed benchmark result exists. Pre-optimization results
> count; failed or stopped attempts never count as DONE. 🟦 means the scene is
> ready but the method has not run. 🟧 means the method failed or stopped and
> needs a rerun. ⬜ means the scene itself is not ready.

## At-a-glance matrix

| Disaster | Locale | Level | Scene ready | Frontier | Lawnmower | VLFM | CoNavGPT2 | RayFronts/RAVEN |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **Fire** | **Urban** | L1 | 🟧 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Fire** | **Urban** | L2 | 🟧 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Fire** | **Urban** | L3 | 🟨 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Fire** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Urban** | L1–L3 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Hurricane** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Suburban** | L3 | 🟩 | 🟩 | 🟧 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Urban** | L1–L3 | 🟨 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Tornado** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Urban** | L1–L3 | 🟨 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Earthquake** | **Suburban** | L1–L3 | 🟨 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |

## Fire

### Urban

| Level | What is done | What is left | Intended run folder |
|---:|---|---|---|
| L1 | Frozen scene is ready | Four baselines + RayFronts | `urban_fire_8robot/<timestamp>` |
| L2 | Frozen scene is ready | Four baselines + RayFronts | `urban_fire_8robot/<timestamp>` |
| L3 | 47/47 bakes complete and normalized on `airstack-dev-198`; RTX OOM was mitigated, then the content gate found 24/85 manifest records drifting from the live Kit layout | Regenerate/reconcile the authoritative Kit dump and fire manifest, then freeze, upload and cold-verify | `urban_fire_8robot/<timestamp>` |

No completed Urban Fire benchmark result is currently present under
`/media/share/coa-sei`. On 2026-09-04 both L1 and L2 resolved their expected
Nucleus filenames, but `Usd.Stage` could not open either published file. Both
preflight attempts were stopped before scoring and were not uploaded. The
cells must be re-exported/re-uploaded and cold-open verified before rerunning.

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
| L3 | Frontier, VLFM, CoNavGPT2 | Lawnmower rerun; RayFronts/RAVEN | `hurricane_suburban_l3_8robot_optimized_batch/2026-09-04_22-56-30` |

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

Urban Earthquake L1–L3 generation is complete and frozen-scene export is in
progress with another agent. No benchmark result has been submitted yet.

### Suburban

Suburban Earthquake L1–L3 generation is complete and frozen-scene export is in
progress with another agent. No benchmark result has been submitted yet.

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

1. 🟨 Finish the Hurricane/Suburban L3 lawnmower rerun on pod 57; its hardened
   pre-arm/takeoff step has passed and the mission is entering the search.
2. 🟧 Re-export/re-upload and cold-open verify Urban Fire L1 and L2 on Nucleus.
   Their current filenames resolve, but both USD stages fail to open.
3. 🟩 Fire/Suburban L1 and Tornado/Suburban L3 CoNavGPT2 focused reruns passed.
4. 🟦 Run the queued 2-GPU RayFronts/RAVEN suburban missions.
5. 🟦 Run the four shared-planner baselines on Urban Fire L1 and L2 after the
   Nucleus cold-open gate passes.
6. 🟨 Finish and publish Urban Fire L3, then run its methods.

## Active batch plan

Only the four shared-planner baselines are assigned to the existing 1-GPU
pods. Urban cells and RayFronts/RAVEN remain in the 2-GPU queue. Each listed
batch is capped below 12 wall-hours; a second identical failure pauses that
cell/method for diagnosis rather than consuming the rest of the batch.

| Order | Pod | Cells | Runs | Expected batch wall time | State |
|---:|---|---|---:|---:|---|
| 1 | `airstack-mission-1gpu-56` | Fire/Suburban L1 CoNavGPT2 | 1 | ~45 min | PASSED — focused rerun completed before the L2 batch |
| 1 | `airstack-mission-1gpu-57` | Tornado/Suburban L3 CoNavGPT2 | 1 | ~45 min | PASSED — focused rerun completed before the L3 batch |
| 2 | `airstack-mission-1gpu-56` | Hurricane/Suburban L2 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | COMPLETE — 4/4 passed, uploaded and verified |
| 2 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | COMPLETE — 3/4 passed and uploaded; lawnmower failed twice and was not uploaded |
| 3 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 lawnmower | 1 | ~1 h (4 h hard cap) | RUNNING — readiness and hardened takeoff passed |
| 3 | `airstack-mission-1gpu-56` | Urban Fire L1/L2 | 8 | <12 h | BLOCKED — L1 and L2 Nucleus USDs resolve but fail to open; zero scored/uploaded runs |

RayFronts/RAVEN is split into three 2-GPU workflows. Each runs three scene
levels under a 12-hour mission cap, while the pod itself remains alive for 48
hours for inspection and corrective reruns.

| Workflow | Cells | State |
|---|---|---|
| `airstack-mission-8robot-2gpu-2` | Fire/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-3` | Hurricane/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-4` | Tornado/Suburban L1–L3 RayFronts | QUEUED |

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
| Hurricane | — | 31.3 min / 0.319 (n=1) |
| Tornado | — | 77.4 min / 0.129 (n=11) |
| Earthquake | — | — |

## Actual results (detector-confirmed team progress and PPL)

A GT victim counts as detected when its world-frame XY location falls inside a **12 m circle around a planner `search_target`** during the 600-s search. A target circle exists only after a `person` detection clears the shared 0.65 confidence gate, is depth-projected, and forms a clustered target instance. One liberal circle can credit multiple GT people; drone proximity alone never counts. Time-integrated progress is normalized area under the cumulative detector-confirmed progress curve; marker chunks were sampled at about 20-s intervals (final persistent target state is always read, so final detection counts are exact). Paths are 1 Hz, world-frame XY odometry. Ideal lengths are OR-Tools oracle estimates for open Euclidean multi-depot routes through victim centres; fixed-sector methods preserve recorded robot ownership, while CoNavGPT2 permits joint assignment. Ground debris does not obstruct an aerial XY geodesic, and no return to launch is required. PPL uses the ideal route through detected GT victims: `progress × ideal_detected / max(actual, ideal_detected)`.

### Per completed run

| Scene | Method | Run folder | GT | Detected | Detector-confirmed progress | Time-integrated progress | Actual team path | Ideal all-target path | PPL |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
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
| CoNavGPT2 | 5/48 | 0.000 | 0.000 | 5.09 km | 1.62 km | 0.0000 |
| Frontier | 7/48 | 0.106 | 0.073 | 9.78 km | 2.23 km | 0.0024 |
| Lawnmower | 7/48 | 0.020 | 0.005 | 12.34 km | 2.07 km | 0.0002 |
| VLFM | 7/48 | 0.081 | 0.067 | 4.04 km | 1.51 km | 0.0055 |

### Target-circle radius sensitivity

These rows change only the GT-to-target-circle association radius; the detector gate and target circles are unchanged.

| Method | Runs | 12 m progress | 17 m progress | Gain vs base | 22 m progress | Gain vs base |
|---|---:|---:|---:|---:|---:|---:|
| CoNavGPT2 | 5 | 0.000 | 0.000 | +0.000 | 0.000 | +0.000 |
| Frontier | 7 | 0.106 | 0.136 | +0.030 | 0.167 | +0.061 |
| Lawnmower | 7 | 0.020 | 0.028 | +0.008 | 0.047 | +0.027 |
| VLFM | 7 | 0.081 | 0.094 | +0.013 | 0.105 | +0.024 |

#### Runs with zero detections at 12 m

| Scene | Method | 12 m detected / GT | 17 m detected / GT | 22 m detected / GT |
|---|---|---:|---:|---:|
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
| unknown | 20/129 | 0.155 |
| upright | 27/476 | 0.057 |

### Detection breakdown by visibility

| Visibility | Detected / opportunities | Detection rate |
|---|---:|---:|
| full | 7/377 | 0.019 |
| partial | 4/333 | 0.012 |
| unknown | 84/799 | 0.105 |

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
| unknown | 84/799 | 0.105 |

### Detection breakdown by environment

| Environment | Detected / opportunities | Detection rate |
|---|---:|---:|
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
