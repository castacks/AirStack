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
| **Fire** | **Urban** | L1 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Fire** | **Urban** | L2 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Fire** | **Urban** | L3 | 🟨 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Fire** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟧 | 🟦 |
| **Fire** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Urban** | L1–L3 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Hurricane** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Suburban** | L2 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Hurricane** | **Suburban** | L3 | 🟩 | 🟦 | 🟦 | 🟦 | 🟦 | 🟦 |
| **Tornado** | **Urban** | L1–L3 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Tornado** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Tornado** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 | 🟦 |
| **Earthquake** | **Urban** | L1–L3 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |
| **Earthquake** | **Suburban** | L1–L3 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ |

## Fire

### Urban

| Level | What is done | What is left | Intended run folder |
|---:|---|---|---|
| L1 | Frozen scene is ready | Four baselines + RayFronts | `urban_fire_8robot/<timestamp>` |
| L2 | Frozen scene is ready | Four baselines + RayFronts | `urban_fire_8robot/<timestamp>` |
| L3 | 47/47 bakes complete and normalized on `airstack-dev-198`; RTX OOM was mitigated, then the content gate found 24/85 manifest records drifting from the live Kit layout | Regenerate/reconcile the authoritative Kit dump and fire manifest, then freeze, upload and cold-verify | `urban_fire_8robot/<timestamp>` |

No completed Urban Fire benchmark result is currently present under
`/media/share/coa-sei`.

### Suburban

The accepted pre-optimization 8-robot sweep is
`frozen_suburban_8robot/2026-08-31_11-11-42`. Frontier, lawnmower and VLFM
passed at L1–L3; CoNavGPT2 passed at L2–L3. Fire L1 CoNavGPT2 failed and is the
only shared-planner Fire/Suburban cell that needs a rerun.

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
| L2 | — | All five methods | — |
| L3 | — | All five methods | — |

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

🟥 No completed Urban Tornado scenes or benchmark results are recorded.

### Suburban

The accepted pre-optimization 8-robot sweep is
`frozen_suburban_8robot/2026-08-31_11-11-42`. All four shared-planner methods
passed at L1–L2. Frontier, lawnmower and VLFM passed at L3; L3 CoNavGPT2 was not
run and is READY. A failed L3 lawnmower attempt is superseded by its passed
canonical iteration and does not change the DONE status.

## Earthquake

### Urban

🟥 No completed Urban Earthquake scene/benchmark matrix is recorded.

### Suburban

🟥 No completed Suburban Earthquake scene/benchmark matrix is recorded.

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

1. 🟧 Rerun Fire/Suburban L1 CoNavGPT2 on pod 56.
2. 🟦 Run Tornado/Suburban L3 CoNavGPT2 on pod 57.
3. 🟦 Split Hurricane/Suburban L2 and L3 shared-planner methods across pods 56
   and 57 after those focused runs finish.
4. 🟦 Run the queued 2-GPU RayFronts/RAVEN suburban missions.
5. 🟦 Run the four shared-planner baselines on Urban Fire L1 and L2.
6. 🟨 Finish and publish Urban Fire L3, then run its methods.

## Active batch plan

Only the four shared-planner baselines are assigned to the existing 1-GPU
pods. Urban cells and RayFronts/RAVEN remain in the 2-GPU queue. Each listed
batch is capped below 12 wall-hours; a second identical failure pauses that
cell/method for diagnosis rather than consuming the rest of the batch.

| Order | Pod | Cells | Runs | Expected batch wall time | State |
|---:|---|---|---:|---:|---|
| 1 | `airstack-mission-1gpu-56` | Fire/Suburban L1 CoNavGPT2 | 1 | ~45 min | RUNNING — `fire_suburban_l1_conavgpt2_optimized_rerun/2026-09-04_20-34-18` |
| 1 | `airstack-mission-1gpu-57` | Tornado/Suburban L3 CoNavGPT2 | 1 | ~45 min | RUNNING — `tornado_suburban_l3_conavgpt2_optimized/2026-09-04_20-34-17` |
| 2 | `airstack-mission-1gpu-56` | Hurricane/Suburban L2 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | QUEUED |
| 2 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | QUEUED |

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
