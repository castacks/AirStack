# Disaster benchmark dashboard

Last reconciled against `/media/share/coa-sei`: **2026-09-04**.

> **Legend:** 🟩 **DONE** · 🟨 **PARTIAL / VERIFY** · 🟥 **LEFT** · ⬜ **NO RESULT EXPECTED YET**
>
> A method is 🟩 only when an uploaded, official **8-robot / 600-s** run reports
> `status: passed`. Older 1- and 5-robot development runs are listed separately
> and do not make a benchmark cell complete.

## At-a-glance matrix

| Disaster | Locale | Level | Scene ready | Frontier | Lawnmower | VLFM | CoNavGPT2 | RayFronts/RAVEN |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **Fire** | **Urban** | L1 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Fire** | **Urban** | L2 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Fire** | **Urban** | L3 | 🟨 FREEZE RETRY NEEDED | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Fire** | **Suburban** | L1 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Fire** | **Suburban** | L2 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Fire** | **Suburban** | L3 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Hurricane** | **Urban** | L1–L3 | ⬜ | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Hurricane** | **Suburban** | L1 | 🟩 DONE | 🟩 DONE | 🟩 DONE | 🟩 DONE | 🟩 DONE | 🟥 LEFT |
| **Hurricane** | **Suburban** | L2 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Hurricane** | **Suburban** | L3 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Tornado** | **Urban** | L1–L3 | ⬜ | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Tornado** | **Suburban** | L1 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Tornado** | **Suburban** | L2 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Tornado** | **Suburban** | L3 | 🟩 DONE | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Earthquake** | **Urban** | L1–L3 | ⬜ | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |
| **Earthquake** | **Suburban** | L1–L3 | ⬜ | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT | 🟥 LEFT |

## Fire

### Urban

| Level | What is done | What is left | Intended run folder |
|---:|---|---|---|
| L1 | Frozen scene is ready | Four baselines + RayFronts | `urban_fire_8robot/<timestamp>` |
| L2 | Frozen scene is ready | Four baselines + RayFronts | `urban_fire_8robot/<timestamp>` |
| L3 | 47/47 bakes complete and normalized on `airstack-dev-198`; first combined freeze attempt hit RTX OOM before export | Retry with reduced RTX footprint, upload, cold verification, four baselines, RayFronts | `urban_fire_8robot/<timestamp>` |

No completed Urban Fire benchmark result is currently present under
`/media/share/coa-sei`.

### Suburban

The frozen L1–L3 scenes exist, but no uploaded official 8-robot summary was
found. The empty `frozen_suburban_8robot*` upload directories are not evidence
of completion.

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

Frozen L1–L3 scenes exist. No uploaded official 8-robot method summary was
found, so all five methods remain 🟥 **LEFT** at every level.

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

1. 🟥 Run the four 8-robot / 600-s baselines on Urban Fire L1 and L2.
2. 🟥 Run 2-GPU RayFronts/RAVEN on Urban Fire L1 and L2.
3. 🟥 Finish and publish Urban Fire L3 later; then repeat the five methods.
4. 🟥 Run RayFronts/RAVEN for Hurricane/Suburban L1.
5. 🟥 Complete Hurricane/Suburban L2–L3 and all Fire/Tornado Suburban cells.

## Active batch plan

Only the four shared-planner baselines are assigned to the existing 1-GPU
pods. Urban cells and RayFronts/RAVEN remain in the 2-GPU queue. Each listed
batch is capped below 12 wall-hours; a second identical failure pauses that
cell/method for diagnosis rather than consuming the rest of the batch.

| Order | Pod | Cells | Runs | Expected batch wall time | State |
|---:|---|---|---:|---:|---|
| 1 | `airstack-mission-1gpu-56` | Fire/Suburban L1–L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 12 | ~9 h (12 h hard cap) | RUNNING — `fire_suburban_8robot_optimized_batch/2026-09-04_19-51-50` |
| 1 | `airstack-mission-1gpu-57` | Tornado/Suburban L1–L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 12 | ~9 h (12 h hard cap) | RUNNING — `tornado_suburban_8robot_optimized_batch/2026-09-04_19-51-50` |
| 2 | first healthy/free pod | Hurricane/Suburban L2–L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 8 | ~6 h (12 h hard cap) | QUEUED |

RayFronts/RAVEN is split into three 2-GPU workflows. Each runs three scene
levels under a 12-hour mission cap, while the pod itself remains alive for 48
hours for inspection and corrective reruns.

| Workflow | Cells | State |
|---|---|---|
| `airstack-mission-8robot-2gpu-2` | Fire/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-3` | Hurricane/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-4` | Tornado/Suburban L1–L3 RayFronts | QUEUED |

## Average 600-s scene performance

Wall clock is the timed 600-s search window; RTF is simulated seconds divided
by that wall clock. Values average across successful optimized runs in each
scene family, independent of baseline. Failed and short tuning runs are
excluded.

| Disaster | Urban avg wall / RTF | Suburban avg wall / RTF |
|---|---:|---:|
| Fire | — | — |
| Hurricane | — | 31.3 min / 0.319 (n=1) |
| Tornado | — | — |
| Earthquake | — | — |
