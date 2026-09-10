# Disaster benchmark dashboard

Latest completion update: **2026-09-09**. Earthquake Suburban L1 RAVEN passed,
was checksum-verified on NAS, and had **79.39 GB** of recoverable local
artifacts pruned. Pod 3 expired before Tornado Suburban L3 completed; replacement
pod 6 resumed that cell at 19:10 UTC. Pod 5 resumed lane B at Earthquake
Suburban L3 at 19:05 UTC. Failed and interrupted attempts remain local and
unuploaded.

> **Legend:** 🟩 **DONE** · 🟦 **READY** · 🟧 **RERUN** · 🟨 **VERIFY / IN PROGRESS** · ⬜ **NOT READY**
>
> 🟩 means a usable passed benchmark result exists. Pre-optimization results
> count; failed or stopped attempts never count as DONE. 🟦 means the scene is
> ready but the method has not run. 🟧 means the method failed or stopped and
> needs a rerun. ⬜ means the scene itself is not ready.

## At-a-glance matrix

September 10 native profiling: five independent main-thread samples on the
active Earthquake Suburban L3 Lawnmower run all landed in
`libomni.fabric.plugin.so`; selective symbol loading exposed a caller in
`libusdrt.hierarchy.plugin.so`. This localizes the sampled CPU hotspot to
Hydra/Fabric scene hierarchy work, not proven PhysX or detector cost. Memory
pressure was effectively zero and I/O pressure below 0.5% in the sampled window.
Brief native stack captures added diagnostic wall-time overhead without
changing scene settings. A fourth matched 50-s candidate now tests the legacy
USD scene delegate with the same 8 groups, burst 8, CPU physics, camera geometry
and flight gates. It is opt-in and **not yet validated for production**.

September 9, 20:08 UTC pod-58 performance diagnostic: the active Earthquake
Suburban L3 Frontier search measures **RTF 0.04876** from a continuous domain-0
clock callback (4.41 simulated seconds / 90.44 wall seconds), corroborated by
domain 1 at **0.04919** (2.22 / 45.13). At this rate a 600-s search takes
approximately **3 h 25 min**, excluding startup and upload, not the 50–67 min
implied by RTF 0.15–0.2. This iteration started 17:38:46 and reached search at
18:09:55; all eight takeoffs passed, robot 2 on its second action attempt, with
no full-iteration retry. Isaac and detector UUIDs match the pod's assigned GPU
(`17194e29-090d-757c-9af0-2b3687c5fee8`), with 12.8 GB VRAM in use. Sampled GPU
utilization was 0–6%, Isaac's main thread approximately 92% CPU, and its cgroup
showed no CPU throttling. These observations suggest a CPU-side bottleneck;
they do not yet establish its exact cause. Camera settings remain 0 empty
groups / burst 8. Investigation of the three skipped Tornado failures and
corrected rerun scheduling is underway; spawn changes require collision or
clearance evidence rather than treating every pre-arm timeout as a spawn bug.

| Disaster | Locale | Level | Scene ready | Frontier | Lawnmower | VLFM | CoNavGPT2 | RayFronts/RAVEN |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **Fire** | **Urban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Urban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Urban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Fire** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Fire** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Fire** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Hurricane** | **Urban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Urban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Urban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Hurricane** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Hurricane** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Hurricane** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Tornado** | **Urban** | L1 | 🟩 | 🟩 | 🟩 | 🟧 | 🟩 | 🟨 |
| **Tornado** | **Urban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟧 | 🟨 |
| **Tornado** | **Urban** | L3 | 🟩 | 🟩 | 🟧 | 🟩 | 🟩 | 🟨 |
| **Tornado** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Tornado** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Tornado** | **Suburban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟨 |
| **Earthquake** | **Urban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Urban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Urban** | L3 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Suburban** | L1 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 |
| **Earthquake** | **Suburban** | L2 | 🟩 | 🟩 | 🟩 | 🟩 | 🟩 | 🟦 |
| **Earthquake** | **Suburban** | L3 | 🟩 | 🟩 | 🟨 | 🟦 | 🟦 | 🟨 |

### Accepted completion counts — September 9, 23:35 UTC

Passed and uploaded unique 8-robot scene/method cells only; failures, active runs,
and duplicate reruns are excluded. These completion counts can precede analysis.

| Method | Completed / planned |
|---|---:|
| Frontier | 24 / 24 |
| Lawnmower | 22 / 24 |
| VLFM | 22 / 24 |
| CoNavGPT2 | 22 / 24 |
| RAVEN / RayFronts | 9 / 24 |
| **Total** | **99 / 120** |

Pod 58 has uploaded 26/32 unique cells in its remaining-work queue and is now
running Earthquake Suburban L3 Lawnmower. Frontier passed at whole-step RTF
approximately 0.04370 (600 / 13730.12 wall seconds), was checksum-verified at
22:03:26 UTC, and had 15.99 GB pruned locally. The three Tornado reruns were
corrected and appended by commit `abb2564b`: fresh pre-arm safety gating and
900-second takeoff feedback, without unsupported spawn changes. The exact
CPU-side cause of the low RTF remains unresolved. Earthquake Suburban L2 CoNavGPT2 was uploaded
at 17:38:46 UTC. Pod 5 Earthquake Suburban L1 RAVEN passed and was NAS
checksum-verified at 16:39:43 UTC; 79.39 GB of local artifacts were pruned.
The user assigned replacement 2-GPU pod 6 for the timed-out RAVEN lane. Its
launcher PID 60 is held, assigned GPU UUIDs map to nested Isaac index 0 and
offboard index 2, the RADSeg cache passed a network-disabled load test, and
Tornado Suburban L3 started at 19:10:03 UTC. Pod 5 resumed at Earthquake
Suburban L3 at 19:05:02 UTC with its verified GPU 0/1 split. This snapshot
supersedes older active-queue descriptions below.

## Overnight completion summary — September 7

### Pod storage retention

After each accepted production run: upload to NAS, checksum-compare every local
artifact against the remote destination, then remove the local artifacts.
Retain only tiny `iteration.json` and `upload_receipt.json` bookkeeping files
so completed cells are not rerun. Verification errors preserve all local files.
This policy no longer depends on host disk free space: pod ephemeral quotas
can be exhausted while the host reports terabytes free. Runtime fix `7acbb11f`
is being deployed at safe process boundaries; already-running processes need
explicit post-upload verification/cleanup. Archived results remain recoverable
from `/media/share/coa-sei`.

September 9 cleanup audit: pod 58 rechecked 25 inactive accepted production
iterations against NAS with checksum-mode rsync, then pruned **128.65 GB** of
local artifacts while retaining 25 completion/upload receipts. The camera-rate
diagnostic, every failed attempt, and all active iterations were excluded.
Pods 3 and 5 had no inactive accepted production artifacts left locally.

### Active completion queue — September 8

| Pod | Current run | Remaining assignment |
|---|---|---|
| dev 191 | Expired: `FAILED_EXEC_TIMEOUT` | No active run |
| 1-GPU 58 | Earthquake Suburban L1 VLFM **passed and NAS-verified** at 23:16:49 UTC (team RTF **0.04825**); audited-missing Fire Suburban L1 CoNavGPT2 attempt 2 passed 8/8 PX4/perception/takeoff gates and entered team search at 23:56:24 UTC | Live persistent-clock RTF **0.04701** over 90.62 wall s / 4.26 sim s. Attempt 1 failed before flight on robot 7's transient mono-RGBD cloud gate and was not uploaded. **20/32 uploaded; 12 outstanding** including the active rerun, next-priority Tornado Suburban L3 CoNav rerun, remaining Earthquake cells, and three Tornado Urban failures needing investigation |
| 2-GPU 1 | `FAILED_EVICTED`, confirmed by OSMO; SSH endpoint gone | RayFronts queue cannot advance on this pod; replacement required |
| 2-GPU 3 | `FAILED_EXEC_TIMEOUT`; Fire Suburban L1/L3, Hurricane Suburban L2 and Tornado Suburban L1 RAVEN remain **passed, uploaded, checksum-verified and locally pruned** | Tornado Suburban L3 did not finish before the pod expired and was never uploaded. Its replacement is active on 2-GPU 6. |
| 2-GPU 4 | Fire Suburban L2 and Hurricane Suburban L1 RAVEN both **passed, uploaded and NAS-verified**; Hurricane Suburban L3 needs a replacement rerun | Kubernetes evicted the workspace at **22:37:33 UTC** for exceeding the **400 GiB ephemeral-storage limit**, with the valid L3 search only **416–427/600 s** complete. The interrupted partial run is invalid and was not uploaded. This was infrastructure storage exhaustion, not a simulator or autonomy failure. L3 is orange until rerun on a replacement 2-GPU pod |
| 2-GPU 5 | Hurricane Suburban L3, Tornado Suburban L2 and Earthquake Suburban L1 RAVEN are **passed, uploaded, checksum-verified and locally pruned**; Earthquake Suburban L3 started at **19:05:02 UTC** under `raven_earthquakesuburbanl3v1_raven_remaining_2gpu1/2026-09-09_19-05-02` | Earthquake L1's NAS copy was verified before 79.39 GB was pruned. Lane B continues with zero empty camera groups, burst 8, renderer GPU 0 and offboard GPU 1. Launcher PID 61 remains T. |
| 2-GPU 6 | Replacement RAVEN lane A: Tornado Suburban L3 started at **19:10:03 UTC** under `raven_tornadosuburbanl3v1_raven_remaining_2gpu1/2026-09-09_19-10-03` | Launcher PID 60 is held T. Assigned UUIDs `GPU-11c974b2...` and `GPU-d239ce7e...` map to nested indices 0 and 2. RADSeg weights were warmed and passed an offline/network-disabled load. The unrelated auto-started Urban Fire baseline was interrupted, retained locally and not uploaded. |

September 8 04:05 UTC: user assigned pods 3/4 to RAVEN and reaffirmed pod 58
for non-RAVEN work. Both RAVEN pods were already protected and running the
current runtime fixes (`9017b418`, RayFronts `128d95e0`); fast-forwarded their
remaining documentation updates to `f0eed04d` without restarting healthy runs.
Those pre-sweep active containers confirmed 12/8 cameras. Live offboard CUDA processes mapped
to the assigned model GPU UUIDs, and the large Isaac allocation maps to each
assigned renderer UUID. Both RAVEN logs are advancing. These are active runs,
not accepted completions. The eviction of pod 1 does not stop pods 3/4.

Pod 58 now has a detached, durable one-cell-at-a-time queue, not just a list
waiting for manual launches. Each cell retains the original 600-s window
and flight gates, with renderer/offboard pinned to owned GPU 2. The active Fire
run retains 32/8 cameras; subsequent runs use the user-approved 12/8 cohort
described below. The Fire completion gate now uses a persistent reliable, transient-local
subscriber for `run_complete=true`, replacing repeated ROS CLI processes that
previously hung. It does **not** accept elapsed wall time or proximity as success.

Every cell has a ≤12-hour launch/cleanup cap and two mission attempts. Exhausted
failures are marked `needs_investigation` while other ready work proceeds; they
are not counted or uploaded. Passing results upload immediately, with a further
upload verification/retry before advancing. An unresolved upload failure stops
the queue and retains the local result. Both mission pods' original launchers
remain stopped to prevent automatic teardown.

Live pod-58 ledger: `/root/AirStack/osmo/results/remaining58_queue/state.json`;
per-cell logs are alongside it. Queue source:
`osmo/workspace/remaining_pod58_queue.py`. Current run folder:
`remaining58_tornadourbanl1v1_lawnmower/2026-09-08_03-05-21`.
This section supersedes historical pod-56/57 assignments below. Queued or
currently flying cells are **not new completions** until their acceptance gates pass.

### Camera schedule change — September 7, user approved

**September 8: controlled RAVEN scheduling sweep requested.** Pod 3 is reserved
for 50-sim-second Fire Suburban L1 diagnostics with 4/8/16/24 empty camera
groups (12/16/24/32 total), keeping eight-update bursts. Pod 4 ran the 8-empty
and zero-empty diagnostics in parallel while both RAVEN production lanes were
paused; pod 58 continued non-RAVEN production. The interrupted pod-3
600-second attempt does not count as completed. Diagnostic missions have no
NAS destination and never enter the accepted-run totals.

Selection rule: among valid eight-robot diagnostics, choose the highest measured
RTF with **at least 2.5 simulated FPS for both RGB and depth on every robot**.
Report maximum frame silence separately; never substitute the within-burst rate
for sustained FPS. If none qualifies, test zero empty groups as an additional
control before choosing a setting. Keep scene, spawns, stereo geometry, LiDAR,
model configuration and GPU pins unchanged throughout the comparison.

| Empty groups | Total groups | Burst updates | Search budget | Min RGB/depth sim FPS | RTF | Maximum frame silence | Status |
|---:|---:|---:|---:|---:|---:|---:|---|
| 0 | 8 | 8 | 50 s | 3.12 / 3.12 | 0.04695 | 1.77 s | Measured on 2-GPU 4; **passes** the 2.5-FPS gate and is the current sole qualifier; `diagnostic_raven_empty0_50sim/2026-09-08_08-30-40`. The 08:12 attempt failed before measurement and was excluded/not uploaded. |
| 4 | 12 | 8 | 50 s | 2.04 / 2.04 | 0.03799 | 2.73 s | Measured on 2-GPU 3; fails the 2.5-FPS gate; `diagnostic_raven_empty4_50sim/2026-09-08_05-27-57` |
| 8 | 16 | 8 | 50 s | 1.56 / 1.56 | 0.04312 | 3.69 s | Measured on 2-GPU 4; fails the 2.5-FPS gate; `diagnostic_raven_empty8_50sim/2026-09-08_07-09-49`. The first attempt never started RAVEN (6/8 frame readiness); the 06:42 retry failed 6/8 takeoff; both are excluded and neither was uploaded. |
| 16 | 24 | 8 | 50 s | 0.96 / 0.96 | 0.04379 | 5.61 s | Measured on 2-GPU 3; fails the 2.5-FPS gate; `diagnostic_raven_empty16_50sim/2026-09-08_06-41-04` |
| 24 | 32 | 8 | 50 s | 0.72 / 0.72 | 0.04207 | 7.53 s | Measured on 2-GPU 3; fails the 2.5-FPS gate; `diagnostic_raven_empty24_50sim/2026-09-08_08-21-39`. The 08:00 attempt failed arm-state confirmation before measurement and was excluded/not uploaded. |

**Selected production setting:** zero empty groups (8 total groups), burst 8.
It was the only setting to pass the all-robot RGB/depth gate. Commit `0819ae3d`
deploys it to every remaining RAVEN 2-GPU mission while preserving stereo
resolution, LiDAR, RAVEN parameters, scene geometry and per-pod GPU pins.

### Accepted RAVEN 600-second results

| Scene | Status | All robot windows | Final RTF | GT matched / total | Recall | Raw detections | Team path | Run folder |
|---|---|---:|---:|---:|---:|---:|---:|---|
| Fire Suburban L1 | Passed; uploaded and verified | 8/8, 600.0–600.1 s | 0.0398 | 12/49 | 24.5% | 224 | 5.66 km | `raven_firesuburbanl1v1_raven_remaining_2gpu1/2026-09-08_09-58-54` |
| Fire Suburban L2 | Passed; uploaded and verified | 8/8, 600.0–600.1 s | 0.0443 | 6/79 | 7.6% | 490 | 8.28 km | `raven_firesuburbanl2v1_raven_remaining_2gpu1/2026-09-08_09-56-33` |
| Hurricane Suburban L1 | Passed; uploaded and verified | 8/8, 599.4–600.1 s | 0.0969 | 0/55 | 0.0% | 250 | 4.66 km | `raven_hurricanesuburbanl1v1_raven_remaining_2gpu1/2026-09-08_17-46-57` |
| Fire Suburban L3 | Passed; uploaded and verified | 8/8, 600.0–600.4 s | 0.0457 | 0/84 | 0.0% | 421 | 6.62 km | `raven_firesuburbanl3v1_raven_remaining_2gpu1/2026-09-08_17-41-24` |
| Hurricane Suburban L2 | Passed; uploaded, checksum-verified and pruned | 8/8, 593.5–597.5 s | 0.0907 | 4/66 | 6.1% | 803 | 8.63 km | `raven_hurricanesuburbanl2v1_raven_remaining_2gpu1/2026-09-09_03-09-11` |
| Hurricane Suburban L3 | Passed; uploaded, checksum-verified and pruned | 8/8, 598.5–599.7 s | 0.1038 | 0/63 | 0.0% | 30 | 0.05 km | `raven_hurricanesuburbanl3v1_raven_remaining_2gpu1/2026-09-09_03-20-27` |
| Tornado Suburban L2 | Passed; uploaded, checksum-verified and pruned | 8/8, 598.5–599.9 s | 0.0446 | 0/40 | 0.0% | 18 | 0.05 km | `raven_tornadosuburbanl2v1_raven_remaining_2gpu1/2026-09-09_05-56-25` |
| Tornado Suburban L1 | Passed; uploaded, checksum-verified and pruned | 8/8, 597.1–600.1 s | 0.0350 | 4/30 | 13.3% | 336 | 6.32 km | `raven_tornadosuburbanl1v1_raven_remaining_2gpu1/2026-09-09_06-39-21` |

The optional post-search land step was rejected in both cells after the complete
600-second search results had already been written; it does not invalidate the
required search windows. L2's NAS copy contains the passed `iteration.json`,
81 GiB of bags and all 11 collected RAVEN result files.

Probe windows start on the first robot-domain clock callback after that robot's
fresh RAVEN log appears. FPS uses the entire 50-sim-second observation window,
including empty intervals; RTF uses paired monotonic clock callbacks. Start/end
wall timestamps allow matching resource samples to the measured window. These
short, freshly initialized runs do not establish long-run map-growth performance.

September 8, after the RAVEN sweep, the user approved the winner for **all
remaining baselines as well as RayFronts**:
`ZED_TIME_SLICE_GROUPS=8`, `ZED_TIME_SLICE_BURST=8`, and
`ZED_HYDRA_TIME_SLICE=true`: eight occupied camera groups and **zero empty
groups**. Pod 58's future mission files were regenerated; its active Earthquake
Suburban L1 Lawnmower retains 12/8. Active iterations keep
their launch settings. Baseline image geometry, RayFronts stereo geometry,
LiDAR configuration, simulation budget and acceptance gates remain unchanged.

This is a new sensor-rate cohort. The preceding baseline cohort was 32/8;
preceding RayFronts runs were unsliced. Do not attribute differences solely to
the planner or merge sensor-rate comparisons without noting this change.
The first live 12/8 Hurricane Urban L2 Frontier run measured mean RGB/depth
cadence **2.303 sim FPS** and **0.491 wall FPS**, with a **2.73-s maximum sim
gap** on every robot. A persistent 90-s `/clock` probe measured **RTF 0.21440**.
This is a live cross-scene observation, not a controlled same-scene FPS/RTF A/B.

Timeout allowance for subsequent pod-58 12/8 runs: completion waits increased
from 13,800 to **21,600 wall seconds (6 hours)**, with the persistent subscriber
deadline 120 seconds inside that limit; takeoff increased to **900 wall seconds**.
The actual search budget remains **600 simulated seconds**, with unchanged
acceptance gates and the existing ≤12-hour one-cell launch/cleanup cap.

**18 new accepted runs since the September 6 evening queue**: 4 on dev 191
and 14 on pod 58. The preceding fourteen were verified September 7; the four
additional rows below were checked September 8 for NAS `iteration.json=passed`
and nonempty MCAPs. Failed attempts and short diagnostics are excluded.

| Pod | Scene | Method | Final team RTF | Storage |
|---|---|---|---:|---|
| 191 | Hurricane Urban L1 | Frontier | 0.17664 | Verified |
| 191 | Hurricane Urban L1 | Lawnmower | 0.15580 | Verified |
| 191 | Hurricane Urban L1 | VLFM | 0.18885 | Verified |
| 191 | Hurricane Urban L1 | CoNavGPT2 | 0.21270 | Verified |
| 58 | Fire Urban L1 | Lawnmower | 0.17612 | Verified |
| 58 | Fire Urban L2 | Frontier | 0.19247 | Verified |
| 58 | Fire Urban L2 | VLFM | 0.20916 | Verified |
| 58 | Fire Urban L2 | CoNavGPT2 | 0.25566 | Verified |
| 58 | Fire Urban L2 | Lawnmower rerun | 0.20944 | Verified |
| 58 | Hurricane Urban L2 | Frontier rerun | 0.20779 | Verified |
| 58 | Hurricane Urban L2 | Lawnmower | 0.19375 | Verified |
| 58 | Hurricane Urban L2 | VLFM | 0.20398 | Verified |
| 58 | Hurricane Urban L2 | CoNavGPT2 | 0.22094 | Verified |
| 58 | Hurricane Urban L3 | Frontier | 0.18542 | Verified |
| 58 | Hurricane Urban L3 | Lawnmower | 0.18846 | Verified |
| 58 | Hurricane Urban L3 | VLFM | 0.20569 | Verified |
| 58 | Hurricane Urban L3 | CoNavGPT2 | 0.20686 | Verified |
| 58 | Tornado Urban L1 | Frontier | 0.17906 | Verified |

Fire Urban is now **12/12** shared-planner runs complete: L1 **4/4**, L2
**4/4**, L3 **4/4**.
Hurricane Urban L1–L3 are all **4/4**. Tornado Urban L1 is **1/4**.
These new bags still need detector-progress/PPL evaluation; completion and
timing above do not imply that the actual-results tables below include them.

## Fire

### Urban

Pod 58 started the five remaining Fire runs at **2026-09-07 06:10:59 UTC**:
L1 Lawnmower, then L2 Frontier, Lawnmower, VLFM and CoNavGPT2. Results root:
`urban_fire_remaining_8robot_optimized_pod58/2026-09-07_06-10-59`.
Its original auto-runner was interrupted cleanly with SIGINT and was not
uploaded; `mission_launcher.sh` remains stopped to prevent auto-teardown.
The manual runner is capped below 12 wall hours, with immediate pass-only
uploads. Owned GPU UUID `GPU-17194e29…` maps to Isaac device 2; both renderer
and model services are pinned to 2. Original 32/8 camera slicing and full
scene contacts are retained.
The pod-local GCS build completed after the initial GCS autolaunch, so the
first bringup exited before `action_relay` existed. Restarting only the GCS
container before takeoff restored all eight relays without disturbing Isaac or
the robots. L1 Lawnmower then passed bridge, perception and first-attempt
takeoff gates for all eight robots. Its persistent `/clock` callback measured
**RTF 0.18093** over 90.04 wall seconds (16.29 simulated seconds); the owned
GPU used about 20.35 GiB. All eight 600-s
windows completed; the final team/bottleneck RTF was **0.17612**. The 4.43 GB
MCAP and passed `iteration.json` were uploaded and independently verified on
NAS. L2 Frontier passed all eight flight gates and all 17 steps at final
team/bottleneck **RTF 0.19247**. Its passed metadata and 3.92 GB MCAP were
independently verified on NAS before L2 Lawnmower started at **08:49:17 UTC**.
L2 Lawnmower passed all eight flight gates and measured persistent-callback
**RTF 0.20327** during its active search window.
Its first attempt completed all eight 600-s planner windows at bottleneck RTF
0.19937, but a stale ROS CLI completion probe wedged and the probe-recovery
signal terminated the step shell. The runner correctly marked that attempt
failed, did not upload it, and started attempt 2 at **10:07:28 UTC**. This was a
completion-gate/tooling failure, not a flight or planner failure.
Attempt 2 brought seven robots up, but robot 5's three relay results were
polluted by a missing generated GCS setup-hook warning and failed the takeoff
gate. It was also rejected and not uploaded. Empty headless-safe setup hooks
now silence that warning for subsequent runs. The focused L2 Lawnmower rerun
then passed all 17 steps and all eight 600-second windows at bottleneck RTF
**0.20944**. Its passed metadata and 3.70 GB MCAP were uploaded and
independently verified on NAS at 16:56 UTC under
`remaining58_fireurbanl2v1_lawnmower/2026-09-07_15-47-29`. L2 VLFM started at
**10:23:44 UTC**, passed all eight flight
gates, completed at team/bottleneck **RTF 0.20916**, and its passed metadata
and 5.92 GB MCAP were verified on NAS. L2 CoNavGPT2 then passed all eight
flight gates and its 600-s team window at **RTF 0.25566**. It generated one
VLM round after spending most of the run without viable voxel frontiers; the
passed metadata and 0.63 GB MCAP were verified on NAS at **12:29 UTC**.

| Level | What is done | What is left | Intended run folder |
|---:|---|---|---|
| L1 | Canonical frozen scene; all four shared-planner baselines | RayFronts | `urban_fire_remaining_8robot_optimized_pod58/2026-09-07_06-10-59` |
| L2 | Canonical frozen scene; all four shared-planner baselines | RayFronts | `urban_fire_remaining_8robot_optimized_pod58/2026-09-07_06-10-59`; `remaining58_fireurbanl2v1_lawnmower/2026-09-07_15-47-29` |
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

Urban Hurricane L1–L3 are canonically exported and ready for benchmarking.
The L1 four-method production batch started on dev pod 191 at 04:58 UTC on
2026-09-07 under
`hurricane_urban_l1_8robot_optimized_dev191/2026-09-07_04-58-24`;
Frontier is the active first iteration. The batch retains the accepted
100 Hz physics / 30 Hz rendering and 32-group / 8-tick camera schedule,
pins both Isaac and the offboard detector to pod 191's owned GPU index 3,
and disables the unused RayFronts server. Only passed iterations upload.
No Urban Hurricane benchmark result exists yet.

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

September 8 05:54 UTC pod-58 update: L1 Frontier and Lawnmower are passed
and uploaded. Lawnmower's accepted folder is
`remaining58_tornadourbanl1v1_lawnmower/2026-09-08_03-05-21/iter_001__tornadourbanl1v1_lawnmower__lawnmower`,
with upload verification logged at 04:18:51 UTC. L1 VLFM failed before search:
robots 1 and 4 exhausted `PREARM_STATE_TIMEOUT` despite successful MAVROS arm
RPC responses. Its failed artifacts remain local, are not counted, and were
explicitly not uploaded; diagnosis and a corrected rerun remain required.
L1 CoNavGPT2 is now running. Pod 58's durable queue and stopped teardown
launcher were verified live; successful cells upload and verify before advancing.

Urban Tornado L1–L3 are canonically published on Nucleus and ready for
benchmarking. Standalone cold opens passed for all three canonical USDs
(`tornado_urban_lvl1_1.usd`, `tornado_urban_lvl2_1.usd` and
`tornado_urban_lvl3_1.usd`) in 7.12, 6.96 and 5.68 seconds. Every cell has its
GT, build, freeze and Nucleus-verification sidecars; the published verification
reports have `ok=true`, no missing Nucleus assets/arcs and `portable_ok=true`.
The generated L1/L2/L3 overlays contain 6/9/11 survivors, all inside their
search areas, and 2,052/2,122/2,008 obstacle boxes. Their eight generated
sector spawns have minimum clearances 9.1/10.1/9.4 m; the sub-10 m L1 and L3
points are the planner's explicit `roomiest` fallback and remain inside their
assigned search sectors. Three distinct four-baseline missions passed dry-run
and geometry/config validation with pod 57's assigned renderer index 2 and GPU
PhysX enabled. They are queued after the remaining Suburban Earthquake L1/L3
work, one level per fresh maximum-12-hour runner. Only passed iterations will
upload, and each upload must be NAS-verified before the queue advances. The
detached pod-57 Tornado queue watcher is PID 943128; it is explicitly held on
the paused Earthquake watcher and cannot start until that predecessor exits.
The separate shared-RAVEN mission was submitted as
`airstack-mission-8robot-2gpu-9` at 15:33 EDT on 2026-09-06 and is scheduling.
It runs L1→L2→L3 from the same frozen plans with one shared RayFronts server on
offboard GPU 1, GPU PhysX enabled, a 12-hour mission cap and a 48-hour
inspectable pod hold. Passed iterations upload and verify immediately; the
final whole-tree upload is disabled so failed attempts remain local and a
successful upload cannot tear down the pod.

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
robot 5 on its first pre-arm check), and passed 17/17 steps at team RTF
0.22040. It was uploaded and NAS-verified at 01:33 UTC. L2 Lawnmower started
immediately afterward; its targeted robot-8 zero-heartbeat recovery succeeded,
all eight takeoffs passed, and it passed 17/17 steps at team RTF 0.22115. It
was uploaded and NAS-verified at 02:48 UTC. L2 VLFM passed 17/17 steps at
team RTF 0.21346, was uploaded, and its NAS manifest was verified at 04:05
UTC. L2 CoNavGPT2 passed 17/17 steps at team RTF 0.23411 and was uploaded
and NAS-verified at 05:40 UTC, completing all eight Urban L1–L2 methods. The queued 2-GPU workflow
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
gate, so it was stopped cleanly, retained locally and not uploaded. Frozen
suburban missions now skip the redundant generated ground-collider sheet and
use the scene's existing Pegasus ground plane, with a 5-hour completion gate.
A fresh optimized L1 run started at 01:29 UTC in
`earthquake_suburban_l1_8robot_optimized_pod57/2026-09-06_01-29-21`. Its first
Frontier attempt reached all eight PX4 heartbeats but produced no raw odometry
within the 3600-second readiness gate: the frozen USD's own many convex-hull
colliders remain the dominant startup cost. The failed attempt was archived,
not uploaded, and clean automatic attempt 2 started at 02:32 UTC. That retry
reproduced 8/8 heartbeats with 0/8 raw odometry. Investigation found the
`colliders=off` optimization had also skipped the launcher's pre-spawn Kit
update barrier, allowing PX4 to start while existing frozen-scene collision
work still monopolized the simulation loop. The retry was stopped cleanly and
not uploaded. The barrier fix was tested and pushed in `6cbedf72`; its 02:59
UTC validation run finished the cook before spawning PX4 but still reproduced
0/8 raw odometry, so it too was stopped and not uploaded. Because the earlier
`ground` configuration is the only one proven to clear takeoff and enter timed
search on this scene, it was restored in `ad1b46bb`. A fresh proven-config L1
run started at 03:24 UTC in
`earthquake_suburban_l1_8robot_optimized_pod57/2026-09-06_03-24-12`.
All eight robots cleared takeoff, and Frontier entered its 600-second timed
search at 03:55–03:58 UTC. It passed all 17 steps at 07:36 UTC: its slowest
robot's 600-second timed window took 13,143.05 wall seconds (219.1 min), for
team RTF 0.04565. The 15.54 GB accepted iteration was copied to
`/media/share/coa-sei/earthquake_suburban_l1_8robot_optimized_pod57/2026-09-06_03-24-12/iter_001__earthquakesuburbanl1v1__frontier`
and its passed `iteration.json`, bag manifest and byte-for-byte rsync state were
verified before it was counted. L1 Lawnmower started immediately afterward.
Its first startup left robot 4 with one stale heartbeat and no live connection;
the other seven robots were healthy. A robot-4-only bringup restart preserved
the simulation, restored its heartbeat and odometry, and allowed the iteration
to advance to takeoff at 08:06 UTC. All eight takeoffs passed, including robot
8 on its bounded second pre-arm attempt. The per-robot Lawnmower planners
entered their timed windows at sim t=92.24–101.60 s; early `/clock` samples put
the live team RTF near 0.046. The run was rejected at final verification:
robots 2–5 had actually exited planner initialization because `camera_info`
arrived just after its hard-coded 60-wall-second deadline, while their
`ros2 launch` parents remained alive and fooled the PID-only mission gate.
The exact runner was stopped, the partial artifacts were retained locally and
nothing from this Lawnmower attempt was uploaded. Commit `5e6ce374` extends the
wall-time camera deadline to 300 seconds and makes a fatal init terminate the
planner process instead of leaving a misleading live launch parent. A fresh
one-cell Lawnmower retry started at 12:04 UTC in
`eq_sub_l1_lawnmower_attempt1/2026-09-06_12-04-23`. One scoped robot-5
readiness recovery succeeded, all eight takeoffs passed, and verification
confirmed all eight planners entered their 600-second sim budgets at
12:35–12:37 UTC; the delayed-camera failure was not present on the retry. All
eight planners later logged a complete 600.03-second window at team RTF
0.04158, but step 11 missed each one-shot completion edge because its late
subscriber did not request the publisher's transient-local history and then
hit the old 13,800-second shell timeout. The harness marked the iteration
failed, so this otherwise complete partial remains local and is not counted or
uploaded. Commit `1dc5367b` makes both L1/L3 gates request the latched sample
and raises their guard to 18,000 seconds. Focused Lawnmower attempt 2 started
at 16:42 UTC, but a queue-generation race had copied the mission before that
patch reached pod 57. It was stopped during startup, retained locally and not
uploaded. Attempt 3 started at 16:48 UTC in
`eq_sub_l1_lawnmower_attempt3/2026-09-06_16-48-51`; its generated mission was
explicitly verified to contain both transient-local/reliable completion gates
and both 18,000-second guards before launch. When robot 6 alone remained stale
after the other seven robots became ready, the scoped recovery restarted only
that robot; readiness then cleared at 17:08 UTC without restarting the scene,
and the attempt advanced without restarting the scene. All eight bridge and
perception checks passed; all eight takeoffs completed at 17:17 UTC. All eight
planners then received `camera_info` and entered their 600-second sim budgets
at sim t=61.52–70.88 s (17:21–17:24 UTC), confirming the initialization fix.
The valid search reached 194.37/600 sim seconds at team RTF 0.0362 before it
was intentionally stopped at 18:56 UTC to apply the newly validated opt-in GPU
PhysX fix. Its partial bag and diagnostics are retained locally, the attempt is
not counted and nothing was uploaded. Pod 57's queue watcher is paused and the
pod-56 smoke was interrupted by that workflow's expiry. Its corrected
100-second equivalent started on pod 57 at 02:31 UTC on 2026-09-07 with uploads
disabled. The authoritative startup gate reported solver CUDA device 2, GPU
broadphase/dynamics enabled, CPU-facing API readback retained and Fabric
disabled. All eight PX4/odometry, bridge and perception gates passed. The
reported 0.053 RTF is **provisional and invalid as a stop gate**: it compared
two separate `ros2 topic echo --once` processes, so DDS discovery/process
startup contaminated the wall interval. The smoke was stopped prematurely at
02:56 UTC, retained locally and not uploaded. Peak GPU usage was 11,200 MiB,
24% utilization and 70 W on the 48,935 MiB card. The replacement near-camera-
off diagnostic (512 groups / burst 1) used a single persistent `/clock`
subscriber after a 10-second warmup and advanced 3.81 simulated seconds over
90.139 wall seconds: authoritative RTF 0.04227. Camera rendering is therefore
not the dominant Earthquake bottleneck. A second diagnostic started at 05:10
UTC with original 32/8 sensing and CPU physics but runtime ground-only contacts;
this opt-in changes no frozen USD or rendered/depth geometry and is not approved
for accepted runs. It disabled 11,343 non-ground colliders and cut all-eight
readiness from roughly 1,084 to 583 wall seconds, but its persistent-callback
RTF was only 0.04558 (4.11 sim s / 90.169 wall s). Robots 6 and 8 then exhausted
all three pre-arm attempts, so it failed the 8/8 flight gate and was stopped
without upload. Workflow 57 has about 75 minutes left before its 100-hour
execution timeout. A parallel
corrected L2 VLFM smoke started on dev pod 191 at 02:38 UTC
with uploads disabled. Its reserved outer UUID `GPU-38264ce2…` maps to Isaac
index 3. Its authoritative banner also confirms GPU broadphase/dynamics with
CPU-facing readback and Fabric disabled. After recovering three initially
wedged MAVROS endpoints, all eight PX4/odometry, bridge and perception gates
passed. The reported 0.0547 RTF used the same two-process endpoint method and
is likewise provisional/invalid for promotion decisions; this smoke was also
stopped prematurely, retained locally and not uploaded. Its peak GPU usage was
10,625 MiB, 30% utilization and 74.6 W on the same card class. Dev 191 is now
re-measuring with the persistent callback method.
The exact outstanding pod-57 list is L1 Lawnmower, VLFM and CoNavGPT2,
followed by L3 Frontier, Lawnmower, VLFM and CoNavGPT2. Detached queue watcher
PID 1776779 enforces the 12-hour cap by stopping the current L1 batch at the
clean iteration-3 boundary, then submitting every remaining method as a
one-cell mission. After a cell passes, the watcher blocks on an explicit NAS
sentinel until its passed `iteration.json` and bag have been copied and checked;
only then can the next method launch. A failed one-cell attempt stays local and
is retried in its own inspectable mission up to three times before the queue
stops for investigation. This both prevents failed/partial uploads and keeps
every submitted mission bounded.

## Failed and superseded attempts

| Run folder | Why it does not count |
|---|---|
| `conavgpt_wildfire_1robot/2026-08-25_03-31-00`, `05-27-48`, `22-26-47` | Robot readiness failures |
| `lawnmower_wildfire_1robot/2026-08-26_00-32-51` | Robot readiness failure |
| `wildfire1km_1robot_b/2026-08-27_07-42-17` | Frontier and lawnmower readiness failures |
| `hurricane_suburban_8robot/2026-09-02_16-35-56` | Readiness failure, then aborted |
| `hurricane_suburban_8robot/2026-09-02_19-02-31`, `19-34-47`, `19-48-19`, `19-56-32`, `20-04-16` | Aborted before a scored cell |
| `hurricane_suburban_l1_conavgpt2_gt600/2026-09-04_14-02-06` | Stale-container/network conflict; superseded by successful `17-23-22` run |

## RayFronts two-GPU pod recovery — September 7

`airstack-mission-8robot-2gpu-1` started at 10:31 UTC. Its original
`raven_suburban_8robot/2026-09-07_10-36-56` batch failed all six Fire/Hurricane
cells during setup (two attempts each); the next Tornado L1 attempt was
interrupted. None is accepted. The encoder could not load missing Hugging Face
cache files because offline mode was enabled. The original renderer was also
unpinned despite the container exposing other tenants' GPUs.

At 15:00 UTC the actual launcher PID **59** was held with SIGSTOP, then the
runner was interrupted with SIGINT (subsequently TERM to finish stopping).
Launcher remains `T`; the old runner is defunct. The model cache has now been
warmed and RADIO + siglip2/SAM adapters successfully reloaded with networking
disabled. Correct device split: Isaac **1**, RayFronts **3**, both belonging
to the pod's assigned UUIDs. A three-cell Fire Suburban L1–L3 recovery spec
is prepared at `osmo/missions/raven_fire_suburban_recovery_2gpu1.yaml`.
The full non-CUDA test harness passed (1,197 passed; CUDA-specific tests not
requested). The first recovery attempt at **15:04:09 UTC** proved that the
offline RADIO encoder and real CUDA-IPC camera frames work, but robot 1 timed
out while arming and its action relay exited. That attempt was stopped cleanly
at 15:24 UTC and was **not uploaded**. A bounded 120-wall-second probe measured
RGB and depth at **1.624 FPS wall time** (33.33 FPS in simulation time), so the
low observed camera rate reflects the current approximately **0.049 RTF**, not
a 3-FPS camera setting.

The takeoff step now pre-arms each vehicle through MAVROS and staggers dispatch
by 3 seconds (`de0bc26b`). The 15:26 retry reached 8/8 readiness, perception,
arming and physical climb, but the persistent callback measurement was only
**0.05002 RTF** over 90.57 wall seconds. Takeoff completed at 19.87 m after
about 477 wall seconds, after the old 420-second relay deadline had already
issued an overlapping retry and crashed the action relays. This attempt was
stopped, retained locally and not uploaded.

The takeoff wall deadline is now 900 seconds while the scored search remains
exactly 600 simulated seconds (`5bd12d9a`). The corrected L1→L2→L3 recovery
started at **16:03:20 UTC**, runner PID **3827962**, under
`raven_fire_suburban_recovery_2gpu1/2026-09-07_16-03-20`, log
`/tmp/ray1_fire_recovery_t900.log`. A durable queue then covers the other 21
ready RayFronts cells in seven three-scene, <12-hour batches: Hurricane,
Tornado and Earthquake Suburban, followed by Fire, Hurricane, Tornado and
Earthquake Urban. Queue PID **3841863** halts on an exhausted batch for
investigation; every successful iteration must upload immediately and failed
attempts never upload. No new RayFronts result is accepted yet.

## Next work queue

1. 🟧 Focused Urban Fire L2 Lawnmower rerun; all other shared-planner Fire Urban runs are complete.
2. 🟦 Urban Hurricane L2–L3 (eight runs); L1 is complete. Use only authorized
   live pods 191/58, checking remaining lifetime before dispatch. No batch is
   currently running; prepared specifications are not proof of dispatch.
3. 🟦 Urban Tornado L1–L3 (twelve shared-planner runs) remain ready.
4. 🟧 Suburban Earthquake: L1 Lawnmower/VLFM reruns + CoNavGPT2; L2 VLFM
   rerun + CoNavGPT2; L3 all four. Its low RTF remains unresolved; the completed
   diagnostics did not justify changing production contacts or sensor cadence.
5. Evaluate the eight newly uploaded bags and update detector-progress/PPL tables.
6. Keep the existing 2-GPU RayFronts/RAVEN missions queued. Pods 56/57 are no
   longer available; do not assign them new work. Upload each passed iteration
   immediately and publish no failed attempts.

## Active batch plan

The four shared-planner baselines are assigned to the existing 1-GPU pods,
including an explicit fallback attempt for the heavier Urban Fire cells.
RayFronts/RAVEN remains in the 2-GPU queue. Each active fallback is capped at
12 wall-hours. A cell that exhausts its automatic attempts is diagnosed and
requeued as a focused rerun; it remains outstanding until a passed result is
uploaded.

| Order | Pod | Cells | Runs | Expected batch wall time | State |
|---:|---|---|---:|---:|---|
| 1 | pod 58 durable queue | Fire/Suburban L1 CoNavGPT2 | 1 | ~45 min | NEEDS RERUN — NAS audit found no accepted iteration; appended to the 32-cell queue source for its next safe start |
| 1 | pod 58 durable queue | Tornado/Suburban L3 CoNavGPT2 | 1 | ~45 min | NEEDS RERUN — NAS audit found no accepted iteration; appended to the 32-cell queue source for its next safe start |
| 2 | `airstack-mission-1gpu-56` | Hurricane/Suburban L2 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | COMPLETE — 4/4 passed, uploaded and verified |
| 2 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 × frontier, lawnmower, VLFM, CoNavGPT2 | 4 | ~3 h (12 h hard cap) | COMPLETE — 3/4 passed and uploaded; lawnmower failed twice and was not uploaded |
| 3 | `airstack-mission-1gpu-57` | Hurricane/Suburban L3 lawnmower | 1 | 61 min | PASSED — uploaded and verified at 03:40 UTC |
| 4 | `airstack-mission-1gpu-57` | Urban Fire L3 Frontier | 1 | — | STOPPED — two attempts hit the obsolete disparity gate; zero scored/uploaded runs |
| 3 | `airstack-mission-1gpu-56` | Urban Fire L1/L2 Frontier | 2 | — | STOPPED — launched before canonical publication; zero scored/uploaded runs |
| 5 | `airstack-mission-1gpu-56` | Urban Fire L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | 8 | 5 h 11 min | STOPPED FOR EARTHQUAKE CUTOVER — L1 Frontier, VLFM and CoNavGPT2 passed/uploaded (RTFs 0.219, 0.2115, 0.18784); Lawnmower failed twice and needs a focused rerun; Fire L2 remains deferred |
| 6 | `airstack-mission-1gpu-56` | Earthquake/Urban L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | L1 4/4; L2 4/4 | 11 h 38 min | COMPLETE — all eight methods passed/uploaded/NAS-verified; L2 RTFs were 0.22040, 0.22115, 0.21346 and 0.23411 |
| 5 | `airstack-mission-1gpu-57` | Urban Fire L3 Frontier | 1 | 86.7 min total / 54.8 min timed | PASSED — uploaded and verified; RTF 0.183 excluded from performance average due wrong-host-GPU placement |
| 6 | `airstack-mission-1gpu-57` | Urban Fire L3 × lawnmower, VLFM, CoNavGPT2 | 3 | ≤12 h | COMPLETE — 3/3 passed, uploaded and verified |
| 7 | `airstack-mission-1gpu-57` | Earthquake/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | 4/4 | 5 h 12 min | COMPLETE — all four passed/uploaded/NAS-verified at team RTFs 0.19188, 0.19888, 0.21211 and 0.21651 |
| 8 | `airstack-mission-1gpu-57` | Earthquake/Suburban L1 × Frontier, lawnmower, VLFM, CoNavGPT2 | 1/4 | ≤12 h | DIAGNOSTICS COMPLETE / RERUN REQUIRED — Frontier passed/uploaded/NAS-verified at team RTF 0.04565. The persistent-callback near-camera-off 512/1 diagnostic measured RTF 0.04227, ruling out camera rendering. The original-sensor ground-only-contact diagnostic disabled 11,343 colliders and halved readiness wall time, but measured RTF 0.04558 and failed 8/8 takeoff when robots 6 and 8 exhausted pre-arm retries. Both diagnostics were stopped and not uploaded. Production specs retain full contacts/original 32/8 sensors with owned GPU pins; accepted queue remains paused |
| 8 | `airstack-mission-1gpu-56` | Earthquake/Suburban L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | 2/4 | ≤12 h per launch | POD EXPIRED / RERUN REQUIRED — Frontier passed/uploaded/NAS-verified at team RTF 0.05664. The fixed Lawnmower retry passed all eight 600.03-s windows at bottleneck RTF 0.04897 and was uploaded/NAS-verified at 17:22 UTC under `earthquake_suburban_l2_remaining_optimized_pod56/2026-09-06_13-18-38/iter_001__earthquakesuburbanl2v1_lawnmower__lawnmower`; rejected partials were never retained on NAS. The full VLFM attempt was deliberately stopped/not uploaded after discovering PhysX was on CPU. GPU-physics smoke testing then identified and fixed the Pegasus direct-GPU-API incompatibility (GPU broadphase/dynamics retained; Fabric and suppressed readback disabled), and all eight PX4/odometry endpoints came ready without the articulation errors. The corrected 100-s smoke was interrupted when workflow 56 reached `FAILED_EXEC_TIMEOUT` and its pod/tunnel disappeared at 20:27 UTC. VLFM and CoNavGPT2 remain outstanding for a fresh pod |
| 8b | Unassigned | Earthquake/Suburban L2 VLFM + CoNavGPT2 replacement | 0/2 | ≤12 h per batch | WAITING — diagnostics did not restore RTF. Pod 58 was reassigned to remaining Urban Fire runs on Sep 7; dev 191 is running Urban Hurricane. No diagnostic counts as an accepted result |
| 8c | `airstack-mission-1gpu-58` | Fire/Urban L1 Lawnmower + L2 four baselines | 5/5 | 7 h 09 min | COMPLETE — all five passed runs are uploaded/NAS-verified. The focused L2 Lawnmower rerun passed at bottleneck/team **RTF 0.20944** under `remaining58_fireurbanl2v1_lawnmower/2026-09-07_15-47-29`; its verified MCAP is 3.70 GB. This final Fire run retained the original **32/8** sensor schedule and full contacts |
| 9 | `airstack-mission-1gpu-57` | Earthquake/Suburban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | QUEUED — all four methods remain outstanding as upload-gated one-cell missions after L1. Every pass must be NAS-verified before the next method; failed cells retry locally and never upload |
| 10 | dev pod 191 | Hurricane/Urban L1 × Frontier, lawnmower, VLFM, CoNavGPT2 | 4/4 | 5 h 00 min | **COMPLETE** — all four passed 17/17 steps and are uploaded/NAS-verified under `hurricane_urban_l1_8robot_optimized_dev191/2026-09-07_04-58-24/`. Bottleneck/team RTFs: Frontier **0.17664** (600.03 sim s / 3396.96 wall s), Lawnmower **0.15580** (600.03 / 3851.17), VLFM **0.18885** (600.03 / 3177.28), CoNavGPT2 **0.21270** (600.03 / 2820.96; 84 VLM rounds). Remote nonempty MCAP sizes are 4.11, 4.53, 6.37, and 0.79 GB respectively. The launcher's storage env expanded an unquoted `$` in the password, so each passed iteration was manually uploaded with literal credential parsing and its remote `iteration.json`/MCAP verified |
| 11 | pod 58 queue | Hurricane/Urban L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | 4/4 | 4 h 20 min | COMPLETE — all four pod 58 methods passed all 17 steps at bottleneck/team RTFs **0.20779**, **0.19375**, **0.20398** and **0.22094**. Their passed metadata and 3.78/3.85/6.38/0.72 GB MCAPs were independently NAS-verified under their respective `remaining58_hurricaneurbanl2v1_*` folders. These runs used the new **12/8** camera cohort, full contacts and GPU 2/2; rejected dev-191 Frontier attempts uploaded nothing |
| 12 | pod 58 queue | Hurricane/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | 1/4 | ≤12 h per launch | FRONTIER PASSED / LAWNMOWER ACTIVE — Frontier passed all 17 steps at bottleneck/team **RTF 0.18542** under `remaining58_hurricaneurbanl3v1_frontier/2026-09-07_21-17-47`; its passed metadata and 3.54 GB MCAP were independently NAS-verified. The queue then started Lawnmower at **22:29:23 UTC** under `remaining58_hurricaneurbanl3v1_lawnmower/2026-09-07_22-29-23`, runner PID 3390857 |
| 13 | `airstack-mission-1gpu-57` | Tornado/Urban L1 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | READY/QUEUED — canonical Nucleus USD/GT/asset verification and cold open passed; 6/6 survivors are inside the generated search area; generated spawns have 9.1–11.0 m clearance; mission/overlay dry-run passed with GPU PhysX. Starts after pod 57's remaining Earthquake work, after GPU-physics smoke authorization |
| 14 | `airstack-mission-1gpu-57` | Tornado/Urban L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | READY/QUEUED — canonical Nucleus USD/GT/asset verification and cold open passed; 9/9 survivors are inside the generated search area; generated spawns have 10.1–11.3 m clearance; mission/overlay dry-run passed with GPU PhysX. Starts after Tornado/Urban L1 under a fresh 12-hour cap |
| 15 | `airstack-mission-1gpu-57` | Tornado/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | 0/4 | ≤12 h | READY/QUEUED — canonical Nucleus USD/GT/asset verification and cold open passed; 11/11 survivors are inside the generated search area; generated spawns have 9.4–10.9 m clearance; mission/overlay dry-run passed with GPU PhysX. Starts after Tornado/Urban L2 under a fresh 12-hour cap |

RayFronts/RAVEN is split into three 2-GPU workflows. Each runs three scene
levels under a 12-hour mission cap, while the pod itself remains alive for 48
hours for inspection and corrective reruns.

| Workflow | Cells | State |
|---|---|---|
| `airstack-mission-8robot-2gpu-2` | Fire/Suburban L1–L3 RayFronts | QUEUED |
| `airstack-mission-8robot-2gpu-3` | RAVEN lane A | FAILED_EXEC_TIMEOUT — four accepted cells remain NAS-verified; interrupted Tornado Suburban L3 was not uploaded |
| `airstack-mission-8robot-2gpu-4` | Fire/Suburban L2 + Hurricane/Suburban lane | FAILED_EVICTED — Fire L2 and Hurricane L1 NAS-verified; partial Hurricane L3 discarded after the workspace exceeded 400 GiB ephemeral storage |
| `airstack-mission-8robot-2gpu-5` | RAVEN lane B | RUNNING — Hurricane Suburban L3, Tornado Suburban L2 and Earthquake Suburban L1 NAS-verified; Earthquake Suburban L3 active from 19:05:02 UTC |
| `airstack-mission-8robot-2gpu-6` | Replacement RAVEN lane A | RUNNING — Tornado Suburban L3 active from 19:10:03 UTC after verified GPU/cache handoff; subsequent lane-A cells remain chained with per-cell upload/checksum/prune |
| `airstack-mission-8robot-2gpu-7` | Earthquake/Urban L1–L2 × Frontier, lawnmower, VLFM, CoNavGPT2 | CANCELED — replaced by the active split run on pod 56 |
| `airstack-mission-8robot-2gpu-8` | Earthquake/Urban L3 × Frontier, lawnmower, VLFM, CoNavGPT2 | CANCELED — replaced by the active split run on pod 57 |
| `airstack-mission-8robot-2gpu-9` | Tornado/Urban L1–L3 RayFronts | PENDING/SCHEDULING — submitted 2026-09-06 15:33 EDT; 12 h mission cap, 48 h inspectable pod; passed-only immediate NAS uploads |
| `airstack-mission-8robot-2gpu-10` | Hurricane/Suburban L3 replacement, then remaining RAVEN lane | PENDING/SCHEDULING — submitted 2026-09-08 18:42 EDT with a no-op hold mission so GPU UUID/index mapping is inspected before production starts |

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
| Hurricane | 55.2 min / 0.1835 (n=4) | 38.3 min / 0.265 (n=9) |
| Tornado | — | 77.4 min / 0.129 (n=11) |
| Earthquake | 47.5 min / 0.212 (n=12) | 197.8 min / 0.051 (n=2) |

### Measured camera cadence — September 7

Direct read-only image subscribers measured unique RGB/depth header timestamps
over 120 wall seconds. The production 32-group/8-update schedule was reproduced
on pod 58 with eight stationary drones, 720×450 mono RGB-D and no search
planner running. These are sensor-cadence diagnostics, not accepted search runs.

| Configuration / robot | RGB FPS per sim second | Depth FPS per sim second | RGB FPS per wall second | Depth FPS per wall second | Maximum gap (sim seconds) |
|---|---:|---:|---:|---:|---:|
| Baseline 32/8, robot 1 | 0.992 | 0.992 | 0.303 | 0.303 | 7.53 |
| Baseline 32/8, robot 2 | 0.992 | 0.992 | 0.272 | 0.272 | 7.53 |
| Baseline 32/8, robot 3 | 0.876 | 0.908 | 0.266 | 0.276 | 7.53 |
| Baseline 32/8, robot 4 | 0.992 | 0.992 | 0.302 | 0.302 | 7.53 |
| Baseline 32/8, robot 5 | 0.939 | 0.939 | 0.287 | 0.287 | 7.53 |
| Baseline 32/8, robot 6 | 0.939 | 0.939 | 0.283 | 0.283 | 7.53 |
| Baseline 32/8, robot 7 | 0.939 | 0.939 | 0.285 | 0.285 | 7.53 |
| Baseline 32/8, robot 8 | 0.992 | 0.992 | 0.307 | 0.307 | 7.53 |
| **Baseline 32/8 mean** | **0.958** | **0.962** | **0.288** | **0.289** | **7.53** |
| **Baseline 12/8 mean, live Frontier** | **2.303** | **2.303** | **0.491** | **0.491** | **2.73** |
| RAVEN unsliced 1/1, robot 1 | 33.333 | 33.333 | 1.625 | 1.625 | 0.03 |

The official all-eight baseline step passed. Each 120-wall-second sample
contained 24–30 unique RGB frames and 24–30 unique depth frames (robot 3
delivered 28 RGB versus 29 depth frames), spanning 23.19–30.87 simulated
seconds between its first and last images. Every robot had three or four gaps
over one simulated second. Within bursts, median image spacing was 0.03
simulated seconds. A preliminary simultaneous all-eight probe produced the same
approximately 0.94–0.99 sim-FPS cadence; the mission runner then repeated the
official per-robot probes sequentially. RAVEN used its own 960×600 stereo
configuration and a different running scene/load, so this is not a controlled
RTF comparison.

The 12/8 row is a simultaneous all-eight probe during Hurricane Urban L2
Frontier's accepted-run attempt. Per-robot RGB and depth delivery matched:
sim FPS ranged **2.127–2.413** and wall FPS **0.454–0.510**. Frames retained the
eight-update burst (0.03-s median intra-burst spacing), with the blind interval
reduced from 7.53 to 2.73 simulated seconds.

**Correction:** scheduling counters advance per application/render-loop update,
not per 100 Hz physics substep. Earlier 80 ms/2.56-second scheduling estimates
and the inferred 3.125 FPS were incorrect. The measured ~1 FPS, with ~7.5-second
blind intervals, is a potential detection-performance confound. Its causal
effect requires a higher-cadence A/B; no production sensor settings have been
changed by this measurement. Probe: `scripts/measure_camera_rates.py`;
diagnostic: `camera_rate_32x8_pod58/2026-09-07_15-17-40`.

## Actual results (detector-confirmed team progress and PPL)

### Strict 10 m progress — all completed runs

Pod 58 upload reconciliation: **23/23 accepted production iterations** match
the NAS file inventory and sizes, with matching `iteration.json` SHA-256.
This includes 19 results from the current queue plus four earlier Urban Fire
results; the camera-rate diagnostic was excluded. No missing accepted upload
was found on pod 58. Audit: `osmo/results/pod58_upload_audit.json`.

The September 10 incremental strict refresh completed **90/90** accepted
standard-baseline cells with zero analysis failures. Output:
`osmo/results/completed_progress_10m.json`.
Credit each distinct GT person whose world XY position lies within 10 m of
any detector target during search; one target can credit multiple people.
No IoU alternative, robot-proximity credit, integrated progress or PPL in this
refresh. Average progress is the arithmetic mean of per-run detected/GT ratios.
Failed/partial attempts never count. Multiple accepted reruns of the same
scene/method count once, using the latest accepted result.

| Method | Storage-confirmed completed / planned eight-robot runs | Average progress, strict 10 m |
|---|---:|---:|
| Frontier | 24/24 | 3.721% |
| Lawnmower | 22/24 | 1.023% |
| VLFM | 22/24 | 2.947% |
| CoNavGPT2 | 22/24 | 0.115% |
| RAVEN | 10/24 | 5.148% |

The full four-plus-eight-robot plan has 48 runs per method; the table above
tracks the current eight-robot sweep (24 scenes). The two previously missing
CoNavGPT2 cells, Fire Suburban L1 and Tornado Suburban L3, now have accepted
NAS results. Completed-run counts are not analysis counts. RAVEN's ninth and
tenth accepted results, Earthquake Suburban L1 and L3, scored 0/12 and 0/70 at
strict 10 m and are now included in its average.

A GT victim counts as detected when its world-frame XY location falls inside a **12 m circle around a planner `search_target`** during the 600-s search. A target circle exists only after a `person` detection clears the shared 0.65 confidence gate, is depth-projected, and forms a clustered target instance. One liberal circle can credit multiple GT people; drone proximity alone never counts. Time-integrated progress is normalized area under the cumulative detector-confirmed progress curve; marker chunks were sampled at about 20-s intervals (final persistent target state is always read, so final detection counts are exact). Paths are 1 Hz, world-frame XY odometry. Ideal lengths are OR-Tools oracle estimates for open Euclidean multi-depot routes through victim centres; fixed-sector methods preserve recorded robot ownership, while CoNavGPT2 permits joint assignment. Ground debris does not obstruct an aerial XY geodesic, and no return to launch is required. PPL uses the ideal route through detected GT victims: `progress × ideal_detected / max(actual, ideal_detected)`.

### Per completed run

| Scene | Method | Run folder | GT | Detected | Detector-confirmed progress | Time-integrated progress | Actual team path | Ideal all-target path | PPL |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| Earthquake / Urban L1 | CoNavGPT2 | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_004__earthquakeurbanl1v1_conavgpt2_team__conavgpt2_team` | 39 | 0 | 0.000 | 0.000 | 7.96 km | 3.19 km | 0.0000 |
| Earthquake / Urban L1 | Frontier | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_001__earthquakeurbanl1v1__frontier` | 39 | 0 | 0.000 | 0.000 | 3.98 km | 5.27 km | 0.0000 |
| Earthquake / Urban L1 | Lawnmower | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_002__earthquakeurbanl1v1_lawnmower__lawnmower` | 39 | 0 | 0.000 | 0.000 | 7.38 km | 4.81 km | 0.0000 |
| Earthquake / Urban L1 | VLFM | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_003__earthquakeurbanl1v1__vlfm` | 39 | 0 | 0.000 | 0.000 | 4.16 km | 5.35 km | 0.0000 |
| Earthquake / Urban L2 | CoNavGPT2 | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_008__earthquakeurbanl2v1_conavgpt2_team__conavgpt2_team` | 64 | 0 | 0.000 | 0.000 | 8.76 km | 5.02 km | 0.0000 |
| Earthquake / Urban L2 | Frontier | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_005__earthquakeurbanl2v1__frontier` | 64 | 0 | 0.000 | 0.000 | 6.24 km | 8.27 km | 0.0000 |
| Earthquake / Urban L2 | Lawnmower | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_006__earthquakeurbanl2v1_lawnmower__lawnmower` | 64 | 0 | 0.000 | 0.000 | 7.91 km | 8.27 km | 0.0000 |
| Earthquake / Urban L2 | VLFM | `urban_earthquake_l12_8robot_optimized_pod56/2026-09-05_18-02-41/iter_007__earthquakeurbanl2v1__vlfm` | 64 | 0 | 0.000 | 0.000 | 5.43 km | 8.27 km | 0.0000 |
| Earthquake / Urban L3 | CoNavGPT2 | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_004__earthquakeurbanl3v1_conavgpt2_team__conavgpt2_team` | 64 | 0 | 0.000 | 0.000 | 6.03 km | 5.15 km | 0.0000 |
| Earthquake / Urban L3 | Frontier | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_001__earthquakeurbanl3v1__frontier` | 64 | 0 | 0.000 | 0.000 | 5.15 km | 8.13 km | 0.0000 |
| Earthquake / Urban L3 | Lawnmower | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_002__earthquakeurbanl3v1_lawnmower__lawnmower` | 64 | 0 | 0.000 | 0.000 | 10.35 km | 8.13 km | 0.0000 |
| Earthquake / Urban L3 | VLFM | `urban_earthquake_l3_8robot_optimized_pod57/2026-09-05_18-31-13/iter_003__earthquakeurbanl3v1__vlfm` | 64 | 0 | 0.000 | 0.000 | 3.31 km | 7.93 km | 0.0000 |
| Fire / Suburban L1 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_001__firesuburbanl1v1__frontier` | 49 | 10 | 0.204 | 0.175 | 5.59 km | 1.71 km | 0.0025 |
| Fire / Suburban L1 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_002__firesuburbanl1v1_lawnmower__lawnmower` | 49 | 0 | 0.000 | 0.000 | 14.53 km | 1.71 km | 0.0000 |
| Fire / Suburban L1 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_003__firesuburbanl1v1__vlfm` | 49 | 11 | 0.224 | 0.204 | 4.79 km | 1.71 km | 0.0126 |
| Fire / Suburban L2 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_008__firesuburbanl2v1_conavgpt2_team__conavgpt2_team` | 79 | 0 | 0.000 | 0.000 | 5.68 km | 1.53 km | 0.0000 |
| Fire / Suburban L2 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_005__firesuburbanl2v1__frontier` | 79 | 15 | 0.190 | 0.129 | 14.11 km | 2.25 km | 0.0045 |
| Fire / Suburban L2 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_006__firesuburbanl2v1_lawnmower__lawnmower` | 79 | 0 | 0.000 | 0.000 | 13.37 km | 2.25 km | 0.0000 |
| Fire / Suburban L2 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_007__firesuburbanl2v1__vlfm` | 79 | 27 | 0.342 | 0.266 | 4.52 km | 2.25 km | 0.0258 |
| Fire / Suburban L3 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_012__firesuburbanl3v1_conavgpt2_team__conavgpt2_team` | 84 | 0 | 0.000 | 0.000 | 6.92 km | 1.89 km | 0.0000 |
| Fire / Suburban L3 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_009__firesuburbanl3v1__frontier` | 84 | 17 | 0.202 | 0.105 | 11.70 km | 2.94 km | 0.0071 |
| Fire / Suburban L3 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_010__firesuburbanl3v1_lawnmower__lawnmower` | 84 | 4 | 0.048 | 0.003 | 14.29 km | 2.94 km | 0.0007 |
| Fire / Suburban L3 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_011__firesuburbanl3v1__vlfm` | 84 | 0 | 0.000 | 0.000 | 4.22 km | 2.94 km | 0.0000 |
| Fire / Urban L2 | Lawnmower | `remaining58_fireurbanl2v1_lawnmower/2026-09-07_15-47-29/iter_001__fireurbanl2v1_lawnmower__lawnmower` | 68 | 3 | 0.044 | 0.008 | 9.73 km | 2.25 km | 0.0048 |
| Hurricane / Suburban L1 | CoNavGPT2 | `hurricane_suburban_l1_conavgpt2_gt600/2026-09-04_17-23-22/iter_001__hurricanesuburbanl1v1_conavgpt2_team__conavgpt2_team` | 55 | 0 | 0.000 | 0.000 | 6.47 km | 2.89 km | 0.0000 |
| Hurricane / Suburban L1 | Frontier | `hurricane_suburban_8robot/2026-09-02_20-13-16/iter_001__hurricanesuburbanl1v1__frontier` | 55 | 1 | 0.018 | 0.018 | 5.21 km | 4.86 km | 0.0002 |
| Hurricane / Suburban L1 | Lawnmower | `hurricane_suburban_8robot/2026-09-02_20-13-16/iter_002__hurricanesuburbanl1v1_lawnmower__lawnmower` | 55 | 0 | 0.000 | 0.000 | 8.69 km | 3.62 km | 0.0000 |
| Hurricane / Suburban L1 | VLFM | `hurricane_suburban_8robot/2026-09-02_20-13-16/iter_003__hurricanesuburbanl1v1__vlfm` | 55 | 0 | 0.000 | 0.000 | 0.06 km | 2.89 km | 0.0000 |
| Hurricane / Urban L2 | CoNavGPT2 | `remaining58_hurricaneurbanl2v1_conavgpt2_team/2026-09-07_20-19-27/iter_001__hurricaneurbanl2v1_conavgpt2_team__conavgpt2_team` | 12 | 0 | 0.000 | 0.000 | 5.75 km | 1.67 km | 0.0000 |
| Hurricane / Urban L2 | Frontier | `remaining58_hurricaneurbanl2v1_frontier/2026-09-07_16-58-05/iter_001__hurricaneurbanl2v1__frontier` | 12 | 0 | 0.000 | 0.000 | 6.07 km | 1.84 km | 0.0000 |
| Hurricane / Urban L2 | Lawnmower | `remaining58_hurricaneurbanl2v1_lawnmower/2026-09-07_18-03-52/iter_001__hurricaneurbanl2v1_lawnmower__lawnmower` | 12 | 1 | 0.083 | 0.036 | 8.43 km | 1.84 km | 0.0038 |
| Hurricane / Urban L2 | VLFM | `remaining58_hurricaneurbanl2v1_vlfm/2026-09-07_19-12-31/iter_001__hurricaneurbanl2v1__vlfm` | 12 | 0 | 0.000 | 0.000 | 5.16 km | 1.84 km | 0.0000 |
| Hurricane / Urban L3 | CoNavGPT2 | `remaining58_hurricaneurbanl3v1_conavgpt2_team/2026-09-08_00-47-54/iter_001__hurricaneurbanl3v1_conavgpt2_team__conavgpt2_team` | 22 | 0 | 0.000 | 0.000 | 8.11 km | 2.54 km | 0.0000 |
| Hurricane / Urban L3 | Frontier | `remaining58_hurricaneurbanl3v1_frontier/2026-09-07_21-17-47/iter_001__hurricaneurbanl3v1__frontier` | 22 | 0 | 0.000 | 0.000 | 4.80 km | 4.03 km | 0.0000 |
| Hurricane / Urban L3 | Lawnmower | `remaining58_hurricaneurbanl3v1_lawnmower/2026-09-07_22-29-23/iter_001__hurricaneurbanl3v1_lawnmower__lawnmower` | 22 | 0 | 0.000 | 0.000 | 9.12 km | 4.03 km | 0.0000 |
| Hurricane / Urban L3 | VLFM | `remaining58_hurricaneurbanl3v1_vlfm/2026-09-07_23-41-35/iter_001__hurricaneurbanl3v1__vlfm` | 22 | 0 | 0.000 | 0.000 | 4.67 km | 4.03 km | 0.0000 |
| Tornado / Suburban L1 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_016__tornadosuburbanl1v1_conavgpt2_team__conavgpt2_team` | 30 | 0 | 0.000 | 0.000 | 1.83 km | 0.85 km | 0.0000 |
| Tornado / Suburban L1 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_013__tornadosuburbanl1v1__frontier` | 30 | 0 | 0.000 | 0.000 | 10.71 km | 0.91 km | 0.0000 |
| Tornado / Suburban L1 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_014__tornadosuburbanl1v1_lawnmower__lawnmower` | 30 | 2 | 0.067 | 0.017 | 11.66 km | 0.91 km | 0.0003 |
| Tornado / Suburban L1 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_015__tornadosuburbanl1v1__vlfm` | 30 | 0 | 0.000 | 0.000 | 5.08 km | 0.91 km | 0.0000 |
| Tornado / Suburban L2 | CoNavGPT2 | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_020__tornadosuburbanl2v1_conavgpt2_team__conavgpt2_team` | 40 | 0 | 0.000 | 0.000 | 4.57 km | 0.92 km | 0.0000 |
| Tornado / Suburban L2 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_017__tornadosuburbanl2v1__frontier` | 40 | 4 | 0.100 | 0.068 | 11.38 km | 1.01 km | 0.0015 |
| Tornado / Suburban L2 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_018__tornadosuburbanl2v1_lawnmower__lawnmower` | 40 | 0 | 0.000 | 0.000 | 10.56 km | 1.01 km | 0.0000 |
| Tornado / Suburban L2 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_019__tornadosuburbanl2v1__vlfm` | 40 | 0 | 0.000 | 0.000 | 4.78 km | 1.01 km | 0.0000 |
| Tornado / Suburban L3 | Frontier | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_021__tornadosuburbanl3v1__frontier` | 70 | 2 | 0.029 | 0.018 | 9.78 km | 1.76 km | 0.0006 |
| Tornado / Suburban L3 | Lawnmower | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_022__tornadosuburbanl3v1_lawnmower__lawnmower` | 70 | 2 | 0.029 | 0.012 | 13.29 km | 1.76 km | 0.0004 |
| Tornado / Suburban L3 | VLFM | `frozen_suburban_8robot/2026-08-31_11-11-42/iter_023__tornadosuburbanl3v1__vlfm` | 70 | 0 | 0.000 | 0.000 | 4.80 km | 1.76 km | 0.0000 |
| Tornado / Urban L1 | CoNavGPT2 | `remaining58_tornadourbanl1v1_conavgpt2_team/2026-09-08_04-49-32/iter_001__tornadourbanl1v1_conavgpt2_team__conavgpt2_team` | 6 | 0 | 0.000 | 0.000 | 6.35 km | 0.78 km | 0.0000 |
| Tornado / Urban L1 | Frontier | `remaining58_tornadourbanl1v1_frontier/2026-09-08_01-50-59/iter_001__tornadourbanl1v1__frontier` | 6 | 0 | 0.000 | 0.000 | 5.63 km | 1.29 km | 0.0000 |
| Tornado / Urban L1 | Lawnmower | `remaining58_tornadourbanl1v1_lawnmower/2026-09-08_03-05-21/iter_001__tornadourbanl1v1_lawnmower__lawnmower` | 6 | 0 | 0.000 | 0.000 | 9.44 km | 1.35 km | 0.0000 |
| Tornado / Urban L2 | Frontier | `remaining58_tornadourbanl2v1_frontier/2026-09-08_06-00-28/iter_001__tornadourbanl2v1__frontier` | 9 | 0 | 0.000 | 0.000 | 6.41 km | 2.28 km | 0.0000 |
| Tornado / Urban L2 | Lawnmower | `remaining58_tornadourbanl2v1_lawnmower/2026-09-08_07-17-25/iter_001__tornadourbanl2v1_lawnmower__lawnmower` | 9 | 0 | 0.000 | 0.000 | 7.28 km | 2.28 km | 0.0000 |
| Tornado / Urban L2 | VLFM | `remaining58_tornadourbanl2v1_vlfm/2026-09-08_08-37-40/iter_001__tornadourbanl2v1__vlfm` | 9 | 1 | 0.111 | 0.021 | 5.80 km | 2.28 km | 0.0025 |
| Tornado / Urban L3 | CoNavGPT2 | `remaining58_tornadourbanl3v1_conavgpt2_team/2026-09-08_14-02-43/iter_001__tornadourbanl3v1_conavgpt2_team__conavgpt2_team` | 11 | 0 | 0.000 | 0.000 | 6.31 km | 1.50 km | 0.0000 |
| Tornado / Urban L3 | Frontier | `remaining58_tornadourbanl3v1_frontier/2026-09-08_10-23-40/iter_001__tornadourbanl3v1__frontier` | 11 | 0 | 0.000 | 0.000 | 6.60 km | 2.49 km | 0.0000 |
| Tornado / Urban L3 | VLFM | `remaining58_tornadourbanl3v1_vlfm/2026-09-08_12-52-38/iter_001__tornadourbanl3v1__vlfm` | 11 | 0 | 0.000 | 0.000 | 5.67 km | 2.49 km | 0.0000 |

### Average by baseline

| Method | Completed / total runs | Avg progress | Avg time-integrated progress | Avg actual team path | Avg ideal all-target path | Avg PPL |
|---|---:|---:|---:|---:|---:|---:|
| CoNavGPT2 | 12/48 | 0.000 | 0.000 | 6.23 km | 2.33 km | 0.0000 |
| Frontier | 15/48 | 0.050 | 0.034 | 7.56 km | 3.27 km | 0.0011 |
| Lawnmower | 15/48 | 0.018 | 0.005 | 10.40 km | 3.14 km | 0.0007 |
| VLFM | 14/48 | 0.048 | 0.035 | 4.46 km | 3.26 km | 0.0029 |

### Target-circle radius sensitivity

These rows change only the GT-to-target-circle association radius; the detector gate and target circles are unchanged.

| Method | Runs | 12 m progress | 17 m progress | Gain vs base | 22 m progress | Gain vs base |
|---|---:|---:|---:|---:|---:|---:|
| CoNavGPT2 | 12 | 0.000 | 0.000 | +0.000 | 0.000 | +0.000 |
| Frontier | 15 | 0.050 | 0.064 | +0.014 | 0.078 | +0.028 |
| Lawnmower | 15 | 0.018 | 0.024 | +0.006 | 0.036 | +0.018 |
| VLFM | 14 | 0.048 | 0.055 | +0.007 | 0.061 | +0.012 |

#### Runs with zero detections at 12 m

| Scene | Method | 12 m detected / GT | 17 m detected / GT | 22 m detected / GT |
|---|---|---:|---:|---:|
| Earthquake / Urban L1 | CoNavGPT2 | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L1 | Frontier | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L1 | Lawnmower | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L1 | VLFM | 0/39 | 0/39 | 0/39 |
| Earthquake / Urban L2 | CoNavGPT2 | 0/64 | 0/64 | 0/64 |
| Earthquake / Urban L2 | Frontier | 0/64 | 0/64 | 0/64 |
| Earthquake / Urban L2 | Lawnmower | 0/64 | 0/64 | 0/64 |
| Earthquake / Urban L2 | VLFM | 0/64 | 0/64 | 0/64 |
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
| Hurricane / Urban L2 | CoNavGPT2 | 0/12 | 0/12 | 0/12 |
| Hurricane / Urban L2 | Frontier | 0/12 | 0/12 | 0/12 |
| Hurricane / Urban L2 | VLFM | 0/12 | 0/12 | 0/12 |
| Hurricane / Urban L3 | CoNavGPT2 | 0/22 | 0/22 | 0/22 |
| Hurricane / Urban L3 | Frontier | 0/22 | 0/22 | 0/22 |
| Hurricane / Urban L3 | Lawnmower | 0/22 | 0/22 | 1/22 |
| Hurricane / Urban L3 | VLFM | 0/22 | 0/22 | 0/22 |
| Tornado / Suburban L1 | CoNavGPT2 | 0/30 | 0/30 | 0/30 |
| Tornado / Suburban L1 | Frontier | 0/30 | 0/30 | 1/30 |
| Tornado / Suburban L1 | VLFM | 0/30 | 2/30 | 2/30 |
| Tornado / Suburban L2 | CoNavGPT2 | 0/40 | 0/40 | 0/40 |
| Tornado / Suburban L2 | Lawnmower | 0/40 | 0/40 | 0/40 |
| Tornado / Suburban L2 | VLFM | 0/40 | 0/40 | 0/40 |
| Tornado / Suburban L3 | VLFM | 0/70 | 0/70 | 0/70 |
| Tornado / Urban L1 | CoNavGPT2 | 0/6 | 0/6 | 0/6 |
| Tornado / Urban L1 | Frontier | 0/6 | 0/6 | 0/6 |
| Tornado / Urban L1 | Lawnmower | 0/6 | 0/6 | 0/6 |
| Tornado / Urban L2 | Frontier | 0/9 | 0/9 | 0/9 |
| Tornado / Urban L2 | Lawnmower | 0/9 | 0/9 | 0/9 |
| Tornado / Urban L3 | CoNavGPT2 | 0/11 | 0/11 | 0/11 |
| Tornado / Urban L3 | Frontier | 0/11 | 0/11 | 0/11 |
| Tornado / Urban L3 | VLFM | 0/11 | 0/11 | 0/11 |

Breakdown opportunities count each GT victim once per completed run/method; the same frozen-scene victim is therefore one opportunity for each baseline that searched that scene.

### Detection breakdown by pose

| Pose | Detected / opportunities | Detection rate |
|---|---:|---:|
| crouched | 0/23 | 0.000 |
| lying | 10/570 | 0.018 |
| seated | 39/423 | 0.092 |
| unknown | 24/943 | 0.025 |
| upright | 27/500 | 0.054 |

### Detection breakdown by visibility

| Visibility | Detected / opportunities | Detection rate |
|---|---:|---:|
| full | 7/377 | 0.019 |
| partial | 4/333 | 0.012 |
| unknown | 89/1749 | 0.051 |

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
| unknown | 89/1749 | 0.051 |

### Detection breakdown by environment

| Environment | Detected / opportunities | Detection rate |
|---|---:|---:|
| Earthquake / unknown | 0/668 | 0.000 |
| Fire / at_home | 0/8 | 0.000 |
| Fire / cul_de_sac | 27/165 | 0.164 |
| Fire / gridlock | 4/216 | 0.019 |
| Fire / open_ground | 21/143 | 0.147 |
| Fire / parking_refuge | 32/224 | 0.143 |
| Fire / pools | 0/43 | 0.000 |
| Fire / unknown | 3/68 | 0.044 |
| Hurricane / flood_stranded | 0/24 | 0.000 |
| Hurricane / glass_debris_injury | 0/64 | 0.000 |
| Hurricane / pinned_tree | 0/16 | 0.000 |
| Hurricane / roof | 1/160 | 0.006 |
| Hurricane / seated_wind_injury | 1/32 | 0.031 |
| Hurricane / water | 0/60 | 0.000 |
| Tornado / pile | 1/141 | 0.007 |
| Tornado / skirt | 3/127 | 0.024 |
| Tornado / street | 1/78 | 0.013 |
| Tornado / trail | 0/4 | 0.000 |
| Tornado / unknown | 1/78 | 0.013 |
| Tornado / yard | 5/140 | 0.036 |
