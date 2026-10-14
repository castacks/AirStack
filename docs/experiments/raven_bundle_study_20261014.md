# RAVEN bundle length and independent target selection

## Protocol

32 trials: four new variants, each using the same original **s1** three-robot
positions in all eight scenes, full communication (5000 m), and 300 simulated
seconds of semantic search. One eight-trial OSMO job per variant, HIGH priority
in dsta, 4 GPUs and 32 CPUs, 6-hour execution timeout. The timeout allows for
eight stack startups, searches and uploads; it is not the scoring horizon.

| Variant | Assignment | Bundle maximum | Trials |
|---|---|---:|---:|
| auction_l1 | Auction | 1 | 8 |
| historical L3 | Auction, existing control | 3 | Reused |
| auction_l6 | Auction | 6 | 8 |
| auction_l12 | Auction | 12 | 8 |
| greedy_l1 | Independent local selection | 1 | 8 |

Length is a maximum reservation count, not a minimum or executed route length.
The head is executed; tails are replanned every tick. Preserve the existing
20 m inter-target detour gate and ray-terminal rule in every auction variant.
Only bundle length differs among the three new auction arms. All variants retain
ray/BB sharing, completion sharing, perception, search boundaries, target queries,
speed limits and navigation safety. Reuse the existing matched s1 L3 results as requested. Record their source
paths and code versions before analysis; a historical L3 comparison can include
software-version or timing differences. L1/L6/L12 and greedy are contemporary.

Independent greedy retains the L1 local surface-distance cost, ray penalty,
eligibility and switch hysteresis. Each robot solves its local choice without
peer allocation inputs. There is no exclusive ownership, reservation, allocation
repulsion, closer-observer ray yielding or proximity-voxel peer deference.
Several robots may choose the same target. An eligible target takes precedence
over frontier exploration, as in the existing single-agent solver. Shared
completed targets remain excluded. Frontier exploration and collision avoidance
are unchanged: this is an ablation of **target allocation**, not all forms of
coordination. Compare greedy primarily against auction L1 to avoid confounding
auction removal with bundle length; additionally show standard L3.

## Environment descriptors

Area below is the configured search polygon, not navigable free-space area.
Target counts are the query-matched GT denominators from the existing
`raven_results/analysis_20260920/scores/3r_full_comms/per_run.json`; verify the
same denominators from each new recording before pooling scores.

| Scene | Search area (m²) | GT targets | Targets/ha |
|---|---:|---:|---:|
| ConstructionSite | 26,350 | 7 | 2.657 |
| AbandonedFactory | 30,646 | 7 | 2.284 |
| FireAcademy | 36,450 | 10 | 2.743 |
| ModernCityDowntown | 37,800 | 14 | 3.704 |
| RetroNeighborhood | 57,600 | 24 | 4.167 |
| DowntownWest | 84,000 | 12 | 1.429 |
| MilitaryBase | 154,875 | 7 | 0.452 |
| Shipyard | 250,000 | 50 | 2.000 |

Use the four smallest and four largest polygons for a descriptive size split.
For target-count panels use <=10 targets versus >10 (four scenes each).
Also show continuous area, count and density, since size and target count are
not equivalent (e.g. MilitaryBase is large but has seven query targets).
These attributes covary with scene geometry and visibility; this experiment
does not isolate their causal effects. With one start and one realization per
scene, differences are preliminary and do not establish repeatability.

## Analysis and figures

1. Score unique query-matched GT objects against the union of the three actual
   trajectories, using the existing 3 m footprint-distance criterion and the
   same 300-second search window. A target contributes once even when multiple
   robots visit it. Do not sum per-robot progress.
2. Report final progress, normalized progress AUC over 0–300 s, PPL-Opt,
   PPL-Act, actual path length, and redundant visits under one consistent
   definition. Preserve per-scene denominators and per-robot motion diagnostics.
3. Plot all five progress curves (including the reused L3 control) with equal scene weights. Plot final progress
   against bundle maximum for each scene, and paired differences from L3
   against polygon area, target count and density. Show scene points, not only
   a grand mean. A scene bootstrap describes across-scene uncertainty only,
   not within-scene run-to-run variability.
4. Use recorded auction traces to compute actual bundle occupancy (including
   zero-task ticks), fraction at the configured maximum, task-table size and
   BB/ray composition. Use each robot's own row, not duplicated peer predictions.
   Weight by simulated time within the search window. Check whether large
   bundles actually form before attributing similar results to insensitivity.
5. Audit all runs for early termination, missing odometry, large odometry jumps,
   stalled robots and missing intent/assignment logs. Never select repeats by
   score. Retain original attempts and report faults separately; a mission's
   automatic startup retry is not an independent experimental replicate.

## Reproduction

`osmo/missions/generate_bundle_study_20261014.py` generates all four mission
files and `bundle_study_20261014.json` with exact scene descriptors and trial
membership. It uses the validated October 12 startup sequence and original
three-robot poses. Scene order is deterministically shuffled and identical
across variants. Each job records odometry, GT, auction tables and per-robot
auction-solve diagnostics in addition to the usual logs.

Push code and specifications before submitting. The workflow clones
`multi-raven`; record its resolved commit for every job. Results go into the
existing `/volume4/dsta/raven-ablations/` share under distinct mission names.

```bash
./airstack.sh osmo:mission osmo/missions/raven_bundle_auction_l1_s1_20261014.yaml \
  --workflow osmo/workflows/airstack-mission-6h.yaml \
  --pool dsta --branch multi-raven --no-keep-alive --priority HIGH
```
