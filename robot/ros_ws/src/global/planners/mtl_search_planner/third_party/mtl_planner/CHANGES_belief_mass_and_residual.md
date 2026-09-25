# Change note for downstream integrators (written for an LLM agent)

**Package:** `cpp_planner/` (namespace `mtl`, libraries `mtl::planner`, `mtl::mapgen`, `mtl::eval`, tools `mtl_demo`, `mtl_plan`)
**Date:** 2026-09-25
**Scope:** two changes to the C++ planner. The folder is being copied **as is** into another simulation stack. Use this note to find and update every place in the *rest* of that stack that depends on the old behaviour. The code inside `cpp_planner/` is already consistent and tested (5/5 `ctest` suites pass), so do **not** edit it unless a task below says so.

1. **The belief prior is now a probability mass function (it sums to 1).** Cells are kept by **per-cell belief mass**, not by mean belief. `meanInformationThresh` is renamed to `minimumBeliefMass`, and what it means has changed.
2. **New post-search metric: residual belief.** `mtl::eval::computeResidualBelief` Bayes-updates every pixel of the prior against every sensor look. It returns `residualMass = P(target missed by the search)`. **Lower is better.** This is the number used to compare planners.

The rest of this note covers: §1 breaking changes and a checklist, §2 exact semantics of change 1, §3 exact semantics of change 2, §4 tools, files and build, §5 how to update a host stack, §6 verification facts.

---

## 1. Breaking changes: quick checklist

Search the host stack for each item in the left column.

| Search for | What happened | What to do |
|---|---|---|
| `meanInformationThresh` (C++) | **Removed** from `mtl::PlannerParams` and replaced by `double minimumBeliefMass = 5e-5;` | Rename. Do **not** carry the old value over (see §2.3). |
| `mean_information_thresh` (JSON / YAML / Python / docs) | `mtl_plan` no longer reads it. It now reads `mapping.minimum_belief_mass`. The old key is **silently ignored**. | Rename the key and give it a value that makes sense as a probability (§2.3). |
| calls to `mtl::mapping::extractValidCells(belief, cellSize, X, verbose)` | The signature is unchanged in types, but the 3rd argument is now `minBeliefMass` (a per-cell probability), not a mean-belief threshold. | **Silent semantic break.** A call that still passes `0.005` compiles, but keeps only cells holding more than 0.5% of the whole map's belief, which is very few. Fix every positional call site. |
| code that reads `CellSet::mass`, `ClusterSet::reward`, `TeamInfo::info`, `InfoScore::info`, `totalMapMass`, `retainedMass` | Units changed. They were raw summed belief (× pixel area), with values of about 1e3 to 1e7. They are now **probabilities**: `totalMapMass == 1`, a cell's mass is about 1e-5 to 1e-2, and a team's info is ≤ 1. | Fix any hard-coded scale, axis limit, absolute threshold or log format that assumed the old magnitude. Fractions (`infoFraction`, `%` columns) are unaffected. |
| code that reads `CellSet::meanBelief` / `peakBelief` | Now taken from the **normalised** prior. `meanBelief` is `mass / nPix`. It is no longer the thresholded quantity. | Do not use `meanBelief` as a filter. |
| `mtl::mapgen::generateBeliefMap` output | Values now sum to 1, and the maximum pixel is about 1e-7 on a 5001² grid (it was ≤ 0.85). It throws `std::invalid_argument` if the map has no mass. | Anything that expected probabilities in [0, 0.85] per pixel, such as colour maps or rendering heights, must rescale. |
| constructing `PlannerParams` with an out-of-range threshold | `validate()` now throws unless `0 <= minimumBeliefMass < 1`. | Make sure configs pass a probability. |
| downstream planner-comparison scripts | A new metric is available (§3). | Report `residualMass` next to (or instead of) detections and info%. |

**Unchanged:** the `mtl.scenario/1` and `mtl.plan/1` JSON schemas, apart from the one renamed input key. The plan output format, the frames (mission NED at the JSON boundary; x East, y North, z Up inside `mtl`), trajectories, gimbal scheduling, budgets and `Planner` entry points (`plan`, `planFromCells`, `planFromClusters`) are all the same. The route optimiser only compares rewards as ratios and relative gains, so plans do not depend on the overall scale of `mass`. They do change when the **set of kept cells** changes (§2.4).

---

## 2. Change 1: normalised prior and mass-threshold cell extraction

### 2.1 Prior normalisation
`src/mapgen/scenario.cpp` → `mtl::mapgen::generateBeliefMap(mapSize, cellSize, BeliefMapParams, seed)`:

```
values = sum_k maxPriorPeak * GaussianBump_k      (unchanged)
values = clamp(values, baseUncertainty, beliefCap) (unchanged)
values = values / values.sum()                     (NEW; throws if sum <= 0)
```

`BeliefMapParams` (`numCentroids`, `maxPriorPeak`, `sigmaMin/Max`, `beliefCap`, `baseUncertainty`) now only **shape** the prior. None of them sets its scale. `BeliefField::values(r, c)` is the probability that the target is in pixel (x = c·res, y = r·res). Rows index y and columns index x, as before.

### 2.2 Cell extraction
`src/mapping/cells.cpp` → `CellSet extractValidCells(const BeliefField&, double targetCellSize, double minBeliefMass, bool verbose)`:

- It first normalises internally: `norm = 1 / belief.values.sum()`. This does nothing to an already-normalised prior, and it means **a host may pass any non-negative grid** and still get probabilities. (Test: a grid scaled by 7.5 gives identical cells and masses.) An empty or all-zero grid returns an empty `CellSet`.
- Blocking is unchanged: `targetCellSize` blocks, with **ceil** so the partial blocks on the far edge are included, and the centre is the mean of the pixel coordinates actually covered.
- Per block: `mass = block.sum() * norm`. The block is **kept iff `mass > minBeliefMass`**. It used to be `mean > meanInfoThresh`.
- Fields filled: `mass` (probability, without the old `*pixelArea` factor), `massNorm = mass/sum(mass)`, `meanBelief = mass/nPix`, `peakBelief = max*norm`, `nPix`, `area = nPix*pixelArea` [m²], `totalMapMass = 1` (up to round-off), `retainedMass = sum(mass)`, **new** `minBeliefMass` (the threshold that was used), `cellSize`, `gridRes`.
- The verbose output now reads `Extracted N valid cells (belief mass > T per cell).` and `retained 0.9980 of 1.0000 total prior mass`.

`Planner::plan(belief, starts)` calls it with `params.minimumBeliefMass`.

### 2.3 Choosing `minimumBeliefMass`
The threshold is the **probability of the target being in one cell**. It therefore depends on how many cells the map is diced into: for a similar prior it scales roughly with `targetCellSize² / mapSize²`.

- Reference mission (5000 m map, 200 m cells, so 26×26 = 676 blocks): default `5e-5`. That keeps about 400 to 490 cells retaining about 99.8% of the prior, close to what the old `meanInformationThresh = 0.005` kept. (Seed 21, C++ generator: 412 cells, 45 clusters, 99.80% retained.)
- **There is no exact conversion from an old mean threshold.** Calibrate each mission. Compute block masses on the normalised prior, then pick the threshold that keeps the cell count or retained-mass fraction you had before. A useful reference point is the uniform share, `1 / number_of_blocks`. The threshold should normally sit well below it (the reference default is about 3% of the uniform share of 1/676 ≈ 1.5e-3).
- Example of a scaled mission that needs recalibrating: the original stack's Gazebo testbed `configs/search_small.yaml` has a 400 m area, 20 m cells (400 blocks, uniform share 2.5e-3) and used `mean_information_thresh: 0.08` for "~143 valid cells holding ~86% of the mass". The value `5e-5` is **not** right there. Recompute it from that mission's prior.

### 2.4 Behavioural consequences
- Few-pixel partial blocks along the map edge used to pass on a high mean over a sliver of area. They are now dropped unless they hold real mass. Blocks with a lower mean but a large area can now be kept. So the cell set, cluster count and plans **change** relative to before, even though the objective is the same.
- If a host uses `planFromCells` with its own masses, any positive scale still plans identically. **Recommendation:** send probabilities (the normalised prior summed per cell). Printed info, efficiency and the residual metric then share units. Avoid extremely small absolute masses (below about 1e-7 per cell): a few solver tie-break tolerances are absolute (1e-9 / 1e-12 on reward).

---

## 3. Change 2: residual belief (the planner-comparison metric)

### 3.1 API (`include/mtl/eval/detection.hpp`, library `mtl::eval`)
```cpp
struct ResidualBelief {
    MatX   residual;            // same shape as prior.values: P(target at pixel AND every look missed it)
    double priorMass    = 0.0;  // 1 (the prior is normalised internally)
    double residualMass = 0.0;  // residual.sum() = P(the search missed the target)  <-- THE METRIC, lower is better
    double detectedMass = 0.0;  // priorMass - residualMass
    Index  nLooks = 0;          // distinct sensor states integrated
    Index  nSteps = 0;          // trajectory steps integrated (all agents)
    MatX posterior() const;     // residual / residualMass : Bayes posterior given no detection
};

ResidualBelief computeResidualBelief(const BeliefField& prior,
                                     const std::vector<AgentTrajectory>& trajectories,
                                     double fov, const SensorModelParams& sensor);
```
`include/mtl/eval/report.hpp` adds `void reportResidualBelief(std::ostream&, const ResidualBelief&);`. It prints the prior mass, the residual mass and the searched mass with its percentage.

Typical call (as in `apps/demo_pipeline.cpp`), after planning:
```cpp
const mtl::PlanningResult r = planner.plan(belief, starts);
const mtl::eval::ResidualBelief rb =
    mtl::eval::computeResidualBelief(belief, r.trajectories, params.fov, params.sensor);
mtl::eval::reportResidualBelief(std::cout, rb);
```

### 3.2 Exact model
This must be reproduced bit-for-bit if you re-implement it in another language. `P(Z|x)` is **identical** to what `eval::updateTargetDetectionProbs` applies to targets. For each agent `a` and each time step `k` of the delivered trajectory (`drone(k) = [px, py, h]`, `sensor(k) = [lx, ly]` = boresight ground point):

```
slantLook = sqrt((px-lx)^2 + (py-ly)^2 + h^2)
radius    = slantLook * tan(fov/2)                       # footprint follows slant range to the boresight point
for every pixel x = (X, Y) with (X-lx)^2 + (Y-ly)^2 <= radius^2:    # inside footprint (inclusive)
    d3 = sqrt((X-px)^2 + (Y-py)^2 + h^2)                 # slant range aircraft -> pixel, ground at z = 0
    p  = (d3 > beta) ? sensor.pOutOfRangeMulti : 1 / (a + exp(b*(d3 - c)))
    logMiss(x) += log(1 - p)
pixels outside the footprint: no update (p = 0)

prior_n      = prior / sum(prior)
residual(x)  = prior_n(x) * exp(logMiss(x))
residualMass = sum_x residual(x)
```
Notes that matter for matching:
- Pixel coordinates are `X = c * gridResX`, `Y = r * gridResY`, with the map spanning `[0, mapSize]` (mtl frame, x East, y North). The metric only uses distances, so the frame only has to be consistent between trajectories and grid.
- **Every trajectory step is one independent look.** The metric therefore depends on the sample period `dt`, exactly as the target detection model does. Compare planners **at the same `dt`**. The hover padding that `PlanningResult` appends to agents that finish early *does* count (it is part of the delivered timeline, and the target accumulator counts it too). Internally, runs of identical consecutive states are integrated once, weighted by the run length, which is mathematically identical.
- Implementation details that do not change the result: only the footprint's columns and rows are visited, and `log(1-p)` is tabulated over squared slant range `[0, beta²]` with 2^20 knots and linear interpolation (error far below round-off). If `beta` is non-finite or ≤ 0, it evaluates directly.
- Invariant, asserted in tests: at any target's pixel, `residual/prior_n == target.pMissTotal` from `updateTargetDetectionProbs` on the same trajectories (to 1e-9, observed 1e-13). Also `0 <= residual <= prior_n` everywhere, `residualMass + detectedMass == 1`, and with no trajectories `residualMass == 1`.
- Cost: O(looks × footprint pixels). Reference scenario (5001×5001 grid, 3 agents, about 7400 looks, footprint radius up to about 350 m): about 12 s on one core, about 10 s on 2 cores with `-DMTL_ENABLE_OPENMP=ON`. Memory: one extra `MatX` the size of the grid for the log accumulator plus the returned `residual` (about 200 MB each at 5001², doubles). `posterior()` allocates another.
- Reference values (C++ generator, seed 21, defaults, 250 s budget, 3 agents, 50° tilt single-axis): `residualMass = 0.115895`. With `--multi-axis`: `0.113224`.

### 3.3 Where it lives
`computeResidualBelief` is in **`mtl_eval`**, not `mtl_planner`. A host that links only `mtl::planner` (such as `mtl_plan`) does not get it. To score in C++, link `mtl::eval`. To score in the host language, re-implement §3.2 (reference numpy sketch in §5.3).

---

## 4. Tools, files and build

### 4.1 `mtl_plan` (JSON bridge, `apps/mtl_plan_json.cpp`)
- Reads `scenario["mapping"]["minimum_belief_mass"]` into `PlannerParams::minimumBeliefMass` (default 5e-5). `mean_information_thresh` is no longer read, and it is ignored if present.
- The value is **informational** for `mtl_plan`: that tool plans from host-supplied `cells.centers` / `cells.mass` (`planFromCells`) and never re-extracts cells. It is still validated, so it must be in [0, 1).
- `cells.total_map_mass` (optional, defaults to the sum of `cells.mass`): send `1.0` if your cell masses come from a normalised prior.
- Output `mtl.plan/1` is unchanged. `mtl_plan` does **not** compute the residual metric.

### 4.2 `mtl_demo` (`apps/demo_pipeline.cpp`)
New flags: `--min-belief-mass P`, `--no-residual`, `--residual-block N` (default 10). `--extend-dist` now appears in `--help`. After the detection, budget, gimbal and audit reports, it prints the residual block. With `--csv DIR` it additionally writes `DIR/residual_belief.csv`:
```
x,y,prior,residual        # one row per N x N pixel block (ragged edge blocks sum what they have)
                          # x,y = block centre [m, mtl frame]; prior/residual = block SUMS (probabilities)
```
The sums of both columns equal 1 and `residualMass` respectively. The other CSVs (`agent*_trajectory.csv`, `cells.csv`, `targets.csv`) have unchanged columns, but `cells.csv:mass` is now a probability.

### 4.3 Files changed inside `cpp_planner/`
| File | Change |
|---|---|
| `include/mtl/params.hpp` | `meanInformationThresh` → `minimumBeliefMass = 5e-5` (+docs); `BeliefMapParams` docs (normalisation) |
| `src/params.cpp` | `validate()`: `0 <= minimumBeliefMass < 1` |
| `include/mtl/types.hpp` | `BeliefField` docs (sums to 1); `CellSet` docs; **new field** `CellSet::minBeliefMass` |
| `include/mtl/mapping/cells.hpp`, `src/mapping/cells.cpp` | mass-threshold extraction, internal normalisation, mass without the pixel-area factor |
| `include/mtl/mapgen/scenario.hpp`, `src/mapgen/scenario.cpp` | normalise the prior; throw on zero mass |
| `src/planner.cpp` | passes `minimumBeliefMass` |
| `include/mtl/eval/detection.hpp`, `src/eval/detection.cpp` | **new** `ResidualBelief`, `computeResidualBelief` (+ internal `LogMissTable`) |
| `include/mtl/eval/report.hpp`, `src/eval/report.cpp` | **new** `reportResidualBelief` |
| `apps/mtl_plan_json.cpp` | JSON key rename (§4.1) |
| `apps/demo_pipeline.cpp` | residual pass, new flags, `residual_belief.csv` |
| `CMakeLists.txt` | when `MTL_ENABLE_OPENMP` is on, also defines `MTL_HAVE_OPENMP=1` for `mtl_eval` |
| `tests/test_pipeline.cpp` | new assertions (§6) |
| `README.md` | new sections: "The prior is a probability mass function", "Scoring a plan: residual belief"; parameter table and performance updated |

No files were added or removed from the CMake target lists, and there are no new dependencies (still Eigen ≥ 3.3, optional OpenMP).

---

## 5. How to update the host stack

### 5.1 Tasks (do in order)
1. **Rename config keys.** Everywhere the host writes `mapping.mean_information_thresh` (YAML configs, config dataclasses, the scenario-JSON writer, docs and runbooks), use `mapping.minimum_belief_mass` with a **recalibrated** probability value (§2.3).
2. **Make the host's own cell extraction match**, if the host extracts cells itself. The original stack did (`testbed/mission.py: extract_valid_cells`, which used `tile.mean() <= mean_information_thresh` and `mass = tile.sum() * pixel_area`). Change it to:
   ```python
   p = belief.values / belief.values.sum()          # normalise the prior
   mass = float(p_tile.sum())                       # per block, a probability
   keep = mass > spec.minimum_belief_mass
   ```
   Send `cells.mass` as these probabilities and `cells.total_map_mass = 1.0`. Keep your existing block tiling convention, or switch to the planner's (ceil, so partial edge blocks are included); just be aware which one you use. Update the "no valid cells" error message to name the new key.
3. **Normalise the host prior**, or be aware that it is not normalised. Anything that renders or samples the prior works either way, because sampling normalises. Anything that compares per-pixel belief to an absolute number must be revisited.
4. **Add the metric to run summaries.** After a mission (or a replay of a plan), compute `residualMass` from the *executed* (or planned) trajectories at the simulation's step `dt`, and log it as the headline planner score (lower is better). Either link `mtl::eval` and call `computeResidualBelief`, or port §3.2 (sketch below). For a like-for-like comparison, all planners must be scored on the same prior, the same sensor parameters (`fov`, `a`, `b`, `c`, `beta`, `pOutOfRangeMulti`) and the same `dt`.
5. **Fix displays and thresholds** that assumed the old mass magnitudes (§1 row 4).
6. Re-run the host's tests and update any golden numbers that involve cell counts, cluster counts, info masses or plans.

### 5.2 Frame conversion, if you score in the host (NED)
The model only uses distances. Using NED (n, e, d) directly: pixel (n, e), aircraft (n, e, h = −d), boresight ground point (n, e). The same formulas apply with (x, y) → (e, n). The ground is assumed flat at height 0.

### 5.3 Reference numpy sketch (equivalent to §3.2)
```python
import numpy as np
def residual_belief(prior, n_axis, e_axis, tracks, fov, a, b, c, beta, p_out):
    """prior[i, j] at (n_axis[i], e_axis[j]); tracks: list of (N,5) arrays [n, e, h, look_n, look_e]."""
    P = prior / prior.sum()
    logmiss = np.zeros_like(P)
    th = np.tan(fov / 2)
    for T in tracks:
        for pn, pe, h, ln, le in T:                       # one look per time step (dt-dependent)
            R = np.sqrt((pn-ln)**2 + (pe-le)**2 + h*h) * th
            i0 = np.searchsorted(n_axis, ln - R, side='left'); i1 = np.searchsorted(n_axis, ln + R, side='right')
            j0 = np.searchsorted(e_axis, le - R, side='left'); j1 = np.searchsorted(e_axis, le + R, side='right')
            if i0 >= i1 or j0 >= j1: continue
            dn, de = n_axis[i0:i1, None], e_axis[None, j0:j1]
            inside = (dn-ln)**2 + (de-le)**2 <= R*R
            d3 = np.sqrt((dn-pn)**2 + (de-pe)**2 + h*h)
            p = np.where(d3 > beta, p_out, 1.0 / (a + np.exp(b*(d3 - c))))
            logmiss[i0:i1, j0:j1] += np.where(inside, np.log1p(-p), 0.0)
    resid = P * np.exp(logmiss)
    return resid, resid.sum()                             # residual map, residual mass (lower = better)
```
(`n_axis`, `e_axis` must be ascending. Collapsing runs of identical consecutive rows and multiplying their `log1p` term by the run length is the main speed-up; restricting each column to the circle's chord is the next.)

---

## 6. Verification already done (so you can trust the package)
- `ctest`: 5/5 pass (`test_dubins`, `test_kmeans`, `test_orienteering`, `test_geometry`, `test_pipeline`), also with `-DMTL_ENABLE_OPENMP=ON`.
- New `test_pipeline` assertions: the prior sums to 1. Every kept cell has `mass > minimumBeliefMass`, `totalMapMass == 1`, and `sum(mass) == retainedMass`. A 7.5× scaled host grid gives identical cells. A 20× threshold keeps a strict subset. The residual with no trajectories equals 1. The residual is finite, non-negative, never above the prior, sums with `detectedMass` to 1, and `posterior()` sums to 1. At every target pixel, `residual/prior == pMissTotal`.
- Cross-language check: the MATLAB reference implementation (`functions/detection/computeResidualBelief.m` in the parent repo) and this C++ port give the same `residualMass` to 1e-12 on identical belief and trajectories.
- The numpy sketch in §5.3 was run against `computeResidualBelief` on a toy case (501² grid, 100 looks): 0.574168246842 in both.
- Build warnings: only the pre-existing `-Wshadow` notes in `apps/json_mini.hpp`. None are new.
