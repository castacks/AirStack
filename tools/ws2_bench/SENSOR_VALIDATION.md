# Sensor effects in Office avoidance challenges

This stage validates the bench and measures sensor perturbations. It does **not**
run the next Random/Search/Claude superiority comparison. Patches are disabled
for both MonoNav and Kim. Rui's trained patch is only for FCRN/Kim; no ZoeDepth
patch attack is available.

## Scenes and controls

`challenge_layouts.py` adds `offset_obstacle`, `slalom` and `offset_gap` alongside
the previous protected straight corridor. An unchanged Office column blocks the
nominal center line in the first two, while the gap is approximately 0.88 m wide.
Surrounding plants retain seeded placements; walls/furniture stay unchanged.
The challenge columns have fixed coordinates, **not arbitrary agent-generated
XYZ**. Each configuration saves all realized offsets and a geometry hash.

The generator checks non-overlap, floor support, launch/goal clearance and a
0.2 m grid route at 1.2 m height inflated by a 0.35 m vehicle radius. This is a
geometric feasibility check, not proof of model/controller success. The route
is evidence only; neither model is commanded to follow that reference route.
Clean qualification is still required. Kim has no goal and is scored over its
original 120 simulation-second avoidance horizon, not arrival at the grid goal.

`sensor_study.py` runs a clean/noise pair followed by a new clean/delay pair.
Sigma is the Gaussian standard deviation in RGB pixel units (0–255), before
JPEG compression and before ZoeDepth/FCRN inference. Delay is additional camera
delivery time in simulation seconds. Lighting/geometry/seed/mission are identical
within each pair; clean disables noise, delay and patch. Defaults test sigma 32
and delay 0.5 s separately. They are exploratory bounds, not CyLab-approved limits.

Failed clean controls stop this study before the paired attack. Reports preserve
the failure, without interpreting it as an attack effect. Optional confirmation
repeats a failing sensor condition once with another clean control. All such
flights count as validation; none are Agent/Random/Search performance results.

## Run and inspect

From the AirStack root, after reserving the local GPU:

```bash
python3 tools/ws2_bench/sensor_study.py \
  --planner mononav --challenge offset_obstacle --seed 42 \
  --output robot/ros_ws/ws2_runtime/campaigns/mononav_sensor_validation
# Change --planner to kim for a separate study.
# --confirm-failures adds one matched repeat for each failing condition.
# --factor rgb_noise or --factor delay restricts a new study to one factor.
python3 tools/ws2_bench/dashboard.py
```

Open `http://127.0.0.1:8892`. It shows real scene/inference, controls and the
current report. The study records `scenario.json`, `provenance.json`,
`condition_evidence.json` (including measured RGB noise RMSE), trajectory samples,
planner logs/commands, contact/termination evidence and final inference images.
Later episodes also save camera age and sensor settings in trajectory samples.
Raw bags are omitted to limit disk use; `episode.py` can record before takeoff
when a bag is required. User records any presentation video.

Resume the exact same command to reuse completed results; changed settings are
rejected. `--reuse-clean PATH` accepts a prior qualification only when the entire
resolved configuration matches (except its descriptive name), and reuses it
only for the first pair. The report references that directory, without inventing
another flight. Historical pilot results remain unchanged.

## English defense proposals

After actual paired data exists, `agent_analysis.write_analysis` creates the
usual English Claude report with `defense_candidates`. Each entry requires:
proposal, failure hypothesis, rationale, tradeoffs, validation plan, cited
existing round IDs and `status: unvalidated_proposal`. Cited rounds must appear
in a finding; a fabricated ID or validated-defense status is rejected.
Empty proposals are allowed when evidence is insufficient. The LLM does not
change measured outcomes, implement defenses, or claim causal proof.

Read `llm_analysis.html` (also `/llm-report`) beside the deterministic
`vulnerability_report.html`. Model/API failure does not erase flight results.
This schema version invalidates only cached interpretations, not saved metrics.

Actual validation results and boundaries are recorded in `VALIDATION.md`.
