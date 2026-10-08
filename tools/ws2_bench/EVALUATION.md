# Expanded WS2 agent and policy comparison

All LLM prompts, hypotheses, selection reasons and generated reports are English.
The lab Team login is used through the official Claude CLI; no token is copied.

The completed2026-10-06 pilot used4MonoNav flights per policy,12total. Each policy
had2/2clean successes and2/2attacked successes; none found a failing condition.
This does not demonstrate a Claude advantage. Separate reference qualification
passed twice. See [VALIDATION.md](VALIDATION.md) for exact settings, evidence,
report revisions and limits. This result concerns a protected straight8m Office
route, not arbitrary navigation missions.

## Current sensor-only profile (supersedes pilot patch selection)

The new expanded profile excludes patch for both models and adds
`scene_challenge: protected|offset_obstacle|slalom|offset_gap`. Challenge columns
have fixed coordinates; seed changes surrounding props. Offset obstacle requires
Easy density, slalom/gap require Medium; corridor/side bias must be 1.2/0.
New placement schemas/hashes reject changed resume configurations. Existing pilot
artifacts are historical and remain untouched. See [SENSOR_VALIDATION.md](SENSOR_VALIDATION.md)
and [AGENT_EVALUATION_PLAN.md](AGENT_EVALUATION_PLAN.md). The next policy comparison
is deferred until the user reviews that proposal.

## Common test space

Random, result-guided search and Claude can use the same continuous bounds:

| Parameter | Range / meaning |
|---|---|
| Placement seed | 0..2^31-1; generates new coordinates, not a saved layout ID |
| Density | Easy:1, Medium:3, Hard:5 additional plants **and** columns |
| Protected corridor half-width | 0.7..1.2m around the nominal 8m route |
| Side bias | -1..1; preference for one side of the corridor |
| RGB noise sigma | 0..32 in 8-bit pixel units (0..255), Gaussian |
| Dome intensity | 800..2400; fill-light intensity is 5.5 times this value |
| Additional camera delay | 0..0.5 simulation seconds |
| Patch | Disabled in the current expanded profile for both models |
| Patch activation | Zero while disabled; old pilot fields retained only for historical interpretation |

These are local exploratory test bounds, not finalized CyLab attack limits.
Depth models still infer from RGB: RGB noise is not a direct perturbation of
the predicted depth tensor. Larger patch size does not guarantee a stronger
attack. Rui's current attack is for FCRN / Kim et al. only; no MonoNav/ZoeDepth
patch attack has been developed. Deployed FCRN model/preprocessing parity and
closed-loop effects still require validation. The existing MonoNav pilot
displayed the FCRN texture and is not a ZoeDepth-targeted patch evaluation.
Target-based patch filtering now rejects MonoNav patch episodes before launch.
The new expanded sampler/schema excludes patch for both targets.

`generated_layouts.py` samples continuous coordinates for one movable existing
plant and additional Office plants/columns. It checks object bounds, floor
support, intersections, the permitted lobby area and a protected nominal flight
corridor. Structural walls remain fixed. The small `office_geometry.json` records
USD-derived bounds and the exact Office asset hash. The simulator checks that
hash before using generated layouts. Geometry checks do not guarantee a policy
can fly through a scene; paired clean controls remain necessary.

Every selected action saves the explicit realized offsets, seed, generation
parameters, bounds and placement identity. Resume uses those saved coordinates.
Clean/attack twins share the exact placement and illumination; clean disables
only sensor noise, additional delay and patch. Lighting/geometry failures in
clean are reported separately, not as added sensor/patch effects.

## Run an expanded campaign

In the existing viewer, choose Random, Search or LLM-guided tests, then
**Generated scenes + noise + light + delay + patch**. Claude mode uses
**Claude · Team login**. The saved-layout delay/patch mode remains available.

```bash
python3 tools/ws2_bench/agent_campaign.py \
  --action-space expanded --policy agent_search --provider claude \
  --planner mononav --budget 4 \
  --clean-validation-runs 2 --infrastructure-retries 0 \
  --clean-failure-policy record \
  --output robot/ros_ws/ws2_runtime/campaigns/expanded_claude
```

The two clean qualification flights are outside this example's four-flight
comparison budget. Use zero extra checks only when the reference has already
been qualified. A campaign remains single-target. The generated-space MonoNav
mission is the protected8m route at0.3m/s, radius0.5m, maximum180sim seconds.
Kim keeps its existing reactive mission, but this pilot evaluates MonoNav only.

## Fair pilot protocol

`compare_policies.py` checks two independent successful reference clean flights
and their saved artifacts before starting a comparison. It writes a fixed
protocol before executing any comparison flight. It gives each policy the same
mission, parameter bounds, seed and number of paired flights. No automatic
infrastructure retries or extra clean-validation flights occur inside the study.

Random samples uniformly within the numerical bounds. Search starts from the
same seed and mutates a previous condition based on measured clean/attack
outcomes. Claude chooses exact bounded values after reading the same type of
result evidence. All three policies repeat a clean-pass/attack-fail candidate
once when budget remains; this repeat consumes the ordinary flight budget.

```bash
python3 tools/ws2_bench/compare_policies.py \
  --flights-per-method 4 --seed 17 \
  --qualification robot/ros_ws/ws2_runtime/campaigns/QUALIFICATION_DIRECTORY \
  --output robot/ros_ws/ws2_runtime/campaigns/COMPARISON_DIRECTORY
```

The comparison reports clean successes/failures, valid paired attack failures,
distinct candidate settings, repeated settings, flights to the first candidate,
infrastructure errors, flight lifecycle time, LLM calls/tokens/latency and CLI
list-price estimates. Subscription dollar estimates are not an invoice.
Call totals retain report-only revisions as well as the original analysis.
Distinct settings do not necessarily mean distinct underlying failure causes.

Four flights per method is only two pairs: it is a pilot, not evidence of
statistical superiority. All three policies already automate configuration
selection; this pilot cannot establish reduced human effort for Claude.
Do not hide unsuccessful runs, clean failures or an absence of an agent advantage.

The viewer exposes **Open comparison** and the English LLM request/response log.
Per-method deterministic and Claude reports remain separate. Comparison artifacts
are `comparison.json`, `.csv`, `.md` and `.html`; raw evidence stays in the ignored
runtime tree. Generate reports from existing results without more flights:

```bash
python3 tools/ws2_bench/compare_policies.py \
  --output robot/ros_ws/ws2_runtime/campaigns/COMPARISON_DIRECTORY --report-only
```
