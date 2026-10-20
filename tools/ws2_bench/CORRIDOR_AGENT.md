# Adaptive corridor testing

Run from the AirStack checkout used by the existing simulator/robot containers.
Install the three target workers as described in their repositories and authenticate
with the official Claude CLI (`claude auth login --claudeai`). Authentication stays
inside Claude Code; the bench never reads or exports OAuth credentials.

## Run one target

```bash
python3 tools/ws2_bench/agent_campaign.py \
  --output robot/ros_ws/ws2_runtime/campaigns/my_corridor_campaign \
  --planner mononav --policy agent_search --provider claude \
  --action-space corridor --budget 12 --clean-validation-runs 0 \
  --infrastructure-retries 0 --clean-failure-policy record --review-seconds 0
```

`--planner`: `mononav`, `kim`, `pulp_dronet`. One target per campaign.
`--policy`: `random`, `search`, `agent_search`. Search is a local heuristic,
not Bayesian optimization. Both baselines have rule-based confirmation,
single-factor isolation and scalar bisection; Claude chooses its follow-ups. Search simplifies the shared layout after a failed
clean control; it does not try to fix clean failures by lowering attack settings.

The budget counts reserved clean/attack slots. Twelve slots permit six pairs.
A failed clean control skips its attack twin; the skipped slot is **not reused**.
Actual executed flights and reserved slots are reported separately. No model failure
is automatically retried. Rerun the same command to resume without repeating saved
flights; changed configuration is rejected. Use a fresh directory for a changed study.

Open the existing viewer at `http://127.0.0.1:8892`: select an adaptive policy and
`Corridors: shape / obstacles / attacks`. The viewer shows the real simulator,
model inference, chosen condition/reason, pause/resume/stop, measurements and report.
The corridor's frozen model preset overrides the Office time/goal controls.

## Configuration files

- `scenarios/corridor_{mononav,kim,pulp}.yaml`: fixed target settings. Corridor
  completion uses ordered gates and the end plane, up to 500 simulation seconds,
  with a 60-second no-progress stop. Kim/PULP receive no GT route or goal.
- `corridor_space.py`: shared selection bounds, JSON schema and baseline policies.
  L/curve/zigzag/U; 4m width; route scale2; original daylight textures. Agent chooses
  obstacle count and positions measured along the route (`route_s`, metres), and
  side (`+1` left, `-1` right). Easy0–2, Medium2–4, Hard4–7 are nominal count ranges,
  **not** proven difficulty ordering. Geometry version4 is distinct from fixed v3 tiers.
- `corridors.py`: versioned geometry, overlap/end-point checks and a continuous
  nominal 0.7m centreline envelope. Geometric feasibility does not guarantee clean flight.
- RGB Gaussian noise sigma0–32 (pixel0–255), additional delay0–0.5 simulation seconds,
  lighting1000–2200. These are local exploratory bounds, not finalized CyLab limits.
- **Kim only**: original opaque Rui FCRN patch. Select on/off, size0.3–0.9m,
  obstacle index, front/inner face, height and start/duration0–120 simulation seconds.
  The patch must fit its surface (inner face permits size≤0.63m). Duration0 means
  remaining mission. No opacity/contrast/color change. Size is not calibrated strength.
  No ZoeDepth or PULP attack patch is available.

Clean/attack share exact geometry, illumination, texture seed and target settings.
Clean disables noise, extra delay and patch. Every flight resets world and worker.
Stored scenario, actual geometry/hash, sensor/command/motion diagnostics and CLI
request/response/usage support replay and audit. No credentials are recorded.

## Follow-ups and reports

Claude can choose `explore`, exact `repeat`, `isolate` (retain one factor from a
combined test), or `threshold` (probe between observed pass/fail scalar values).
A conditional bracket may keep another factor active only when an actual passing
endpoint has that same fixed factor; an all-off clean control cannot anchor such a
conditional bracket. Follow-ups reference actual prior rounds; the validator rejects changes that would
confound a matched test. A bracket is an observation, not an assumed monotonic or
statistically established threshold. Final report suggestions do not authorize extra flights.

`vulnerability_report.*` contains bench measurements. `llm_analysis.*` contains
English interpretation, cited evidence, failure hypotheses, defense candidates,
trade-offs and validation plans. Every defense remains `unvalidated_proposal`.
`diagnostic_audit.json` lists actual isolation/bracket evidence and reference/field
checks for defenses. `audit_corridor_campaign.py` separately checks realized geometry, applied sensor
values, patch schedule/mount, paired settings and source/model identities. The
comparison stops if these checks fail. Neither audit automatically certifies semantic correctness or
prove that a defense works. Patch exposure/FCRN change must be measured before
claiming patch causality.

## Equal-budget comparison

```bash
python3 tools/ws2_bench/compare_corridor_policies.py \
  --output robot/ros_ws/ws2_runtime/campaigns/corridor_comparison \
  --flights-per-method 12 --targets mononav kim pulp_dronet
```

This reserves **108 slots**, with a separate 12-slot campaign for each target/policy.
No extra qualification/retry flights are added. Each pair has its own clean control.
The same Claude provider writes final reports for all methods; only agent mode uses
LLM selection. Reporting and selection costs are separated. Method order is seeded;
source hashes, model settings and bounds are frozen in `protocol.json`. Small
source snapshots are saved with the study so uncommitted development code remains
reviewable; weights, credentials and unrelated workspace files are excluded.

Results include reproduced conditions, descriptive failure families, first reproduction
cost, real isolation/bracket tests, clean failures, executed/skipped slots and resources.
One seed per policy/target is an initial comparison, not statistical proof. The search
baseline is a heuristic; a strong optimizer and held-out seeds are needed before
broad research claims. No human-performance comparison is implied.

The completed October18–19 pilot and separate defense review are documented in
[CORRIDOR_AGENT_VALIDATION.md](CORRIDOR_AGENT_VALIDATION.md). It did not establish
a Claude advantage. Report annotations identify design issues; they are not
defense-effectiveness measurements. Use a fresh campaign directory for the updated
prompts; the completed study retains its original frozen source snapshot.
