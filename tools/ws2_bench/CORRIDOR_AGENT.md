# Adaptive corridor testing

Run from your own AirStack checkout. Choose the [local Docker runtime](README.md)
or [remote OSMO runtime](osmo/README.md). The campaign and LLM run on your
computer in both modes; the execution backend only changes where flights run.

## Sign in with your own account

Install the selected official CLI, then authenticate as the **same OS user**
that starts the viewer/campaign. Use your own ChatGPT account with Codex access
or your own invited Claude Team/Enterprise account with Claude Code access:

```bash
# Codex
npm install -g @openai/codex
codex login
codex login status

# Or Claude Code: install using https://code.claude.com/docs/en/setup
claude auth login --claudeai
claude auth status
```

Do not copy another person's login files, share a home directory, run the viewer
with `sudo`, or place tokens in Git/YAML/images. The bench calls `codex exec`
or `claude -p` using that user's saved official login. API-key/provider conflicts
fail visibly rather than switching billing. Random/Search requires no LLM login.
Usage counts against the signed-in account's limits; model availability and
Claude Code access depend on that person's plan/organization permissions.
References: [Codex authentication](https://developers.openai.com/codex/auth),
[Codex non-interactive execution](https://developers.openai.com/codex/noninteractive),
[Claude authentication](https://code.claude.com/docs/en/authentication).

Check your selected account without making an LLM request or starting a job:

```bash
.venv-ws2/bin/python tools/ws2_bench/check_install.py --planner pulp_dronet \
  --execution-backend osmo --osmo-config robot/ros_ws/ws2_runtime/osmo.yaml \
  --check-accounts --provider codex
```

Use `--provider claude` for Claude; for local flights use `--execution-backend
local` and omit `--osmo-config`. This verifies authentication, not model access
or quota for a future inference request.

## Run one target

```bash
python3 tools/ws2_bench/agent_campaign.py \
  --output robot/ros_ws/ws2_runtime/campaigns/my_corridor_campaign \
  --planner mononav --policy agent_search --provider codex \
  --action-space corridor --budget 12 --clean-validation-runs 0 \
  --infrastructure-retries 0 --clean-failure-policy record --review-seconds 0
```

For OSMO, prefix that command with `WS2_EXECUTION_BACKEND=osmo
WS2_OSMO_CONFIG="$PWD/robot/ros_ws/ws2_runtime/osmo.yaml"`. For local flights,
use `WS2_EXECUTION_BACKEND=local`. The selected provider never runs inside the
GPU job. Use a new campaign directory when changing backend or runtime image.

`--planner`: `mononav`, `kim`, `pulp_dronet`. One target per campaign.
`--provider codex`: saved ChatGPT subscription, default `gpt-6-astra` / `high`.
`--provider claude`: saved Claude subscription, default `sonnet`;
use `WS2_CLAUDE_MODEL` to select an available model. The viewer exposes both choices.
Optional Codex settings: `WS2_CODEX_MODEL`, `WS2_CODEX_EFFORT`,
`WS2_CODEX_TIMEOUT_S` (default600s). API-key/provider environment conflicts are
rejected instead of silently switching billing. Keep tokens out of YAML/Git.

Run the account watcher independently of flights:

```bash
python3 tools/ws2_bench/codex_quota.py --watch --interval 30
```

At weekly remaining<=1%, it stops new Codex calls. If you have configured your
own optional `~/notify_slack.py`, it also sends three notifications through that
helper; no author-specific webhook is shipped. Each Codex call checks quota. No new calls start at low
or unknown quota. Reset and resume the same campaign, or start a new campaign
with Claude. Provider changes never rewrite the original campaign identity.
`--policy`: `random`, `search`, `agent_search`. Search is a local heuristic,
not Bayesian optimization. Both baselines have rule-based confirmation,
single-factor isolation and scalar bisection; Claude chooses its follow-ups. Search simplifies the shared layout after a failed
clean control; it does not try to fix clean failures by lowering attack settings.

The budget counts reserved clean/attack slots. Twelve slots permit six pairs.
Standard corridor campaigns skip the attack twin after a failed clean control;
the unused slot is not reassigned. `record` continues to the next configuration,
while `halt` stops the campaign. The separately configured matched pilot runs
both flights even after a model clean failure, excluding invalid clean pairs
from attack attribution. Infrastructure failures are not model failures.
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
- New `corridor_agent_v2` actions use layout5: each obstacle specifies `route_s`,
  `lateral_m`, `width_m`, `depth_m`, `height_m`, `yaw_deg` relative to the route.
  Validators check rotated geometry, walls, other boxes and a continuous free
  route envelope. Existing v3/v4 geometry remains replayable. Width/curvature
  remain fixed in this version; these dimensions do not imply measured difficulty.
- `noise_schedule` / `delay_schedule`: `{start_s,duration_s,period_s}` in
  simulation seconds from the first accepted planner trajectory. Duration0 means
  remainder, period0 means one-shot; repeating windows require finite duration.
  Captured-frame metadata verifies actual pixels/age and exposure. A flight that
  ends before activation is not evidence of an attack effect. These are time
  windows, not route-progress-triggered attacks.
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

The current structured providers can inspect saved evidence before choosing a
flight. `agent_exploration.py` requests bounded inference images/GT trajectory,
records testable hypotheses and predicted outcomes, and allocates remaining pairs
to exploration, confirmation and diagnosis. Only the next action is executed;
the rest of a proposed plan is revised after feedback. Up to two evidence queries
per next-test decision bound LLM overhead. No file/shell/flight tools are exposed.
`agent_evidence/`, `llm_calls/`, `hypothesis_predictions.json` and each episode's
`visual_evidence.json` preserve what was shown and predicted. Image hashes prevent
mixing another run's preview or mismatched worker metadata. Prediction matches
and self-reviews are not independent validation of a causal explanation.

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

Report annotations identify design issues; they are not defense-effectiveness
measurements. Use a fresh campaign directory after changing prompts; completed
studies retain their original results and frozen source snapshots outside Git.

## Prompt locations

- `agent_exploration_v2.py`, `SYSTEM`: current experiment planning prompt;
  appends `experiment_protocol.py` and `planning_contract.py` instructions.
- `agent_analysis.py`, `write_analysis`: final interpretation and unvalidated
  defense proposals. `matched_policy.py` contains the matched diagnostic prompt.
- `codex_provider.py` / `claude_provider.py`: provider transport and authentication.
- Each campaign's `llm_calls/*.json`: exact `request.system`, `request.input`,
  `request.schema`, attached-image metadata and actual response/usage.

The context is generated from that campaign's bounds, budget and measured history;
editing a provider's name does not replace the experiment prompt.
