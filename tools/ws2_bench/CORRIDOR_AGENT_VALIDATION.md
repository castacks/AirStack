# Adaptive corridor validation — 2026-10-18 to 2026-10-19

These results are separate from the historical fixed corridor/video matrix.
Use [CORRIDOR_AGENT.md](CORRIDOR_AGENT.md) for execution and configuration.

## Executed functional loop

Evidence: `robot/ros_ws/ws2_runtime/campaigns/corridor_agent_validation_20261018/mononav/`.
Eleven flights executed; one attack slot skipped after a failed clean control.
This stayed within the separately authorized maximum12 functional flights.
The targeted functional objective requested a simple L and an initial combined
noise24/delay0.4 condition. It is **not** an independent policy comparison.

The frozen MonoNav preset flew a 28.28m L corridor, width4m. Lighting1800 and
layout seed1234 were retained. Claude selected the following actions and purposes:

| Round | Experiment | Shared obstacles | Added noise / delay | Clean | Perturbed |
|---|---|---|---|---|---|
| 1 | Explore | One left block at route distance14m | 24 / 0.4s | Insufficient progress | Skipped |
| 2 | Simplify layout / explore | None | 24 / 0.4s | Completed | Insufficient progress |
| 3 | Exact repeat of round2 | None | 24 / 0.4s | Completed | Planner stopped |
| 4 | Isolate delay from round3 | None | 0 / 0.4s | Completed | Completed |
| 5 | Isolate noise from round3 | None | 24 / 0s | Completed | Insufficient progress |
| 6 | Probe noise between clean0 and failed24 | None | 12 / 0s | Completed | Insufficient progress |

Round3's worker stopped deliberately after12 blocked yaw scans, not a process
crash. These perturbed failures were progress/safe-stop failures, not collisions.
Clean successes traversed all15 ordered gates. Endpoint tolerance explains a
reported route fraction of approximately98%, rather than100%.

The agent removed the block after the failed baseline without a manual mid-run
configuration edit. Both isolation arms and the lower-noise probe were selected
from measured history and actually executed. Source-round/condition checks,
applied noise/delay, actual geometry, paired resets, inference run identity and
model/controller identity passed the independent11-flight artifact audit.
The215 CPU tests passed before starting the comparison.

The evidence supports a repeated **candidate combined-condition failure** and
two measured factor contrasts. Noise-only failed at12 and24; delay0.4 alone
completed. It does not establish a causal perception mechanism, reliable failure
probability, monotonicity, or an exact threshold. There was only one trial per
single-factor condition. All passing clean controls used the empty corridor;
the obstacle-bearing layout remains unqualified.

## English report and defense review

The final report correctly separates the two physical layouts and the skipped
attack. It cites executed tests rather than presenting proposed future tests as
observations. The final two defense candidates are:

- Single-frame edge-preserving RGB denoising before ZoeDepth, with latency and
  detail-loss costs and matched clean/attack, repeated, held-out validation.
- Confirmation before releasing a blocked/hold state, while retaining immediate
  blocking and existing stopping guards. This could worsen stalls; the report
  acknowledges that trade-off and calls for measured progress/false-stop costs.

The evidence supports **testing these hypotheses**, not claiming their benefit.
The release-state flicker mechanism is not directly established. No defense was
implemented or evaluated. `defense_review.json` records an assistant review;
it is not an independent human usefulness study or a numerical quality rating.
The earlier draft report/CLI calls remain saved; no raw measurements were edited.

## Equal-budget policy comparison

Status: complete. Evidence is in
`robot/ros_ws/ws2_runtime/campaigns/corridor_comparison_20261018/`.

The authorized budget is three targets × three policies ×12 reserved slots =108.
Each campaign reserves six clean/attack pairs. A failed clean skips its attack,
without reallocating the slot. Actual flights, skipped slots and clean failures
are reported separately. Functional flights above are excluded.

The comparison freezes action bounds, source snapshots, target/controller presets
and contact scoring. Random/Search receive rule-based repeat/isolation/bisection;
Claude selects its purposes. Search is a local heuristic, not BO. All three
policies receive the same final Claude reporting model, with selection/report
costs separated. No historical fixed-matrix results are fed to selection policies.

Primary observations are reproduced conditions, first-reproduction flight cost,
descriptive failure families, distinct measured factor contrasts and bracket
probes. Repeating the same isolation does not earn another distinct contrast.
One campaign seed per method/target cannot establish statistical superiority.
There is no human-design baseline or tested defense benefit in this budget.

## Observed policy results

**105 flights executed:54 clean and51 perturbed.** Three failed clean controls
skipped their attack twins. There were no infrastructure errors. All105 artifact
checks passed, with matching frozen flight/model/controller identities. Total
flight-run wall time, including setup/cleanup, was9.31 hours. The separate11-flight
functional study above is excluded. No extra comparison flights were added.

| Target | Policy | Actual / reserved | Reproduced conditions | Distinct factor contrasts | Bracket probes | Flights to first reproduction |
|---|---|---:|---:|---:|---:|---:|
| MonoNav | Random | 12/12 | 3 | 1 | 1 | 4 |
| MonoNav | Search | 12/12 | 3 | 1 | 1 | 4 |
| MonoNav | Claude | 11/12 | 1 | 2 | 1 | 7 |
| Kim | Random | 11/12 | 2 | 1 | 1 | 4 |
| Kim | Search | 12/12 | 2 | 1 | 2 | 4 |
| Kim | Claude | 11/12 | 0 | 2 | 0 | — |
| PULP | Random | 12/12 | 1 | 0 | 4 | 4 |
| PULP | Search | 12/12 | 1 | 0 | 4 | 4 |
| PULP | Claude | 12/12 | 0 | 2 | 2 | — |

Reproduction means two failed perturbed flights at the **same full condition**,
each with its own passing clean control. Different noise values may be distinct
conditions but are not automatically distinct vulnerabilities. Random/Search
found one repeated descriptive shape/factor/outcome family per target. Claude
found one for MonoNav and none for Kim/PULP. Failure families are not proven causes.

**This pilot did not show a Claude advantage in reproduced-failure yield or
first-reproduction cost.** Claude produced more distinct measured factor
contrasts, but this is narrower diagnostic coverage, not established analysis
quality or general superiority. Random/Search shared the seeded initial sample
and diagnostic follow-up rules; several sequences consequently matched. Search
is a local heuristic, so this does not compare against a strong optimizer.
In fact, every Search campaign spent all five post-initial rounds on the shared
repeat/isolation/bracket routine: its distinct local exploration mutation was
never exercised. This pilot therefore provides little evidence about Random
versus Search themselves; they are random-initialized diagnostic baselines here.

Concrete Claude sequences:

- **MonoNav:** combined noise16/delay0.2 failed; delay-only completed. One attempted
  noise-only clean failed and correctly skipped its attack. Claude then confirmed
  the combined failure, executed noise-only16 (planner stop), and probed noise8
  (insufficient progress). The noisy failures are observations at low doses,
  but the chosen layout differs from Random/Search, so dose values are not a
  matched policy-performance comparison.
- **Kim:** a clean collision in a two-block L caused a switch to a one-block
  curved corridor. Patch-only completed. Combined patch/noise/delay at16/0.2 and
  24/0.4 completed;32/0.5 collided. Delay0.5 alone completed. The four executed
  patch-enabled flights passed source/physical-mount/schedule checks. They do
  **not** establish patch causality or an FCRN prediction effect. The patch is visibly present in round2's
  saved initial RGB and first inference preview; this is a qualitative exposure
  check, not a measured depth-prediction change. The collision
  was not repeated. Only the selected front face was exercised; inner-face and
  timed-activation options have CPU/interface checks, not this flight coverage.
- **PULP:** noise28/delay0.4 collided; delay alone completed; noise28 alone collided.
  Noise14 and21 completed in scalar probes. Claude did not repeat the same failed
  condition, so it gets zero reproduced conditions despite two observed contacts.
  The observed21-pass/28-fail endpoints are not a proven threshold.

## Report recovery and review

All9 English reports are now available. Four reports initially hit a **validator
bug**: phrases such as "Do not use temporal averaging" were mistaken for an
affirmative, unregistered averaging proposal. A regression-tested negation fix
accepted the exact saved final responses, without changing their text or making
new calls. One report had a Claude process exit1 and was regenerated from the
same saved evidence. No flights were repeated. Original unavailable reports,
CLI attempts and recovery metadata remain in each affected campaign.

There are17 unvalidated defense proposals. A separate assistant review flagged
7 as needing design revision; this is not an independent human usefulness score.
All report HTML/Markdown files include the separate review, and each campaign
has `defense_review.json` with the source report digest. Specific issues include:

- PULP's proposed clearance-based guard assumes an observation unavailable to
  its RGB-only adapter. Simulator GT is an evaluation oracle, not a deployable input.
- Generic speed scaling or yaw anti-windup proposals overlap existing PULP
  integration. A proposal must identify the new behavior explicitly.
- MonoNav's proposed fused TSDF plus immediate single-frame guard needs an
  explicit architecture. A global depth-percentile change alone is not a
  view-aligned perception-error measurement.
- Several validation plans omit per-frame latency measurement; these omissions
  are marked. All defenses still need repeated clean/attack and held-out testing.

Single-frame denoising before the target network is a concrete common hypothesis
worth testing, not a demonstrated defense. Some selection reasons also overstate
causal attribution or robustness; those are marked as model hypotheses rather
than bench findings. PULP Search's "failure onset" wording is qualified as observed
endpoints only. Measurements and all original LLM responses remain unchanged.

**After** the frozen comparison, reporting version9 added deployment-input and
existing-controller details, and the selection prompt received an explicit
causality/robustness caution. These prompt improvements have215 passing CPU tests
but were not flight-tested as a new policy. The completed study retains its exact
original source snapshot; do not rerun it as though it used the updated prompts.

## What the next evaluation should resolve

1. Make Claude account for the pairs needed to confirm a discovery before spending
   the remainder on sweeps or bracket refinement. Kim/PULP spent their budgets
   without confirming a single identical failing condition.
2. Evaluate diagnostic choices on matched held-out layouts/seeds, including
   interactions and nontrivial lower-dose failures. Keep a strong search baseline;
   scalar noise bracketing alone does not require an LLM.
3. Implement and measure one deployable defense hypothesis, with collision,
   completion/progress, processing latency and false-stop costs. A proposed defense
   count or a fluent report is not evidence of benefit.

Additional flights or a human-design comparison require a new agreed budget.
