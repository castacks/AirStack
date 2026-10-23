# Expanded agent validation — October 22, 2026

Status: first 12 functional flights completed; all 12 finished their routes and
all 12 artifact audits passed. This is development
validation, **not evidence of superiority over Random/Search**.

Runtime evidence: `robot/ros_ws/ws2_runtime/campaigns/codex_extensions_20261022/`.
The original protocol and source snapshot precede the flights. Audit/transport
corrections have separate amendment files; original outcomes, prompts, proposals
and agent inputs are retained.

## Implemented path under test

- Selectable Claude or Codex; this study requests `gpt-6-astra`, `high`, through
  official `codex exec` and the saved ChatGPT subscription login. No API fallback.
- V2 feasible obstacle geometry (position, dimensions, orientation), plus
  simulation-time noise/delay windows. Corridor width/curvature remain fixed.
- Agent requests recorded inference/trajectory images, makes a falsifiable
  hypothesis, allocates pairs and selects the next validated experiment.
- Bench retains control, reset, scoring and evidence responsibility. Kim alone
  permits the original opaque FCRN patch. No defense intervention is tested.
- Official weekly quota monitoring; at≤1% remaining send Mason three Slack
  notifications and block new Codex calls. Unknown quota also blocks new calls.

## First functional observations

PULP completed all four flights: two clean controls and two scheduled 0.3s delay
flights on one curved corridor. After requesting actual images, Astra moved the
delay window from mission seconds2–37 to40–120. The second choice followed two
inspection rounds with2 and5 attached images. Both attack flights completed.
Clean minimum clearance varied0.710→0.439m; attacked minima were0.576/0.578m.
Single-pair differences therefore do not establish attack degradation.

Kim completed all four flights, including the exact repeat Astra requested after
inspecting clean and patch views. The original 0.5m patch is visible in one saved
inference image; this is not a measured FCRN effect. Clearance and duration
differences reversed direction between repeats, providing no reproducible
adverse effect. The recovered English report retains this limitation.

MonoNav completed all four flights. Astra inspected recorded trajectories and
images, identified that one montage preceded the noise window, requested later
frames, then moved only the eight-second sigma16 pulse from start5 to start18.
Both conditions completed. Each attack had22 recorded released active frames;
subsequent exact same-run joins verified10 observed consumed active updates in
each. These are sampled inputs, not all model inferences. No failure or
comparative advantage was found in these 12 flights.

## Defects found by actual execution

1. **Exposure audit clock mismatch.** Scene state was read before bridge HTTP
   state. Eleven inactive-delay frames appeared0.06s in the future, incorrectly
   rejecting PULP round1 exposure. The corrected audit uses the bridge's own
   simulation clock and explicitly counts unknown observations. All252/601
   released active-delay frames in the two PULP attacks have timestamp support.
   These are released-frame counts, not proof that every frame was consumed by
   the network. Exact worker/capture timestamp joins establish244/588 consumed
   active frames, with unmatched observations reported separately. Original
   agent inputs retain their earlier error flags; `exposure_reaudit_v2.json`
   contains the corrected analysis. All four PULP artifact audits pass.
2. **Codex report schema incompatibility.** `uniqueItems` is unsupported in the
   strict transport schema. Remove only this transport keyword, retain the
   original schema/audit hashes and local duplicate-reference validation, and
   save sanitized provider error details. A real synthetic schema probe passed.
   The recovered PULP report is in `pulp_dronet/posthoc_audit_v2/`; its conclusion
   is no demonstrated vulnerability, with limited coverage and mixed deltas.
3. **Unintended planning constraint.** V1 equated `repeat` with confirmation
   budget, rejecting Astra's valid intent to repeat for diagnosis. The repair
   changed its choice to exploration. Both original/repaired proposals remain
   recorded; do not describe the final choice as unconstrained original intent.
4. **Overlarge context.** Repeated full diagnostic objects dominated prompts.
   A pending context module retains exact actions/results/references while
   bounding diagnostic detail and permitting targeted telemetry requests.

5. **Patch exposure scoring gap.** Disabled noise and delay could make the
   aggregate sensor-exposure flag true in a patch-only run. That flag does not
   prove patch visibility. Per-active-factor verification must be connected
   before future scored campaigns; raw flight outcomes remain unchanged.

## Second development batch (separate from the first 12 protocol)

The second version connects explicit budget intent independent of experiment kind,
conditional outcome probability scoring, bounded context, targeted telemetry,
and sampled attack/obstacle/turn overlap diagnostics. New worker metadata binds
disturbance measurements to the current input update consumed by each model.
Planning attempts are immutable across interruptions. Canonical eligibility
requires both applied-factor evidence and observed consumed source inputs;
patch visibility remains a separate, unimplemented automatic verifier.
Historical unversioned resumes retain their original planning/scoring protocol.
New Random/Search campaigns receive the same canonical measurement checks;
their proposal distributions were subsequently expanded before a matched comparison.

`campaigns/codex_adaptive_v2_20261022/pulp_dronet/` completed six new clean/attack
pairs: **12/12 route completions**, with all twelve artifact audits and all six
factor-exposure checks passing. This is development evidence, not comparative
advantage. The target/controller stayed fixed.

Astra moved a finite noise pulse from12–36s to50–74s using measured turn timing,
raised noise24→32, added0.5s delay, then moved the combined pulse back to12–36s.
It used saved telemetry and, for the final decision, one actual inference image.
Rejected image requests are not counted as delivered evidence. It spent the last
pair repeating the combined pulse after observing reduced clearance.

Both repetitions completed, with attack-minus-clean minimum clearance changes
of−0.461m and−0.382m. This is a repeated proximity observation, not a demonstrated
completion failure, causal mechanism, or general robustness result. Timing and
individual-factor contributions remain unresolved. All six pairs used one
physical environment. The original report counted two because `6` and `6.0`
serialized differently; a derived correction preserves the original report.

The saved resource audit records13 CLI attempts including repairs,565,719
reported tokens and1,150.7s summed call latency. These are subscription usage
observations, not an API bill or end-to-end wall time. The next context projection
reduced a real four-round request177.6KB→131.3KB while retaining exact settings,
results, evidence references and every sampled encounter phase.

## Matched pilot prepared after development

The next PULP pilot reserves at most96 flights: two independent seeds × three
selectors × eight newly executed clean/attack pairs. The first six pairs measure
discovery/confirmation; the final two test a prospectively frozen conditional
diagnostic claim using the same text reporter for all selectors. Common initial
settings are rerun separately; no development outcomes or successful controls
are inherited. Failed clean controls remain ineligible for attack attribution,
while the attack still executes when infrastructure permits.

Random now samples the full feasible action domain. Search uses genuine elite
mutation/restarts, ranks observed failures first and uses measured progress and
clearance loss as a disclosed search proxy. Neither baseline is consumed by the
old shared diagnosis queue. Duplicate physical executions cannot earn repeat
credit. Phase-A efficiency excludes phase-B observations. Early stopping records
actual used and unused slots. Two seeds are a descriptive pilot, not proof of
general superiority. The launch protocol freezes code, weights, runtime identity,
priors, model/effort, metrics and execution order before outcomes.

The first pilot attempt stopped before its first takeoff: patch-disabled PULP
still computed unused patch vertices against a small obstacle surface. One startup
infrastructure result is retained, with zero performance flights. The narrow fix
skips nonexistent patch geometry while preserving explicit clean mounts and
enabled-patch checks;25 focused tests passed. A fresh r2 source snapshot checks
identical seeds, initial draws, order, priors, runtime identity and budgets.

The first r2 campaign, Search seed73001, completed16 flights:15 route completions
and one clean collision in the descriptive phase-B fallback. Phase A completed
all12 flights with zero confirmed failures;5/6 pairs had full factor exposure.
Overall6/8 pairs were eligible. Diagnosis is unavailable because phase A found
no eligible failure. All16 artifact checks and389 saved inference JPEG identities
passed. The English report required one repair after an unsupported candidate
claim was rejected:126,956 reported tokens and225.803s across two attempts.
Random seed73001 is next; these partial results establish no selector advantage.

A separate sensitivity audit checks the final diagnostic row against physical
contact time: the collector runs before collision detection and can sample just
after contact, although cleanup adds no rows. The observed clean collision does
not alter attack eligibility; its cited output was observed before contact.
Future versions should enforce this cutoff directly. Keep the current all-factor
primary endpoint frozen and flag any affected result before drawing conclusions.

Continue development and compare Random/Search/LLM with the **same effective action
space, evidence access and flight budget**. Historical Search mutation never ran
because shared follow-ups consumed its budget, and its random sampler was narrower
than the validator. The new matched protocol corrects both limitations.
Preserve unsuccessful development and comparison results. Defense implementation
and effect validation remain deferred until attack testing and agent benefits
have been evaluated.
