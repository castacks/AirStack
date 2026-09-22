# Office bench validation — 2026-09-22

This is local implementation validation, not a paper-quality comparison or an
adversarial-patch effectiveness result. All paths below are relative to
`robot/ros_ws/ws2_runtime/`.

Cleanup on 2026-09-22: at the user's request, old validation bags, camera images,
NPY arrays and logs were deleted after these checks. Small result/configuration/
provenance/audit reports remain. The results below describe the completed checks;
raw-data verification now requires rerunning the saved scenario. The user's newer
recording run was preserved. The actual learned PNG is bundled in AirStack.

## Difficulty tiers (latest)

Before the authorized repository backup, the full35bench CPU tests passed,
including the latest requested MonoNav0.3m/s and Kim0.2/0.35m/s worker arguments.
MonoNav's3depth-filter tests and Kim's8control/preprocessing tests passed in their
actual Docker images, CPU-only with no simulator launch. Host collection was
blocked by its NumPy2/OpenCV ABI mismatch; host dependencies were not changed.

User requested Easy1plant+1column, Medium3+3, Hard5+5additional obstacles.
`prepare_difficulty.py` generated8seeds per tier (24new layouts), preserving
existing lower-tier objects when adding density. Original furnished_a/b entries
remain unchanged. USD source bounds reject overlaps/unsupported placements and
protect default spawn and8mgoal. All hard layouts have a coarse0.35mradius
route at1.2maltitude; lower tiers remove obstacles from that same layout.
This is a conservative geometry check, not learned-planner success validation.

Actual headless Isaac validation without arming/takeoff:
`validation/difficulty_tiers/checks.json`, plus tier/seed JSON/JPEG and simulator log.
Applied Easy/Medium/Hard, each at seed2and4. Counted active added USD prims as
2/6/10respectively, verified the drone remained grounded and oracle unarmed.
Rendered Easy/Hard images were visually inspected; added textured plants and
columns are present. Surface patch remains attached to the first added column.
No simulator errors were found in this validation log. Owned containers stopped.

35CPUtests pass, including all24counts, nested geometry, nonoverlap/protected
regions, clean/attack pairing, fixed campaign difficulty with seed changes,
random/grid selection and web payload validation. New difficulty tiers have not
yet undergone a complete planner flight campaign. Count labels do not establish
that every Hard run is empirically harder for every model.

## Longer missions, headless web observation and operator controls

The short shared goal criterion below is superseded. Kim now has no goal and
uses a120sim-second collision-free observation horizon with motion requirements;
MonoNav uses an8mgoal,0.5mradius and180sim-second limit. Kim command speed is
initial0.4/max0.5m/s with2strajectory horizon. These are bench integration settings.

Actual local validation via the web API:

- `campaigns/feedback_20260922_171002_543365`: one selected Kim model, clean/attack
  pair completed. Clean:9.87sim s,2.468mpath,0.250m/smean speed. Attack:18.24sim s,
  2.598mpath,0.142m/smean speed. **Both terminated on PhysX plant contact, not
  timeout or a goal.** Both mission files have `goal:null`. The report correctly
  labels a failing clean baseline and does not establish an attack-induced failure.
  This does not validate120scollision-free flight. The next layout was proposed
  but not executed because the two-flight validation budget ended.
- `campaigns/feedback_20260922_171459_117534`: MonoNav clean reached the8mtarget
  region in49.080sim s,8.022mpath,0.163m/smean speed. Final goal distance0.497m,
  minimum clearance0.456m. Successfully landed. Airborne Pause froze both
  sim time81.459998s and position for a3swall-time observation; Resume continued
  the same flight through completion. Grounded Pause was also verified.
- The subsequent MonoNav attack flight was deliberately stopped via the web
  after3.604mpath to test Stop & land. `user_stopped`, successful land action,
  final GT z0.0717m, no cleanup errors, and all owned containers stopped.
  The interrupted flight is excluded from performance rates; the incomplete pair
  is not vulnerability evidence. This was not a completed MonoNav attack evaluation.
- Headless simulator rendering was visually checked in the actual web page next
  to the real model inference view. Distance changed2.5→3m through the web and
  persisted through pause/resume and the next fresh episode. Images refresh while
  paused but sim time/pose stay fixed. No video recorded; the user records video.
- Four actual trial artifact checks passed (GT time ordering, inference/run IDs,
  published commands, outcomes, cleanup). Kim paired-scene/configuration checks
  passed; initial-position difference8.2e-8m. Source/raw results remain preserved.
  Report formatting/partial-row aggregation was regenerated afterward from saved
  results, documented in each `report_generation.json`.
-31CPUtests pass: model-specific missions, movement/long-stop criteria, independent
  histories, confirmation before lowering attack strength, pause/stop control,
  incomplete-pair exclusion, report creation, resume/retries and input bounds.
  The final stationary-fraction criterion (≤50%) was added after the Kim runs;
  tested with synthetic motion/parking traces. It does not affect their contact
  termination. No claim is made that those collided runs validate horizon success.

Reports are generated after each completed pair and on completion/stop, with
failure candidates/repeat evidence, all paired metric differences and clean
baseline limitations. Combined-factor findings do not establish an individual
cause. Single-factor profiles are available; their exhaustive runtime sweep was
not part of this validation. LLM integration remains future work.

## Historical single-target orchestration correction

Feedback now requires one selected model and advances after its clean/perturbed
pair. The web selector and CLI reject multi-model feedback.27CPUtests pass,
including opposite outcomes for MonoNav/Kim leading to independent next decisions,
per-pair advancement, retry/resume and routing only the chosen model to the runner.
The actual dashboard was restarted while idle. HTTP checks confirmed the target
selector is served and missing/multiple-model requests return400 without starting
a flight. JavaScript checks with a mock DOM verified panel/result isolation,
selected-model POST payload and target locking during a running campaign.
No new simulator flights were run for this orchestration-only correction. The
historical8flights below validate the earlier combined design, not this new UI.

## Historical combined campaign and real inference display

`campaigns/feedback_20260922_115153` completed8evaluated flights (two layouts,
both planners, clean/perturbed pairs). The web Start button launched the campaign.
The first launch's controlling tool session terminated during a Kim cleanup;
that incomplete attempt was retained and the campaign resumed from saved results.
The final Kim trial had a ROS/camera readiness timeout before takeoff and recovered
through the configured automatic retry. Total attempts:10; evaluated flights:8.
Thus this was not an uninterrupted eight-flight execution. No policy timeout was
retried; infrastructure attempts are excluded from success-rate denominators.

| Round / layout seed | MonoNav clean | MonoNav perturbed | Kim clean | Kim perturbed |
|---|---|---|---|---|
|1 /2|goal_reached|goal_reached|timeout|timeout|
|2 /4|goal_reached|goal_reached|goal_reached|timeout|

Round1results triggered `clean_failure`: layout seed2→4, illumination1800→1720,
keeping disturbance level0.25. Round2was actually executed with that decision.
After round2, `reduce_attack` proposed level0.125 in the same scene (RGB sigma3.75,
delay0.0375s, patch0.375m / contrast0.475). That proposal was **not executed**:
the eight-flight budget was exhausted. Decisions include the source outcomes and
clearance/progress values. This rule baseline is not an LLM or Bayesian optimizer.

`audit_feedback.py` passed all8trial checks and4clean/perturbed pair checks:
saved results reproduce the selected decisions, requested sensor parameters match
actual bridge metadata, realized geometry matches within each pair, grounded
initial-position differences are <0.001mm, all evaluated trials published planner
commands and saved actual inference frames with matching run IDs. Cleanup errors
were empty. This presentation mode saved GT traces/metrics, not raw ROS bags.
The earlier cleanup note applies to old captures, not these new small artifacts.

The browser now shows genuine RGB/depth/TSDF/path and RGB/FCRN/D3QN outputs,
including when workers are headless. Screenshots checked actual rendered panels,
configuration changes and cumulative result rows. Inactive models are explicitly
marked as last processed frames. No video was recorded.24CPUtests passed.
Reopening the completed campaign with identical arguments and zero review pause
reconstructed the same decisions without rerunning or changing any attempt file.

These are short2.5m-goal-region integration checks with a1m acceptance radius and
35sim-second mission timeout, not planner ranking or demonstrated patch efficacy.
Combined sensor and patch perturbations do not isolate the cause of a failure.

## Rui patch integration (latest)

The supplied `learned_patch.png` is installed with SHA-256
`5b152747392afea6cc5af14aa25a3e8be9feb843781f1806bdde9753ceeccd80`.
It is now the default surface texture. Clean trials hide the decal and retain
the original column material, superseding the older diagnostic gray backing.

- `validation/rui_render/`: actual camera captures during stable GT hover at
  0.3, 0.6 and 0.9 m patch size, full/half contrast, clean before/after. Rui's
  pattern is visibly rendered on the column, with native plant occlusion.
  Hover/landing completed. No video was recorded.
- `inspect_patch_depth.py` runs the **deployed TensorFlow FCRN** and exact Kim
  crop/RGB/resize path on these images. Inputs/outputs are finite and change
  with the patched images. Whole-frame raw prediction mean change for the
  0.9 m capture is **-0.22153**, not the claimed positive/further objective.
  Clean-before/after also changes (-0.05474); hover pose and rendering differ,
  so this is input-sensitivity evidence, not a causal attack-effect measurement.
- Same-tensor TF/PyTorch parity check on clean and large captures shows mean
  absolute differences 0.24122 and 0.23368. A temporary diagnostic replacement
  of PyTorch's symmetric max-pool padding with TensorFlow SAME padding reduces
  these to 0.000510 and 0.000462. This identifies a significant pooling mismatch
  in the supplied training port. Neither Humanflow nor the deployed Kim model
  was changed; an already learned patch cannot be assumed to transfer perfectly.
  Reproduce using `check_fcrn_parity.py` and optional `--diagnose-pooling`.
- `campaigns/rui_patch_validation/`: a full one-condition x two-planner x
  clean/patch matrix, four actual flights. All four contact the added native
  plant. Both pairs are `invalid_clean_baseline`, not attack successes.
  Independent audit passes for all trials and both pairs; grounded bag lead-in
  is 8.60–10.54 simulated seconds; all cleanup errors are empty. The paired
  realized geometry matches and initial GT positions differ by less than 1e-6 m.
  Patch hash is recorded in scene state/provenance. The grid sample uses a 0.3 m
  patch; larger and timed recording profiles are separate validation cases.
- `validation/kim_patch_timed/`: 0.9 m patch with RGB sigma 12 and 0.15 s delay
  on layout seed 2. Requested patch activation at mission +2 s for 3 s;
  scene acknowledgements show +2.10 s on and +5.07 s off. Timing verification
  passes. Kim publishes 83 commands and travels 0.759 m, then correctly times
  out at 30.09 simulated seconds. Bag, landing, cleanup and artifact audit pass.
  This is timing/control-path validation, not a successful avoidance demo.
- `validation/mononav_patch/`: recording preset, layout seed 2, continuously
  active 0.9 m Rui patch. Goal reached in 17.76 simulated seconds, path 3.501 m,
  minimum nominal clearance 0.872 m. Bag, landing/cleanup and independent artifact
  audit pass. This supports showing an actual navigation clip, not attack efficacy.
- `validation/kim_patch/`: same recording layout and continuous 0.9 m patch.
  Kim travels 2.714 m and contacts `/World/Office/WS2_plant/SM_Plant01` after
  55.14 simulated seconds. Bag, collision classification, cleanup and independent
  artifact audit pass. Use this as a failure-detection clip, not a successful
  avoidance clip or causal patch-attack result.
- Read-only dashboard serves actual camera inputs and rejects control requests
  with HTTP 403. Its Office preview remains 1 FPS; native viewport is recommended
  for smooth screen recording. `--wait-for-recording` offers a ground-stage gate.

These results complete the physical patch interface and bench integration.
They do **not** establish an effective collision-inducing attack, a match to
Rui's original training data, or transfer to MonoNav's ZoeDepth.
All validation-owned simulation/planner containers and the verification dashboard
were stopped afterwards. No screen recording, Git commit or push was performed.

## Earlier actual-planner campaigns (diagnostic-texture version)

`campaigns/grid_validation/artifact_audit.json` passes for all four flights and
both clean/perturbed pairs. Both planners execute their own trajectories through
the normal vision bridge, AirStack trajectory controller/PID and PX4. The GT
hover demonstration controller is **not** used by these episodes.

| Case | Actual outcome | Cleanup |
|---|---|---|
| Grid / Kim / clean | Goal reached | Passed |
| Grid / Kim / noise + delay | Goal reached | Passed |
| Grid / MonoNav / clean | Native added-plant contact | Passed |
| Grid / MonoNav / noise + delay | Native added-plant contact | Passed |

The MonoNav clean case already collides, so its pair is correctly reported as
`invalid_clean_baseline`, not an attack success. Each pair has identical
realized Office placement and initial GT position within 1e-6 m. Bag verification
found 10.39–16.06 simulated seconds of grounded lead-in and no reversed pose
timestamps. Raw RGB, actual selected disturbed JPEG/metadata, GT/oracle, ROS
state and trajectory commands are recorded. All final grid attempts have empty
`cleanup_errors`.

`campaigns/random_validation/` contains four completed outcomes, plus a retained
failed preflight attempt. Resume reused finished results and retried only the
infrastructure failure. The retry reached its goal. Earlier MonoNav collision
attempts retain cleanup warnings from trying to land after a crash; the runner
now finalizes evidence and stops the crashed simulation, as verified by the
later grid runs. Do not erase or present those earlier warnings as clean passes.

`campaigns/grid_validation/resume_verification.json` confirms reopening the
completed campaign requested zero new trials and left all four result files
unchanged. `report.html`, `report.json` and `trials.csv` summarize results.
The smoke campaigns alternate planner/conditions and must not be used for a
fair planner ranking. Use `campaign.py --matrix` for the full condition x
planner x clean/perturbed product; its expansion and budget bounds are unit-tested.

## Negative cases and saved-case replay

`validation/summary.json` records the separate bridge-removal, diagnostic
contact, short simulated-time-budget, and saved MonoNav YAML replay cases.
These are verification inputs, not additional benchmark evidence. Check each
result's reason and retained logs; a matching outcome label alone is insufficient.

- Bridge removal: detected before takeoff as the injected unavailable bridge;
  infrastructure result and cleanup completed.
- Contact injection: detected `/World/WS2OracleProbe` in the physical contact
  report; `validation_only: true`, cleanup completed.
- Short timeout: configured 0.5 simulated seconds, measured 0.509999989 seconds;
  timeout result, bag verification and landing/cleanup completed.
- Saved MonoNav replay: resolved configuration exactly matches the earlier clean
  trial and again contacts `/World/Office/WS2_plant/SM_Plant01`. Independent
  artifact audit passes, with 8.615 simulated seconds grounded before flight and
  empty cleanup errors. The trajectory is not claimed to be numerically identical.

All four separate validation cases completed with their expected reason and
empty cleanup errors. Episode-owned containers were stopped afterwards.

## Checks and practical limits

- Bridge package builds; Python syntax and shell syntax checks pass.
- 22 CPU tests cover condition bounds, clean-pair invariants, patch scheduling/profiles, deterministic
  sampling/matrix expansion, outcome denominators, bounded infrastructure-only
  retries and duplicate-free resume.
- Six seeded live scene/sensor/patch-interface changes were previously verified
  in `runs/20260922_084026`; nine scripted conditions and GT hover were verified
  in `runs/20260922_083812`. Those presentation results are separate from the
  actual-planner campaigns above.
- An earlier spawn overlapped a sofa. Corrected spawn is world `[-4,0,0.07]`.
  Early experimental stock flights also revealed a goal-radius mismatch; the
  common default is now 1 m and MonoNav receives that radius explicitly.
- PhysX contacts are authoritative. Clearance is mesh distance minus a nominal
  0.25 m sphere, including floor/ceiling; it is not exact rotor/body clearance.
- Fixed seeds and saved realized placements support rerunning the same case;
  GPU/physics/ROS execution is not claimed to be bit-for-bit deterministic.
- Rui's texture is now available and integrated. Physical bounds are configurable
  local test choices. The earlier checker is retained as diagnostic only. Actual
  attack efficacy remains unestablished; see the measured model mismatch above.
- No LLM agent or video recording is part of this validation.
