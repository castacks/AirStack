# EXP089: unchanged interval model on shared controller prefixes

Declared 2026-10-31 America/New_York, base351cb32bb, before restoring/scoring
EXP074/077 payloads in this implementation. Bounded integration diagnostic.

Use EXP073 early19,EXP074 middle99 andEXP077 day99 original decision boundaries
(217decisions/434camera slots). Stream their existing controller timelines; use
only supported complete exact-interval first writes already received. Apply
static nominal frame metadata through an explicit predictor-only contract. No
reference poses/losses enter prediction. Keep EXP088 stationary-body-twist model
and one-native-duration horizon unchanged. Baselines: existing fixed RGB-only,
thermal-only, raw availability advice and symmetric two-interval history with
fixed RGB preference. No new preference or parameters.

Freeze predictions before separate scoring. Use existing nominal-body recorded-
SLAM target only in scorer. Report full-population availability and pairwise
exact-matched translation/rotation loss; keep absent motions and cold-start
abstentions. Compare shared-prefix motion/model values with EXP088 matching rows.
Independent prefix/motion/policy and arithmetic audit plus mutation challenges.
No model fitting, training, inference, new scientific recording or reserved
capture. Offline serial replay does not establish live latency, physical frame,
acquisition-clock mapping or independent physical accuracy. Model outputs remain
unverified research projections; existing admission/default behavior unchanged.
No empirical accuracy acceptance threshold or generalization claim declared.

## Execution and outcomes

Completed2026-10-31 America/New_York.953members of restored EXP074/077 archives
match their SHA256 manifests; only eight top-level protocol/timeline/summary/
proof files were restored. EXP073's four existing files were retained. All three
timelines match their original saved independent PASS proofs and summaries.
These are three repeatedly inspected windows from two development recordings,
not three independent held-out recordings.

The preparation step binds every source interval to the original EXP084–086
report population, extracts only static nominal body transforms and manifest
rows into `predictor-contract.json`, and hashes all source files, proofs and
model/policy dependencies. Prediction streams each timeline in controller order.
It retains only supported exact-interval first writes with complete health
already observed at that boundary, records the later health/motion controller
receipt, and applies the unchanged EXP088 model. No worker timestamp is used as
a controller receipt. Native source values and projected values match all331
eligible EXP088 outputs; maximum native-matrix difference5.55e-16. This checks
integration parity, not physical calibration or independent reproduction of the
original neural runs.

All217decisions/434camera slots remain.331model projections are available;
103camera slots are unavailable at the corresponding prefix. Every selected
model projection has its required source received by its original controller
boundary. `pair_processing` stage alone is no longer the availability test.
Shutdown/late events cannot revise an emitted prediction. This is measured
logical prefix membership in a saved offline serial controller, not a real-time
performance or sensor-clock synchronization claim.

| Window | RGB-only available | Thermal-only available | Raw advice available | History available | Total decisions |
|---|---:|---:|---:|---:|---:|
| Early night |18|18|18|17|19|
| Middle night |1|98|98|97|99|
| Day |98|98|98|97|99|

Each baseline's own conditional losses and all six pairwise comparisons,
including exact matched pair IDs, are saved. The primary integration comparison
is unchanged raw advice versus unchanged history on identical available IDs:

| Window | Exact matched intervals | Raw translation proxy RMSE (m) | History translation proxy RMSE (m) | Raw rotation proxy RMSE (deg) | History rotation proxy RMSE (deg) |
|---|---:|---:|---:|---:|---:|
| Early night |17|0.010000|0.010000|0.06261|0.06261|
| Middle night |97|0.110529|0.043138|5.14257|0.70530|
| Day |97|0.031505|0.031505|0.12478|0.12478|

Middle-night history avoids the previously known isolated unsupported-history
RGB recovery. It equals thermal-only on its matched97intervals. Early/day it
equals fixed RGB on matched17/97intervals. It pays one additional cold-start
abstention in each window. Therefore this experiment does not establish a new
quality-selection benefit over the appropriate fixed expert; it validates the
existing fallback/history behavior with the fixed asynchronous model included.
Full-population availability is separate from conditional error, and no missing
interval is assigned zero loss. No policy/threshold/horizon or model was fitted.

## Validation

169CPU tests pass:17new prefix integration tests and152prior model/layer
regressions. Tests include no predictor file reads, delayed health, delayed
motion, future/shutdown exclusion, independent worker clocks, controller
receipt/deadline equality, static-frame rotation, unsupported/mismatched source
intervals, history cold start and rejected malformed source timelines.

The independent auditor imports no predictor, scheduler, history-policy or
production model code. It reconstructs each source prefix and both fixed rules,
uses independent quaternion conversion and scipy.linalg.logm/expm for331model
outputs, and audits all217choices and1896loss/summary values, including every
matched denominator and pair ID. Maximum model-matrix discrepancy4.41e-15.
Ten semantic mutations are rejected: receipt, matrix, policy, loss, availability,
matched population, missing group/policy/comparison and missing row. Reordering
predictions leaves results unchanged. The auditor's initial PASS is retained as
`validation-initial.json`; before final audit, group/policy/comparison schema
coverage and declaration-hash checks were strengthened. Predictions and scores
were unchanged by this audit refinement. EXP086's two documented gradient
exceptions remain in the frozen protocol without reinterpretation.

All prediction-side frame metadata are nominal and conditional. Result flags
retain `admission_ready=false`, unverified physical frame/acquisition clock and
unverified measured live latency. No model output is promoted into exact motion
admission/composition. Existing expert/arbiter production defaults remain intact.
No neural inference, checkpoint training, new scientific recording, reserved
capture, external message or physical/held-out accuracy claim occurred.

## Reproduction and decision

From MAC-VO-main in the recorded GCS CPU image with ore-ms at `/workspace`,
restore the existing source report/proof JSON and shared timeline JSONL files.
Copy the saved frozen declaration into a fresh output directory, then:

```bash
python3 -m Scripts.Setup.evaluate_prefix_interval_model prepare --root /workspace --output /workspace/results/prefix-model-pipeline/new-run
python3 -m Scripts.Setup.evaluate_prefix_interval_model predict --root /workspace --output /workspace/results/prefix-model-pipeline/new-run
python3 -m Scripts.Setup.evaluate_prefix_interval_model score --root /workspace --output /workspace/results/prefix-model-pipeline/new-run
python3 -m Scripts.Setup.validate_prefix_interval_model --root /workspace --output /workspace/results/prefix-model-pipeline/new-run
python3 -m pytest -o addopts= -q Scripts/UnitTest/test_prefix_interval_model.py Scripts/UnitTest/test_causal_interval_model.py Scripts/UnitTest/test_arbiter_motion_chain.py Scripts/UnitTest/test_arbiter_motion_output.py Scripts/UnitTest/test_pair_arbiter_events.py Scripts/UnitTest/test_shared_arbiter_replay.py Scripts/UnitTest/test_arbiter_history.py
```

This completes the causal-prefix integration check for the existing model and
policies. It supplies no evidence for another heuristic sweep on these windows.
A future learned quality gate still needs a frozen recording-level method/test
assignment and justified target. Physical rig/reference provenance, checkpoint
training membership and measured live timing remain open. New inference on the
same windows would not answer those source questions.

Recovery target: `ore_ms:ore-ms/backups/2026-10-31-exp-089`. Includes source,
source JSON/JSONL/proofs, predictor-only contract, frozen prediction/scoring/
audit artifacts, notebook JUnit/challenges and incremental Git history based on
`351cb32bb`. EXP088prediction/audit context is included; original checkpoints
and images are unnecessary for this CPU replay.
