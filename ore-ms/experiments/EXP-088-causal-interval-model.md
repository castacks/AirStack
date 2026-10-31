# EXP088: frozen constant-twist interval-model diagnostic

Declared 2026-10-31 America/New_York, base cef2a104b, before reading/scoring the
restored EXP084–086 report payloads in this implementation.

One model: constant body twist, D_target=Exp(Log(D_native)*duration_ratio), full
SE3 coupling. Native input is the original first pre-refinement motion. Maximum
extrapolation outside native endpoints is one native duration; no horizon search.
No reference poses or label-assisted bridges enter prediction. This is a
model-only projection, not an admitted or deployable output. Frames/clocks remain
unverified and the separate saved solver reports do not establish a shared
controller decision deadline.

Population: all434 original-arm camera slots from EXP084early19,EXP085middle99,
EXP086day99 paired intervals. Missing/unsupported sources and terminal writes
stay explicit; only available pair_processing first writes are eligible. No
selector, parameter tuning, neural inference, training, new scientific recording
or reserved capture access. Existing expert/arbiter defaults unchanged.

Produce predictions before scoring. Primary descriptive diagnostic: model versus
historic reference-assisted projection of the same source delta, translation
metres and separate rotation degrees. Also report model, unchanged-native and
historic-bridge proxy losses on identical eligible populations per window/camera.
The bridge is an offline comparison, not causal input or independent truth.
Independent matrix-log/exp recomputation and label-mutation guard required.
No empirical success threshold or deployment claim is declared.

## Execution and result

Completed 2026-10-31 America/New_York. All1635 members of the existing EXP084–086
recovery archives match their recorded SHA256 values. Only12 report/protocol/
proof JSON files were restored; no images, checkpoints or inference outputs were
regenerated. Historical evaluation/protocol proof hash chains and the434-row
original-arm population pass. Prediction sources, declaration and predictions
were frozen before a separate scoring invocation. Removing and corrupting label
fields produces identical predictions.

331model projections are eligible: early18RGB/18thermal, middle1RGB/98thermal,
day98RGB/98thermal.97unsupported/missing RGB source rows and6terminal-stage
writes stay explicitly unavailable. No rows are removed for model error, and
no horizon rejection occurs. Terminal-stage exclusion changes these populations
from older all-write summaries; compare only the matched populations below.

| Window / camera | Eligible / slots | Model vs historical bridge translation RMSE (mm) | Rotation disagreement RMSE (deg) | Model proxy translation RMSE (m) | Unchanged-native proxy RMSE (m) | Historical bridge proxy RMSE (m) |
|---|---:|---:|---:|---:|---:|---:|
| Early RGB |18/19|~0|~0|0.009891|0.009891|0.009891|
| Early thermal |18/19|0.403|0.02272|0.009427|0.009427|0.009248|
| Middle RGB |1/99|~0|~0|1.003081|1.003081|1.003081|
| Middle thermal |98/99|4.227|0.05944|0.042956|0.042954|0.042620|
| Day RGB |98/99|~0|~0|0.031459|0.031459|0.031459|
| Day thermal |98/99|19.650|0.18901|0.079808|0.090324|0.077786|

RGB is the target interval, so its duration ratio is1 and this is an identity
control. Thermal day duration scaling reduces the unchanged-native proxy RMSE
by11.64%, while remaining2.60% above the reference-assisted bridge comparison.
It is approximately unchanged from native at night. Rotation outcomes remain
separate: thermal model proxy rotation RMSE is0.02741/0.70193/0.60026degrees
for early/middle/day, versus bridge0.03231/0.67862/0.61578degrees. Neither the
bridge nor recorded-SLAM proxy is independent physical truth. Projection
agreement and proxy loss are distinct quantities; reference-assisted bridge
agreement is not a physical error estimate or a deployable quality signal.

The model assumes stationary body twist across source and requested intervals,
using the principal rotation logarithm without angular-winding recovery. Full
SE(3) translation/rotation coupling is retained through a left-Jacobian solve,
then exponential scaling. The implementation uses SciPy's documented
[rotation-vector API](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_rotvec.html);
[the Sophus SE3 implementation](https://github.com/strasdat/Sophus/blob/main/sophus/se3.hpp)
is a primary implementation reference for coupled Lie-group exp/log. No source
code is copied. Runtime is NumPy1.26.4/SciPy1.11.4 in the existing GCS image.

152CPU tests pass:33model geometry/assumption tests plus119existing layer
regressions. Analytic tests include pure translation, rotating lever arm,
small/near-pi rotations, independent4x4exponentials and an accelerating-motion
counterexample where a perfect native estimate still projects incorrectly.
The independent auditor uses scipy.linalg.logm/expm rather than the production
Jacobian implementation for all331 projections: maximum element difference
4.34e-15. All434row/status checks and2888scalar checks pass. Seven semantic
mutations are rejected; row reordering preserves the result. Existing EXP086's
two documented near-cancellation gradient exceptions remain in protocol.json;
this report does not erase or reclassify them.

## Decision and limits

Keep this as an opt-in, model-only research baseline. This experiment quantifies
a timing approximation; it is not another gate sweep, a new learned selector or
a deployment/accuracy acceptance test. No empirical acceptance threshold was
predeclared, so no post-hoc pass/fail accuracy claim is assigned. A constant-twist
projection can change a meaningful amount under day offsets and must be counted
in downstream comparisons rather than treated as neutral synchronization.

Prediction takes a completed original motion and timestamps only. Because these
are separate saved solver reports, we cannot establish a shared controller
cutoff or live input availability from `pair_processing` alone. Outputs retain
`admission_ready=false` and unverified physical-frame/acquisition-clock/controller-
deadline flags. They are not supplied to the exact admission/composition path,
which still requires its own verified inputs. Physical rig/GNSS provenance and
checkpoint training membership remain unresolved. No model training, new neural
inference, reserved capture, production default or external message occurred.

## Reproduction and recovery

From MAC-VO-main in the recorded GCS CPU image, with ore-ms mounted at
`/workspace`, restore the four JSON files per EXP084–086 run and copy the frozen
`declaration.json` into a fresh output directory. Then run:

```bash
python3 -m Scripts.Setup.evaluate_interval_model predict --root /workspace --output /workspace/results/causal-interval-model/new-run
python3 -m Scripts.Setup.evaluate_interval_model score --root /workspace --output /workspace/results/causal-interval-model/new-run
python3 -m Scripts.Setup.validate_interval_model --root /workspace --run /workspace/results/causal-interval-model/new-run
python3 -m pytest -o addopts= -q Scripts/UnitTest/test_causal_interval_model.py Scripts/UnitTest/test_arbiter_motion_chain.py Scripts/UnitTest/test_arbiter_motion_output.py Scripts/UnitTest/test_pair_arbiter_events.py Scripts/UnitTest/test_shared_arbiter_replay.py Scripts/UnitTest/test_arbiter_history.py
```

Final evidence: `results/causal-interval-model/run-001/`. Fresh outputs are required;
commands refuse overwrites. Backup: `ore_ms:ore-ms/backups/2026-10-31-exp-088`,
including source/tests, frozen declaration/predictions/scores/audits, source
report/proof JSON, notebook results and incremental Git history based on
`cef2a104b`. Original checkpoints/images are unnecessary for this CPU diagnostic.
