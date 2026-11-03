# EXP111: fixed sensor-admission comparison preparation

2026-11-02 America/New_York. User approved direction recorded at `69f1d960e`.
**Preparation complete; empirical comparison not executed.**

[Working direction](../docs/research/sensor-admission-direction-2026-11-02.md):
sustained degradation/recovery and remaining geometric support, with original
frontends frozen. Current infrastructure selects independent expert motions;
this is not joint measurement-level fusion. No LSTM, agent or learned layer chosen.

[Executable protocol](../docs/research/sensor-admission-comparison-protocol-2026-11-02.md)
fixes one policy, engineering adequacy budgets, current-geometry/raw/history/fixed-
expert controls, label-free prediction, separate scoring and strict per-record
translation/rotation/coverage criteria. Missing geometry fails the input contract.

Added research-only modules for copied returned-pose Gaussian disparity information,
optional causal recorder/capture, nominal-frame/interval information transport,
source-prefix contract binding and predict/score CLI. Original recorder, native
optimizer, original runners and production defaults are unchanged. Information
excludes robust correction/damping and uncertain parent/map/extrinsics; it is not
calibrated pose covariance. The initial-linearization design was revised before
scientific capture to returned-pose geometry, binding it to the actual motion.

78 tests pass in the recovered immutable runtime: native graph/precision parity,
finite-difference Jacobians, rigid/time transport, singular directional geometry,
recorder observation order and legacy behavior, recovery/reconsideration, missing/
late/wrong-frame evidence, label rejection, matched scoring and exclusive CLI.
Earlier passing runs are retained as implementation advanced. No scientific
inference, fitting, reserve access or empirical policy score was produced.

Actual historical readiness: six timelines, 2,474 observed worker events, zero
geometry events. They cannot supply the new comparison by metadata relabelling.
Next necessary execution: freeze exact development capture inputs, run matched
flag-off/on preservation, then project fresh expert motions and score those same
new executions. Historical expert losses cannot label new captures. This requires
no new architecture selection or reserve assignment.

Results: `results/sensor-admission-preparation/run-001/`; recovery target:
`ore_ms:ore-ms/backups/2026-11-02-sensor-admission-preparation`.
