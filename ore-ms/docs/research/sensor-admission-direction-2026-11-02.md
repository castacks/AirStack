# Working direction: sustained sensor degradation and geometric contribution

2026-11-02 America/New_York. User approved recording this direction and proceeding.
Working hypothesis, not an adopted method or architecture:

> Tracking sustained degradation and recovery, while accounting for geometric
> information supplied by the remaining sensors, improves estimation over current
> covariance weighting alone.

RGB–thermal remains the first empirical case; eventual other-sensor generalization
requires an actual additional measurement type. MAC-VO/MACThermal are testbeds.
LSTM memory, a layer and an agent are potential implementations, not selected paths.

## Evidence and decisions preserved

- Native covariance already influences feature selection and optimization.
- Small RGB covariance-head adaptations did not transfer consistently; keep original
  checkpoints frozen as the reference rather than extend those pilots by default.
- EXP110 distinguishes thermal covariance persistence from its inconsistent future
  disagreement association. It does not establish forecasting impossibility.
- Persistent degradation can be useful without predicting exact long-term error.
- Existing history/shadow gains concentrate on one known RGB recovery event. They
  are controls, not sufficient evidence for general superiority or more capacity.
- Smooth covariance, native trace comparisons, feature count and image occupancy
  alone do not establish the remaining pose directions are adequately constrained.

## Order of work

1. Bind the actual baseline and intervention. Current shared-controller replay
   chooses between independently optimized VO outputs; it is not joint factor fusion.
2. Keep both original frontends running. Compare fixed native experts, original
   raw fallback, existing history, and one fixed degradation/recovery candidate.
   Initially change only the selected whole motion. Do not claim this tests joint
   factor weighting, compute scheduling or skipped-inference recovery.
3. Record causal geometric information from the native measurement model, rather
   than reconstruct future-refined states or rename point counts as observability.
4. Evaluate translation, rotation, coverage, recovery delay and discarded usable
   opportunities on complete declared populations. Keep proxy-reference limits.
5. Choose more complex memory only if it adds beyond current state/simple history;
   choose an agent only if sensing/compute actions demonstrate sequential value.
6. Validate the surviving mechanism externally, then with another sensor type.

## Immediate comparison and engineering prerequisite

The first executable comparison is explicitly a **whole-motion admission/selection
experiment**. Both experts retain native covariance weighting. The primary
candidate adds persistent recovery state and a geometric adequacy check; existing
raw/history and both fixed experts remain controls. This does not substitute for
a future measurement-level fusion experiment.

Historical causal health events have native covariance quantiles, support and image
coverage but no returned-pose information matrix. Final tensor maps are not assumed
to prove what geometric evidence was available at an earlier controller receipt.
Therefore new geometry observation must first preserve native execution. Add it
opt-in in research instrumentation; preserve default output and production behavior.
No fabricated zero matrices, reference-assisted information, or rescoring old
metadata as a completed geometry-aware experiment.

The accompanying fixed protocol and implementation specify exact information
semantics, state transitions, baseline controls and a fail-closed input contract.
Synthetic checks establish engineering behavior only. An actual capture freeze
must bind the recording/window, original config/checkpoint/runtime, interval and
nominal frame transforms before empirical execution. No reserve is assigned here.
